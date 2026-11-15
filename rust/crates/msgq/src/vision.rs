use crate::{
    queue::{Kind, Namespace, Queue},
    vision_memory::Buffer,
    vision_socket, vision_wire, Error, VisionLayout, VisionMetadata, VisionStream,
};
use std::{
    marker::PhantomData,
    os::fd::{AsFd, AsRawFd},
    path::PathBuf,
    rc::Rc,
    time::Duration,
};
mod retained;
pub use retained::RetainedBuffer;

/// Client for the original trusted local VisionIPC server protocol.
pub struct VisionClient {
    socket_path: PathBuf,
    stream: VisionStream,
    queue: Queue,
    buffers: Vec<Buffer>,
    connected: bool,
    thread: PhantomData<Rc<()>>,
}

impl VisionClient {
    pub fn new(name: &str, stream: VisionStream, conflate: bool) -> Result<Self, Error> {
        let socket_path = vision_socket::path(name)?;
        let queue = Queue::open(
            &format!("visionipc_{name}_{}", stream.native()),
            Kind::Subscriber { conflate },
            1024 * 1024,
            Namespace::Runtime,
        )?;
        Ok(Self {
            socket_path,
            stream,
            queue,
            buffers: Vec::new(),
            connected: false,
            thread: PhantomData,
        })
    }

    /// Attempts discovery once; an absent server returns an empty list.
    pub fn available_streams(name: &str) -> Result<Vec<VisionStream>, Error> {
        let Some(socket) = vision_socket::connect(&vision_socket::path(name)?)? else {
            return Ok(Vec::new());
        };
        vision_socket::send(socket.as_fd(), &4_i32.to_le_bytes(), &[])?;
        let (payload, descriptors) = match vision_socket::receive(socket.as_fd(), 16) {
            Err(Error::Io(_, error)) if error.kind() == std::io::ErrorKind::ConnectionReset => {
                return Ok(Vec::new())
            }
            value => value?,
        };
        if !descriptors.is_empty() {
            return Err(Error::Corrupt(
                "VisionIPC discovery transferred unexpected descriptors",
            ));
        }
        vision_wire::decode_streams(&payload)
    }

    /// Attempts connection once, without the native infinite retry loop.
    pub fn connect(&mut self) -> Result<bool, Error> {
        self.connected = false;
        self.buffers.clear();
        let Some(socket) = vision_socket::connect(&self.socket_path)? else {
            return Ok(false);
        };
        vision_socket::send(socket.as_fd(), &self.stream.native().to_le_bytes(), &[])?;
        let (payload, descriptors) = match vision_socket::receive(
            socket.as_fd(),
            vision_wire::BUFFER_BYTES * vision_wire::MAX_FDS,
        ) {
            Err(Error::Io(_, error)) if error.kind() == std::io::ErrorKind::ConnectionReset => {
                return Ok(false)
            }
            value => value?,
        };
        if payload.is_empty() && descriptors.is_empty() {
            return Ok(false);
        }
        let records = vision_wire::decode_buffers(&payload, descriptors.len(), self.stream)?;
        let buffers: Result<Vec<_>, _> = records
            .into_iter()
            .zip(descriptors)
            .map(|(wire, fd)| Buffer::import(wire, fd))
            .collect();
        self.buffers = buffers?;
        self.connected = true;
        Ok(true)
    }

    pub fn is_connected(&self) -> bool {
        self.connected
    }

    /// Matches the original layout properties without consuming a frame.
    pub fn layout(&self) -> Option<VisionLayout> {
        self.buffers.first().map(|buffer| buffer.wire.layout)
    }

    /// A timeout or a changed server returns no frame; check `is_connected`
    /// to distinguish a reconnect requirement from an ordinary timeout.
    pub fn receive(&mut self, timeout: Duration) -> Result<Option<VisionFrame<'_>>, Error> {
        if !self.connected {
            return Err(Error::Invalid("VisionIPC client is not connected"));
        }
        self.receive_retained(timeout)
    }

    /// Encoder opt-in matching the original receive-only loop after disconnection.
    /// Validated imported buffers must exist. This method never reconnects automatically.
    pub fn receive_retained(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<VisionFrame<'_>>, Error> {
        let timeout = i32::try_from(timeout.as_millis()).map_err(|_| Error::TimeoutRange)?;
        if self.buffers.is_empty() {
            return Err(Error::Invalid(
                "VisionIPC has no validated imported buffers",
            ));
        }
        let Some(payload) = self.queue.receive(timeout)? else {
            return Ok(None);
        };
        let packet = vision_wire::decode_packet(&payload)?;
        let Some(buffer) = self.buffers.get(packet.index) else {
            self.connected = false;
            return Ok(None);
        };
        if buffer.wire.server_id != packet.server_id {
            self.connected = false;
            return Ok(None);
        }
        // The original client cleans (writes back) the CPU cache over the
        // whole buffer on every receive. A subscriber never writes a camera
        // buffer, so there is nothing to write back, yet the kernel walks all
        // 4.8 MB: 33 % of driving-modeld's time and most of encoderd's kernel
        // time on a comma 3X (simpleperf, __clean_dcache_area_poc). Readers
        // see current pixels through the server's invalidate at publish
        // (vision_server; the data cache is physically tagged).
        let layout = buffer.wire.layout;
        let metadata = VisionMetadata {
            width: layout.width,
            height: layout.height,
            stride: layout.stride,
            uv_offset: layout.uv_offset,
            len: layout.len,
            frame_id: packet.frame_id,
            timestamp_sof: packet.timestamp_sof,
            timestamp_eof: packet.timestamp_eof,
            valid: packet.valid,
            received: true,
            index: packet.index,
            fd: buffer.fd.as_raw_fd(),
        };
        Ok(Some(VisionFrame {
            buffer,
            metadata,
            client: PhantomData,
        }))
    }
}

/// Keeps the imported mapping alive and excludes another receive/reconnect.
/// The producer may still recycle the buffer: this is not a producer lease.
pub struct VisionFrame<'a> {
    buffer: &'a Buffer,
    metadata: VisionMetadata,
    client: PhantomData<&'a mut VisionClient>,
}

impl VisionFrame<'_> {
    pub fn metadata(&self) -> &VisionMetadata {
        &self.metadata
    }

    /// Borrowed FD/scalars, without a camera-memory slice or producer lease.
    /// Duplicate the FD before retaining it beyond this frame's lifetime.
    pub fn descriptor(&self) -> Result<crate::VisionBufferDescriptor<'_>, Error> {
        Ok(crate::VisionBufferDescriptor {
            fd: self.buffer.fd.as_fd(),
            mmap_len: self.buffer.mapping.mapped_length(),
            data_len: self.buffer.wire.layout.len,
            index: self.buffer.wire.index,
            server_id: self.buffer.wire.server_id,
            buffer_frame_id: self.buffer.mapping.frame_id()?,
        })
    }

    /// Address of the mapped pixels (`metadata().len` bytes), for a consumer
    /// such as the GPU that reads the buffer in place like the source API's
    /// `VisionBuf.data`. Stays mapped until this client's generation ends.
    pub fn data_address(&self) -> usize {
        self.buffer.mapping.address()
    }

    /// Copies the original mapped bytes into exactly `metadata().len` bytes.
    /// Like the source API, copying does not make producer updates atomic.
    pub fn copy_into(&self, destination: &mut [u8]) -> Result<(), Error> {
        self.buffer.mapping.copy_into(destination)
    }
}

impl AsFd for VisionFrame<'_> {
    fn as_fd(&self) -> std::os::fd::BorrowedFd<'_> {
        self.buffer.fd.as_fd()
    }
}
