use crate::Error;
use openpilot_modeld::camera::{CameraSource, Captured, FrameMeta};
use openpilot_msgq::{VisionClient, VisionMetadata};
use std::{
    cell::{OnceCell, RefCell},
    rc::Rc,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};

/// A received camera frame, read in place from its VisionIPC mapping as
/// carrot's modeld does (`VisionBuf.data`); a CPU copy is made only for a
/// consumer that needs bytes.
pub struct Image {
    pub layout: VisionMetadata,
    address: usize,
    bytes: OnceCell<Vec<u8>>,
    spare: Rc<RefCell<Spare>>,
}

impl Image {
    /// Pixels in place; mapped until the camera generation ends, which ends
    /// this process.
    pub fn address(&self) -> usize {
        self.address
    }

    pub fn len(&self) -> usize {
        self.layout.len
    }

    pub fn is_empty(&self) -> bool {
        self.layout.len == 0
    }

    /// A CPU copy, made on first use.
    pub fn bytes(&self) -> &[u8] {
        self.bytes.get_or_init(|| {
            let mut bytes = {
                let mut spare = self.spare.borrow_mut();
                if spare.len != self.layout.len {
                    spare.len = self.layout.len;
                    spare.bytes = None;
                }
                spare.bytes.take().unwrap_or_else(|| vec![0; self.layout.len])
            };
            // SAFETY: the VisionIPC mapping holds `layout.len` pixel bytes at
            // `address` for the client's lifetime, which outlives this frame.
            // Like the source API, the producer may recycle the buffer.
            unsafe {
                std::ptr::copy_nonoverlapping(
                    self.address as *const u8,
                    bytes.as_mut_ptr(),
                    self.layout.len,
                );
            }
            bytes
        })
    }
}

#[derive(Default)]
struct Spare {
    len: usize,
    bytes: Option<Vec<u8>>,
}

impl Drop for Image {
    fn drop(&mut self) {
        // A source may already have gone away or changed layouts while this snapshot was retained.
        // Keep at most one spare and never panic while releasing an image.
        if let Ok(mut spare) = self.spare.try_borrow_mut() {
            if let Some(bytes) = self.bytes.take() {
                if bytes.len() == spare.len && spare.bytes.is_none() {
                    spare.bytes = Some(bytes);
                }
            }
        }
    }
}

pub struct Source {
    pub client: VisionClient,
    pub stop: Arc<AtomicBool>,
    spare: Rc<RefCell<Spare>>,
}

impl Source {
    pub fn new(client: VisionClient, stop: Arc<AtomicBool>) -> Self {
        Self {
            client,
            stop,
            spare: Rc::default(),
        }
    }
}

impl CameraSource for Source {
    type Buffer = Image;
    type Error = Error;

    fn receive(&mut self) -> Result<Option<Captured<Image>>, Error> {
        if self.stop.load(Ordering::Relaxed) {
            return Ok(None);
        }
        if !self.client.is_connected() {
            return Err(Error::Contract("camera generation lost; restart modeld"));
        }
        let Some(frame) = self.client.receive(Duration::from_millis(100))? else {
            if !self.stop.load(Ordering::Relaxed) && !self.client.is_connected() {
                return Err(Error::Contract("camera generation lost; restart modeld"));
            }
            return Ok(None);
        };
        let layout = *frame.metadata();
        let address = frame.data_address();
        Ok(Some(Captured {
            metadata: FrameMeta {
                frame_id: layout.frame_id,
                timestamp_sof: layout.timestamp_sof,
                timestamp_eof: layout.timestamp_eof,
            },
            buffer: Image {
                layout,
                address,
                bytes: OnceCell::new(),
                spare: Rc::clone(&self.spare),
            },
        }))
    }
}
