use crate::{
    queue::{self, Kind, Namespace, PublisherMode, Queue},
    Error,
};
use std::{collections::BTreeSet, marker::PhantomData, rc::Rc, time::Duration};

fn milliseconds(timeout: Duration) -> Result<i32, Error> {
    i32::try_from(timeout.as_millis()).map_err(|_| Error::TimeoutRange)
}

pub struct Publisher {
    queue: Queue,
    thread: PhantomData<Rc<()>>,
}

impl Publisher {
    /// Temporary UI feeds yield ownership to a later daemon publisher. They must
    /// use `send_if_current` and stop on false rather than reclaim the endpoint.
    pub fn transient_for_runtime(endpoint: &str, capacity: usize) -> Result<Self, Error> {
        Self::open(
            endpoint,
            capacity,
            Namespace::Runtime,
            PublisherMode::Transient,
        )
    }

    pub fn for_runtime(endpoint: &str, capacity: usize) -> Result<Self, Error> {
        Self::open(
            endpoint,
            capacity,
            Namespace::Runtime,
            PublisherMode::Exclusive,
        )
    }

    pub fn new(endpoint: &str) -> Result<Self, Error> {
        Self::with_capacity(endpoint, 1024 * 1024)
    }

    pub fn with_capacity(endpoint: &str, capacity: usize) -> Result<Self, Error> {
        Self::open(
            endpoint,
            capacity,
            Namespace::Isolated,
            PublisherMode::Exclusive,
        )
    }

    fn open(
        endpoint: &str,
        capacity: usize,
        namespace: Namespace,
        mode: PublisherMode,
    ) -> Result<Self, Error> {
        Ok(Self {
            queue: Queue::open(endpoint, Kind::Publisher(mode), capacity, namespace)?,
            thread: PhantomData,
        })
    }

    pub fn send(&mut self, bytes: &[u8]) -> Result<(), Error> {
        self.queue.send(bytes)
    }

    pub fn send_if_current(&mut self, bytes: &[u8]) -> Result<bool, Error> {
        match self.queue.send(bytes) {
            Ok(()) => Ok(true),
            Err(Error::PublisherReplaced) => Ok(false),
            Err(error) => Err(error),
        }
    }

    pub fn readers_caught_up(&mut self) -> bool {
        self.queue.readers_caught_up()
    }
}

pub struct Subscriber {
    queue: Queue,
    thread: PhantomData<Rc<()>>,
}

pub struct Subscription<'a> {
    pub endpoint: &'a str,
    pub capacity: usize,
    pub polled: bool,
}

#[derive(Debug)]
pub struct QueuedMessage {
    pub index: usize,
    pub bytes: Vec<u8>,
}

struct Endpoint {
    name: String,
    capacity: usize,
    polled: bool,
}

pub struct MultiSubscriber {
    endpoints: Vec<Endpoint>,
    queues: Vec<Option<Queue>>,
    lazy: bool,
    thread: PhantomData<Rc<()>>,
}

impl MultiSubscriber {
    /// Open each non-conflated queue only while its bridge has connected peers.
    pub fn lazy_for_runtime(specifications: &[Subscription<'_>]) -> Result<Self, Error> {
        Self::open(specifications, Namespace::Runtime, false, true)
    }

    pub fn set_active(&mut self, index: usize, active: bool) -> Result<(), Error> {
        if !self.lazy {
            return Err(Error::Invalid(
                "subscription activation requires a lazy batch",
            ));
        }
        let queue = self
            .queues
            .get_mut(index)
            .ok_or(Error::Invalid("subscription index"))?;
        if active == queue.is_some() {
            return Ok(());
        }
        let endpoint = &self.endpoints[index];
        *queue = if active {
            Some(Queue::open(
                &endpoint.name,
                Kind::Subscriber { conflate: false },
                endpoint.capacity,
                Namespace::Runtime,
            )?)
        } else {
            None
        };
        Ok(())
    }

    pub fn new(specifications: &[Subscription<'_>]) -> Result<Self, Error> {
        Self::open(specifications, Namespace::Isolated, true, false)
    }

    pub fn for_runtime(specifications: &[Subscription<'_>]) -> Result<Self, Error> {
        Self::open(specifications, Namespace::Runtime, true, false)
    }

    /// Retain every queued packet for consumers such as route logging.
    pub fn queued_for_runtime(specifications: &[Subscription<'_>]) -> Result<Self, Error> {
        Self::open(specifications, Namespace::Runtime, false, false)
    }

    /// Poll without consuming packets; callers control drain limits and stop checks.
    pub fn poll_ready(&mut self, timeout: Duration) -> Result<Vec<usize>, Error> {
        self.poll(milliseconds(timeout)?)
    }

    /// `poll_ready`, returning no earlier than `not_before`: messages that
    /// arrive meanwhile are collected in one scan (see `queue::poll_not_before`).
    pub fn poll_ready_not_before(
        &mut self,
        timeout: Duration,
        not_before: std::time::Instant,
    ) -> Result<Vec<usize>, Error> {
        self.poll_with(milliseconds(timeout)?, Some(not_before))
    }

    pub fn receive_one(&mut self, index: usize) -> Result<Option<Vec<u8>>, Error> {
        self.queues
            .get_mut(index)
            .ok_or(Error::Invalid("subscription index"))?
            .as_mut()
            .ok_or(Error::Invalid("inactive subscription"))?
            .receive(0)
    }

    /// Copy one packet into caller-owned storage, reusing its allocation.
    /// Returns false and clears the buffer when the queue is empty. Buffer
    /// contents must not be consumed on error. Queue ownership is unchanged.
    pub fn receive_one_into(&mut self, index: usize, bytes: &mut Vec<u8>) -> Result<bool, Error> {
        self.queues
            .get_mut(index)
            .ok_or(Error::Invalid("subscription index"))?
            .as_mut()
            .ok_or(Error::Invalid("inactive subscription"))?
            .receive_now_into(bytes)
    }

    fn open(
        specifications: &[Subscription<'_>],
        namespace: Namespace,
        conflate: bool,
        lazy: bool,
    ) -> Result<Self, Error> {
        if specifications.is_empty() || specifications.len() > 256 {
            return Err(Error::Invalid("invalid subscription count"));
        }
        let mut names = BTreeSet::new();
        let mut endpoints = Vec::with_capacity(specifications.len());
        let mut queues = Vec::with_capacity(specifications.len());
        for specification in specifications {
            if !names.insert(specification.endpoint) {
                return Err(Error::Invalid("duplicate subscription"));
            }
            endpoints.push(Endpoint {
                name: specification.endpoint.to_owned(),
                capacity: specification.capacity,
                polled: specification.polled,
            });
            queues.push(if lazy {
                None
            } else {
                Some(Queue::open(
                    specification.endpoint,
                    Kind::Subscriber { conflate },
                    specification.capacity,
                    namespace,
                )?)
            });
        }
        if !lazy && !endpoints.iter().any(|endpoint| endpoint.polled) {
            return Err(Error::Invalid(
                "at least one polled subscription is required",
            ));
        }
        Ok(Self {
            endpoints,
            queues,
            lazy,
            thread: PhantomData,
        })
    }

    fn poll(&mut self, timeout_ms: i32) -> Result<Vec<usize>, Error> {
        self.poll_with(timeout_ms, None)
    }

    fn poll_with(
        &mut self,
        timeout_ms: i32,
        not_before: Option<std::time::Instant>,
    ) -> Result<Vec<usize>, Error> {
        let mut ready = Vec::with_capacity(self.queues.len());
        let endpoints = &self.endpoints;
        let queues = &mut self.queues;
        let scan = || {
            ready.clear();
            for (index, queue) in queues.iter_mut().enumerate() {
                if endpoints[index].polled {
                    if let Some(queue) = queue {
                        if queue.ready()? {
                            ready.push(index);
                        }
                    }
                }
            }
            Ok(!ready.is_empty())
        };
        match not_before {
            Some(not_before) => queue::poll_not_before(timeout_ms, not_before, scan)?,
            None => queue::poll(timeout_ms, scan)?,
        }
        for (index, queue) in self.queues.iter().enumerate() {
            if queue.is_some() && !self.endpoints[index].polled {
                ready.push(index);
            }
        }
        Ok(ready)
    }

    pub fn receive(&mut self, timeout: Duration) -> Result<Vec<QueuedMessage>, Error> {
        let ready = self.poll(milliseconds(timeout)?)?;
        let mut messages = Vec::with_capacity(ready.len());
        for index in ready {
            if let Some(bytes) = self.receive_one(index)? {
                messages.push(QueuedMessage { index, bytes });
            }
        }
        Ok(messages)
    }
}

impl Subscriber {
    pub fn for_runtime(endpoint: &str, conflate: bool, capacity: usize) -> Result<Self, Error> {
        Self::open(endpoint, conflate, capacity, Namespace::Runtime)
    }

    pub fn new(endpoint: &str, conflate: bool) -> Result<Self, Error> {
        Self::with_capacity(endpoint, conflate, 1024 * 1024)
    }

    pub fn with_capacity(endpoint: &str, conflate: bool, capacity: usize) -> Result<Self, Error> {
        Self::open(endpoint, conflate, capacity, Namespace::Isolated)
    }

    fn open(
        endpoint: &str,
        conflate: bool,
        capacity: usize,
        namespace: Namespace,
    ) -> Result<Self, Error> {
        Ok(Self {
            queue: Queue::open(endpoint, Kind::Subscriber { conflate }, capacity, namespace)?,
            thread: PhantomData,
        })
    }

    pub fn receive(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, Error> {
        self.queue.receive(milliseconds(timeout)?)
    }
}
