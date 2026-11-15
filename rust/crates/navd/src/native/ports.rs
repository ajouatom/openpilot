use super::{diagnostics, http, publisher::Publisher, stop::Stop, timestamp};
use crate::{
    geometry::Coordinate,
    route::{Diagnostic, Instruction, Ports, RequestError},
    wire, Error,
};
use openpilot_logging::producer::Logger;
use openpilot_params::Params;
use std::{sync::mpsc, thread};

pub struct NativePorts<'a> {
    pub params: Params,
    /// `/dev/shm/params`.
    pub memory: Params,
    pub logger: Logger,
    pub publisher: Publisher,
    pub stop: &'a Stop,
    /// The route request running on its own thread, and the token it carries
    /// (masked out of any error text: navd never logs it).
    pub pending: Option<(mpsc::Receiver<Result<http::Response, RequestError>>, String)>,
}

fn token_of(url: &str) -> String {
    url::Url::parse(url)
        .ok()
        .and_then(|url| {
            url.query_pairs()
                .find(|(key, _)| key == "access_token")
                .map(|(_, value)| value.into_owned())
        })
        .unwrap_or_default()
}

fn masked(error: RequestError, token: &str) -> RequestError {
    match error {
        RequestError::Transport(text) if !token.is_empty() => {
            RequestError::Transport(text.replace(token, "***"))
        }
        RequestError::Status { status, body } if !token.is_empty() => RequestError::Status {
            status,
            body: body.replace(token, "***"),
        },
        error => error,
    }
}

impl Ports for NativePorts<'_> {
    fn parameter(&mut self, name: &str) -> Result<Option<String>, Error> {
        self.stop.check()?;
        Ok(openpilot_params_typed::get_string(
            &self.params,
            name,
            &mut self.logger,
        )?)
    }

    fn remove_parameter(&mut self, name: &str) -> Result<(), Error> {
        self.stop.check()?;
        super::parameters::write_status(self.params.remove(name))
    }

    fn memory_parameter(&mut self, name: &str) -> Result<Option<String>, Error> {
        self.stop.check()?;
        Ok(match self.memory.get(name) {
            Ok(Some(bytes)) => String::from_utf8(bytes).ok(),
            Ok(None) | Err(openpilot_params::Error::Io(_)) => None,
            Err(error) => return Err(error.into()),
        })
    }

    fn remove_memory_parameter(&mut self, name: &str) -> Result<(), Error> {
        self.stop.check()?;
        super::parameters::write_status(self.memory.remove(name))
    }

    fn submit(&mut self, url: &str) -> Result<(), Error> {
        self.stop.check()?;
        let (sender, receiver) = mpsc::sync_channel(1);
        let owned = url.to_owned();
        thread::Builder::new()
            .name("mapbox-route".into())
            .spawn(move || {
                // A closed receiver means navd stopped or dropped the request.
                let _ = sender.send(http::get(&owned));
            })?;
        self.pending = Some((receiver, token_of(url)));
        Ok(())
    }

    fn poll(&mut self) -> Option<Result<serde_json::Value, RequestError>> {
        let (receiver, token) = self.pending.as_ref()?;
        let result = match receiver.try_recv() {
            Ok(result) => result,
            Err(mpsc::TryRecvError::Empty) => return None,
            Err(mpsc::TryRecvError::Disconnected) => {
                Err(RequestError::Transport("route request worker ended".into()))
            }
        };
        let token = token.clone();
        self.pending = None;
        Some(match result {
            Ok(response) => {
                if response.status != 200 {
                    diagnostics::api_failure(
                        &mut self.logger,
                        response.status,
                        &response.text.replace(&token, "***"),
                    );
                }
                if response.status >= 400 {
                    Err(masked(
                        RequestError::Status {
                            status: response.status,
                            body: response.text,
                        },
                        &token,
                    ))
                } else {
                    response.json.map_err(|error| masked(error, &token))
                }
            }
            Err(error) => Err(masked(error, &token)),
        })
    }

    fn instruction(&mut self, message: &Instruction) -> Result<(), Error> {
        self.stop.check()?;
        self.publisher
            .send("navInstruction", wire::instruction(message, timestamp()?)?)
    }

    fn route(&mut self, coordinates: &[Coordinate]) -> Result<(), Error> {
        self.stop.check()?;
        self.publisher
            .send("navRouteNavd", wire::route(coordinates, timestamp()?)?)
    }

    fn diagnostic(&mut self, event: Diagnostic) {
        diagnostics::event(&mut self.logger, event);
    }
}
