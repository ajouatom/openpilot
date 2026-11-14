use crate::Error;
use openpilot_params::Params;

pub trait Parameters {
    fn integer(&mut self, key: &'static str) -> Result<i32, Error>;
    fn float(&mut self, key: &'static str) -> Result<f64, Error>;
    fn boolean(&mut self, key: &'static str) -> Result<bool, Error>;
    fn string(&mut self, key: &'static str) -> Result<Option<String>, Error>;
    fn put_integer(&mut self, key: &'static str, value: i32) -> Result<(), Error>;
    fn put_boolean(&mut self, key: &'static str, value: bool) -> Result<(), Error>;
}
pub fn raw(params: &Params, key: &str) -> Result<Vec<u8>, Error> {
    match params.get(key) {
        Ok(value) => Ok(value.unwrap_or_default()),
        Err(openpilot_params::Error::Io(_)) => Ok(Vec::new()),
        Err(error) => Err(error.into()),
    }
}
#[cfg(feature = "native")]
impl Parameters for Params {
    // A stored value that is not a number reads as 0, like a missing one:
    // a single corrupt setting must not fail every control frame.
    fn integer(&mut self, key: &'static str) -> Result<i32, Error> {
        Ok(openpilot_beepd::integer(&raw(self, key)?).unwrap_or(0))
    }
    fn float(&mut self, key: &'static str) -> Result<f64, Error> {
        Ok(openpilot_calibrationd::parameters::parse_float(&raw(self, key)?).unwrap_or(0.))
    }
    fn boolean(&mut self, key: &'static str) -> Result<bool, Error> {
        Ok(raw(self, key)? == b"1")
    }
    fn string(&mut self, key: &'static str) -> Result<Option<String>, Error> {
        let bytes = raw(self, key)?;
        if bytes.is_empty() {
            return Ok(None);
        }
        Ok(Some(
            String::from_utf8(bytes).map_err(|_| Error::Contract("non-UTF8 Params string"))?,
        ))
    }
    fn put_integer(&mut self, key: &'static str, value: i32) -> Result<(), Error> {
        match self.put(key, value.to_string().as_bytes()) {
            Ok(()) | Err(openpilot_params::Error::Io(_)) => Ok(()),
            Err(error) => Err(error.into()),
        }
    }
    fn put_boolean(&mut self, key: &'static str, value: bool) -> Result<(), Error> {
        match self.put_bool(key, value) {
            Ok(()) | Err(openpilot_params::Error::Io(_)) => Ok(()),
            Err(error) => Err(error.into()),
        }
    }
}
