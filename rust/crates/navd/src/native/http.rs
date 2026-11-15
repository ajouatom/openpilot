use super::http_body::{body_json, decompress};
use crate::route::RequestError;
use openpilot_http_transport::socket_timeout_agent;
use std::{io::Read, time::Duration};

pub struct Response {
    pub status: u16,
    pub text: String,
    pub json: Result<serde_json::Value, RequestError>,
}

pub fn get(url: &str) -> Result<Response, RequestError> {
    get_inner(url).map_err(|error| RequestError::Transport(error.to_string()))
}

#[derive(Debug, thiserror::Error)]
enum Error {
    #[error(transparent)]
    Http(#[from] ureq::Error),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Url(#[from] url::ParseError),
    #[error(transparent)]
    Text(#[from] openpilot_registration::Error),
    #[error("route request exceeded 30 redirects")]
    RedirectLimit,
    #[error("route response exceeds 8 MB")]
    TooLarge,
}

/// navd.py `requests.get(..., timeout=(3, 8))` and its 8 MB body limit.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(3);
const READ_TIMEOUT: Duration = Duration::from_secs(8);
const MAX_BODY: u64 = 8 * 1024 * 1024;

fn get_inner(url: &str) -> Result<Response, Error> {
    let mut url = url::Url::parse(url)?;
    let config = ureq::Agent::config_builder()
        .http_status_as_error(false)
        .max_redirects(0)
        .max_redirects_will_error(false)
        .timeout_connect(Some(CONNECT_TIMEOUT))
        .timeout_global(None)
        .max_idle_connections(0)
        .user_agent("python-requests/2.34.2")
        .build();
    let agent = socket_timeout_agent(config, READ_TIMEOUT);
    let mut cookies = cookie_store::CookieStore::default();
    for redirects in 0..=30 {
        let mut request = agent
            .get(url.as_str())
            .header("accept", "*/*")
            .header("accept-encoding", "gzip, deflate, br")
            .header("connection", "keep-alive");
        let mut matched = cookies.matches(&url);
        matched.retain(|cookie| cookie.secure() != Some(true) || url.scheme() == "https");
        matched.sort_by_key(|cookie| std::cmp::Reverse(cookie.path.as_ref().len()));
        if !matched.is_empty() {
            let header = matched
                .into_iter()
                .map(|cookie| format!("{}={}", cookie.name(), cookie.value()))
                .collect::<Vec<_>>()
                .join("; ");
            request = request.header("cookie", &header);
        }
        agent.cookie_jar_lock().clear();
        let mut response = request.call()?;
        for value in response.headers().get_all("set-cookie") {
            if let Ok(value) = value.to_str() {
                if let Err(error) = cookies.parse(value, &url) {
                    eprintln!("navd: ignored invalid response cookie: {error}");
                }
            }
        }
        let status = response.status().as_u16();
        let location = response
            .headers()
            .get("location")
            .and_then(|value| value.to_str().ok())
            .map(str::to_owned);
        let content_type = response
            .headers()
            .get("content-type")
            .and_then(|value| value.to_str().ok())
            .unwrap_or("")
            .to_owned();
        let encoding = response
            .headers()
            .get("content-encoding")
            .and_then(|value| value.to_str().ok())
            .unwrap_or("")
            .to_owned();
        let mut bytes = Vec::new();
        response
            .body_mut()
            .as_reader()
            .take(MAX_BODY + 1)
            .read_to_end(&mut bytes)?;
        if bytes.len() as u64 > MAX_BODY {
            return Err(Error::TooLarge);
        }
        bytes = decompress(bytes, &encoding)?;
        if let Some(location) = location.filter(|_| matches!(status, 301 | 302 | 303 | 307 | 308)) {
            if redirects == 30 {
                return Err(Error::RedirectLimit);
            }
            url = url.join(&location)?;
        } else {
            let text = openpilot_registration::response_text(&bytes, &content_type)?;
            let json = body_json(&bytes, &content_type, &text);
            return Ok(Response { status, text, json });
        }
    }
    Err(Error::RedirectLimit)
}
