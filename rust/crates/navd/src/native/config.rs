use super::{diagnostics, Options};
use crate::{route::Config, Error};
use openpilot_logging::{producer::Logger, record::Level};
use openpilot_params::Params;

/// carrot's navd.py talks to Mapbox only, with `MAPBOX_TOKEN` or the
/// `MapboxPublicKey` the user enters; the token is read every update
/// (`refresh_token`), so only the host is fixed here. comma's own map proxy
/// (Prime JWT) is no longer used.
pub fn load(_params: &Params, logger: &mut Logger, options: &Options) -> Result<Config, Error> {
    if options.mapbox_host.is_some() {
        diagnostics::text(
            logger,
            Level::Info,
            "navd HTTP fixture endpoint selected".into(),
        );
    }
    Ok(Config {
        host: options
            .mapbox_host
            .clone()
            .unwrap_or_else(|| "https://api.mapbox.com".to_owned()),
    })
}
