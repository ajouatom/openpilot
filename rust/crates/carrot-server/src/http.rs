pub use crate::http_request::read_json;
pub use crate::http_server::serve;
pub use crate::request_body::{decode_request, DecodedBody as RequestBody};
use crate::{
    config::Config,
    param_changes::{History, Paths},
    params::Backend,
    settings::SettingsCache,
    static_web::StaticWeb,
    Error, Value,
};
use bytes::Bytes;
use http_body_util::Full;
use std::sync::{Arc, Mutex};

pub type Body = Full<Bytes>;

pub struct Application {
    pub config: Config,
    pub params: Mutex<Backend>,
    pub history: History,
    live_snapshot: Mutex<Option<Value>>,
    settings: Mutex<SettingsCache>,
    pub static_web: Arc<StaticWeb>,
    pub intro: Arc<crate::intro::Intro>,
    pub cars: Arc<crate::cars::Cars>,
    pub mapbox_online: Arc<crate::mapbox_tokens::Online>,
    pub ssh_online: Option<Arc<crate::ssh_keys::Online>>,
    pub ssh_timestamp: Option<i64>,
    pub egpu_model: Arc<crate::egpu_model::ModelFiles>,
    pub popular_values: Arc<crate::popular_values::Service>,
    pub xiaoge_online: Arc<crate::xiaoge::Online>,
    pub bluetooth_http: Arc<crate::bluetooth_http::Service>,
    pub screenrecord: Arc<crate::screenrecord::Screenrecord>,
    pub dashcam: Arc<crate::dashcam::Service>,
    pub dashcam_metadata: Arc<crate::dashcam::MetadataFiles>,
    pub dashcam_media: Arc<crate::dashcam::Media>,
    pub dashcam_uploads: Arc<crate::dashcam::Uploads>,
    pub dashcam_upload_health: Arc<crate::dashcam::UploadHealth>,
    pub dashcam_sync_uploads: Arc<crate::dashcam::SyncUploads>,
    pub git_status: Option<Arc<crate::git_status::Service>>,
    pub git_state: crate::git_state::Store,
    pub heartbeat: Arc<crate::heartbeat::Service>,
    pub heartbeat_params: Option<openpilot_params::Params>,
    pub auto_update: Option<Arc<crate::auto_update_runtime::Runtime>>,
    pub live: Option<Arc<crate::live::Service>>,
    pub live_error: Option<String>,
    pub web_navi: Option<Arc<crate::web_navi::Service>>,
    pub system: Arc<crate::system::Service>,
    pub network_refresh: bool,
    pub qr_dependency: crate::qr_dependency::Provider,
    pub tools: Option<Arc<crate::tools::service::Service>>,
    pub terminal: Option<Arc<crate::terminal::Service>>,
    pub support_terminal: Option<Arc<crate::support_terminal::Service>>,
    pub stream_proxy: Option<Arc<crate::stream_proxy::Service>>,
    pub vision_test: Option<crate::vision_test::Config>,
    pub vision_diag: Option<crate::vision_diag::Config>,
}

impl Application {
    pub fn new(config: Config, params: Backend) -> Arc<Self> {
        Arc::new(Self::initialize(config, params, None))
    }

    pub fn with_git_status(
        config: Config,
        params: Backend,
        service: Arc<crate::git_status::Service>,
    ) -> Arc<Self> {
        Arc::new(Self::initialize(config, params, Some(service)))
    }

    pub fn for_runtime(
        config: Config,
        params: Backend,
        git_status: Arc<crate::git_status::Service>,
    ) -> Arc<Self> {
        let heartbeat_params = params.native_params().cloned();
        let auto_update = crate::auto_update_runtime::Runtime::new(
            &config,
            heartbeat_params.clone(),
            Arc::clone(&git_status),
        );
        let mut application = Self::initialize(config, params, Some(git_status));
        application.heartbeat_params = heartbeat_params;
        application.auto_update = Some(auto_update);
        application.network_refresh = true;
        application.tools = Some(crate::tools::service::Service::new(
            crate::tools::config::Config::original(&application),
        ));
        Arc::new(application)
    }

    fn initialize(
        config: Config,
        params: Backend,
        git_status: Option<Arc<crate::git_status::Service>>,
    ) -> Self {
        config.migrate_legacy_state();
        let dashcam = crate::dashcam::Service::original(&config);
        let dashcam_metadata = crate::dashcam::MetadataFiles::original(Arc::clone(&dashcam));
        let dashcam_media = crate::dashcam::Media::original(&dashcam, &config);
        let dashcam_uploads = crate::dashcam::Uploads::original(&dashcam);
        let dashcam_sync_uploads = crate::dashcam::SyncUploads::original(&dashcam);
        let dashcam_upload_health =
            crate::dashcam::UploadHealth::original(&config, params.native_params().cloned());
        let (live, live_error) = match crate::live::Service::start(params.native_params().cloned())
        {
            Ok(service) => (Some(service), None),
            Err(error) => (None, Some(error.to_string())),
        };
        let web_navi = crate::web_navi::Service::start(params.native_params().cloned()).ok();
        crate::egpu_library::start_maintenance(&config.repository, params.native_params().cloned());
        crate::web_settings::forget_stored_upload_default(&config.state.join("web_settings.json"));
        let mut application = Self {
            terminal: crate::terminal::Config::original()
                .and_then(crate::terminal::Service::start)
                .ok(),
            vision_test: None,
            vision_diag: None,
            support_terminal: None,
            stream_proxy: crate::stream_proxy::Service::original(params.native_params().cloned())
                .map(Arc::new)
                .map_err(|error| eprintln!("Stream proxy startup: {error}"))
                .ok(),
            tools: None,
            qr_dependency: crate::qr_dependency::Provider::original(&config),
            system: crate::system::Service::original(&config),
            network_refresh: false,
            static_web: StaticWeb::new(config.clone()),
            intro: crate::intro::Intro::new(config.clone()),
            cars: crate::cars::Cars::original(),
            mapbox_online: Arc::new(crate::mapbox_tokens::Online::default()),
            ssh_online: Some(Arc::new(crate::ssh_keys::Online::default())),
            ssh_timestamp: None,
            egpu_model: Arc::new(crate::egpu_model::ModelFiles::original(&config.repository)),
            popular_values: crate::popular_values::Service::new(true),
            xiaoge_online: Arc::new(crate::xiaoge::Online::default()),
            bluetooth_http: crate::bluetooth_http::Service::original(),
            screenrecord: crate::screenrecord::Screenrecord::original(&config),
            dashcam,
            dashcam_metadata,
            dashcam_media,
            dashcam_uploads,
            dashcam_upload_health,
            dashcam_sync_uploads,
            git_status,
            git_state: crate::git_state::Store::new(config.state.clone()),
            heartbeat: crate::heartbeat::Service::new(),
            heartbeat_params: None,
            auto_update: None,
            live,
            live_error,
            web_navi,
            history: History::new(Paths {
                log: config.state.join("param_changes.jsonl"),
                baseline: config.state.join("fingerprint_baseline.json"),
            }),
            live_snapshot: Mutex::new(None),
            settings: Mutex::new(SettingsCache::new(config.settings.clone())),
            config,
            params: Mutex::new(params),
        };
        application.tools = Some(crate::tools::service::Service::new(
            crate::tools::config::Config::original(&application),
        ));
        application.support_terminal = application.terminal.as_ref().and_then(|terminal| {
            crate::support_terminal::Config::original(&application, Arc::clone(terminal))
                .and_then(crate::support_terminal::Service::start)
                .map_err(|error| eprintln!("Support terminal startup: {error}"))
                .ok()
        });
        application
    }

    pub(crate) fn settings_payload(&self) -> Result<Value, Error> {
        let params = self
            .params
            .lock()
            .map_err(|_| Error::Source("Params lock poisoned".into()))?;
        let catalog = self
            .catalog(&params)?
            .for_brand(&params.vehicle_brand(), params.ray_pedal_control())?;
        let payload = catalog.payload(&self.config.settings, params.has_params());
        let mut response = Value::object([("ok", Value::Bool(true))]);
        if let Value::Object(items) = payload {
            if let Value::Object(fields) = &mut response {
                fields.extend(items);
            }
        }
        Ok(response)
    }

    pub(crate) fn catalog(&self, params: &Backend) -> Result<crate::settings::Catalog, Error> {
        let mut cache = self
            .settings
            .lock()
            .map_err(|_| Error::Source("settings lock poisoned".into()))?;
        cache.load(params.maximum_gap_levels())
    }

    pub fn update_live_snapshot(&self, snapshot: Value) -> Result<(), Error> {
        *self
            .live_snapshot
            .lock()
            .map_err(|_| Error::Source("live snapshot lock poisoned".into()))? = Some(snapshot);
        Ok(())
    }

    pub fn drive_engaged(&self) -> bool {
        self.live.as_ref().is_some_and(|service| service.engaged())
            || self
                .live_snapshot
                .lock()
                .ok()
                .and_then(|snapshot| {
                    snapshot.as_ref().map(|snapshot| {
                        let services = snapshot.get("services");
                        ["selfdriveState", "controlsState"]
                            .into_iter()
                            .any(|name| services.get(name).get("enabled").truth())
                    })
                })
                .unwrap_or(false)
    }
}
