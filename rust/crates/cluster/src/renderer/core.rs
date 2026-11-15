use super::{display, Graphics, Options, RenderedFrame, Result, SystemStats, TextureView};
use crate::config::{self, ClusterTheme};
use chrono::Timelike;
use openpilot_startup_ui::renderer::{DynamicTexture, Renderer};
use std::{
    collections::{BTreeMap, BTreeSet, VecDeque},
    path::{Path, PathBuf},
    time::Instant,
};

pub trait StatsProvider {
    fn sample(&mut self, now: f64) -> SystemStats;
    fn close(&mut self);
}

pub struct ClusterUiRenderer {
    pub width: f64,
    pub height: f64,
    pub language: String,
    pub is_metric: bool,
    pub screen_mode: i64,
    pub panel_layout: i64,
    pub gfx: Graphics,
    pub profile_enabled: bool,
    pub(super) profile_samples: Vec<(String, f64)>,
    pub(super) hud: super::hud::HudState,
    pub(super) world: super::world::WorldResources,
    pub(super) camera_overlay: super::camera_overlay::OverlayState,
    pub(super) camera_resources: super::background::CameraResources,
    pub(super) scene_cache: super::scene_cache::SceneCache,
    pub(super) output_resources: super::output::OutputResources,
    pub(super) static_images: BTreeMap<super::assets::StaticImage, DynamicTexture>,
    pub(super) world_text: BTreeMap<super::text::WorldKey, super::text::CachedText>,
    pub(super) world_text_order: VecDeque<super::text::WorldKey>,
    pub(super) text_policy: super::text::TextCachePolicy,
    pub(super) trip_cache: super::trip_cache::TripCache,
    pub(super) route_video: super::route_video::RouteVideoCache,
    pub navi_guidance_size: Option<(i32, i32)>,
    pub(super) root: PathBuf,
    pub(super) options: Options,
    pub(super) font: u32,
    pub(super) korean_font: Option<u32>,
    pub(super) theme_mode: String,
    pub(super) started: Instant,
    pub(super) stats: Box<dyn StatsProvider>,
    pub(super) measure_cache: BTreeMap<super::text::MeasureKey, (f64, f64)>,
    pub(super) strokes: BTreeMap<super::text::StrokeKey, super::text::CachedText>,
    pub(super) stroke_order: VecDeque<super::text::StrokeKey>,
    pub(super) pending_strokes: BTreeSet<super::text::StrokeKey>,
    pub(super) pending_order: VecDeque<super::text::StrokeKey>,
    pub(super) guidance: Option<DynamicTexture>,
    pub(super) guidance_hash: String,
    pub(super) media: BTreeMap<String, super::media::CachedMedia>,
    pub(super) media_support: super::media_support::MediaSupport,
    pub(super) nav_model: super::nav_model::NavModelMap,
}

impl ClusterUiRenderer {
    #[must_use]
    pub fn new(root: &Path, options: Options) -> Self {
        let theme_mode = options.theme_mode.clone();
        let language = options.language.clone();
        let screen_mode = options.screen_mode;
        let panel_layout = options.panel_layout;
        let is_metric = options.is_metric;
        let mut value = Self {
            width: f64::from(options.width),
            height: f64::from(options.height),
            language: String::new(),
            is_metric: options.is_metric,
            screen_mode: 0,
            panel_layout: 0,
            gfx: Graphics::default(),
            navi_guidance_size: None,
            profile_enabled: std::env::var("CLUSTER_PROFILE_RENDER").as_deref() == Ok("1"),
            profile_samples: Vec::new(),
            hud: super::hud::HudState::default(),
            world: super::world::WorldResources::default(),
            camera_overlay: super::camera_overlay::OverlayState::default(),
            camera_resources: super::background::CameraResources::default(),
            scene_cache: super::scene_cache::SceneCache::default(),
            output_resources: super::output::OutputResources::default(),
            static_images: BTreeMap::new(),
            world_text: BTreeMap::new(),
            world_text_order: VecDeque::new(),
            text_policy: super::text::TextCachePolicy::from_environment(),
            trip_cache: super::trip_cache::TripCache::default(),
            route_video: super::route_video::RouteVideoCache::default(),
            root: root.into(),
            theme_mode: String::new(),
            font: 0,
            korean_font: None,
            started: Instant::now(),
            stats: Box::new(super::stats::Sampler::default()),
            measure_cache: BTreeMap::new(),
            strokes: BTreeMap::new(),
            stroke_order: VecDeque::new(),
            pending_strokes: BTreeSet::new(),
            pending_order: VecDeque::new(),
            guidance: None,
            guidance_hash: String::new(),
            media: BTreeMap::new(),
            media_support: super::media_support::MediaSupport::default(),
            nav_model: super::nav_model::NavModelMap::default(),
            options,
        };
        value.set_theme_mode(&theme_mode);
        value.set_screen_mode(screen_mode);
        value.set_panel_layout(panel_layout);
        value.set_display_preferences(&language, is_metric);
        value
    }
    /// # Errors
    /// Returns the native window or font initialization error.
    pub fn open(&mut self, hidden: bool) -> Result<()> {
        if self.gfx.native.is_some() {
            return Ok(());
        }
        let total = self.profile_start();
        self.output_resources.hidden = hidden;
        let start = self.profile_start();
        self.gfx.native = Some(if hidden {
            Renderer::raw_window(
                (self.options.width, self.options.height),
                &self.options.title,
                128,
                0,
            )?
        } else {
            Renderer::raw_window_inheriting_flags(
                (self.options.width, self.options.height),
                &self.options.title,
                0,
            )?
        });
        self.profile_add("renderer.open.init_window", start);
        if self.options.target_fps > 0 {
            let start = self.profile_start();
            self.gfx.backend()?.set_target_fps(self.options.target_fps);
            self.profile_add("renderer.open.set_target_fps", start);
        }
        let start = self.profile_start();
        self.load_fonts()?;
        self.profile_add("renderer.open.load_font", start);
        let start = self.profile_start();
        let timings = self.world.load_model(&mut self.gfx, &self.root)?;
        for timing in timings {
            self.profile_add_elapsed(timing.name, timing.elapsed_ms);
        }
        self.profile_add("renderer.open.load_vehicle_model", start);
        let start = self.profile_start();
        self.load_static_images(super::assets::ImageGroup::Follow)?;
        self.profile_add("renderer.open.load_follow_vehicle_texture", start);
        let start = self.profile_start();
        self.load_static_images(super::assets::ImageGroup::Status)?;
        self.profile_add("renderer.open.load_drive_status_textures", start);
        self.profile_add("renderer.open.total", total);
        Ok(())
    }
    /// # Errors
    /// Resource cleanup currently completes synchronously on the render thread.
    pub fn close(&mut self) -> Result<()> {
        self.stats.close();
        let camera_result = self.close_live_road_camera();
        self.scene_cache.clear();
        self.output_resources.clear();
        self.world.close_resources();
        self.static_images.clear();
        self.world_text.clear();
        self.world_text_order.clear();
        self.camera_overlay.close_resources();
        self.trip_cache.close();
        self.route_video = super::route_video::RouteVideoCache::default();
        self.guidance = None;
        self.navi_guidance_size = None;
        self.guidance_hash.clear();
        self.media.clear();
        self.nav_model.clear();
        self.strokes.clear();
        self.stroke_order.clear();
        self.pending_strokes.clear();
        self.pending_order.clear();
        self.measure_cache.clear();
        if let Some(native) = self.gfx.native.as_mut() {
            native.release_dynamic_textures();
        }
        self.media_support = super::media_support::MediaSupport::default();
        self.gfx.native = None;
        self.font = 0;
        self.korean_font = None;
        camera_result
    }
    pub fn set_stats_provider(&mut self, provider: Box<dyn StatsProvider>) {
        self.stats.close();
        self.stats = provider;
    }
    pub fn set_profile_enabled(&mut self, enabled: bool) {
        self.profile_enabled = enabled;
    }
    pub fn clear_profile_samples(&mut self) {
        self.profile_samples.clear();
    }
    #[must_use]
    pub fn profile_samples(&self) -> &[(String, f64)] {
        &self.profile_samples
    }
    #[must_use]
    pub fn profile_start(&self) -> Option<Instant> {
        self.profile_enabled.then(Instant::now)
    }
    pub fn profile_add(&mut self, name: &str, start: Option<Instant>) {
        if let Some(start) = start {
            self.profile_add_elapsed(name, start.elapsed().as_secs_f64() * 1000.0);
        }
    }
    pub fn profile_add_elapsed(&mut self, name: &str, elapsed_ms: f64) {
        if self.profile_enabled {
            self.profile_samples.push((name.into(), elapsed_ms));
        }
    }
    #[must_use]
    pub fn driving_panel_offset_design_x(&self) -> f64 {
        if self.panel_swap_active() {
            super::tokens::NAVI_LIVE_PANEL_W
        } else {
            0.0
        }
    }
    pub fn set_theme_mode(&mut self, mode: &str) {
        let text = mode
            .trim_matches(openpilot_startup_ui::text::whitespace)
            .to_lowercase();
        self.theme_mode = match text.as_str() {
            "dark" | "1" => "dark",
            "light" | "2" => "light",
            _ => "auto",
        }
        .into();
        self.trip_cache.invalidate();
    }
    pub fn set_screen_mode(&mut self, mode: i64) {
        self.screen_mode = if (-1..=6).contains(&mode) { mode } else { 0 };
        self.trip_cache.invalidate();
    }
    pub fn set_panel_layout(&mut self, mode: i64) {
        self.panel_layout = i64::from(mode == 1);
        self.trip_cache.invalidate();
    }
    pub fn set_display_preferences(&mut self, language: &str, is_metric: bool) {
        self.language = display::normalize_language(language, "ko").into();
        self.is_metric = is_metric;
        self.trip_cache.invalidate();
    }
    #[must_use]
    pub fn current_theme(&self) -> ClusterTheme {
        let hour = chrono::Local::now().hour();
        if self.theme_mode == "dark" || self.theme_mode == "auto" && !(6..18).contains(&hour) {
            config::dark_theme()
        } else {
            config::light_theme()
        }
    }
    #[must_use]
    pub fn text<'a>(&self, key: &'a str) -> &'a str {
        display::text(&self.language, key)
    }
    #[must_use]
    pub fn panel_swap_active(&self) -> bool {
        self.panel_layout == 1 && !matches!(self.screen_mode, 3 | 6 | 7)
    }
    #[must_use]
    pub fn information_panel_x(&self, x: f64) -> f64 {
        x - if self.panel_swap_active() {
            super::tokens::NAVI_LIVE_PANEL_X
        } else {
            0.0
        }
    }
    /// # Errors
    /// Returns drawing or target readback errors; always ends the active target.
    pub fn capture_draw(
        &mut self,
        draw: impl FnOnce(&mut Self) -> Result<()>,
    ) -> Result<RenderedFrame> {
        self.render_to_capture(draw)?;
        let raw = self.gfx.backend()?.capture_pixels()?;
        let stride = usize::try_from(self.options.width)
            .map_err(|_| super::Error::Contract("capture width invalid"))?
            * 4;
        let mut rgba = Vec::with_capacity(raw.len());
        for row in raw.chunks_exact(stride).rev() {
            rgba.extend_from_slice(row);
        }
        Ok(RenderedFrame {
            width: self.options.width,
            height: self.options.height,
            rgba,
        })
    }
    /// # Errors
    /// Returns drawing or target allocation errors; always ends the active target.
    pub fn render_to_capture(&mut self, draw: impl FnOnce(&mut Self) -> Result<()>) -> Result<()> {
        self.open(self.output_resources.hidden)?;
        self.gfx
            .backend()?
            .ensure_render_target((self.options.width, self.options.height))?;
        self.gfx.backend()?.begin_capture()?;
        let result = draw(self);
        self.gfx.backend()?.end_capture();
        result?;
        self.flush_pending_strokes()
    }
    /// # Errors
    /// Returns an invalid RGBA buffer or image export error.
    pub fn write_png(&self, frame: &RenderedFrame, path: &Path) -> Result<()> {
        Ok(Renderer::export_rgba(
            (frame.width, frame.height),
            &frame.rgba,
            path,
        )?)
    }
    pub(super) fn texture_view(texture: &DynamicTexture) -> TextureView {
        TextureView {
            id: texture.id(),
            width: texture.width,
            height: texture.height,
        }
    }
    fn load_fonts(&mut self) -> Result<()> {
        let fonts = self.root.join("openpilot/selfdrive/assets/fonts");
        let candidates = [
            fonts.join("KaiGenGothicKR-Bold.ttf"),
            self.root
                .join("openpilot/selfdrive/assets/addon/font/KaiGenGothicKR-Bold.ttf"),
            PathBuf::from(std::env::var("WINDIR").unwrap_or_else(|_| "C:/Windows".into()))
                .join("Fonts/malgun.ttf"),
            fonts.join("JetBrainsMono-Medium.ttf"),
            fonts.join("JetBrainsMono-Bold.ttf"),
            PathBuf::from(
                "/data/openpilot/openpilot/selfdrive/assets/fonts/KaiGenGothicKR-Bold.ttf",
            ),
            PathBuf::from("/usr/share/fonts/truetype/jetbrains-mono/JetBrainsMono-Medium.ttf"),
            PathBuf::from("/usr/share/fonts/TTF/JetBrainsMono-Medium.ttf"),
            PathBuf::from("/usr/local/share/fonts/JetBrainsMono-Medium.ttf"),
        ];
        let points: Vec<i32> = (0x20..0x0250)
            .chain(0x2000..0x2070)
            .chain([0x20A9])
            .chain(0x3131..0x3190)
            .chain(0xAC00..0xD7A4)
            .collect();
        for candidate in candidates.iter().filter(|path| path.exists()) {
            if let Ok(font) = self.gfx.backend()?.raw_font(candidate, 160, &[]) {
                self.font = font;
                break;
            }
        }
        for candidate in candidates.iter().filter(|path| path.exists()) {
            if let Ok(font) = self.gfx.backend()?.raw_font(candidate, 32, &points) {
                self.korean_font = Some(font);
                break;
            }
        }
        Ok(())
    }
}
impl Drop for ClusterUiRenderer {
    fn drop(&mut self) {
        if let Err(error) = self.close() {
            eprintln!("cluster renderer close: {error}");
        }
    }
}
