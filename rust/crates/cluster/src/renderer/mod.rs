mod assets;
mod background;
pub mod camera_overlay;
mod core;
pub mod display;
mod error;
mod frame;
pub mod geometry;
mod graphics;
mod graphics_3d;
pub mod hud;
pub mod linux_stats;
mod media;
mod media_draw;
mod media_hardware;
mod media_support;
mod media_yuv;
mod nav_model;
pub mod navigation;
pub mod nv12;
mod output;
mod panels;
mod replay_input;
mod route_video;
mod scene_cache;
mod signal_display;
mod stats;
mod system_panel;
mod text;
pub mod tokens;
mod trip_cache;
mod types;
pub mod world;
pub use assets::StaticImage;
pub use core::{ClusterUiRenderer, StatsProvider};
pub use error::{Error, Result};
pub use geometry::{with_alpha, Anchor, IntoColor, MediaFit, Point, Rect, TextureView};
pub use graphics::Graphics;
pub use openpilot_startup_ui::renderer::{
    MeshResource, ModelResource, PointBuffer2D, PointBuffer3D, RawPoint,
};
pub use output::CaptureOptions;
pub use replay_input::ReplayInput;
pub use types::{
    FrameClock, Options, RenderedFrame, Ring, StrokeStyle, SystemPanelOptions, SystemStats,
};

#[cfg(test)]
#[path = "display_tests.rs"]
mod tests;
