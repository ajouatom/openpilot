//! `ClusterHudScreenMode` 7 (carrot `_draw_nav_model_map`): the normal
//! driving view on the left, the N16 navigation model's black-and-white route
//! map in the right information panel.
//!
//! carrot loads mapsd's PNG preview; this reads mapsd's raw 256x256 map
//! (`openpilot_mapsd::output`), the same image, without the PNG encode and
//! decode. As carrot: checked at most twice a second, uploaded only when it
//! changed, shown while under 2 s old, a waiting line otherwise.
use super::tokens::{NAVI_LIVE_PANEL_W, NAVI_LIVE_PANEL_X};
use super::{geometry::Anchor, ClusterUiRenderer, Point, Rect, Result};
use openpilot_startup_ui::{
    draw::PixelBuffer,
    renderer::{DynamicTexture, TextureFilter},
};
use std::{
    path::Path,
    time::{Duration, Instant, SystemTime},
};

/// `crate::config::DESIGN_HEIGHT`.
const DESIGN_HEIGHT: f64 = 480.0;
const SIZE: i32 = 256;
const CHECK_EVERY: Duration = Duration::from_millis(500);
const MAX_AGE: Duration = Duration::from_secs(2);

#[derive(Default)]
pub(super) struct NavModelMap {
    texture: Option<DynamicTexture>,
    checked: Option<Instant>,
    modified: Option<SystemTime>,
    fresh: bool,
    rgba: Vec<u8>,
}

impl NavModelMap {
    pub fn clear(&mut self) {
        *self = Self::default();
    }
}

/// The gray map as RGBA, into `rgba`'s reused storage.
pub(super) fn expand_gray(gray: &[u8], rgba: &mut Vec<u8>) {
    rgba.clear();
    rgba.reserve(gray.len() * 4);
    for &value in gray {
        rgba.extend_from_slice(&[value, value, value, 255]);
    }
}

fn modified(path: &Path) -> Option<SystemTime> {
    std::fs::metadata(path)
        .and_then(|meta| meta.modified())
        .ok()
}

impl ClusterUiRenderer {
    fn refresh_nav_model_map(&mut self) -> Result<()> {
        let now = Instant::now();
        if self
            .nav_model
            .checked
            .is_some_and(|checked| now.duration_since(checked) < CHECK_EVERY)
        {
            return Ok(());
        }
        self.nav_model.checked = Some(now);
        let path = Path::new(openpilot_mapsd::output::PATH);
        let stamp = modified(path);
        self.nav_model.fresh = stamp
            .and_then(|stamp| SystemTime::now().duration_since(stamp).ok())
            .is_some_and(|age| age < MAX_AGE);
        if !self.nav_model.fresh || stamp == self.nav_model.modified {
            return Ok(());
        }
        let Some(map) = openpilot_mapsd::output::read(path) else {
            self.nav_model.fresh = false;
            return Ok(());
        };
        let mut rgba = std::mem::take(&mut self.nav_model.rgba);
        expand_gray(&map.image, &mut rgba);
        let pixels = PixelBuffer {
            dimensions: (SIZE, SIZE),
            rgba: &rgba,
        };
        let uploaded = match &self.nav_model.texture {
            Some(texture) => self
                .gfx
                .backend()?
                .update_dynamic_pixels(texture, pixels)
                .is_ok(),
            None => match self
                .gfx
                .backend()?
                .dynamic_filtered_image(pixels, TextureFilter::Bilinear)
            {
                Ok(texture) => {
                    self.nav_model.texture = Some(texture);
                    true
                }
                Err(_) => false,
            },
        };
        self.nav_model.rgba = rgba;
        if uploaded {
            self.nav_model.modified = stamp;
        } else {
            self.nav_model.fresh = false;
        }
        Ok(())
    }

    /// # Errors
    /// Returns graphics errors from the panel, the texture or the text.
    pub(super) fn draw_nav_model_map(&mut self) -> Result<()> {
        // Only the right information panel, in 3D views too.
        self.gfx.rectangle(
            Rect::new(NAVI_LIVE_PANEL_X, 0., NAVI_LIVE_PANEL_W, DESIGN_HEIGHT),
            [0, 0, 0, 255],
        )?;
        let center_x = NAVI_LIVE_PANEL_X + NAVI_LIVE_PANEL_W / 2.;
        self.refresh_nav_model_map()?;
        let view = self
            .nav_model
            .texture
            .as_ref()
            .filter(|_| self.nav_model.fresh)
            .map(Self::texture_view);
        if let Some(view) = view {
            let size = (DESIGN_HEIGHT - 80.).min(NAVI_LIVE_PANEL_W - 32.);
            self.gfx.texture_pro(
                view,
                Rect::new(0., 0., f64::from(SIZE), f64::from(SIZE)),
                Rect::new(center_x - size / 2., 20., size, size),
                Point::default(),
                0.,
                [255, 255, 255],
            )?;
            self.draw_text(
                "Mapbox / Protomaps | (c) OpenStreetMap",
                center_x,
                DESIGN_HEIGHT - 40.,
                14.,
                [180, 180, 180],
                Anchor::Center,
            )
        } else {
            self.draw_text(
                "NAV MAP: waiting for route / GPS / tiles",
                center_x,
                DESIGN_HEIGHT / 2.,
                24.,
                [255, 255, 255],
                Anchor::Center,
            )
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn gray_expands_to_opaque_rgba_in_reused_storage() {
        let mut rgba = vec![9; 100];
        super::expand_gray(&[0, 128, 255], &mut rgba);
        assert_eq!(rgba, [0, 0, 0, 255, 128, 128, 128, 255, 255, 255, 255, 255]);
    }
}
