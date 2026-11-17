//! Renders the N16 route map for each frame of an N16 replay
//! (tools/jetlink/n16_replay) with mapsd's renderer, route tracker and tile store
//! and Mapbox tiles (fetched once, then kept in TILE_DIR):
//! cargo run --release --features native --example replay_maps -- POSITIONS.json OUT.bin TILE_DIR
//! Token from MAPBOX_TOKEN. OUT.bin: per frame u32 frame id, u8 valid, 65536 bytes.
use openpilot_mapsd::{
    remote::Mapbox,
    renderer::{MapRenderer, Position},
    route::RouteTracker,
    tiles::HybridTiles,
};
use std::io::Write;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let input: serde_json::Value = serde_json::from_slice(&std::fs::read(&args[1])?)?;
    let routes: Vec<Vec<[f64; 2]>> = serde_json::from_value(input["routes"].clone())?;
    let start = std::time::Instant::now();
    let clock: Box<dyn FnMut() -> f64 + Send> = Box::new(move || start.elapsed().as_secs_f64());
    let mut tiles = HybridTiles::new(
        std::path::Path::new(&args[3]),
        Some(Mapbox::new(std::env::var("MAPBOX_TOKEN")?)),
        clock,
    )?;
    let mut renderer = MapRenderer::default();
    let mut out = std::fs::File::create(&args[2])?;
    let (mut tracker, mut current) = (None::<RouteTracker>, usize::MAX);
    let (mut valid_count, mut total) = (0, 0);
    for f in input["frames"].as_array().unwrap() {
        let r = f["route"].as_u64().unwrap() as usize;
        if r != current {
            current = r;
            tracker = RouteTracker::new(routes[r].clone());
        }
        let (lat, lon, adv) = (
            f["lat"].as_f64().unwrap(),
            f["lon"].as_f64().unwrap(),
            f["advance"].as_f64().unwrap(),
        );
        let mut image = vec![0u8; 65536];
        let mut valid = false;
        if let Some((plon, plat, bearing)) = tracker
            .as_mut()
            .and_then(|t| t.update_pose(lon, lat, adv, 50.))
        {
            let result = renderer.render(
                &mut tiles,
                Position {
                    latitude: plat,
                    longitude: plon,
                    bearing,
                },
                Some(&routes[r]),
            )?;
            if result.valid() {
                valid = true;
                image = result.image;
            }
        }
        total += 1;
        valid_count += usize::from(valid);
        out.write_all(&(f["frame"].as_u64().unwrap() as u32).to_le_bytes())?;
        out.write_all(&[u8::from(valid)])?;
        out.write_all(&image)?;
    }
    eprintln!("rendered {valid_count}/{total} valid maps");
    Ok(())
}
