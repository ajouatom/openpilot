//! The last device build and update cleanup, appended to the startup report.
//! Both run before openpilot starts, outside tmux, so the report showed
//! neither whether an update rebuilt everything nor what the cleanup freed.
use std::{
    fs,
    io::{Read, Seek, SeekFrom},
    path::Path,
    time::SystemTime,
};

/// Lines the summary keeps from each log.
const LINES: usize = 20;
/// Bytes read from the end of a log (cargo output can make build.log large).
const TAIL_BYTES: u64 = 256 << 10;

fn tail(path: &Path) -> Option<String> {
    let mut file = fs::File::open(path).ok()?;
    let length = file.metadata().ok()?.len();
    file.seek(SeekFrom::Start(length.saturating_sub(TAIL_BYTES)))
        .ok()?;
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes).ok()?;
    Some(String::from_utf8_lossy(&bytes).into_owned())
}

fn age(path: &Path, now: SystemTime) -> String {
    fs::metadata(path)
        .and_then(|m| m.modified())
        .ok()
        .and_then(|modified| now.duration_since(modified).ok())
        .map_or_else(
            || "age unknown".to_owned(),
            |age| format!("{} min before this report", age.as_secs() / 60),
        )
}

/// build.log's own status lines (`[12s] ...`), not cargo's output.
fn status_line(line: &str) -> bool {
    line.strip_prefix('[')
        .and_then(|rest| rest.split_once("s] "))
        .is_some_and(|(seconds, _)| {
            !seconds.is_empty() && seconds.bytes().all(|b| b.is_ascii_digit())
        })
}

fn last(lines: Vec<&str>) -> String {
    let start = lines.len().saturating_sub(LINES);
    lines[start..].join("\n")
}

pub fn summary(store: &Path, now: SystemTime) -> String {
    let mut out = String::new();
    let build = store.join("build.log");
    if let Some(text) = tail(&build) {
        out.push_str(&format!(
            "\n==== last device build ({}, {}) ====\n{}\n",
            build.display(),
            age(&build, now),
            last(text.lines().filter(|l| status_line(l)).collect())
        ));
    }
    let cleanup = store.join("cleanup.log");
    if let Some(text) = tail(&cleanup) {
        out.push_str(&format!(
            "==== update cleanup ({}, {}) ====\n{}\n",
            cleanup.display(),
            age(&cleanup, now),
            last(text.lines().collect())
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keeps_status_lines_and_the_cleanup_record() {
        let dir = std::env::temp_dir().join(format!("carrot-build-summary-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        fs::write(
            dir.join("build.log"),
            "[0s] Compiling with 6 jobs\n   Compiling foo v1.0\nwarning: x\n[612s] Build complete in 612s: 3 crates compiled for 24d52e677\n",
        )
        .unwrap();
        fs::write(
            dir.join("cleanup.log"),
            "[t] update cleanup: free 5000 MB -> 7400 MB\n",
        )
        .unwrap();
        let text = summary(&dir, SystemTime::now());
        assert!(text.contains("[0s] Compiling with 6 jobs"));
        assert!(text.contains("3 crates compiled for 24d52e677"));
        assert!(!text.contains("Compiling foo"));
        assert!(!text.contains("warning"));
        assert!(text.contains("free 5000 MB -> 7400 MB"));
        assert!(text.contains("0 min before this report"));
        let _ = fs::remove_dir_all(&dir);
        assert_eq!(summary(&dir, SystemTime::now()), "");
    }

    #[test]
    fn status_lines_only() {
        assert!(status_line("[0s] Installing"));
        assert!(status_line("[1234s] Build complete"));
        assert!(!status_line("[s] x"));
        assert!(!status_line("[ab] x"));
        assert!(!status_line("   Compiling foo"));
    }
}
