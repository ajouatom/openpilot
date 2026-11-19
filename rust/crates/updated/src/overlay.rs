use crate::{common, process::args, report, updater::Updater, Error};
use std::{
    fs::{self, OpenOptions},
    os::unix::fs::{DirBuilderExt, MetadataExt},
    path::Path,
};
fn text(path: &Path) -> Result<&str, Error> {
    path.to_str()
        .ok_or(Error::Contract("non UTF8 updater path"))
}
impl Updater<'_> {
    pub fn dismount_overlay(&mut self) -> Result<(), Error> {
        let merged = self.context.paths.merged();
        if common::is_mount(&merged) {
            report::info(self.context.logger, "unmounting existing overlay")?;
            self.context
                .commands
                .run(&args(&["sudo", "umount", "-l", text(&merged)?]), None)?;
        }
        Ok(())
    }
    pub fn init_overlay(&mut self) -> Result<(), Error> {
        let paths = self.context.paths.clone();
        let merged = paths.merged();
        fs::create_dir_all(&paths.staging)?;
        let transaction_lock = OpenOptions::new()
            .create(true)
            .truncate(false)
            .read(true)
            .write(true)
            .open(paths.staging.join(".runtime-update.lock"))?;
        transaction_lock
            .try_lock()
            .map_err(|_| Error::Contract("native source/package transaction is busy"))?;
        if paths
            .staging
            .join(".runtime-update-journal.json")
            .try_exists()?
        {
            return Err(Error::Contract(
                "pending activation journal requires startup recovery",
            ));
        }
        if paths.overlay_init().is_file() && common::is_mount(&merged) {
            let new_files = self.context.commands.run(
                &args(&[
                    "find",
                    text(&paths.base.join(".git"))?,
                    "-newer",
                    text(&paths.overlay_init())?,
                ]),
                None,
            )?;
            if new_files.lines().next().is_none() {
                return Ok(());
            }
            report::info(
                self.context.logger,
                ".git directory changed, recreating overlay",
            )?;
        }
        report::info(self.context.logger, "preparing new safe staging area")?;
        let ready = paths
            .finalized()
            .join(".runtime-update-ready.json")
            .try_exists()?;
        if !ready {
            self.params.put_bool("UpdateAvailable", false)?;
            common::set_consistent_flag(&paths, &paths.finalized(), false)?;
        }
        self.dismount_overlay()?;
        // Keep complete rollback checkouts, package helpers and the transaction
        // lock. Only the updater's disposable overlay directories are recreated.
        for directory in [
            &paths.upper(),
            &paths.metadata(),
            &merged,
            &paths.finalized(),
        ] {
            if directory == &paths.finalized() && ready {
                continue;
            }
            if directory.try_exists()? {
                common::remove_tree(directory)?;
            }
        }
        for directory in [&paths.upper(), &paths.metadata(), &merged] {
            fs::DirBuilder::new().mode(0o755).create(directory)?;
        }
        if fs::symlink_metadata(&paths.base)?.dev() != fs::symlink_metadata(&merged)?.dev() {
            return Err(Error::Contract("base and overlay merge directories are on different filesystems; not valid for overlay FS!"));
        }
        let flag = paths.base.join(".overlay_consistent");
        if flag.is_file() {
            fs::remove_file(flag)?;
        }
        common::touch(&paths.overlay_init())?;
        paths.sync()?;
        let options = format!(
            "lowerdir={},upperdir={},workdir={}",
            paths.base.display(),
            paths.upper().display(),
            paths.metadata().display()
        );
        self.context.commands.run(
            &args(&[
                "sudo",
                "mount",
                "-t",
                "overlay",
                "-o",
                &options,
                "none",
                text(&merged)?,
            ]),
            None,
        )?;
        self.context.commands.run(
            &args(&[
                "sudo",
                "chmod",
                "755",
                text(&paths.metadata().join("work"))?,
            ]),
            None,
        )?;
        let diff = self
            .context
            .commands
            .run(&args(&["git", "diff", "--submodule=diff"]), Some(&merged))?;
        self.params.put("GitDiff", diff.as_bytes())?;
        report::info(self.context.logger, &format!("git diff output:\n{diff}"))?;
        Ok(())
    }
    /// The finalized copy is a whole checkout (several GB). On a small disk it
    /// pushed free space under the deleter's floor, and the deleter then
    /// removed every drive log as soon as it closed (C3X Lite, 2026-11-17).
    /// Refuse the copy unless the deleter floor plus a log reserve stays free;
    /// an old copy that cannot be replaced is dropped too, since a start would
    /// install it over a checkout that has moved on.
    fn ensure_copy_space(&mut self) -> Result<(), Error> {
        let paths = self.context.paths;
        let finalized = paths.finalized();
        let reclaim = if finalized.try_exists()? {
            common::tree_bytes(&finalized)?
        } else {
            0
        };
        let needed = common::tree_bytes(&paths.merged())?;
        let (available, total) = common::filesystem_bytes(&paths.staging)?;
        let Some(error) = common::copy_space_shortage(available, reclaim, needed, total) else {
            return Ok(());
        };
        common::set_consistent_flag(paths, &finalized, false)?;
        if finalized.try_exists()? {
            common::remove_tree(&finalized)?;
        }
        Err(error)
    }
    pub fn finalize_update(&mut self) -> Result<(), Error> {
        report::info(
            self.context.logger,
            "creating finalized version of the overlay",
        )?;
        let paths = self.context.paths;
        if paths.base.join("rust/runtime-manifest.json").is_file() {
            let target = self.context.commands.run(
                &args(&["git", "rev-parse", "--verify", "HEAD^{commit}"]),
                Some(&paths.merged()),
            )?;
            self.context.commands.run(
                &args(&[
                    text(&paths.base.join("rust/bin/openpilot-runtime-install"))?,
                    "--stage-update",
                    "--root",
                    text(&paths.base)?,
                    "--source",
                    text(&paths.merged())?,
                    "--target",
                    target.trim(),
                    "--staging",
                    text(&paths.staging)?,
                    "--params-root",
                    text(&paths.system("/data/params"))?,
                ]),
                Some(&paths.base),
            )?;
            report::info(
                self.context.logger,
                "verified native source/package update staged for reboot",
            )?;
            return Ok(());
        }
        self.ensure_copy_space()?;
        common::set_consistent_flag(paths, &paths.finalized(), false)?;
        if paths.finalized().exists() {
            common::remove_tree(&paths.finalized())?;
        }
        common::copy_tree(&paths.merged(), &paths.finalized())?;
        self.context
            .commands
            .run(&args(&["git", "reset", "--hard"]), Some(&paths.finalized()))?;
        self.context.commands.run(
            &args(&[
                "git",
                "submodule",
                "foreach",
                "--recursive",
                "git",
                "reset",
                "--hard",
            ]),
            Some(&paths.finalized()),
        )?;
        report::info(
            self.context.logger,
            "Starting LFS cleanup in finalized update (Git repack skipped)",
        )?;
        let started = self.context.clock.monotonic()?;
        match self
            .context
            .commands
            .run(&args(&["git", "lfs", "prune"]), Some(&paths.finalized()))
        {
            Ok(_) => report::event(
                self.context.logger,
                "Done LFS cleanup",
                serde_json::json!({"duration":(self.context.clock.monotonic()? - started) as f64 / 1e9}),
            )?,
            Err(error @ Error::Command { .. }) => report::exception(
                self.context.logger,
                &format!(
                    "Failed LFS cleanup, took {:.3} s",
                    (self.context.clock.monotonic()? - started) as f64 / 1e9
                ),
                &error,
            )?,
            Err(error) => return Err(error),
        }
        common::finalize_runtime_package(paths, self.context.commands)?;
        report::info(self.context.logger, "done finalizing overlay")?;
        Ok(())
    }
    pub fn handle_agnos_update(&mut self) -> Result<(), Error> {
        let paths = self.context.paths;
        let current = self.context.hardware.get_os_version()?;
        let updated = self.context.commands.run(&args(&["bash", "-c", "unset AGNOS_VERSION && source launch_env.sh && \\\n                          echo -n $AGNOS_VERSION"]), Some(&paths.merged()))?.trim().to_owned();
        report::info(
            self.context.logger,
            &format!(
                "AGNOS version check: {} vs {updated}",
                current.as_deref().unwrap_or("None")
            ),
        )?;
        if current.as_deref() == Some(&updated) {
            return Ok(());
        }
        common::set_consistent_flag(paths, &paths.finalized(), false)?;
        report::info(
            self.context.logger,
            &format!("Beginning background installation for AGNOS {updated}"),
        )?;
        self.params.alert("Offroad_NeosUpdate", true, None)?;
        let mut manifest = paths
            .merged()
            .join("openpilot/system/hardware/tici/agnos.json");
        match fs::read_to_string(paths.system("/sys/firmware/devicetree/base/model")) {
            Ok(model) => {
                let model = model.replace('\0', "").trim().to_lowercase();
                let model = model.strip_prefix("comma ").unwrap_or(&model);
                println!("[agnos] device model: {model}");
                if matches!(model, "c3" | "tici") {
                    manifest = paths
                        .merged()
                        .join("openpilot/system/hardware/tici/agnos-tici.json");
                    println!("[agnos] manifest_path: {}", manifest.display());
                }
            }
            Err(error) if error.kind() != std::io::ErrorKind::InvalidData => {
                println!("[agnos] model read failed: {error}")
            }
            Err(error) => return Err(error.into()),
        }
        let slot = self.context.agnos.get_target_slot_number()?;
        self.context.agnos.flash_agnos_update(&manifest, slot)?;
        self.params.alert("Offroad_NeosUpdate", false, None)?;
        Ok(())
    }
}
