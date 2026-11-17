//! Scoped NCM address, exact INPUT rule and helper-owned DHCP lifecycle.
use super::{
    platform::{self, remove, text, valid_interface, Commands, Paths},
    process,
};
use crate::Error;
use std::{
    fs,
    os::unix::fs::{MetadataExt, PermissionsExt},
};
const ADDRESS: &str = "192.168.60.1/24";
fn backend(paths: &Paths, os: &impl Commands) -> Result<String, Error> {
    let saved = paths.state.join("firewall-backend");
    if saved.try_exists()? {
        let name = text(&saved)?;
        if !matches!(name.as_str(), "iptables" | "iptables-legacy") {
            return Err(Error::Contract("invalid owned firewall backend"));
        }
        platform::checked(os, &name, &["-w", "2", "-S", "INPUT"])?;
        return Ok(name);
    }
    for name in ["iptables", "iptables-legacy"] {
        match platform::run(os, name, &["-w", "2", "-S", "INPUT"]) {
            Ok(output) if output.code == 0 => {
                fs::write(&saved, format!("{name}\n"))?;
                return Ok(name.into());
            }
            Ok(_) => {}
            Err(Error::Io(error)) if error.kind() == std::io::ErrorKind::NotFound => {}
            Err(error) => return Err(error),
        }
    }
    Err(Error::Contract("no operational INPUT isolation backend"))
}
fn rule(os: &impl Commands, backend: &str, action: &str, interface: &str) -> Result<i32, Error> {
    if !valid_interface(interface) {
        return Err(Error::Contract("invalid owned firewall interface"));
    }
    let mut args = vec!["-w", "2", action, "INPUT"];
    if action == "-I" {
        args.push("1");
    }
    args.extend([
        "-d",
        "192.168.60.1/32",
        "-p",
        "tcp",
        "--dport",
        "5599",
        "!",
        "-i",
        interface,
        "-m",
        "comment",
        "--comment",
        "jetlink-mobile-owned-5599",
        "-j",
        "DROP",
    ]);
    Ok(platform::run(os, backend, &args)?.code)
}
fn firewall_down(paths: &Paths, os: &impl Commands) -> Result<(), Error> {
    let saved = paths.state.join("firewall-interface");
    if saved.try_exists()? {
        let interface = text(&saved)?;
        if !valid_interface(&interface) {
            return Err(Error::Contract("invalid owned firewall interface"));
        }
        if !paths.state.join("firewall-backend").try_exists()? {
            fs::write(paths.state.join("firewall-backend"), "iptables\n")?;
        }
        let backend = backend(paths, os)?;
        match rule(os, &backend, "-C", &interface)? {
            0 => {
                if rule(os, &backend, "-D", &interface)? != 0 {
                    return Err(Error::Contract("could not remove exact owned INPUT rule"));
                }
            }
            1 => {}
            _ => return Err(Error::Contract("could not inspect exact owned INPUT rule")),
        }
        remove(&saved)?;
        remove(&paths.state.join("firewall-backend"))?;
    }
    Ok(())
}
fn firewall_up(paths: &Paths, os: &impl Commands, interface: &str) -> Result<(), Error> {
    firewall_down(paths, os)?;
    let backend = backend(paths, os)?;
    fs::write(
        paths.state.join("firewall-interface"),
        format!("{interface}\n"),
    )?;
    if rule(os, &backend, "-I", interface)? != 0 || rule(os, &backend, "-C", interface)? != 0 {
        return Err(Error::Contract(
            "mobile INPUT interface isolation not verified",
        ));
    }
    Ok(())
}
fn leases(paths: &Paths, os: &impl Commands) -> Result<String, Error> {
    let uid = platform::checked(os, "id", &["-u", "nobody"])?
        .trim()
        .parse::<u32>()
        .map_err(|_| Error::Contract("DHCP nobody UID unavailable"))?;
    let group = platform::checked(os, "id", &["-gn", "nobody"])?
        .trim()
        .to_owned();
    let gid = platform::checked(os, "id", &["-g", "nobody"])?
        .trim()
        .parse::<u32>()
        .map_err(|_| Error::Contract("DHCP nobody GID unavailable"))?;
    if paths.leases == paths.state || paths.leases.is_symlink() {
        return Err(Error::Contract("unsafe DHCP lease directory"));
    }
    if paths.leases.try_exists()? {
        let meta = fs::metadata(&paths.leases)?;
        if !meta.is_dir() || meta.uid() != uid {
            return Err(Error::Contract("DHCP lease directory not owned by nobody"));
        }
    } else {
        fs::create_dir_all(&paths.leases)?;
        rustix::fs::chown(
            &paths.leases,
            Some(rustix::process::Uid::from_raw(uid)),
            Some(rustix::process::Gid::from_raw(gid)),
        )
        .map_err(std::io::Error::from)?;
    }
    fs::set_permissions(&paths.leases, fs::Permissions::from_mode(0o700))?;
    let path = paths.leases.join("dnsmasq.leases");
    if path.is_symlink() {
        return Err(Error::Contract("DHCP lease file symlink"));
    }
    if path.try_exists()? {
        let meta = fs::metadata(&path)?;
        if !meta.is_file() || meta.uid() != uid {
            return Err(Error::Contract("DHCP lease file not owned by nobody"));
        }
    } else {
        let file = fs::OpenOptions::new()
            .create_new(true)
            .write(true)
            .open(&path)?;
        file.set_permissions(fs::Permissions::from_mode(0o600))?;
        rustix::fs::fchown(
            &file,
            Some(rustix::process::Uid::from_raw(uid)),
            Some(rustix::process::Gid::from_raw(gid)),
        )
        .map_err(std::io::Error::from)?;
    }
    Ok(group)
}
pub(crate) fn down(paths: &Paths, os: &impl Commands) -> Result<(), Error> {
    process::stop(paths)?;
    firewall_down(paths, os)?;
    let saved = paths.state.join("interface");
    if saved.try_exists()? {
        let value = text(&saved)?;
        let fields = value.split_whitespace().collect::<Vec<_>>();
        if fields.len() == 2 && valid_interface(fields[0]) {
            let index = paths.net.join(fields[0]).join("ifindex");
            if index.try_exists()? && text(&index)? == fields[1] {
                // The cable can be pulled between the ifindex check and `ip`:
                // the interface and its address are then already gone, which
                // is the state this cleanup wants (route 0000027e: jetlinkd
                // died on `Cannot find device "usb0"`).
                let gone = || !index.exists();
                match platform::checked(
                    os,
                    "ip",
                    &["-o", "-4", "address", "show", "dev", fields[0]],
                ) {
                    Ok(output) if output.contains(" 192.168.60.1/24 ") => {
                        match platform::checked(
                            os,
                            "ip",
                            &["address", "del", ADDRESS, "dev", fields[0]],
                        ) {
                            Err(_) if gone() => {}
                            other => {
                                other?;
                            }
                        }
                    }
                    Ok(_) => {}
                    Err(_) if gone() => {}
                    Err(error) => return Err(error),
                }
            }
        }
        remove(&saved)?;
    }
    Ok(())
}
pub(crate) fn up(paths: &Paths, os: &impl Commands) -> Result<(), Error> {
    let (interface, index) = paths.interface()?;
    if text(&paths.gadget.join("UDC"))?.is_empty() {
        return Err(Error::Contract("NCM controller not bound"));
    }
    let saved = paths.state.join("interface");
    let identity = format!("{interface} {index}");
    if saved.try_exists()? && text(&saved)? != identity {
        down(paths, os)?;
    }
    fs::write(saved, format!("{identity}\n"))?;
    match platform::run(os, "nmcli", &["device", "set", &interface, "managed", "no"]) {
        Ok(output) if output.code == 0 => {}
        Err(Error::Io(error)) if error.kind() == std::io::ErrorKind::NotFound => {}
        _ => return Err(Error::Contract("could not unmanage owned NCM interface")),
    }
    platform::checked(
        os,
        "ip",
        &["address", "replace", ADDRESS, "dev", &interface],
    )?;
    platform::checked(os, "ip", &["link", "set", &interface, "up"])?;
    firewall_up(paths, os, &interface)?;
    if paths.state.join("dnsmasq.pid").try_exists()? {
        if !process::alive(paths)? {
            return Err(Error::Contract("existing DHCP state not live and owned"));
        }
    } else {
        let group = leases(paths, os)?;
        let arguments = vec![
            "--conf-file=/dev/null".into(),
            "--bind-interfaces".into(),
            format!("--interface={interface}"),
            "--listen-address=192.168.60.1".into(),
            "--except-interface=lo".into(),
            "--port=0".into(),
            "--dhcp-range=192.168.60.2,192.168.60.254,255.255.255.0,10m".into(),
            "--dhcp-option=3".into(),
            "--dhcp-option=6".into(),
            format!(
                "--dhcp-leasefile={}",
                paths.leases.join("dnsmasq.leases").display()
            ),
            "--user=nobody".into(),
            format!("--group={group}"),
            format!("--pid-file={}", paths.state.join("dnsmasq.pid").display()),
        ];
        let output = os.run("dnsmasq", &arguments)?;
        if output.code != 0 {
            return Err(Error::Native(format!(
                "mobile DHCP startup: {}",
                output.stdout.chars().take(1024).collect::<String>()
            )));
        }
        process::record(paths)?;
    }
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::super::platform::Output;
    use super::*;
    use std::cell::RefCell;
    struct Os {
        calls: RefCell<Vec<Vec<String>>>,
        check: i32,
    }
    impl Commands for Os {
        fn run(&self, program: &str, args: &[String]) -> Result<Output, Error> {
            self.calls.borrow_mut().push(
                std::iter::once(program.to_owned())
                    .chain(args.iter().cloned())
                    .collect(),
            );
            Ok(Output {
                code: if args.iter().any(|s| s == "-C") {
                    self.check
                } else {
                    0
                },
                stdout: if program == "ip" {
                    "2: usb0 inet 192.168.60.1/24 scope global usb0".into()
                } else {
                    String::new()
                },
            })
        }
    }
    fn fixture() -> (tempfile::TempDir, Paths) {
        let dir = tempfile::tempdir().unwrap();
        let paths = Paths {
            gadget: dir.path().join("gadget"),
            state: dir.path().join("state"),
            net: dir.path().join("net"),
            leases: dir.path().join("leases"),
            proc: dir.path().join("proc"),
        };
        fs::create_dir_all(&paths.state).unwrap();
        (dir, paths)
    }
    #[test]
    fn exact_interface_firewall_is_first_and_verified_before_listener() {
        let (_dir, paths) = fixture();
        let os = Os {
            calls: RefCell::new(Vec::new()),
            check: 0,
        };
        firewall_up(&paths, &os, "usb0").unwrap();
        let calls = os.calls.borrow();
        let insertion = calls
            .iter()
            .find(|args| args.contains(&"-I".to_owned()))
            .unwrap();
        assert_eq!(&insertion[1..6], ["-w", "2", "-I", "INPUT", "1"]);
        assert!(insertion.windows(3).any(|v| v == ["!", "-i", "usb0"]));
        assert!(insertion.contains(&"jetlink-mobile-owned-5599".into()));
        assert_eq!(
            text(&paths.state.join("firewall-interface")).unwrap(),
            "usb0"
        );
        drop(calls);
        firewall_down(&paths, &os).unwrap();
        assert!(!paths.state.join("firewall-interface").exists());
    }
    #[test]
    fn network_cleanup_never_removes_address_from_reused_interface() {
        let (_dir, paths) = fixture();
        fs::create_dir_all(paths.net.join("usb0")).unwrap();
        fs::write(paths.net.join("usb0/ifindex"), "9").unwrap();
        fs::write(paths.state.join("interface"), "usb0 8").unwrap();
        let os = Os {
            calls: RefCell::new(Vec::new()),
            check: 0,
        };
        down(&paths, &os).unwrap();
        assert!(os.calls.borrow().is_empty());
        fs::write(paths.state.join("interface"), "usb0 9").unwrap();
        down(&paths, &os).unwrap();
        assert!(os
            .calls
            .borrow()
            .iter()
            .any(|v| v.windows(2).any(|v| v == ["address", "del"])));
    }
    struct Unplugged {
        index: std::path::PathBuf,
    }
    impl Commands for Unplugged {
        fn run(&self, _program: &str, _args: &[String]) -> Result<Output, Error> {
            // The cable goes between the ifindex check and `ip`.
            let _ = fs::remove_dir_all(self.index.parent().unwrap());
            Ok(Output {
                code: 1,
                stdout: "Cannot find device \"usb0\"".into(),
            })
        }
    }
    #[test]
    fn network_cleanup_of_an_interface_unplugged_meanwhile_succeeds() {
        let (_dir, paths) = fixture();
        fs::create_dir_all(paths.net.join("usb0")).unwrap();
        fs::write(paths.net.join("usb0/ifindex"), "9").unwrap();
        fs::write(paths.state.join("interface"), "usb0 9").unwrap();
        let os = Unplugged {
            index: paths.net.join("usb0/ifindex"),
        };
        down(&paths, &os).unwrap();
        assert!(!paths.state.join("interface").exists());
    }
    #[test]
    fn network_cleanup_error_on_a_present_interface_is_kept() {
        let (_dir, paths) = fixture();
        fs::create_dir_all(paths.net.join("usb0")).unwrap();
        fs::write(paths.net.join("usb0/ifindex"), "9").unwrap();
        fs::write(paths.state.join("interface"), "usb0 9").unwrap();
        struct Fails;
        impl Commands for Fails {
            fn run(&self, _program: &str, _args: &[String]) -> Result<Output, Error> {
                Ok(Output {
                    code: 1,
                    stdout: "busy".into(),
                })
            }
        }
        assert!(down(&paths, &Fails).is_err());
        assert!(paths.state.join("interface").exists());
    }
    #[test]
    fn firewall_inspection_error_retains_owned_state_and_refuses_cleanup() {
        let (_dir, paths) = fixture();
        fs::write(paths.state.join("firewall-interface"), "usb0").unwrap();
        let os = Os {
            calls: RefCell::new(Vec::new()),
            check: 2,
        };
        assert!(firewall_down(&paths, &os).is_err());
        assert!(paths.state.join("firewall-interface").exists());
        assert!(!os.calls.borrow().iter().any(|v| v.contains(&"-D".into())));
    }
}
