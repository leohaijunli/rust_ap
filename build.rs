use std::{env, fs, path::Path, path::PathBuf, process::Command};

fn source_files(path: &Path, files: &mut Vec<PathBuf>) {
    for entry in fs::read_dir(path).expect("read source directory") {
        let path = entry.expect("read source entry").path();
        if path.is_dir() {
            source_files(&path, files);
        } else {
            files.push(path);
        }
    }
}

fn main() {
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").expect("Cargo provides OUT_DIR"));
    if env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("none") {
        fs::write(out_dir.join("memory.x"), include_bytes!("memory.x"))
            .expect("write the linker memory layout");
        println!("cargo:rustc-link-search={}", out_dir.display());
        let target_dir = env::var_os("CARGO_TARGET_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("target"));
        let release_dir = target_dir
            .join(env::var("TARGET").expect("TARGET"))
            .join(env::var("PROFILE").expect("PROFILE"));
        fs::create_dir_all(&release_dir).expect("create artifact directory");
        println!(
            "cargo:rustc-link-arg-bin=v6x-rust-led=-Map={}",
            release_dir.join("v6x-rust-led.map").display()
        );
    }

    // Deterministic diagnostic ID, not a cryptographic integrity check. The
    // build report separately records SHA-256 for sources and all artifacts.
    let mut files: Vec<_> = [
        "Cargo.toml",
        "Cargo.lock",
        "build.rs",
        "memory.x",
        "rust-toolchain.toml",
        ".cargo/config.toml",
    ]
    .into_iter()
    .map(PathBuf::from)
    .collect();
    source_files(Path::new("src"), &mut files);
    files.sort();
    let mut id = 0xcbf2_9ce4_8422_2325u64;
    let mut hash = |bytes: &[u8]| {
        for &byte in bytes {
            id = (id ^ u64::from(byte)).wrapping_mul(0x100_0000_01b3);
        }
    };
    for path in files {
        println!("cargo:rerun-if-changed={}", path.display());
        // Posix separators keep the identity identical across host platforms.
        hash(path.to_string_lossy().replace('\\', "/").as_bytes());
        hash(&[0]);
        hash(&fs::read(path).expect("read source"));
        hash(&[0]);
    }
    let mut features = Vec::new();
    if env::var_os("CARGO_FEATURE_FAULT_INJECTION").is_some() {
        features.push("fault-injection");
    }
    if env::var_os("CARGO_FEATURE_FAULT_DUMP").is_some() {
        features.push("fault-dump");
    }
    features.sort();
    if features.is_empty() {
        hash(b"default");
    }
    for feature in features {
        hash(b"feature:");
        hash(feature.as_bytes());
        hash(&[0]);
    }

    let git = Command::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()
        .filter(|output| output.status.success())
        .map(|output| String::from_utf8_lossy(&output.stdout).trim().to_owned())
        .unwrap_or_else(|| "unknown".to_owned());
    println!("cargo:rustc-env=BUILD_GIT={git}");
    println!("cargo:rustc-env=BUILD_SOURCE_ID={id:016x}");
    println!("cargo:rerun-if-changed=src");
    println!("cargo:rerun-if-changed=.git/HEAD");
    println!("cargo:rerun-if-changed=.git/refs");
    println!("cargo:rerun-if-env-changed=CARGO_TARGET_DIR");

    let major: u32 = env::var("CARGO_PKG_VERSION_MAJOR")
        .unwrap()
        .parse()
        .unwrap();
    let minor: u32 = env::var("CARGO_PKG_VERSION_MINOR")
        .unwrap()
        .parse()
        .unwrap();
    let patch: u32 = env::var("CARGO_PKG_VERSION_PATCH")
        .unwrap()
        .parse()
        .unwrap();
    let version = major << 24 | minor << 16 | patch << 8; // MAVLink development type = 0.
    fs::write(
        out_dir.join("build_info.rs"),
        format!(
            "pub const SOURCE_ID: u64 = {id};\npub const FLIGHT_SW_VERSION: u32 = {version};\n"
        ),
    )
    .expect("write build identity");
}
