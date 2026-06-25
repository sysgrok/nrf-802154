//! Build Script for nrf-802154-sys
//!
//! Calls out to bindgen to generate a Rust crate from the Nordic header
//! files.

use std::env;
use std::fs::OpenOptions;
use std::io::Write;
use std::path::PathBuf;

enum Series {
    Nrf52,
    Nrf53,
    Nrf54l,
    Nrf54lNs,
    Nrf54h,
}

impl Series {
    fn get() -> Self {
        let nrf52 = cfg!(feature = "nrf52");
        let nrf53 = cfg!(feature = "nrf53");
        let nrf54l_s = cfg!(feature = "nrf54l-s");
        let nrf54l_ns = cfg!(feature = "nrf54l-ns");
        let nrf54h = cfg!(feature = "nrf54h");
        match (nrf52, nrf53, nrf54l_s, nrf54l_ns, nrf54h) {
            (true, false, false, false, false) => Series::Nrf52,
            (false, true, false, false, false) => Series::Nrf53,
            (false, false, true, false, false) => Series::Nrf54l,
            (false, false, false, true, false) => Series::Nrf54lNs,
            (false, false, false, false, true) => Series::Nrf54h,
            _ => panic!("Exactly one architecture feature must be enabled for nrf-802154-sys"),
        }
    }
}

#[derive(Debug)]
struct Target {
    target: String,
    cpu: &'static str,
    float_abi: &'static str,
    chip_family: &'static str,
    chip: &'static str,
    core: Option<&'static str>,
    chip_core: Option<&'static str>,
}

impl Target {
    fn new(series: Series, target: String) -> Self {
        let (cpu, float_abi, chip_family, chip, core, chip_core) = match (series, target.as_str()) {
            (Series::Nrf52, "thumbv7em-none-eabihf") => {
                ("cortex-m4", "hard", "nrf52840", "NRF52840_XXAA", None, None)
            }
            (Series::Nrf52, "thumbv7em-none-eabi") => {
                ("cortex-m4", "soft", "nrf52840", "NRF52840_XXAA", None, None)
            }
            (Series::Nrf53, "thumbv8m.main-none-eabi") => (
                "cortex-m33+nodsp",
                "soft",
                "nrf5340",
                "NRF5340_XXAA",
                Some("NRF_NETWORK"),
                Some("NRF5340_XXAA_NETWORK"),
            ),
            (Series::Nrf54l, "thumbv8m.main-none-eabihf") => (
                "cortex-m33",
                "hard",
                "nrf54l15_cpuapp",
                "NRF54L15_XXAA",
                Some("NRF_APPLICATION"),
                None,
            ),
            (Series::Nrf54l, "thumbv8m.main-none-eabi") => (
                "cortex-m33",
                "soft",
                "nrf54l15_cpuapp",
                "NRF54L15_XXAA",
                Some("NRF_APPLICATION"),
                None,
            ),
            (Series::Nrf54lNs, "thumbv8m.main-none-eabihf") => (
                "cortex-m33",
                "hard",
                "nrf54l15_cpuapp_ns",
                "NRF54L15_XXAA",
                Some("NRF_APPLICATION"),
                None,
            ),
            (Series::Nrf54lNs, "thumbv8m.main-none-eabi") => (
                "cortex-m33",
                "soft",
                "nrf54l15_cpuapp_ns",
                "NRF54L15_XXAA",
                Some("NRF_APPLICATION"),
                None,
            ),
            (Series::Nrf54h, "thumbv8m.main-none-eabihf") => (
                "cortex-m33",
                "hard",
                "nrf54h20_cpurad",
                "NRF54H20_XXAA",
                Some("NRF_RADIOCORE"),
                None,
            ),
            (Series::Nrf54h, "thumbv8m.main-none-eabi") => (
                "cortex-m33",
                "soft",
                "nrf54h20_cpurad",
                "NRF54H20_XXAA",
                Some("NRF_RADIOCORE"),
                None,
            ),
            _ => panic!("Unsupported target: {:?}", target),
        };

        Self {
            target,
            cpu,
            float_abi,
            chip_family,
            chip,
            core,
            chip_core,
        }
    }
}

fn bindgen(target: &Target) -> bindgen::Builder {
    // NOTE:
    // This is a terrible hack for (seemingly)
    // `nrf_802154_swi.c` NOT including `nrf_802154_peripherals.h` directly or indirectly
    // while it should, as it references `NRF_802154_EGU_INSTANCE` which is defined by that header.
    let nrf_802154_egu_instance_workaround = format!(
        "-DNRF_802154_EGU_INSTANCE={}",
        match Series::get() {
            Series::Nrf52 | Series::Nrf53 => "NRF_EGU0",
            Series::Nrf54l | Series::Nrf54lNs => "NRF_EGU10",
            Series::Nrf54h => "NRF_EGU020",
        }
    );

    bindgen::Builder::default()
        .use_core()
        .size_t_is_usize(true)
        .clang_arg(format!("--target={}", target.target))
        .clang_arg(format!("-mcpu={}", target.cpu))
        .clang_arg(format!("-mfloat-abi={}", target.float_abi))
        .clang_arg("-mthumb")
        .clang_arg("-fshort-enums")
        .clang_arg("-I./third_party/arm/CMSIS_5/CMSIS/Core/Include")
        .clang_arg("-I./include")
        .clang_arg("-I./third_party/nordic/nrfx")
        .clang_arg("-I./third_party/nordic/nrfx/mdk")
        .clang_arg("-I./third_party/nordic/nrfxlib/nrf_802154/common/include")
        .clang_arg("-I./third_party/nordic/nrfxlib/nrf_802154/driver/include")
        .clang_arg("-I./third_party/nordic/nrfxlib/nrf_802154/sl/include")
        .clang_arg("-I./third_party/nordic/nrfx/templates")
        .clang_arg(format!("-D{}", target.chip))
        .clang_arg("-DCONFIG_MPSL")
        .clang_arg("-DNRF_802154_INTERNAL_SWI_IRQ_HANDLING=0")
        .clang_arg("-DNRF_802154_REQUEST_IMPL=0")
        .clang_arg("-DNRF_802154_NOTIFICATION_IMPL=0")
        .clang_arg(&nrf_802154_egu_instance_workaround)
        .clang_args(target.core.map(|x| format!("-D{}", x)))
        .clang_args(target.chip_core.map(|x| format!("-D{}", x)))
        .prepend_enum_name(false)
}

fn build(target: &Target) {
    let manifest_path = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());

    let include_paths = [
        "third_party/arm/CMSIS_5/CMSIS/Core/Include",
        "include",
        "third_party/nordic/nrfx",
        "third_party/nordic/nrfx/mdk",
        "third_party/nordic/nrfxlib/mpsl/include",
        "third_party/nordic/nrfxlib/mpsl/fem/include",
        "third_party/nordic/nrfxlib/nrf_802154/sl/sl/include",
        "third_party/nordic/nrfx/templates",
    ]
    .iter()
    .map(|path| manifest_path.join(path));

    let include_paths_args = include_paths
        .map(|path| format!("-I{}", path.to_str().unwrap()))
        .collect::<Vec<_>>()
        .join(" ");

    // NOTE:
    // This is a terrible hack for (seemingly)
    // `nrf_802154_swi.c` NOT including `nrf_802154_peripherals.h` directly or indirectly
    // while it should, as it references `NRF_802154_EGU_INSTANCE` which is defined by that header.
    let nrf_802154_egu_instance_workaround = format!(
        "-DNRF_802154_EGU_INSTANCE={}",
        match Series::get() {
            Series::Nrf52 | Series::Nrf53 => "NRF_EGU0",
            Series::Nrf54l | Series::Nrf54lNs => "NRF_EGU10",
            Series::Nrf54h => "NRF_EGU020",
        }
    );

    let nrf_target_args = target
        .core
        .map(|x| format!("-D{}", x))
        .into_iter()
        .chain(std::iter::once(format!("-D{}", target.chip)))
        .chain(target.chip_core.map(|x| format!("-D{}", x)))
        .chain(std::iter::once(format!("-mcpu={}", target.cpu)))
        .chain(std::iter::once(format!("-mfloat-abi={}", target.float_abi)))
        .chain(std::iter::once("-mthumb".to_string()))
        .collect::<Vec<_>>()
        .join(" ");

    if let Err(err) = patch_cmake_config(&manifest_path) {
        println!("cargo:warning=Could not patch CMake configuration: {err}");
    }

    cmake::Config::new("third_party/nordic/nrfxlib/nrf_802154")
        .define("CMAKE_BUILD_TYPE", "MinSizeRel")
        .define(
            "CMAKE_C_FLAGS",
            format!(
                "-Werror=implicit-function-declaration -fshort-enums -DCONFIG_MPSL -DNRF_802154_INTERNAL_SWI_IRQ_HANDLING=0 -DNRF_802154_REQUEST_IMPL=0 -DNRF_802154_NOTIFICATION_IMPL=0 {} {} {}",
                nrf_802154_egu_instance_workaround, nrf_target_args, include_paths_args
            ),
        )
        .define("CMAKE_C_FLAGS_RELEASE", "-O3 -DNDEBUG")
        .define("CMAKE_SYSTEM_NAME", "Generic")
        .define("CMAKE_SYSTEM_PROCESSOR", "ARM")
        .define("CMAKE_TRY_COMPILE_TARGET_TYPE", "STATIC_LIBRARY")
        .define("CMAKE_C_COMPILER", "clang")
        .define("CMAKE_C_COMPILER_TARGET", &target.target)
        .define("CMAKE_CXX_COMPILER", "clang")
        .define("CMAKE_CXX_COMPILER_TARGET", &target.target)
        .define("BUILD_SHARED_LIBS", "OFF")
        .define("BUILD_TESTING", "OFF")
        .build_target("nrf-802154-driver")
        .build();
}

/// Adds `cmake_minimum_required` to the upstream `CMakeLists.txt`.
///
/// This is a required property starting from CMake 4.0. See
/// [CMP0000](https://cmake.org/cmake/help/latest/policy/CMP0000.html) for more details.
fn patch_cmake_config(manifest_path: &PathBuf) -> Result<(), String> {
    let cmake_major_version = get_cmake_major_version().unwrap_or(0);
    if cmake_major_version < 4 {
        return Ok(());
    }

    let cmake_lists_path =
        manifest_path.join("third_party/nordic/nrfxlib/nrf_802154/CMakeLists.txt");
    let content = std::fs::read_to_string(&cmake_lists_path)
        .map_err(|err| format!("failed to read {}: {err}", cmake_lists_path.display()))?;
    if content.contains("cmake_minimum_required") {
        return Ok(());
    }

    let patched = format!("cmake_minimum_required(VERSION 3.5)\n{content}");

    let mut file = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(&cmake_lists_path)
        .map_err(|err| {
            format!(
                "failed to open {} for patching: {err}",
                cmake_lists_path.display()
            )
        })?;
    file.write_all(patched.as_bytes())
        .map_err(|err| format!("failed to write {}: {err}", cmake_lists_path.display()))?;

    Ok(())
}

fn get_cmake_major_version() -> Option<u32> {
    let output = std::process::Command::new("cmake")
        .arg("--version")
        .output()
        .ok()?;
    String::from_utf8_lossy(&output.stdout)
        .lines()
        .next()
        .and_then(|line| line.strip_prefix("cmake version "))
        .and_then(|version| version.split('.').next())
        .and_then(|major| major.parse().ok())
}

fn main() {
    let target = Target::new(Series::get(), env::var("TARGET").unwrap());

    build(&target);

    let out_path = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    let bindings = bindgen(&target)
        .header("./third_party/nordic/nrfxlib/nrf_802154/common/include/nrf_802154.h")
        .generate()
        .expect("Unable to generate bindings");

    let file = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(out_path.join("bindings.rs"))
        .unwrap();
    bindings
        .write(Box::new(&file))
        .expect("Couldn't write bindgen output");

    let lib_sl_path = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap())
        .join("third_party/nordic/nrfxlib/nrf_802154/sl/sl/lib")
        .join(target.chip_family)
        .join(format!("{}-float", target.float_abi));

    let lib_out_path = out_path.join("build");
    let lib_driver_path = lib_out_path.join("driver");
    let lib_common_path = lib_out_path.join("common");

    println!("cargo:rustc-link-search={}", lib_sl_path.to_str().unwrap());
    println!(
        "cargo:rustc-link-search={}",
        lib_driver_path.to_str().unwrap()
    );
    println!(
        "cargo:rustc-link-search={}",
        lib_common_path.to_str().unwrap()
    );

    println!("cargo:rustc-link-lib=static=nrf-802154-driver");
    println!("cargo:rustc-link-lib=static=nrf-802154-common");
    println!("cargo:rustc-link-lib=static=nrf-802154-sl");
}
