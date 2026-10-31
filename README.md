# FMUv6X: Rust/Embassy platform bring-up

A bare-metal Rust application for the **STM32H753 main processor** on the
Pixhawk 6X / FMUv6X, using `no_std`, `cortex-m-rt`, `embassy-stm32`, and
`embassy-usb`. The existing PX4 USB bootloader starts the application.

- **PWR:** steady on; the Ethernet PHY stays powered.
- **LINK:** 500 ms on, 500 ms off (1 Hz).
- **USB CDC ACM:** MAVLink 2 telemetry, PHY status, and software reboot to the bootloader.

Development now follows the selected **A3 hybrid** approach. The first integration
goal is IMU/AHRS, DroneCAN, ULog, USB/TELEM1, and upstream QGroundControl message
inspection and parameter read/set. See the [M1 verification plan](docs/m1-verification.md)
and [P0 evidence](docs/acceptance/p0/README.md). These capabilities are planned;
version **0.2.0** adds P1 platform/timing/fault diagnostics. **P1 is complete:**
software, on-board timing, restart/fault recovery, and the independent QGC round
trip passed. The final normal Rust restore completed at 21:28 UTC on 2026-10-31,
with source **`a946a529360eac83`**, disarmed, fault injection disabled, and no valid
retained fault. See the [P1 evidence](docs/acceptance/p1/README.md) and
[essential-check plan](plan.md). Next is P3a: FRAM identification/backup and parameters.
The known-good 0.1.0 USB/LED recovery package remains in `.p0-baseline/20261031/`.
See the [P1 operation sheet](docs/acceptance/p1/bring-up.md) before any new upload.

## Hardware

The connected board reported `PX4_FMU_V6X`, FMUM ID `0x003` (revision 3), and
STM32H7 silicon revision V during identification.

| Item | Configuration |
| --- | --- |
| MCU | STM32H753II revision V, Arm Cortex-M7 |
| Rust target | `thumbv7em-none-eabihf` |
| USB bootloader | First 128 KiB of flash |
| Application/reset vector | `0x08020000` |
| Internal flash | 2 MiB |
| Application flash partition | 1920 KiB |
| USB bootloader board ID | 53 |
| Data/stack memory | AXI SRAM: 447 KiB static allocation, 1 KiB fault stack, 64 KiB shared stack |
| Core clock | P1: HSE/PLL1 480 MHz; HCLK 240 MHz; PCLK 120 MHz; register and TIM2 timebase checks passed |
| Monotonic timer | TIM2, 1 MHz; HAL uses a 16-bit counter with half-overflow interrupts |
| USB clock | 48 MHz from PLL3Q: 16 MHz HSE / 4 × 48 / 4 |
| USB data pins | PA11 (DM), PA12 (DP), OTG FS |
| USB VID/PID | `1209:0001` |
| Ethernet PHY | LAN8742A, MDIO address 0; measured IDs `0007:c131` |
| MDIO / MDC | PA2 open-drain with pull-up / PC1 |
| PHY power / PWR LED | PG15, active-high, held on |
| Status LEDs | PE3/PE4/PE5 open-drain: red retained fault, green 1 Hz, blue during startup |

The earlier hardware-verified 0.1.0 baseline used HSI at 64 MHz. P1 configures
LDO/Scale0 (Scale1 + ODEN), flash LATENCY=4/WRHIGHFREQ=2, I-cache on and D-cache
off. Register configuration is not proof of physical clock frequency.

For this **Ethernet-cable-disconnected blink demo**, LINK switches between
deasserted nINT (high, LED on) and the inactive link/activity indication (low,
LED off). WUCSR (MMD 3, address `0x8010`) selects the signal, with readback
verification. SPD uses its native PHY speed indication. PWR stays on.

An Embassy ticker and monotonic time set the blink rates. MDIO frames release
the bus for turnaround and sample the PHY data after MDC rises; the first read
clock consumes the driven turnaround bit, followed by exactly 16 data bits.

## Build

### RustRover and terminal workflow

Open this directory in RustRover. The shared run configurations in `.run/`
appear in the toolbar's run selector; select one and press **Shift+F10**:

| Configuration | Action |
| --- | --- |
| **Build firmware** | Compile the release image and create the validated `.px4` package |
| **Upload firmware (USB)** | Build, reboot the connected Rust application into its PX4 bootloader, upload, verify CRC, and restart |
| **Monitor MAVLink** | Decode telemetry for 10 seconds |
| **Check firmware** | Formatting, normal/injection/Dump Clippy, host tests, normal/Dump build and package/layout/hash reports |

On this machine, launch the configured IDE with `rustrover ~/rust_ap` or the
**RustRover** application-menu entry. Its launcher loads the Rust environment.
The project uses the `thumbv7em-none-eabihf` target; `rust-src` provides standard
library sources for code navigation. Complete JetBrains activation on first
launch, choosing the non-commercial license if applicable.

The same actions work from any terminal:

```sh
./tools/firmware.sh build
./tools/firmware.sh upload
./tools/firmware.sh monitor 10
./tools/firmware.sh check
```

`./tools/firmware.sh upload --dry-run` checks USB access, the uploader, and the
built package without rebooting or uploading. Stop other serial readers before
uploading or monitoring. Upload handles both the Rust CDC and PX4 bootloader
USB identities, including a changed tty number after reboot.

For setup on another Linux machine, install Rust with rustup, then run:

```sh
source "$HOME/.cargo/env"
rustup show
python3 -m venv .venv
.venv/bin/python -m pip install -r tools/requirements.txt
```

The upload action defaults to `~/PX4-Autopilot/Tools/px4_uploader.py`. Set
`PX4_UPLOADER` to another copy of the uploader if needed. `PX4_PORTS` can
override the comma-separated serial-port patterns. Defaults recognize the Rust
CDC, the official Auterion PX4 FMUv6X application, and the PX4 bootloader. Custom
patterns must include the application and bootloader identities when starting
from an application, so discovery continues after its reboot.

The post-PX4-restore `found []` preflight regression is fixed and covered by
six host tests; the current Python suite has 64 tests. See the
[upload discovery record](docs/acceptance/p1/20261031-upload-discovery.md).
Ordinary `upload` rebuilds and flashes the current P1; `upload --dry-run` checks
prerequisites and the package without rebooting or flashing. The corrected P1
source ID is `a946a529360eac83`, including the backup-domain fix. Its on-device
source, protocol, platform, timing, and recovery checks passed; exact results are
in the [HIL record](docs/acceptance/p1/20261031-hil.md).

The provided udev rules grant the active local desktop user access to both
USB identities. Install once from an interactive terminal:

```sh
sudo install -m 0644 tools/60-fmuv6x.rules /etc/udev/rules.d/60-fmuv6x.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=change --subsystem-match=tty
```

### Manual build commands

Install [Rust with rustup](https://rustup.rs/). The project toolchain file
selects the embedded target and LLVM tools automatically.

```sh
cargo build --release --locked
```

The ELF image is:

```text
target/thumbv7em-none-eabihf/release/v6x-rust-led
```

Generate the USB-upload image, map, resource/hash report and disassembly:

```sh
./tools/firmware.sh build
```

The resulting file is:

```text
target/thumbv7em-none-eabihf/release/v6x-rust-led.px4
```

The `.px4` container is the USB bootloader's upload format. Its payload is the
standalone Rust binary. The packager checks the stack/reset vectors and image
size, then pads to STM32H7's 32-byte flash programming boundary.
`tools/build_report.py` also checks all nonzero vectors, ELF/bin agreement,
the emergency/shared-stack separation and the embedded source ID.
`v6x-rust-led.build.json` and `SHA256SUMS` identify the exact artifacts.
`./tools/firmware.sh build-injection` creates a **separate** test-only package
under `target/p1-fault-injection/`; the upload action always builds the normal version.

`./tools/firmware.sh build-dump` creates the P1 startup-regression diagnostic
image under `target/p1-fault-dump/`: it captures `BKP0`/`BKP16..31`/`RCC_RSR`
before any HAL init, boots the known-good P0 clock tree and prints the report
over USB CDC. The shared startup wrapper preserves inherited RTC/LSE settings
and compares all 32 BKP words after HAL initialization, without restoring lost
words. Schema-2 reports include before/after BDCR/CSR and preservation status.
Dump never writes the fault record and accepts `!bootloader`.
`./tools/firmware.sh upload-dump` is a separate engineer-run diagnostic action.
See the [current backup-domain behavior](docs/acceptance/p1/20261031-backup-domain.md)
and [historical Dump image record](docs/acceptance/p1/20261031-fault-dump-image.md).

When the board is running **V6X Rust Dump**, first send `!bootloader` to the
**Dump** CDC port and confirm `3185:0035 / PX4 BL FMU v6X.x` with `firmware.sh ports`.
Then use ordinary `upload` to install the corrected P1. The default uploader does
not match the Dump identity. Exact engineer-run commands and retest criteria are in
the [startup fix handoff](docs/acceptance/p1/20261031-startup-assertion.md#5-工程师复验从当前-dump-返回普通-p1).

## Flash over normal USB

Use the standalone
[USB uploader](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/px4_uploader.py)
with Python and `pyserial` installed on the computer:

```sh
python3 /path/to/px4_uploader.py --port '/dev/serial/by-id/*PX4_BL*' --verbose --noninteractive-json target/thumbv7em-none-eabihf/release/v6x-rust-led.px4
```

For the first upload, enter the bootloader from the original PX4 firmware using
its NSH shell:

```text
reboot -b
```

If the PX4 USB port is streaming MAVLink, send
`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` with parameter 1 set to `3`. This path was
verified on the connected board. The bootloader enumerates as `3185:0035`,
product `PX4 BL FMU v6X.x`; its tty number may change after the reboot.

For subsequent uploads from this Rust application, send the complete line
`!bootloader` to the Rust CDC port:

```sh
python3 - <<'PY'
import serial

port = '/dev/serial/by-id/usb-Rust_FMUv6X_V6X_Rust_CDC_V6X-RUST-001-if00'
with serial.Serial(port, 115200, timeout=2, write_timeout=2) as device:
    device.write(b'\n!bootloader\n')
    device.flush()
    print(device.read_until(b'OK bootloader\n').decode(errors='replace'))
PY
```

The application writes `0xb007b007` to RTC backup register 0 and resets the
MCU. The reviewed PX4 bootloader source handles this request by staying in the
bootloader. Run the uploader once its port appears; it checks board ID 53 and
verifies the programmed image with the bootloader's CRC.

The source's 5000 ms constant does not guarantee a fixed upload window for every
power-up: VBUS, reset conditions, enumeration, and the installed bootloader matter.
Before flashing new functionality, complete the [QGC recovery procedure](docs/recovery.md),
including an entry method that does not depend on the Rust application responding.

### VMware USB connection

Connect both identities to the Linux VM: `3185:0035` for the PX4 bootloader and
`1209:0001` for the Rust application. If the device disappears from guest Linux
after a firmware reboot, reconnect the newly enumerated device through
VMware's removable-device menu.

`lsusb` may label `1209:0001` as **Generic pid.codes Test PID** from its local
USB ID database. The actual device product string is **V6X Rust CDC** and its
serial number is **V6X-RUST-001**.

## MAVLink ground-station connection

The CDC data stream carries MAVLink 2 with system ID **1**, component ID **1**,
and firmware type **Generic**:

| Message | Schedule / purpose |
| --- | --- |
| `HEARTBEAT` | 1 Hz; Generic vehicle, standby state |
| `SYS_STATUS` | 1 Hz; P1 measured CPU load in permille, unknown batteries unchanged; CPU validity is in `!metrics` |
| `SYSTEM_TIME` | 1 Hz; the actual MCU boot time in milliseconds |
| `STATUSTEXT` | At connection, every 5 heartbeats, and on request; identifies Rust firmware |
| `AUTOPILOT_VERSION` | At connection and on request; Cargo version, source ID (little-endian custom version), STM32 UID |
| `PARAM_VALUE` | Read-only `MAV_SYS_ID` and `LINK_LED_HZ` discovery/read replies |

Receive supports unsigned MAVLink v1/v2, fragmented frames, CRC validation,
`PING`, `TIMESYNC`, and basic `COMMAND_LONG` replies. Unsupported commands
return an explicit unsupported result. Reboot-to-bootloader is supported via
`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`, parameter 1 = `3`.

Periodic messages are generated from current state. Missed send periods are
skipped instead of replayed when a host opens the serial port later. USB reset
and reconfiguration notifications restart transport tasks and discard old
queued replies.

For a direct serial connection, add a **Serial** communication link, select the **V6X Rust
CDC** port (Linux `/dev/ttyACM*` or the host's COM port), set **115200 baud**,
and connect. The custom USB ID may require manual link configuration. Inspect
system 1 in the ground station's MAVLink inspector, or look for:

```text
Rust FMUv6X v0.2.0 running; USB MAVLink OK
```

For a ground station running on the VMware host, attach the Rust USB device to the host.
For a Linux terminal test, attach it to the guest instead.

### Tested VMware USB-to-UDP workflow

Keep the Rust USB device attached to Linux and let one MAVProxy instance own
the CDC port. Forward its MAVLink traffic to the Windows ground station over UDP:

```sh
mavproxy.py --master=/dev/serial/by-id/usb-Rust_FMUv6X_V6X_Rust_CDC_V6X-RUST-001-if00 \
  --baud=115200 --mav20 --out=udp:192.168.242.1:14550 \
  --default-modules=link --no-state
```

`192.168.242.1` is this setup's Windows VMnet8 address; replace it with the
host's VMnet8 IPv4 address on another machine. Enable the ground station's UDP
connection listening on port `14550`. Vehicle discovery and Rust status
reception were reported working through this path. The client was subsequently
identified as **q-MAG / MagGCS**, rather than upstream QGroundControl.

The traffic path is **Rust USB CDC → Linux MAVProxy → UDP → Windows ground station**.
The last hop uses UDP, while the board-to-Linux connection still uses USB.
This avoids transferring USB ownership between host and guest. A stable
`by-id` path also avoids depending on a ttyACM number that can change after
re-enumeration. Restart a client whose serial handle died during a transfer.

Use one serial reader at a time. Exit the interactive relay with `quit` before
running a separate serial checker or monitor.

### Known USB handoff instability

The q-MAG connection can fail on the next attachment after moving the physical
USB device between the Windows host and the Linux VM. CDC enumeration alone
does not demonstrate that the data connection has recovered. This switching
case remains unresolved.

The firmware restarts its MAVLink transport tasks and discards queued replies
on USB reset/reconfiguration, but this has not established stable direct
q-MAG connections across host/VM switches. Kernel logs showed ttyACM numbers
changing and existing client handles becoming invalid.

For direct q-MAG serial testing, stop every MAVProxy/terminal reader and select
the current Rust CDC attachment. The UDP forwarding workflow keeps USB with
one owner and avoids the handoff. The firmware truthfully reports a Generic
autopilot; q-MAG's automatic FC discovery policy currently targets PX4
multirotors, so automatic-platform support and manual telemetry reception are
separate integration concerns.

An independent live-wire check using `pyserial` and `pymavlink` is available:

```sh
.venv/bin/python tools/check_mavlink.py
```

It checks heartbeat timing, firmware identity, parameter discovery, fragmented
v1/v2 requests, CRC rejection, partial-frame expiry, and advancing boot time.

To monitor the connected Rust image for ten seconds (close other serial readers first):

```sh
bash tools/firmware.sh monitor 10
# Equivalent: .venv/bin/python -u tools/check_mavlink.py --monitor 10
```

The tool waits up to ten seconds for USB enumeration before starting the full
requested capture duration. Use `--wait 0` with the Python entry point for an
immediate scan. Every run prints its script path/hash and saves USB discovery
snapshots and the result to `target/monitor-logs/<UTC timestamp>.json`, including
discovery failures (honors `CARGO_TARGET_DIR`). To inspect identities without
opening a serial port:

```sh
bash tools/firmware.sh ports
```

The monitor selects the protocol from the USB identity:

- `V6X-RUST-001`: print decoded MAVLink messages.
- `V6X-RUST-DUMP`: capture diagnostic text and then print the decoded fault,
  CRC status, source location and clock registers. Exact captured bytes are saved
  to a new `target/fault-dumps/<UTC timestamp>.txt` file (honors `CARGO_TARGET_DIR`),
  including partial data on disconnect. Dump firmware has no MAVLink/QGC heartbeat.

Use `--port /dev/serial/by-id/...` to select among multiple devices, and
`--raw path/to/new-file.txt` to choose a Dump capture filename. Existing captures
are preserved. No device, a bootloader, an incomplete/empty capture or serial errors
produce a clear error and nonzero exit status. A complete Dump with `valid=NO`
means capture succeeded but no valid retained fault was found. The default command
without `--monitor` continues to require MAVLink firmware for the full wire checks.
See the [monitor fix](docs/acceptance/p1/20261031-monitor-dump.md) and
[re-enumeration diagnostics](docs/acceptance/p1/20261031-monitor-rediscovery.md).

## USB diagnostic console

Open the Rust CDC port with a serial terminal or `pyserial`:

- Full IN packets are followed by a zero-length packet to complete host reads.
- `!status` followed by a newline returns USB power, PHY IDs, BMCR/BMSR,
  WUCSR, and LINK state. The response ends with `END`.
- `!bootloader` followed by a newline acknowledges the request and reboots
  into the existing PX4 bootloader.
- `!platform`: build identity, configured clocks, raw RCC/PWR/core registers.
  Schema 2 adds `BACKUP`: inherited/post-init BDCR/CSR and RTC/BKP preservation.
- `!metrics`: cumulative 1 kHz release/body/IRQ statistics, last 1 s CPU window,
  shared-stack watermark; `!metrics-reset` asynchronously starts a new statistics epoch.
- `!fault`: CRC-validated retained record; `!fault-clear` explicitly clears it.
- `!panic` / `!hardfault`: **only** in `fault-injection` builds, never in the normal package.

Engineer-run captures (close QGC and all other readers first):

```sh
.venv/bin/python tools/p1_capture.py --seconds 65 --output target/p1-hil-idle
.venv/bin/python tools/p1_capture.py --seconds 65 --pressure-hz 100 --output target/p1-hil-usb
```

The tool preserves raw USB bytes and host timestamps, demultiplexes MAVLink,
and reports timebase uncertainty instead of treating serial latency as clock drift.
The capture gate requires schema-2 backup-domain evidence; legacy reports have
unknown preservation status and cannot pass that gate.
It does not upload, reboot, clear faults or inject failures. See [measurement
limits and manual acceptance](docs/acceptance/p1/bring-up.md).

Commands support LF or CRLF and can span multiple USB packets. Prefix a
command with a newline. Console replies share the serial stream with MAVLink
telemetry; ordinary binary data is processed as MAVLink rather than echoed.

## DroneCAN firmware updates

The project retains the original PX4 USB bootloader, its USB identity, and the
`0x08020000` application boundary so QGroundControl can restore official PX4.
The earlier idea of installing a replacement Rust CAN bootloader is superseded
by this recovery requirement.

DroneCAN in the first goal covers node services, ESC status/zero commands, and
H-RTK GNSS data. CAN firmware updates are outside that goal. See
[DroneCAN notes](docs/dronecan-notes.md) and [recovery](docs/recovery.md).

## Verification status

P1 passed the [current essential checks](plan.md#4-仅保留必要测试).
M1 hardware acceptance has not started.

**P1 software checks:** fmt, normal/injection/Dump target Clippy, host Clippy,
12 Rust tests, 64 Python tests, and normal/Dump/injection release builds and
package/layout checks pass. Each image passes 132 compiled backup-domain cases
(396 total), and the current normal image passes 14 clock-predicate cases.
Exact artifacts and logs are in the
[P1 evidence index](docs/acceptance/p1/README.md). Remote CI status has not been checked.

**P1 hardware:** source/protocol and platform/backup-domain checks, two 65-second
idle/USB-pressure captures, text/MAVLink reboots, and panic/HardFault retention
and clearing passed. QGC restored official PX4 v1.17.0 through an independent
USB entry; the final direct normal Rust restore verified CRC **`B9FB636E`**,
three disarmed heartbeats, `fault_injection=false`, and fault `valid=false`.
See the [HIL record and measurement limits](docs/acceptance/p1/20261031-hil.md).
QGC Inspector's missing AUTOPILOT_VERSION display is tracked in P2.

**Backup-domain fix (2026-10-31; on-board acceptance passed):** both HAL low-speed
configuration and the generated RTC clock mux now match inherited state. The LSE
drive mapping follows the selected H753 PAC variants, preserving raw bits.
The wrapper checks BDCR configuration and all 32 BKP words instead of writing
them back after a reset. Current normal source: **`a946a529360eac83`**, `.px4` SHA-256:
`7652fa90836be3182becf8f888a052aebc982ecbf67e6ba303dc66719db8fff5`.
The replay executes compiled startup/HAL code with synthetic MMIO; it does not
measure board behavior or RTC timekeeping. See the [fix and evidence](docs/acceptance/p1/20261031-backup-domain.md).

**Startup assertion fixed (2026-10-31; basic connectivity restored):** the user captured
6456 bytes from `V6X Rust Dump`, including a CRC-valid panic from normal source
`afaa140f499ff958` at `src/platform.rs:91`. The old check required SCUEN=1, while
the board reads `PWR_CR3=0x05000042` before and after the working P0 HAL init.
SCUEN controls supply updates; the fix removes that requirement while keeping
LDO/bypass and voltage-ready checks. With the physical CR3 and modeled P1 clocks,
the actual ELF predicate changes from rejection to acceptance; 12 invalid-state
cases remain rejected. This is not a full HAL/board simulation. The normal image
from that earlier fix had
source: **`4cba95aa54cd899a`**, `.px4` SHA-256:
`51fdd8a4189f519c014d3e78554f8eb422813656332a4f4b751626cebcb9739a`.
See the [raw evidence, fix and engineer retest steps](docs/acceptance/p1/20261031-startup-assertion.md).

The engineer subsequently confirmed QGC connection. Saved logs show the original
bootloader at 16:14 UTC and successful normal Rust CDC/MAVLink monitoring at
16:15 UTC. That historical monitor log does not record a firmware source ID;
the later HIL record binds the accepted image. See the [retest evidence](docs/acceptance/p1/20261031-qgc-retest.md).

The earlier 134208-byte upload remains unidentified by path/hash. The engineer
cleared the separate `/tmp` quota/serial-lock problem and reports restoring PX4;
the subsequent QGC round trip and final Rust restore are recorded in the HIL evidence. See the
[incident history](docs/acceptance/p1/20261031-startup-regression.md) and
[host lock investigation](docs/acceptance/p1/20261031-qgc-lock.md).

**Historical 0.1.0 baseline, not new P1 hardware evidence:** the release build, `cargo fmt --check`, and
`cargo clippy --release --locked -- -D warnings` pass. The generated ELF was
checked for its flash origin (`0x08020000`), initial stack pointer
(`0x24080000`), and Thumb reset vector. The USB container passes four packaging
tests covering image preservation, alignment, and invalid target vectors/sizes.

Hardware flashing and CRC verification pass. The independent MAVLink probe
measured a **1.000 s** heartbeat period and verified Rust status text, MCU UID,
version/parameter replies, v1/v2 fragmentation, CRC rejection and partial-frame
recovery. The PHY reads as `0007:c131`, and LINK blinking was visually confirmed.
Live telemetry was also verified after USB reattachment under a single reader.
Ground-station discovery and status reception were reported working through
the USB-to-UDP workflow. Direct q-MAG serial stability across host/VM USB
handoffs remains unresolved. The Windows COM26 direct-serial path has not been
independently validated.

On 2026-10-31, the user reported that the current version can restore PX4,
connect to QGroundControl, and show HEARTBEAT and other messages in MAVLink
Inspector. The user then confirmed that the current Rust firmware connects to
the locally installed QGC 5.1.5, and that recovery used USB reconnection/reset
without a Rust reboot command. Those early reports lacked exact image/log bindings;
the subsequent HIL evidence records the current round trip. See the
[user recovery report](docs/acceptance/p1/20261031-user-recovery-report.md)
and [local QGC installation notes](docs/qgc-notes.md).

## Project layout

- `src/main.rs`: measured Embassy thread executor, PHY/status LEDs and USB setup.
- `src/platform.rs`: 480/240/120 MHz clock/power/cache/IRQ setup and diagnostics.
- `src/backup_domain.rs`: shared retained RTC/LSE configuration and post-HAL checks.
- `src/diagnostics.rs`: 1 kHz substitute, CPU accounting and shared-stack monitoring.
- `src/fault.rs`: RTC record/recovery and HardFault emergency-stack entry.
- `src/fault_record.rs`, `src/metrics.rs`: no_std host-testable record/statistics logic.
- `src/eth_led.rs`: LAN8742A MDIO, MMD LED function control, and diagnostics.
- `src/mavlink_usb.rs`: MAVLink transport, scheduling, requests, USB session
  reset handling, and packet termination.
- `src/usb_console.rs`: bounded PHY/platform/metrics/fault diagnostic responses.
- `memory.x`: application flash partition and SRAM layout.
- `build.rs`: linker layout/map and source/feature identity.
- `.cargo/config.toml`: Cortex-M7 target and `cortex-m-rt` linker script.
- `tools/package.py`: host-side USB image validation and packaging.
- `tools/check_mavlink.py`: independent MAVLink verification and automatic normal/Dump monitoring.
- `tools/decode_fault_dump.py`: retained-fault CRC, source mapping and register decoding.
- `tools/replay_platform_check.py`: optional host Cortex-M7 replay of the compiled clock predicate.
- `tools/replay_backup_domain.py`: compiled startup/HAL replay with synthetic retained RTC/BKP states.
- `tools/backup_domain.py`: shared schema-2 preservation evidence decoder.
- `tools/firmware.sh`: shared build/upload/monitor/ports/check, separate injection build.
- `tools/build_report.py`: ELF/bin/vector/layout/source checks and SHA-256 manifest.
- `tools/p1_capture.py`: engineer-run USB capture and timing uncertainty analysis.
- `tools/test_*.py`: package/build-identity/capture regression checks, not HIL.
- `.github/workflows/p1.yml`: minimal CI; never uploads firmware.

## Adding drivers, one model at a time

Before adding each driver, confirm the exact sensor or peripheral **model** to
support. For that model, identify the bus (SPI, I2C, UART, or CAN), board wiring,
and required functionality.

Keep register definitions and device behavior in `src/drivers/<model>.rs`.
Configure the STM32 peripheral, chip-select, interrupt pins, and power controls
alongside the board setup in `src/main.rs`, then bring the selected driver up
on the board.

Use Embassy's asynchronous peripheral drivers and the compatible
`embedded-hal` / `embedded-hal-async` traits when integrating further devices.

## Hardware references

- [Pixhawk FMUv6X hardware standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-012%20Pixhawk%20Autopilot%20v6X%20Standard.pdf)
- [Holybro Pixhawk 6X documentation](https://docs.holybro.com/autopilot/pixhawk-6x)
