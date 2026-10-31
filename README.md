# FMUv6X: standalone Rust USB CDC and Ethernet LED firmware

A bare-metal Rust application for the **STM32H753 main processor** on the
Pixhawk 6X / FMUv6X, using `no_std`, `cortex-m-rt`, `embassy-stm32`, and
`embassy-usb`. The existing PX4 USB bootloader starts the application.

- **PWR:** steady on; the Ethernet PHY stays powered.
- **LINK:** 500 ms on, 500 ms off (1 Hz).
- **USB CDC ACM:** MAVLink 2 telemetry, PHY status, and software reboot to the bootloader.

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
| Data/stack memory | AXI SRAM, `0x24000000`, 512 KiB |
| Core clock | 64 MHz from the internal HSI oscillator |
| USB clock | 48 MHz from PLL3Q: 16 MHz HSE / 4 × 48 / 4 |
| USB data pins | PA11 (DM), PA12 (DP), OTG FS |
| USB VID/PID | `1209:0001` |
| Ethernet PHY | LAN8742A, MDIO address 0; measured IDs `0007:c131` |
| MDIO / MDC | PA2 open-drain with pull-up / PC1 |
| PHY power / PWR LED | PG15, active-high, held on |

For this **Ethernet-cable-disconnected blink demo**, LINK switches between
deasserted nINT (high, LED on) and the inactive link/activity indication (low,
LED off). WUCSR (MMD 3, address `0x8010`) selects the signal, with readback
verification. SPD uses its native PHY speed indication. PWR stays on.

An Embassy ticker and monotonic time set the blink rates. MDIO frames release
the bus for turnaround and sample the PHY data after MDC rises; the first read
clock consumes the driven turnaround bit, followed by exactly 16 data bits.

## Build

Install [Rust with rustup](https://rustup.rs/). The project toolchain file
selects the embedded target and LLVM tools automatically.

```sh
cargo build --release --locked
```

The ELF image is:

```text
target/thumbv7em-none-eabihf/release/v6x-rust-led
```

Generate and validate the USB-upload image:

```sh
cargo install cargo-binutils --locked
cargo objcopy --release --locked -- -O binary target/thumbv7em-none-eabihf/release/v6x-rust-led.bin
python3 tools/package.py
```

The resulting file is:

```text
target/thumbv7em-none-eabihf/release/v6x-rust-led.px4
```

The `.px4` container is the USB bootloader's upload format. Its payload is the
standalone Rust binary. The packager checks the stack/reset vectors and image
size, then pads to STM32H7's 32-byte flash programming boundary.

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
MCU. The existing bootloader stays available for an upload after this software
request. Its normal power-up window remains **5 seconds**. Run the uploader
command above once the bootloader port appears. It checks board ID 53 and
verifies the programmed image with the bootloader's CRC.

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
| `SYS_STATUS` | 1 Hz; unavailable battery values use the MAVLink unknown values |
| `SYSTEM_TIME` | 1 Hz; the actual MCU boot time in milliseconds |
| `STATUSTEXT` | At connection, every 5 heartbeats, and on request; identifies Rust firmware |
| `AUTOPILOT_VERSION` | At connection and on request; Rust version and STM32 UID |
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
Rust FMUv6X v0.1.0 running; USB MAVLink OK
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
python3 tools/check_mavlink.py
```

It checks heartbeat timing, firmware identity, parameter discovery, fragmented
v1/v2 requests, CRC rejection, partial-frame expiry, and advancing boot time.

To print live decoded messages for ten seconds:

```sh
python3 -u tools/check_mavlink.py --monitor 10
```

## USB diagnostic console

Open the Rust CDC port with a serial terminal or `pyserial`:

- Full IN packets are followed by a zero-length packet to complete host reads.
- `!status` followed by a newline returns USB power, PHY IDs, BMCR/BMSR,
  WUCSR, and LINK state. The response ends with `END`.
- `!bootloader` followed by a newline acknowledges the request and reboots
  into the existing PX4 bootloader.

Commands support LF or CRLF and can span multiple USB packets. Prefix a
command with a newline. Console replies share the serial stream with MAVLink
telemetry; ordinary binary data is processed as MAVLink rather than echoed.

## DroneCAN firmware updates

DroneCAN supports firmware transfer through `uavcan.protocol.file.BeginFirmwareUpdate`
and `uavcan.protocol.file.Read`. The target needs a DroneCAN-capable bootloader
that receives the image, programs flash, and starts the application.

The standard FMUv6X bootloader is configured for USB/UART. A PX4-independent
DroneCAN update path would require a separate **Rust DroneCAN bootloader**:

1. Install the Rust bootloader through STM32 ROM DFU or SWD initially.
2. Connect a USB-to-CAN adapter or gateway to the board's CAN bus.
3. Use DroneCAN firmware-update services for subsequent application updates.

The current application partition at `0x08020000` already reserves the first
128 KiB for a bootloader. A future Rust CAN bootloader would need to agree on
that application address and the firmware-image format.

Confirm the adapter/gateway model and whether a CAN-capable bootloader is
already installed before implementing this update path. The current milestone
contains the LED driver and USB-bootloader-compatible Rust application.

Protocol references:

- [BeginFirmwareUpdate](https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/file/40.BeginFirmwareUpdate.uavcan)
- [File Read](https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/file/48.Read.uavcan)

## Verification status

The release build, `cargo fmt --check`, and
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
handoffs remains unresolved. Upstream QGroundControl and the Windows COM26
direct-serial path have not been independently validated.

## Project layout

- `src/main.rs`: Embassy startup, clock/GPIO configuration, USB and blink tasks.
- `src/eth_led.rs`: LAN8742A MDIO, MMD LED function control, and diagnostics.
- `src/mavlink_usb.rs`: MAVLink transport, scheduling, requests, USB session
  reset handling, and packet termination.
- `src/usb_console.rs`: diagnostic commands and RTC-signature bootloader reboot.
- `memory.x`: application flash partition and SRAM layout.
- `build.rs`: makes the memory layout available to the Rust linker.
- `.cargo/config.toml`: Cortex-M7 target and `cortex-m-rt` linker script.
- `tools/package.py`: host-side USB image validation and packaging.
- `tools/check_mavlink.py`: independent live MAVLink verification.

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
