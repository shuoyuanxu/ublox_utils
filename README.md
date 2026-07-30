# ublox_utils

RTK GNSS rover. A ZED-F9P on the robot receives corrections from a standalone
ZED-F9P base over a 433 MHz radio link — no internet, no NTRIP, no computer at
the base.

Measured: **RTK FIXED at 1.6 cm horizontal**, base running headless on a power bank.

## Fork

Forked from [ethz-asl/ublox_utils](https://github.com/ethz-asl/ublox_utils)
(MIT, ETH Zürich ASL). Upstream is built for a different topology — two
receivers on the same vehicle in a wired moving-baseline pair, corrections from
an NTRIP caster over the internet. None of that applies here, so the upstream
launch files, u-center configs and the `ublox2nmea` node were removed.

What remains is ours: `launch/rover.launch`, `config/rover.yaml`,
`scripts/rtk_status.py` and `tools/`, written for the 433 MHz carrier boards.

## Hardware

| Role | Serial | Device |
|---|---|---|
| Rover | `D30G6YBQ` | `/dev/ublox_rover` |
| Base | `D30G6YBU` | `/dev/ublox_base` |

- **Bridge CB1 (HM-TRP) on both boards, leave CB2 open.** Both boards must use
  the same radio module with a solid red Led as a indicator.

## Build:

```bash
catkin_make
source devel/setup.bash
```

## Usage

```bash
roslaunch ublox_utils rover.launch
```

## License

MIT, see `LICENSE`.

---

# Appendix

Reference only — not needed to run the thing.

## A. Receiver settings

Written to **flash** on both boards, so they survive power cycles. Nothing
re-applies them at runtime.

| | Base (`D30G6YBU`) | Rover (`D30G6YBQ`) |
|---|---|---|
| UART2 baud | 38400 | 38400 |
| UART2 direction | RTCM3 **out** | RTCM3 **in**, all outputs off |
| UART1 baud | 38400 | 460800 |
| TMODE | survey-in, 60 s | rover |
| Dynamic model | stationary | automotive |
| Solution rate | 1 Hz | 5 Hz |
| RTCM out | 1005, 1074, 1084, 1094, 1124 @1 Hz; 1230 @0.2 Hz | — |

Three choices that are not obvious:

**UART2 at 38400**, not the 460800 in upstream's u-center configs. That value is
for a direct wire between two receivers; this link goes through the radio, and
38400 is what passes bytes cleanly.

**The rover's UART2 outputs are all disabled.** The radios are half-duplex, so a
rover that transmits collides with the base's corrections.

**Survey-in is limited by time only.** The F9P ends a survey when *both* the
minimum duration and the accuracy limit are satisfied, and the accuracy limit
cannot be switched off — so it is set to 100 m where it can never block. The 60 s
duration becomes the only constraint and the survey always completes, in any sky
view. The base's absolute position error then becomes a constant offset shared by
every rover fix, which cancels in relative measurement. For genuine absolute
accuracy, raise the duration and tighten the limit.

Check what is actually stored:

```bash
python3 tools/verify_flash.py -d /dev/ublox_base  --role base
python3 tools/verify_flash.py -d /dev/ublox_rover --role rover
```

## B. Changes from upstream

### Added

| Path | What |
|---|---|
| `launch/rover.launch` | rover launch |
| `config/rover.yaml` | driver parameters |
| `scripts/rtk_status.py` | RTK health monitor node; publishes `rtk_state` / `rtk_ok` |
| `tools/` | standalone diagnostics, no ROS dependency |
| `tools/99-ublox.rules` | udev rules for `/dev/ublox_base` and `/dev/ublox_rover` |
| `docs/board-433mhz-carrier.jpeg` | board photo: CB1/CB2/CB3, SW1, CN2, CN6/CN7, SMAs |

### Removed

`launch/ublox.launch`, `config/zed_f9p.yaml`, `config/moving_base.txt`,
`config/rover.txt`, `src/ublox2nmea.cc`, `resources/` — all served upstream's
moving-baseline and NTRIP paths, neither of which is used here.

### Changed

`CMakeLists.txt` — the package has no C++ left; it now only installs the Python
node plus `launch/` and `config/`.

`package.xml` — dependencies cut to `rospy`, `std_msgs`, `ublox_gps`,
`ublox_msgs`. Dropped `roscpp`, `nmea_msgs`, `mavros_msgs`, `rtcm_msgs` and
`ntrip_client`, which existed only for the removed code.

`config/rover.yaml` sets **`config_on_startup: false`**. The receiver's settings
live in its flash; if the driver were allowed to configure on startup it would
overwrite them and UART2 would stop accepting corrections. This presents later as
"RTK just stopped working" with no obvious cause.

## C. Tools

Plain Python 3 and pyserial. They need exclusive access to the serial port, so
stop `ublox_gps` first. Baud is autodetected; `-b` forces it.

| Tool | Purpose |
|---|---|
| `field_monitor.py` | live view of the whole correction chain — the field one |
| `gnss_diag.py` | firmware, ports, link use, satellites, AGC, fix, survey-in |
| `verify_flash.py` | confirm the config survives a power cycle |
| `check_rtk.py` | one-line RTK status |
| `stability.py` | dropouts, satellite churn, position scatter |
| `configure_f9p.py` | flash a board into a role (`--dry-run` first) |
| `hmtrp_config.py` | query or factory-reset the HM-TRP radio |
| `ubx.py` | shared UBX/NMEA/RTCM3 library, no CLI |

## D. Topics

Published under `node_name`, below as `<ns>` (default `ublox_rover`).

| Topic | Type |
|---|---|
| `/<ns>/fix` | `sensor_msgs/NavSatFix` — only once the receiver has a fix |
| `/<ns>/fix_velocity` | `geometry_msgs/TwistWithCovarianceStamped` |
| `/<ns>/navpvt` | `ublox_msgs/NavPVT` |
| `/<ns>/navstatus` | `ublox_msgs/NavSTATUS` |
| `/<ns>/rxmrtcm` | `ublox_msgs/RxmRTCM` — proof corrections are arriving |
| `/<ns>/rtk_state` | `std_msgs/String` — `none` / `FLOAT` / `FIXED`, latched |
| `/<ns>/rtk_ok` | `std_msgs/Bool` — latched |

RTK state is bits 6–7 of `NavPVT.flags`:

```python
carr = (msg.flags >> 6) & 0x03      # 0 none, 1 FLOAT, 2 FIXED
```

`fixType == 3` means a 3D fix, **not** RTK — a rover with no corrections still
reports 3.

Launch arguments: `node_name`, `device`, `frame_id`, `config`, `rtk_status`,
`respawn`.

## E. Gotchas

- Never run `roslaunch` under `sudo` — it strips `LD_LIBRARY_PATH` and the driver
  respawn-loops on a missing `libublox_msgs.so`, looking exactly like a hardware
  fault.
- `MON-MSGPP`'s RTCM3 column reads zero even while valid RTCM is arriving. Use
  `UBX-RXM-RTCM` (class `0x02`, id `0x32`); its `crcFailed` flag is authoritative.
- `CFG-TMODE-FIXED_POS_ACC` is `0x4003000F`. `0x40030014` does not exist and NAKs
  the whole VALSET batch, silently leaving the base in survey-in.
- AGC is counter-intuitive: an absent or disconnected band pegs **high** (~6669);
  RF overload reads **low** (~1400); healthy is ~4200–4600.
- `numSV` counts satellites *used*, not tracked.
- `rx = 0` with zero framing errors means the line is idle. A baud mismatch or
  bad wiring produces errors, not silence.
- Never hold two file descriptors on one tty — termios is per-device, not per-fd,
  so a second `open()` at a different baud silently reconfigures the first.
- A factory-reset F9P emits NMEA only, at 38400.
