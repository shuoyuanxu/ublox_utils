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

Three things the boards will not tell you:

- **Bridge CB1 (HM-TRP) on both boards, leave CB2 open.** Both boards must use
  the same radio module. Bridging two lights **LD1 red** — a module conflict, not
  a data LED.
- **The rover needs a dual-band L1/L2 antenna.** L1-only tops out at FLOAT (~4 cm).
- **There are four SMA connectors.** The 433 antenna in a GNSS port gives a link
  that looks dead but works intermittently at very short range.

## Setup

```bash
sudo cp tools/99-ublox.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER      # log out and back in

catkin_make --pkg ublox_utils
source devel/setup.bash
```

> If the workspace sets `CATKIN_WHITELIST_PACKAGES`, this package is silently
> skipped. Clear it with `catkin_make -DCATKIN_WHITELIST_PACKAGES=""`.

## Usage

```bash
roslaunch ublox_utils rover.launch
```

Or from a robot launch file:

```xml
<include file="$(find ublox_utils)/launch/rover.launch">
  <arg name="node_name" value="gnss"/>
  <arg name="frame_id"  value="gnss_link"/>
  <arg name="output"    value="log"/>
</include>
```

### Topics

Published under `node_name`, below as `<ns>` (default `ublox_rover`).

| Topic | Type | Purpose |
|---|---|---|
| `/<ns>/fix` | `sensor_msgs/NavSatFix` | position; only published once the receiver has a fix |
| `/<ns>/fix_velocity` | `geometry_msgs/TwistWithCovarianceStamped` | NED velocity |
| `/<ns>/navpvt` | `ublox_msgs/NavPVT` | `fixType`, `flags`, `numSV`, `hAcc` |
| `/<ns>/navstatus` | `ublox_msgs/NavSTATUS` | fix and differential flags |
| `/<ns>/rxmrtcm` | `ublox_msgs/RxmRTCM` | each correction received — proof the radio link is alive |
| `/<ns>/rtk_state` | `std_msgs/String` | `none` / `FLOAT` / `FIXED`, latched |
| `/<ns>/rtk_ok` | `std_msgs/Bool` | true when FLOAT or FIXED, latched |

RTK state lives in bits 6–7 of `NavPVT.flags`:

```python
carr = (msg.flags >> 6) & 0x03      # 0 none, 1 FLOAT, 2 FIXED
```

`fixType == 3` means a 3D fix, **not** RTK — a rover with no corrections still
reports 3.

### Launch arguments

| Arg | Default | Meaning |
|---|---|---|
| `node_name` | `ublox_rover` | node name and topic namespace |
| `device` | `/dev/ublox_rover` | serial port |
| `frame_id` | `gnss` | TF frame on the NavSatFix |
| `config` | `config/rover.yaml` | driver parameters |
| `rtk_status` | `true` | run the RTK health monitor |
| `respawn` | `true` | restart the driver if it dies |

**Nothing in ROS supplies corrections.** They arrive over UART2 from the radio,
inside the receiver. There is no RTCM subscriber and no NTRIP client.

**`config_on_startup` is `false` deliberately.** The receiver's settings live in
its flash; letting the driver reconfigure on startup would overwrite them and
stop UART2 accepting corrections.

## Receiver configuration

In flash on both boards, survives power cycles.

| | Base | Rover |
|---|---|---|
| UART2 | 38400, RTCM3 **out** | 38400, RTCM3 **in**, all outputs off |
| UART1 | 38400 | 460800 |
| TMODE | survey-in, 60 s | rover |
| Dynamic model | stationary | automotive |
| Rate | 1 Hz | 5 Hz |
| RTCM out | 1005, 1074, 1084, 1094, 1124 @1 Hz, 1230 @0.2 Hz | — |

The rover's UART2 outputs are all off because the radios are half-duplex — a
transmitting rover collides with the base.

Survey-in is limited by **time only**. The F9P finishes a survey when both the
duration and the accuracy limit are met, and the accuracy limit cannot be
disabled, so it is set to 100 m where it can never block. The 60 s duration is
therefore the only constraint and the survey always completes, in any sky view.
The base's absolute position error becomes a constant offset shared by every
rover fix, which cancels in relative measurement. For absolute accuracy, raise
the duration and tighten the limit.

```bash
python3 tools/verify_flash.py -d /dev/ublox_base  --role base
python3 tools/verify_flash.py -d /dev/ublox_rover --role rover
```

## Field checklist

1. 433 antenna in the correct SMA on both boards; GNSS antennas attached.
2. Power the base with sky view. Survey completes in 60 s.
3. `python3 tools/field_monitor.py --seconds 30` — expect `radio` ≈ 570 B/s,
   `RTCM/s` ≈ 5, `crcErr` 0, `1005 yes`, `RTK FIXED`.
4. Launch the robot stack; confirm `/<ns>/rtk_state` is `FIXED`.

## Tools

Plain Python 3 and pyserial, no ROS. **They need exclusive access to the serial
port — stop `ublox_gps` first.** Baud is autodetected; `-b` forces it.

| Tool | Purpose |
|---|---|
| `field_monitor.py` | live view of the whole correction chain |
| `gnss_diag.py` | firmware, ports, link use, satellites, AGC, fix, survey-in |
| `verify_flash.py` | confirm the config survives a power cycle |
| `check_rtk.py` | one-line RTK status |
| `stability.py` | dropouts, satellite churn, position scatter |
| `configure_f9p.py` | flash a board into a role (`--dry-run` first) |
| `hmtrp_config.py` | query or factory-reset the HM-TRP radio |
| `ubx.py` | shared UBX/NMEA/RTCM3 library, no CLI |

## Troubleshooting

| Symptom | Cause |
|---|---|
| `radio 0 B/s` | Wrong SMA port, base unpowered, or CB bridges differ between boards |
| Bytes arriving, `RTCM/s` 0 | UART2 baud differs between base and rover |
| RTCM arriving, no `1005` | Base survey has not completed; RTK cannot engage without it |
| `1005` seen, RTK `none` | Too few satellites — needs sky view |
| FLOAT but never FIXED | No L2 — check the antenna is dual-band |
| LD1 red | Two radio modules bridged at once |
| 0 satellites, `hAcc 4294967.295` | `0xFFFFFFFF` = invalid. No sky view, or antenna disconnected |

**Diagnosing L2** with `gnss_diag.py`:

```
L2  AGC 4212/8191   healthy, receiving L2
L2  AGC 6669/8191   pegged: nothing on L2, antenna is L1-only
```

AGC is counter-intuitive: an absent or disconnected band pegs **high** (~6669)
because gain winds up finding nothing; RF overload reads **low** (~1400); healthy
is ~4200–4600. Confirm by comparing against the other board in the same room —
poor sky view affects both, a bad antenna affects one.

**Gotchas that cost real time**

- Never run `roslaunch` under `sudo`. It strips `LD_LIBRARY_PATH` and the driver
  respawn-loops on a missing `libublox_msgs.so`, looking exactly like a hardware
  fault. Use the `dialout` group.
- `MON-MSGPP`'s RTCM3 column reads zero even while valid RTCM is arriving. Use
  `UBX-RXM-RTCM` (class `0x02`, id `0x32`); its `crcFailed` flag is authoritative.
- `CFG-TMODE-FIXED_POS_ACC` is `0x4003000F`. `0x40030014` does not exist and NAKs
  the whole VALSET batch, silently leaving the base in survey-in.
- `numSV` counts satellites *used*, not tracked. Cross-check `NAV-SAT`.
- `rx = 0` with zero framing errors means the line is idle. A baud mismatch or
  bad wiring produces errors, not silence.
- Never hold two file descriptors on one tty — termios is per-device, not per-fd,
  so a second `open()` at a different baud silently reconfigures the first.
- A factory-reset F9P emits NMEA only, at 38400.

## License

MIT, see `LICENSE`.
