# ublox_utils

RTK GNSS for the robot: a u-blox ZED-F9P rover taking corrections from a
standalone ZED-F9P base over a 433 MHz radio link. No internet, no NTRIP, no
base-station computer.

**Verified: RTK FIXED at 1.6 cm horizontal**, base running headless on a power bank.

---

## Origin and what we changed

Forked from **[ethz-asl/ublox_utils](https://github.com/ethz-asl/ublox_utils)**
(MIT, Rik Bähnemann, ETH Zürich ASL).

Upstream targets a **different topology**: two receivers on the *same* vehicle in
a wired moving-baseline pair, with corrections from an **NTRIP caster over the
internet**. Our hardware is a **separated base and rover on custom 433 MHz
carrier boards**, deployed where there is no network at all. Almost none of
upstream's correction path applies.

### Added for our board

| Path | What it is |
|---|---|
| `launch/rover.launch` | Rover launch, designed to be included from a robot launch file |
| `config/rover.yaml` | Driver params, `config_on_startup: false` so the flashed receiver config is left alone |
| `scripts/rtk_status.py` | RTK health monitor node; usable while the driver holds the serial port |
| `tools/*.py` | 11 standalone diagnostic and configuration tools (no ROS dependency) |
| `tools/99-ublox.rules` | udev rules giving `/dev/ublox_base` and `/dev/ublox_rover` |
| `docs/board-433mhz-carrier.jpeg` | Board photo: CB1/CB2/CB3, SW1, CN2, CN6/CN7, SMA connectors |

### Changed

- `CMakeLists.txt` — install `launch/` and `config/`, register the Python node.
- `package.xml` — added `rospy`, `std_msgs`, `ublox_gps`.
- `launch/ublox.launch` — NTRIP defaults repointed from the Swiss `swipos`
  caster to `rtk2go.com`; credentials no longer mandatory args.
- `.gitignore` — Python bytecode.

### Kept from upstream, unused by the radio setup

`launch/ublox.launch`, `config/zed_f9p.yaml`, `config/moving_base.txt`,
`config/rover.txt`, `src/ublox2nmea.cc`, `resources/`. These implement the
moving-baseline and NTRIP paths. Note the u-center configs specify **UART2 at
460800**, correct for a direct wire and **wrong for this radio link (38400)**.

### Removed from the package

The supplier's ROS 2 packages and `f9p_config.py` were moved to
`../../reference/`. They contained **five ament packages nested inside this
catkin package**, which breaks `catkin build`. They are reference only — that
code uses MQTT/NTRIP corrections and a wired moving-baseline pair.

---

## Hardware

Two identical 433 MHz carrier boards, each with a ZED-F9P-01B-01 (FW HPG 1.32,
PROTVER 27.31) and an FTDI FT230X USB-serial bridge.

| Role | Serial | udev symlink |
|---|---|---|
| Rover | `D30G6YBQ` | `/dev/ublox_rover` |
| Base | `D30G6YBU` | `/dev/ublox_base` |

**Both boards must bridge CB1 (HM-TRP 433), with CB2 open.** The board has three
radio positions with selector bridges — CB1 (HM-TRP), CB2 (HC-12), CB3 (EBYTE
E22) — and the silkscreen warns *"ONLY USE 1 433MHz MODULE"*. Bridging two
lights **LD1 red**: a module-conflict warning, not a data LED. Base and rover
must use the *same* module; different chipsets cannot talk.

**The rover needs a dual-band (L1+L2) antenna.** L1-only gives FLOAT (~4 cm) and
never FIXED.

**Four SMA connectors — check the 433 antenna is in the radio port.** In a GNSS
port it produces an apparently dead link that intermittently works at very short
range, which reads convincingly as a bad solder joint.

## Quick start

```bash
sudo cp tools/99-ublox.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER      # then log out and back in

catkin_make --pkg ublox_utils       # see note on CATKIN_WHITELIST_PACKAGES below
source devel/setup.bash
roslaunch ublox_utils rover.launch
```

The receiver configuration lives in the receiver's own flash — nothing to
configure at launch time.

> If the workspace has `CATKIN_WHITELIST_PACKAGES` set (e.g. for velodyne), this
> package is silently skipped. Clear it with
> `catkin_make -DCATKIN_WHITELIST_PACKAGES=""`.

## ROS interface

`rover.launch` starts one `ublox_gps` node plus the health monitor. Topics are
published under the node name (default `ublox_rover`).

| Topic | Type | Purpose |
|---|---|---|
| `~fix` | `sensor_msgs/NavSatFix` | position |
| `~fix_velocity` | `geometry_msgs/TwistWithCovarianceStamped` | NED velocity |
| `~navpvt` | `ublox_msgs/NavPVT` | `fixType`, `flags`, `numSV`, `hAcc` |
| `~navstatus` | `ublox_msgs/NavSTATUS` | fix flags, differential status |
| `~rxmrtcm` | `ublox_msgs/RxmRTCM` | each correction received — proof the radio link is alive |
| `~rtk_state` | `std_msgs/String` | `none` / `FLOAT` / `FIXED` |
| `~rtk_ok` | `std_msgs/Bool` | true when FLOAT or FIXED |

RTK state is bits 6–7 of `NavPVT.flags`:

```python
carr = (msg.flags >> 6) & 0x03      # 0 = none, 1 = FLOAT, 2 = FIXED
```

`fixType == 3` means a 3D fix, **not** RTK — a rover with no corrections at all
still reports 3.

### Launch arguments

| Arg | Default | Meaning |
|---|---|---|
| `node_name` | `ublox_rover` | node name, and namespace for all topics |
| `device` | `/dev/ublox_rover` | serial port |
| `frame_id` | `gnss` | TF frame on the NavSatFix |
| `config` | `config/rover.yaml` | driver parameters |
| `rtk_status` | `true` | run the RTK health monitor |
| `respawn` | `true` | restart the driver if it dies |

### Integrating into a robot launch

```xml
<include file="$(find ublox_utils)/launch/rover.launch">
  <arg name="node_name" value="gnss"/>
  <arg name="frame_id"  value="gnss_link"/>
  <arg name="output"    value="log"/>
</include>
```

Gives `/gnss/fix`, `/gnss/navpvt`, `/gnss/rtk_state`, etc.

- **Nothing needs to publish corrections.** They arrive over UART2 from the
  radio, inside the receiver. No RTCM subscriber, no NTRIP client.
- **`config_on_startup` is `false` on purpose.** The receiver's settings are in
  flash; letting the driver reconfigure would overwrite them and stop UART2
  accepting corrections.
- **Covariance follows RTK state.** `NavSatFix.position_covariance` tightens by
  roughly two orders of magnitude when RTK engages. Gate on `~rtk_ok` if a
  downstream filter cares.

## Receiver configuration

Stored in **flash** on both boards; survives power cycles.

| | Base (`D30G6YBU`) | Rover (`D30G6YBQ`) |
|---|---|---|
| UART2 baud | **38400** | **38400** |
| UART2 direction | RTCM3 **out** | RTCM3 **in** |
| UART2 other protocols | off | **all outputs off** |
| UART1 baud | 38400 | 460800 |
| TMODE | survey-in, 60 s | rover |
| Dynamic model | stationary | automotive |
| Rate | 1 Hz | 5 Hz |
| RTCM out | 1005, 1074, 1084, 1094, 1124 @1 Hz, 1230 @0.2 Hz | — |

**The rover's UART2 outputs are all disabled** — the radios are half-duplex, so a
transmitting rover collides with the base's corrections.

**Survey-in is time-limited, not accuracy-limited.** The F9P completes a survey
only when *both* the minimum duration and the accuracy limit are met, and the
accuracy limit cannot be switched off. It is set to 100 m so it can never block,
making the 60 s duration the only binding constraint — the survey therefore
always completes, in any sky view.

That is deliberate: the base's absolute position error becomes a constant offset
shared by every rover fix, and a constant offset cancels exactly in relative
measurement. Absolute accuracy degrades; relative does not. For true absolute
accuracy, raise the duration and tighten `--svin-acc`.

```bash
python3 tools/verify_flash.py -d /dev/ublox_base  --role base
python3 tools/verify_flash.py -d /dev/ublox_rover --role rover
```

Non-zero exit if anything would not survive a power cycle.

## Field checklist

1. **433 antenna in the correct SMA**, both boards. GNSS antennas attached.
2. Power the base, give it sky view. Survey completes in 60 s.
3. `python3 tools/field_monitor.py --seconds 30` — expect `radio` ≈ 570 B/s,
   `RTCM/s` ≈ 5, `crcErr` 0, `1005 yes`, `RTK FIXED`.
4. Launch the robot stack; confirm `/gnss/rtk_state` reports `FIXED`.

## Tools

`tools/` — plain Python 3 + pyserial, no ROS. **They need exclusive access to the
serial port, so stop `ublox_gps` first.** Baud is autodetected; `-b` forces it.

| Tool | Purpose |
|---|---|
| `field_monitor.py` | live view of the whole correction chain — the one for the field |
| `gnss_diag.py` | firmware, ports, link use, message mix, satellites, AGC, fix, survey-in |
| `verify_flash.py` | confirm config will survive a power cycle |
| `check_rtk.py` | one-line RTK status |
| `stability.py` | dropouts, satellite churn, real position scatter |
| `configure_f9p.py` | flash a board into a role (`--dry-run` first) |
| `set_uart2_baud.py` | UART2 baud and protocol direction only |
| `link_sweep.py` | find the baud that makes the radio link pass bytes |
| `hmtrp_config.py` | query/factory-reset the HM-TRP over its own UART |
| `hc12_config.py` | talk to an HC-12 directly (only if you switch to CB2) |
| `rtcm_usb_relay.py` | base → rover over USB, bypassing the radio |
| `ubx.py` | shared UBX/NMEA/RTCM3 library (no CLI) |

`configure_f9p.py` roles: `static-base`, `moving-base`, `moving-rover`,
`rtk-rover`, `factory-reset`.

## Troubleshooting

| Symptom | Cause |
|---|---|
| `radio 0 B/s` | Wrong SMA port, base unpowered, or CB bridges mismatched between boards |
| Bytes arriving, `RTCM/s` 0 | UART2 baud differs between base and rover |
| RTCM arriving, no `1005` | Base survey has not completed; without it RTK cannot engage at all |
| `1005` seen, RTK stays `none` | Too few satellites — needs sky view |
| FLOAT but never FIXED | No L2. Check the antenna is dual-band |
| LD1 red | Two radio modules bridged at once |
| 0 satellites, `hAcc 4294967.295` | `0xFFFFFFFF` = invalid. No sky view, or antenna disconnected — check AGC |

**Diagnosing L2** with `tools/gnss_diag.py`:

```
L1  AGC 4563/8191  noise 79     <- healthy
L2  AGC 4212/8191  noise 48     <- healthy, receiving L2
L2  AGC 6669/8191  noise 48     <- pegged: nothing on L2, antenna is L1-only
```

AGC reads counter-intuitively: an absent or disconnected band pegs **high**
(~6669) because gain winds up finding nothing; RF overload reads **low**
(~1400); healthy is ~4200–4600, noise 76–86. Confirm by A/B against the other
board in the same room — indoor attenuation affects both, a bad antenna affects
one.

### Traps worth knowing

- **Never run `roslaunch` under `sudo`.** It strips `LD_LIBRARY_PATH`; the driver
  respawn-loops on `libublox_msgs.so: cannot open shared object file`, which
  looks exactly like a hardware fault. Use the `dialout` group.
- **`MON-MSGPP`'s RTCM3 column is not trustworthy** — it read zero while 240
  valid RTCM messages were arriving. Use **`UBX-RXM-RTCM`** (class `0x02`, id
  `0x32`); its `crcFailed` flag is authoritative.
- **`CFG-TMODE-FIXED_POS_ACC` is `0x4003000F`.** `0x40030014` does not exist and
  NAKs the whole VALSET batch, silently leaving the base in survey-in.
- **`numSV` counts satellites used, not tracked.** Cross-check `NAV-SAT`. A stale
  fixed base position 161 km away once caused 12 tracked / 0 used.
- **`rx = 0` with zero framing errors means the line is idle** — nothing driving
  it. A baud mismatch or bad wiring produces *errors*, not silence.
- **Never hold two file descriptors on one tty.** termios is per-device, not
  per-fd, so a second `open()` at a different baud silently reconfigures the
  first. This once produced a false "rover is dead" reading.
- **A factory-reset F9P emits NMEA only** at 38400. Baud detection scoring UBX
  frames alone reports a healthy receiver as dead.
- **Payloads contain stray `$` and `0xD3` bytes.** Sync on a verified
  checksum/CRC, never the sync byte alone.
- **UART2 traffic is invisible from a UART1 connection.** Read `MON-IO` and
  `MON-MSGPP` per-port counters instead.

## License

MIT — see `LICENSE`. Upstream: [ethz-asl/ublox_utils](https://github.com/ethz-asl/ublox_utils).
