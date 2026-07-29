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

Once per machine — the udev rule creates `/dev/ublox_rover`, which the launch
file expects, and `dialout` lets the driver open it:

```bash
sudo cp tools/99-ublox.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER      # log out and back in
```

Then build:

```bash
catkin_make
source devel/setup.bash
```

> If the workspace sets `CATKIN_WHITELIST_PACKAGES`, this package is silently
> skipped. Clear it with `catkin_make -DCATKIN_WHITELIST_PACKAGES=""`.

## Usage

```bash
roslaunch ublox_utils rover.launch
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

## License

MIT, see `LICENSE`.
