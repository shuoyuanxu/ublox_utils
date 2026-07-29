#!/usr/bin/env python3
"""Flash a ZED-F9P into one of the roles this package supports.

Roles
  static-base    Survey-in base for a separated base/rover pair. Emits RTCM
                 1005 + MSM4 observations. 1005 is what a static rover needs to
                 resolve an absolute baseline, and it only appears once the
                 survey completes.
  moving-base    The supplier's and ETH's configuration: RTCM 1074/1084/1094/
                 1124/1230 + 4072.0 on UART2. 4072.0 is u-blox proprietary and
                 exists only for moving baseline; there is deliberately no 1005.
                 Matches config/moving_base.txt and f9p_config.py.
  moving-rover   Companion to moving-base. Listens for RTCM3 on UART2 and emits
                 NAV-RELPOSNED on UART1. Matches config/rover.txt.
  rtk-rover      Rover for a separated base, or for NTRIP corrections injected
                 over USB by ublox_gps. Listens for RTCM3 on UART2.
  factory-reset  UBX-CFG-CFG full reset. This is what cleared the undocumented
                 class 0x27 debug flood that was saturating UART1.

    python3 tools/configure_f9p.py --role rtk-rover
    python3 tools/configure_f9p.py --role static-base --rtcm-port usb
    python3 tools/configure_f9p.py --role moving-base --dry-run

After a factory-reset the receiver drops to 38400 and emits NMEA only; that is
expected, not a failure.
"""

import argparse
import sys
import time

import serial

import ubx

# CFG-MSGOUT keys are laid out I2C, UART1, UART2, USB, SPI at consecutive ids,
# so a port variant is a fixed offset from the UART2 key.
PORT_OFFSET = {"i2c": -2, "uart1": -1, "uart2": 0, "usb": +1, "spi": +2}

NAV_RELPOSNED_UART1 = 0x2091008E  # from config/rover.txt

MSM4 = ("1074", "1084", "1094", "1124")


def msgout(uart2_key, port):
    return uart2_key + PORT_OFFSET[port]


def inprot(port):
    base = {"uart1": 0x10730000, "uart2": 0x10750000}[port]
    return {"UBX": base + 1, "NMEA": base + 2, "RTCM3X": base + 4}


def outprot(port):
    base = {"uart1": 0x10740000, "uart2": 0x10760000}[port]
    return {"UBX": base + 1, "NMEA": base + 2, "RTCM3X": base + 4}


def build(role, args):
    """Return {key: value} for the requested role."""
    items = {}

    if role == "static-base":
        items[ubx.KEY["CFG-TMODE-MODE"]] = 1  # survey-in
        items[ubx.KEY["CFG-TMODE-SVIN_MIN_DUR"]] = args.svin_dur
        # SVIN_ACC_LIMIT is in 0.1 mm units.
        items[ubx.KEY["CFG-TMODE-SVIN_ACC_LIMIT"]] = int(args.svin_acc * 10000)
        items[ubx.KEY["CFG-NAVSPG-DYNMODEL"]] = 2  # stationary
        items[ubx.KEY["CFG-RATE-MEAS"]] = 1000     # 1 Hz is plenty for a base
        items[ubx.KEY["CFG-RATE-NAV"]] = 1
        p = args.rtcm_port
        for m in MSM4:
            items[msgout(ubx.RTCM_UART2[m], p)] = 1
        items[msgout(ubx.RTCM_UART2["1005"], p)] = 1
        items[msgout(ubx.RTCM_UART2["1230"], p)] = 10  # every 10th epoch
        items[msgout(ubx.RTCM_UART2["4072_0"], p)] = 0  # moving-base only
        if p in ("uart1", "uart2"):
            items[outprot(p)["RTCM3X"]] = 1
            items[inprot(p)["RTCM3X"]] = 0
            items[inprot(p)["NMEA"]] = 0
        if p == "uart2":
            items[ubx.KEY["CFG-UART2-BAUDRATE"]] = args.baud2

    elif role == "moving-base":
        items[ubx.KEY["CFG-TMODE-MODE"]] = 0
        items[ubx.KEY["CFG-RATE-MEAS"]] = 125  # 0x7d, per moving_base.txt
        items[ubx.KEY["CFG-RATE-NAV"]] = 1
        items[ubx.KEY["CFG-UART1-BAUDRATE"]] = 460800
        items[ubx.KEY["CFG-UART2-BAUDRATE"]] = 460800
        for m in MSM4:
            items[ubx.RTCM_UART2[m]] = 1
        items[ubx.RTCM_UART2["1230"]] = 1
        items[ubx.RTCM_UART2["4072_0"]] = 1
        items[ubx.RTCM_UART2["1005"]] = 0  # not used for moving baseline
        items[outprot("uart2")["RTCM3X"]] = 1
        items[outprot("uart2")["NMEA"]] = 0

    elif role == "moving-rover":
        items[ubx.KEY["CFG-TMODE-MODE"]] = 0
        items[ubx.KEY["CFG-RATE-MEAS"]] = 125
        items[ubx.KEY["CFG-RATE-NAV"]] = 1
        items[ubx.KEY["CFG-UART1-BAUDRATE"]] = 460800
        items[ubx.KEY["CFG-UART2-BAUDRATE"]] = 460800
        items[NAV_RELPOSNED_UART1] = 1
        items[inprot("uart2")["RTCM3X"]] = 1
        items[inprot("uart2")["NMEA"]] = 0
        items[outprot("uart2")["RTCM3X"]] = 0
        items[outprot("uart2")["NMEA"]] = 0

    elif role == "rtk-rover":
        items[ubx.KEY["CFG-TMODE-MODE"]] = 0
        items[ubx.KEY["CFG-NAVSPG-DYNMODEL"]] = 4  # automotive
        items[ubx.KEY["CFG-RATE-MEAS"]] = int(1000 / args.rate)
        items[ubx.KEY["CFG-RATE-NAV"]] = 1
        items[ubx.KEY["CFG-UART2-BAUDRATE"]] = args.baud2
        items[inprot("uart2")["RTCM3X"]] = 1
        items[inprot("uart2")["NMEA"]] = 0
        items[inprot("uart2")["UBX"]] = 0
        # Half-duplex radios: a talking rover collides with the base.
        items[outprot("uart2")["RTCM3X"]] = 0
        items[outprot("uart2")["NMEA"]] = 0
        items[outprot("uart2")["UBX"]] = 0
        items[outprot("uart1")["NMEA"]] = 0  # NMEA off, keeps UART1 clear

    return items


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--device", default="/dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=0, help="0 = autodetect")
    ap.add_argument("--role", required=True,
                    choices=["static-base", "moving-base", "moving-rover",
                             "rtk-rover", "factory-reset"])
    ap.add_argument("--rtcm-port", default="uart2",
                    choices=["uart1", "uart2", "usb", "i2c", "spi"],
                    help="where a base emits RTCM (use 'usb' for rtcm_usb_relay.py)")
    ap.add_argument("--baud2", type=int, default=460800,
                    help="UART2 baud (default 460800, the value both reference "
                         "configs use)")
    ap.add_argument("--rate", type=float, default=5.0, help="rover Hz")
    ap.add_argument("--svin-dur", type=int, default=300, help="survey-in seconds")
    ap.add_argument("--svin-acc", type=float, default=5.0, help="survey-in metres")
    ap.add_argument("--ram-only", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    layers = ubx.LAYER_RAM if args.ram_only else (ubx.LAYER_RAM | ubx.LAYER_FLASH)
    names = {v: k for k, v in ubx.KEY.items()}
    for m, k in ubx.RTCM_UART2.items():
        for p, off in PORT_OFFSET.items():
            names[k + off] = "CFG-MSGOUT-RTCM_%s_%s" % (m, p.upper())
    names[NAV_RELPOSNED_UART1] = "CFG-MSGOUT-UBX_NAV_RELPOSNED_UART1"
    for p in ("uart1", "uart2"):
        for n, k in inprot(p).items():
            names[k] = "CFG-%sINPROT-%s" % (p.upper(), n)
        for n, k in outprot(p).items():
            names[k] = "CFG-%sOUTPROT-%s" % (p.upper(), n)

    print("Device: %s" % args.device)
    print("Role:   %s" % args.role)
    print("Layers: %s" % ("RAM only" if args.ram_only else "RAM + Flash (persistent)"))

    if args.role == "factory-reset":
        print("\nWill send UBX-CFG-CFG clearing all sections in BBR + Flash.")
        print("This ERASES the current configuration on this board, including any")
        print("survey-in result or fixed base position. It is not reversible from")
        print("here -- note down anything you need first.")
        items = None
    else:
        items = build(args.role, args)
        print("\nChanges (%d keys):" % len(items))
        for k in sorted(items):
            print("  %-42s = %s" % (names.get(k, hex(k)), items[k]))

    if args.dry_run:
        print("\nDry run, nothing written.")
        return 0

    try:
        ser = serial.Serial(args.device, args.baud or 38400, timeout=0.1)
    except serial.SerialException as e:
        print("\nCannot open %s: %s" % (args.device, e))
        if "Permission denied" in str(e):
            print("sudo usermod -aG dialout $USER   (then log out and back in)")
        return 2

    with ser:
        if args.baud:
            ser.baudrate = args.baud
        else:
            b = ubx.detect_baud(ser, verbose=False)
            if b is None:
                print("\nNo valid frames at any baud on %s." % args.device)
                return 1
            print("\nTalking to receiver at %d baud." % b)

        if args.role == "factory-reset":
            # clearMask=all, saveMask=0, loadMask=all, deviceMask=BBR|Flash|EEPROM|SPI
            payload = (b"\xff\xff\xff\xff" b"\x00\x00\x00\x00"
                       b"\xff\xff\xff\xff" b"\x0f")
            ser.write(ubx.frame(0x06, 0x09, payload))
            ser.flush()
            time.sleep(2.0)
            print("Reset sent. The receiver reboots at 38400 emitting NMEA only.")
            print("Re-run gnss_diag.py to confirm, then reconfigure for its role.")
            return 0

        ok = ubx.valset(ser, items, layers=layers)
        if ok is None:
            print("\nNo ACK/NAK within the timeout. Nothing confirmed.")
            return 1
        if not ok:
            print("\nUBX-ACK-NAK: the receiver rejected the configuration.")
            print("A NAK on a batch means none of it applied. Try --dry-run to")
            print("review, or split the role into smaller writes.")
            return 1
        print("UBX-ACK-ACK: applied.")

        if args.role in ("moving-base", "moving-rover") or args.baud2 != ser.baudrate:
            print("\nNote: UART1 baud may have changed. Re-run gnss_diag.py without")
            print("--baud so it re-detects before you trust the next reading.")

    if args.role == "static-base":
        print("\nSurvey-in started: %d s minimum, %.1f m accuracy limit." %
              (args.svin_dur, args.svin_acc))
        print("It emits observations but NOT 1005 until it completes, so no FIXED")
        print("during the window. It will not converge indoors.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
