#!/usr/bin/env python3
"""Set UART2 baud to the value this hardware was designed around.

Both the supplier's f9p_config.py (cfg_valget_uart2_baudrate, key 0x40530001 =
0x00070800) and ETH's config/moving_base.txt + config/rover.txt set UART2 to
460800. Anything else is non-standard for these boards, and if the 433 MHz
modules were factory-programmed to match the supplier's config, 460800 is what
they expect on their UART.

Writes to RAM and Flash so it survives a power cycle, matching the supplier's
layers = 0x05.

    python3 tools/set_uart2_baud.py                          # 460800, rover roles
    python3 tools/set_uart2_baud.py --role base              # also enable RTCM3 out
    python3 tools/set_uart2_baud.py --baud 38400 --role rover  # revert
    python3 tools/set_uart2_baud.py --dry-run

Run it once per board. The base is remote, so do that board when you next have
it on a USB cable.
"""

import argparse
import sys

import serial

import ubx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--device", default="/dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=0,
                    help="current UART1/USB baud of the receiver, 0 = autodetect")
    ap.add_argument("--baud2", type=int, default=460800,
                    help="UART2 baud to program (default 460800)")
    ap.add_argument("--role", choices=["base", "rover", "none"], default="none",
                    help="also set UART2 protocol direction for this role")
    ap.add_argument("--ram-only", action="store_true",
                    help="do not write flash (reverts on power cycle)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    items = {ubx.KEY["CFG-UART2-BAUDRATE"]: args.baud2}

    if args.role == "base":
        # Base transmits corrections, listens to nothing.
        items[ubx.KEY["CFG-UART2OUTPROT-RTCM3X"]] = 1
        items[ubx.KEY["CFG-UART2OUTPROT-UBX"]] = 0
        items[ubx.KEY["CFG-UART2OUTPROT-NMEA"]] = 0
        items[ubx.KEY["CFG-UART2INPROT-RTCM3X"]] = 0
        items[ubx.KEY["CFG-UART2INPROT-UBX"]] = 0
        items[ubx.KEY["CFG-UART2INPROT-NMEA"]] = 0
    elif args.role == "rover":
        # Rover listens for corrections and stays quiet: these radios are
        # half-duplex, so a transmitting rover collides with the base.
        items[ubx.KEY["CFG-UART2INPROT-RTCM3X"]] = 1
        items[ubx.KEY["CFG-UART2INPROT-UBX"]] = 0
        items[ubx.KEY["CFG-UART2INPROT-NMEA"]] = 0
        items[ubx.KEY["CFG-UART2OUTPROT-RTCM3X"]] = 0
        items[ubx.KEY["CFG-UART2OUTPROT-UBX"]] = 0
        items[ubx.KEY["CFG-UART2OUTPROT-NMEA"]] = 0

    layers = ubx.LAYER_RAM if args.ram_only else (ubx.LAYER_RAM | ubx.LAYER_FLASH)

    print("Device:  %s" % args.device)
    print("Role:    %s" % args.role)
    print("Layers:  %s" % ("RAM only" if args.ram_only else "RAM + Flash (persistent)"))
    print("Changes:")
    names = {v: k for k, v in ubx.KEY.items()}
    for k, v in items.items():
        print("  %-28s = %s" % (names.get(k, hex(k)), v))

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
                print("\nNo valid frames at any baud. Cannot talk to the receiver.")
                return 1
            print("\nTalking to receiver at %d baud." % b)

        before = ubx.valget(ser, [ubx.KEY["CFG-UART2-BAUDRATE"]])
        if before:
            print("UART2 baud before: %s" % before.get(ubx.KEY["CFG-UART2-BAUDRATE"]))

        ok = ubx.valset(ser, items, layers=layers)
        if ok is None:
            print("\nNo ACK/NAK within the timeout. Nothing confirmed.")
            return 1
        if not ok:
            print("\nUBX-ACK-NAK: the receiver rejected the configuration.")
            return 1
        print("UBX-ACK-ACK: applied.")

        after = ubx.valget(ser, [ubx.KEY["CFG-UART2-BAUDRATE"]])
        if after:
            print("UART2 baud after:  %s" % after.get(ubx.KEY["CFG-UART2-BAUDRATE"]))

    print("\nRemember: both ends must match. Program the other board the same way.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
