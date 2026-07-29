#!/usr/bin/env python3
"""One-shot RTK status plus UART2 correction counters.

Answers the two questions you actually care about mid-experiment: is the rover
in FLOAT/FIXED, and are any correction bytes arriving on UART2 at all.

    python3 tools/check_rtk.py                  # single snapshot
    python3 tools/check_rtk.py --watch          # live, Ctrl-C to stop
"""

import argparse
import sys
import time

import serial

import ubx


def snapshot(ser):
    """Return (pvt, svin, uart2_rx_delta, uart2_rtcm_delta) over ~1 s."""
    io_a = ubx.poll(ser, 0x0A, 0x02)
    pp_a = ubx.poll(ser, 0x0A, 0x06)
    time.sleep(1.0)
    io_b = ubx.poll(ser, 0x0A, 0x02)
    pp_b = ubx.poll(ser, 0x0A, 0x06)

    rx = rtcm = None
    if io_a and io_b:
        a = {p["port"]: p for p in ubx.decode_mon_io(io_a)}
        b = {p["port"]: p for p in ubx.decode_mon_io(io_b)}
        if "UART2" in a and "UART2" in b:
            rx = b["UART2"]["rx"] - a["UART2"]["rx"]
    if pp_a and pp_b:
        a = {p["port"]: p for p in ubx.decode_mon_msgpp(pp_a)}
        b = {p["port"]: p for p in ubx.decode_mon_msgpp(pp_b)}
        if "UART2" in a and "UART2" in b:
            rtcm = b["UART2"]["counts"]["RTCM3"] - a["UART2"]["counts"]["RTCM3"]

    p = ubx.poll(ser, 0x01, 0x07)
    s = ubx.poll(ser, 0x01, 0x3B)
    return (ubx.decode_nav_pvt(p) if p else None,
            ubx.decode_nav_svin(s) if s else None, rx, rtcm)


def line(pvt, svin, rx, rtcm):
    if pvt is None:
        return "no NAV-PVT"
    parts = [
        "%-9s" % ubx.FIX_TYPES.get(pvt["fix_type"], "?"),
        "RTK %-5s" % ubx.CARR_SOLN.get(pvt["carr_soln"], "?"),
        "%2d sv" % pvt["num_sv"],
        "hAcc %6.3f m" % pvt["h_acc"],
    ]
    parts.append("UART2 rx %s B/s" % ("?" if rx is None else rx))
    parts.append("RTCM3 %s/s" % ("?" if rtcm is None else rtcm))
    if svin and svin["active"]:
        parts.append("svin %ds %.2fm" % (svin["dur"], svin["mean_acc"]))
    return "  ".join(parts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--device", default="/dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=0, help="0 = autodetect")
    ap.add_argument("-w", "--watch", action="store_true")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.device, args.baud or 38400, timeout=0.1)
    except serial.SerialException as e:
        print("Cannot open %s: %s" % (args.device, e))
        if "Permission denied" in str(e):
            print("sudo usermod -aG dialout $USER   (then log out and back in)")
        return 2

    with ser:
        if args.baud:
            ser.baudrate = args.baud
        else:
            b = ubx.detect_baud(ser, verbose=False)
            if b is None:
                print("No valid frames at any baud on %s." % args.device)
                return 1
            print("# %s @ %d baud" % (args.device, b))

        if not args.watch:
            print(line(*snapshot(ser)))
            return 0

        print("# Ctrl-C to stop")
        try:
            while True:
                print(line(*snapshot(ser)))
                sys.stdout.flush()
        except KeyboardInterrupt:
            print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
