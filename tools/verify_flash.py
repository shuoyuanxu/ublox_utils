#!/usr/bin/env python3
"""Verify what is actually stored in FLASH, not just RAM.

CFG-VALGET reads a specific storage layer: 0=RAM, 1=BBR, 2=Flash, 7=Default.
Every other tool here reads RAM, which reflects the running config and says
nothing about what survives a power cycle. Before deploying, check layer 2.

    python3 tools/verify_flash.py --role base  -d /dev/ublox_base
    python3 tools/verify_flash.py --role rover -d /dev/ublox_rover

Exit code is 0 only if every checked key matches the expected deployment value.
"""

import argparse
import sys

import serial

import ubx

U1 = -1  # UART1 msgout key = UART2 key - 1

TMODE_MODE = ubx.KEY["CFG-TMODE-MODE"]
SVIN_DUR = ubx.KEY["CFG-TMODE-SVIN_MIN_DUR"]
SVIN_ACC = ubx.KEY["CFG-TMODE-SVIN_ACC_LIMIT"]
DYNMODEL = ubx.KEY["CFG-NAVSPG-DYNMODEL"]
RATE = ubx.KEY["CFG-RATE-MEAS"]
U2BAUD = ubx.KEY["CFG-UART2-BAUDRATE"]
U1BAUD = ubx.KEY["CFG-UART1-BAUDRATE"]
U2OUT_RTCM = ubx.KEY["CFG-UART2OUTPROT-RTCM3X"]
U2IN_RTCM = ubx.KEY["CFG-UART2INPROT-RTCM3X"]
U2OUT_UBX = ubx.KEY["CFG-UART2OUTPROT-UBX"]
U2OUT_NMEA = ubx.KEY["CFG-UART2OUTPROT-NMEA"]

R = ubx.RTCM_UART2


def expected(role):
    """(key, label, predicate, description-of-expected)"""
    common = [
        (U2BAUD, "UART2 baud", lambda v: v == 38400, "38400 (the proven working rate)"),
    ]
    if role == "base":
        return common + [
            (U2OUT_RTCM, "UART2 out RTCM3", lambda v: v == 1, "enabled"),
            (R["1005"], "RTCM 1005 -> UART2", lambda v: v and v >= 1, "enabled (base coordinate)"),
            (R["1074"], "RTCM 1074 -> UART2", lambda v: v and v >= 1, "enabled"),
            (R["1084"], "RTCM 1084 -> UART2", lambda v: v and v >= 1, "enabled"),
            (R["1094"], "RTCM 1094 -> UART2", lambda v: v and v >= 1, "enabled"),
            (R["1124"], "RTCM 1124 -> UART2", lambda v: v and v >= 1, "enabled"),
            (TMODE_MODE, "TMODE", lambda v: v in (1, 2), "1=survey-in or 2=fixed"),
            (DYNMODEL, "dynModel", lambda v: v == 2, "2=stationary"),
        ]
    return common + [
        (U2IN_RTCM, "UART2 in RTCM3", lambda v: v == 1, "enabled"),
        (U2OUT_RTCM, "UART2 out RTCM3", lambda v: v == 0, "disabled (half-duplex radio)"),
        (U2OUT_UBX, "UART2 out UBX", lambda v: v == 0, "disabled"),
        (U2OUT_NMEA, "UART2 out NMEA", lambda v: v == 0, "disabled"),
        (TMODE_MODE, "TMODE", lambda v: v == 0, "0=rover"),
        (U1BAUD, "UART1 baud", lambda v: v == 460800, "460800 (host link)"),
    ]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--device", required=True)
    ap.add_argument("--role", required=True, choices=["base", "rover"])
    ap.add_argument("-b", "--baud", type=int, default=0)
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.device, args.baud or 38400, timeout=0.1)
    except serial.SerialException as e:
        print("Cannot open %s: %s" % (args.device, e))
        return 2

    with ser:
        if args.baud:
            ser.baudrate = args.baud
        elif ubx.detect_baud(ser, verbose=False) is None:
            print("No response at any baud on %s." % args.device)
            return 2

        checks = expected(args.role)
        keys = [k for k, _, _, _ in checks]
        # Survey-in params are reported, not pass/failed, so they are not in
        # `checks` -- query them explicitly or the summary below reads empty.
        extra = [SVIN_DUR, SVIN_ACC, RATE] if args.role == "base" else [RATE]
        flash = ubx.valget(ser, keys + extra, layer=2)   # 2 = Flash
        ram = ubx.valget(ser, keys, layer=0)             # 0 = RAM

        print("%s  role=%s" % (args.device, args.role))
        print("Reading FLASH layer (layer 2) -- this is what survives a power cycle.\n")
        print("%-24s %12s %12s   %s" % ("setting", "FLASH", "RAM", "verdict"))
        print("-" * 78)

        bad = []
        for key, label, ok, want in checks:
            fv = flash.get(key)
            rv = ram.get(key)
            if fv is None:
                verdict = "NOT IN FLASH -- will revert on power cycle"
                bad.append((label, want, "absent"))
            elif not ok(fv):
                verdict = "WRONG, expected %s" % want
                bad.append((label, want, fv))
            elif rv is not None and rv != fv:
                verdict = "flash ok, but RAM differs (running config drifted)"
            else:
                verdict = "ok"
            print("%-24s %12s %12s   %s"
                  % (label,
                     "-" if fv is None else fv,
                     "-" if rv is None else rv,
                     verdict))

        print()
        if bad:
            print("NOT SAFE TO DEPLOY -- %d setting(s) will not survive a power cycle:"
                  % len(bad))
            for label, want, got in bad:
                print("   %-24s expected %s, flash has %s" % (label, want, got))
            return 1

        print("All checked settings are in FLASH and correct.")
        if args.role == "base":
            tm = flash.get(TMODE_MODE)
            if tm == 1:
                d = flash.get(SVIN_DUR)
                a = flash.get(SVIN_ACC)
                print("\nBase is in SURVEY-IN. It emits observations but NOT 1005 until the")
                print("survey completes, so the rover cannot reach RTK during that window.")
                if d is not None:
                    acc_m = (a or 0) / 10000.0
                    print("  min duration:    %d s" % d)
                    print("  accuracy limit:  %.1f m%s"
                          % (acc_m, "  (effectively disabled -- duration is the"
                             " binding constraint)" if acc_m >= 50 else
                             "  <-- may block completion in poor sky view"))
            elif tm == 2:
                print("\nBase is in FIXED position mode -- it sends 1005 immediately.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
