#!/usr/bin/env python3
"""Measure how steady the rover's solution actually is over a window.

Reports solution continuity (rate, dropouts, fix-type changes), satellite count
stability, accuracy stability, and real position scatter in metres. Scatter is
the number that matters for a robot: hAcc is the receiver's own estimate, while
scatter is what your control loop will actually see.

    python3 tools/stability.py -d /dev/ublox_rover -s 60
"""

import argparse
import math
import sys

import serial

import ubx


def stats(v):
    if not v:
        return (0.0, 0.0, 0.0)
    m = sum(v) / len(v)
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / len(v)) if len(v) > 1 else 0.0
    return (m, sd, max(v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--device", default="/dev/ublox_rover")
    ap.add_argument("-b", "--baud", type=int, default=0)
    ap.add_argument("-s", "--seconds", type=float, default=60.0)
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.device, args.baud or 38400, timeout=0.1)
    except serial.SerialException as e:
        print("Cannot open %s: %s" % (args.device, e))
        return 2

    with ser:
        if args.baud:
            ser.baudrate = args.baud
        else:
            b = ubx.detect_baud(ser, verbose=False)
            if b is None:
                print("No valid frames at any baud.")
                return 1
            print("%s @ %d baud" % (args.device, b))

        print("Sampling %.0f s ...\n" % args.seconds)
        raw = ubx.read_for(ser, args.seconds)
        msgs, _ = ubx.parse_stream(raw)

        pvts = [ubx.decode_nav_pvt(m["payload"]) for m in msgs
                if m["kind"] == "ubx" and (m["cls"], m["id"]) == (0x01, 0x07)]
        pvts = [p for p in pvts if p]
        if len(pvts) < 5:
            print("Only %d NAV-PVT in the window -- nothing to judge." % len(pvts))
            return 1

        # --- continuity
        itows = [p["itow"] for p in pvts]
        gaps = [(b - a) / 1000.0 for a, b in zip(itows, itows[1:])]
        nominal = min(gaps)
        dropouts = [g for g in gaps if g > nominal * 1.5]
        rate = len(pvts) / args.seconds

        print("SOLUTION CONTINUITY")
        print("  epochs        %d in %.0f s = %.2f Hz" % (len(pvts), args.seconds, rate))
        print("  nominal gap   %.3f s" % nominal)
        print("  largest gap   %.3f s" % max(gaps))
        print("  dropouts      %d%s" % (len(dropouts),
                                        "" if not dropouts else "  <-- %s" % dropouts[:8]))

        fixes = set(p["fix_type"] for p in pvts)
        carrs = set(p["carr_soln"] for p in pvts)
        print("  fix type      %s%s"
              % (", ".join(ubx.FIX_TYPES.get(f, str(f)) for f in sorted(fixes)),
                 "  <-- UNSTABLE, changed during window" if len(fixes) > 1 else "  (constant)"))
        print("  RTK mode      %s"
              % ", ".join(ubx.CARR_SOLN.get(c, str(c)) for c in sorted(carrs)))

        # --- satellites and accuracy
        sv = [p["num_sv"] for p in pvts]
        ha = [p["h_acc"] for p in pvts]
        m, sd, mx = stats(sv)
        print("\nSATELLITES USED")
        print("  min %d   max %d   mean %.1f   sd %.2f%s"
              % (min(sv), max(sv), m, sd,
                 "  <-- fluctuating" if max(sv) - min(sv) > 4 else "  (steady)"))

        m, sd, mx = stats(ha)
        print("\nREPORTED ACCURACY (hAcc)")
        print("  min %.3f m   max %.3f m   mean %.3f m   sd %.3f m" % (min(ha), mx, m, sd))

        # --- real scatter, converted to metres at this latitude
        lat0 = sum(p["lat"] for p in pvts) / len(pvts)
        lon0 = sum(p["lon"] for p in pvts) / len(pvts)
        h0 = sum(p["height"] for p in pvts) / len(pvts)
        mlat = 111132.0
        mlon = 111320.0 * math.cos(math.radians(lat0))
        dn = [(p["lat"] - lat0) * mlat for p in pvts]
        de = [(p["lon"] - lon0) * mlon for p in pvts]
        dh = [p["height"] - h0 for p in pvts]
        hor = [math.hypot(a, b) for a, b in zip(dn, de)]

        _, sd_n, _ = stats(dn)
        _, sd_e, _ = stats(de)
        _, sd_h, _ = stats(dh)
        drms = math.hypot(sd_n, sd_e)

        print("\nACTUAL POSITION SCATTER (stationary receiver)")
        print("  north sd      %.3f m" % sd_n)
        print("  east  sd      %.3f m" % sd_e)
        print("  horizontal    %.3f m 1-sigma (DRMS), %.3f m worst excursion"
              % (drms, max(hor)))
        print("  vertical sd   %.3f m" % sd_h)

        # --- signal strength stability
        cnos = []
        for m_ in msgs:
            if m_["kind"] == "ubx" and (m_["cls"], m_["id"]) == (0x01, 0x35):
                sats = ubx.decode_nav_sat(m_["payload"])
                used = [s["cno"] for s in sats if s["used"] and s["cno"]]
                if used:
                    cnos.append(sum(used) / len(used))
        if cnos:
            m, sd, _ = stats(cnos)
            print("\nSIGNAL STRENGTH (mean C/N0 of used satellites)")
            print("  mean %.1f dB-Hz   sd %.2f%s"
                  % (m, sd, "  (steady)" if sd < 1.5 else "  <-- varying"))

        # --- verdict
        print("\nVERDICT")
        ok = True
        if dropouts:
            print("  x %d dropouts in %.0f s" % (len(dropouts), args.seconds))
            ok = False
        if len(fixes) > 1:
            print("  x fix type changed during the window")
            ok = False
        if max(sv) - min(sv) > 4:
            print("  x satellite count swung by %d" % (max(sv) - min(sv)))
            ok = False
        if drms > 1.5:
            print("  x horizontal scatter %.2f m is high even for standalone" % drms)
            ok = False
        if ok:
            print("  Solution is STABLE: no dropouts, constant fix type, steady")
            print("  satellite count, %.2f m 1-sigma scatter." % drms)
            print("  This is normal standalone (non-RTK) performance.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
