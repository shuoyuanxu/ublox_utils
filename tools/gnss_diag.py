#!/usr/bin/env python3
"""Full-picture diagnostic for a ZED-F9P over USB/UART.

Reports firmware, link utilisation, per-port byte counters (the only way to see
UART2 traffic from a UART1 connection), message mix, satellites, C/N0, AGC and
jamming, RTK state, and survey-in progress.

    python3 tools/gnss_diag.py                     # autodetect baud on ttyUSB0
    python3 tools/gnss_diag.py -d /dev/ttyUSB1 -b 460800
    python3 tools/gnss_diag.py --seconds 20        # longer traffic sample
"""

import argparse
import sys
import time
from collections import Counter

import serial

import ubx


def human(n):
    return "%.1f kB" % (n / 1000.0) if n >= 10000 else "%d B" % n


def section(title):
    print("\n" + title)
    print("-" * len(title))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--device", default="/dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=0, help="0 = autodetect")
    ap.add_argument("-s", "--seconds", type=float, default=10.0,
                    help="traffic sampling window")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.device, args.baud or 38400, timeout=0.1)
    except serial.SerialException as e:
        print("Cannot open %s: %s" % (args.device, e))
        if "Permission denied" in str(e):
            print("\nYou are not in the 'dialout' group. Fix with:")
            print("    sudo usermod -aG dialout $USER")
            print("then log out and back in (or: newgrp dialout).")
        return 2

    # One handle for the whole run. Never open this tty twice concurrently.
    with ser:
        print("Device: %s" % args.device)

        if args.baud:
            baud = args.baud
            ser.baudrate = baud
            print("Baud:   %d (forced)" % baud)
        else:
            print("\nProbing baud rates (counts UBX + NMEA + RTCM3):")
            baud = ubx.detect_baud(ser)
            if baud is None:
                print("\nNo valid frames at any baud. The receiver is silent or the")
                print("line is not connected. Try --baud to force one and re-check.")
                return 1
            print("Baud:   %d (detected)" % baud)

        # --- identity -----------------------------------------------------
        section("Firmware")
        ver = ubx.poll(ser, 0x0A, 0x04)
        if ver is None:
            print("MON-VER did not answer. Receiver may be emitting NMEA only")
            print("(that is normal right after a factory reset) or UBX input is off.")
        else:
            print(ver[0:30].split(b"\x00")[0].decode("ascii", "replace"))
            print(ver[30:40].split(b"\x00")[0].decode("ascii", "replace"))
            for i in range(40, len(ver), 30):
                ext = ver[i:i + 30].split(b"\x00")[0].decode("ascii", "replace")
                if ext:
                    print("  " + ext)

        # --- configuration snapshot ---------------------------------------
        section("Port configuration (RAM)")
        cfg = ubx.valget(ser, [
            ubx.KEY["CFG-UART1-BAUDRATE"], ubx.KEY["CFG-UART2-BAUDRATE"],
            ubx.KEY["CFG-UART2INPROT-UBX"], ubx.KEY["CFG-UART2INPROT-NMEA"],
            ubx.KEY["CFG-UART2INPROT-RTCM3X"],
            ubx.KEY["CFG-UART2OUTPROT-UBX"], ubx.KEY["CFG-UART2OUTPROT-NMEA"],
            ubx.KEY["CFG-UART2OUTPROT-RTCM3X"],
        ])
        if not cfg:
            print("CFG-VALGET did not answer.")
        else:
            u1 = cfg.get(ubx.KEY["CFG-UART1-BAUDRATE"])
            u2 = cfg.get(ubx.KEY["CFG-UART2-BAUDRATE"])
            print("UART1 baud: %s" % u1)
            print("UART2 baud: %s%s" % (
                u2, "" if u2 == 460800 else
                "   <-- supplier f9p_config.py and config/moving_base.txt both use 460800"))
            inp = [n for n, k in (("UBX", "CFG-UART2INPROT-UBX"),
                                  ("NMEA", "CFG-UART2INPROT-NMEA"),
                                  ("RTCM3X", "CFG-UART2INPROT-RTCM3X"))
                   if cfg.get(ubx.KEY[k])]
            outp = [n for n, k in (("UBX", "CFG-UART2OUTPROT-UBX"),
                                   ("NMEA", "CFG-UART2OUTPROT-NMEA"),
                                   ("RTCM3X", "CFG-UART2OUTPROT-RTCM3X"))
                    if cfg.get(ubx.KEY[k])]
            print("UART2 in:   %s" % (", ".join(inp) or "none"))
            print("UART2 out:  %s" % (", ".join(outp) or "none"))

        tm = ubx.valget(ser, [ubx.KEY["CFG-TMODE-MODE"],
                              ubx.KEY["CFG-NAVSPG-DYNMODEL"],
                              ubx.KEY["CFG-RATE-MEAS"]])
        if tm:
            mode = tm.get(ubx.KEY["CFG-TMODE-MODE"])
            mode_s = {0: "disabled (rover)", 1: "survey-in", 2: "fixed"}.get(mode, str(mode))
            dyn = tm.get(ubx.KEY["CFG-NAVSPG-DYNMODEL"])
            dyn_s = {0: "portable", 2: "stationary", 3: "pedestrian",
                     4: "automotive", 8: "airborne1g"}.get(dyn, str(dyn))
            meas = tm.get(ubx.KEY["CFG-RATE-MEAS"]) or 0
            print("TMODE:      %s" % mode_s)
            print("dynModel:   %s" % dyn_s)
            if meas:
                print("Rate:       %d ms (%.2f Hz)" % (meas, 1000.0 / meas))

        # --- per-port counters, sampled over the window --------------------
        io_before = ubx.poll(ser, 0x0A, 0x02)
        pp_before = ubx.poll(ser, 0x0A, 0x06)

        section("Traffic on this link (%.0f s)" % args.seconds)
        t0 = time.time()
        raw = ubx.read_for(ser, args.seconds)
        elapsed = time.time() - t0

        msgs, leftover = ubx.parse_stream(raw)
        rate = len(raw) / elapsed
        capacity = baud / 10.0  # 8N1
        print("Received:   %s in %.1f s = %.0f B/s" % (human(len(raw)), elapsed, rate))
        print("Link use:   %.1f%% of %d baud" % (100.0 * rate / capacity, baud))
        if leftover:
            print("Unparsed:   %d trailing bytes (partial frame, normal)" % len(leftover))

        kinds = Counter(m["kind"] for m in msgs)
        print("Frames:     " + (", ".join("%s=%d" % kv for kv in sorted(kinds.items()))
                                or "none"))

        ubx_mix = Counter("%02X-%02X" % (m["cls"], m["id"])
                          for m in msgs if m["kind"] == "ubx")
        if ubx_mix:
            named = {"01-07": "NAV-PVT", "01-3B": "NAV-SVIN", "01-35": "NAV-SAT",
                     "01-3C": "NAV-RELPOSNED", "01-03": "NAV-STATUS",
                     "0A-04": "MON-VER", "04-00": "INF-ERROR", "04-01": "INF-WARNING"}
            print("\nUBX message mix:")
            for k, c in ubx_mix.most_common():
                label = named.get(k, "")
                bytes_of = sum(m["raw_len"] for m in msgs
                               if m["kind"] == "ubx" and "%02X-%02X" % (m["cls"], m["id"]) == k)
                pct = 100.0 * bytes_of / max(1, len(raw))
                flag = ""
                if k.startswith(("27-", "00-", "08-", "0C-")) or bytes_of > len(raw) * 0.4:
                    flag = "   <-- undocumented/debug, consider a CFG-CFG factory reset"
                print("  %-8s %-14s %4d msgs  %8s  %5.1f%%%s"
                      % (k, label, c, human(bytes_of), pct, flag))

        rtcm_mix = Counter(m["type"] for m in msgs if m["kind"] == "rtcm3")
        if rtcm_mix:
            print("\nRTCM3 on this port:")
            for t, c in sorted(rtcm_mix.items()):
                print("  %-6d %3d msgs  (%.2f Hz)" % (t, c, c / elapsed))

        nmea_mix = Counter(m["talker"] for m in msgs if m["kind"] == "nmea")
        if nmea_mix:
            print("\nNMEA: " + ", ".join("%s=%d" % kv for kv in sorted(nmea_mix.items())))

        # --- gaps in the position solution ---------------------------------
        pvts = [ubx.decode_nav_pvt(m["payload"]) for m in msgs
                if m["kind"] == "ubx" and (m["cls"], m["id"]) == (0x01, 0x07)]
        pvts = [p for p in pvts if p]
        if len(pvts) > 2:
            itows = [p["itow"] for p in pvts]
            gaps = [(b - a) / 1000.0 for a, b in zip(itows, itows[1:])]
            worst = max(gaps)
            nominal = min(gaps)
            print("\nNAV-PVT:    %d msgs = %.2f Hz, largest gap %.2f s%s"
                  % (len(pvts), len(pvts) / elapsed, worst,
                     "   <-- DROPOUTS" if worst > nominal * 2.5 else ""))

        # --- port counters delta -------------------------------------------
        io_after = ubx.poll(ser, 0x0A, 0x02)
        if io_before and io_after:
            section("Per-port byte counters (delta over the window)")
            a = ubx.decode_mon_io(io_before)
            b = ubx.decode_mon_io(io_after)
            print("%-7s %10s %10s %8s %8s %8s"
                  % ("port", "rx", "tx", "parity", "framing", "overrun"))
            for pa, pb in zip(a, b):
                drx = pb["rx"] - pa["rx"]
                dtx = pb["tx"] - pa["tx"]
                note = ""
                if pa["port"] == "UART2":
                    if drx == 0 and dtx > 0:
                        note = "  <-- transmitting, receiving nothing"
                    elif drx > 0:
                        note = "  <-- receiving"
                if pb["framing_err"] > pa["framing_err"] or pb["parity_err"] > pa["parity_err"]:
                    note = "  <-- LINE ERRORS: baud mismatch or noise"
                print("%-7s %10d %10d %8d %8d %8d%s"
                      % (pa["port"], drx, dtx,
                         pb["parity_err"] - pa["parity_err"],
                         pb["framing_err"] - pa["framing_err"],
                         pb["overrun_err"] - pa["overrun_err"], note))
            print("\nNote: rx=0 with zero framing errors means the line is electrically")
            print("idle, i.e. nothing is driving it. Garbled wiring or a baud mismatch")
            print("would show errors instead.")

        pp_after = ubx.poll(ser, 0x0A, 0x06)
        if pp_before and pp_after:
            section("Messages parsed per port (delta)")
            a = ubx.decode_mon_msgpp(pp_before)
            b = ubx.decode_mon_msgpp(pp_after)
            print("%-7s %8s %8s %8s" % ("port", "UBX", "NMEA", "RTCM3"))
            for pa, pb in zip(a, b):
                row = [pb["counts"][k] - pa["counts"][k] for k in ("UBX", "NMEA", "RTCM3")]
                if any(row) or pa["port"] in ("UART1", "UART2", "USB"):
                    print("%-7s %8d %8d %8d" % (pa["port"], row[0], row[1], row[2]))

        # --- RF health ------------------------------------------------------
        rf = ubx.poll(ser, 0x0A, 0x38)
        if rf:
            section("RF / interference")
            for blk in ubx.decode_mon_rf(rf):
                band = "L1" if blk["block"] == 0 else "L2"
                verdict = ""
                if blk["agc"] > 6000:
                    verdict = "  <-- very high: antenna likely disconnected"
                elif blk["agc"] < 2500:
                    verdict = "  <-- very low: RF overload (433 radio too close?)"
                print("%s  AGC %5d/8191  noise %4d  jamInd %3d%s"
                      % (band, blk["agc"], blk["noise"], blk["jam_ind"], verdict))
            print("Healthy on these boards: AGC ~4200-4600, noise ~76-86.")

        # --- fix / RTK ------------------------------------------------------
        section("Position")
        pvt = pvts[-1] if pvts else None
        if pvt is None:
            p = ubx.poll(ser, 0x01, 0x07)
            pvt = ubx.decode_nav_pvt(p) if p else None
        if pvt is None:
            print("No NAV-PVT available.")
        else:
            print("fixType:    %d (%s)%s"
                  % (pvt["fix_type"], ubx.FIX_TYPES.get(pvt["fix_type"], "?"),
                     "   <-- base in TMODE, not a rover fix" if pvt["fix_type"] == 5 else ""))
            carr = ubx.CARR_SOLN.get(pvt["carr_soln"], "?")
            if pvt["carr_soln"]:
                note = " -- carrier phase corrections in use"
            elif pvt["diff_soln"]:
                note = " -- DGNSS only (code-phase, e.g. SBAS). No RTCM base."
            else:
                note = " -- standalone"
            print("RTK:        %s%s" % (carr, note))
            print("numSV used: %d" % pvt["num_sv"])
            print("lat/lon:    %.7f, %.7f" % (pvt["lat"], pvt["lon"]))
            print("height:     %.2f m" % pvt["height"])
            print("hAcc:       %.3f m   vAcc: %.3f m" % (pvt["h_acc"], pvt["v_acc"]))

        sat = ubx.poll(ser, 0x01, 0x35)
        if sat:
            sats = ubx.decode_nav_sat(sat)
            used = [s for s in sats if s["used"]]
            strong = [s for s in sats if s["cno"] >= 35]
            print("\nSatellites: %d tracked, %d used, %d at C/N0>=35"
                  % (len(sats), len(used), len(strong)))
            if sats and not used:
                print("  <-- tracking but using none. A stale fixed base position that")
                print("      disagrees with reality will do exactly this.")
            by = {}
            for s in sats:
                by.setdefault(s["gnss"], []).append(s["cno"])
            for g in sorted(by):
                c = by[g]
                print("  %-4s %2d sats, C/N0 max %2d avg %2d"
                      % (g, len(c), max(c), sum(c) // len(c)))

        svin = ubx.poll(ser, 0x01, 0x3B)
        if svin:
            s = ubx.decode_nav_svin(svin)
            if s and (s["active"] or s["valid"]):
                section("Survey-in")
                print("active: %s   valid: %s" % (s["active"], s["valid"]))
                print("elapsed: %d s   observations: %d" % (s["dur"], s["obs"]))
                print("mean accuracy: %.3f m" % s["mean_acc"])
                if s["active"]:
                    print("\nWhile survey-in is active the base emits observations but")
                    print("not RTCM 1005, so the rover cannot reach RTK FIXED yet.")
                    print("Indoors this will plateau around 20 m and never converge.")

    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
