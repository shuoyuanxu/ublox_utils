#!/usr/bin/env python3
"""Live rover monitor: is the 433 link delivering corrections, and is RTK engaging?

Shows the whole correction chain in one line per second:

  radio  -> bytes arriving on UART2 (the 433 link itself)
  RTCM   -> messages the receiver actually DECODED, from UBX-RXM-RTCM, with CRC
            failures counted. This is authoritative; MON-MSGPP's RTCM3 column
            reads zero even when valid messages are arriving.
  1005   -> whether the base coordinate has been received. Without it the rover
            cannot engage RTK at all, and a base still in survey-in never sends it.
  RTK    -> none / FLOAT / FIXED

    python3 tools/field_monitor.py                    # /dev/ublox_rover
    python3 tools/field_monitor.py --seconds 120

Reading it:
  radio 0 B/s            -> link down. Check CB1 joints, antennas, base power.
  radio >0, RTCM 0       -> bytes crossing but corrupted; check both UART2 bauds.
  RTCM ok, no 1005       -> base survey-in has not completed yet. Wait it out.
  1005 seen, RTK none    -> rover needs more satellites; give it sky view.
  FLOAT but never FIXED  -> expected without L2. Check the antenna is dual-band.
"""

import argparse
import sys
import time
from collections import Counter

import serial

import ubx

RXM_RTCM_UART1 = 0x20910269


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--device", default="/dev/ublox_rover")
    ap.add_argument("-b", "--baud", type=int, default=0)
    ap.add_argument("-s", "--seconds", type=float, default=0.0, help="0 = until Ctrl-C")
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
        elif ubx.detect_baud(ser, verbose=False) is None:
            print("No response at any baud on %s." % args.device)
            return 2

        # Make sure the receiver reports decoded RTCM. RAM-only: this is a
        # diagnostic aid, not a config change worth flashing.
        ubx.valset(ser, {RXM_RTCM_UART1: 1}, layers=ubx.LAYER_RAM)

        def uart2_rx():
            p = ubx.poll(ser, 0x0A, 0x02, timeout=1.5)
            if not p:
                return None
            for port in ubx.decode_mon_io(p):
                if port["port"] == "UART2":
                    return port["rx"]
            return None

        print("Monitoring %s. Ctrl-C to stop.\n" % args.device)
        print("%6s %10s %8s %7s %6s %-6s %5s %9s"
              % ("t", "radio B/s", "RTCM/s", "crcErr", "1005", "RTK", "sv", "hAcc"))
        print("-" * 68)

        seen_1005 = False
        total_rtcm = Counter()
        prev_rx = uart2_rx()
        buf = bytearray()
        t0 = time.time()
        last_tick = t0
        rtcm_this_sec = 0
        crc_this_sec = 0
        pvt = None

        try:
            while True:
                if args.seconds and time.time() - t0 >= args.seconds:
                    break
                c = ser.read(4096)
                if c:
                    buf += c
                    msgs, buf = ubx.parse_stream(buf)
                    for m in msgs:
                        if m["kind"] != "ubx":
                            continue
                        if (m["cls"], m["id"]) == (0x02, 0x32) and len(m["payload"]) >= 8:
                            mt = int.from_bytes(m["payload"][6:8], "little")
                            total_rtcm[mt] += 1
                            rtcm_this_sec += 1
                            if m["payload"][1] & 0x01:
                                crc_this_sec += 1
                            if mt == 1005:
                                seen_1005 = True
                        elif (m["cls"], m["id"]) == (0x01, 0x07):
                            d = ubx.decode_nav_pvt(m["payload"])
                            if d:
                                pvt = d
                else:
                    time.sleep(0.005)

                now = time.time()
                if now - last_tick >= 1.0:
                    rx = uart2_rx()
                    bps = "?" if (rx is None or prev_rx is None) else "%d" % (rx - prev_rx)
                    prev_rx = rx
                    print("%5.0fs %10s %8d %7d %6s %-6s %5s %9s"
                          % (now - t0, bps, rtcm_this_sec, crc_this_sec,
                             "yes" if seen_1005 else "NO",
                             ubx.CARR_SOLN.get(pvt["carr_soln"], "?") if pvt else "-",
                             pvt["num_sv"] if pvt else "-",
                             "%.3f m" % pvt["h_acc"] if pvt else "-"))
                    sys.stdout.flush()
                    rtcm_this_sec = crc_this_sec = 0
                    last_tick = now
        except KeyboardInterrupt:
            pass

        print("\nRTCM decoded over the session:")
        if total_rtcm:
            for t in sorted(total_rtcm):
                print("   type %-5d %4d" % (t, total_rtcm[t]))
            if not seen_1005:
                print("\n   No 1005: the base has not finished its survey, so the rover")
                print("   cannot engage RTK yet no matter how good the link is.")
        else:
            print("   none -- no corrections reached this receiver")
    return 0


if __name__ == "__main__":
    sys.exit(main())
