#!/usr/bin/env python3
"""Find the baud that makes the 433 MHz link actually pass bytes.

Connect BOTH boards to this PC over USB, put them near each other with antennas
on, and run this. For each candidate baud it sets the base's UART2 to transmit
RTCM and the rover's UART2 to receive, waits, then reads the rover's MON-IO
counter to see whether any bytes physically arrived.

All writes are RAM-only, so nothing is made permanent and a power cycle undoes
everything. Re-run the winner with --commit to flash it.

    python3 tools/link_sweep.py --base /dev/ublox_base --rover /dev/ublox_rover
    python3 tools/link_sweep.py --base ... --rover ... --commit 9600

The radio's own UART baud is what matters here, NOT the 460800 in the supplier's
config -- that value is for their wired moving-base setup, which has no radio in
the path. An HC-12 ships at 9600 by default; an HM-TRP at 9600 or 38400.

Reading the result:
  rx > 0   bytes crossed the link. That baud works.
  rx = 0 and framing errors = 0   nothing is driving the rover's RX pin at all.
  rx = 0 but framing errors > 0   something IS driving it, at the wrong speed.
"""

import argparse
import sys
import time

import serial

import ubx

CANDIDATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800]

MSM4 = ("1074", "1084", "1094", "1124")


def base_items(baud2):
    items = {ubx.KEY["CFG-UART2-BAUDRATE"]: baud2,
             ubx.KEY["CFG-UART2OUTPROT-RTCM3X"]: 1,
             ubx.KEY["CFG-UART2OUTPROT-UBX"]: 0,
             ubx.KEY["CFG-UART2OUTPROT-NMEA"]: 0}
    for m in MSM4:
        items[ubx.RTCM_UART2[m]] = 1
    items[ubx.RTCM_UART2["1005"]] = 1
    return items


def rover_items(baud2):
    return {ubx.KEY["CFG-UART2-BAUDRATE"]: baud2,
            ubx.KEY["CFG-UART2INPROT-RTCM3X"]: 1,
            ubx.KEY["CFG-UART2INPROT-UBX"]: 0,
            ubx.KEY["CFG-UART2INPROT-NMEA"]: 0,
            # Half-duplex: the rover must stay silent or it collides.
            ubx.KEY["CFG-UART2OUTPROT-RTCM3X"]: 0,
            ubx.KEY["CFG-UART2OUTPROT-UBX"]: 0,
            ubx.KEY["CFG-UART2OUTPROT-NMEA"]: 0}


def uart2_counters(ser):
    p = ubx.poll(ser, 0x0A, 0x02)
    if not p:
        return None
    for port in ubx.decode_mon_io(p):
        if port["port"] == "UART2":
            return port
    return None


def uart2_rtcm_count(ser):
    p = ubx.poll(ser, 0x0A, 0x06)
    if not p:
        return None
    for port in ubx.decode_mon_msgpp(p):
        if port["port"] == "UART2":
            return port["counts"]["RTCM3"]
    return None


def open_and_sync(dev, forced):
    ser = serial.Serial(dev, forced or 38400, timeout=0.1)
    if forced:
        ser.baudrate = forced
        return ser, forced
    b = ubx.detect_baud(ser, verbose=False)
    if b is None:
        ser.close()
        return None, None
    return ser, b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", required=True)
    ap.add_argument("--rover", required=True)
    ap.add_argument("--base-baud", type=int, default=0, help="UART1 baud, 0=autodetect")
    ap.add_argument("--rover-baud", type=int, default=0)
    ap.add_argument("--dwell", type=float, default=6.0,
                    help="seconds to listen at each candidate")
    ap.add_argument("--commit", type=int, default=0,
                    help="skip the sweep and FLASH this baud on both boards")
    args = ap.parse_args()

    if args.base == args.rover:
        print("Base and rover must be different devices.")
        return 2

    base, bb = open_and_sync(args.base, args.base_baud)
    if base is None:
        print("Base %s is silent at every baud." % args.base)
        return 1
    rover, rb = open_and_sync(args.rover, args.rover_baud)
    if rover is None:
        base.close()
        print("Rover %s is silent at every baud." % args.rover)
        return 1

    with base, rover:
        print("Base  %s @ %d" % (args.base, bb))
        print("Rover %s @ %d" % (args.rover, rb))

        if args.commit:
            print("\nFlashing UART2 = %d on both boards (RAM + Flash)..." % args.commit)
            layers = ubx.LAYER_RAM | ubx.LAYER_FLASH
            ok_b = ubx.valset(base, base_items(args.commit), layers=layers)
            ok_r = ubx.valset(rover, rover_items(args.commit), layers=layers)
            print("  base:  %s" % ("ACK" if ok_b else "FAILED"))
            print("  rover: %s" % ("ACK" if ok_r else "FAILED"))
            return 0 if (ok_b and ok_r) else 1

        print("\nSweeping. All writes are RAM-only; power cycle undoes them.")
        print("Make sure both antennas are attached and the boards are close.\n")
        print("%-9s %10s %10s %9s %9s %9s   %s"
              % ("UART2", "base tx", "rover rx", "RTCM3", "framing", "parity", "result"))
        print("-" * 82)

        results = []
        for baud in CANDIDATES:
            ubx.valset(base, base_items(baud), layers=ubx.LAYER_RAM)
            ubx.valset(rover, rover_items(baud), layers=ubx.LAYER_RAM)
            time.sleep(1.0)

            b0 = uart2_counters(base)
            r0 = uart2_counters(rover)
            m0 = uart2_rtcm_count(rover)
            if b0 is None or r0 is None or m0 is None:
                print("%-9d %10s %10s %9s %9s %9s   counters unavailable"
                      % (baud, "?", "?", "?", "?", "?"))
                continue

            time.sleep(args.dwell)

            b1 = uart2_counters(base)
            r1 = uart2_counters(rover)
            m1 = uart2_rtcm_count(rover)
            if b1 is None or r1 is None or m1 is None:
                continue

            tx = b1["tx"] - b0["tx"]
            rx = r1["rx"] - r0["rx"]
            rtcm = m1 - m0
            fr = r1["framing_err"] - r0["framing_err"]
            pa = r1["parity_err"] - r0["parity_err"]

            if rtcm > 0:
                verdict = "*** VALID RTCM CROSSED ***"
            elif rx > 0:
                verdict = "bytes crossed, but RTCM is invalid"
            elif fr or pa:
                verdict = "signal present, wrong speed"
            elif tx == 0:
                verdict = "base sent nothing - check base config"
            else:
                verdict = "silent - nothing driving rover RX"
            results.append((baud, tx, rx, rtcm, fr, pa, verdict))
            print("%-9d %10d %10d %9d %9d %9d   %s"
                  % (baud, tx, rx, rtcm, fr, pa, verdict))
            sys.stdout.flush()

        print()
        winners = [r for r in results if r[3] > 0]
        invalid = [r for r in results if r[2] > 0 and r[3] == 0]
        noisy = [r for r in results if r[2] == 0 and (r[4] or r[5])]

        if winners:
            best = max(winners, key=lambda r: (r[3], r[2]))
            print("WORKS at %d baud (%d valid RTCM frames in %.0f s)."
                  % (best[0], best[3], args.dwell))
            print("Make it permanent:")
            print("  python3 tools/link_sweep.py --base %s --rover %s --commit %d"
                  % (args.base, args.rover, best[0]))
        elif invalid:
            print("Bytes crossed at %s, but the rover decoded zero valid RTCM frames."
                  % ", ".join(str(r[0]) for r in invalid))
            print("The radio path is connected, but the radio pair is altering or")
            print("corrupting the byte stream. Match/reset the radio modules' own")
            print("air-rate, channel, addressing, encryption and packet settings.")
        elif noisy:
            print("No baud passed clean bytes, but the rover's RX pin IS being driven")
            print("(framing errors at %s)." % ", ".join(str(r[0]) for r in noisy))
            print("The radio link is alive. The radio's own UART baud is fixed and is")
            print("not in the list above -- reconfigure the module itself, or try its")
            print("config mode.")
        else:
            print("Nothing at any baud, and zero framing errors throughout.")
            print("The rover's UART2 RX pin is electrically idle: no radio is connected")
            print("to it. No baud setting can fix that. Options:")
            print("  1. Check SW1 is in the same position on BOTH boards.")
            print("  2. Confirm which of CB1/CB2/CB3 is bridged, on BOTH boards.")
            print("  3. Wire an external telemetry radio pair to the UART2 pins.")
        print("\nNothing was flashed. Power cycle both boards to restore them.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
