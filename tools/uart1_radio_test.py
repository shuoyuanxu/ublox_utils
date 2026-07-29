#!/usr/bin/env python3
"""Test whether the 433 MHz radio is wired to the F9P's UART1 rather than UART2.

Background: every earlier test assumed UART2, because that is what the supplier's
f9p_config.py and ETH's config/moving_base.txt use. Both of those describe a
*wired* moving-base pair with no radio in the path, so they are the wrong
reference for this board. A base running headless on a power bank with nothing
but a 433 antenna must send its RTCM out of a UART that reaches the radio.

Two modes:

  --led       Turn on RTCM output on UART1 and hold it, so you can watch LD1
              (the red LED beside the radio module). Continuous flashing means
              the radio is fed from UART1. Needs only the base connected.

  --pair      Base transmits RTCM on UART1 while the rover's UART1 receive
              counter is watched. Needs both boards connected.

All writes are RAM-only. Power cycle restores everything.

    python3 tools/uart1_radio_test.py --led --base /dev/ublox_base
    python3 tools/uart1_radio_test.py --pair --base /dev/ublox_base --rover /dev/ublox_rover

Caveat for --pair: while USB is plugged in, the FTDI also drives the F9P's UART1
RX line, so the radio and the FTDI may contend. Framing errors in the result are
therefore still a POSITIVE signal -- they mean something is driving the pin.
"""

import argparse
import sys
import time

import serial

import ubx

MSM4 = ("1074", "1084", "1094", "1124")

# CFG-MSGOUT keys are I2C, UART1, UART2, USB, SPI at consecutive ids, so the
# UART1 variant is the UART2 key minus one.
UART1_OFF = -1


def base_rtcm_on_uart1(enable):
    items = {ubx.KEY["CFG-UART1OUTPROT-RTCM3X"]: 1 if enable else 0}
    for m in MSM4:
        items[ubx.RTCM_UART2[m] + UART1_OFF] = 1 if enable else 0
    items[ubx.RTCM_UART2["1005"] + UART1_OFF] = 1 if enable else 0
    return items


def uart_counters(ser, name="UART1"):
    p = ubx.poll(ser, 0x0A, 0x02)
    if not p:
        return None
    for port in ubx.decode_mon_io(p):
        if port["port"] == name:
            return port
    return None


def open_sync(dev, forced):
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
    ap.add_argument("--rover")
    ap.add_argument("--base-baud", type=int, default=0)
    ap.add_argument("--rover-baud", type=int, default=0)
    ap.add_argument("--led", action="store_true", help="LED-watch mode")
    ap.add_argument("--pair", action="store_true", help="two-board counter mode")
    ap.add_argument("--hold", type=float, default=45.0, help="--led hold time")
    ap.add_argument("--dwell", type=float, default=10.0, help="--pair measure time")
    args = ap.parse_args()

    if not (args.led or args.pair):
        print("Pick --led or --pair.")
        return 2
    if args.pair and not args.rover:
        print("--pair needs --rover as well.")
        return 2

    base, bb = open_sync(args.base, args.base_baud)
    if base is None:
        print("Base %s is silent at every baud." % args.base)
        return 1

    with base:
        print("Base %s @ %d baud" % (args.base, bb))

        if args.led:
            b0 = uart_counters(base)
            print("\nEnabling RTCM 1005 + MSM4 on the base's UART1 (RAM only)...")
            ok = ubx.valset(base, base_rtcm_on_uart1(True), layers=ubx.LAYER_RAM)
            print("  %s" % ("ACK" if ok else "FAILED -- receiver rejected it"))
            if not ok:
                return 1

            print("\n" + "=" * 60)
            print("WATCH LD1 -- the red LED beside the 433 module -- FOR %.0f SECONDS"
                  % args.hold)
            print("=" * 60)
            print("  flashing continuously  ->  the radio IS fed from UART1")
            print("  dark the whole time    ->  UART1 is not the radio port either")
            print()

            t0 = time.time()
            while time.time() - t0 < args.hold:
                time.sleep(2.0)
                raw = ubx.read_for(base, 2.0)
                msgs, _ = ubx.parse_stream(raw)
                n = sum(1 for m in msgs if m["kind"] == "rtcm3")
                types = sorted(set(m["type"] for m in msgs if m["kind"] == "rtcm3"))
                print("  [%3.0fs] base UART1 emitting %d RTCM frames %s"
                      % (time.time() - t0, n, types if types else ""))
                sys.stdout.flush()

            b1 = uart_counters(base)
            if b0 and b1:
                print("\n  base UART1 tx over the test: %d bytes" % (b1["tx"] - b0["tx"]))

            print("\nRestoring (RTCM off UART1)...")
            ubx.valset(base, base_rtcm_on_uart1(False), layers=ubx.LAYER_RAM)
            print("Done. Nothing was flashed.")
            return 0

        # --- pair mode
        rover, rb = open_sync(args.rover, args.rover_baud)
        if rover is None:
            print("Rover %s is silent at every baud." % args.rover)
            return 1

        with rover:
            print("Rover %s @ %d baud" % (args.rover, rb))
            if bb != rb:
                print("\nNOTE: UART1 bauds differ (%d vs %d). A transparent radio would"
                      % (bb, rb))
                print("deliver base-rate bytes into a rover expecting a different rate,")
                print("so expect FRAMING ERRORS rather than clean bytes. Errors still")
                print("prove a connection exists.")

            ubx.valset(rover, {ubx.KEY["CFG-UART1INPROT-RTCM3X"]: 1},
                       layers=ubx.LAYER_RAM)

            print("\n--- baseline: base NOT sending RTCM on UART1 ---")
            ubx.valset(base, base_rtcm_on_uart1(False), layers=ubx.LAYER_RAM)
            time.sleep(1.0)
            r0 = uart_counters(rover)
            time.sleep(args.dwell)
            r1 = uart_counters(rover)
            base_rx = r1["rx"] - r0["rx"]
            base_fr = r1["framing_err"] - r0["framing_err"]
            print("  rover UART1: rx %d, framing %d" % (base_rx, base_fr))

            print("\n--- test: base SENDING RTCM on UART1 ---")
            ubx.valset(base, base_rtcm_on_uart1(True), layers=ubx.LAYER_RAM)
            time.sleep(1.0)
            b0 = uart_counters(base)
            r0 = uart_counters(rover)
            time.sleep(args.dwell)
            b1 = uart_counters(base)
            r1 = uart_counters(rover)
            tx = b1["tx"] - b0["tx"]
            rx = r1["rx"] - r0["rx"]
            fr = r1["framing_err"] - r0["framing_err"]
            print("  base  UART1: tx %d" % tx)
            print("  rover UART1: rx %d, framing %d" % (rx, fr))

            print("\n" + "=" * 60)
            delta_rx = rx - base_rx
            delta_fr = fr - base_fr
            if delta_rx > 200:
                print("BYTES CROSSED. The radio is wired to UART1.")
                print("Reconfigure both boards to use UART1 for corrections.")
            elif delta_fr > 0:
                print("FRAMING ERRORS APPEARED (+%d). Something IS driving the rover's"
                      % delta_fr)
                print("UART1 RX pin. The radio is connected -- match the bauds and retry.")
            else:
                print("No change from baseline. UART1 is not the radio path either.")
                print("Both UARTs are now eliminated; this is a hardware question.")
            print("=" * 60)

            print("\nRestoring...")
            ubx.valset(base, base_rtcm_on_uart1(False), layers=ubx.LAYER_RAM)
            print("Done. Nothing was flashed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
