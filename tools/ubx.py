"""Shared UBX/NMEA/RTCM3 helpers for the ZED-F9P tooling in this package.

Deliberately dependency-light: pyserial only, Python 3.6+.

Notes that cost us time before, kept here so the next reader gets them for free:

  * Never hold two file descriptors on the same tty. termios is per-device, not
    per-fd, so a second open() at a different baud silently reconfigures the
    first. Probe bauds by assigning to Serial.baudrate on ONE handle instead.
  * A factory-reset F9P speaks NMEA only. Baud detection that scores UBX frames
    alone will report a perfectly healthy receiver as dead.
  * Payloads contain stray 0x24 ('$') and 0xD3 bytes. Sync on a full frame with
    a verified checksum/CRC, never on the sync byte alone.
"""

import time

# ---------------------------------------------------------------- config keys

# Ports are indexed the same way in MON-IO and MON-MSGPP.
PORTS = ["I2C", "UART1", "UART2", "USB", "SPI"]

KEY = {
    "CFG-UART1-BAUDRATE": 0x40520001,
    "CFG-UART2-BAUDRATE": 0x40530001,
    "CFG-UART1INPROT-UBX": 0x10730001,
    "CFG-UART1INPROT-NMEA": 0x10730002,
    "CFG-UART1INPROT-RTCM3X": 0x10730004,
    "CFG-UART1OUTPROT-UBX": 0x10740001,
    "CFG-UART1OUTPROT-NMEA": 0x10740002,
    "CFG-UART1OUTPROT-RTCM3X": 0x10740004,
    "CFG-UART2INPROT-UBX": 0x10750001,
    "CFG-UART2INPROT-NMEA": 0x10750002,
    "CFG-UART2INPROT-RTCM3X": 0x10750004,
    "CFG-UART2OUTPROT-UBX": 0x10760001,
    "CFG-UART2OUTPROT-NMEA": 0x10760002,
    "CFG-UART2OUTPROT-RTCM3X": 0x10760004,
    "CFG-NAVSPG-DYNMODEL": 0x20110021,
    "CFG-RATE-MEAS": 0x30210001,
    "CFG-RATE-NAV": 0x30210002,
    "CFG-TMODE-MODE": 0x20030001,
    "CFG-TMODE-SVIN_MIN_DUR": 0x40030010,
    "CFG-TMODE-SVIN_ACC_LIMIT": 0x40030011,
    "CFG-TMODE-ECEF_X": 0x40030003,
    "CFG-TMODE-ECEF_Y": 0x40030004,
    "CFG-TMODE-ECEF_Z": 0x40030005,
}

# RTCM output on UART2, as used by both the supplier script and ETH's
# config/moving_base.txt.
RTCM_UART2 = {
    "1005": 0x209102BF,  # station ARP -- static base only, absent from the supplier config
    "1074": 0x20910360,
    "1084": 0x20910365,
    "1094": 0x2091036A,
    "1124": 0x2091036F,
    "1230": 0x20910305,
    "4072_0": 0x20910300,  # u-blox proprietary -- moving base only
}

# Storage layers for CFG-VALSET.
LAYER_RAM = 0x01
LAYER_BBR = 0x02
LAYER_FLASH = 0x04
LAYER_ALL = LAYER_RAM | LAYER_BBR | LAYER_FLASH

BAUD_CANDIDATES = [460800, 230400, 115200, 57600, 38400, 19200, 9600]


# ------------------------------------------------------------------- checksum

def _ck(body):
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return a, b


def frame(cls, mid, payload=b""):
    """Build a complete UBX frame."""
    body = bytes([cls, mid, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload
    a, b = _ck(body)
    return b"\xb5\x62" + body + bytes([a, b])


# ------------------------------------------------------------------- RTCM CRC

_CRC24_POLY = 0x1864CFB


def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= _CRC24_POLY
    return crc & 0xFFFFFF


# -------------------------------------------------------------------- parsing

def parse_stream(buf):
    """Split a byte buffer into verified messages.

    Returns (messages, remainder). Each message is a dict with a 'kind' of
    'ubx', 'nmea' or 'rtcm3', and a 'raw' holding the exact on-wire bytes so
    callers can forward a frame verbatim. Anything that fails verification is
    skipped one byte at a time, so stray sync bytes inside payloads cannot
    desynchronise us.
    """
    msgs = []
    i = 0
    n = len(buf)
    while i < n:
        b = buf[i]

        if b == 0xB5:
            if i + 1 >= n:
                break
            if buf[i + 1] != 0x62:
                i += 1
                continue
            if i + 6 > n:
                break
            ln = buf[i + 4] | (buf[i + 5] << 8)
            end = i + 6 + ln + 2
            if end > n:
                break
            a, bb = _ck(buf[i + 2:i + 6 + ln])
            if a == buf[end - 2] and bb == buf[end - 1]:
                msgs.append({
                    "kind": "ubx",
                    "cls": buf[i + 2],
                    "id": buf[i + 3],
                    "payload": bytes(buf[i + 6:i + 6 + ln]),
                    "raw": bytes(buf[i:end]),
                    "raw_len": end - i,
                })
                i = end
                continue
            i += 1
            continue

        if b == 0xD3:
            if i + 3 > n:
                break
            ln = ((buf[i + 1] & 0x03) << 8) | buf[i + 2]
            end = i + 3 + ln + 3
            if end > n:
                break
            got = (buf[end - 3] << 16) | (buf[end - 2] << 8) | buf[end - 1]
            if got == crc24q(buf[i:end - 3]) and ln >= 2:
                mtype = (buf[i + 3] << 4) | (buf[i + 4] >> 4)
                msgs.append({
                    "kind": "rtcm3",
                    "type": mtype,
                    "raw": bytes(buf[i:end]),
                    "raw_len": end - i,
                })
                i = end
                continue
            i += 1
            continue

        if b == 0x24:  # '$'
            j = buf.find(b"\n", i, min(n, i + 128))
            if j < 0:
                if n - i > 128:
                    i += 1
                    continue
                break
            line = bytes(buf[i:j + 1])
            star = line.rfind(b"*")
            ok = False
            if star > 0 and star + 3 <= len(line):
                cs = 0
                for x in line[1:star]:
                    cs ^= x
                try:
                    ok = cs == int(line[star + 1:star + 3], 16)
                except ValueError:
                    ok = False
            if ok:
                msgs.append({
                    "kind": "nmea",
                    "talker": line[1:6].decode("ascii", "replace"),
                    "line": line,
                    "raw": line,
                    "raw_len": len(line),
                })
                i = j + 1
                continue
            i += 1
            continue

        i += 1

    return msgs, buf[i:]


# -------------------------------------------------------------- port handling

def read_for(ser, seconds):
    """Read raw bytes for a wall-clock duration."""
    out = bytearray()
    deadline = time.time() + seconds
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            out += chunk
        else:
            time.sleep(0.005)
    return out


def detect_baud(ser, candidates=None, dwell=1.2, verbose=True):
    """Find the receiver's baud on an already-open handle.

    Scores UBX frames, NMEA sentences and RTCM3 frames together, because a
    factory-reset receiver emits NMEA only and a base's UART2 emits RTCM only.
    Mutates ser.baudrate; leaves it at the winner.
    """
    if candidates is None:
        candidates = BAUD_CANDIDATES
    best, best_score = None, 0
    for baud in candidates:
        ser.baudrate = baud
        ser.reset_input_buffer()
        time.sleep(0.15)
        raw = read_for(ser, dwell)
        msgs, _ = parse_stream(raw)
        score = len(msgs)
        if verbose:
            kinds = {}
            for m in msgs:
                kinds[m["kind"]] = kinds.get(m["kind"], 0) + 1
            desc = ", ".join("%s=%d" % kv for kv in sorted(kinds.items())) or "nothing"
            print("  %7d baud: %5d bytes, %s" % (baud, len(raw), desc))
        if score > best_score:
            best, best_score = baud, score
    if best is None:
        ser.baudrate = candidates[0]
        return None
    ser.baudrate = best
    ser.reset_input_buffer()
    return best


def poll(ser, cls, mid, payload=b"", timeout=2.0, want=None):
    """Send a poll and wait for the matching response.

    'want' defaults to (cls, mid). Returns the payload, or None on timeout.
    """
    if want is None:
        want = (cls, mid)
    ser.reset_input_buffer()
    ser.write(frame(cls, mid, payload))
    ser.flush()
    buf = bytearray()
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
            msgs, buf = parse_stream(buf)
            for m in msgs:
                if m["kind"] == "ubx" and (m["cls"], m["id"]) == want:
                    return m["payload"]
        else:
            time.sleep(0.005)
    return None


def wait_ack(ser, cls, mid, timeout=2.0):
    """Wait for UBX-ACK-ACK / ACK-NAK for a given class+id. True if ACKed."""
    buf = bytearray()
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
            msgs, buf = parse_stream(buf)
            for m in msgs:
                if m["kind"] != "ubx" or m["cls"] != 0x05:
                    continue
                if len(m["payload"]) >= 2 and tuple(m["payload"][:2]) == (cls, mid):
                    return m["id"] == 0x01
        else:
            time.sleep(0.005)
    return None


# ------------------------------------------------------------ CFG-VALSET/GET

_TYPE_LEN = {0x1: 1, 0x2: 1, 0x3: 2, 0x4: 4, 0x5: 8}


def key_len(key):
    """Bytes of value storage implied by a configuration key's type nibble."""
    return _TYPE_LEN[(key >> 28) & 0x7]


def valset(ser, items, layers=LAYER_RAM | LAYER_FLASH, timeout=2.0):
    """Apply {key: value} via UBX-CFG-VALSET. Returns True/False/None(=timeout)."""
    payload = bytearray([0x00, layers, 0x00, 0x00])
    for key, val in items.items():
        payload += key.to_bytes(4, "little")
        payload += int(val).to_bytes(key_len(key), "little", signed=int(val) < 0)
    ser.write(frame(0x06, 0x8A, bytes(payload)))
    ser.flush()
    return wait_ack(ser, 0x06, 0x8A, timeout)


def valget(ser, keys, layer=0, timeout=2.0):
    """Read configuration keys. layer 0=RAM, 1=BBR, 2=Flash, 7=Default.

    Returns {key: int}; keys the receiver declines to answer are absent.
    """
    payload = bytearray([0x00, layer, 0x00, 0x00])
    for key in keys:
        payload += key.to_bytes(4, "little")
    resp = poll(ser, 0x06, 0x8B, bytes(payload), timeout=timeout, want=(0x06, 0x8B))
    if resp is None or len(resp) < 4:
        return {}
    out = {}
    i = 4
    while i + 4 <= len(resp):
        key = int.from_bytes(resp[i:i + 4], "little")
        i += 4
        n = key_len(key)
        if i + n > len(resp):
            break
        out[key] = int.from_bytes(resp[i:i + n], "little")
        i += n
    return out


# ------------------------------------------------------------ message decoders

def decode_mon_io(payload):
    """Per-port byte counters and line errors. This is the only honest way to
    see UART2 traffic while connected over UART1/USB."""
    out = []
    for p in range(len(payload) // 20):
        o = p * 20
        out.append({
            "port": PORTS[p] if p < len(PORTS) else "port%d" % p,
            "rx": int.from_bytes(payload[o:o + 4], "little"),
            "tx": int.from_bytes(payload[o + 4:o + 8], "little"),
            "parity_err": int.from_bytes(payload[o + 8:o + 10], "little"),
            "framing_err": int.from_bytes(payload[o + 10:o + 12], "little"),
            "overrun_err": int.from_bytes(payload[o + 12:o + 14], "little"),
            "break_cond": int.from_bytes(payload[o + 14:o + 16], "little"),
        })
    return out


def decode_mon_msgpp(payload):
    """Messages parsed per port, per protocol. Index 3 is RTCM3."""
    protos = ["UBX", "NMEA", "RTCM2", "RTCM3", "p4", "p5", "p6", "other"]
    out = []
    for p in range(6):
        counts = {}
        for k in range(8):
            o = p * 16 + k * 2
            counts[protos[k]] = int.from_bytes(payload[o:o + 2], "little")
        out.append({"port": PORTS[p] if p < len(PORTS) else "port%d" % p, "counts": counts})
    return out


def decode_mon_rf(payload):
    """AGC and jamming indicators. Healthy AGC on these boards is ~4200-4600;
    a disconnected antenna reads HIGH (~6600), RF overload reads LOW (~1400)."""
    if len(payload) < 4:
        return []
    nblocks = payload[1]
    out = []
    for b in range(nblocks):
        o = 4 + b * 24
        if o + 24 > len(payload):
            break
        # Block layout: blockId, flags, antStatus, antPower, postStatus(U4),
        # reserved(4), noisePerMS(U2)@12, agcCnt(U2)@14, jamInd@16.
        out.append({
            "block": payload[o],
            "ant_status": payload[o + 2],
            "ant_power": payload[o + 3],
            "noise": int.from_bytes(payload[o + 12:o + 14], "little"),
            "agc": int.from_bytes(payload[o + 14:o + 16], "little"),
            "jam_ind": payload[o + 16],
        })
    return out


FIX_TYPES = {0: "no fix", 1: "dead reckoning", 2: "2D", 3: "3D",
             4: "GNSS+DR", 5: "time only"}
CARR_SOLN = {0: "none", 1: "FLOAT", 2: "FIXED"}


def decode_nav_pvt(payload):
    if len(payload) < 92:
        return None
    flags = payload[21]
    return {
        "itow": int.from_bytes(payload[0:4], "little"),
        "fix_type": payload[20],
        "fix_ok": bool(flags & 0x01),
        "diff_soln": bool(flags & 0x02),
        "carr_soln": (flags >> 6) & 0x03,
        "num_sv": payload[23],
        "lon": int.from_bytes(payload[24:28], "little", signed=True) * 1e-7,
        "lat": int.from_bytes(payload[28:32], "little", signed=True) * 1e-7,
        "height": int.from_bytes(payload[32:36], "little", signed=True) / 1000.0,
        "h_acc": int.from_bytes(payload[40:44], "little") / 1000.0,
        "v_acc": int.from_bytes(payload[44:48], "little") / 1000.0,
    }


def decode_nav_svin(payload):
    if len(payload) < 40:
        return None
    return {
        "dur": int.from_bytes(payload[8:12], "little"),
        "mean_acc": int.from_bytes(payload[28:32], "little") / 10000.0,
        "obs": int.from_bytes(payload[32:36], "little"),
        "valid": bool(payload[36]),
        "active": bool(payload[37]),
    }


_GNSS = {0: "GPS", 1: "SBAS", 2: "GAL", 3: "BDS", 4: "IMES", 5: "QZSS", 6: "GLO"}


def decode_nav_sat(payload):
    """Tracked satellites. numSV in NAV-PVT counts satellites USED, not tracked
    -- always cross-check here before concluding the antenna is bad."""
    if len(payload) < 8:
        return []
    n = payload[1]
    out = []
    for s in range(n):
        o = 8 + s * 12
        if o + 12 > len(payload):
            break
        flags = int.from_bytes(payload[o + 8:o + 12], "little")
        out.append({
            "gnss": _GNSS.get(payload[o], str(payload[o])),
            "sv": payload[o + 1],
            "cno": payload[o + 2],
            "used": bool(flags & 0x08),
        })
    return out
