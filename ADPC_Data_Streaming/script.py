#!/usr/bin/env python3

"""

ALMOST ENTIRELY AI-GENERATED USING CLAUDE SONNET 4.6


"""

"""
adpc_shell.py  —  ADPC firmware shell + ADC data capture

Usage:
    python3 script.py [/dev/cu.usbmodem<XXXX>]

Transparent shell passthrough to the ADPC firmware with three additions:
  • rstream          — captures binary ADC stream; press Enter to stop
  • irun stream      — captures binary ADC stream during program run (auto-stops)
  • iprog <file.csv> [timestep_ms]
                     — auto-fills the iprog prompts from a CSV file.
                       CSV format: one setpoint per line (floats).
                       If timestep_ms is omitted, the first line of the CSV
                       is used as the timestep.
"""

import sys
import os
import struct
import time
import threading
import serial

# ── Configuration ─────────────────────────────────────────────────────────────

BAUD           = 115200
STREAM_TIMEOUT = 15.0   # seconds of serial silence before giving up on a stream
QUIET_S        = 0.35   # seconds of silence that signals a command has finished
BYTES_PER_WORD = 4
END_MARKER     = b"END RAW DATA STREAM"

# ── ADC decoding ──────────────────────────────────────────────────────────────

def sign_extend_24(v: int) -> int:
    v &= 0xFFFFFF
    return v - 0x1000000 if v & 0x800000 else v

def decode_word(word: int):
    """(chid, signed_value) from a 32-bit MCP word (CHID[3:0] + SGN[3:0] + DATA[23:0])."""
    return (word >> 28) & 0xF, sign_extend_24(word & 0xFFFFFF)

def parse_and_save(raw: bytes):
    """Decode a raw uint32 stream and write a timestamped CSV log file."""
    n_words  = len(raw) // BYTES_PER_WORD
    leftover = len(raw) % BYTES_PER_WORD
    if leftover:
        print(f"  WARNING: {leftover} trailing bytes dropped (not 32-bit aligned)")

    if n_words == 0:
        print("  No samples to save.")
        return

    words = struct.unpack_from(f"<{n_words}I", raw)
    data = {}
    for w in words:
        ch, val = decode_word(w)
        data.setdefault(str(ch), []).append(val)

    os.makedirs("logs", exist_ok=True)
    fname = "log [" + time.strftime("%Y-%m-%d-%H-%M-%S") + "].txt"
    fpath = os.path.join("logs", fname)
    keys  = list(data.keys())
    n     = max(len(v) for v in data.values())
    with open(fpath, "x") as f:
        f.write(", ".join(f"CH {k}" for k in keys) + "\n")
        for i in range(n):
            f.write(", ".join(str(data[k][i]) if i < len(data[k]) else "" for k in keys) + "\n")

    print(f"  Saved → {fpath}  ({n_words} words)")
    for k, v in sorted(data.items()):
        print(f"    Channel {k}: {len(v)} samples")

# ── Serial helpers ────────────────────────────────────────────────────────────

def fw_print(line: str):
    """Print one firmware text line, suppressing the shell prompt."""
    t = line.strip()
    if t and t not in (">>", ">"):
        print(f"  [fw] {t}")

def drain(port: serial.Serial, quiet_s: float = QUIET_S) -> bytes:
    """Read until quiet_s seconds of silence; print text lines; return raw bytes."""
    buf = b""
    deadline = time.time() + quiet_s
    while time.time() < deadline:
        n = port.in_waiting
        if n:
            buf += port.read(n)
            deadline = time.time() + quiet_s
        else:
            time.sleep(0.01)
    for line in buf.decode(errors="replace").splitlines():
        fw_print(line)
    return buf

def wait_for(port: serial.Serial, marker: str, timeout: float = STREAM_TIMEOUT) -> str:
    """Read and print text lines until one contains marker. Returns matching line."""
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        n = port.in_waiting
        if n:
            buf += port.read(n)
        else:
            time.sleep(0.01)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode(errors="replace").strip()
            fw_print(text)
            if marker in text:
                return text
    raise TimeoutError(f"Timed out waiting for: {marker!r}")

# ── Binary stream collectors ──────────────────────────────────────────────────

def _collect_until_end(port: serial.Serial, buf: bytearray, timeout: float) -> bytes:
    """Append bytes to buf until END_MARKER found; return bytes before marker."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        n = port.in_waiting
        if n:
            buf.extend(port.read(n))
            deadline = time.time() + timeout
            if END_MARKER in buf:
                return bytes(buf[:buf.find(END_MARKER)]).rstrip(b"\r\n")
        else:
            time.sleep(0.005)
    print("  WARNING: stream timed out without END marker — saving partial data")
    return bytes(buf)

def collect_auto(port: serial.Serial) -> bytes:
    """Collect binary until firmware sends END_MARKER (program-driven stop)."""
    return _collect_until_end(port, bytearray(), STREAM_TIMEOUT)

def collect_user_stop(port: serial.Serial) -> bytes:
    """
    Collect binary stream; sends stop char 'c' to firmware when user presses Enter,
    then collects the remainder until END_MARKER.
    """
    buf        = bytearray()
    stop_event = threading.Event()

    def _wait_enter():
        try:
            sys.stdin.readline()
        except Exception:
            pass
        stop_event.set()

    print("  [streaming — press Enter to stop]")
    threading.Thread(target=_wait_enter, daemon=True).start()

    # Phase 1: collect while firmware streams
    while not stop_event.is_set():
        n = port.in_waiting
        if n:
            buf.extend(port.read(n))
            if END_MARKER in buf:           # firmware stopped itself
                return bytes(buf[:buf.find(END_MARKER)]).rstrip(b"\r\n")
        else:
            time.sleep(0.005)

    # Phase 2: user pressed Enter — send stop char, wait for END
    port.write(b"c")
    port.flush()
    return _collect_until_end(port, buf, STREAM_TIMEOUT)

# ── CSV iprog loader ──────────────────────────────────────────────────────────

def handle_iprog_csv(port: serial.Serial, filename: str, timestep_ms):
    """Drive the firmware iprog dialogue automatically from a CSV file."""
    try:
        with open(filename) as f:
            rows = [r.strip() for r in f if r.strip()]
    except FileNotFoundError:
        print(f"  File not found: {filename}")
        return

    # If no timestep argument given, treat the first CSV line as the timestep
    if timestep_ms is None:
        try:
            timestep_ms = float(rows[0].replace(",", "").strip())
            rows = rows[1:]
        except (ValueError, IndexError):
            print(f"  Cannot parse timestep from first line: {rows[0] if rows else '(empty)'}")
            return

    # Parse setpoints: first comma-separated token per row, skip non-numeric / headers
    setpoints = []
    for row in rows:
        token = row.split(",")[0].strip()
        if token.upper() == "END":
            break
        try:
            setpoints.append(float(token))
        except ValueError:
            pass

    if not setpoints:
        print("  No setpoints found in file.")
        return

    print(f"  Loading {len(setpoints)} setpoints @ {timestep_ms} ms/step from {filename}")

    port.write(b"iprog\n")
    port.flush()
    wait_for(port, "Timestep (ms):")

    port.write(f"{timestep_ms}\n".encode())
    port.flush()

    for i, val in enumerate(setpoints):
        wait_for(port, f"setpoint #{i + 1}")
        port.write(f"{val}\n".encode())
        port.flush()

    port.write(b"END\n")
    port.flush()
    drain(port, quiet_s=0.5)

# ── Shell loop ────────────────────────────────────────────────────────────────

HELP = """
All input is forwarded verbatim to the ADPC firmware shell.
Python-side additions:
  rstream                          stream ADC data (press Enter to stop)
  irun stream                      run current program with streaming (auto-stops)
  iprog <file.csv> [timestep_ms]   load iprog setpoints from a CSV file
  q                                reboot device and exit
"""

def shell_loop(port: serial.Serial):
    print(HELP)

    while True:
        drain(port, quiet_s=0.05)       # flush any pending firmware output

        try:
            ui = input(">> ")
        except (EOFError, KeyboardInterrupt):
            print()
            break

        ui = ui.strip()
        if not ui:
            continue

        parts = ui.split()

        # ── Quit ──────────────────────────────────────────────────────────────
        if parts[0] == "q":
            port.write(b"q\n")
            port.flush()
            try:
                wait_for(port, "REBOOT", timeout=3.0)
            except TimeoutError:
                pass
            print("Done.")
            break

        # ── CSV iprog ─────────────────────────────────────────────────────────
        # Syntax: iprog <file.csv> [timestep_ms]
        # Falls through to passthrough if no filename is given (interactive mode).
        if parts[0] == "iprog" and len(parts) >= 2:
            csv_file = next((p for p in parts[1:] if "." in p), None)
            if csv_file:
                timestep = None
                for p in parts[1:]:
                    if p != csv_file:
                        try:
                            timestep = float(p)
                        except ValueError:
                            pass
                handle_iprog_csv(port, csv_file, timestep)
                continue
            # No filename → fall through to passthrough (interactive iprog)

        # ── rstream ───────────────────────────────────────────────────────────
        if parts[0] == "rstream":
            port.write(b"rstream\n")
            port.flush()
            try:
                wait_for(port, "STREAMING RAW DATA")
            except TimeoutError:
                print("  Timed out waiting for stream start.")
                continue
            raw = collect_user_stop(port)
            print(f"  Received {len(raw)} raw bytes.")
            parse_and_save(raw)
            drain(port, quiet_s=0.5)
            continue

        # ── irun stream ───────────────────────────────────────────────────────
        if parts[0] == "irun" and len(parts) > 1 and parts[1] == "stream":
            port.write((ui + "\n").encode())
            port.flush()
            try:
                # Print header / setpoint messages up to stream start
                wait_for(port, "STREAMING RAW DATA", timeout=60.0)
            except TimeoutError:
                print("  Timed out waiting for stream start.")
                continue
            raw = collect_auto(port)
            print(f"  Received {len(raw)} raw bytes.")
            parse_and_save(raw)
            drain(port, quiet_s=0.5)
            continue

        # ── Default passthrough ───────────────────────────────────────────────
        port.write((ui + "\n").encode())
        port.flush()
        drain(port, quiet_s=QUIET_S)


def main():
    # ── Device selection ──────────────────────────────────────────────────────
    if len(sys.argv) >= 2:
        device_path = sys.argv[1]
    else:
        device_path = None
        try:
            modems = [d for d in os.listdir("/dev/") if "cu.usbmodem" in d]
            if len(modems) == 1:
                device_path = "/dev/" + modems[0]
            elif len(modems) > 1:
                print("Multiple usbmodem devices found; please specify one:")
                for m in modems:
                    print(f"  /dev/{m}")
                sys.exit(1)
        except OSError:
            pass
        if device_path is None:
            print(f"Usage: {sys.argv[0]} /dev/cu.usbmodem<XXXX>")
            sys.exit(1)

    print(f"Opening {device_path} at {BAUD} baud...")

    with serial.Serial(device_path, BAUD, timeout=0.1) as port:
        time.sleep(0.5)
        port.reset_input_buffer()

        # Handshake — firmware waits for any single character before starting
        print("Sending handshake...")
        port.write(b"c")
        port.flush()
        time.sleep(0.3)
        port.reset_input_buffer()

        shell_loop(port)


if __name__ == "__main__":
    main()