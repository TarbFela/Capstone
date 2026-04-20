#!/usr/bin/env python3

"""

ENTIRELY AI-GENERATED USING CLAUDE SONNET 4.6

"""


"""
mcp_reader.py  --  MCP3562R USB data capture tool

Usage:
    python3 mcp_reader.py /dev/cu.usbmodem<XXXX>

Sends the appropriate prompts to the RP2350 firmware, captures the raw
binary stream between "STREAMING RAW DATA:" and "END RAW DATA STREAM",
and decodes it as signed 24-bit samples packed in 32-bit words.
"""

import sys
import struct
import time
import serial

# ── Configuration ────────────────────────────────────────────────────────────

BAUD            = 115200
TIMEOUT         = 10.0      # seconds to wait for expected responses
DMA_BUFF_SIZE   = 128       # must match firmware DMA_BUFF_SIZE
BYTES_PER_WORD  = 4
HALF_BUF_BYTES  = DMA_BUFF_SIZE * BYTES_PER_WORD   # 512 bytes per half-buffer

# MCP3562R 32-bit format: [CHID(4) | SGN(4) | DATA(24)]
# We sign-extend the lower 24 bits.
def sign_extend_24(value: int) -> int:
    value &= 0xFFFFFF
    if value & 0x800000:
        value -= 0x1000000
    return value

def decode_sample(word: int):
    """Returns (channel_id, signed_24bit_value) from a 32-bit MCP word."""
    chid  = (word >> 28) & 0xF
    data  = word & 0xFFFFFF
    return chid, sign_extend_24(data)

# ── Helpers ───────────────────────────────────────────────────────────────────

def send_prompt(port: serial.Serial, prompt: str = 'c'):
    """Send a single character prompt."""
    port.write(prompt.encode())
    port.flush()

def wait_for(port: serial.Serial, marker: str, timeout: float = TIMEOUT) -> str:
    """
    Read lines until one contains `marker`. Returns the matching line.
    Raises TimeoutError if not found within `timeout` seconds.
    Prints all lines seen (for visibility).
    """
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = port.read(port.in_waiting or 1)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="replace").strip()
                if text:
                    print(f"  [fw] {text}")
                if marker in text:
                    return text
    raise TimeoutError(f"Timed out waiting for: {repr(marker)}")

def read_raw_stream(port: serial.Serial) -> bytes:
    """
    Read raw binary data from the port until "END RAW DATA STREAM" is seen.
    The firmware interleaves text lines and binary blobs, so we accumulate
    everything and strip the sentinel line at the end.
    """
    raw = b""
    deadline = time.time() + TIMEOUT
    END_MARKER = b"END RAW DATA STREAM"

    print("  [capturing binary stream...]")
    while time.time() < deadline:
        chunk = port.read(port.in_waiting or 1)
        if chunk:
            raw += chunk
            deadline = time.time() + TIMEOUT  # reset on activity
            if END_MARKER in raw:
                # Split off everything before the marker
                data, _ = raw.split(END_MARKER, 1)
                # Strip any trailing newline/CR that precede the marker
                data = data.rstrip(b"\r\n")
                return data

    raise TimeoutError("Timed out waiting for END RAW DATA STREAM")

# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} /dev/cu.usbmodem<XXXX>")
        sys.exit(1)

    device_path = sys.argv[1]
    print(f"Opening {device_path} at {BAUD} baud...")

    with serial.Serial(device_path, BAUD, timeout=1) as port:
        time.sleep(0.5)  # let the port settle
        port.reset_input_buffer()

        # ── Step 1: initial prompt (firmware waits for any char) ────────────
        print("\n[1] Sending initial prompt...")
        send_prompt(port, prompt='c')
        wait_for(port, "input")
        print("input")
        wait_for(port, "initialized")
        print("    ADC initialized.")

        # ── Step 2: second prompt to start sampling ──────────────────────────
        print("\n[2] Sending start prompt...")
        send_prompt(port)
        wait_for(port, "input recieved")
        print("    Firmware acknowledged.")

        # ── Step 3: capture stream ───────────────────────────────────────────
        print("\n[3] Waiting for stream start...")
        wait_for(port, "STREAMING RAW DATA")
        print("    Stream started.")

        raw_bytes = read_raw_stream(port)

        # ── Step 4: decode ───────────────────────────────────────────────────
        n_bytes = len(raw_bytes)
        n_words = n_bytes // BYTES_PER_WORD
        leftover = n_bytes % BYTES_PER_WORD

        print(f"\n[4] Received {n_bytes} bytes ({n_words} samples, {leftover} leftover bytes)")

        if leftover:
            print(f"    WARNING: {leftover} bytes not aligned to uint32 boundary — truncating.")

        samples = struct.unpack_from(f"<{n_words}I", raw_bytes)

        print(f"\n{'Index':>6}  {'Raw (hex)':>12}  {'CH':>3}  {'Value':>12}")
        print("-" * 42)
        for i, word in enumerate(samples):
            ch, val = decode_sample(word)
            print(f"{i:>6}  {word:#012x}  {ch:>3}  {val:>12}")

        # ── Step 5: send 'q' to exit ─────────────────────────────────────────
        print("\n[5] Sending 'q' to reboot device...")
        # wait for the firmware's "Done." prompt first
        try:
            wait_for(port, "Done.", timeout=5.0)
        except TimeoutError:
            pass  # firmware may have already moved on
        send_prompt(port, 'q')
        try:
            wait_for(port, "REBOOT", timeout=3.0)
        except TimeoutError:
            pass
        print("    Done.")

if __name__ == "__main__":
    main()