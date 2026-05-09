#!/usr/bin/env python3

"""

ALMOST ENTIRELY AI-GENERATED USING CLAUDE SONNET 4.6


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
import os

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

def send_char(port: serial.Serial, ch: str):
    """Send a single character (no newline — firmware uses getchar_timeout_us)."""
    port.write(ch[0].encode())
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
    The firmware mutes stdio_usb during the blast, then restores it, so
    everything between the markers is pure binary.
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
                data, _ = raw.split(END_MARKER, 1)
                data = data.rstrip(b"\r\n")
                return data

    raise TimeoutError("Timed out waiting for END RAW DATA STREAM")

# ── Main ─────────────────────────────────────────────────────────────────────

HELP = """
Commands:
  r        stream ADC data and save log
  e / d    enable / disable H-bridge
  0-9      set PWM level (digit × 15 offset)
  p / l    nudge PWM level +5 / -5
  [space]  toggle PWM on/off
  q        reboot device and exit
"""

def main():
    if len(sys.argv) < 2:
        try:
            devs = os.listdir("/dev/")
            sel = 0
            for dev in devs:
                if "cu.usbmodem" in dev:
                    if sel != 0:
                        print("Multiple usbmodems found. Please specify!")
                        sys.exit(2)
                    sel = dev
            if sel != 0:
                device_path = "/dev/" + sel
        except:
            print(f"Usage: {sys.argv[0]} /dev/cu.usbmodem<XXXX>")
            sys.exit(1)
    else:
        device_path = sys.argv[1]
    print(f"Opening {device_path} at {BAUD} baud...")

    os.makedirs("logs", exist_ok=True)

    with serial.Serial(device_path, BAUD, timeout=1) as port:
        time.sleep(0.5)
        port.reset_input_buffer()

        # ── Step 1: initial handshake ────────────────────────────────────────
        # Firmware: scanf(" %c", ui) → prints "[input recieved]"
        print("\n[1] Sending initial prompt...")
        send_char(port, 'c')
        #wait_for(port, "input recieved")
        #print("    Firmware ready.")

        # Drain the menu text the firmware prints
        time.sleep(0.3)
        port.reset_input_buffer()

        # ── Step 2: command loop ─────────────────────────────────────────────
        print(HELP)
        while True:
            ui = input("cmd> ").strip()
            if not ui:
                continue

            ch = ui[0]

            if ch == 'q':
                print("\nSending 'q' → rebooting device...")
                send_char(port, 'q')
                try:
                    wait_for(port, "REBOOT", timeout=3.0)
                except TimeoutError:
                    pass
                print("Done.")
                break

            elif ch is 'r' and ui[1:].isnumeric():

                STREAM_DURATION = int(ui[1:])
                # ── stream & decode ──────────────────────────────────────────
                send_char(port, 'r')
                print("\n[r] Waiting for stream start...")
                wait_for(port, "STREAMING RAW DATA")
                print("    Stream started.")
                print(f"    Stream started. Collecting...")

                while(True):
                    ui = input("cmd> ").strip()
                    if not ui:
                        break
                    ch = ui[0]
                    send_char(port, ch)

                send_char(port, 'c')  # stop the stream

                raw_bytes = read_raw_stream(port)

                n_bytes  = len(raw_bytes)
                n_words  = n_bytes // BYTES_PER_WORD
                leftover = n_bytes % BYTES_PER_WORD

                print(f"\n    Received {n_bytes} bytes ({n_words} samples, {leftover} leftover bytes)")
                if leftover:
                    print(f"    WARNING: {leftover} bytes not aligned — truncating.")

                # RP2350 is little-endian; dma_buff is uint32_t[] written directly
                samples = struct.unpack_from(f"<{n_words}I", raw_bytes)

                data = {}
                for word in samples:
                    ch_id, val = decode_sample(word)
                    if str(ch_id) not in data.keys():
                        data[str(ch_id)] = []
                    data[str(ch_id)].append(val)

                fname = "log [" + time.strftime("%Y-%m-%d-%H-%M-%S") + "].txt"
                fpath = "logs/" + fname
                with open(fpath, "x") as f:
                    keys = list(data.keys())
                    for key in keys:
                        f.write(f"CH {key}, ")
                    f.write("\n")
                    for i in range(len(data[keys[0]])):
                        for key in keys:
                            f.write(f"{data[key][i]}, ")
                        f.write("\n")

                print(f"    Saved → {fpath}")
                for k, v in data.items():
                    print(f"    Channel {k}: {len(v)} samples")

                # Drain any trailing firmware text (batch count line etc.)
                time.sleep(0.3)
                port.reset_input_buffer()

            elif ch in ('e', 'd', 'p', 'l', ' ') or ch.isdigit():
                send_char(port, ch)
                time.sleep(0.1)
                # Echo whatever the firmware sends back
                resp = port.read(port.in_waiting)
                for line in resp.decode(errors="replace").splitlines():
                    if line.strip():
                        print(f"  [fw] {line.strip()}")

            else:
                print(f"  Unknown command '{ch}'. {HELP}")


if __name__ == "__main__":
    main()