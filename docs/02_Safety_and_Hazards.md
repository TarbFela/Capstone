# Safety & Hazards

This system drives large currents through real heaters. There are also a number of software footguns that have bitten developers (including the original author) and will bite you. Read this section before you start changing the firmware.

The hazards are ordered roughly by potential consequence — hardware issues first, software second.

---

## ⚡ Hardware hazards

### 1. Always disable `PH_EN` before reconfiguring PWM

The `PH_EN` ("phase enable") pin on each MPHB-002 channel is a **gate** that disconnects the power stage without interrupting the PWM clock. The intended sequence for changing PWM safely is:

1. Set `PH_EN` low (`mphb_set_ph_en(HBxX, false)` or `mphb_set_ph_en_all(false)`).
2. Reconfigure PWM levels, change the differential level, switch dithering on/off, etc.
3. Re-enable `PH_EN`.

If you change the PWM levels *while `PH_EN` is high*, the power stage will instantaneously follow — which is fine if the new level is close to the old one, but very much not fine if you're going from a tiny duty cycle to a large one. Always cycle `PH_EN` for any big change.

### 2. The "level" is differential about a common-mode duty cycle

`mphb_set_dlevel_all(level)` does **not** set the duty cycle directly. It sets a *differential* level about `mphb_pwm_cm_level` (default: `MPHB_PWM_WRAP / 2 = 500`, i.e. 50% duty). Concretely:

```c
// Inside mphb_set_dlevel():
mphb_set_levels(i,
    mphb_pwm_cm_level + (dlevel + 1)/2,    // A side
    mphb_pwm_cm_level - (dlevel / 2));     // C side
```

So `dlevel = 0` produces matched 50%/50% PWM on the two halves and **zero net voltage across the load**. Positive `dlevel` drives one way, negative the other. The maximum useful range is roughly `±500` (full positive or full negative drive), but the firmware clamps the *closed-loop* controller to `±0.3` of full scale (i.e. ±150 PWM counts) — see `core1_ictl()`. There is no equivalent clamp on the *open-loop* `level <n>` command. Be careful when testing manually.

### 3. The current setpoint scaling is hand-calibrated

In `app_cmd_isp()`:
```c
int32_t sp_scaled = sp * 162.4203245 - -25.41180842;
```
These coefficients are calibration values for a *specific physical setup* (a specific shunt resistor, a specific PGIA gain, a specific common-mode). **If you change any of those, the scaling is wrong and the controller will drive the wrong current.** Re-derive them by running `rstream` at known currents, fitting a line through the data, and updating `app_cmd_isp()` and the matching line in `core1_ictl()` (yes, the constants are duplicated — there's a TODO to fix this).

### 4. There is no over-current shutdown in firmware

If the controller becomes unstable (e.g. because someone set bad PI coefficients with `coeff i` / `coeff p`), nothing in firmware will save the hardware. The hardware does have current-sense and fuse protection at the board level, but you should not rely on that for software bugs.

Practical mitigation: **start with `phen` off, then enable** after you've verified the level is sensible:
```
>> start
>> level 0
>> phen           ← only after confirming
```

### 5. `q` is not a soft reboot — it puts the chip into firmware-flash mode

If someone hits `q` thinking they're restarting the firmware, the firmware **stops running** and the USB device re-enumerates as a mass-storage device. PWM, PH_EN, and all output state at the moment of the `q` press persist as whatever the hardware retained, and the firmware will not service any further commands until new code is flashed.

This is *intentional* — it makes flashing easy during development — but operators should know that `q` means "I am done and ready to flash new firmware."

To safely shut down the outputs *first*, run `phd` and `pwmd` before `q`.

### 6. The RUN button is a hard reset

Useful when the firmware is stuck (hung in a busy-wait, etc.) without wanting to reflash. It does not save state — DMAs are aborted mid-transfer, the USB session drops, etc. Generally safe to use because the MPHB-002 hardware should bring outputs to zero when PWM stops, but verify on your specific setup.

### 7. The ADC nIRQ pin needs an *external* pull-up

This is a documented quirk of the MCP3x6xR family: without an external pull-up on the nIRQ line, **the ADC will not perform conversions**. The ADPC board has these pull-ups installed. If you breadboard one of these ADCs onto a new board for development, you must add the resistor.

---

## 💾 Software hazards

### 1. Shared state between cores is `volatile` but not atomic

`ui_state` lives in shared RAM and is touched by both cores. It's declared `volatile`, which prevents the compiler from caching its value in a register, but it does **not** make multi-byte writes atomic. If core 0 is updating `ui_state.level` (a float, 4 bytes) at the moment core 1 reads it, core 1 can in principle observe a half-written value.

In practice the writes are infrequent enough and the consequences of one bad value are mild enough (the next loop iteration corrects it) that this hasn't been observed to cause problems. But if you add a multi-word shared state — say, a struct of calibration coefficients written from core 0 and read from core 1 — wrap the updates in `multicore_lockout` or use the FIFO to message them.

### 2. The DMA-tracking globals are also volatile-but-not-atomic

`dma0_last_written` and `dma1_last_written` are written from inside ISRs and read from both cores' main loops. Each is a single `int`, so the read/write is atomic on the RP2350 (single 32-bit memory access). You can rely on observing 0, 1, or 2 — never a torn value.

**However:** there is a subtle window where the ISR has updated `dma_last_written` but the DMA `transfer_count` register hasn't yet been re-initialized for the next half. Code that mixes `dma_last_written` and `dma_channel_hw_addr(...)->transfer_count` (as `core1_ictl()` does) must guard against this — the existing code uses a `goto start` to retry whenever it detects an inconsistency. If you write new code that consumes these globals, follow the same pattern.

### 3. DMA buffer indexing is genuinely confusing

The buffers are arranged as one contiguous block of `2 × DMA_BUFF_SIZE` samples, with buffer A in the lower half and buffer B in the upper half. To compute the current write position you have to flip the meaning of `lw_current`:

```c
// if dma0_last_written == 1, then DMA A just finished;
// DMA B is now writing, into the upper half (base = DMA_BUFF_SIZE).
// if dma0_last_written == 2, then DMA B just finished;
// DMA A is now writing, into the lower half (base = 0).

int current_base = (2 - lw_current) * DMA_BUFF_SIZE;
```

The previous author originally wrote `((2 - lw_current) * DMA_BUFF_SIZE + DMA_BUFF_SIZE + ...) & mask`, which silently reads from the wrong buffer — this was a real bug, found by hand, and the fix went into `core1_ictl()`. If you copy this pattern elsewhere, double-check by walking through both cases.

### 4. `getchar_timeout_us(1000)` blocks for up to 1 ms

The shell loop polls USB CDC with a 1-millisecond timeout per character. This is fine for the shell itself but means the *main core does almost nothing during the shell* — including not flushing USB writes. If you add code that needs to run at higher rate than 1 kHz from core 0, either drop the timeout or move it off core 0.

### 5. `printf()` from inside the streaming loop will corrupt the data

When `rstream` is active, stdio over USB is muted (`stdio_set_driver_enabled(&stdio_usb, false)`) so that `printf` calls produce no output. **But** the USB CDC is still being written to directly with raw DMA bytes. If you add a `printf` inside the streaming path — or, more sneakily, if a function deep inside one of your handlers has a debug `printf` — the bytes will interleave with the binary stream and the host-side parser will choke.

The convention in this codebase is: wrap any potentially-streaming debug print in `if(!s->is_streaming) printf(...)`. Several existing functions already do this. Follow the pattern.

### 6. `is_streaming` must be cleared on every exit path

`app_cmd_rstream` and `app_cmd_irun_streaming` both set `s->is_streaming = true` at entry. They must clear it on **every** exit path or `app_shell_task` will think the device is still streaming and behave strangely (no prompt, no echo, etc.). The current code clears it in the normal path; double-check this if you add new exit conditions.

### 7. The MCP PIO driver can only be initialized twice

In `mcp_pio_init()`:
```c
static int call_count = 0;
if(call_count == 0) { pio = pio0; ... }
else if (call_count == 1) { pio = pio1; ... }
else { panic("\n\tPANIC!! PIO INIT MAY ONLY BE CALLED TWICE.\n"); }
```
There are only two PIO blocks on an RP2350, and each MCP ADC consumes one. If you initialize a third instance, the firmware will panic at boot. If you need PIO for something else (a custom UART, a fancy SPI, etc.), you must either combine state machines on a shared PIO block or rewrite the driver to claim an unused PIO+SM pair properly.

### 8. The DMA IRQ handler does work that should arguably be elsewhere

The DMA IRQ handlers in `adpc_app_funcs.c` are short, but at one point there was a `>>16` shift loop running inside `dma_irq_handler_0` (now commented out). **Don't do this.** Interrupt handlers should clear the IRQ flag, update one or two volatile flags, and return. Long work inside an ISR will:

- Block other interrupts (USB, the other DMA, the PWM wrap, etc.).
- Get re-entered by the same DMA finishing the next half before you've finished the previous one.
- Make the system's worst-case latency unpredictable.

If you need to post-process samples (sign extension, scaling, decimation), do it on core 1, not in the ISR.

### 9. Floating-point math in the control loop has limited resolution

`ui_state.level` is now a `float`, but it eventually gets cast to `int` when it's written into the PWM hardware. Tiny per-iteration corrections smaller than 1 PWM count get truncated. If you change the control law, watch out for situations where the integral term should be slowly winding up but isn't, because each contribution is rounding to zero.

### 10. The `src2/` and `*_Dev/` directories contain stale code

There's a strong urge to copy something from `src2/main.c` or one of the standalone test projects when you need a quick example. Those files **predate the current architecture** and have known bugs and deprecated patterns. Refer to them for inspiration only; do not copy code from them into `ADPC_Data_Streaming/`.

---

## Quick checklist before you flash new firmware

- [ ] All outputs in your test setup can tolerate full-scale PWM if the firmware misbehaves
- [ ] `PH_EN` defaults to off at boot (it does, but verify nothing you added enables it early)
- [ ] No `printf` calls were added to streaming paths without an `is_streaming` guard
- [ ] DMA IRQ handlers are still short (≤ ~5 lines of real work)
- [ ] Any new shared state between cores is either `volatile` *and* single-word, or protected
- [ ] You haven't added a third `mcp_pio_init` call
- [ ] If you changed shunt resistors, PGIA gain, or common-mode, you re-derived the `app_cmd_isp` scaling
