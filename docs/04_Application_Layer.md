# Application Layer

This document covers the middle of the firmware — the shell, command dispatch, app state, and the core-1 control loop. If you want to add a new shell command or change how the controller behaves, this is the layer you're working in.

The files involved are all in `ADPC_Data_Streaming/`:

| File | Role |
|------|------|
| `adpc_data_streaming.c` | `main()` entry point — almost everything else delegates here |
| `adpc_app.[ch]` | App state struct, line buffer, command dispatch |
| `adpc_app_funcs.[ch]` | Implementations of the individual commands |
| `adpc_core1.[ch]` | The closed-loop current controller (runs on core 1) |

---

## Boot flow

`main()` in `adpc_data_streaming.c` is tiny:

```c
int main() {
    stdio_init_all();           // sets up USB CDC
    char uic;
    scanf(" %c", &uic);         // wait for any character (the handshake)
    printf("[input recieved]\n");
    if(uic == 'q') goto reboot;
    sleep_ms(100);

    while(1) {
        if(app_shell_task(&ui_state) == APP_REBOOT) goto reboot;
    }

reboot:
    printf("\n\nREBOOT!\n");
    reset_usb_boot(0,0);        // re-enumerates as a flash drive
    return 0;
}
```

The `scanf(" %c", &uic)` is a *handshake*. Many serial terminals connect to USB CDC slowly enough that the firmware's early output gets dropped. By waiting for the host to send a byte first, we know the connection is established and our subsequent `printf` will be received.

The host-side `script.py` automatically sends `c\n` as a handshake — operators using a terminal manually just have to press any key.

After the handshake, the main loop just calls `app_shell_task` over and over. Each call processes at most one character of user input.

---

## App state: `ui_state`

```c
typedef struct {
    char ui[UI_BUFF_SIZE];    // line buffer for input
    char ui_cursor;           // how far into the line buffer we are
    float level;              // last commanded differential PWM level
    float current_setpoint;   // last commanded current setpoint (amps)
    bool initialized;         // has `start` or `init` been run?
    bool is_streaming;        // are we currently in rstream / irun stream?
    bool pwm_enabled;         // (informational, not all paths update this)
    bool ph_enabled;          // (informational, not all paths update this)
    bool cl_ictl;             // is core 1 running the closed-loop controller?
    struct ictl_prog current_program;   // the loaded irun program
} app_state_t;

extern volatile app_state_t ui_state;
```

There's one global instance, `ui_state`, declared `volatile` because both cores touch it. The shell passes a pointer to it (`&ui_state`) into every command handler.

A few fields are aspirationally tracked (`pwm_enabled`, `ph_enabled`) — not all paths update them consistently, so don't rely on them as the source of truth for hardware state. The driver-level structs in `mphb2_arr` are more reliable.

### `current_program`

This holds the loaded "current program" — a sequence of setpoints to step through:

```c
struct ictl_prog {
    float timestep;             // milliseconds between setpoints
    float setpoints[ICTL_PROG_MAX_SIZE];
    int N;                       // how many setpoints actually used
    int cycles;                  // (reserved for future use)
};
```

`ICTL_PROG_MAX_SIZE` is 128. The `iprog` shell command populates this struct; `irun` and `irun stream` consume it.

---

## The shell: `app_shell_task`

The shell is a single non-blocking function that the main loop calls repeatedly:

```c
app_result_t app_shell_task(app_state_t *s) {
    // 1. If we're at the start of a line and the buffer still has old data,
    //    print the ">> " prompt and clear the first byte
    if(s->ui_cursor == 0 && s->ui[0] != 0) {
        if(!s->is_streaming) printf(">> ");
        s->ui[0] = 0;
    }
    
    // 2. Try to read one character, with a 1 ms timeout
    int uic = getchar_timeout_us(1000);
    if(uic == PICO_ERROR_TIMEOUT) return APP_RUNNING;
    
    // 3. Backspace handling
    if((char)uic == UI_BACKSPACE) {
        if(s->ui_cursor == 0) return APP_RUNNING;
        printf("\b \b");
        s->ui_cursor -= 1;
        return APP_RUNNING;
    }
    
    // 4. Echo the character back to the user
    if(!s->is_streaming) putchar((char)uic);
    
    // 5. If it's a newline, terminate the buffer and dispatch
    if((char)uic == '\n') {
        s->ui[s->ui_cursor] = 0;
        s->ui_cursor = 0;
        return app_dispatch(s);
    }
    
    // 6. Otherwise, append the character (with overflow guard)
    s->ui[s->ui_cursor] = (char)uic;
    if(s->ui_cursor < UI_BUFF_SIZE - 1) s->ui_cursor++;
    else printf("\b");
    return APP_RUNNING;
}
```

Key points:

- It's non-blocking. Each call either returns `APP_RUNNING` (still building a line) or whatever `app_dispatch` returns.
- The 1 ms timeout makes it safe to call from a busy loop without burning the CPU.
- Echo is suppressed when streaming so that user characters don't corrupt the binary output.
- The buffer is `UI_BUFF_SIZE = 64` bytes. Lines longer than that get truncated (the last character gets a backspace echoed).

---

## Command dispatch: `app_dispatch`

When the shell sees a newline, it calls `app_dispatch`. This is currently implemented as a long sequence of `strncmp` checks:

```c
app_result_t app_dispatch(app_state_t *s) {
    char *ui = s->ui;

    if (s->ui[0] == 'q') return APP_REBOOT;
    if (strncmp(ui, "init", 4) == 0) { ... }
    if (strncmp(ui, "level", 5) == 0) { ... }
    if (strncmp(ui, "isp ", 4) == 0) { app_cmd_isp(s, atof(ui + 4)); }
    if (strncmp(ui, "phen", 4) == 0) { ... }
    if (strncmp(ui, "phd", 3) == 0) { ... }
    if (strncmp(ui, "pwmen", 5) == 0) { ... }
    if (strncmp(ui, "pwmd", 4) == 0) { ... }
    if (strncmp(ui, "start", 5) == 0) { ... }
    if (strncmp(ui, "rstream", 7) == 0) { return app_cmd_rstream(s); }
    if (strncmp(ui, "r0", 2) == 0) { ... }
    if (strncmp(ui, "r1", 2) == 0) { ... }
    if (strncmp(ui, "ictl", 4) == 0) { app_cmd_ictl(s); }
    if (strncmp(ui, "coeff i ", 8) == 0) { ... }
    if (strncmp(ui, "coeff p ", 8) == 0) { ... }
    if (strncmp(ui, "iprog", 5) == 0) { ... }
    if (strncmp(ui, "irun", 4) == 0) { ... }
    if (strncmp(ui, "sdith", 5) == 0) { ... }
    
    printf("%s\n", ui);    // echo unrecognized commands
    return APP_OK;
}
```

This isn't the prettiest dispatch in the world, but it works. There are a few gotchas:

- The checks are **prefix matches** and **not mutually exclusive**. For example, `ictl` would also match a hypothetical `ictlfoo` command. Order matters somewhat.
- After running a command, execution falls through to the next `if` — most commands aren't `return`ed early. This means harmless extra `strncmp` work but is otherwise OK as long as you don't make two prefixes collide.
- The trailing `printf("%s\n", ui)` echoes the command back. This is sometimes redundant (the command itself printed feedback) and sometimes useful (unrecognized commands get echoed so the operator sees they were received but not understood).

### Adding a new shell command

The minimal pattern is:

1. **Write the implementation** in `adpc_app_funcs.[ch]` (or a new file if it's substantial):

   ```c
   // in adpc_app_funcs.h
   app_result_t app_cmd_mything(app_state_t *s, float param);

   // in adpc_app_funcs.c
   app_result_t app_cmd_mything(app_state_t *s, float param) {
       if(!s->initialized) {
           if(!s->is_streaming) printf("Not initialized — run 'start' first\n");
           return APP_ERROR;
       }
       // ... do the thing ...
       if(!s->is_streaming) printf("[did the thing with param = %.2f]\n", param);
       return APP_OK;
   }
   ```

2. **Add the dispatch** in `app_dispatch`:

   ```c
   if(strncmp(ui, "mything ", 8) == 0) {
       return app_cmd_mything(s, atof(ui + 8));
   }
   ```

3. **Document it** in `HOW_TO_ADPC_Streaming.md`.

The "non-blocking" model means your command handler is allowed to spend time — block, loop, talk to hardware. The shell will pick up new input on the next call to `app_shell_task` *after* yours returns. If your command takes more than a fraction of a second, it should poll for user input itself (the way `app_cmd_irun` does) so the user can abort.

---

## Command catalog

A summary of what each existing command does and where its implementation lives. For operator-level usage details, see `HOW_TO_ADPC_Streaming.md`.

| Command | Implementation | Notes |
|---------|----------------|-------|
| `start` | `app_dispatch` (inline) | Calls `adpc_init`, enables PWM and PH_EN |
| `init` | `adpc_init` in `adpc_app_funcs.c` | Lower-level init; doesn't enable PH_EN |
| `level <n>` | `app_dispatch` (inline) | Open-loop set differential PWM level |
| `phen` / `phd` | `app_dispatch` (inline) | Phase-enable gate |
| `pwmen` / `pwmd` | `app_dispatch` (inline) | PWM peripheral on/off, sets level to 0 |
| `sdith <f>` | `app_dispatch` (inline) | Spatial-dithering level set |
| `isp <amps>` | `app_cmd_isp` | Set current setpoint for closed-loop |
| `ictl` | `app_cmd_ictl` | Start or stop the core-1 controller |
| `coeff i <f>` / `coeff p <f>` | `app_dispatch` (inline) | Tune PI coefficients live |
| `iprog` | `app_dispatch` (inline) | Interactive program entry |
| `irun` | `app_cmd_irun` | Run loaded program |
| `irun stream` | `app_cmd_irun_streaming` | Run loaded program with simultaneous streaming |
| `rstream` | `app_cmd_rstream` | Stream raw ADC data |
| `r0 <n>` / `r1 <n>` | `app_dispatch` (inline) | Print N samples from the DMA buffers for inspection |
| `q` | `app_dispatch` (returns `APP_REBOOT`) | Reboot to flash mode |

---

## The core-1 controller (`adpc_core1.c`)

When `ictl` is invoked, core 1 is launched and starts running `core1_ictl()`. This is the closed-loop current controller. It:

1. Polls the DMA write position on ADC 0 (the 16-bit ISNS ADC).
2. When a new sample arrives, computes a 5-sample moving average.
3. Subtracts the setpoint to get an error.
4. Runs a PI law (proportional + integral with anti-windup clamps).
5. Writes the new level to the H-bridges using `mphb_set_dlevel_all_spatial_dithering`.
6. Checks the inter-core FIFO for a stop request.

### The control state

```c
typedef struct {
    float accum;     // integrator state
    float i_coeff;   // integral gain
    float p_coeff;   // proportional gain
} ictl_info_t;

extern volatile ictl_info_t ictlInfo;
```

This is global, declared `volatile` so that the shell on core 0 can update `i_coeff` and `p_coeff` live via the `coeff i` and `coeff p` commands and core 1 will see the changes.

### The control law

```c
float err = (avg - ictl_sp);
float incr = err * ictlInfo.i_coeff;
ictlInfo.accum += incr;
if (ictlInfo.accum < -50000) ictlInfo.accum = -50000;
if (ictlInfo.accum > 5000)   ictlInfo.accum = 5000;

float level = (ictlInfo.accum + ictlInfo.p_coeff * err) / 100000;
if(level >  0.3) level =  0.3;
if(level < -0.3) level = -0.3;
ui_state.level = level;
mphb_set_dlevel_all_spatial_dithering(ui_state.level);
```

The setpoint `ictl_sp` is computed from `ui_state.current_setpoint` using the same hand-calibrated scaling as `app_cmd_isp`. Both the integrator (`accum`) and the output (`level`) are clamped — the integrator asymmetrically because the system can only push current one way effectively (positive heating current; negative is impractical for the heater load), and the output to ±0.3 to keep things safe.

### Inter-core handshake

```c
if(multicore_fifo_rvalid()) {
    int cmd = multicore_fifo_pop_blocking_inline();
    if(cmd == MC_FIFO_STOP_FLAG) break;
}
// ... after the loop:
ui_state.cl_ictl = false;
if(!ui_state.is_streaming) printf("Exiting core 1.\n");
multicore_fifo_push_blocking(1);   // ack to core 0
```

Core 0 (via `app_cmd_ictl`) pushes `MC_FIFO_STOP_FLAG` into the FIFO, then waits for an ack, then calls `multicore_reset_core1()`. This is the *only* clean way to stop core 1.

`MC_FIFO_STOP_FLAG` is `0xBEEF` — chosen to be distinguishable from any sample value you'd reasonably get in the FIFO. If you start using the FIFO for actual data, pick a sentinel that can't collide with valid data values, or use multiple words (e.g. a command word + a payload word).

### Modifying the controller

The control law itself is one screen of code in `core1_ictl()`. To change it (e.g. add a derivative term, change the filter, use a different ADC channel), edit that function directly. Don't worry about backwards compatibility — there are no consumers of the controller's internal state outside core 1.

A few sane patterns when you do this:

- **Print sparingly.** The default code prints `ERR / INCR / ACC / LVL` once every 128 samples. More frequent prints will starve the USB pipe.
- **Always guard prints with `if(!ui_state.is_streaming)`** so they don't corrupt streaming output.
- **Re-clamp the integrator** if you change anything about the gain. Old clamps may no longer be appropriate.
- **Watch for divisions and casts.** The existing code mixes `float`, `int32_t`, and integer divisions — easy to introduce truncation errors.

---

## How `irun` works under the hood

It's instructive to read `app_cmd_irun` end-to-end because it's a representative example of a complex command:

```c
app_result_t app_cmd_irun(app_state_t *s) {
    mphb_set_ph_en_all(true);                       // turn outputs on
    if(!ui_state.cl_ictl) {                         // ensure controller running
        if(app_cmd_ictl(s) != APP_OK) return APP_ERROR;
    }
    uint64_t ts = (int)(s->current_program.timestep * 1000);   // µs
    uint64_t t = time_us_64();
    bool ui_break = false;
    for(int i = 0; i < s->current_program.N; i++) {
        while(time_us_64() - t <= ts) {
            if(stdio_getchar_timeout_us(10) != PICO_ERROR_TIMEOUT) {
                ui_break = true;
                break;
            }
        }
        t += ts;                                    // advance, don't drift
        if(ui_break) break;
        if(app_cmd_isp(s, s->current_program.setpoints[i]) != APP_OK) break;
    }
    app_cmd_ictl(s);                                // stop the controller
    mphb_set_ph_en_all(false);                      // turn outputs off
    return APP_OK;
}
```

Notice:

- It enables `PH_EN` before doing anything else (and disables it on exit). The controller has authority over the level, but `irun` owns the on/off.
- It launches core 1 if not already running, and tears it down on exit.
- The timing loop uses `t += ts` (not `t = now`) to avoid drift. If your command takes 50 µs longer than expected on one iteration, the next iteration is 50 µs shorter — total time stays correct.
- It polls for any user input as an abort signal. This is the standard pattern for blocking commands.

The streaming variant (`app_cmd_irun_streaming`) interleaves USB CDC writes with the program-stepping loop. See [`05_DMA_PIO_Streaming.md`](05_DMA_PIO_Streaming.md) for the streaming details.
