## MSIF Development Getting Started


Now that we've got the pico software development kit (SDK) up and running, let'stake a look at the overall structure of the project and where things can get started.

From the top-level down:

1. Provide a clean interface by which a controller can communicate mass spec configuration, "programs", and data streaming configurations/requests.

2. Communicate with another controller (ADPC) over UART

3. Translate abstract requests (e.g. "set mass target to x" or "scan from a to b") into QMS-112 remote control actions (e.g. set FMASS+ analog to 3.2V)

4. Time and organize these actions 

5. Interface with MSIF hardware to perform these actions

Let's talk a bit about the hardware tasks because those should probably be the starting point. There are essentially four things between the MSIF RP2350 and the QMS remote control QDP connector.

1. MSIF Digital Inputs: read outputs from the QMS to RP2350 GPIO inputs

2. MSIF Digital Outputs: convert RP2350 3.3V GPIO outputs to QMS-accepted digital inputs

3. MSIF Analog Inputs: convert analog readings to numeric representations that the RP2350 can request from the ADC chips.

4. MSIF Analog Outputs: provide analog signals to the QMS using a digital-to-analog converter (DAC) according to numeric outputs from the RP2350.

## Coding

### Getting the code and prepping the space.

You can get the code I've started by doing:

`git clone https://github.com/TarbFela/Capstone.git`

You'll want to do this into a directory that is convenient for you. This will create a folder called `Capstone` within that directory. This contains `MSIF_dev/`, which is where you should develop your code. Please put any documentation in the relevant directory.

To get started with this, get into that directory:

`cd Capstone/MSIF_dev`

Then set up the build directory:

`mkdir build`

Go to that directory:

`cd build`

And run CMake so that you can compile the code:

`cmake -DPICO_BOARD=pico2 ..`

_You may need to set your PICO_SDK_PATH; find where `pico-sdk/` lives on your computer, copy that path, and paste it into_:

`export PICO_SDK_PATH=______`

Once this is done you should be all set to compile the project (from the `build/` directory):

`make`

Or remove all compiled files:

`make clean`

### Git

When you make changes, do:

`git commit <files> -m "<some comment>"`

When uou make new files, you'll have to precede that with:

`git add <files>`

This makes it easy to undo changes or mix-and-match. Once you're set up with a Github account, you'll be able to:

`git push origin main`

Which saves those changes to the Github servers and allows me to retrieve them. **Also, regularly run**:

`git pull`

This pulls any changes I've pushed. If you don't do this you'll have to mess around with git's "merging" tool which can be a bit of a pain.

### Digital Inputs: A starting point

The easiest place to start is the digital inputs. Once you have serial debugging (`printf()`) working, it's easy enough to start reading inputs. Right now, the code I've provided looks for inputs over USB; if the input is a "q" then it goes into USB flash mode so that new programs can be loaded; otherwise it just tells you that it received your input.

```
    while(1) {
        int uii = stdio_getchar_timeout_us(100);
        if(uii != PICO_ERROR_TIMEOUT) {
            char ui = (char)uii;
            // QUIT
            if (ui == 'q') break;
            else printf("Input recieved! [press q for BOOTSEL mode]\n");
        }
    }
```

The next step is to get the SDK's `gpio` functions working. A good resource for all of the relevant function is the SDK itself. It should exist somewhere locally on your machine, but it also exists on Github where you can easily reference it:

https://github.com/raspberrypi/pico-sdk/tree/master/src/rp2_common

From here you can navigate to `hardware_gpio/include/hardware/gpio.h`. This same patterns extends for all of the tools you'll be using, including `uart.h` and `spi.h`. 

You'll need to put `#include "gpio.h"` in your main.c file. You'll also need to put `hardware_gpio` in the `CMakeLists.txt` file under `target_link_libraries`. You may have to run the `cmake` command again (see above). Also refer to the gpio examples from Raspberry Pi  (also on Github). GPIOs need to be configured, then you can read them as you wish, assign the results to variables, and print them out with `printf("You variable is %d \n", x);` or similar. You can trigger these prints by putting them in `if() {}` statements similar to the one for "q" in the template. 

Now comes the MSIF-specific stuff!

### MSIF Digital Inputs

You'll want to refer to the schematics and define macros for the pins according to their purposes/names. Find the PGA2350 symbol and look at the digital io page of the schematic. Use something like `#define MSIF_EMIS_OK_IN_PIN 30` for all the pin defs (for the digital outputs as well). Then write code to configure the inputs as inputs, the outputs as outputs, and write functions to read/set these. You can write the functions in `main.c` for now; we'll move them to separate source/header files later. 
