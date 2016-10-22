# Connect-Four-BLE
An example project utilizing the Nordic Semiconductor nRF52 perpherials and SoftDevice to implement a Connect Four game to be played on a mobile phone through Bluetooth Low Energy and a computer through Segger RTT Viewer. Adapted from Nordic's sample bluetooth UART perphieral project.

# Hardware Requirements
* nRF52 Developer Kit
* Adafruit 16x32 RGB LED matrix panel
* A 5V power supply for the matrix panel - I used a battery compartment that connects four AA batteries in series to get a 6V power supply inputted into a LM7805 regulator
* SparkFun microSD Transflash Breakout (not used yet)
* Lots of jumper cables

# Building
Download and extract the [Nordic Semiconductor nRF5 SDK v12.0.0](http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk%2Fdita%2Fsdk%2Fsdk.html&cp=6), [GNU ARM Embedded Toolchain](https://launchpad.net/gcc-arm-embedded/), [SEGGER J-Link](https://www.segger.com/downloads/jlink), and a MinGW environment if you're running Windows (Git). Add to your path environment variable the GNU ARM tools bin directory.

Then go into the pca10040/s132/armgcc directory and compile: ``make``

In the newly created _build directory find the hex file, open J-Link, connect and flash to the nRF52 development kit:
``loadfile connect_four.hex 0x1f000``

#Playing Connect Four from a mobile device
Coming soon

#Playing Connect Four from Segger RTT Viewer
Open the Segger RTT Client application, which should automatically connect to the onboard debugger when the program is running. You may view printf messages sent from the system or send your own keystrokes from the terminal.

#Debugging with GDB
A J-Link remote server may be created to allow a GDB client to connect. Open the J-Link GDB Server program, set the config as a USB connection, target device nRF52832_xxAA (choose from Nordic Semi manufacturer), SWD interface. You may need to connect via J-Link commander if not already done so. Note the TCP/IP port, this is what you use to connect to the server on localhost. To connect, use GDB from GNU Tools ARM Embedded (arm-none-eabi-gdb). Open another terminal, go to where the build folder is, and type (assuming port 2331):

    arm-none-eabi-gdb nrf52832_xxaa.out
    target remote localhost:2331
    break ../../../main.c:178

#License
Currently under the license agreement from the Nordic Semiconductor ASA ("Nordic") Software Development Kit.