# Connect-Four-BLE
An example project utilizing the Nordic Semiconductor nRF52 perpherials and SoftDevice to implement a Connect Four game to be played on a mobile phone through Bluetooth Low Energy and a computer through Segger RTT Viewer. Adapted from Nordic's sample bluetooth UART perphieral project.

# Building
Download and extract the [Nordic Semiconductor nRF5 SDK v11.0.0](http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk%2Fdita%2Fsdk%2Fsdk.html&cp=6), [GNU ARM Embedded Toolchain](https://launchpad.net/gcc-arm-embedded/), [SEGGER J-Link](https://www.segger.com/downloads/jlink), and a MinGW environment if you're running Windows (Git). Add to your path environment variable the GNU ARM tools bin directory.

You'll need the config folder from any ble_perpherial sample project, place it at the project root.

Then go into the pca10040/s132/armgcc directory and compile with ``make``

In the newly created _build directory find the hex file, open J-Link, connect and flash to the nRF52 development kit:
``loadfile connect_four.hex 0x1f000``

#Playing Connect Four from a mobile device
Coming soon

#Playing Connect Four from Segger RTT Viewer
Open the Segger RTT Viewer application and connect to the nRF52 development kit. You may view printf messages sent from the system or send your own keystrokes from the terminal.

#License
Currently under the license agreement from the Nordic Semiconductor ASA ("Nordic") Software Development Kit.