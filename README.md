
# Crazyflie Firmware

This repository is based on the following repositories, considering some changes in the codes to have compatibility with ubuntu 20.4 and with Optitrack's Motion Capture system and in addition control algorithms applied to individual nano quadrotors and in swarm will be added.
*** Repository of the original unmodified codes
crazyflie-firmware
https://github.com/bitcraze/crazyflie-firmware


This project contains the source code for the firmware used in the Crazyflie range of platforms, including the Crazyflie 2.X and the Roadrunner.


## Building and Flashing
### Requirements
To be able to execute the programs it is necessary to install the following dependencies, executing the following commands in the console
```
sudo apt-get update
sudo apt install python3-pip
sudo apt-get install make gcc-arm-none-eabi
```
### Cloning
This repository uses git submodules. Clone with the `--recursive` flag
```
cd
git clone --recursive https://github.com/juliordzcer/crazyflie-firmware.git
```
Note Make sure there are no spaces in the folder structure leading up to the repository (example: /a/path/with/no/spaces/crazyflie-firmware/ vs a/path with spaces/crazyflie-firmware/). Our build system can not handle file system paths with spaces in them, and compilation will fail.

If you already have cloned the repo without the `--recursive option`, you need to get the submodules manually

```
cd crazyflie-firmware
git submodule init
git submodule update
```

## Compiling
### Building the default firmware 
Before you can build the firmware, you will need to configure it. To get the default configuration you can write:
```
make cf2_defconfig
make -j 12
```
## Make targets

### General targets
```
all             : Shortcut for build
compile         : Compile cflie.hex. WARNING: Do NOT update version.c
build           : Update version.c and compile cflie.elf/hex
clean_o         : Clean only the Objects files, keep the executables (ie .elf, .hex)
clean           : Clean every compiled files
mrproper        : Clean every compiled files and the classical editors backup files

cload           : If the crazyflie-clients-python is placed on the same directory level and
             the Crazyradio/Crazyradio PA is inserted it will try to flash the firmware
             using the wireless bootloader.
flash           : Flash .elf using OpenOCD
halt            : Halt the target using OpenOCD
reset           : Reset the target using OpenOCD
openocd         : Launch OpenOCD
rtt             : Start RTT server. Compile the firmware with "DEBUG_PRINT_ON_SEGGER_RTT=1"
             and the console is visible over TCP on port 2000 "telnet localhost 2000".
bindings_python : Build the python bindings for firmware wrappers
```


## Flashing
Writing a new binary to the Crazyflie is called flashing (writing it to the flash memory). This page describes how to flash from the command line and there are a few different ways to do it.

### Using Crazyradio

The supported way to flash when developping for the Crazyflie is to use the Crazyradio and the radio bootloader.

#### Prerequisites
* A Crazyradio with drivers installed
* [Crazyflie Client installed](https://github.com/bitcraze/crazyflie-clients-python) with Python's pip (so not by Snap (Ubuntu) or the .exe (Windows))
  * Note than when developping in WSL on Windows, the client needs to be installed on Windows. See the [Windows build instruction](#windows) above.
* The firmware has been built
* The current working directory is the root of the crazyflie-firmware project

#### Manually entering bootloader mode

* Turn the Crazyflie off
* Start the Crazyflie in bootloader mode by pressing the power button for 3 seconds. Both the blue LEDs will blink.
* In your terminal, run

```bash
$ make cload
```

It will try to find a Crazyflie in bootloader mode and flash the binary to it.

Warning: if multiple Crazyflies within range are in bootloader mode the result is unpredictable. This method is not suitable in classroom situation where it is likely that several students are flashing at the same time. Also remember that the Crazyradio PA often reaches into the next room.

#### Automatically enter bootloader mode
* Make sure the Crazyflie is on
* In your terminal, run `CLOAD_CMDS="-w radio://0/80/2M" make cload`

It will connect to the Crazyflie with the specified address, put it in bootloader mode and flash the binary. This method is suitable for classroom situations.

Note: this method does not work if the Crazyflie does not start, for instance if the current flashed binary is corrupt. You will have to fall back to manually entering bootloader mode.
