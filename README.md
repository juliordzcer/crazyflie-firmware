
# Master's thesis firmware (archived)

Crazyflie 2.x firmware used in my master's thesis (2023): formation control of
multiple quadrotors with continuous sliding-mode attitude controllers.

- Author: Julio Cesar Rodriguez Cervantes - website: https://juliordz.com/
- Thesis (corrected version, errata and stability appendix): https://juliordz.com/thesis/
- Status: **archived**. This branch keeps the code used in the thesis experiments,
  plus the fixes listed below. Current work continues on the ESP-Drone firmware.

## Attitude controllers

Selected with the `stabilizer.controller` parameter. Names as in the thesis:

| Thesis | File | Law |
|---|---|---|
| PID | `controller_pidn.c` | PID on the attitude error |
| ST | `controller_sta.c` | super-twisting on s = e' + k0 e, k1 = 1.5 zeta^(1/2), k2 = 1.1 zeta |
| TC (CTA) | `controller_tc.c` | continuous twisting: k0 [e]^(1/3) + k1 [e']^(1/2) + v, v' = k2 [e]^0 + k3 [e']^0 |
| MDST | `controller_stsmc.c` | super-twisting on s = e' + k0 [e]^(2/3) |
| MDNST | `controller_ntsmc.c` | non-singular terminal surface s = e + k0 [e']^(3/2) |
| - | `controller_smc.c`, `controller_bc.c` | classic sliding mode and backstepping (not used in the thesis) |

Units: until July 2023 the laws computed torques in N*m (inertia, gyroscopic term)
and converted them to PWM with the thrust curve; the final experiments scale the
law output directly to PWM (x1000, or x10000 for ST and MDST).

## Fixes after the thesis (2026)

- ST: the yaw sliding surface used `k0_phi` instead of `k0_psi`.
- MDNST: the yaw surface used the exponent 2/3 instead of 3/2.

## TODO

- **FT-SMO observer** (thesis Section 3.2.1, Eq. 1): `FTSMO.c` is written but never
  called (the call in `stabilizer.c` is commented out), so the experiments ran
  without the disturbance estimates xi_3 and eta_3 of Eqs. (10a) and (11).
- `position_controller_JCRC.c` (Eqs. 10-11: thrust from the norm of the virtual
  control, desired roll/pitch by asin/atan) is not called either; it would need
  the angles in degrees and the mass corrected (0.32 -> 0.032 kg) before use.

---

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
sudo apt install python3-pip -y
sudo apt-get install make gcc-arm-none-eabi -y
sudo apt install git -y
sudo apt install git python3-pip libxcb-xinerama0 

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
make cload
```

It will try to find a Crazyflie in bootloader mode and flash the binary to it.

Warning: if multiple Crazyflies within range are in bootloader mode the result is unpredictable. This method is not suitable in classroom situation where it is likely that several students are flashing at the same time. Also remember that the Crazyradio PA often reaches into the next room.

#### Automatically enter bootloader mode
* Make sure the Crazyflie is on
* In your terminal, run `CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E7" make cload`

It will connect to the Crazyflie with the specified address, put it in bootloader mode and flash the binary. This method is suitable for classroom situations.

Note: this method does not work if the Crazyflie does not start, for instance if the current flashed binary is corrupt. You will have to fall back to manually entering bootloader mode.


## USB permissions
The following steps make it possible to use the USB Radio and Crazyflie 2 over USB without being root.
```
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER
```

You will need to log out and log in again in order to be a member of the plugdev group.

Copy-paste the following in your console, this will create the file `/etc/udev/rules.d/99-bitcraze.rules`:

```
cat <<EOF | sudo tee /etc/udev/rules.d/99-bitcraze.rules > /dev/null
# Crazyradio (normal operation)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
# Bootloader
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
# Crazyflie (over USB)
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
EOF
```
You can reload the udev-rules using the following:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Crazyflie Client
### Prerequisites

This project requires Python 3.7+.

```
sudo apt install git python3-pip libxcb-xinerama0
pip3 install --upgrade pip
```
#### Installing the lib
```
pip3 install cflib
 ```
#### Installing the client
```
pip3 install cfclient
```


