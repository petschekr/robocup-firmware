
# RoboJackets RoboCup MBED Toolchain

The toolchain that we use to compile robocup-firmware is [this one](https://launchpad.net/gcc-arm-embedded). Installing it should get you what you need on ubuntu.

# Manually building

Manually building will probably be difficult.

1. You will need to compile gcc-arm-none-eabi from scratch
2. You will (probably) need arm buildutils
3. You will need arm newlib. When compiling newlib, MAKE SURE TO ENABLE `--disable-newlib-supplied-syscalls` in the `./configure` step.

Good luck!

# Gentoo

On gentoo, compiling the toolchain is (somewhat) easier.

```sh
sudo crossdev --stable -t arm-none-eabi
# Add the +cxx -noxx use flags to cross-arm-none-eabi/gcc and reemerge
sudo emerge cross-arm-none-eabi/gcc
# Get the testing version of newlib (at least 2.4.0) to get the disable syscall flag for free and remerge
sudo emerge cross-arm-none-eabi/newlib
```

You can also try getting the binary version of the above package from an overlay.
