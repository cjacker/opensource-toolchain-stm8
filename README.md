# Opensource toolchain for STM8

**NOTE, this tutorial is not complete now** 

ST's 8-bit microcontroller platform is implemented around a high-performance 8-bit core and a state-of-the-art set of peripherals. This platform is manufactured using an ST-proprietary 130 nm embedded non-volatile memory technology.
The STM8 allows fast and safe development through enhanced stack pointer operations, advanced addressing modes and new instructions.

For more infomation, refer to https://www.st.com/en/microcontrollers-microprocessors/stm8-8-bit-mcus.html

# Hardware requirements
You need:
* a development board with STM8 MCU.
* a STLINK USB adapter(no matter version) with SWIM interface, for flashing and debugging.
* a USB/UART adapter if it is not on development board. it can be used for UART flashing/programming if with bootloader support, but lack of debugging support.


# Toolchain overview

The opensource toolchain for STM8 under linux includes:
* SDCC as compiler
* stm8flash, flashing with STLINK adapter
* stm8gal, flashing with USB/UART adapter
* openocd/stm8-binutils-gdb, as debugger, a STLINK adapter is mandary.

# SDCC Compiler
Most Linux distributions shipped SDCC in their repositories. You can install it by using yum or apt.

If you really want to build it yourself, at least you need make/bison/flex/libtool/g++/boost development package/zlib development package/etc. installed, and the building process is really simple:
```
./configure --prefix=<where you want to install SDCC>
make
make install
```
if the `prefix` does not set to standard dir (such as '/usr' or '/usr/local'), you need add the `<prefix>/bin` dir to PATH env.

# OpenOCD

Most Linux dist shipped OpenOCD, you can install it from repositories by yum or apt.

For STM8 development, it's not neccesary to build OpenOCD yourself, there is no more patches needed.

If you really want to build it, please refer to "OpenOCD for Programming and Debugging" section of ![Opensource toolchain for gd32vf103](https://github.com/cjacker/opensource-toolchain-gd32vf103).

# SDK

TODO, at least provide some examples and project templates.

# Flashing/Programming
There is two flashing tools for STM8 you can use under linux, it depends on how you connect the development board to PC.

## for STLINK adapter
You can use STLINK SWIM to connect STM8 development board to PC linux, the PROS is that it does support flashing and debugging. the CONS is that you need buy a STLINK adapter and wire it up, it not very convenient since a lot of developement board today have a USB/UART chip on board. But you need activate BSL and 'BSL activate' under linux need a STLINK and stm8flash, it's better for you to prepare one STLINK adapter. 

You need have gcc/libusb development package installed before building and installing stm8flash:

```
git clone https://github.com/vdudouyt/stm8flash.git 
cd stm8flash
make
sudo install -m0755 stm8flash /usr/bin/
```


## for USB/UART adapter or on board chip (no debugging support)
TODO, stm8gal and how to activate BSL.

# Debugging with stm8-gdb
TODO, stm8-binutils-gdb

