# Opensource toolchain for STM8

**NOTE, most of works of STM8 development with Linux was done by Georg Icking-Konert, 
for example, stm8gal flashing tool, SPL patches for SDCC and opensource sdk STM8_headers, please refer to https://github.com/gicking** 

ST's 8-bit microcontroller platform is implemented around a high-performance 8-bit core and a state-of-the-art set of peripherals. This platform is manufactured using an ST-proprietary 130 nm embedded non-volatile memory technology.
The STM8 allows fast and safe development through enhanced stack pointer operations, advanced addressing modes and new instructions.

For more infomation, refer to https://www.st.com/en/microcontrollers-microprocessors/stm8-8-bit-mcus.html

# Hardware requirements
You need:
* a development board with STM8 MCU.
* a STLINK USB adapter(no matter version) with SWIM interface, for flashing and debugging.
* a USB/UART adapter if it is not on development board. it can be used for UART flashing/programming if with bootloader support, but lack of debugging support.

# Toolchain overview

The opensource toolchain for STM8 under linux consist of:
* SDCC as compiler
* stm8flash, flashing with STLINK adapter
* stm8gal, flashing with USB/UART adapter
* openocd/stm8-binutils-gdb, as debugger, a STLINK adapter is mandary.

# SDCC Compiler
Most Linux distributions shipped SDCC in their repositories. You can install it by yum or apt.

If you really want to build it yourself, at least you need make/bison/flex/libtool/g++/boost development package/zlib development package/etc. installed, and the building process is really simple:
```
./configure --prefix=<where you want to install SDCC>
make
make install
```
if the `prefix` does not set to standard dir (such as '/usr' or '/usr/local'), you need add the `<prefix>/bin` dir to PATH env.

# OpenOCD

Most Linux dist ships OpenOCD package, you can install it by yum or apt.

For STM8 development, it's not neccesary to patch and build OpenOCD yourself.

If you really want to build it, please refer to "OpenOCD for Programming and Debugging" section of ![Opensource toolchain for gd32vf103](https://github.com/cjacker/opensource-toolchain-gd32vf103).

# SDK

TODO, at least provide some examples and project templates.

# Flashing/Programming
There is two flashing tools for STM8 you can use with linux, it depends on how you wire up the development board to PC.

It may be a little bit weird, but you should understand that 'If you want to enable UART flashing support, you have to have a STLINK adapter and use stm8flash to flash a special firmware first' or 'you have an empty development board never flashed with STLINK'

## for STLINK adapter
You can use STLINK SWIM interface to connect STM8 development board to PC linux, the PROS is it does support flashing and debugging. the CONS is you have to buy a STLINK adapter and wire it up. But If you need to activate STM8 bootloader(BSL) under linux, a STLINK adapter and stm8flash are mandary. 

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

