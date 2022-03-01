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
## Baremetal
"Baremetal programming" is not that difficult for MCU, you can always do something without using any libraries/SDKs.

Here is an example for blink a LED wired up as PD0->Resister->LED->GND.

You must read the [datasheet](https://www.alldatasheet.com/view.jsp?Searchword=STM8S208MB) first to findout the memory and register map, here I use STM8S208MB and the io register map as below:

<img src="https://user-images.githubusercontent.com/1625340/156101703-486034e5-f086-4f96-925f-3e35a1e5a943.png" width="50%"/>

These registers are pretty much self-explanatory. here’s a brief overview: `DDR` is the direction register, which configures a pin as either an input or an output. After we configured `DDR` we can use `ODR` for writing or `IDR` for reading pin state. Control registers `CR1` and `CR2` are used for configuring internal pull-ups, output speed and selecting between push-pull or pseudo open-drain.

```
//blink.c

#include <stdint.h>
#define F_CPU 2000000UL
#define _SFR_(mem_addr)     (*(volatile uint8_t *)(0x5000 + (mem_addr)))
/* PORT D */
#define PD_ODR      _SFR_(0x0F)
#define PD_DDR      _SFR_(0x11)
#define PD_CR1      _SFR_(0x12)
#define LED_PIN     0
static inline void delay_ms(uint16_t ms) {
    uint32_t i;
    for (i = 0; i < ((F_CPU / 18000UL) * ms); i++)
        __asm__("nop");
}
void main() {
    PD_DDR |= (1 << LED_PIN); // configure PD0 as output
    PD_CR1 |= (1 << LED_PIN); // push-pull mode
    while (1) {
        /* toggle pin every 250ms */
        PD_ODR ^= (1 << LED_PIN);
        delay_ms(250);
    }
}
```
and built it:

```
sdcc -lstm8 -mstm8 blink.c
```

## SPL - ST's official Standard Peripheral Library

ST officially provide 'STM8 Standard Peripheral Library' for STM8 MCUs, you can download it from [here](https://www.st.com/content/st_com/en/search.html#q=STM8%20Standard%20Peripheral%20Library-t=tools-page=1)

**NOTE:**
* **you have register and login before download, also there is a license you need to read and agree before download**
* **There are 4 packages, choose the correct one according to your MCU model.**

According the license, it seems can be redistributed with original license kept.

As metioned beginning, Georg Icking-Konert done a great job to provide a set of patches to enable SPL work with SDCC.

Please refer to https://github.com/gicking/STM8-SPL_SDCC_patch and patch the SPL yourself after download.


## STM8_headers
Georg Icking-Konert also had another great opensource project named ["STM8_headers"](https://github.com/gicking/STM8_headers) for all STM8 microcontroller series, namely STM8A, STM8S, STM8L, STM8AF and STM8AL. it's MIT license and you can use this project instead of SPL.



# Flashing/Programming
There is two flashing tools for STM8 you can use with linux, it depends on how you wire up the development board to PC.

It may be a little bit weird, but you should understand that 'If you want to enable UART flashing support, you have to have a STLINK adapter and use stm8flash to flash a special firmware first' or 'you have an empty development board never flashed with STLINK'

## with STLINK adapter
You can use STLINK SWIM interface to connect STM8 development board to PC linux, the PROS is it does support flashing and debugging. the CONS is you have to buy a STLINK adapter and wire it up. But If you need to activate STM8 bootloader(BSL) under linux, a STLINK adapter and stm8flash are mandary. 

You need gcc/libusb development package installed before building and installing stm8flash:

```
git clone https://github.com/vdudouyt/stm8flash.git 
cd stm8flash
make
sudo install -m0755 stm8flash /usr/bin/
```


## with USB/UART adapter or on board UART chip (no debugging support)
TODO, how to activate BSL and stm8gal

# Debugging with stm8-gdb
TODO, stm8-binutils-gdb

