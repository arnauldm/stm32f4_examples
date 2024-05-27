# stm32f4_examples

Concise C code examples for STM32F4 Discovery board

- `01_blink_led`, classical "blinky" example in C
- `02_interrupts`, blue button usage in C

You first have to install the ARM toolchain. On Debian/Ubuntu:
	
	apt install gcc-arm-none-eabi binutils-arm-none-eabi

To run each example, connect the stm32f407 discovery board, compile the
firmware and flash it with openocd:

	make
	make flash

For further informations about how to program that board, you might need to
read the datasheets and the manuals provided by ST micro:

	https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series/documentation.html

The main reference manual for stm32f407 MCU can be found here:

	https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf


