
# This is only a stub for various commands.
# Tup is used for the actual compilation.

BUILD_DIR = build
FIRMWARE = $(BUILD_DIR)/ODriveFirmware.elf
FIRMWARE_HEX = $(BUILD_DIR)/ODriveFirmware.hex
OPENOCD := openocd -f interface/stlink-v2.cfg \
		$(if $(value PROGRAMMER),-c 'hla_serial $(PROGRAMMER)',) \
		-f target/stm32f4x.cfg


all:
	@tup --quiet --no-environ-check

flash: all
	$(OPENOCD) -c init \
		-c 'reset halt' \
		-c 'flash write_image erase $(FIRMWARE)' \
		-c 'reset run' \
		-c exit

flashbmp: all
	arm-none-eabi-gdb --ex 'target extended-remote $(BMP_PORT)' \
		--ex 'monitor swdp_scan' \
		--ex 'attach 1' \
		--ex 'load' \
		--ex 'detach' \
		--ex 'quit' \
		$(FIRMWARE)
# J-Link 配置
JLINK_DEVICE := STM32F405RG
JLINK_IF := SWD
JLINK_SPEED := 4000
JLINK_PORT := 2331

# J-Link 配置 - 使用你的实际安装路径
JLINK_PATH := "C:/Program Files/SEGGER/JLink_V854"
JLINK_GDB_SERVER := $(JLINK_PATH)/JLinkGDBServerCL.exe
JLINK_COMMANDER := $(JLINK_PATH)/JLink.exe
JLINK_DEVICE := STM32F405RG
JLINK_IF := SWD
JLINK_SPEED := 4000
JLINK_PORT := 2331

# J-Link 烧录（使用 Commander）
jlink-flash: all
	@echo "Flashing with J-Link..."
	@if [ ! -f jlink_flash.jlink ]; then \
		echo "Creating jlink_flash.jlink..."; \
		echo "device $(JLINK_DEVICE)" > jlink_flash.jlink; \
		echo "si $(JLINK_IF)" >> jlink_flash.jlink; \
		echo "speed $(JLINK_SPEED)" >> jlink_flash.jlink; \
		echo "connect" >> jlink_flash.jlink; \
		echo "erase" >> jlink_flash.jlink; \
		echo "loadfile $(FIRMWARE_HEX)" >> jlink_flash.jlink; \
		echo "r" >> jlink_flash.jlink; \
		echo "g" >> jlink_flash.jlink; \
		echo "exit" >> jlink_flash.jlink; \
	fi
	"$(JLINK_PATH)/JLink.exe" -device $(JLINK_DEVICE) -if $(JLINK_IF) -speed $(JLINK_SPEED) -CommanderScript jlink_flash.jlink

# J-Link 调试（手动模式）
jlink: all
	@echo ""
	@echo "========================================="
	@echo "J-Link Debug Session"
	@echo "========================================="
	@echo "Please follow these steps:"
	@echo ""
	@echo "1. Open a new terminal and run:"
	@echo "   \"$(JLINK_GDB_SERVER)\" -device $(JLINK_DEVICE) -if $(JLINK_IF) -speed $(JLINK_SPEED) -port $(JLINK_PORT)"
	@echo ""
	@echo "2. Press Enter here after GDB Server is running"
	@echo "========================================="
	@read -p "Press Enter to continue..."
	@echo ""
	@echo "Starting GDB..."
	arm-none-eabi-gdb $(FIRMWARE) -x jlink.gdbinit

# J-Link 调试（自动启动 Server，需要 PowerShell）
jlink-auto: all
	@echo "Starting J-Link GDB Server..."
	@powershell.exe -Command "Start-Process -WindowStyle Minimized '$(JLINK_GDB_SERVER)' -ArgumentList '-device $(JLINK_DEVICE)','-if $(JLINK_IF)','-speed $(JLINK_SPEED)','-port $(JLINK_PORT)'"
	@echo "Waiting for server (3 seconds)..."
	@sleep 3
	@echo "Starting GDB..."
	arm-none-eabi-gdb $(FIRMWARE) -x jlink.gdbinit
# 注意：如果 GDB 无法连接，请确保 J-Link GDB Server 已正确启动，并且端口号与 jlink.gdbinit 中的设置一致。
gdb: all
	arm-none-eabi-gdb $(FIRMWARE) -x openocd.gdbinit

dfu: all
	python ../tools/odrivetool $(if $(value SERIAL_NUMBER),--serial-number $(SERIAL_NUMBER),) dfu $(FIRMWARE_HEX)

bmp: all
	arm-none-eabi-gdb --ex 'target extended-remote /dev/stlink' \
		--ex 'monitor swdp_scan' \
		--ex 'attach 1' \
		--ex 'load' $(FIRMWARE)

# Erase entire STM32
erase:
	$(OPENOCD) -c init -c reset\ halt -c flash\ erase_address\ 0x8000000\ 0x100000 -c reset\ run -c exit

# Erase all configuration from the ODrive
erase_config:
	$(OPENOCD) -c init -c reset\ halt -c flash\ erase_address\ 0x80C0000\ 0x40000 -c reset\ init -c reset\ run -c exit

# Sometimes the STM32 will get it's protection bits set for unknown reasons. Unlock it with this command
unlock:
	$(OPENOCD) -c init -c reset\ halt -c stm32f2x\ unlock\ 0

# The one-time programmable memory stores the board version
# has the following format:
#  - OTP format version (0xFE: version 1)
#  - vendor ID (01: ODrive Robotics - do not use this on custom incompatible hardware!)
#  - product ID (01: ODrive)
#  - hardware major version
#  - hardware minor version
#  - hardware variant (equal to the board nominal voltage)
# Bits in the OTP can only ever be set to 0 but never back to 1.
# Therefore do not try to run this command on the same board
# twice with different data.
#
# This OpenOCD command is intended for a STM32F405 and does the following:
#  FLASH_KEYR = 0x45670123; // unlock FLASH_CR
#  FLASH_KEYR = 0xCDEF89AB; // unlock FLASH_CR
#  FLASH_CR = (1 << FLASH_CR_PG); // unlock flash memory
#  [write OTP]
write_otp:
ifeq ($(OTP_CONFIRM),TRUE)
	# Data: 
	$(OPENOCD) \
		-c init \
		-c 'reset halt' \
		-c 'mww 0x40023C04 0x45670123' \
		-c 'mww 0x40023C04 0xCDEF89AB' \
		-c 'mww 0x40023C10 0x00000001' -c 'sleep 10' \
		-c 'mwb 0x1fff7800 0xFE' -c 'sleep 10' \
		-c 'mwb 0x1fff7801 0x01' -c 'sleep 10' \
		-c 'mwb 0x1fff7802 0x01' -c 'sleep 10' \
		-c 'mwb 0x1fff7803 3' -c 'sleep 10' \
		-c 'mwb 0x1fff7804 6' -c 'sleep 10' \
		-c 'mwb 0x1fff7805 56' -c 'sleep 10' \
		-c 'reset run' \
		-c exit
	@echo "OK"
else
	@echo "The one-time programmable memory can only be"
	@echo "written ONCE on every board (what a surprise)."
	@echo "If you're on an ODrive v3.5 or later we already did this for you."
	@echo "Otherwise, if you're mentally ready for this irreversible action,"
	@echo "take the following steps:"
	@echo " 1. open the Makefile and look at the write_otp target"
	@echo " 2. understand the structure of the OTP"
	@echo " 3. edit the bytes that are written to match your board version"
	@echo "Run this command again, this time with OTP_CONFIRM=TRUE appended"
	@echo "to the command in the terminal"
endif

clean:
	-rm -fR .dep $(BUILD_DIR)

.PHONY: all flash gdb dfu bmp clean erase_config

