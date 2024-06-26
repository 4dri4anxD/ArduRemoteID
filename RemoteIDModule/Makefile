# Makefile using arduino-cli

ARDUINO_HOME=$(HOME)/.arduino15
ESP32_VER=2.0.3
ESP32_TOOLS=$(ARDUINO_HOME)/packages/esp32/hardware/esp32/$(ESP32_VER)/tools
ESPTOOL=$(ESP32_TOOLS)/esptool.py

ESP32_FQBN=esp32:esp32:$(CHIP)
SERDEV := $(wildcard /dev/serial/by-id/usb-Espressif_*)
ARDUINO_CLI=../bin/arduino-cli
APP_PARTITION_SIZE=2031616

# ensure python tools are in $PATH
export PATH := $(HOME)/.local/bin:$(PATH)

.PHONY: headers

all: headers aurelia-rid esp32s3dev esp32c3dev bluemark-db200 bluemark-db110 jw-tbd mro-rid jwrid-esp32s3 bluemark-db202 bluemark-db210 bluemark-db203 holybro-RemoteID CUAV-RID

esp32s3dev: CHIP=esp32s3
esp32s3dev: ArduRemoteID-ESP32S3_DEV.bin

esp32c3dev: CHIP=esp32c3
esp32c3dev: ArduRemoteID-ESP32C3_DEV.bin

bluemark-db200: CHIP=esp32c3
bluemark-db200: ArduRemoteID-BLUEMARK_DB200.bin

bluemark-db110: CHIP=esp32c3
bluemark-db110: ArduRemoteID-BLUEMARK_DB110.bin

jw-tbd: CHIP=esp32s3
jw-tbd: ArduRemoteID-JW_TBD.bin

jwrid-esp32s3: CHIP=esp32s3
jwrid-esp32s3: ArduRemoteID-JWRID_ESP32S3.bin

mro-rid: CHIP=esp32c3
mro-rid: ArduRemoteID-MRO_RID.bin

bluemark-db202: CHIP=esp32c3
bluemark-db202: ArduRemoteID-BLUEMARK_DB202.bin

bluemark-db210: CHIP=esp32s3
bluemark-db210: ArduRemoteID-BLUEMARK_DB210.bin

bluemark-db203: CHIP=esp32c3
bluemark-db203: ArduRemoteID-BLUEMARK_DB203.bin

holybro-RemoteID: CHIP=esp32c3
holybro-RemoteID: ArduRemoteID-Holybro_RemoteID.bin

CUAV-RID: CHIP=esp32s3
CUAV-RID: ArduRemoteID-CUAV_RID.bin

aurelia-rid: CHIP=esp32c3
aurelia-rid: ArduRemoteID-AURELIA_RID.bin

setup:
	@echo "Installing ESP32 support"
	$(ARDUINO_CLI) core update-index --config-file arduino-cli.yaml
	$(ARDUINO_CLI) core install esp32:esp32@$(ESP32_VER)
	$(ARDUINO_CLI) lib install "Adafruit NeoPixel"

gitversion:
	@../scripts/git-version.sh

headers: gitversion
	@cd .. && scripts/regen_headers.sh

romfs_files.h: web/*.html web/js/*.js web/styles/*css web/images/*.jpg web/images/*.png public_keys/*.dat
	@../scripts/make_romfs.py romfs_files.h web/*.html web/js/*.js web/styles/*css web/images/*.jpg web/images/*.png public_keys/*.dat

ArduRemoteID-%.bin: gitversion *.cpp *.ino *.h romfs_files.h
	@echo "Building $* on $(CHIP)"
	@BUILD_FLAGS="-DBOARD_$*"
	@rm -rf build build-$*
	@$(ARDUINO_CLI) compile -b esp32:esp32:$(CHIP) --export-binaries --build-property build.extra_flags="-DBOARD_$* -DESP32" --build-property upload.maximum_size=$(APP_PARTITION_SIZE)
	@cp build/esp32.esp32.$(CHIP)/RemoteIDModule.ino.bin ArduRemoteID_$*_OTA.bin
	@echo "Merging $*"
	@python3 $(ESPTOOL) --chip $(CHIP) merge_bin -o ArduRemoteID-$*.bin --flash_size 4MB 0x0 build/esp32.esp32.$(CHIP)/RemoteIDModule.ino.bootloader.bin 0x8000 build/esp32.esp32.$(CHIP)/RemoteIDModule.ino.partitions.bin 0xe000 $(ESP32_TOOLS)/partitions/boot_app0.bin 0x10000 build/esp32.esp32.$(CHIP)/RemoteIDModule.ino.bin
	@mv build build-$*

boards:
	@echo "Listing boards"
	@$(ARDUINO_CLI) board list

checkdev:
	@[ "${SERDEV}" ] && echo "Using device $(SERDEV)" || ( echo "Failed to find serial device"; exit 1 )

upload-%: checkdev
	@echo "Flashing ArduRemoteID-$*.bin"
	@python3 $(ESPTOOL) --port $(SERDEV) write_flash 0x0 ArduRemoteID-$*.bin

uploadota-%: checkdev
	@echo "Flashing ArduRemoteID-$*_OTA.bin"
	@python3 $(ESPTOOL) --port $(SERDEV) write_flash 0x10000 ArduRemoteID_$*_OTA.bin

upload: upload-ESP32S3_DEV

submodule_update:
	git submodule update --recursive --force --init

clean:
	rm -rf ..esp32* *.bin build* romfs_files.h

