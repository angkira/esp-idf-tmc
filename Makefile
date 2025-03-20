build:
	cargo build
flash:
	espflash ./target/xtensa-esp32s3-espidf/debug/esp-stepper
monitor:
	espflash monitor
clean:
	cargo clean
deploy: build flash monitor