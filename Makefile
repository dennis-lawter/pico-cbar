build:
	cargo build --release -Zbuild-std --target thumbv6m-none-eabi
run: build
	echo "Searching for Raspberry Pi Pico..."
	@while [ ! -d "/media/$$USER/RPI-RP2" ]; do \
		sleep 1; \
	done; \
	echo "Raspberry Pi Pico mounted."
	cargo run --release
fmt:
	rustfmt src/*.rs
init:
	rustup target add thumbv6m-none-eabi

