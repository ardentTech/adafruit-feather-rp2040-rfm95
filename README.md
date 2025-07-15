# Adafruit Feather RP2040 RFM95
TODO

## Packages
* `bsp`    board support package
* `node_a` remote env monitoring node
* `sht30`  SHT30 temperature+humidity sensor driver

### Dev Profile
Building the `node_a` package with the `dev` release (which is the default) will result in a binary with multicore
functionality that exposes a USB serial device:
1. `$ cargo build -p node_a`
2. `$ cargo run -p node_a`
3. `$ minicom -D /dev/ttyACM0 -b 9600`

### Release Profile
Building+flashing the `node_a` package with the `release` profile will result in a binary with single core functionality
and NO exposed USB serial device:
1. `$ cargo build --release -p node_a`
2. `$ cargo run --release -p node_a`