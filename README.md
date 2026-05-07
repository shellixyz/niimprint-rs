# niimprint-rs

`niimprint-rs` is a Rust client library for Niimbot label printers over USB or Bluetooth.

This project is a Rust port of the original Python library by AndBondStyle:
<https://github.com/AndBondStyle/niimprint>

## Features

- Print images to supported Niimbot label printers
- USB serial transport
- Bluetooth transport on Linux
- Small CLI for testing and local printing workflows

## Supported models

The current CLI includes support for:

- `B1`
- `B18`
- `B21`
- `D11`
- `D110`

## Usage

Add the crate to your project:

```toml
[dependencies]
niimprint-rs = "0.1.0"
```

The repository also includes a small CLI binary:

```bash
cargo run --bin niimprint -- --help
```

Example:

```bash
cargo run --bin niimprint -- \
  --model b21 \
  --conn usb \
  --addr /dev/ttyACM0 \
  --rotate 90 \
  --image label.png
```

## License

MIT.
