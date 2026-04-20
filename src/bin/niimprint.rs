#![warn(clippy::pedantic)]

use std::path::PathBuf;

use clap::{Parser, ValueEnum};
use env_logger::Env;
use image::{DynamicImage, imageops};
use log::warn;
use niimprint_rs::{BluetoothTransport, PrinterClient, PrinterError, SerialTransport, Transport};

#[derive(Debug, Clone, Copy, ValueEnum)]
enum Model {
    B1,
    B18,
    B21,
    D11,
    D110,
}

impl Model {
    fn max_width_px(self) -> u32 {
        match self {
            Self::B1 | Self::B18 | Self::B21 => 384,
            Self::D11 | Self::D110 => 96,
        }
    }

    fn max_density(self) -> u8 {
        match self {
            Self::B18 | Self::D11 | Self::D110 => 3,
            Self::B1 | Self::B21 => 5,
        }
    }
}

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq)]
enum Connection {
    Usb,
    Bluetooth,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum Rotation {
    #[value(name = "0")]
    Deg0,
    #[value(name = "90")]
    Deg90,
    #[value(name = "180")]
    Deg180,
    #[value(name = "270")]
    Deg270,
}

impl Rotation {
    fn apply(self, image: DynamicImage) -> DynamicImage {
        match self {
            Self::Deg0 => image,
            Self::Deg90 => DynamicImage::ImageRgba8(imageops::rotate270(&image)),
            Self::Deg180 => DynamicImage::ImageRgba8(imageops::rotate180(&image)),
            Self::Deg270 => DynamicImage::ImageRgba8(imageops::rotate90(&image)),
        }
    }
}

#[derive(Debug, Parser)]
#[command(name = "niimprint")]
#[command(about = "Niimbot printer test CLI")]
struct Cli {
    #[arg(short = 'm', long, value_enum, default_value_t = Model::B21)]
    model: Model,

    #[arg(short = 'c', long, value_enum, default_value_t = Connection::Usb)]
    conn: Connection,

    #[arg(
        short = 'a',
        long,
        help = "Bluetooth MAC address OR serial device path"
    )]
    addr: Option<String>,

    #[arg(short = 'd', long, default_value_t = 5, value_parser = clap::value_parser!(u8).range(1..=5))]
    density: u8,

    #[arg(short = 'r', long, value_enum, default_value_t = Rotation::Deg0)]
    rotate: Rotation,

    #[arg(short = 'i', long)]
    image: PathBuf,

    #[arg(short = 'v', long)]
    verbose: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();
    init_logging(cli.verbose);

    let mut density = cli.density;
    let max_density = cli.model.max_density();
    if density > max_density {
        warn!(
            "{:?} only supports density up to {}; using {}",
            cli.model, max_density, max_density
        );
        density = max_density;
    }

    let image = image::open(&cli.image)?;
    let image = cli.rotate.apply(image);
    if image.width() > cli.model.max_width_px() {
        return Err(format!(
            "Image width too big for {:?}: {} > {}",
            cli.model,
            image.width(),
            cli.model.max_width_px()
        )
        .into());
    }

    let transport = create_transport(cli.conn, cli.addr)?;
    let mut printer = PrinterClient::new(transport);
    printer.print_image(&image, density)?;
    Ok(())
}

fn init_logging(verbose: bool) {
    let default_level = if verbose { "debug" } else { "info" };
    let env = Env::default().default_filter_or(default_level);
    let mut builder = env_logger::Builder::from_env(env);
    builder.format_timestamp(None);
    let _ = builder.try_init();
}

fn create_transport(
    conn: Connection,
    addr: Option<String>,
) -> Result<Box<dyn Transport>, PrinterError> {
    match conn {
        Connection::Bluetooth => {
            let addr = addr.ok_or(PrinterError::MissingBluetoothAddress)?;
            Ok(Box::new(BluetoothTransport::new(&addr)?))
        }
        Connection::Usb => {
            let port = addr.unwrap_or_else(|| "auto".to_owned());
            Ok(Box::new(SerialTransport::new(&port)?))
        }
    }
}
