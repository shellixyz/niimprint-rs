#![warn(clippy::pedantic)]

mod packet;
mod printer;

pub use packet::{NiimbotPacket, PacketError};
pub use printer::{
    BluetoothTransport, HeartbeatStatus, InfoKey, InfoValue, PrintStatus, PrinterClient,
    PrinterError, RequestCode, RfidInfo, SerialTransport, Transport,
};
