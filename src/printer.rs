use std::fmt;
use std::io::{self, Read, Write};
use std::mem::size_of;
use std::thread;
use std::time::Duration;

use image::{DynamicImage, GrayImage, Luma, imageops};

use crate::packet::{NiimbotPacket, PacketError};

const BLUETOOTH_RFCOMM_CHANNEL: u8 = 1;
#[cfg(any(target_os = "linux", target_os = "android"))]
const BLUETOOTH_RFCOMM_PROTOCOL: i32 = 3;
const PACKET_READ_SIZE: usize = 1024;
const TRANSCEIVE_ATTEMPTS: usize = 6;
const TRANSCEIVE_DELAY: Duration = Duration::from_millis(100);
const END_PRINT_SETTLE_DELAY: Duration = Duration::from_millis(300);

pub trait Transport {
    fn read(&mut self, length: usize) -> io::Result<Vec<u8>>;
    fn write(&mut self, data: &[u8]) -> io::Result<usize>;
}

pub struct SerialTransport {
    inner: Box<dyn serialport::SerialPort>,
}

impl SerialTransport {
    pub fn new(port: impl AsRef<str>) -> Result<Self, PrinterError> {
        let port = if port.as_ref() == "auto" {
            Self::detect_port()?
        } else {
            port.as_ref().to_owned()
        };

        let inner = serialport::new(port, 115_200)
            .timeout(Duration::from_millis(500))
            .open()
            .map_err(PrinterError::IoSerial)?;
        Ok(Self { inner })
    }

    fn detect_port() -> Result<String, PrinterError> {
        let ports = serialport::available_ports().map_err(PrinterError::IoSerial)?;
        match ports.as_slice() {
            [] => Err(PrinterError::NoSerialPortsDetected),
            [port] => Ok(port.port_name.clone()),
            many => Err(PrinterError::TooManySerialPorts(
                many.iter()
                    .map(|port| format!("{} ({:?})", port.port_name, port.port_type))
                    .collect(),
            )),
        }
    }
}

impl Transport for SerialTransport {
    fn read(&mut self, length: usize) -> io::Result<Vec<u8>> {
        let mut buf = vec![0_u8; length];
        let read = self.inner.read(&mut buf)?;
        buf.truncate(read);
        Ok(buf)
    }

    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        self.inner.write(data)
    }
}

#[cfg(any(target_os = "linux", target_os = "android"))]
pub struct BluetoothTransport {
    fd: i32,
}

#[cfg(any(target_os = "linux", target_os = "android"))]
impl BluetoothTransport {
    pub fn new(address: &str) -> Result<Self, PrinterError> {
        let mut addr = parse_bluetooth_address(address)?;
        addr.reverse();

        let fd = unsafe {
            libc::socket(
                libc::AF_BLUETOOTH,
                libc::SOCK_STREAM,
                BLUETOOTH_RFCOMM_PROTOCOL,
            )
        };
        if fd < 0 {
            return Err(PrinterError::Io(io::Error::last_os_error()));
        }

        let sockaddr = SockAddrRc {
            rc_family: libc::AF_BLUETOOTH as libc::sa_family_t,
            rc_bdaddr: addr,
            rc_channel: BLUETOOTH_RFCOMM_CHANNEL,
        };

        let connect_result = unsafe {
            libc::connect(
                fd,
                (&sockaddr as *const SockAddrRc).cast::<libc::sockaddr>(),
                size_of::<SockAddrRc>() as libc::socklen_t,
            )
        };
        if connect_result != 0 {
            let err = io::Error::last_os_error();
            unsafe { libc::close(fd) };
            return Err(PrinterError::Io(err));
        }

        Ok(Self { fd })
    }
}

#[cfg(any(target_os = "linux", target_os = "android"))]
impl Drop for BluetoothTransport {
    fn drop(&mut self) {
        unsafe {
            libc::shutdown(self.fd, libc::SHUT_RDWR);
            libc::close(self.fd);
        }
    }
}

#[cfg(any(target_os = "linux", target_os = "android"))]
impl Transport for BluetoothTransport {
    fn read(&mut self, length: usize) -> io::Result<Vec<u8>> {
        let mut buf = vec![0_u8; length];
        let read = unsafe { libc::read(self.fd, buf.as_mut_ptr().cast(), buf.len()) };
        if read < 0 {
            return Err(io::Error::last_os_error());
        }
        buf.truncate(read as usize);
        Ok(buf)
    }

    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        let written = unsafe { libc::write(self.fd, data.as_ptr().cast(), data.len()) };
        if written < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(written as usize)
    }
}

#[cfg(not(any(target_os = "linux", target_os = "android")))]
pub struct BluetoothTransport;

#[cfg(not(any(target_os = "linux", target_os = "android")))]
impl BluetoothTransport {
    pub fn new(_address: &str) -> Result<Self, PrinterError> {
        Err(PrinterError::UnsupportedBluetoothPlatform)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum InfoKey {
    Density = 1,
    PrintSpeed = 2,
    LabelType = 3,
    LanguageType = 6,
    AutoShutdownTime = 7,
    DeviceType = 8,
    SoftVersion = 9,
    Battery = 10,
    DeviceSerial = 11,
    HardVersion = 12,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RequestCode {
    GetInfo = 64,
    GetRfid = 26,
    Heartbeat = 220,
    SetLabelType = 35,
    SetLabelDensity = 33,
    StartPrint = 1,
    EndPrint = 243,
    StartPagePrint = 3,
    EndPagePrint = 227,
    AllowPrintClear = 32,
    SetDimension = 19,
    SetQuantity = 21,
    GetPrintStatus = 163,
}

#[derive(Debug, Clone, PartialEq)]
pub enum InfoValue {
    Integer(u32),
    Version(f32),
    DeviceSerial(String),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RfidInfo {
    pub uuid: String,
    pub barcode: String,
    pub serial: String,
    pub used_len: u16,
    pub total_len: u16,
    pub label_type: u8,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct HeartbeatStatus {
    pub closingstate: Option<u8>,
    pub powerlevel: Option<u8>,
    pub paperstate: Option<u8>,
    pub rfidreadstate: Option<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PrintStatus {
    pub page: u16,
    pub progress1: u8,
    pub progress2: u8,
}

pub struct PrinterClient<T: Transport> {
    transport: T,
    packetbuf: Vec<u8>,
}

impl<T: Transport> PrinterClient<T> {
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            packetbuf: Vec::new(),
        }
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn print_image(&mut self, image: &DynamicImage, density: u8) -> Result<(), PrinterError> {
        self.set_label_density(density)?;
        self.set_label_type(1)?;
        self.start_print()?;
        self.start_page_print()?;
        self.set_dimension(image.height() as u16, image.width() as u16)?;
        for packet in self.encode_image(image)? {
            self.send(&packet)?;
        }
        self.end_page_print()?;
        thread::sleep(END_PRINT_SETTLE_DELAY);
        while !self.end_print()? {
            thread::sleep(TRANSCEIVE_DELAY);
        }
        Ok(())
    }

    pub fn encode_image(&self, image: &DynamicImage) -> Result<Vec<NiimbotPacket>, PrinterError> {
        let mut img = grayscale_to_binary(image);
        imageops::invert(&mut img);
        let width = img.width() as usize;
        let height = img.height();
        let bytes_per_row = width.div_ceil(8);

        let packets = (0..height)
            .map(|y| {
                let mut line_data = vec![0_u8; bytes_per_row];
                for x in 0..width {
                    let pixel = img.get_pixel(x as u32, y)[0];
                    if pixel != 0 {
                        let byte_index = x / 8;
                        let bit_index = 7 - (x % 8);
                        line_data[byte_index] |= 1 << bit_index;
                    }
                }

                let mut payload = Vec::with_capacity(7 + line_data.len());
                payload.extend_from_slice(&(y as u16).to_be_bytes());
                payload.extend_from_slice(&[0, 0, 0, 1]);
                payload.extend_from_slice(&line_data);
                NiimbotPacket::new(0x85, payload)
            })
            .collect::<Vec<_>>();

        Ok(packets)
    }

    pub fn get_info(&mut self, key: InfoKey) -> Result<Option<InfoValue>, PrinterError> {
        let response = match self.transceive(RequestCode::GetInfo, &[key as u8], key as u8)? {
            Some(packet) => packet,
            None => return Ok(None),
        };

        let value = match key {
            InfoKey::DeviceSerial => InfoValue::DeviceSerial(hex_lower(response.data())),
            InfoKey::SoftVersion | InfoKey::HardVersion => {
                InfoValue::Version(packet_data_to_u32(response.data()) as f32 / 100.0)
            }
            _ => InfoValue::Integer(packet_data_to_u32(response.data())),
        };

        Ok(Some(value))
    }

    pub fn get_rfid(&mut self) -> Result<Option<RfidInfo>, PrinterError> {
        let packet = match self.transceive(RequestCode::GetRfid, &[0x01], 1)? {
            Some(packet) => packet,
            None => return Ok(None),
        };
        let data = packet.data();
        if data.is_empty() {
            return Err(PrinterError::MalformedResponse(
                "rfid response missing body",
            ));
        }
        if data[0] == 0 {
            return Ok(None);
        }
        if data.len() < 9 {
            return Err(PrinterError::MalformedResponse("rfid response too short"));
        }

        let uuid = hex_lower(&data[..8]);
        let mut idx = 8;
        let barcode = read_len_prefixed_string(data, &mut idx, "barcode")?;
        let serial = read_len_prefixed_string(data, &mut idx, "serial")?;
        if idx + 5 > data.len() {
            return Err(PrinterError::MalformedResponse("rfid footer too short"));
        }
        let total_len = u16::from_be_bytes([data[idx], data[idx + 1]]);
        let used_len = u16::from_be_bytes([data[idx + 2], data[idx + 3]]);
        let label_type = data[idx + 4];

        Ok(Some(RfidInfo {
            uuid,
            barcode,
            serial,
            used_len,
            total_len,
            label_type,
        }))
    }

    pub fn heartbeat(&mut self) -> Result<HeartbeatStatus, PrinterError> {
        let packet = self
            .transceive(RequestCode::Heartbeat, &[0x01], 1)?
            .ok_or(PrinterError::Timeout)?;
        let data = packet.data();

        let status = match data.len() {
            20 => HeartbeatStatus {
                paperstate: Some(data[18]),
                rfidreadstate: Some(data[19]),
                ..HeartbeatStatus::default()
            },
            13 => HeartbeatStatus {
                closingstate: Some(data[9]),
                powerlevel: Some(data[10]),
                paperstate: Some(data[11]),
                rfidreadstate: Some(data[12]),
            },
            19 => HeartbeatStatus {
                closingstate: Some(data[15]),
                powerlevel: Some(data[16]),
                paperstate: Some(data[17]),
                rfidreadstate: Some(data[18]),
            },
            10 => HeartbeatStatus {
                closingstate: Some(data[8]),
                powerlevel: Some(data[9]),
                rfidreadstate: Some(data[8]),
                paperstate: None,
            },
            9 => HeartbeatStatus {
                closingstate: Some(data[8]),
                ..HeartbeatStatus::default()
            },
            _ => HeartbeatStatus::default(),
        };

        Ok(status)
    }

    pub fn set_label_type(&mut self, value: u8) -> Result<bool, PrinterError> {
        if !(1..=3).contains(&value) {
            return Err(PrinterError::InvalidLabelType(value));
        }
        self.bool_command(RequestCode::SetLabelType, &[value], 16)
    }

    pub fn set_label_density(&mut self, value: u8) -> Result<bool, PrinterError> {
        if !(1..=5).contains(&value) {
            return Err(PrinterError::InvalidDensity(value));
        }
        self.bool_command(RequestCode::SetLabelDensity, &[value], 16)
    }

    pub fn start_print(&mut self) -> Result<bool, PrinterError> {
        self.bool_command(RequestCode::StartPrint, &[0x01], 1)
    }

    pub fn end_print(&mut self) -> Result<bool, PrinterError> {
        self.bool_command(RequestCode::EndPrint, &[0x01], 1)
    }

    pub fn start_page_print(&mut self) -> Result<bool, PrinterError> {
        self.bool_command(RequestCode::StartPagePrint, &[0x01], 1)
    }

    pub fn end_page_print(&mut self) -> Result<bool, PrinterError> {
        self.bool_command(RequestCode::EndPagePrint, &[0x01], 1)
    }

    pub fn allow_print_clear(&mut self) -> Result<bool, PrinterError> {
        self.bool_command(RequestCode::AllowPrintClear, &[0x01], 16)
    }

    pub fn set_dimension(&mut self, width: u16, height: u16) -> Result<bool, PrinterError> {
        let mut payload = Vec::with_capacity(4);
        payload.extend_from_slice(&width.to_be_bytes());
        payload.extend_from_slice(&height.to_be_bytes());
        self.bool_command(RequestCode::SetDimension, &payload, 1)
    }

    pub fn set_quantity(&mut self, quantity: u16) -> Result<bool, PrinterError> {
        self.bool_command(RequestCode::SetQuantity, &quantity.to_be_bytes(), 1)
    }

    pub fn get_print_status(&mut self) -> Result<PrintStatus, PrinterError> {
        let packet = self
            .transceive(RequestCode::GetPrintStatus, &[0x01], 16)?
            .ok_or(PrinterError::Timeout)?;
        if packet.data().len() < 4 {
            return Err(PrinterError::MalformedResponse("print status too short"));
        }
        Ok(PrintStatus {
            page: u16::from_be_bytes([packet.data()[0], packet.data()[1]]),
            progress1: packet.data()[2],
            progress2: packet.data()[3],
        })
    }

    fn bool_command(
        &mut self,
        code: RequestCode,
        payload: &[u8],
        response_offset: u8,
    ) -> Result<bool, PrinterError> {
        let packet = self
            .transceive(code, payload, response_offset)?
            .ok_or(PrinterError::Timeout)?;
        Ok(packet.data().first().copied().unwrap_or_default() != 0)
    }

    fn recv(&mut self) -> Result<Vec<NiimbotPacket>, PrinterError> {
        let data = self.transport.read(PACKET_READ_SIZE)?;
        self.packetbuf.extend_from_slice(&data);

        let mut packets = Vec::new();
        while self.packetbuf.len() > 4 {
            let packet_len = self.packetbuf[3] as usize + 7;
            if self.packetbuf.len() < packet_len {
                break;
            }
            let raw_packet = self.packetbuf.drain(..packet_len).collect::<Vec<_>>();
            let packet = NiimbotPacket::from_bytes(&raw_packet)?;
            packets.push(packet);
        }

        Ok(packets)
    }

    fn send(&mut self, packet: &NiimbotPacket) -> Result<(), PrinterError> {
        self.transport.write(&packet.to_bytes())?;
        Ok(())
    }

    fn transceive(
        &mut self,
        request: RequestCode,
        data: &[u8],
        response_offset: u8,
    ) -> Result<Option<NiimbotPacket>, PrinterError> {
        let response_code = response_offset.wrapping_add(request as u8);
        let packet = NiimbotPacket::new(request as u8, data.to_vec());
        self.send(&packet)?;

        for _ in 0..TRANSCEIVE_ATTEMPTS {
            for packet in self.recv()? {
                match packet.packet_type() {
                    219 => return Err(PrinterError::DeviceError),
                    0 => return Err(PrinterError::UnsupportedResponse),
                    code if code == response_code => return Ok(Some(packet)),
                    _ => {}
                }
            }
            thread::sleep(TRANSCEIVE_DELAY);
        }
        Ok(None)
    }
}

fn grayscale_to_binary(image: &DynamicImage) -> GrayImage {
    let mut grayscale = image.to_luma8();
    for pixel in grayscale.pixels_mut() {
        let value = if pixel[0] > 127 { 255 } else { 0 };
        *pixel = Luma([value]);
    }
    grayscale
}

fn packet_data_to_u32(data: &[u8]) -> u32 {
    data.iter()
        .fold(0_u32, |acc, byte| (acc << 8) | (*byte as u32))
}

fn read_len_prefixed_string(
    data: &[u8],
    idx: &mut usize,
    field: &'static str,
) -> Result<String, PrinterError> {
    if *idx >= data.len() {
        return Err(PrinterError::MalformedResponse(field));
    }
    let len = data[*idx] as usize;
    *idx += 1;
    if *idx + len > data.len() {
        return Err(PrinterError::MalformedResponse(field));
    }
    let value = std::str::from_utf8(&data[*idx..*idx + len])
        .map_err(PrinterError::Utf8)?
        .to_owned();
    *idx += len;
    Ok(value)
}

fn hex_lower(data: &[u8]) -> String {
    let mut out = String::with_capacity(data.len() * 2);
    for byte in data {
        use std::fmt::Write as _;
        let _ = write!(out, "{byte:02x}");
    }
    out
}

#[cfg(any(target_os = "linux", target_os = "android"))]
fn parse_bluetooth_address(address: &str) -> Result<[u8; 6], PrinterError> {
    let parts = address.split(':').collect::<Vec<_>>();
    if parts.len() != 6 {
        return Err(PrinterError::InvalidBluetoothAddress(address.to_owned()));
    }

    let mut bytes = [0_u8; 6];
    for (idx, part) in parts.iter().enumerate() {
        if part.len() != 2 {
            return Err(PrinterError::InvalidBluetoothAddress(address.to_owned()));
        }
        bytes[idx] = u8::from_str_radix(part, 16)
            .map_err(|_| PrinterError::InvalidBluetoothAddress(address.to_owned()))?;
    }
    Ok(bytes)
}

#[cfg(any(target_os = "linux", target_os = "android"))]
#[repr(C)]
struct SockAddrRc {
    rc_family: libc::sa_family_t,
    rc_bdaddr: [u8; 6],
    rc_channel: u8,
}

#[derive(Debug)]
pub enum PrinterError {
    Io(io::Error),
    IoSerial(serialport::Error),
    Packet(PacketError),
    Utf8(std::str::Utf8Error),
    InvalidDensity(u8),
    InvalidLabelType(u8),
    InvalidBluetoothAddress(String),
    UnsupportedBluetoothPlatform,
    NoSerialPortsDetected,
    TooManySerialPorts(Vec<String>),
    MalformedResponse(&'static str),
    Timeout,
    DeviceError,
    UnsupportedResponse,
}

impl fmt::Display for PrinterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(err) => write!(f, "{err}"),
            Self::IoSerial(err) => write!(f, "{err}"),
            Self::Packet(err) => write!(f, "{err}"),
            Self::Utf8(err) => write!(f, "{err}"),
            Self::InvalidDensity(value) => write!(f, "invalid density: {value}"),
            Self::InvalidLabelType(value) => write!(f, "invalid label type: {value}"),
            Self::InvalidBluetoothAddress(value) => {
                write!(f, "invalid bluetooth MAC address: {value}")
            }
            Self::UnsupportedBluetoothPlatform => {
                write!(
                    f,
                    "bluetooth transport is only supported on Linux and Android"
                )
            }
            Self::NoSerialPortsDetected => write!(f, "no serial ports detected"),
            Self::TooManySerialPorts(ports) => {
                write!(f, "multiple serial ports detected: {}", ports.join(", "))
            }
            Self::MalformedResponse(message) => write!(f, "malformed device response: {message}"),
            Self::Timeout => write!(f, "timed out waiting for printer response"),
            Self::DeviceError => write!(f, "printer reported a device error"),
            Self::UnsupportedResponse => write!(f, "printer returned an unsupported response"),
        }
    }
}

impl std::error::Error for PrinterError {}

impl From<io::Error> for PrinterError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

impl From<PacketError> for PrinterError {
    fn from(value: PacketError) -> Self {
        Self::Packet(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::ImageBuffer;

    #[test]
    fn image_encoding_matches_expected_bits() {
        let image = DynamicImage::ImageLuma8(ImageBuffer::from_fn(8, 1, |x, _| {
            if x < 4 { Luma([255]) } else { Luma([0]) }
        }));

        let transport = FakeTransport::default();
        let client = PrinterClient::new(transport);
        let packets = client.encode_image(&image).unwrap();

        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].packet_type(), 0x85);
        assert_eq!(packets[0].data()[..6], [0, 0, 0, 0, 0, 1]);
        assert_eq!(packets[0].data()[6], 0x0f);
    }

    #[derive(Default)]
    struct FakeTransport;

    impl Transport for FakeTransport {
        fn read(&mut self, _length: usize) -> io::Result<Vec<u8>> {
            Ok(Vec::new())
        }

        fn write(&mut self, data: &[u8]) -> io::Result<usize> {
            Ok(data.len())
        }
    }
}
