use bluer::gatt::{WriteOp, remote::CharacteristicWriteRequest};
use std::fmt;
use std::io::{self, Read, Write};
use std::thread;
use std::time::Duration;

#[cfg(target_os = "linux")]
use crate::packet::{NiimbotPacket, PacketError};
use image::{DynamicImage, GrayImage, Luma, imageops};

const PACKET_READ_SIZE: usize = 1024;
const TRANSCEIVE_ATTEMPTS: usize = 6;
const TRANSCEIVE_DELAY: Duration = Duration::from_millis(100);
const END_PRINT_SETTLE_DELAY: Duration = Duration::from_millis(300);

pub trait Transport {
    /// Reads up to `length` bytes from the device transport.
    ///
    /// # Errors
    ///
    /// Returns an I/O error when the underlying transport cannot be read.
    fn read(&mut self, length: usize) -> io::Result<Vec<u8>>;

    /// Writes raw protocol bytes to the device transport.
    ///
    /// # Errors
    ///
    /// Returns an I/O error when the underlying transport cannot be written.
    fn write(&mut self, data: &[u8]) -> io::Result<usize>;
}

impl<T: Transport + ?Sized> Transport for Box<T> {
    fn read(&mut self, length: usize) -> io::Result<Vec<u8>> {
        (**self).read(length)
    }

    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        (**self).write(data)
    }
}

pub struct SerialTransport {
    inner: Box<dyn serialport::SerialPort>,
}

impl SerialTransport {
    /// Opens a serial connection to the printer.
    ///
    /// Passing `"auto"` will attempt to discover a single attached serial port.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when port detection fails or the selected port
    /// cannot be opened.
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

#[cfg(target_os = "linux")]
pub struct BluetoothTransport {
    runtime: tokio::runtime::Runtime,
    characteristic: bluer::gatt::remote::Characteristic,
    notify: std::pin::Pin<Box<dyn tokio_stream::Stream<Item = Vec<u8>> + Send>>,
    read_buf: Vec<u8>,
}

#[cfg(target_os = "linux")]
impl BluetoothTransport {
    /// Opens a BLE GATT Bluetooth connection to the printer.
    ///
    /// # Panics
    ///
    /// Panics if the hardcoded Bluetooth service UUIDs are invalid.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the address is invalid, the Tokio runtime
    /// cannot be created, or the printer cannot be reached.
    pub fn new(address: &str) -> Result<Self, PrinterError> {
        let address = address
            .parse::<bluer::Address>()
            .map_err(|_| PrinterError::InvalidBluetoothAddress(address.to_owned()))?;
        let runtime = tokio::runtime::Runtime::new().map_err(PrinterError::Io)?;

        let mut attempt = 0;
        let (characteristic, notify) = loop {
            match runtime.block_on(async {
                let session = bluer::Session::new().await.map_err(io::Error::other)?;
                let adapter = session.default_adapter().await.map_err(io::Error::other)?;
                let device = adapter.device(address).map_err(io::Error::other)?;
                if !device.is_connected().await.unwrap_or(false) {
                    device.connect().await.map_err(io::Error::other)?;
                }

                let mut target_char = None;
                for service in device.services().await.map_err(io::Error::other)? {
                    if service.uuid().await.unwrap_or_default()
                        == "e7810a71-73ae-499d-8c15-faa9aef0c3f2"
                            .parse::<bluer::Uuid>()
                            .unwrap()
                    {
                        for char in service.characteristics().await.map_err(io::Error::other)? {
                            if char.uuid().await.unwrap_or_default()
                                == "bef8d6c9-9c21-4c9e-b632-bd58c1009f9f"
                                    .parse::<bluer::Uuid>()
                                    .unwrap()
                            {
                                target_char = Some(char);
                            }
                        }
                    }
                }

                let characteristic = target_char.ok_or_else(|| {
                    io::Error::new(
                        io::ErrorKind::NotFound,
                        "Niimbot GATT characteristic not found",
                    )
                })?;
                let notify_stream = characteristic.notify().await.map_err(io::Error::other)?;

                let notify: std::pin::Pin<Box<dyn tokio_stream::Stream<Item = Vec<u8>> + Send>> =
                    Box::pin(notify_stream);
                Ok::<_, io::Error>((characteristic, notify))
            }) {
                Ok(res) => break res,
                Err(err) => {
                    attempt += 1;
                    if attempt >= 5 {
                        return Err(PrinterError::Io(err));
                    }
                    thread::sleep(Duration::from_millis(500));
                }
            }
        };

        Ok(Self {
            runtime,
            characteristic,
            notify,
            read_buf: Vec::new(),
        })
    }
}

#[cfg(target_os = "linux")]
impl Transport for BluetoothTransport {
    fn read(&mut self, length: usize) -> io::Result<Vec<u8>> {
        use tokio_stream::StreamExt;

        if !self.read_buf.is_empty() {
            let read_len = self.read_buf.len().min(length);
            let result: Vec<u8> = self.read_buf.drain(..read_len).collect();
            return Ok(result);
        }

        let runtime = &self.runtime;
        let notify = &mut self.notify;
        let data = runtime
            .block_on(async { notify.next().await })
            .ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::ConnectionAborted,
                    "GATT notification stream ended",
                )
            })?;

        if data.len() <= length {
            Ok(data)
        } else {
            self.read_buf = data[length..].to_vec();
            Ok(data[..length].to_vec())
        }
    }

    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        let runtime = &self.runtime;
        let characteristic = &self.characteristic;

        let chunk_size = 20;
        for chunk in data.chunks(chunk_size) {
            runtime
                .block_on(async {
                    characteristic
                        .write_ext(
                            chunk,
                            &CharacteristicWriteRequest {
                                op_type: WriteOp::Command,
                                ..Default::default()
                            },
                        )
                        .await
                })
                .map_err(io::Error::other)?;
        }

        // Pace per row (packet), not per chunk!
        std::thread::sleep(std::time::Duration::from_millis(15));

        Ok(data.len())
    }
}

#[cfg(not(target_os = "linux"))]
pub struct BluetoothTransport;

#[cfg(not(target_os = "linux"))]
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
    PrinterStatusData = 165,
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

    /// Initializes connection with the printer by sending handshake packets.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the printer does not acknowledge the
    /// commands or the transport exchange fails.
    pub fn init_connection(&mut self) -> Result<(), PrinterError> {
        let init_packet = [0x03, 0x55, 0x55, 0xc1, 0x01, 0x01, 0xc1, 0xaa, 0xaa];
        self.transport.write(&init_packet)?;
        thread::sleep(Duration::from_millis(200));

        let _ = self.send(&NiimbotPacket::new(
            RequestCode::PrinterStatusData as u8,
            vec![0x01],
        ));
        thread::sleep(Duration::from_millis(50));
        let _ = self.send(&NiimbotPacket::new(RequestCode::GetInfo as u8, vec![0x08]));
        thread::sleep(Duration::from_millis(50));
        let _ = self.send(&NiimbotPacket::new(
            RequestCode::Heartbeat as u8,
            vec![0x04],
        ));
        thread::sleep(Duration::from_millis(50));

        Ok(())
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    /// Sends the image to the printer using the requested density.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the image dimensions exceed the protocol
    /// limits, command exchange fails, or the transport reports an I/O error.
    pub fn print_image(&mut self, image: &DynamicImage, density: u8) -> Result<(), PrinterError> {
        let image_height = u16::try_from(image.height())
            .map_err(|_| PrinterError::ImageTooLarge("image height exceeds printer limits"))?;
        let image_width = u16::try_from(image.width())
            .map_err(|_| PrinterError::ImageTooLarge("image width exceeds printer limits"))?;

        // Initialize BLE connection (required for some devices like B1/D110 over BLE)
        let _ = self.init_connection();

        let label_type = self.get_rfid()?.map_or(1, |rfid_info| rfid_info.label_type);

        with_command_context("set label density", self.set_label_density(density))?;
        with_command_context("set label type", self.set_label_type(label_type))?;
        with_command_context("start print", self.start_print())?;

        with_command_context("start page print", self.start_page_print())?;
        with_command_context(
            "set dimension",
            self.set_dimension(image_height, image_width),
        )?;
        with_command_context("set quantity", self.set_quantity(1))?;
        for packet in self.encode_image(image)? {
            self.send(&packet)?;
        }
        with_command_context("end page print", self.end_page_print())?;

        // Wait for the printer to finish printing the page so it doesn't abort
        // the feed-out by receiving EndPrint too early.
        let start_wait = std::time::Instant::now();
        loop {
            if start_wait.elapsed() > std::time::Duration::from_secs(10) {
                return Err(PrinterError::Timeout);
            }
            if let Ok(status) = self.get_print_status()
                && status.page >= 1
            {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(150));
        }

        thread::sleep(END_PRINT_SETTLE_DELAY);
        with_command_context("end print", self.end_print())?;
        Ok(())
    }

    /// Converts an image into the line packets expected by the printer.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the image width or row index exceeds the
    /// packet format's numeric limits.
    pub fn encode_image(&self, image: &DynamicImage) -> Result<Vec<NiimbotPacket>, PrinterError> {
        let mut img = grayscale_to_binary(image);
        imageops::invert(&mut img);
        let width = usize::try_from(img.width())
            .map_err(|_| PrinterError::ImageTooLarge("image width exceeds platform limits"))?;
        let height = img.height();
        let bytes_per_row = width.div_ceil(8);

        let packets = (0..height)
            .map(|y| {
                let mut line_data = vec![0_u8; bytes_per_row];
                let mut popcount = 0u16;
                for x in 0..width {
                    let x_u32 = u32::try_from(x).map_err(|_| {
                        PrinterError::ImageTooLarge("image width exceeds u32 coordinates")
                    })?;
                    let pixel = img.get_pixel(x_u32, y)[0];
                    if pixel != 0 {
                        let byte_index = x / 8;
                        let bit_index = 7 - (x % 8);
                        line_data[byte_index] |= 1 << bit_index;
                        popcount += 1;
                    }
                }

                let mut payload = Vec::with_capacity(7 + line_data.len());
                payload.extend_from_slice(
                    &u16::try_from(y)
                        .map_err(|_| PrinterError::ImageTooLarge("image height exceeds u16"))?
                        .to_be_bytes(),
                );

                // Header format: y(2 bytes, big-endian), 0, total(2 bytes, little-endian), run(1)
                payload.push(0);
                payload.push((popcount & 0xff) as u8);
                payload.push((popcount >> 8) as u8);
                payload.push(1);

                payload.extend_from_slice(&line_data);
                Ok(NiimbotPacket::new(0x85, payload))
            })
            .collect::<Result<Vec<_>, PrinterError>>()?;

        Ok(packets)
    }

    /// Reads a printer information field.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the printer times out, returns malformed
    /// data, or the transport fails.
    pub fn get_info(&mut self, key: InfoKey) -> Result<Option<InfoValue>, PrinterError> {
        let Some(response) = self.transceive(RequestCode::GetInfo, &[key as u8], key as u8)? else {
            return Ok(None);
        };

        let value = match key {
            InfoKey::DeviceSerial => InfoValue::DeviceSerial(hex_lower(response.data())),
            InfoKey::SoftVersion | InfoKey::HardVersion => {
                #[allow(clippy::cast_precision_loss)]
                {
                    InfoValue::Version(packet_data_to_u32(response.data()) as f32 / 100.0)
                }
            }
            _ => InfoValue::Integer(packet_data_to_u32(response.data())),
        };

        Ok(Some(value))
    }

    /// Reads RFID metadata from the loaded label.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the printer response is malformed, the
    /// transport fails, or the request times out.
    pub fn get_rfid(&mut self) -> Result<Option<RfidInfo>, PrinterError> {
        let Some(packet) = self.transceive(RequestCode::GetRfid, &[0x01], 1)? else {
            return Ok(None);
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

    /// Reads the current heartbeat payload from the printer.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the request times out or the transport
    /// exchange fails.
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

    /// Updates the label type configured on the printer.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the provided label type is invalid, the
    /// printer times out, or the transport exchange fails.
    pub fn set_label_type(&mut self, value: u8) -> Result<bool, PrinterError> {
        if !matches!(value, 1 | 2 | 3 | 4 | 5 | 6 | 10 | 11) {
            return Err(PrinterError::InvalidLabelType(value));
        }
        self.bool_command(RequestCode::SetLabelType, &[value], 16)
    }

    /// Updates the print density configured on the printer.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the density is outside the supported
    /// range, the printer times out, or the transport exchange fails.
    pub fn set_label_density(&mut self, value: u8) -> Result<bool, PrinterError> {
        if !(1..=5).contains(&value) {
            return Err(PrinterError::InvalidDensity(value));
        }
        self.bool_command(RequestCode::SetLabelDensity, &[value], 16)
    }

    /// Starts a print job.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the printer does not acknowledge the
    /// command or the transport exchange fails.
    pub fn start_print(&mut self) -> Result<bool, PrinterError> {
        self.bool_command(RequestCode::StartPrint, &[0x01], 1)
    }

    /// Finalizes a print job.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the printer does not acknowledge the
    /// command or the transport exchange fails.
    pub fn end_print(&mut self) -> Result<bool, PrinterError> {
        self.bool_command(RequestCode::EndPrint, &[0x01], 1)
    }

    /// Starts a page within the current print job.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the printer does not acknowledge the
    /// command or the transport exchange fails.
    pub fn start_page_print(&mut self) -> Result<bool, PrinterError> {
        self.bool_command(RequestCode::StartPagePrint, &[0x01], 1)
    }

    /// Finishes the current page within the print job.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the printer does not acknowledge the
    /// command or the transport exchange fails.
    pub fn end_page_print(&mut self) -> Result<bool, PrinterError> {
        self.bool_command(RequestCode::EndPagePrint, &[0x01], 1)
    }

    /// Allows the printer to clear buffered page data.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the printer does not acknowledge the
    /// command or the transport exchange fails.
    pub fn allow_print_clear(&mut self) -> Result<bool, PrinterError> {
        self.bool_command(RequestCode::AllowPrintClear, &[0x01], 16)
    }

    /// Sets the label dimensions used for the current job.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the printer does not acknowledge the
    /// command or the transport exchange fails.
    pub fn set_dimension(&mut self, width: u16, height: u16) -> Result<bool, PrinterError> {
        let mut payload = Vec::with_capacity(4);
        payload.extend_from_slice(&width.to_be_bytes());
        payload.extend_from_slice(&height.to_be_bytes());
        self.bool_command(RequestCode::SetDimension, &payload, 1)
    }

    /// Sets how many copies the printer should produce.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the printer does not acknowledge the
    /// command or the transport exchange fails.
    pub fn set_quantity(&mut self, quantity: u16) -> Result<bool, PrinterError> {
        self.bool_command(RequestCode::SetQuantity, &quantity.to_be_bytes(), 1)
    }

    /// Reads the current status of the printer's active job.
    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the printer times out, returns malformed
    /// data, or the transport exchange fails.
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

    ///
    /// # Errors
    ///
    /// Returns [`PrinterError`] when the printer does not acknowledge the
    /// command, the response times out, or the transport exchange fails.
    pub fn bool_command(
        &mut self,
        request: RequestCode,
        data: &[u8],
        response_offset: u8,
    ) -> Result<bool, PrinterError> {
        match self.transceive(request, data, response_offset)? {
            Some(packet) => Ok(packet.data().first().is_some_and(|&d| d != 0)),
            None => Err(PrinterError::Timeout),
        }
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
                    219 => return Err(PrinterError::DeviceError(packet.data().to_vec())),
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
        .fold(0_u32, |acc, byte| (acc << 8) | u32::from(*byte))
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

fn with_command_context<T>(
    command: &'static str,
    result: Result<T, PrinterError>,
) -> Result<T, PrinterError> {
    result.map_err(|source| PrinterError::CommandFailed {
        command,
        source: Box::new(source),
    })
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
    MissingBluetoothAddress,
    UnsupportedBluetoothPlatform,
    NoSerialPortsDetected,
    TooManySerialPorts(Vec<String>),
    MalformedResponse(&'static str),
    ImageTooLarge(&'static str),
    Timeout,
    DeviceError(Vec<u8>),
    UnsupportedResponse,
    CommandFailed {
        command: &'static str,
        source: Box<PrinterError>,
    },
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
            Self::MissingBluetoothAddress => {
                write!(f, "bluetooth connection requires --addr with a MAC address")
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
            Self::ImageTooLarge(message) => write!(f, "{message}"),
            Self::Timeout => write!(f, "timed out waiting for printer response"),
            Self::DeviceError(data) => write!(f, "printer reported {}", format_device_error(data)),
            Self::UnsupportedResponse => write!(f, "printer returned an unsupported response"),
            Self::CommandFailed { command, source } => write!(f, "{command} failed: {source}"),
        }
    }
}

fn format_device_error(data: &[u8]) -> String {
    if let Some(code) = data.first() {
        format!("a device error: code {code} ({})", device_error_name(*code))
    } else {
        "a device error without an error code".to_string()
    }
}

fn device_error_name(code: u8) -> &'static str {
    match code {
        3 => "low battery",
        7 => "overheat",
        8 => "paper out",
        9 => "printer busy",
        10 => "no printer head",
        12 => "printer head loose",
        13 => "no ribbon",
        25 => "rotation parameter exception",
        29 => "RFID tag not written",
        34 => "RFID writing not supported",
        52 => "receive data timeout",
        53 => "non-dedicated ribbon",
        _ => "unknown",
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

        let transport = FakeTransport;
        let client = PrinterClient::new(transport);
        let packets = client.encode_image(&image).unwrap();

        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].packet_type(), 0x85);
        assert_eq!(packets[0].data()[..6], [0, 0, 0, 4, 0, 1]);
        assert_eq!(packets[0].data()[6], 0x0f);
    }

    #[derive(Default)]
    struct FakeTransport;

    impl Transport for FakeTransport {
        fn read(&mut self, _length: usize) -> io::Result<Vec<u8>> {
            Ok(Vec::new())
        }

        fn write(&mut self, data: &[u8]) -> io::Result<usize> {
            std::thread::sleep(std::time::Duration::from_millis(10));
            println!("Sending {} bytes", data.len());
            Ok(data.len())
        }
    }
}
