use std::{collections::VecDeque, io};

use image::{DynamicImage, GrayImage, Luma};
use niimprint_rs::{NiimbotPacket, PrinterClient, Transport};

// Replays the D110_M startup metadata and successful control responses. Physical
// testing established that the legacy payloads are ACKed but print blank on boot.
struct PrinterReplay {
    status: Vec<u8>,
    ble: bool,
    coalesce: bool,
    replies: VecDeque<Vec<u8>>,
    sent: Vec<NiimbotPacket>,
}

impl PrinterReplay {
    fn new(status: Vec<u8>, ble: bool) -> Self {
        Self {
            status,
            ble,
            coalesce: false,
            replies: VecDeque::new(),
            sent: Vec::new(),
        }
    }

    fn payload(&self, code: u8) -> &[u8] {
        self.sent
            .iter()
            .find(|packet| packet.packet_type() == code)
            .expect("command was sent")
            .data()
    }
}

impl Transport for PrinterReplay {
    fn read(&mut self, _: usize) -> io::Result<Vec<u8>> {
        if self.coalesce && !self.replies.is_empty() {
            return Ok(self.replies.drain(..).flatten().collect());
        }
        self.replies
            .pop_front()
            .ok_or_else(|| io::Error::other("unexpected read"))
    }

    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        let bytes = if data.starts_with(&[3, 0x55, 0x55]) {
            &data[1..]
        } else {
            data
        };
        let packet = NiimbotPacket::from_bytes(bytes).map_err(io::Error::other)?;
        let response = match packet.packet_type() {
            0xc1 => Some((0xc2, vec![3])),
            0xa5 => Some((0xb5, self.status.clone())),
            0x40 => Some((0x48, vec![9, 16])),
            0xdc => Some((0xd9, vec![0; 11])),
            0x1a => Some((0x1b, vec![0])), // No RFID: default gap label type.
            0x21 => Some((0x31, vec![1])),
            0x23 => Some((0x33, vec![1])),
            0x01 => Some((0x02, vec![1])),
            0x03 => Some((0x04, vec![1])),
            0x13 => Some((0x14, vec![1, 0])),
            0x15 => Some((0x16, vec![1])),
            0x85 => None,
            0xe3 => Some((0xe4, vec![1])),
            0xa3 => Some((0xb3, vec![0, 3, 100, 100, 0, 0, 0, 0])),
            0xf3 => Some((0xf4, vec![1])),
            code => return Err(io::Error::other(format!("unexpected command {code:02x}"))),
        };
        self.sent.push(packet);
        if let Some((code, payload)) = response {
            self.replies
                .push_back(NiimbotPacket::new(code, payload).to_bytes());
        }
        Ok(data.len())
    }

    fn requires_ble_handshake(&self) -> bool {
        self.ble
    }
}

fn label() -> DynamicImage {
    DynamicImage::ImageLuma8(GrayImage::from_pixel(96, 320, Luma([0])))
}

#[test]
fn cold_d110m_job_declares_copies_in_start_and_page_size() {
    let status = vec![
        0x30, 0x30, 0x23, 0x28, 0, 0xc8, 0, 0, 0, 10, 0, 3, 1, 0x12, 0xee, 0,
    ];
    let mut printer = PrinterClient::new(PrinterReplay::new(status, true));
    printer.print_image(&label(), 3, 3).unwrap();
    let transport = printer.transport_mut();
    assert_eq!(
        (transport.payload(0x01), transport.payload(0x13)),
        (
            &[0, 3, 0, 0, 0, 0, 0, 0, 0][..],
            &[1, 64, 0, 96, 0, 3, 0, 0, 0, 0, 0, 0, 0][..]
        )
    );
}

#[test]
fn older_ble_protocol_keeps_legacy_print_payloads() {
    let mut status = vec![0; 16];
    status[11] = 2;
    status[12] = 4;
    let mut printer = PrinterClient::new(PrinterReplay::new(status, true));
    printer.print_image(&label(), 3, 1).unwrap();
    let transport = printer.transport_mut();
    assert_eq!(
        (transport.payload(0x01), transport.payload(0x13)),
        (&[1][..], &[1, 64, 0, 96][..])
    );
}

#[test]
fn serial_transport_keeps_legacy_print_payloads() {
    let mut printer = PrinterClient::new(PrinterReplay::new(vec![], false));
    printer.print_image(&label(), 3, 1).unwrap();
    let transport = printer.transport_mut();
    assert_eq!(
        (transport.payload(0x01), transport.payload(0x13)),
        (&[1][..], &[1, 64, 0, 96][..])
    );
}

#[test]
fn coalesced_startup_responses_still_select_extended_payloads() {
    let mut status = vec![0; 16];
    status[11] = 3;
    let mut replay = PrinterReplay::new(status, true);
    replay.coalesce = true;
    let mut printer = PrinterClient::new(replay);
    printer.print_image(&label(), 3, 1).unwrap();
    assert_eq!(
        printer.transport_mut().payload(0x13),
        &[1, 64, 0, 96, 0, 1, 0, 0, 0, 0, 0, 0, 0]
    );
}

#[test]
fn short_legacy_status_does_not_select_extended_payloads() {
    let mut printer = PrinterClient::new(PrinterReplay::new(vec![1], true));
    printer.print_image(&label(), 3, 1).unwrap();
    assert_eq!(printer.transport_mut().payload(0x13), &[1, 64, 0, 96]);
}
