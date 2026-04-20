use std::fmt;

const PACKET_HEADER: [u8; 2] = [0x55, 0x55];
const PACKET_FOOTER: [u8; 2] = [0xAA, 0xAA];

#[derive(Clone, PartialEq, Eq)]
pub struct NiimbotPacket {
    packet_type: u8,
    data: Vec<u8>,
}

impl NiimbotPacket {
    pub fn new(packet_type: u8, data: impl Into<Vec<u8>>) -> Self {
        Self {
            packet_type,
            data: data.into(),
        }
    }

    #[must_use]
    pub fn packet_type(&self) -> u8 {
        self.packet_type
    }

    #[must_use]
    pub fn data(&self) -> &[u8] {
        &self.data
    }

    #[must_use]
    pub fn into_data(self) -> Vec<u8> {
        self.data
    }

    /// Parses a protocol packet from its raw wire representation.
    ///
    /// # Errors
    ///
    /// Returns [`PacketError`] when the packet header, footer, checksum, or
    /// declared payload length does not match the provided bytes.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, PacketError> {
        if bytes.len() < 7 {
            return Err(PacketError::TooShort(bytes.len()));
        }
        if bytes[..2] != PACKET_HEADER {
            return Err(PacketError::InvalidHeader);
        }
        if bytes[bytes.len() - 2..] != PACKET_FOOTER {
            return Err(PacketError::InvalidFooter);
        }

        let packet_type = bytes[2];
        let data_len = bytes[3] as usize;
        let expected_len = data_len + 7;
        if bytes.len() != expected_len {
            return Err(PacketError::InvalidLength {
                declared: data_len,
                actual: bytes.len(),
            });
        }

        let data = &bytes[4..4 + data_len];
        let expected_checksum = checksum(packet_type, data);
        let actual_checksum = bytes[bytes.len() - 3];
        if expected_checksum != actual_checksum {
            return Err(PacketError::InvalidChecksum {
                expected: expected_checksum,
                actual: actual_checksum,
            });
        }

        Ok(Self::new(packet_type, data.to_vec()))
    }

    /// Serializes this packet into the wire format expected by the printer.
    ///
    /// # Panics
    ///
    /// Panics if the payload is larger than the protocol's one-byte length
    /// field can encode.
    #[must_use]
    pub fn to_bytes(&self) -> Vec<u8> {
        let data_len = u8::try_from(self.data.len()).expect("packet data exceeds protocol limit");
        let mut out = Vec::with_capacity(self.data.len() + 7);
        out.extend_from_slice(&PACKET_HEADER);
        out.push(self.packet_type);
        out.push(data_len);
        out.extend_from_slice(&self.data);
        out.push(checksum(self.packet_type, &self.data));
        out.extend_from_slice(&PACKET_FOOTER);
        out
    }
}

impl fmt::Debug for NiimbotPacket {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("NiimbotPacket")
            .field("packet_type", &self.packet_type)
            .field("data", &self.data)
            .finish()
    }
}

fn checksum(packet_type: u8, data: &[u8]) -> u8 {
    let data_len = u8::try_from(data.len()).expect("packet data exceeds protocol limit");
    let mut checksum = packet_type ^ data_len;
    for byte in data {
        checksum ^= byte;
    }
    checksum
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PacketError {
    TooShort(usize),
    InvalidHeader,
    InvalidFooter,
    InvalidLength { declared: usize, actual: usize },
    InvalidChecksum { expected: u8, actual: u8 },
}

impl fmt::Display for PacketError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooShort(len) => write!(f, "packet too short: {len} bytes"),
            Self::InvalidHeader => write!(f, "invalid packet header"),
            Self::InvalidFooter => write!(f, "invalid packet footer"),
            Self::InvalidLength { declared, actual } => {
                write!(
                    f,
                    "invalid packet length: declared {declared}, actual {actual}"
                )
            }
            Self::InvalidChecksum { expected, actual } => {
                write!(
                    f,
                    "invalid packet checksum: expected 0x{expected:02x}, got 0x{actual:02x}"
                )
            }
        }
    }
}

impl std::error::Error for PacketError {}

#[cfg(test)]
mod tests {
    use super::NiimbotPacket;

    #[test]
    fn packet_round_trip() {
        let packet = NiimbotPacket::new(0x40, vec![0x01, 0x02, 0x03]);
        let bytes = packet.to_bytes();
        let decoded = NiimbotPacket::from_bytes(&bytes).unwrap();
        assert_eq!(decoded, packet);
    }
}
