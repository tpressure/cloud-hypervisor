// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

use std::io::{self, Read, Write};

const MAGIC: [u8; 4] = *b"VHID";
const VERSION: u8 = 1;
const RECORD_SIZE: usize = 16;
const PAYLOAD_SIZE: usize = 8;

const KIND_KEY: u8 = 1;
const KIND_MOUSE: u8 = 2;
const KIND_RELEASE_ALL: u8 = 3;
const KIND_KEYBOARD_LEDS: u8 = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InputEvent {
    Key {
        usage: u8,
        pressed: bool,
    },
    Mouse {
        dx: i16,
        dy: i16,
        wheel: i16,
        buttons: u8,
    },
    ReleaseAll,
    KeyboardLeds {
        num_lock: bool,
        caps_lock: bool,
        scroll_lock: bool,
    },
}

impl InputEvent {
    pub fn encode(self) -> [u8; RECORD_SIZE] {
        let mut record = [0; RECORD_SIZE];
        record[..4].copy_from_slice(&MAGIC);
        record[4] = VERSION;
        let (kind, length) = match self {
            Self::Key { usage, pressed } => {
                record[8] = usage;
                record[9] = u8::from(pressed);
                (KIND_KEY, 2)
            }
            Self::Mouse {
                dx,
                dy,
                wheel,
                buttons,
            } => {
                record[8..10].copy_from_slice(&dx.to_le_bytes());
                record[10..12].copy_from_slice(&dy.to_le_bytes());
                record[12..14].copy_from_slice(&wheel.to_le_bytes());
                record[14] = buttons;
                (KIND_MOUSE, 7)
            }
            Self::ReleaseAll => (KIND_RELEASE_ALL, 0),
            Self::KeyboardLeds {
                num_lock,
                caps_lock,
                scroll_lock,
            } => {
                record[8] =
                    u8::from(num_lock) | (u8::from(caps_lock) << 1) | (u8::from(scroll_lock) << 2);
                (KIND_KEYBOARD_LEDS, 1)
            }
        };
        record[5] = kind;
        record[6..8].copy_from_slice(&(length as u16).to_le_bytes());
        record
    }

    pub fn decode(record: &[u8; RECORD_SIZE]) -> io::Result<Self> {
        if record[..4] != MAGIC {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "input protocol magic does not match",
            ));
        }
        if record[4] != VERSION {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("unsupported input protocol version {}", record[4]),
            ));
        }
        let length = usize::from(u16::from_le_bytes([record[6], record[7]]));
        if length > PAYLOAD_SIZE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "input protocol payload is too large",
            ));
        }
        match (record[5], length) {
            (KIND_KEY, 2) if record[9] <= 1 => Ok(Self::Key {
                usage: record[8],
                pressed: record[9] != 0,
            }),
            (KIND_MOUSE, 7) => Ok(Self::Mouse {
                dx: i16::from_le_bytes([record[8], record[9]]),
                dy: i16::from_le_bytes([record[10], record[11]]),
                wheel: i16::from_le_bytes([record[12], record[13]]),
                buttons: record[14],
            }),
            (KIND_RELEASE_ALL, 0) => Ok(Self::ReleaseAll),
            (KIND_KEYBOARD_LEDS, 1) => Ok(Self::KeyboardLeds {
                num_lock: record[8] & 1 != 0,
                caps_lock: record[8] & 2 != 0,
                scroll_lock: record[8] & 4 != 0,
            }),
            _ => Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "invalid input protocol record kind={} length={length}",
                    record[5]
                ),
            )),
        }
    }
}

pub fn write_event<W: Write>(writer: &mut W, event: InputEvent) -> io::Result<()> {
    writer.write_all(&event.encode())
}

pub fn read_event<R: Read>(reader: &mut R) -> io::Result<Option<InputEvent>> {
    let mut record = [0; RECORD_SIZE];
    let mut offset = 0;
    while offset != record.len() {
        match reader.read(&mut record[offset..]) {
            Ok(0) if offset == 0 => return Ok(None),
            Ok(0) => {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "input protocol record was truncated",
                ));
            }
            Ok(length) => offset += length,
            Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
            Err(error) => return Err(error),
        }
    }
    InputEvent::decode(&record).map(Some)
}

#[cfg(test)]
mod tests {
    use std::io::Cursor;

    use super::*;

    #[test]
    fn events_round_trip_through_fixed_records() {
        let events = [
            InputEvent::Key {
                usage: 0xe1,
                pressed: true,
            },
            InputEvent::Mouse {
                dx: -300,
                dy: 400,
                wheel: -2,
                buttons: 5,
            },
            InputEvent::ReleaseAll,
            InputEvent::KeyboardLeds {
                num_lock: true,
                caps_lock: false,
                scroll_lock: true,
            },
        ];
        let mut bytes = Vec::new();
        for event in events {
            write_event(&mut bytes, event).unwrap();
        }
        let mut cursor = Cursor::new(bytes);
        for event in events {
            assert_eq!(read_event(&mut cursor).unwrap(), Some(event));
        }
        assert_eq!(read_event(&mut cursor).unwrap(), None);
    }

    #[test]
    fn rejects_bad_magic_version_and_truncation() {
        let mut record = InputEvent::ReleaseAll.encode();
        record[0] = 0;
        InputEvent::decode(&record).unwrap_err();
        record = InputEvent::ReleaseAll.encode();
        record[4] = VERSION + 1;
        InputEvent::decode(&record).unwrap_err();
        read_event(&mut Cursor::new(&record[..5])).unwrap_err();
    }
}
