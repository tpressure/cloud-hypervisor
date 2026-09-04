// Copyright © 2026 Cyberus Technology GmbH
//
// SPDX-License-Identifier: Apache-2.0

pub mod keyboard;
pub mod mouse;

pub const DESCRIPTOR_DEVICE: u8 = 0x01;
pub const DESCRIPTOR_CONFIGURATION: u8 = 0x02;
pub const DESCRIPTOR_HID: u8 = 0x21;
pub const DESCRIPTOR_REPORT: u8 = 0x22;

pub const REQUEST_TYPE_STANDARD: u8 = 0x00;
pub const REQUEST_TYPE_CLASS: u8 = 0x20;

pub fn device_descriptor(product_id: u16) -> Vec<u8> {
    let [vendor_lo, vendor_hi] = 0x1b36u16.to_le_bytes();
    let [product_lo, product_hi] = product_id.to_le_bytes();
    vec![
        18,
        DESCRIPTOR_DEVICE,
        0x10,
        0x01,
        0,
        0,
        0,
        8,
        vendor_lo,
        vendor_hi,
        product_lo,
        product_hi,
        0x00,
        0x01,
        0,
        0,
        0,
        1,
    ]
}

pub fn configuration_descriptor(protocol: u8, report_length: usize, packet_size: u16) -> Vec<u8> {
    let [report_lo, report_hi] = (report_length as u16).to_le_bytes();
    let [packet_lo, packet_hi] = packet_size.to_le_bytes();
    vec![
        9,
        DESCRIPTOR_CONFIGURATION,
        34,
        0,
        1,
        1,
        0,
        0x80,
        50,
        9,
        0x04,
        0,
        0,
        1,
        0x03,
        0x01,
        protocol,
        0,
        9,
        DESCRIPTOR_HID,
        0x11,
        0x01,
        0,
        1,
        DESCRIPTOR_REPORT,
        report_lo,
        report_hi,
        7,
        0x05,
        0x81,
        0x03,
        packet_lo,
        packet_hi,
        10,
    ]
}

pub fn hid_descriptor(report_length: usize) -> Vec<u8> {
    let [length_lo, length_hi] = (report_length as u16).to_le_bytes();
    vec![
        9,
        DESCRIPTOR_HID,
        0x11,
        0x01,
        0,
        1,
        DESCRIPTOR_REPORT,
        length_lo,
        length_hi,
    ]
}
