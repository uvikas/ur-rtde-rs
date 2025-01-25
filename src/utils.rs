// RTDE Utility functions for byte manipulation

use std::fmt::Write;

pub fn double2hexstr(x: f64) -> String {
    // Union-like reinterpretation of f64 as i64
    let value: i64 = unsafe { std::mem::transmute(x) };

    // Format the value as a hex string with std::fmt
    let mut buf = String::new();
    write!(&mut buf, "{:06x}", value).unwrap();

    buf
}

pub fn hex2bytes(hex: &str) -> Vec<u8> {
    let mut bytes = Vec::new();

    for i in (0..hex.len()).step_by(2) {
        if let Some(byte) = hex.get(i..i + 2) {
            if let Ok(parsed_byte) = u8::from_str_radix(byte, 16) {
                bytes.push(parsed_byte);
            } else {
                // Handle invalid hex input if needed
                panic!("Invalid hex input: {}", byte);
            }
        }
    }

    bytes
}
