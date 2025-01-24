// RTDE Utility functions for byte manipulation

use std::fmt::Write;

pub fn double2hexstr(x: f64) -> String {
    // Use a union-like structure with the `transmute` method
    let value: u64 = unsafe { std::mem::transmute(x) };

    // Create a string buffer and format the hex value
    let mut buf = String::new();
    write!(&mut buf, "{:016x}", value).unwrap(); // Format as a zero-padded 16-character hexadecimal
    buf
}

pub fn hex2bytes(hexstr: String) -> String {
    hexstr
}
