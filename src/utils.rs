// RTDE Utility functions for byte manipulation

use std::fmt::Write;

#[derive(Debug)]
pub struct RTDEControlHeader {
    pub msg_size: u16,
    pub msg_cmd: u8,
}

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

pub fn get_u64(data: &[u8], message_offset: &mut u32) -> u64 {
    let result = u64::from_be_bytes([
        data[*message_offset as usize],
        data[*message_offset as usize + 1],
        data[*message_offset as usize + 2],
        data[*message_offset as usize + 3],
        data[*message_offset as usize + 4],
        data[*message_offset as usize + 5],
        data[*message_offset as usize + 6],
        data[*message_offset as usize + 7],
    ]);
    *message_offset += 8;
    result
}

pub fn get_u32(data: &[u8], message_offset: &mut u32) -> u32 {
    let result = u32::from_be_bytes([
        data[*message_offset as usize],
        data[*message_offset as usize + 1],
        data[*message_offset as usize + 2],
        data[*message_offset as usize + 3],
    ]);
    *message_offset += 4;
    result
}

pub fn get_u16(data: &[u8], message_offset: &mut u32) -> u16 {
    let result =
        u16::from_be_bytes([data[*message_offset as usize], data[*message_offset as usize + 1]]);
    *message_offset += 2;
    result
}

pub fn get_u8(data: &[u8], message_offset: &mut u32) -> u8 {
    let result = data[*message_offset as usize];
    *message_offset += 1;
    result
}

pub fn get_double(data: &[u8], message_offset: &mut u32) -> f64 {
    let result = f64::from_be_bytes([
        data[*message_offset as usize],
        data[*message_offset as usize + 1],
        data[*message_offset as usize + 2],
        data[*message_offset as usize + 3],
        data[*message_offset as usize + 4],
        data[*message_offset as usize + 5],
        data[*message_offset as usize + 6],
        data[*message_offset as usize + 7],
    ]);
    *message_offset += 8;
    result
}

pub fn get_i32(data: &[u8], message_offset: &mut u32) -> i32 {
    let result = i32::from_be_bytes([
        data[*message_offset as usize],
        data[*message_offset as usize + 1],
        data[*message_offset as usize + 2],
        data[*message_offset as usize + 3],
    ]);
    *message_offset += 4;
    result
}

pub fn get_bool(data: &[u8], message_offset: &mut u32) -> bool {
    let result = data[*message_offset as usize] != 0;
    *message_offset += 1;
    result
}

pub fn read_rtde_header(data: &[u8], message_offset: &mut u32) -> RTDEControlHeader {
    RTDEControlHeader {
        msg_size: get_u16(data, message_offset),
        msg_cmd: get_u8(data, message_offset),
    }
}

pub fn unpack_vector3d(packet: &[u8], packet_data_offset: &mut u32) -> Vec<f64> {
    let mut vector_3d = Vec::new();
    for i in 0..3 {
        let d = get_double(packet, packet_data_offset);
        vector_3d.push(d);
    }
    vector_3d
}

pub fn unpack_vector6d(packet: &[u8], packet_data_offset: &mut u32) -> Vec<f64> {
    let mut vector_6d = Vec::new();
    for i in 0..6 {
        let d = get_double(packet, packet_data_offset);
        vector_6d.push(d);
    }
    vector_6d
}

pub fn unpack_vector6_i32(data: &[u8], message_offset: &mut u32) -> Vec<i32> {
    let mut vector_6_int32 = Vec::with_capacity(6);
    for _ in 0..6 {
        let int32_value = get_i32(data, message_offset);
        vector_6_int32.push(int32_value);
    }
    vector_6_int32
}

pub fn pack_int32(value: i32) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(4);
    bytes.push((value >> 24) as u8);
    bytes.push((value >> 16) as u8);
    bytes.push((value >> 8) as u8);
    bytes.push(value as u8);
    bytes
}

pub fn pack_uint32(value: u32) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(4);
    bytes.push((value >> 24) as u8);
    bytes.push((value >> 16) as u8);
    bytes.push((value >> 8) as u8);
    bytes.push(value as u8);
    bytes
}

pub fn pack_vector_n_int32(vector: Vec<i32>) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(vector.len() * 4);
    for value in vector {
        bytes.extend(pack_int32(value));
    }
    bytes
}

pub fn pack_vector_n_double(vector: Vec<f64>) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(vector.len() * 8);
    for value in vector {
        bytes.extend(pack_double(value));
    }
    bytes
}

pub fn pack_double(value: f64) -> Vec<u8> {
    let mut output: Vec<u8> = Vec::with_capacity(8);
    let bytes: [u8; 8] = unsafe { std::mem::transmute(value) };

    for &byte in bytes.iter().rev() {
        output.push(byte);
    }
    output
}
