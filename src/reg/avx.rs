/* ============================================================================
 * File:   avx.rs
 * Author: Cole Johnson
 * ============================================================================
 * Copyright (c) 2020 Cole Johnson
 *
 * This file is part of r86.
 *
 * r86 is free software: you can redistribute it and/or modify it under the
 *   terms of the GNU General Public License as published by the Free Software
 *   Foundation, either version 3 of the License, or (at your option) any later
 *   version.
 *
 * r86 is distributed in the hope that it will be useful, but WITHOUT ANY
 *   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 *   FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 *   details.
 *
 * You should have received a copy of the GNU General Public License along with
 *   r86. If not, see <http://www.gnu.org/licenses/>.
 * ============================================================================
 */
// TODO: Lots of repeated code; learn and use macros
// TODO: Implement f16, f32, and f64 accessors
// TODO: Implement saturation functions (should they be here?)
use std::mem;

/// An AVX register.
/// Internally represented by a 512-bit (64-byte) byte array.
pub struct Avx {
    value: [u8; 64],
}

impl Avx {
    pub fn new() -> Avx {
        Avx { value: [0; 64] }
    }

    pub fn clear_zmm(&mut self) {
        self.value = [0; 64];
    }
    pub fn clear_zmm_low_256(&mut self) {
        for b in &mut self.value[0..32] {
            *b = 0;
        }
    }
    pub fn clear_zmm_high_256(&mut self) {
        for b in &mut self.value[32..64] {
            *b = 0;
        }
    }

    pub fn clear_ymm(&mut self) {
        self.clear_zmm_low_256();
    }
    pub fn clear_ymm_low_128(&mut self) {
        for b in &mut self.value[0..16] {
            *b = 0;
        }
    }
    pub fn clear_ymm_high_128(&mut self) {
        for b in &mut self.value[16..32] {
            *b = 0;
        }
    }

    pub fn clear_xmm(&mut self) {
        self.clear_ymm_low_128();
    }
    pub fn clear_xmm_low_64(&mut self) {
        for b in &mut self.value[0..8] {
            *b = 0;
        }
    }
    pub fn clear_xmm_high_64(&mut self) {
        for b in &mut self.value[8..16] {
            *b = 0;
        }
    }

    pub unsafe fn i8(&self, idx: usize) -> i8 {
        assert!(idx < self.value.len());
        mem::transmute::<u8, i8>(self.value[idx])
    }
    pub unsafe fn set_i8(&mut self, idx: usize, val: i8) {
        assert!(idx < self.value.len());
        let val = mem::transmute::<i8, u8>(val);
        self.value[idx] = val;
    }
    pub unsafe fn i8_arr(&self) -> [i8; 64] {
        mem::transmute::<[u8; 64], [i8; 64]>(self.value)
    }
    pub unsafe fn set_i8_arr(&mut self, arr: [i8; 64]) {
        self.value = mem::transmute::<[i8; 64], [u8; 64]>(arr);
    }

    pub fn u8(&self, idx: usize) -> u8 {
        assert!(idx < self.value.len());
        self.value[idx]
    }
    pub fn set_u8(&mut self, idx: usize, val: u8) {
        assert!(idx < self.value.len());
        self.value[idx] = val;
    }
    pub fn u8_arr(&self) -> [u8; 64] {
        self.value
    }
    pub fn set_u8_arr(&mut self, arr: [u8; 64]) {
        self.value = arr;
    }

    pub fn i16(&self, idx: usize) -> i16 {
        let size = mem::size_of::<i16>();
        let idx = idx * size;

        let mut bits: [u8; 2] = [0; 2];
        bits.copy_from_slice(&self.value[idx..idx + size]);
        i16::from_le_bytes(bits)
    }
    pub unsafe fn set_i16(&mut self, idx: usize, val: i16) {
        let size = mem::size_of::<i16>();
        let idx = idx * size;
        assert!(idx < self.value.len());

        let bits = mem::transmute::<i16, [u8; 2]>(val);
        for i in idx..idx + size {
            self.value[i] = bits[i];
        }
    }
    pub unsafe fn i16_arr(&self) -> [i16; 32] {
        mem::transmute::<[u8; 64], [i16; 32]>(self.value)
    }
    pub unsafe fn set_i16_arr(&mut self, arr: [i16; 32]) {
        self.value = mem::transmute::<[i16; 32], [u8; 64]>(arr);
    }

    pub fn u16(&self, idx: usize) -> u16 {
        let size = mem::size_of::<u16>();
        let idx = idx * size;

        let mut bits: [u8; 2] = [0; 2];
        bits.copy_from_slice(&self.value[idx..idx + size]);
        u16::from_le_bytes(bits)
    }
    pub unsafe fn set_u16(&mut self, idx: usize, val: u16) {
        let size = mem::size_of::<i16>();
        let idx = idx * size;
        assert!(idx < self.value.len());

        let bits = mem::transmute::<u16, [u8; 2]>(val);
        for i in idx..idx + size {
            self.value[i] = bits[i];
        }
    }
    pub unsafe fn u16_arr(&self) -> [u16; 32] {
        mem::transmute::<[u8; 64], [u16; 32]>(self.value)
    }
    pub unsafe fn set_u16_arr(&mut self, arr: [u16; 32]) {
        self.value = mem::transmute::<[u16; 32], [u8; 64]>(arr);
    }

    pub fn i32(&self, idx: usize) -> i32 {
        let size = mem::size_of::<i32>();
        let idx = idx * size;

        let mut bits: [u8; 4] = [0; 4];
        bits.copy_from_slice(&self.value[idx..idx + size]);
        i32::from_le_bytes(bits)
    }
    pub unsafe fn set_i32(&mut self, idx: usize, val: i32) {
        let size = mem::size_of::<i16>();
        let idx = idx * size;
        assert!(idx < self.value.len());

        let bits = mem::transmute::<i32, [u8; 4]>(val);
        for i in idx..idx + size {
            self.value[i] = bits[i];
        }
    }
    pub unsafe fn i32_arr(&self) -> [i32; 16] {
        mem::transmute::<[u8; 64], [i32; 16]>(self.value)
    }
    pub unsafe fn set_i32_arr(&mut self, arr: [i32; 16]) {
        self.value = mem::transmute::<[i32; 16], [u8; 64]>(arr);
    }

    pub fn u32(&self, idx: usize) -> u32 {
        let size = mem::size_of::<u32>();
        let idx = idx * size;

        let mut bits: [u8; 4] = [0; 4];
        bits.copy_from_slice(&self.value[idx..idx + size]);
        u32::from_le_bytes(bits)
    }
    pub unsafe fn set_u32(&mut self, idx: usize, val: u32) {
        let size = mem::size_of::<i16>();
        let idx = idx * size;
        assert!(idx < self.value.len());

        let bits = mem::transmute::<u32, [u8; 4]>(val);
        for i in idx..idx + size {
            self.value[i] = bits[i];
        }
    }
    pub unsafe fn u32_arr(&self) -> [u32; 16] {
        mem::transmute::<[u8; 64], [u32; 16]>(self.value)
    }
    pub unsafe fn set_u32_arr(&mut self, arr: [u32; 16]) {
        self.value = mem::transmute::<[u32; 16], [u8; 64]>(arr);
    }

    pub fn i64(&self, idx: usize) -> i64 {
        let size = mem::size_of::<i64>();
        let idx = idx * size;
        assert!(idx < self.value.len());

        let mut bits: [u8; 8] = [0; 8];
        bits.copy_from_slice(&self.value[idx..idx + size]);
        i64::from_le_bytes(bits)
    }
    pub unsafe fn set_i64(&mut self, idx: usize, val: i64) {
        let size = mem::size_of::<i64>();
        let idx = idx * size;
        assert!(idx < self.value.len());

        let bits = mem::transmute::<i64, [u8; 8]>(val);
        for i in idx..idx + size {
            self.value[i] = bits[i];
        }
    }
    pub unsafe fn i64_arr(&self) -> [i64; 8] {
        mem::transmute::<[u8; 64], [i64; 8]>(self.value)
    }
    pub unsafe fn set_i64_arr(&mut self, arr: [i64; 8]) {
        self.value = mem::transmute::<[i64; 8], [u8; 64]>(arr);
    }

    pub fn u64(&self, idx: usize) -> u64 {
        let size = mem::size_of::<u64>();
        let idx = idx * size;
        assert!(idx < self.value.len());

        let mut bits: [u8; 8] = [0; 8];
        bits.copy_from_slice(&self.value[idx..idx + size]);
        u64::from_le_bytes(bits)
    }
    pub unsafe fn set_u64(&mut self, idx: usize, val: u64) {
        let size = mem::size_of::<u64>();
        let idx = idx * size;
        assert!(idx < self.value.len());

        let bits = mem::transmute::<u64, [u8; 8]>(val);
        for i in idx..idx + size {
            self.value[i] = bits[i];
        }
    }
    pub unsafe fn u64_arr(&self) -> [u64; 8] {
        mem::transmute::<[u8; 64], [u64; 8]>(self.value)
    }
    pub unsafe fn set_u64_arr(&mut self, arr: [u64; 8]) {
        self.value = mem::transmute::<[u64; 8], [u8; 64]>(arr);
    }
}
