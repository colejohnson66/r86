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
// TODO: Implement f16, f32, and f64 accessors
// TODO: Implement saturation functions (should they be here?)
use std::mem;

macro_rules! avx_accessors {
    ($get:ident,$set:ident,$type:ty,$size:expr) => {
        pub fn $get(&self, idx: usize) -> $type {
            let size = mem::size_of::<$type>();
            let idx = idx * size;
            assert!(idx < self.value.len());

            let mut bits: [u8; $size] = [0; $size];
            bits.copy_from_slice(&self.value[idx..idx + size]);
            <$type>::from_le_bytes(bits)
        }
        pub unsafe fn $set(&mut self, idx: usize, val: $type) {
            let size = mem::size_of::<$type>();
            let idx = idx * size;
            assert!(idx < self.value.len());

            let bits = mem::transmute::<$type, [u8; $size]>(val);
            for i in idx..idx + size {
                self.value[i] = bits[i];
            }
        }
    };
}
macro_rules! avx_arr_accessor {
    ($get:ident,$set:ident,$type:ty,$size:expr) => {
        pub unsafe fn $get(&self) -> [$type; 64 / $size] {
            mem::transmute::<[u8; 64], [$type; 64 / $size]>(self.value)
        }
        pub unsafe fn $set(&mut self, arr: &[$type; 64 / $size]) {
            self.value = mem::transmute::<[$type; 64 / $size], [u8; 64]>(*arr);
        }
    };
}

/// An AVX register.
/// Internally represented by a 512-bit (64-byte) byte array.
pub struct Avx {
    value: [u8; 64],
}

impl Avx {
    avx_accessors!(i16, set_i16, i16, 2);

    avx_arr_accessor!(i16_arr, set_i16_arr, i16, 2);

    avx_accessors!(u16, set_u16, u16, 2);

    avx_arr_accessor!(u16_arr, set_u16_arr, u16, 2);

    avx_accessors!(i32, set_i32, i32, 4);

    avx_arr_accessor!(i32_arr, set_i32_arr, i32, 4);

    avx_accessors!(u32, set_u32, u32, 4);

    avx_arr_accessor!(u32_arr, set_u32_arr, u32, 4);

    avx_accessors!(i64, set_i64, i64, 8);

    avx_arr_accessor!(i64_arr, set_i64_arr, i64, 8);

    avx_accessors!(u64, set_u64, u64, 8);

    avx_arr_accessor!(u64_arr, set_u64_arr, u64, 8);

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
}
