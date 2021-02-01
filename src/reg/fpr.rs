/* ============================================================================
 * File:   fpr.rs
 * Author: Cole Johnson
 * ============================================================================
 * Copyright (c) 2020-2021 Cole Johnson
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
use std::mem;

macro_rules! fpr_accessors {
    ($get:ident,$set:ident,$type:ty,$size:expr) => {
        pub fn $get(&self, idx: usize) -> $type {
            let size = mem::size_of::<$type>();
            let idx = idx * size;
            assert!(idx < 8);

            let significand_bytes = self.significand.to_le_bytes();

            let mut bits: [u8; $size] = [0; $size];
            bits.copy_from_slice(&significand_bytes[idx..idx + $size]);
            <$type>::from_le_bytes(bits)
        }
        pub unsafe fn $set(&mut self, idx: usize, val: $type) {
            let size = mem::size_of::<$type>();
            let idx = idx * size;
            assert!(idx < 8);

            let mut significand_bytes = self.significand.to_le_bytes();

            let bits = mem::transmute::<$type, [u8; $size]>(val);
            for i in idx..idx + size {
                significand_bytes[i] = bits[i];
            }

            self.significand = mem::transmute::<[u8; 8], u64>(significand_bytes);
            self.after_mmx_reg_set();
        }
    };
}

// Little endian
// TODO: make endian independent
#[derive(Clone, Copy)]
pub struct Fpr {
    sign_exponent: u16,
    significand: u64,
}

impl Fpr {
    fpr_accessors!(mmx_i8, set_mmx_i8, i8, 1);

    fpr_accessors!(mmx_u8, set_mmx_u8, u8, 1);

    fpr_accessors!(mmx_i16, set_mmx_i16, i16, 2);

    fpr_accessors!(mmx_u16, set_mmx_u16, u16, 2);

    fpr_accessors!(mmx_i32, set_mmx_i32, i32, 4);

    fpr_accessors!(mmx_u32, set_mmx_u32, u32, 4);

    pub fn new() -> Fpr {
        Fpr {
            sign_exponent: 0,
            significand: 0,
        }
    }

    pub fn sign(&self) -> bool {
        (self.sign_exponent & 0x80) == 0x80
    }

    pub fn sign_exponent(&self) -> u16 {
        self.sign_exponent
    }

    pub fn significand(&self) -> u64 {
        self.significand
    }

    pub unsafe fn mmx_i64(&self) -> i64 {
        mem::transmute::<u64, i64>(self.significand)
    }

    pub unsafe fn set_mmx_i64(&mut self, val: i64) {
        self.significand = mem::transmute::<i64, u64>(val);
        self.after_mmx_reg_set();
    }

    pub fn mmx_u64(&self) -> u64 {
        self.significand
    }

    pub fn set_mmx_u64(&mut self, val: u64) {
        self.significand = val;
        self.after_mmx_reg_set();
    }

    fn after_mmx_reg_set(&mut self) {
        self.sign_exponent = 0xFFFF;
    }
}
