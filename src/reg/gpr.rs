/* ============================================================================
 * File:   mod.rs
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
// endian independent
pub struct Gpr {
    value: u64,
}

impl Gpr {
    pub fn new() -> Gpr {
        Gpr { value: 0 }
    }

    pub fn byte_low(&self) -> u8 {
        (self.value & 0xFF) as u8
    }

    pub fn set_byte_low(&mut self, val: u8) {
        let temp = self.value & 0xFFFF_FFFF_FFFF_FF00;
        self.value = temp | (val as u64);
    }

    pub fn byte_high(&self) -> u8 {
        ((self.value >> 8) & 0xFF) as u8
    }

    pub fn set_byte_high(&mut self, val: u8) {
        let temp = self.value & 0xFFFF_FFFF_FFFF_00FF;
        self.value = temp | ((val as u64) << 8);
    }

    pub fn word(&self) -> u16 {
        (self.value & 0xFFFF) as u16
    }

    pub fn set_word(&mut self, val: u16) {
        let temp = self.value & 0xFFFF_FFFF_FFFF_0000;
        self.value = temp | (val as u64);
    }

    pub fn dword(&self) -> u32 {
        (self.value & 0xFFFF_FFFF) as u32
    }

    pub fn set_dword(&mut self, val: u32) {
        // zero extend when setting ErX
        self.value = val.into();
    }

    pub fn qword(&self) -> u64 {
        self.value
    }

    pub fn set_qword(&mut self, val: u64) {
        self.value = val;
    }
}
