/* ============================================================================
 * File:   cr3.rs
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
use crate::bitfield;
use crate::Address;

/* -----------------------------------------------------------------
 * | 63-14 | 13| 12| 11| 10|  9|  8|  7|  6|  5|  4|  3|  2|  1|  0|
 * |   PageDirBase    0   0   0   0   0   0   0 PCD PWT   0   0   0|
 * -----------------------------------------------------------------
 */

const ALWAYS_SET_BITS: u64 = 0;
const ALWAYS_UNSET_BITS: u64 = 0x0000_0000_0000_0FE7;
const EDITABLE_BITS: u64 = 0xFFFF_FFFF_FFFF_F018;

pub struct Cr3 {
    value: u64,
}

impl Cr3 {
    bitfield!(pcd, set_pcd, u64, 4);

    bitfield!(pwt, set_pwt, u64, 3);

    pub fn new() -> Cr3 {
        Cr3 {
            value: ALWAYS_SET_BITS,
        }
    }

    pub fn raw_value(&self) -> u64 {
        self.value
    }

    pub fn set_raw_value(&mut self, value: u64) {
        let temp = value & EDITABLE_BITS;
        self.value = temp | ALWAYS_SET_BITS;
    }

    pub fn set_raw_value_unchecked(&mut self, value: u64) {
        self.value = value;
    }

    pub fn page_directory_base(&self) -> Address {
        self.value >> 11
    }

    pub fn set_page_directory_base(&mut self, value: Address) {
        // TODO: ensure upper 11 bits are unset
        let temp = self.value & 0x18;
        self.value = (value << 11) | temp;
    }
    // TODO
}
