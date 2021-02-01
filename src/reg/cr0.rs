/* ============================================================================
 * File:   cr0.rs
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
use crate::bitfield;

/* -----------------------------------------------------------------
 * | 31| 30| 29| 28| 27| 26| 25| 24| 23| 22| 21| 20| 19| 18| 17| 16|
 * | PG  CD  NW   0   0   0   0   0   0   0   0   0   0  AM   0  WP|
 * -----------------------------------------------------------------
 * | 15| 14| 13| 12| 11| 10|  9|  8|  7|  6|  5|  4|  3|  2|  1|  0|
 * |  0   0   0   0   0   0   0   0   0   0  NE  ET  TS  EM  MP  PE|
 * -----------------------------------------------------------------
 */

const ALWAYS_SET_BITS: u32 = 0;
const ALWAYS_UNSET_BITS: u32 = 0b0001_1111_1111_1010_1111_1111_1100_0000;
const EDITABLE_BITS: u32 = 0b1110_0000_0000_0101_0000_0000_0011_1111;

pub struct Cr0 {
    value: u32,
}

impl Cr0 {
    bitfield!(pg, set_pg, u32, 31);

    bitfield!(cd, set_cd, u32, 30);

    bitfield!(nw, set_nw, u32, 29);

    bitfield!(am, set_am, u32, 18);

    bitfield!(wp, set_wp, u32, 18);

    bitfield!(ne, set_ne, u32, 5);

    bitfield!(et, set_et, u32, 4);

    bitfield!(ts, set_ts, u32, 3);

    bitfield!(em, set_em, u32, 2);

    bitfield!(mp, set_mp, u32, 1);

    bitfield!(pe, set_pe, u32, 0);

    pub fn new() -> Cr0 {
        Cr0 {
            value: ALWAYS_SET_BITS,
        }
    }

    pub fn raw_value(&self) -> u32 {
        self.value
    }

    pub fn set_raw_value(&mut self, value: u32) {
        let temp = value & EDITABLE_BITS;
        self.value = temp | ALWAYS_SET_BITS;
    }

    pub fn set_raw_value_unchecked(&mut self, value: u32) {
        self.value = value;
    }
}
