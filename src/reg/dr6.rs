/* ============================================================================
 * File:   dr6.rs
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
 * |  1   1   1   1   1   1   1   1   1   1   1   1   1   1   1 RTM|
 * -----------------------------------------------------------------
 * | 15| 14| 13| 12| 11| 10|  9|  8|  7|  6|  5|  4|  3|  2|  1|  0|
 * | BT  BS  BD   0   1   1   1   1   1   1   1   1  B3  B2  B1  B0|
 * -----------------------------------------------------------------
 */

const ALWAYS_SET_BITS: u32 = 0b1111_1111_1111_1110_0000_1111_1111_0000;
const ALWAYS_UNSET_BITS: u32 = 0b1_0000_0000_0000;
const EDITABLE_BITS: u32 = 0b0000_0000_0000_0001_1110_0000_0000_1111;

pub struct Dr6 {
    value: u32,
}

impl Dr6 {
    bitfield!(rtm, set_rtm, u32, 16);

    bitfield!(bt, set_bt, u32, 15);

    bitfield!(bs, set_bs, u32, 14);

    bitfield!(bd, set_bd, u32, 13);

    bitfield!(b3, set_b3, u32, 3);

    bitfield!(b2, set_b2, u32, 2);

    bitfield!(b1, set_b1, u32, 1);

    bitfield!(b0, set_b0, u32, 0);

    pub fn new() -> Dr6 {
        Dr6 {
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
