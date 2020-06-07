/* ============================================================================
 * File:   swd.rs
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
use crate::multibit_bitfield;

/* -----------------------------------------------------------------
 * | 15| 14| 13| 12| 11| 10|  9|  8|  7|  6|  5|  4|  3|  2|  1|  0|
 * |  B  C3  TopOfStack  C2  C1  C0  ES  SF  PE  UE  OE  ZE  DE  IE|
 * -----------------------------------------------------------------
 */

const ALWAYS_SET_BITS: u16 = 0;
const ALWAYS_UNSET_BITS: u16 = 0;
const EDITABLE_BITS: u16 = 0xFFFF;

pub struct StatusWord {
    value: u16,
}

impl StatusWord {
    pub fn new() -> StatusWord {
        StatusWord {
            value: ALWAYS_SET_BITS,
        }
    }

    pub fn raw_value(&self) -> u16 {
        self.value
    }
    pub fn set_raw_value(&mut self, value: u16) {
        let temp = value & EDITABLE_BITS;
        self.value = temp | ALWAYS_SET_BITS;
    }
    pub fn set_raw_value_unchecked(&mut self, value: u16) {
        self.value = value;
    }

    bitfield!(b, set_b, u16, 15);
    // cc handled below
    multibit_bitfield!(tos, set_tos, u16, 11, 3);
    bitfield!(es, set_es, u16, 7);
    bitfield!(sf, set_sf, u16, 6);
    bitfield!(pe, set_pe, u16, 5);
    bitfield!(ue, set_ue, u16, 4);
    bitfield!(oe, set_oe, u16, 3);
    bitfield!(ze, set_ze, u16, 2);
    bitfield!(de, set_de, u16, 1);
    bitfield!(ie, set_ie, u16, 0);

    pub fn cc(&self) -> u16 {
        let c3 = (self.value & (1u16 << 14)) != 0;
        let c012 = (self.value >> 8) & 7;
        let mut cc = c012;
        if c3 {
            cc |= 1u16 << 3;
        }
        cc
    }
    pub fn set_cc(&mut self, val: u16) {
        let mask = !((1u16 << 8) | (1u16 << 9) | (1u16 << 10) | (1u16 << 14));
        let mut temp = self.value & mask;

        let c3 = (val & (1u16 << 3)) != 0;
        let c012 = val & 7;

        temp |= c012 << 8;
        if c3 {
            temp |= 1u16 << 14;
        }
        self.value = temp;
    }
}
