/* ============================================================================
 * File:   cwd.rs
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
use crate::multibit_bitfield;

/* -----------------------------------------------------------------
 * | 15| 14| 13| 12| 11| 10|  9|  8|  7|  6|  5|  4|  3|  2|  1|  0|
 * |  0   0   0   X    RC      PC     0   0  PM  UM  OM  ZM  DM  IM|
 * -----------------------------------------------------------------
 */

const ALWAYS_SET_BITS: u16 = 0;
const ALWAYS_UNSET_BITS: u16 = 0b1110_0000_1100_0000;
const EDITABLE_BITS: u16 = 0b0001_1111_0011_1111;

pub struct ControlWord {
    value: u16,
}

impl ControlWord {
    bitfield!(x, set_x, u16, 12);

    multibit_bitfield!(rc, set_rc, u16, 10, 2);

    multibit_bitfield!(pc, set_pc, u16, 8, 2);

    bitfield!(pm, set_pm, u16, 5);

    bitfield!(um, set_um, u16, 4);

    bitfield!(om, set_om, u16, 3);

    bitfield!(zm, set_zm, u16, 2);

    bitfield!(dm, set_dm, u16, 1);

    bitfield!(im, set_im, u16, 0);

    pub fn new() -> ControlWord {
        ControlWord {
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
}
