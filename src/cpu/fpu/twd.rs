/* ============================================================================
 * File:   twd.rs
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
/* -----------------------------------------------------------------
 * | 15| 14| 13| 12| 11| 10|  9|  8|  7|  6|  5|  4|  3|  2|  1|  0|
 * | TAG(7)  TAG(6)  TAG(5)  TAG(4)  TAG(3)  TAG(2)  TAG(1)  TAG(0)|
 * -----------------------------------------------------------------
 */

const ALWAYS_SET_BITS: u16 = 0;
const ALWAYS_UNSET_BITS: u16 = 0;
const EDITABLE_BITS: u16 = 0xFFFF;

pub struct TagWord {
    value: u16,
}

impl TagWord {
    pub fn new() -> TagWord {
        TagWord {
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

    pub fn get(&self, reg: u32) -> u16 {
        assert!(reg < 8);
        (self.value >> (reg * 2)) & 3
    }
    pub fn set(&mut self, reg: u32, val: u16) {
        assert!(reg < 8);
        assert!(val <= 0b11);

        // clear old bits
        let temp = self.value & !(0b11 << (reg * 2));
        self.value = temp | (val << (reg * 2));
    }
}
