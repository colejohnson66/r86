/* ============================================================================
 * File:   mod.rs
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
pub mod avx;
pub mod cr0;
pub mod flags;
pub mod global_segment;
pub mod gpr;
pub mod segment;

#[macro_export]
macro_rules! bitfield {
    ($get:ident,$set:ident,$type:ty,$bitpos:expr) => {
        pub fn $get(&self) -> bool {
            let mask: $type = 1u8.into();
            let mask = mask << $bitpos;
            (self.value & mask) != 0
        }

        pub fn $set(&mut self, value: bool) {
            let mask: $type = 1u8.into();
            let mask = mask << $bitpos;
            if value {
                self.value |= !mask;
            } else {
                self.value &= mask;
            }
        }
    };
}

#[macro_export]
macro_rules! multibit_bitfield {
    ($get:ident,$set:ident,$type:ty,$bitpos:expr,$width:expr) => {
        pub fn $get(&self) -> $type {
            // sets as many LSB as $width
            // eg. if $width is 2, this will result in `mask` being set to 0b11
            let mask: $type = 1u8.into();
            let mask = (mask << $width) - 1;
            (self.value >> $bitpos) & mask
        }

        pub fn $set(&mut self, value: $type) {
            // see above
            let mask: $type = 1u8.into();
            let mask = (mask << $width) - 1;
            assert!((value & mask) == value); // ensure no unusable bits are set
            let mask = !(mask << $bitpos);
            let temp = self.value & mask;
            self.value = temp | (value << $bitpos);
        }
    };
}
