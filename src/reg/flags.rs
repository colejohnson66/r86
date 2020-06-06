/* ============================================================================
 * File:   flags.rs
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
use crate::{bitfield, multibit_bitfield};

/* -----------------------------------------------------------------
 * | 31| 30| 29| 28| 27| 26| 25| 24| 23| 22| 21| 20| 19| 18| 17| 16|
 * |  0   0   0   0   0   0   0   0   0   0  ID VIP VIF  AC  VM  RF|
 * -----------------------------------------------------------------
 * | 15| 14| 13| 12| 11| 10|  9|  8|  7|  6|  5|  4|  3|  2|  1|  0|
 * |  0  NT   IOPL   OF  DF  IF  TF  SF  ZF   0  AF   0  PF   1  CF|
 * -----------------------------------------------------------------
 *
 * bit[1] is VF^[0] on 8085; always 1 on 8086+
 * bit[3] is always 0 on 8085+
 * bit[5] is KF^[0] on 8085; always 0 on 8086+
 * bit[13:12] is always 11b on 8086 and 80186
 * bit[15] is 1 on 8086 and 80186; always 0 on 80286+
 *
 *   [0]: http://www.righto.com/2013/02/looking-at-silicon-to-understanding.html
 */

const ALWAYS_SET_BITS: u32 = 0b10;
const ALWAYS_UNSET_BITS: u32 = 0b1111_1111_1100_0000_1000_0000_0010_1000;
const EDITABLE_BITS: u32 = 0b0000_0000_0011_1111_0111_1111_1101_0111;

pub struct Flags {
    value: u32,
}

impl Flags {
    pub fn new() -> Flags {
        Flags {
            value: ALWAYS_SET_BITS,
        }
    }

    pub fn raw_value(&self) -> u64 {
        self.value as u64
    }
    pub fn set_raw_value(&mut self, value: u64) {
        // bit[63:22] are reserved, so ensure the upper 32-bits aren't set
        // TODO: should we just ignore them instead of panic!()?
        assert!(((value as u64) & 0xFFFF_FFFF_0000_0000) == 0);
        let temp = value as u32;
        let temp = temp & EDITABLE_BITS;
        self.value = temp | ALWAYS_SET_BITS;
    }

    bitfield!(id, set_id, u32, 21);
    bitfield!(vip, set_vip, u32, 20);
    bitfield!(vif, set_vif, u32, 19);
    bitfield!(ac, set_ac, u32, 18);
    bitfield!(vm, set_vm, u32, 17);
    bitfield!(rf, set_rf, u32, 16);
    bitfield!(nt, set_nt, u32, 14);
    multibit_bitfield!(iopl, set_iopl, u32, 12, 2);
    bitfield!(of, set_of, u32, 11);
    bitfield!(df, set_df, u32, 10);
    bitfield!(if_, set_if, u32, 9);
    bitfield!(tf, set_tf, u32, 8);
    bitfield!(sf, set_sf, u32, 7);
    bitfield!(zf, set_zf, u32, 6);
    bitfield!(af, set_af, u32, 4);
    bitfield!(pf, set_pf, u32, 2);
    bitfield!(cf, set_cf, u32, 0);
}
