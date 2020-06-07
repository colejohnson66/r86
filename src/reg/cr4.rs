/* ============================================================================
 * File:   cr4rs
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

/* -----------------------------------------------------------------
 * | 31| 30| 29| 28| 27| 26| 25| 24| 23| 22| 21| 20| 19| 18| 17| 16|
 * |  0   0   0   0   0   0   0   0   0 PKE  SM  SM   0 OSX PCI  FS|
 * |                                         AP  EP    SAVE  DE  GS|
 * |                                                           BASE|
 * -----------------------------------------------------------------
 * | 15| 14| 13| 12| 11| 10|  9|  8|  7|  6|  5|  4|  3|  2|  1|  0|
 * |  0  SM  VM   0  UM OSX  OS PCE PGE MCE PAE PSE  DE TSD PVI VME|
 * |     XE  XE      IP MME  FX                                    |
 * |                   XCPT  SR                                    |
 * -----------------------------------------------------------------
 */

const ALWAYS_SET_BITS: u32 = 0;
const ALWAYS_UNSET_BITS: u32 = 0b1111_1111_1000_1000_1001_0000_0000_0000;
const EDITABLE_BITS: u32 = 0b0000_0000_0111_0111_0110_1111_1111_1111;

pub struct Cr4 {
    value: u32,
}

impl Cr4 {
    pub fn new() -> Cr4 {
        Cr4 {
            value: ALWAYS_SET_BITS,
        }
    }

    pub fn raw_value(&self) -> u64 {
        self.value as u64
    }
    pub fn set_raw_value(&mut self, value: u64) {
        // bit[63:23] are reserved, so ensure the upper 32-bits aren't set
        // TODO: should we just ignore them instead of panic!()?
        assert!((value & 0xFFFF_FFFF_0000_0000) == 0);
        let temp = value as u32;
        let temp = temp & EDITABLE_BITS;
        self.value = temp | ALWAYS_SET_BITS;
    }
    pub fn set_raw_value_unchecked(&mut self, value: u64) {
        // just lob off the upper bits if they're set
        self.value = (value & 0xFFFF_FFFF) as u32;
    }

    bitfield!(pke, set_pke, u32, 22);
    bitfield!(smap, set_smap, u32, 21);
    bitfield!(smep, set_smep, u32, 20);
    bitfield!(osxsave, set_osxsave, u32, 18);
    bitfield!(pcide, set_pcide, u32, 17);
    bitfield!(fsgsbase, set_fsgsbase, u32, 16);
    bitfield!(smxe, set_smxe, u32, 14);
    bitfield!(vmxe, set_vmxe, u32, 13);
    bitfield!(umip, set_umip, u32, 11);
    bitfield!(osxmmexcpt, set_osxmmexcpt, u32, 10);
    bitfield!(osfxsr, set_osfxsr, u32, 9);
    bitfield!(pce, set_pce, u32, 8);
    bitfield!(pge, set_pge, u32, 7);
    bitfield!(mce, set_mce, u32, 6);
    bitfield!(pae, set_pae, u32, 5);
    bitfield!(pse, set_pse, u32, 4);
    bitfield!(de, set_de, u32, 3);
    bitfield!(tsd, set_tsd, u32, 2);
    bitfield!(pvi, set_pvi, u32, 1);
    bitfield!(vme, set_vme, u32, 0);
}
