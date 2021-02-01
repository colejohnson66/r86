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
pub mod cwd;
pub mod swd;
pub mod twd;

use crate::reg::fpr::Fpr;
use crate::Address;

pub struct I387 {
    cwd: cwd::ControlWord,
    swd: swd::StatusWord,
    twd: twd::TagWord,
    fop: u16, // last opcode; [15:11] unused (must be 0)

    fcs: u16,
    fip: Address,
    fds: u16,
    fdp: Address,

    st: [Fpr; 8],
}

impl I387 {
    pub fn new() -> I387 {
        // technically, these values are undefined
        I387 {
            cwd: cwd::ControlWord::new(),
            swd: swd::StatusWord::new(),
            twd: twd::TagWord::new(),
            fop: 0,
            fcs: 0,
            fip: 0,
            fds: 0,
            fdp: 0,
            st: [Fpr::new(); 8],
        }
    }

    pub fn init(&mut self) {
        self.cwd.set_raw_value_unchecked(0x037F); // all exception masks set to on, set rounding to nearest, and precision to 64-bits
        self.swd.set_raw_value_unchecked(0);
        self.twd.set_raw_value_unchecked(0xFFFF); // set all to 0b11 for empty
        self.fop = 0;

        self.fcs = 0;
        self.fip = 0;
        self.fds = 0;
        self.fdp = 0;

        // upon init, `ST(i)` is undefined
    }

    pub fn reset(&mut self) {
        self.cwd.set_raw_value_unchecked(0x0040);
        self.swd.set_raw_value_unchecked(0);
        self.twd.set_raw_value_unchecked(0xFFFF); // set all to 0b01 for zero
        self.fop = 0;

        self.fcs = 0;
        self.fip = 0;
        self.fds = 0;
        self.fdp = 0;

        self.st = [Fpr::new(); 8];
    }

    pub fn tag(&self, reg: u16) -> u16 {
        let reg = (reg + self.swd.tos()) & 7;
        self.twd.get(reg as u32)
    }

    pub fn set_tag_valid(&mut self, reg: u16) {
        let reg = (reg + self.swd.tos()) & 7;
        self.twd.set(reg as u32, 0);
    }

    pub fn set_tag(&mut self, reg: u16, val: u16) {
        let reg = (reg + self.swd.tos()) & 7;
        self.twd.set(reg as u32, val);
    }

    // TODO: push, pop, get_ST(i), set_ST(i)
}
