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
pub mod decoder;

use crate::reg::avx::Avx;
use crate::reg::cr0::Cr0;
use crate::reg::gpr::Gpr;

type Address = u64;

const AVX_REGISTER_COUNT: usize = 32;

pub struct Cpu {
    /* In order:
     *  0 - rax: accumulator
     *  1 - rcx: counter
     *  2 - rdx: data
     *  3 - rbx: base
     *  4 - rbp: base pointer
     *  5 - rsi: source index
     *  6 - rdi: desination index
     *  7 - rsp: stack pointer
     *  8 - r8-r15: x86-64 extended registers
     * 16 - rip: instruction pointer
     * 17 - tmp: temp register
     * 18 - nil: null register
     */
    gpr: [Gpr; 19],

    flags: (),

    // saved for backing up during a fault
    prev_rip: Address,
    prev_rsp: Address,

    instr_count: u64,

    segment: [(); 6],

    gdtr: (),
    idtr: (),
    ldtr: (),
    tr: (),

    dr: [Address; 4], // DR0-DR3
    dr6: (),
    dr7: u32,

    cr0: Cr0,
    cr2: Address,
    cr3: (),
    cr4: (),
    cr8: (),

    i387: (),

    // vmm0-vmm31: vector registers
    // vtmp: temp register
    vmm: [Avx; 33],
    opmask: [Gpr; 8],

    mxcsr: (),
    mxcsr_mask: u32,

    pkru: (),

    activity_state: (),
}

macro_rules! gpr {
    ($name:ident,$idx:expr) => {
        pub fn $name(&mut self) -> &Gpr {
            &self.gpr[$idx]
        }
    };
}

impl Cpu {
    gpr!(rax, 0);
    gpr!(rcx, 1);
    gpr!(rdx, 2);
    gpr!(rbx, 3);
    gpr!(rbp, 4);
    gpr!(rsi, 5);
    gpr!(rdi, 6);
    gpr!(rsp, 7);
    gpr!(r8, 8);
    gpr!(r9, 9);
    gpr!(r10, 10);
    gpr!(r11, 11);
    gpr!(r12, 12);
    gpr!(r13, 13);
    gpr!(r14, 14);
    gpr!(r15, 15);
    gpr!(rip, 16);
    gpr!(tmp, 17);
    gpr!(nil, 18);

    pub fn gpr(&mut self, reg: usize) -> &Gpr {
        assert!(reg < self.gpr.len());
        &self.gpr[reg]
    }

    pub fn avx(&mut self, reg: usize) -> &Avx {
        assert!(reg < self.vmm.len());
        &self.vmm[reg]
    }

    pub fn avx_opmask(&mut self, idx: usize) -> &Gpr {
        assert!(idx < self.opmask.len());
        &self.opmask[idx]
    }
}
