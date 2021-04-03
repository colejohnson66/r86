/* ============================================================================
 * File:   jcc.rs
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
use crate::cpu::decoder::Instr;
use crate::cpu::Cpu;
use crate::stub_handler;

pub struct Jo;
pub struct Jno;
pub struct Jb;
pub struct Jae;
pub struct Je;
pub struct Jne;
pub struct Jbe;
pub struct Ja;
pub struct Js;
pub struct Jns;
pub struct Jp;
pub struct Jnp;
pub struct Jl;
pub struct Jge;
pub struct Jle;
pub struct Jg;
pub struct Jcxz;

impl Jo {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Jno {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Jb {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Jae {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Je {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Jne {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Jbe {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Ja {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Js {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Jns {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Jp {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Jnp {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Jl {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Jge {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Jle {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Jg {
    stub_handler!(jb);

    stub_handler!(jw);

    stub_handler!(jd);
}

impl Jcxz {
    stub_handler!(jb);
}
