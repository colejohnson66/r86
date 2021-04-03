/* ============================================================================
 * File:   rcl_rcr_rol_ror.rs
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

pub struct Rcl;
pub struct Rcr;
pub struct Rol;
pub struct Ror;

impl Rcl {
    stub_handler!(eb_1);

    stub_handler!(eb_cl);

    stub_handler!(eb_ib);

    stub_handler!(ew_1);

    stub_handler!(ew_cl);

    stub_handler!(ew_ib);

    stub_handler!(ed_1);

    stub_handler!(ed_cl);

    stub_handler!(ed_ib);

    stub_handler!(eq_1);

    stub_handler!(eq_cl);

    stub_handler!(eq_ib);
}

impl Rcr {
    stub_handler!(eb_1);

    stub_handler!(eb_cl);

    stub_handler!(eb_ib);

    stub_handler!(ew_1);

    stub_handler!(ew_cl);

    stub_handler!(ew_ib);

    stub_handler!(ed_1);

    stub_handler!(ed_cl);

    stub_handler!(ed_ib);

    stub_handler!(eq_1);

    stub_handler!(eq_cl);

    stub_handler!(eq_ib);
}

impl Rol {
    stub_handler!(eb_1);

    stub_handler!(eb_cl);

    stub_handler!(eb_ib);

    stub_handler!(ew_1);

    stub_handler!(ew_cl);

    stub_handler!(ew_ib);

    stub_handler!(ed_1);

    stub_handler!(ed_cl);

    stub_handler!(ed_ib);

    stub_handler!(eq_1);

    stub_handler!(eq_cl);

    stub_handler!(eq_ib);
}

impl Ror {
    stub_handler!(eb_1);

    stub_handler!(eb_cl);

    stub_handler!(eb_ib);

    stub_handler!(ew_1);

    stub_handler!(ew_cl);

    stub_handler!(ew_ib);

    stub_handler!(ed_1);

    stub_handler!(ed_cl);

    stub_handler!(ed_ib);

    stub_handler!(eq_1);

    stub_handler!(eq_cl);

    stub_handler!(eq_ib);
}
