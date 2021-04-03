/* ============================================================================
 * File:   pcmp.rs
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

pub struct Pcmpb;
pub struct Pcmpub;
pub struct Pcmpw;
pub struct Pcmpuw;
pub struct Pcmpd;
pub struct Pcmpud;
pub struct Pcmpq;
pub struct Pcmpuq;

impl Pcmpb {
    stub_handler!(v_kgq_hdq_wdq_ib_e128);

    stub_handler!(v_kgq_hqq_wqq_ib_e256);

    stub_handler!(v_kgq_hdqq_wdqq_ib_e512);
}

impl Pcmpub {
    stub_handler!(v_kgq_hdq_wdq_ib_e128);

    stub_handler!(v_kgq_hqq_wqq_ib_e256);

    stub_handler!(v_kgq_hdqq_wdqq_ib_e512);
}

impl Pcmpw {
    stub_handler!(v_kgq_hdq_wdq_ib_e128);

    stub_handler!(v_kgq_hqq_wqq_ib_e256);

    stub_handler!(v_kgq_hdqq_wdqq_ib_e512);
}

impl Pcmpuw {
    stub_handler!(v_kgq_hdq_wdq_ib_e128);

    stub_handler!(v_kgq_hqq_wqq_ib_e256);

    stub_handler!(v_kgq_hdqq_wdqq_ib_e512);
}

impl Pcmpd {
    stub_handler!(v_kgq_hdq_wdq_ib_e128);

    stub_handler!(v_kgq_hqq_wqq_ib_e256);

    stub_handler!(v_kgq_hdqq_wdqq_ib_e512);
}

impl Pcmpud {
    stub_handler!(v_kgq_hdq_wdq_ib_e128);

    stub_handler!(v_kgq_hqq_wqq_ib_e256);

    stub_handler!(v_kgq_hdqq_wdqq_ib_e512);
}

impl Pcmpq {
    stub_handler!(v_kgq_hdq_wdq_ib_e128);

    stub_handler!(v_kgq_hqq_wqq_ib_e256);

    stub_handler!(v_kgq_hdqq_wdqq_ib_e512);
}

impl Pcmpuq {
    stub_handler!(v_kgq_hdq_wdq_ib_e128);

    stub_handler!(v_kgq_hqq_wqq_ib_e256);

    stub_handler!(v_kgq_hdqq_wdqq_ib_e512);
}
