/* ============================================================================
 * File:   pmovzx.rs
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

pub struct Pmovzxbw;
pub struct Pmovzxbd;
pub struct Pmovzxbq;
pub struct Pmovzxwd;
pub struct Pmovzxwq;
pub struct Pmovzxdq;

impl Pmovzxbw {
    stub_handler!(vdq_wq);

    stub_handler!(v_vdq_wq_v128);

    stub_handler!(v_vqq_wdq_v256);

    stub_handler!(v_vdq_wq_e128);

    stub_handler!(v_vqq_wdq_e256);

    stub_handler!(v_vdqq_wqq_e512);
}

impl Pmovzxbd {
    stub_handler!(vdq_wd);

    stub_handler!(v_vdq_wd_v128);

    stub_handler!(v_vqq_wq_v256);

    stub_handler!(v_vdq_wd_e128);

    stub_handler!(v_vqq_wq_e256);

    stub_handler!(v_vdqq_wdq_e512);
}

impl Pmovzxbq {
    stub_handler!(vdq_ww);

    stub_handler!(v_vdq_ww_v128);

    stub_handler!(v_vqq_wd_v256);

    stub_handler!(v_vdq_ww_e128);

    stub_handler!(v_vqq_wd_e256);

    stub_handler!(v_vdqq_wdq_e512);
}

impl Pmovzxwd {
    stub_handler!(vdq_wq);

    stub_handler!(v_vdq_wq_v128);

    stub_handler!(v_vqq_wdq_v256);

    stub_handler!(v_vdq_wq_e128);

    stub_handler!(v_vqq_wdq_e256);

    stub_handler!(v_vdqq_wqq_e512);
}

impl Pmovzxwq {
    stub_handler!(vdq_wd);

    stub_handler!(v_vdq_wd_v128);

    stub_handler!(v_vqq_wq_v256);

    stub_handler!(v_vdq_wd_e128);

    stub_handler!(v_vqq_wq_e256);

    stub_handler!(v_vdqq_wdq_e512);
}

impl Pmovzxdq {
    stub_handler!(vdq_wq);

    stub_handler!(v_vdq_wq_v128);

    stub_handler!(v_vqq_wqq_v256);

    stub_handler!(v_vdq_wq_e128);

    stub_handler!(v_vqq_wdq_e256);

    stub_handler!(v_vdqq_wqq_e512);
}
