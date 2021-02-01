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
mod maskmovdqu;
mod maskmovp;
mod maskmovq;
mod maxpd;
mod maxps;
mod maxsd;
mod maxss;
mod mfence;
mod minpd;
mod minps;
mod minsd;
mod minss;
mod monitor;
mod mov;
mod mov_ctrl;
mod mov_dbg;
mod movapd;
mod movaps;
mod movbe;
mod movd_movq;
mod movddup;
mod movdir64b;
mod movdiri;
mod movdq2q;
mod movdqa;
mod movdqu;
mod movhlps;
mod movhpd;
mod movhps;
mod movlhps;
mod movlpd;
mod movlps;
mod movmskpd;
mod movmskps;
mod movntdq;
mod movntdqa;
mod movnti;
mod movntpd;
mod movntps;
mod movntq;
mod movq;
mod movq2dq;
mod movs;
mod movsd;
mod movshdup;
mod movsldup;
mod movss;
mod movsx;
mod movupd;
mod movups;
mod movzx;
mod mpsadbw;
mod mul;
mod mulpd;
mod mulps;
mod mulsd;
mod mulss;
mod mulx;
mod mwait;

pub use maskmovdqu::*;
pub use maskmovp::*;
pub use maskmovq::*;
pub use maxpd::*;
pub use maxps::*;
pub use maxsd::*;
pub use maxss::*;
pub use mfence::*;
pub use minpd::*;
pub use minps::*;
pub use minsd::*;
pub use minss::*;
pub use monitor::*;
pub use mov::*;
pub use mov_ctrl::*;
pub use mov_dbg::*;
pub use movapd::*;
pub use movaps::*;
pub use movbe::*;
pub use movd_movq::*;
pub use movddup::*;
pub use movdir64b::*;
pub use movdiri::*;
pub use movdq2q::*;
pub use movdqa::*;
pub use movdqu::*;
pub use movhlps::*;
pub use movhpd::*;
pub use movhps::*;
pub use movlhps::*;
pub use movlpd::*;
pub use movlps::*;
pub use movmskpd::*;
pub use movmskps::*;
pub use movntdq::*;
pub use movntdqa::*;
pub use movnti::*;
pub use movntpd::*;
pub use movntps::*;
pub use movntq::*;
pub use movq::*;
pub use movq2dq::*;
pub use movs::*;
pub use movsd::*;
pub use movshdup::*;
pub use movsldup::*;
pub use movss::*;
pub use movsx::*;
pub use movupd::*;
pub use movups::*;
pub use movzx::*;
pub use mpsadbw::*;
pub use mul::*;
pub use mulpd::*;
pub use mulps::*;
pub use mulsd::*;
pub use mulss::*;
pub use mulx::*;
pub use mwait::*;
