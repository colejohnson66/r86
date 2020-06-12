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
mod f2xm1;
mod fabs;
mod fadd;
mod fbld;
mod fbstp;
mod fchs;
mod fclex;
mod fcmovcc;
mod fcom;
mod fcomi;
mod fcos;
mod fdecstp;
mod fdiv;
mod fdivr;
mod ffree;
mod ficom;
mod fild;
mod fincstp;
mod finit;
mod fist;
mod fisttp;
mod fld;
mod fldc;
mod fldcw;
mod fldenv;
mod fmul;
mod fnop;
mod fpatan;
mod fprem;
mod fprem1;
mod fptan;
mod frndint;
mod frstor;
mod fsave;
mod fscale;
mod fsin;
mod fsincos;
mod fsqrt;
mod fst;
mod fstcw;
mod fstenv;
mod fstsw;
mod fsub;
mod fsubr;
mod ftst;
mod fucom;
mod fxam;
mod fxch;
mod fxrstor;
mod fxsave;
mod fxtract;
mod fyl2x;
mod fyl2xp1;

pub use f2xm1::*;
pub use fabs::*;
pub use fadd::*;
pub use fbld::*;
pub use fbstp::*;
pub use fchs::*;
pub use fclex::*;
pub use fcmovcc::*;
pub use fcom::*;
pub use fcomi::*;
pub use fcos::*;
pub use fdecstp::*;
pub use fdiv::*;
pub use fdivr::*;
pub use ffree::*;
pub use ficom::*;
pub use fild::*;
pub use fincstp::*;
pub use finit::*;
pub use fist::*;
pub use fisttp::*;
pub use fld::*;
pub use fldc::*;
pub use fldcw::*;
pub use fldenv::*;
pub use fmul::*;
pub use fnop::*;
pub use fpatan::*;
pub use fprem::*;
pub use fprem1::*;
pub use fptan::*;
pub use frndint::*;
pub use frstor::*;
pub use fsave::*;
pub use fscale::*;
pub use fsin::*;
pub use fsincos::*;
pub use fsqrt::*;
pub use fst::*;
pub use fstcw::*;
pub use fstenv::*;
pub use fstsw::*;
pub use fsub::*;
pub use fsubr::*;
pub use ftst::*;
pub use fucom::*;
pub use fxam::*;
pub use fxch::*;
pub use fxrstor::*;
pub use fxsave::*;
pub use fxtract::*;
pub use fyl2x::*;
pub use fyl2xp1::*;
