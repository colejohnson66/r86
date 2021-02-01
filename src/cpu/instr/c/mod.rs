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
mod call;
mod cbw_cwde_cdqe;
mod clac;
mod clc;
mod cld;
mod cldemote;
mod clflush;
mod clflushopt;
mod cli;
mod clrssbsy;
mod clts;
mod clwb;
mod cmc;
mod cmovcc;
mod cmp;
mod cmppd;
mod cmpps;
mod cmps;
mod cmpsd;
mod cmpss;
mod cmpxchg;
mod cmpxchgxb;
mod comisd;
mod comiss;
mod compresspd;
mod compressps;
mod cpuid;
mod crc32;
mod cvtdq2pd;
mod cvtdq2ps;
mod cvtpd2dq;
mod cvtpd2pi;
mod cvtpd2ps;
mod cvtpd2qq;
mod cvtpd2udq;
mod cvtpd2uqq;
mod cvtph2ps;
mod cvtpi2pd;
mod cvtpi2ps;
mod cvtps2dq;
mod cvtps2pd;
mod cvtps2ph;
mod cvtps2pi;
mod cvtps2qq;
mod cvtps2udq;
mod cvtps2uqq;
mod cvtqq2pd;
mod cvtqq2ps;
mod cvtsd2si;
mod cvtsd2ss;
mod cvtsd2usi;
mod cvtsi2sd;
mod cvtsi2ss;
mod cvtss2sd;
mod cvtss2si;
mod cvtss2usi;
mod cvttpd2dq;
mod cvttpd2pi;
mod cvttpd2qq;
mod cvttpd2udq;
mod cvttpd2uqq;
mod cvttps2dq;
mod cvttps2pi;
mod cvttps2qq;
mod cvttps2uqq;
mod cvttsd2si;
mod cvttsd2usi;
mod cvttss2si;
mod cvttss2usi;
mod cvtudq2pd;
mod cvtudq2ps;
mod cvtuqq2pd;
mod cvtuqq2ps;
mod cvtusi2sd;
mod cvtusi2ss;
mod cwd_cdq_cqo;

pub use call::*;
pub use cbw_cwde_cdqe::*;
pub use clac::*;
pub use clc::*;
pub use cld::*;
pub use cldemote::*;
pub use clflush::*;
pub use clflushopt::*;
pub use cli::*;
pub use clrssbsy::*;
pub use clts::*;
pub use clwb::*;
pub use cmc::*;
pub use cmovcc::*;
pub use cmp::*;
pub use cmppd::*;
pub use cmpps::*;
pub use cmps::*;
pub use cmpsd::*;
pub use cmpss::*;
pub use cmpxchg::*;
pub use cmpxchgxb::*;
pub use comisd::*;
pub use comiss::*;
pub use compresspd::*;
pub use compressps::*;
pub use cpuid::*;
pub use crc32::*;
pub use cvtdq2pd::*;
pub use cvtdq2ps::*;
pub use cvtpd2dq::*;
pub use cvtpd2pi::*;
pub use cvtpd2ps::*;
pub use cvtpd2qq::*;
pub use cvtpd2udq::*;
pub use cvtpd2uqq::*;
pub use cvtph2ps::*;
pub use cvtpi2pd::*;
pub use cvtpi2ps::*;
pub use cvtps2dq::*;
pub use cvtps2pd::*;
pub use cvtps2ph::*;
pub use cvtps2pi::*;
pub use cvtps2qq::*;
pub use cvtps2udq::*;
pub use cvtps2uqq::*;
pub use cvtqq2pd::*;
pub use cvtqq2ps::*;
pub use cvtsd2si::*;
pub use cvtsd2ss::*;
pub use cvtsd2usi::*;
pub use cvtsi2sd::*;
pub use cvtsi2ss::*;
pub use cvtss2sd::*;
pub use cvtss2si::*;
pub use cvtss2usi::*;
pub use cvttpd2dq::*;
pub use cvttpd2pi::*;
pub use cvttpd2qq::*;
pub use cvttpd2udq::*;
pub use cvttpd2uqq::*;
pub use cvttps2dq::*;
pub use cvttps2pi::*;
pub use cvttps2qq::*;
pub use cvttps2uqq::*;
pub use cvttsd2si::*;
pub use cvttsd2usi::*;
pub use cvttss2si::*;
pub use cvttss2usi::*;
pub use cvtudq2pd::*;
pub use cvtudq2ps::*;
pub use cvtuqq2pd::*;
pub use cvtuqq2ps::*;
pub use cvtusi2sd::*;
pub use cvtusi2ss::*;
pub use cwd_cdq_cqo::*;
