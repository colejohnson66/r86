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
mod rangepd;
mod rangeps;
mod rangesd;
mod rangess;
mod rcl_rcr_rol_ror;
mod rcp14pd;
mod rcp14ps;
mod rcp14sd;
mod rcp14ss;
mod rcpps;
mod rcpss;
mod rdfsbase_rdgsbase;
mod rdmsr;
mod rdpid;
mod rdpkru;
mod rdpmc;
mod rdrand;
mod rdseed;
mod rdsspd;
mod rdtsc;
mod rdtscp;
mod reducepd;
mod reduceps;
mod reducesd;
mod reducess;
mod ret;
mod rndscalepd;
mod rndscaleps;
mod rndscalesd;
mod rndscaless;
mod rorx;
mod roundpd;
mod roundps;
mod roundsd;
mod roundss;
mod rsm;
mod rsqrt14pd;
mod rsqrt14ps;
mod rsqrt14sd;
mod rsqrt14ss;
mod rsqrtps;
mod rsqrtss;
mod rstorssp;

pub use rangepd::*;
pub use rangeps::*;
pub use rangesd::*;
pub use rangess::*;
pub use rcl_rcr_rol_ror::*;
pub use rcp14pd::*;
pub use rcp14ps::*;
pub use rcp14sd::*;
pub use rcp14ss::*;
pub use rcpps::*;
pub use rcpss::*;
pub use rdfsbase_rdgsbase::*;
pub use rdmsr::*;
pub use rdpid::*;
pub use rdpkru::*;
pub use rdpmc::*;
pub use rdrand::*;
pub use rdseed::*;
pub use rdsspd::*;
pub use rdtsc::*;
pub use rdtscp::*;
pub use reducepd::*;
pub use reduceps::*;
pub use reducesd::*;
pub use reducess::*;
pub use ret::*;
pub use rndscalepd::*;
pub use rndscaleps::*;
pub use rndscalesd::*;
pub use rndscaless::*;
pub use rorx::*;
pub use roundpd::*;
pub use roundps::*;
pub use roundsd::*;
pub use roundss::*;
pub use rsm::*;
pub use rsqrt14pd::*;
pub use rsqrt14ps::*;
pub use rsqrt14sd::*;
pub use rsqrt14ss::*;
pub use rsqrtps::*;
pub use rsqrtss::*;
pub use rstorssp::*;
