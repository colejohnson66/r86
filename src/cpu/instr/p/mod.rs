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
mod pabs;
mod packss;
mod packusdw;
mod packuswb;
mod padd;
mod padds;
mod paddus;
mod palignr;
mod pand;
mod pandn;
mod pause;
mod pavg;
mod pblendvb;
mod pblendvw;
mod pclmulqdq;
mod pcmpeq;
mod pcmpestri;
mod pcmpestrm;
mod pcmpgt;
mod pcmpistri;
mod pcmpistrm;
mod pdep;
mod pext;
mod pextr;
mod phadd;
mod phaddsw;
mod phminposuw;
mod phsub;
mod phsubsw;
mod pinsr;
mod pmaddubsw;
mod pmaddwd;
mod pmaxs;
mod pmaxu;
mod pmins;
mod pminu;
mod pmovmskb;
mod pmovsx;
mod pmovzx;
mod pmuldq;
mod pmulhrsw;
mod pmulhuw;
mod pmulhw;
mod pmull;
mod pmulludq;
mod pop;
mod popa;
mod popcnt;
mod popf;
mod por;
mod prefetch;
mod psadbw;
mod pshufb;
mod pshufd;
mod pshufhw;
mod pshuflw;
mod pshufw;
mod psign;
mod psll;
mod pslldq;
mod psra;
mod psrl;
mod psrldq;
mod psub;
mod psubs;
mod psubus;
mod ptest;
mod ptwrite;
mod punpckh;
mod punpckl;
mod push;
mod pusha;
mod pushf;
mod pxor;

pub use pabs::*;
pub use packss::*;
pub use packusdw::*;
pub use packuswb::*;
pub use padd::*;
pub use padds::*;
pub use paddus::*;
pub use palignr::*;
pub use pand::*;
pub use pandn::*;
pub use pause::*;
pub use pavg::*;
pub use pblendvb::*;
pub use pblendvw::*;
pub use pclmulqdq::*;
pub use pcmpeq::*;
pub use pcmpestri::*;
pub use pcmpestrm::*;
pub use pcmpgt::*;
pub use pcmpistri::*;
pub use pcmpistrm::*;
pub use pdep::*;
pub use pext::*;
pub use pextr::*;
pub use phadd::*;
pub use phaddsw::*;
pub use phminposuw::*;
pub use phsub::*;
pub use phsubsw::*;
pub use pinsr::*;
pub use pmaddubsw::*;
pub use pmaddwd::*;
pub use pmaxs::*;
pub use pmaxu::*;
pub use pmins::*;
pub use pminu::*;
pub use pmovmskb::*;
pub use pmovsx::*;
pub use pmovzx::*;
pub use pmuldq::*;
pub use pmulhrsw::*;
pub use pmulhuw::*;
pub use pmulhw::*;
pub use pmull::*;
pub use pmulludq::*;
pub use pop::*;
pub use popa::*;
pub use popcnt::*;
pub use popf::*;
pub use por::*;
pub use prefetch::*;
pub use psadbw::*;
pub use pshufb::*;
pub use pshufd::*;
pub use pshufhw::*;
pub use pshuflw::*;
pub use pshufw::*;
pub use psign::*;
pub use psll::*;
pub use pslldq::*;
pub use psra::*;
pub use psrl::*;
pub use psrldq::*;
pub use psub::*;
pub use psubs::*;
pub use psubus::*;
pub use ptest::*;
pub use ptwrite::*;
pub use punpckh::*;
pub use punpckl::*;
pub use push::*;
pub use pusha::*;
pub use pushf::*;
pub use pxor::*;
