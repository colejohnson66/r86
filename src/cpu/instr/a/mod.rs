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
mod aaa;
mod aad;
mod aam;
mod aas;
mod adc;
mod adcx;
mod add;
mod addpd;
mod addps;
mod addsd;
mod addss;
mod addsubpd;
mod addsubps;
mod adox;
mod aesdec;
mod aesdec128kl;
mod aesdec256kl;
mod aesdeclast;
mod aesdecwide128kl;
mod aesdecwide256kl;
mod aesenc;
mod aesenc128kl;
mod aesenc256kl;
mod aesenclast;
mod aesencwide128kl;
mod aesencwide256kl;
mod aesimc;
mod aeskeygenassist;
mod align;
mod and;
mod andn;
mod andnpd;
mod andnps;
mod andpd;
mod andps;
mod arpl;

pub use aaa::*;
pub use aad::*;
pub use aam::*;
pub use aas::*;
pub use adc::*;
pub use adcx::*;
pub use add::*;
pub use addpd::*;
pub use addps::*;
pub use addsd::*;
pub use addss::*;
pub use addsubpd::*;
pub use addsubps::*;
pub use adox::*;
pub use aesdec::*;
pub use aesdec128kl::*;
pub use aesdec256kl::*;
pub use aesdeclast::*;
pub use aesdecwide128kl::*;
pub use aesdecwide256kl::*;
pub use aesenc::*;
pub use aesenc128kl::*;
pub use aesenc256kl::*;
pub use aesenclast::*;
pub use aesencwide128kl::*;
pub use aesencwide256kl::*;
pub use aesimc::*;
pub use aeskeygenassist::*;
pub use align::*;
pub use and::*;
pub use andn::*;
pub use andnpd::*;
pub use andnps::*;
pub use andps::*;
pub use arpl::*;
