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
mod xabort;
mod xacquire_xrelease;
mod xadd;
mod xbegin;
mod xchg;
mod xend;
mod xgetbv;
mod xlat;
mod xor;
mod xorpd;
mod xorps;
mod xrstor;
mod xrstors;
mod xsave;
mod xsavec;
mod xsaveopt;
mod xsaves;
mod xsetbv;
mod xtest;

pub use xabort::*;
pub use xacquire_xrelease::*;
pub use xadd::*;
pub use xbegin::*;
pub use xchg::*;
pub use xend::*;
pub use xgetbv::*;
pub use xlat::*;
pub use xor::*;
pub use xorpd::*;
pub use xorps::*;
pub use xrstor::*;
pub use xrstors::*;
pub use xsave::*;
pub use xsavec::*;
pub use xsaveopt::*;
pub use xsaves::*;
pub use xsetbv::*;
pub use xtest::*;
