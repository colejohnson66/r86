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
mod kadd;
mod kand;
mod kandn;
mod kmov;
mod knot;
mod kor;
mod kortest;
mod kshiftl;
mod kshiftr;
mod ktest;
mod kunpckb;
mod kxnor;
mod kxor;

pub use kadd::*;
pub use kand::*;
pub use kandn::*;
pub use kmov::*;
pub use knot::*;
pub use kor::*;
pub use kortest::*;
pub use kshiftl::*;
pub use kshiftr::*;
pub use ktest::*;
pub use kunpckb::*;
pub use kxnor::*;
pub use kxor::*;
