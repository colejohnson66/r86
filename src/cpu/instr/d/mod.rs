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
mod daa;
mod das;
mod dec;
mod div;
mod divpd;
mod divps;
mod divsd;
mod divss;
mod dppd;
mod dpps;

pub use daa::*;
pub use das::*;
pub use dec::*;
pub use div::*;
pub use divpd::*;
pub use divps::*;
pub use divsd::*;
pub use divss::*;
pub use dppd::*;
pub use dpps::*;
