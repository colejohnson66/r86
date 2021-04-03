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
mod a;
mod b;
mod c;
mod d;
mod e;
mod error;
mod f;
mod g;
mod h;
mod i;
mod j;
mod k;
mod l;
mod m;
mod n;
mod o;
mod p;
mod r;
mod s;
mod t;
mod u;
mod v;
mod w;
mod x;
mod z;

pub use a::*;
pub use b::*;
pub use c::*;
pub use d::*;
pub use e::*;
pub use error::*;
pub use f::*;
pub use g::*;
pub use h::*;
pub use i::*;
pub use j::*;
pub use k::*;
pub use l::*;
pub use m::*;
pub use n::*;
pub use o::*;
pub use p::*;
pub use r::*;
pub use s::*;
pub use t::*;
pub use u::*;
pub use v::*;
pub use w::*;
pub use x::*;
pub use z::*;

#[macro_export]
macro_rules! stub_handler {
    ($func:ident) => {
        pub fn $func(_cpu: &mut Cpu, _instr: &Instr) -> u32 {
            unimplemented!();
        }
    };
}
