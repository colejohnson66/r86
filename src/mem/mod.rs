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
use crate::Address;

pub const CHUNK_SIZE: usize = 1 * 1024 * 1024;

pub struct Ram {
    chunks: Vec<&'static dyn ChunkBase>,
}

pub trait ChunkBase {
    fn u8(&self, idx: Address) -> u8;
    fn set_u8(&mut self, idx: Address, val: u8);

    /// NOTE: Must be aligned to a multiple of two
    fn u16(&self, idx: Address) -> u16;
    /// NOTE: Must be aligned to a multiple of two
    fn set_u16(&mut self, idx: Address, val: u16);

    /// NOTE: Must be aligned to a multiple of four
    fn u32(&self, idx: Address) -> u32;
    /// NOTE: Must be aligned to a multiple of four
    fn set_u32(&mut self, idx: Address, val: u32);

    /// NOTE: Must be aligned to a multiple of eight
    fn u64(&self, idx: Address) -> u64;
    /// NOTE: Must be aligned to a multiple of eight
    fn set_u64(&mut self, idx: Address, val: u64);
}

pub struct LowMegabyte {
    data: [u8; CHUNK_SIZE],
}

impl LowMegabyte {
    pub fn new() -> LowMegabyte {
        LowMegabyte {
            data: [0u8; CHUNK_SIZE],
        }
    }
}
