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
use crate::cpu::decoder::*;
use crate::cpu::instr::*;

// opcode(op, dismIntel, dismAtt, handler, arg1, arg2, arg3, arg4, suffix, lockable, extensions)
//   op:         OpcodeName
//   dismIntel:  str
//   dismAtt:    str
//   handler:    InstrHandler
//   arg1:       OperandSource
//   arg2:       OperandSource
//   arg3:       OperandSource
//   arg4:       OperandSource
//   suffix:     OpSuffix
//   lockable:   bool
//   extensions: Vec<IsaExtension>
macro_rules! opcode {
    ($op:expr, $dismIntel:literal, $dismAtt:literal, $handler:ident, $arg1:expr, $arg2:expr, $arg3:expr, $arg4:expr, $suffix:expr, $lockable:expr, $extensions:expr) => {{
        let args: [OperandSource; 4] = [
            OperandSource::from_encoded_str($arg1),
            OperandSource::from_encoded_str($arg2),
            OperandSource::from_encoded_str($arg3),
            OperandSource::from_encoded_str($arg4),
        ];
        opcodes.push(Opcode::new(
            $op,
            $dismIntel,
            $dismAtt,
            $handler,
            args,
            $suffix,
            $lockable,
            $extensions,
        ));
    }};
}

pub struct OpcodeList {
    opcodes: Vec<Opcode>,
}

impl OpcodeList {
    pub fn new() -> OpcodeList {
        let mut opcodes = Vec::with_capacity(4096);

        opcode!(
            OpcodeName::AAA,
            "aaa",
            "aaa",
            Aaa::noarg,
            "",
            "",
            "",
            "",
            OpSuffix::None,
            false,
            vec![]
        );

        opcode!(
            OpcodeName::AAD_Ib,
            "aad",
            "aad",
            Aad::ib,
            "Ib",
            "",
            "",
            "",
            OpSuffix::None,
            false,
            vec![]
        );
    }
}
