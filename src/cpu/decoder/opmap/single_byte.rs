/* ============================================================================
 * File:   single_byte.rs
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
//use crate::cpu::decoder::flag::*;
use crate::cpu::decoder::OpcodeMapEntry;
use crate::cpu::instr::*;

/* A.2.1: Codes for addressing method:
 * -----------------------------------
 * A - Direct address: the instruction has no ModR/M byte; the address
 *     of the operand is encoded in the instruction. No base register,
 *     index register, or scaling factor can be applied (eg.
 *     JMPF_Ap [EA]).
 * B - The VEX.vvvv field of the VEX prefix selects a general purpose
 *     register.
 * C - The reg field of the ModR/M byte selects a control register (eg.
 *     MOV_RdCd [0F 20] or MOV_CdRd [0F 22]).
 * D - The reg field of the ModR/M byte selects a debug register (eg.
 *     MOV_RdDd [0F 21] or MOV_DdRd [0F 23]).
 * E - A ModR/M byte follows the opcode and specifies the operand. The
 *     operand is either a general-purpose register or a memory
 *     address. If it is a memory address, the address is computed from
 *     a segment register and any of the following values: a base
 *     register, an index register, a scaling factor, [and] a
 *     displacement.
 * F - EFLAGS/RFLAGS register.
 * G - The reg field of the ModR/M byte selects a general register (eg.
 *     AX [000b]).
 * H - The VEX.vvvv field of the VEX prefix selects a 128-bit XMM
 *     register or a 256-bit YMM register, determined by the operand
 *     type. For legacy SSE encodings this operand does not exist,
 *     changing the instruction to [a] destructive form.
 * I - Immediate data: the operand value is encoded in subsequent bytes
 *     of the instruction.
 * J - The instruction contains a relative offset to be added to the
 *     instruction pointer register (eg. JMP_Jz [E9]).
 * K - UNUSED
 * L - The upper 4 bits of the 8-bit immediate selects a 128-bit XMM
 *     register or a 256-bit YMM register, determined by the operand
 *     type. (the MSB is ignored in 32-bit mode)
 * M - The ModR/M byte may refer only to memory (eg. BOUND_GvMa [62],
 *     LES_GvMp [C4], LDS_GzMp [C5], LSS_GvMp [0F B2],
 *     LFS_GvMp [0F B4], LGS_GvMp [0F B5], CMPXCHG8B_Mq [0F C7 /1]).
 * N - The R/M field of the ModR/M byte selects a packed quadword MMX
 *     technology register.
 * O - UNUSED
 * P - The reg field of the ModR/M byte selects a packed quadword MMX
 *     technology register.
 * Q - A ModR/M byte follows the opcode and specifies the operand. The
 *     operand is either an MMX technology register or a memory
 *     address. If it is a memory address, the address is computed from
 *     a segment register and any of the following values: a base
 *     register, an index register, a scaling factor, and a
 *     displacement.
 * R - The R/M field of the ModR/M byte may refer only to a general
 *     register (eg. MOV_RdCd [0F 20], MOV_RdDd [0F 21],
 *     MOV_CdRd [0F 22], or MOV_DdRd [0F 23]).
 * S - The reg field of the ModR/M byte selects a segment register (eg.
 *     MOV_EvSw [8C] or MOV_SwEw [8E]).
 * T - UNUSED
 * U - The R/M field of the ModR/M byte selects a 128-bit XMM register
 *     or a 256-bit YMM register, determined by the operand type.
 * V - The reg field of the ModR/M byte selects a 128-bit XMM register
 *     or a 256-bit YMM register, determined by the operand type.
 * W - A ModR/M byte follows the opcode and specifies the operand. The
 *     operand is either a 128-bit XMM register, a 256-bit YMM register
 *     (determined by the operand type), or a memory address. If it is
 *     a memory address, the address is computed from a segment
 *     register and any of the following values: a base register, an
 *     index register, a scaling factor, and a displacement.
 * X - Memory addressed by the DS:rSI register pair (eg. MOVS [??], CMPS [??], OUTS [??], or LODS [??]).
 * Y - Memory addressed by the ES:rDI register pair (eg. MOVS [??], CMPS [??], INS [??], STOS [??], or SCAS [??]).
 * Z - UNUSED
 *
 * A.2.2: Codes for operand type:
 * ------------------------------
 * a  - Two one-word operands in memory or two doubleword operands in
 *      memory, depending on operand-size attribute (used only by the
 *      BOUND [??] instruction).
 * b  - Byte, regardless of operand-size attribute.
 * c  - Byte or word, depending on operand-size attribute.
 * d  - Doubleword, regardless of operand-size attribute.
 * dq - Doublequadword, regardless of operand-size attribute.
 * p  - 32-bit, 48-bit, or 80-bit pointer, depending on operand-size
 *      attribute.
 * pd - 128-bit or 256-bit packed double-precision floating-point data.
 * pi - Quadword MMX technology register (eg. MM0).
 * ps - 128-bit or 256-bit packed single-precision floating-point data.
 * q  - Quadword, regardless of operand-size attribute.
 * qq - Quadquadword (256-bits), regardless of operand-size attribute.
 * s  - 6-byte or 10-byte pseudo-descriptor.
 * sd - Scalar element of a 128-bit double-precision floating data.
 * ss - Scalar element of a 128-bit single-precision floating data.
 * si - Doubleword integer register (eg. EAX).
 * v  - Word, doubleword, or quadword (in 64-bit mode), depending on
 *      operand-size attribute.
 * w  - Word, regardless of the operand-size attribute.
 * x  - dq or qq based on the operand-size attribute.
 * y  - Doubleword or quadword (in 64-bit mode), depending on operand-
 *      size attribute.
 * z  - Word for 16-bit operand-size or doubleword for 32- or 64-bit
 *      operand size.
 *
 * - Intel 64 and IA-32 Architectures Software Developer's Manual
 *   - Version 71 (October 2019)
 *   - Volume 2D, pages A-1 to A-3
 */
/* Custom addressing methods:
 * BE - Same as `E`, but for 128-bit BNDx registers
 * BG - Same as `G`, but for 128-bit BNDx registers
 * KE - Same as `E`, but for mask registers
 * KG - Same as `G`, but for mask registers
 * KR - Same as `R`, but for mask registers
 *
 * Custom operand types:
 * dqq - Doublequadquadword (512-bits), regardles of operand-size
 *       attribute
 * t   - Tenbyte (80 bits)
 *
 * Custom suffixes:
 * Op16 - 16-bit version
 * Op32 - 32-bit version
 * Op64 - 64-bit version
 * V    - VEX prefix with the L bits set to anything (`LIG`)
 * V128 - VEX prefix with the L bit set to 0 (128-bit XMM registers)
 * V256 - VEX prefix with the L bit set to 1 (256-bit YMM registers)
 * E    - EVEX prefix with the L bits set to anything (`LIG`)
 * E128 - EVEX prefix with the L bits set to 00 (128-bit XMM registers)
 * E256 - EVEX prefix with the L bits set to 01 (256-bit YMM registers)
 * E512 - EVEX prefix with the L bits set to 10 (512-bit ZMM registers)
 */

pub struct SingleByte {
    ops: [Vec<OpcodeMapEntry>; 256],
}

impl SingleByte {
    pub fn new() -> SingleByte {
        let op00 = vec![];

        let op01 = vec![];

        let op02 = vec![];

        let op03 = vec![];

        let op04 = vec![];

        let op05 = vec![];

        let op06 = vec![];

        let op07 = vec![];

        let op08 = vec![];

        let op09 = vec![];

        let op0a = vec![];

        let op0b = vec![];

        let op0c = vec![];

        let op0d = vec![];

        let op0e = vec![];

        let op0f = vec![];

        let op10 = vec![];

        let op11 = vec![];

        let op12 = vec![];

        let op13 = vec![];

        let op14 = vec![];

        let op15 = vec![];

        let op16 = vec![];

        let op17 = vec![];

        let op18 = vec![];

        let op19 = vec![];

        let op1a = vec![];

        let op1b = vec![];

        let op1c = vec![];

        let op1d = vec![];

        let op1e = vec![];

        let op1f = vec![];

        let op20 = vec![];

        let op21 = vec![];

        let op22 = vec![];

        let op23 = vec![];

        let op24 = vec![];

        let op25 = vec![];

        let op26 = vec![];

        let op27 = vec![];

        let op28 = vec![];

        let op29 = vec![];

        let op2a = vec![];

        let op2b = vec![];

        let op2c = vec![];

        let op2d = vec![];

        let op2e = vec![];

        let op2f = vec![];

        let op30 = vec![];

        let op31 = vec![];

        let op32 = vec![];

        let op33 = vec![];

        let op34 = vec![];

        let op35 = vec![];

        let op36 = vec![];

        let op37 = vec![OpcodeMapEntry::new(Aaa::noarg, vec![])];

        let op38 = vec![];

        let op39 = vec![];

        let op3a = vec![];

        let op3b = vec![];

        let op3c = vec![];

        let op3d = vec![];

        let op3e = vec![];

        let op3f = vec![];

        let op40 = vec![];

        let op41 = vec![];

        let op42 = vec![];

        let op43 = vec![];

        let op44 = vec![];

        let op45 = vec![];

        let op46 = vec![];

        let op47 = vec![];

        let op48 = vec![];

        let op49 = vec![];

        let op4a = vec![];

        let op4b = vec![];

        let op4c = vec![];

        let op4d = vec![];

        let op4e = vec![];

        let op4f = vec![];

        let op50 = vec![];

        let op51 = vec![];

        let op52 = vec![];

        let op53 = vec![];

        let op54 = vec![];

        let op55 = vec![];

        let op56 = vec![];

        let op57 = vec![];

        let op58 = vec![];

        let op59 = vec![];

        let op5a = vec![];

        let op5b = vec![];

        let op5c = vec![];

        let op5d = vec![];

        let op5e = vec![];

        let op5f = vec![];

        let op60 = vec![];

        let op61 = vec![];

        let op62 = vec![];

        let op63 = vec![];

        let op64 = vec![];

        let op65 = vec![];

        let op66 = vec![];

        let op67 = vec![];

        let op68 = vec![];

        let op69 = vec![];

        let op6a = vec![];

        let op6b = vec![];

        let op6c = vec![];

        let op6d = vec![];

        let op6e = vec![];

        let op6f = vec![];

        let op70 = vec![];

        let op71 = vec![];

        let op72 = vec![];

        let op73 = vec![];

        let op74 = vec![];

        let op75 = vec![];

        let op76 = vec![];

        let op77 = vec![];

        let op78 = vec![];

        let op79 = vec![];

        let op7a = vec![];

        let op7b = vec![];

        let op7c = vec![];

        let op7d = vec![];

        let op7e = vec![];

        let op7f = vec![];

        let op80 = vec![];

        let op81 = vec![];

        let op82 = vec![];

        let op83 = vec![];

        let op84 = vec![];

        let op85 = vec![];

        let op86 = vec![];

        let op87 = vec![];

        let op88 = vec![];

        let op89 = vec![];

        let op8a = vec![];

        let op8b = vec![];

        let op8c = vec![];

        let op8d = vec![];

        let op8e = vec![];

        let op8f = vec![];

        let op90 = vec![];

        let op91 = vec![];

        let op92 = vec![];

        let op93 = vec![];

        let op94 = vec![];

        let op95 = vec![];

        let op96 = vec![];

        let op97 = vec![];

        let op98 = vec![];

        let op99 = vec![];

        let op9a = vec![];

        let op9b = vec![];

        let op9c = vec![];

        let op9d = vec![];

        let op9e = vec![];

        let op9f = vec![];

        let opa0 = vec![];

        let opa1 = vec![];

        let opa2 = vec![];

        let opa3 = vec![];

        let opa4 = vec![];

        let opa5 = vec![];

        let opa6 = vec![];

        let opa7 = vec![];

        let opa8 = vec![];

        let opa9 = vec![];

        let opaa = vec![];

        let opab = vec![];

        let opac = vec![];

        let opad = vec![];

        let opae = vec![];

        let opaf = vec![];

        let opb0 = vec![];

        let opb1 = vec![];

        let opb2 = vec![];

        let opb3 = vec![];

        let opb4 = vec![];

        let opb5 = vec![];

        let opb6 = vec![];

        let opb7 = vec![];

        let opb8 = vec![];

        let opb9 = vec![];

        let opba = vec![];

        let opbb = vec![];

        let opbc = vec![];

        let opbd = vec![];

        let opbe = vec![];

        let opbf = vec![];

        let opc0 = vec![];

        let opc1 = vec![];

        let opc2 = vec![];

        let opc3 = vec![];

        let opc4 = vec![];

        let opc5 = vec![];

        let opc6 = vec![];

        let opc7 = vec![];

        let opc8 = vec![];

        let opc9 = vec![];

        let opca = vec![];

        let opcb = vec![];

        let opcc = vec![];

        let opcd = vec![];

        let opce = vec![];

        let opcf = vec![];

        let opd0 = vec![];

        let opd1 = vec![];

        let opd2 = vec![];

        let opd3 = vec![];

        let opd4 = vec![];

        let opd5 = vec![];

        let opd6 = vec![];

        let opd7 = vec![];

        let opd8 = vec![];

        let opd9 = vec![];

        let opda = vec![];

        let opdb = vec![];

        let opdc = vec![];

        let opdd = vec![];

        let opde = vec![];

        let opdf = vec![];

        let ope0 = vec![];

        let ope1 = vec![];

        let ope2 = vec![];

        let ope3 = vec![];

        let ope4 = vec![];

        let ope5 = vec![];

        let ope6 = vec![];

        let ope7 = vec![];

        let ope8 = vec![];

        let ope9 = vec![];

        let opea = vec![];

        let opeb = vec![];

        let opec = vec![];

        let oped = vec![];

        let opee = vec![];

        let opef = vec![];

        let opf0 = vec![];

        let opf1 = vec![];

        let opf2 = vec![];

        let opf3 = vec![];

        let opf4 = vec![];

        let opf5 = vec![];

        let opf6 = vec![];

        let opf7 = vec![];

        let opf8 = vec![];

        let opf9 = vec![];

        let opfa = vec![];

        let opfb = vec![];

        let opfc = vec![];

        let opfd = vec![];

        let opfe = vec![];

        let opff = vec![];

        let ops: [Vec<OpcodeMapEntry>; 256] = [
            op00, op01, op02, op03, op04, op05, op06, op07, op08, op09, op0a, op0b, op0c, op0d,
            op0e, op0f, op10, op11, op12, op13, op14, op15, op16, op17, op18, op19, op1a, op1b,
            op1c, op1d, op1e, op1f, op20, op21, op22, op23, op24, op25, op26, op27, op28, op29,
            op2a, op2b, op2c, op2d, op2e, op2f, op30, op31, op32, op33, op34, op35, op36, op37,
            op38, op39, op3a, op3b, op3c, op3d, op3e, op3f, op40, op41, op42, op43, op44, op45,
            op46, op47, op48, op49, op4a, op4b, op4c, op4d, op4e, op4f, op50, op51, op52, op53,
            op54, op55, op56, op57, op58, op59, op5a, op5b, op5c, op5d, op5e, op5f, op60, op61,
            op62, op63, op64, op65, op66, op67, op68, op69, op6a, op6b, op6c, op6d, op6e, op6f,
            op70, op71, op72, op73, op74, op75, op76, op77, op78, op79, op7a, op7b, op7c, op7d,
            op7e, op7f, op80, op81, op82, op83, op84, op85, op86, op87, op88, op89, op8a, op8b,
            op8c, op8d, op8e, op8f, op90, op91, op92, op93, op94, op95, op96, op97, op98, op99,
            op9a, op9b, op9c, op9d, op9e, op9f, opa0, opa1, opa2, opa3, opa4, opa5, opa6, opa7,
            opa8, opa9, opaa, opab, opac, opad, opae, opaf, opb0, opb1, opb2, opb3, opb4, opb5,
            opb6, opb7, opb8, opb9, opba, opbb, opbc, opbd, opbe, opbf, opc0, opc1, opc2, opc3,
            opc4, opc5, opc6, opc7, opc8, opc9, opca, opcb, opcc, opcd, opce, opcf, opd0, opd1,
            opd2, opd3, opd4, opd5, opd6, opd7, opd8, opd9, opda, opdb, opdc, opdd, opde, opdf,
            ope0, ope1, ope2, ope3, ope4, ope5, ope6, ope7, ope8, ope9, opea, opeb, opec, oped,
            opee, opef, opf0, opf1, opf2, opf3, opf4, opf5, opf6, opf7, opf8, opf9, opfa, opfb,
            opfc, opfd, opfe, opff,
        ];

        SingleByte { ops }
    }
}
