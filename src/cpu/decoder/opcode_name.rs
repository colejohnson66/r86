/* ============================================================================
 * File:   opcode_name.rs
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
 * X - Memory addressed by the DS:rSI register pair (eg. MOVSB [A4],
 *     CMPSB [A6], OUTSB [6E], or LODSB [AC]).
 * Y - Memory addressed by the ES:rDI register pair (eg. MOVSB [A4],
 *     CMPSB [A6], INSB [6C], STOS [AA], or SCAS [AE]).
 * Z - UNUSED
 *
 * A.2.2: Codes for operand type:
 * ------------------------------
 * a  - Two one-word operands in memory or two doubleword operands in
 *      memory, depending on operand-size attribute (used only by the
 *      BOUND [62] instruction).
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
 *   - Version 73 (November 2020)
 *   - Volume 2D, pages A-1 to A-3
 */
/* Custom addressing methods:
 * BE - Same as `E`, but for 128-bit BNDx registers
 * BG - Same as `G`, but for 128-bit BNDx registers
 * KE - Same as `E`, but for mask registers
 * KG - Same as `G`, but for mask registers
 * KR - Same as `R`, but for mask registers
 * O  - Offset (Ob: moffs8)
 * VM - vm##x
 *
 * Custom operand types:
 * dqq - Doublequadquadword (512-bits), regardless of operand-size
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
pub enum OpcodeName {
    Error,

    // [37] AAA
    Aaa,

    // [D5 ib] AAD imm8
    AadIb,

    // [D4 ib] AAM imm8
    AamIb,

    // [3F] AAS
    Aas,

    // [14 ib] ADC AL, imm8
    AdcALIb,
    // [15 iw] ADC AX, imm16
    AdcAXIw,
    // [15 id] ADC EAX, imm32
    AdcEAXId,
    // [REX.W 15 id] ADC RAX, imm32
    AdcRAXId,
    // [80 /2 ib] ADC r/m8, imm8
    // [REX 80 /2 ib] ADC r/m8, imm8
    AdcEbIb,
    // [81 /2 iw] ADC r/m16, imm16
    AdcEwIw,
    // [81 /2 iw] ADC r/m32, imm32
    AdcEdId,
    // [REX.W 81 /2 id] ADC r/m64, imm32
    AdcEqId,
    // [83 /2 ib] ADC r/m16, imm8
    AdcGwIb,
    // [83 /2 ib] ADC r/m32, imm8
    AdcGdIb,
    // [REX.W 83 /2 ib] ADDC r/m64, imm8
    AdcGqIb,
    // [10 /r] ADC r/m8, r8
    // [REX 10 /r] ADC r/m8, r8
    AdcEbGb,
    // [11 /r] ADC r/m16, r16
    AdcEwGw,
    // [11 /r] ADC r/m32, r32
    AdcEdGd,
    // [REX.W 11 /r] ADC r/m64, r64
    AdcEqGq,
    // [12 /r] ADC r8, r/m8
    // [REX 12 /r] ADC r8, r/m8
    AdcGbEb,
    // [13 /r] ADC r16, r/m16
    AdcGwEw,
    // [13 /r] ADC r32, r/m32
    AdcGdEd,
    // [REX.W 13 /r] ADC r64, r/m64
    AdcGqEq,

    // [66 0F 38 F6 /r] ADCX r32, r/m32
    AdcxGdEd,
    // [66 REX.W 0F 38 F6 /r] ADCX r64, r/m64
    AdcxGqEq,

    // [04 ib] ADD AL, imm8
    AddALIb,
    // [05 iw] ADD AX, imm16
    AddAXIw,
    // [05 id] ADD EAX, imm32
    AddEAXId,
    // [REX.W 05 id] ADD RAX, imm32
    AddRAXId,
    // [80 /0 ib] ADD r/m8, imm8
    // [REX 80 /0 ib] ADD r/m8, imm8
    AddEbIb,
    // [81 /0 iw] ADD r/m16, imm16
    AddEwIw,
    // [81 /0 iw] ADD r/m32, imm32
    AddEdId,
    // [REX.W 81 /0 id] ADD r/m64, imm32
    AddEqId,
    // [83 /0 ib] ADD r/m16, imm8
    AddGwIb,
    // [83 /0 ib] ADD r/m32, imm8
    AddGdIb,
    // [REX.W 83 /0 ib] ADDC r/m64, imm8
    AddGqIb,
    // [00 /r] ADD r/m8, r8
    // [REX 00 /r] ADD r/m8, r8
    AddEbGb,
    // [01 /r] ADD r/m16, r16
    AddEwGw,
    // [01 /r] ADD r/m32, r32
    AddEdGd,
    // [REX.W 01 /r] ADD r/m64, r64
    AddEqGq,
    // [02 /r] ADD r8, r/m8
    // [REX 02 /r] ADD r8, r/m8
    AddGbEb,
    // [03 /r] ADD r16, r/m16
    AddGwEw,
    // [03 /r] ADD r32, r/m32
    AddGdEd,
    // [REX.W 03 /r] ADD r64, r/m64
    AddGqEq,

    // [66 0F 58 /r] ADDPD xmm1, xmm2/m128
    AddpdVdqWdq,
    // [VEX.128.66.0F.WIG 58 /r] VADDPD xmm1, xmm2, xmm3/m128
    VaddpdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 58 /r] VADDPD ymm1, ymm2, ymm3/m256
    VaddpdVqqHqqWqqV256,
    // [EVEX.128.66.0F.W1 58 /r] VADDPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VaddpdVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 58 /r] VADDPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VaddpdVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 58 /r] VADDPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VaddpdVdqqHdqqWdqqE512,

    // [NP 0F 58 /r] ADDPS xmm1, xmm2/m128
    AddpsVdqWdq,
    // [VEX.128.0F.WIG 58 /r] VADDPS xmm1, xmm2, xmm3/m128
    VaddpsVdqHdqWdqV128,
    // [VEX.256.0F.WIG 58 /r] VADDPS ymm1, ymm2, ymm3/m256
    VaddpsVqqHqqWqqV256,
    // [EVEX.128.0F.W1 58 /r] VADDPS xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VaddpsVdqHdqWdqE128,
    // [EVEX.256.0F.W1 58 /r] VADDPS ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VaddpsVqqHqqWqqE256,
    // [EVEX.512.0F.W1 58 /r] VADDPS zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VaddpsVdqqHdqqWdqqE512,

    // [F2 0F 58 /r] ADDSD xmm1, xmm2/m64
    AddsdVdqWq,
    // [VEX.LIG.F2.OF.WIG 58 /r] VADDSD xmm1, xmm2, xmm3/m64
    VaddsdVdqHdqWqV,
    // [EVEX.LIG.F2.0F.W1 58 /r] VADDSD xmm1 {k1}{z}, xmm2, xmm3/m64{er}
    VaddsdVdqHdqWqE,

    // [F3 0F 58 /r] ADDSS xmm1, xmm2/m32
    AddssVdqWd,
    // [VEX.LIG.F3.0F.WIG 58 /r] VADDSS xmm1, xmm2, xmm3/m32
    VaddssVdqHdqWdV,
    // [EVEX.LIG.F3.0F.W0 58 /r] VADDSS xmm1 {k1}{z}, xmm2, xmm3/m32{er}
    VaddssVdqHdqWdE,

    // [66 0F D0 /r] ADDSUBPD xmm1, xmm2/m128
    AddsubpdVdqWdq,
    // [VEX.128.66.0F.WIG D0 /r] VADDSUBPD xmm1, xmm2, xmm3/m128
    VaddsubpdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG D0 /r] VADDSUBPD ymm1, ymm2, ymm3/m256
    VaddsubpdVqqHqqWqqV256,

    // [F2 0F D0 /r] ADDSUBPS xmm1, xmm2/m128
    AddsubpsVdqWdq,
    // [VEX.128.F2.0F.WIG D0 /r] VADDSUBPS xmm1, xmm2, xmm3/m128
    VaddsubpsVdqHdqWdqV128,
    // [VEX.256.F2.0F.WIG D0 /r] VADDSUBPS ymm1, ymm2, ymm3/m256
    VaddsubpsVqqHqqWqqV256,

    // [F3 0F 38 F6 /r] ADOX r32, r/m32
    AdoxGdEd,
    // [F3 REX.W 0F 38 F6 /r] ADOX r64, r/m64
    AdoxGqEq,

    // [66 0F 38 DE /r] AESDEC xmm1, xmm2/m128
    AesdecVdqWdq,
    // [VEX.128.66.0F38.WIG DE /r] VAESDEC xmm1, xmm2, xmm3/m128
    VaesdecVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG DE /r] VAESDEC ymm1, ymm2, ymm3/m256
    VaesdecVqqHqqWqqV256,
    // [EVEX.128.66.0F38.WIG DE /r] VAESDEC xmm1, xmm2, xmm3/m128
    VaesdecVdqHdqWdqE128,
    // [EVEX.256.66.0F38.WIG DE /r] VAESDEC ymm1, ymm2, ymm3/m256
    VaesdecVqqHqqWqqE256,
    // [EVEX.512.66.0F38.WIG DE /r] VAESDEC zmm1, zmm2, zmm3/m512
    VaesdecVdqqHdqqWdqqE512,

    // [F3 0F 38 DD !(11):rrr:bbb] AESDEC128KL xmm, m384
    Aesdec128klVdqM,

    // [F3 0F 38 DF !(11):rrr:bbb] AESDEC256KL xmm, m512
    Aesdec256klVdqMdqq,

    // [66 0F 38 DF /r] AESDECLAST xmm1, xmm2/m128
    AesdeclastVdqWdq,
    // [VEX.128.66.0F38.WIG DF /r] VAESDECLAST xmm1, xmm2, xmm3/m128
    VaesdeclastVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG DF /r] VAESDECLAST ymm1, ymm2, ymm3/m256
    VaesdeclastVqqHqqWqqV256,
    // [EVEX.128.66.0F38.WIG DF /r] VAESDECLAST xmm1, xmm2, xmm3/m128
    VaesdeclastVdqHdqWdqE128,
    // [EVEX.256.66.0F38.WIG DF /r] VAESDECLAST ymm1, ymm2, ymm3/m256
    VaesdeclastVqqHqqWqqE256,
    // [EVEX.512.66.0F38.WIG DF /r] VAESDECLAST zmm1, zmm2, zmm3/m512
    VaesdeclastVdqqHdqqWdqqE512,

    // [F3 0F 38 D8 !(11):rrr:bbb] AESDECWIDE128KL m384, <XMM0-7>
    Aesdecwide128klM,

    // [F3 0F 38 D8 !(11):rrr:bbb] AESDECWIDE256KL m512, <XMM0-7>
    Aesdecwide256klMdqq,

    // [66 0F 38 DC /r] AESENC xmm1, xmm2/m128
    AesencVdqWdq,
    // [VEX.128.66.0F38.WIG DC /r] VAESENC xmm1, xmm2, xmm3/m128
    VaesencVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG DC /r] VAESENC ymm1, ymm2, ymm3/m256
    VaesencVqqHqqWqqV256,
    // [EVEX.128.66.0F38.WIG DC /r] VAESENC xmm1, xmm2, xmm3/m128
    VaesencVdqHdqWdqE128,
    // [EVEX.256.66.0F38.WIG DC /r] VAESENC ymm1, ymm2, ymm3/m256
    VaesencVqqHqqWqqE256,
    // [EVEX.512.66.0F38.WIG DC /r] VAESENC zmm1, zmm2, zmm3/m512
    VaesencVdqqHdqqWdqqE512,

    // [F3 0F 38 DC !(11):rrr:bbb] AESENC128KL xmm, m384
    Aesenc128klVdqM,

    // [F3 0F 38 DE !(11):rrr:bbb] AESENC256KL xmm, m512
    Aesenc256klVdqMdqq,

    // [66 0F 38 DD /r] AESENCLAST xmm1, xmm2/m128
    AesenclastVdqWdq,
    // [VEX.128.66.0F38.WIG DD /r] VAESENCLAST xmm1, xmm2, xmm3/m128
    VaesenclastVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG DD /r] VAESENCLAST ymm1, ymm2, ymm3/m256
    VaesenclastVqqHqqWqqV256,
    // [EVEX.128.66.0F38.WIG DD /r] VAESENCLAST xmm1, xmm2, xmm3/m128
    VaesenclastVdqHdqWdqE128,
    // [EVEX.256.66.0F38.WIG DD /r] VAESENCLAST ymm1, ymm2, ymm3/m256
    VaesenclastVqqHqqWqqE256,
    // [EVEX.512.66.0F38.WIG DD /r] VAESENCLAST zmm1, zmm2, zmm3/m512
    VaesenclastVdqqHdqqWdqqE512,

    // [F3 0F 38 D8 !(11):000:bbb] AESENCWIDE128KL m384, <XMM0-7>
    Aesencwide128klM,

    // [F3 0F 38 D8 !(11):010:bbb] AESENCWIDE256KL m512, <XMM0-7>
    Aesencwide256klMdqq,

    // [66 0F 38 DB /r] AESIMC xmm1, xmm2/m128
    AesimcVdqWdq,
    // [VEX.128.66.0F38.WIG DB /r] VAESIMC xmm1, xmm2/m128
    VaesimcVdqWdqV128,

    // [66 0F 3A DF /r ib] AESKEYGENASSIST xmm1, xmm2/m128, imm8
    AeskeygenassistVdqWdqIb,
    // [VEX.128.66.0F3A.WIG DF /r ib] AESKEYGENASSIST xmm1, xmm2/m128, imm8
    VaeskeygenassistVdqWdqIbV128,

    // [24 ib] AND AL, imm8
    AndALIb,
    // [25 iw] AND AX, imm16
    AndAXIw,
    // [25 id] AND EAX, imm32
    AndEAXId,
    // [REX.W 25 id] AND RAX, imm32
    AndRAXId,
    // [80 /4 ib] AND r/m8, imm8
    // [REX 80 /4 ib] AND r/m8, imm8
    AndEbIb,
    // [81 /4 iw] AND r/m16, imm16
    AndEwIw,
    // [81 /4 iw] AND r/m32, imm32
    AndEdId,
    // [REX.W 81 /4 id] AND r/m64, imm32
    AndEqId,
    // [83 /4 ib] AND r/m16, imm8
    AndGwIb,
    // [83 /4 ib] AND r/m32, imm8
    AndGdIb,
    // [REX.W 83 /4 ib] ANDC r/m64, imm8
    AndGqIb,
    // [20 /r] AND r/m8, r8
    // [REX 20 /r] AND r/m8, r8
    AndEbGb,
    // [21 /r] AND r/m16, r16
    AndEwGw,
    // [21 /r] AND r/m32, r32
    AndEdGd,
    // [REX.W 21 /r] AND r/m64, r64
    AndEqGq,
    // [22 /r] AND r8, r/m8
    // [REX 22 /r] AND r8, r/m8
    AndGbEb,
    // [23 /r] AND r16, r/m16
    AndGwEw,
    // [23 /r] AND r32, r/m32
    AndGdEd,
    // [REX.W 23 /r] AND r64, r/m64
    AndGqEq,

    // [VEX.LZ.0F38.W0 F2 /r] ANDN r32a, r32b, r/m32
    AndnGdBdEd,
    // [VEX.LZ.0F38.W1 F2 /r] ANDN r64a, r64b, r/m64
    AndnGqBqEq,

    // [66 0F 54 /r] ANDPD xmm1, xmm2/m128
    AndpdVdqWdq,
    // [VEX.128.66.0F 54 /r] VANDPD xmm1, xmm2, xmm3/m128
    VandpdVdqHdqWdqV128,
    // [VEX.256.66.0F 54 /r] VANDPD ymm1, ymm2, ymm3/m256
    VandpdVqqHqqWqqV256,
    // [EVEX.128.66.0F.W1 54 /r] VANDPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VandpdVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 54 /r] VANDPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VandpdVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 54 /r] VANDPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VandpdVdqqHdqqWdqqE512,

    // [NP 0F 54 /r] ANDPS xmm1, xmm2/m128
    AndpsVdqWdq,
    // [VEX.128.0F 54 /r] VANDPS xmm1, xmm2, xmm3/m128
    VandpsVdqHdqWdqV128,
    // [VEX.256.0F 54 /r] VANDPS ymm1, ymm2, ymm3/m256
    VandpsVqqHqqWqqV256,
    // [EVEX.128.0F.W0 54 /r] VANDPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VandpsVdqHdqWdqE128,
    // [EVEX.256.0F.W0 54 /r] VANDPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VandpsVqqHqqWqqE256,
    // [EVEX.512.0F.W0 54 /r] VANDPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VandpsVdqqHdqqWdqqE512,

    // [66 0F 55 /r] ANDNPD xmm1, xmm2/m128
    AndnpdVdqWdq,
    // [VEX.128.66.0F 55 /r] VANDNPD xmm1, xmm2, xmm3/m128
    VandnpdVdqHdqWdqV128,
    // [VEX.256.66.0F 55 /r] VANDNPD ymm1, ymm2, ymm3/m256
    VandnpdVqqHqqWqqV256,
    // [EVEX.128.66.0F.W1 55 /r] VANDNPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VandnpdVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 55 /r] VANDNPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VandnpdVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 55 /r] VANDNPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VandnpdVdqqHdqqWdqqE512,

    // [NP 0F 55 /r] ANDNPS xmm1, xmm2/m128
    AndnpsVdqWdq,
    // [VEX.128.0F 55 /r] VANDNPS xmm1, xmm2, xmm3/m128
    VandnpsVdqHdqWdqV128,
    // [VEX.256.0F 55 /r] VANDNPS ymm1, ymm2, ymm3/m256
    VandnpsVqqHqqWqqV256,
    // [EVEX.128.0F.W0 55 /r] VANDNPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VandnpsVdqHdqWdqE128,
    // [EVEX.256.0F.W0 55 /r] VANDNPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VandnpsVqqHqqWqqE256,
    // [EVEX.512.0F.W0 55 /r] VANDNPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VandnpsVdqqHdqqWdqqE512,

    // [63 /r] ARPL r/m16, r16
    ArplEwGw,

    // [VEX.LZ.0F38.W0 F7 /r] BEXTR r32a, r/m32, r32b
    BextrGdEdBd,
    // [VEX.LZ.0F38.W1 F7 /r] BEXTR r64a, r/m64, r64b
    BextrGqEqBq,

    // [66 0F 3A 0D /r ib] BLENDPD xmm1, xmm2/m128, imm8
    BlendpdVdqWdqIb,
    // [VEX.128.66.0F3A.WIG 0D /r ib] VBLENDPD xmm1, xmm2, xmm3/m128, imm8
    VblendpdVdqHdqWdqIbV128,
    // [VEX.256.66.0F3A.WIG 0D /r ib] VBLENDPD ymm1, ymm2, ymm3/m256, imm8
    VblendpdVqqHqqWqqIbV256,

    // [66 0F 3A 0C /r ib] BLENDPS xmm1, xmm2/m128, imm8
    BlendpsVdqWdqIb,
    // [VEX.128.66.0F3A.WIG 0C /r ib] VBLENDPS xmm1, xmm2, xmm3/m128, imm8
    VblendpsVdqHdqWdqIbV128,
    // [VEX.256.66.0F3A.WIG 0C /r ib] VBLENDPS ymm1, ymm2, ymm3/m256, imm8
    VblendpsVqqHqqWqqIbV256,

    // [66 0F 38 15 /r] BLENDVPD xmm1, xmm2/m128, <XMM0>
    BlendvpdVdqWdq,
    // [VEX.128.66.0F3A.W0 4B /r /is4] VBLENDVPD xmm1, xmm2, xmm3/m128, xmm4
    VblendvpdVdqHdqWdqIbV128,
    // [VEX.256.66.0F3A.W0 4B /r /is4] VBLENDVPD ymm1, ymm2, ymm3/m256, ymm4
    VblendvpdVqqHqqWqqIbV256,

    // [66 0F 38 14 /r] BLENDVPS xmm1, xmm2/m128, <XMM0>
    BlendvpsVdqWdq,
    // [VEX.128.66.0F3A.W0 4A /r /is4] VBLENDVPS xmm1, xmm2, xmm3/m128, xmm4
    VblendvpsVdqHdqWdqIbV128,
    // [VEX.256.66.0F3A.W0 4A /r /is4] VBLENDVPS ymm1, ymm2, ymm3/m256, ymm4
    VblendvpsVqqHqqWqqIbV256,

    // [VEX.LZ.0F38.W0 F3 /3] BLSI r32, r/m32
    BlsiBdEd,
    // [VEX.LZ.0F37.W1 F3 /3] BLSI r64, r/m64
    BlsiBqEq,

    // [VEX.LZ.0F38.W0 F3 /2] BLSMSK r32, r/m32
    BlsmskBdEd,
    // [VEX.LZ.0F37.W1 F3 /2] BLSMSK r64, r/m64
    BlsmskBqEq,

    // [VEX.LZ.0F38.W0 F3 /1] BLSR r32, r/m32
    BlsrBdEd,
    // [VEX.LZ.0F37.W1 F3 /1] BLSR r64, r/m64
    BlsrBqEq,

    // [F3 0F 1A /r] BNDCL bnd, r/m32
    BndclBGdqEd,
    // [F3 0F 1A /r] BNDCL bnd, r/m64
    BndclBGdqEq,

    // [F2 0F 1A /r] BNDCU bnd, r/m32
    BndcuBGdqEd,
    // [F2 0F 1A /r] BNDCU bnd, r/m64
    BndcuBGdqEq,
    // [F2 0F 1B /r] BNDCN bnd, r/m32
    BndcnBGdqEd,
    // [F2 0F 1B /r] BNDCN bnd, r/m64
    BndcnBGdqEq,

    // [NP 0F 1A /r] BNDLDX bnd, mib
    BndldxBGdqM,

    // [F3 0F 1B /r] BNDMK bnd, m32
    BndmkBGdqMd,
    // [F3 0F 1B /r] BNDMK bnd, m64
    BndmkBGdqMq,

    // [66 0F 1A /r] BNDMOV bnd1, bnd2/m64
    BndmovBGdqBEq,
    // [66 0F 1A /r] BNDMOV bnd1, bnd2/m128
    BndmovBGdqBEdq,
    // [66 0F 1B /r] BNDMOV bnd1/m64, bnd2
    BndmovBEqBGdq,
    // [66 0F 1B /r] BNDMOV bnd1/m128, bnd2
    BndmovBEdqBGdq,

    // [NP 0F 1B /r] BNDSTX mib, bnd
    BndstxMBGdq,

    // [62 /r] BOUND r16, m16&16
    BoundGwMa,
    // [62 /r] BOUND r32, m32&32
    BoundGdMa,

    // [0F BC /r] BSF r16, r/m16
    BsfGwEw,
    // [0F BC /r] BSF r32, r/m32
    BsfGdEd,
    // [REX.W 0F BC /r] BSF r64, r/m64
    BsfGqEq,

    // [0F BD /r] BSR r16, r/m16
    BsrGwEw,
    // [0F BD /r] BSR r32, r/m32
    BsrGdEd,
    // [REX.W 0F BD /r] BSR r64, r/m64
    BsrGqEq,

    // [0F C8+rd] BSWAP r32
    BswapGd,
    // [REX.W 0F C8+rd] BSWAP r64
    BswapGq,

    // [0F A3 /r] BT r/m16, r16
    BtEwGw,
    // [0F A3 /r] BT r/m32, r32
    BtEdGd,
    // [REX.W 0F A3 /r] BT r/m64, r64
    BtEqGq,
    // [0F BA /4 ib] BT r/m16, imm8
    BtEwIb,
    // [0F BA /4 ib] BT r/m32, imm8
    BtEdIb,
    // [REX.W 0F BA /4 ib] BT r/m64, imm8
    BtEqIb,

    // [0F BB /r] BTC r/m16, r16
    BtcEwGw,
    // [0F BB /r] BTC r/m32, r32
    BtcEdGd,
    // [REX.W 0F BB /r] BTC r/m64, r64
    BtcEqGq,
    // [0F BA /7 ib] BTC r/m16, imm8
    BtcEwIb,
    // [0F BA /7 ib] BTC r/m32, imm8
    BtcEdIb,
    // [REX.W 0F BA /7 ib] BTC r/m64, imm8
    BtcEqIb,

    // [0F B3 /r] BTR r/m16, r16
    BtrEwGw,
    // [0F B3 /r] BTR r/m32, r32
    BtrEdGd,
    // [REX.W 0F B3 /r] BTR r/m64, r64
    BtrEqGq,
    // [0F BA /6 ib] BTR r/m16, imm8
    BtrEwIb,
    // [0F BA /6 ib] BTR r/m32, imm8
    BtrEdIb,
    // [REX.W 0F BA /6 ib] BTR r/m64, imm8
    BtrEqIb,

    // [0F AB /r] BTS r/m16, r16
    BtsEwGw,
    // [0F AB /r] BTS r/m32, r32
    BtsEdGd,
    // [REX.W 0F AB /r] BTS r/m64, r64
    BtsEqGq,
    // [0F BA /5 ib] BTS r/m16, imm8
    BtsEwIb,
    // [0F BA /5 ib] BTS r/m32, imm8
    BtsEdIb,
    // [REX.W 0F BA /5 ib] BTS r/m64, imm8
    BtsEqIb,

    // [VEX.LZ.0F38.W0 F5 /r] BZHI r32a, r/m32, r32b
    BzhiGdDdEd,
    // [VEX.LZ.0F38.W1 F5 /r] BZHI r64a, r/m64, r64b
    BzhiGqBqEq,

    // [E8 cw] CALL rel16
    CallJw,
    // [E8 cw] CALL rel32
    CallJd,
    // [FF /2] CALL r/m16
    CallEw,
    // [FF /2] CALL r/m32
    CallEd,
    // [FF /2] CALL r/m64
    CallEq,
    // [9A cd] CALL ptr16:16
    CallApOp16,
    // [9A cp] CALL ptr16:32
    CallApOp32,
    // [FF /3] CALL m16:16
    CallEpOp16,
    // [FF /3] CALL m16:32
    CallEpOp32,
    // [REX.W FF /3] CALL m16:64
    CallEpOp64,

    // [98] CBW
    Cbw,
    // [98] CWDE
    Cwde,
    // [REX.W 98] CDQE
    Cdqe,

    // [NP 0F 01 CA] CLAC
    Clac,

    // [F8] CLC
    Clc,

    // [FC] CLD
    Cld,

    // [NP 0F 1C /0] CLDEMOTE m8
    CldemoteMb,

    // [NP 0F AE /7] CLFLUSH m8
    ClflushMb,

    // [NFx 66 0F AE /7] CLFLUSHOPT m8
    ClflushoptMb,

    // [FA] CLI
    Cli,

    // [F3 0F AE /6] CLRSSBSY m64
    ClrssbsyMq,

    // [0F 06] CLTS
    Clts,

    // [66 0F AE /6] CLWB m8
    ClwbMb,

    // [F5] CMC
    Cmc,

    // [0F 40 /r] CMOVO r16, r/m16
    CmovoGwEw,
    // [0F 40 /r] CMOVO r32, r/m32
    CmovoGdEd,
    // [REX.W 0F 40 /r] CMOVO r64, r/m64
    CmovoGqEq,
    // [0F 41 /r] CMOVNO r16, r/m16
    CmovnoGwEw,
    // [0F 41 /r] CMOVNO r32, r/m32
    CmovnoGdEd,
    // [REX.W 0F 41 /r] CMOVNO r64, r/m64
    CmovnoGqEq,
    // [0F 42 /r] CMOVB r16, r/m16
    // [0F 42 /r] CMOVC r16, r/m16
    // [0F 42 /r] CMOVNAE r16, r/m16
    CmovbGwEw,
    // [0F 42 /r] CMOVB r32, r/m32
    // [0F 42 /r] CMOVC r32, r/m32
    // [0F 42 /r] CMOVNAE r32, r/m32
    CmovbGdEd,
    // [REX.W 0F 42 /r] CMOVB r64, r/m64
    // [REX.W 0F 42 /r] CMOVC r64, r/m64
    // [REX.W 0F 42 /r] CMOVNAE r64, r/m64
    CmovbGqEq,
    // [0F 43 /r] CMOVAE r16, r/m16
    // [0F 43 /r] CMOVNB r16, r/m16
    CmovaeGwEw,
    // [0F 43 /r] CMOVAE r32, r/m32
    // [0F 43 /r] CMOVNB r32, r/m32
    CmovaeGdEd,
    // [REX.W 0F 43 /r] CMOVAE r64, r/m64
    // [REX.W 0F 43 /r] CMOVNB r64, r/m64
    CmovaeGqEq,
    // [0F 44 /r] CMOVE r16, r/m16
    // [0F 44 /r] CMOVZ r16, r/m16
    CmoveGwEw,
    // [0F 44 /r] CMOVE r32, r/m32
    // [0F 44 /r] CMOVZ r32, r/m32
    CmoveGdEd,
    // [REX.W 0F 44 /r] CMOVE r64, r/m64
    // [REX.W 0F 44 /r] CMOVZ r64, r/m64
    CmoveGqEq,
    // [0F 45 /r] CMOVNE r16, r/m16
    // [0F 45 /r] CMOVNZ r16, r/m16
    CmovneGwEw,
    // [0F 45 /r] CMOVNE r32, r/m32
    // [0F 45 /r] CMOVNZ r32, r/m32
    CmovneGdEd,
    // [REX.W 0F 45 /r] CMOVNE r64, r/m64
    // [REX.W 0F 45 /r] CMOVNZ r64, r/m64
    CmovneGqEq,
    // [0F 46 /r] CMOVBE r16, r/m16
    // [0F 46 /r] CMOVNA r16, r/m16
    CmovbeGwEw,
    // [0F 46 /r] CMOVBE r32, r/m32
    // [0F 46 /r] CMOVNA r32, r/m32
    CmovbeGdEd,
    // [REX.W 0F 46 /r] CMOVBE r64, r/m64
    // [REX.W 0F 46 /r] CMOVNA r64, r/m64
    CmovbeGqEq,
    // [0F 47 /r] CMOVA r16, r/m16
    // [0F 47 /r] CMOVNBE r16, r/m16
    // [0F 47 /r] CMOVNC r16, r/m16
    CmovaGwEw,
    // [0F 47 /r] CMOVA r32, r/m32
    // [0F 47 /r] CMOVNBE r32, r/m32
    // [0F 47 /r] CMOVNC r32, r/m32
    CmovaGdEd,
    // [REX.W 0F 47 /r] CMOVA r64, r/m64
    // [REX.W 0F 47 /r] CMOVNBE r64, r/m64
    // [REX.W 0F 47 /r] CMOVNC r64, r/m64
    CmovaGqEq,
    // [0F 48 /r] CMOVS r16, r/m16
    CmovsGwEw,
    // [0F 48 /r] CMOVS r32, r/m32
    CmovsGdEd,
    // [REX.W 0F 48 /r] CMOVS r64, r/m64
    CmovsGqEq,
    // [0F 49 /r] CMOVNS r16, r/m16
    CmovnsGwEw,
    // [0F 49 /r] CMOVNS r32, r/m32
    CmovnsGdEd,
    // [REX.W 0F 49 /r] CMOVNS r64, r/m64
    CmovnsGqEq,
    // [0F 4A /r] CMOVP r16, r/m16
    // [0F 4A /r] CMOVPE r16, r/m16
    CmovpGwEw,
    // [0F 4A /r] CMOVP r32, r/m32
    // [0F 4A /r] CMOVPE r32, r/m32
    CmovpGdEd,
    // [REX.W 0F 4A /r] CMOVP r64, r/m64
    // [REX.W 0F 4A /r] CMOVPE r64, r/m64
    CmovpGqEq,
    // [0F 4B /r] CMOVNP r16, r/m16
    // [0F 4B /r] CMOVPO r16, r/m16
    CmovnpGwEw,
    // [0F 4B /r] CMOVNP r32, r/m32
    // [0F 4B /r] CMOVPO r32, r/m32
    CmovnpGdEd,
    // [REX.W 0F 4B /r] CMOVNP r64, r/m64
    // [REX.W 0F 4B /r] CMOVPO r64, r/m64
    CmovnpGqEq,
    // [0F 4C /r] CMOVL r16, r/m16
    // [0F 4C /r] CMOVNGE r16, r/m16
    CmovlGwEw,
    // [0F 4C /r] CMOVL r32, r/m32
    // [0F 4C /r] CMOVNGE r32, r/m32
    CmovlGdEd,
    // [REX.W 0F 4C /r] CMOVL r64, r/m64
    // [REX.W 0F 4C /r] CMOVNGE r64, r/m64
    CmovlGqEq,
    // [0F 4D /r] CMOVGE r16, r/m16
    // [0F 4D /r] CMOVNL r16, r/m16
    CmovgeGwEw,
    // [0F 4D /r] CMOVGE r32, r/m32
    // [0F 4D /r] CMOVNL r32, r/m32
    CmovgeGdEd,
    // [REX.W 0F 4D /r] CMOVGE r64, r/m64
    // [REX.W 0F 4D /r] CMOVNL r64, r/m64
    CmovgeGqEq,
    // [0F 4E /r] CMOVLE r16, r/m16
    // [0F 4E /r] CMOVNG r16, r/m16
    CmovleGwEw,
    // [0F 4E /r] CMOVLE r32, r/m32
    // [0F 4E /r] CMOVNG r32, r/m32
    CmovleGdEd,
    // [REX.W 0F 4E /r] CMOVLE r64, r/m64
    // [REX.W 0F 4E /r] CMOVNG r64, r/m64
    CmovleGqEq,
    // [0F 4F /r] CMOVG r16, r/m16
    // [0F 4F /r] CMOVNLE r16, r/m16
    CmovgGwEw,
    // [0F 4F /r] CMOVG r32, r/m32
    // [0F 4F /r] CMOVNLE r32, r/m32
    CmovgGdEd,
    // [REX.W 0F 4F /r] CMOVG r64, r/m64
    // [REX.W 0F 4F /r] CMOVNLE r64, r/m64
    CmovgGqEq,

    // [3C ib] CMP AL, imm8
    CmpALIb,
    // [3D iw] CMP AX, imm16
    CmpAXIw,
    // [3D id] CMP EAX, imm32
    CmpEAXId,
    // [REX.W 3D id] CMP RAX, imm32
    CmpRAXId,
    // [80 /7 ib] CMP r/m8, imm8
    // [REX 80 /7 ib] CMP r/m8, imm8
    CmpEbIb,
    // [81 /7 ib] CMP r/m16, imm16
    CmpEwIw,
    // [81 /7 ib] CMP r/m32, imm32
    CmpEdId,
    // [REX.W 81 /7 id] CMP r/m64, imm32
    CmpEqId,
    // [83 /7 ib] CMP r/m16, imm8
    CmpEwIb,
    // [83 /7 ib] CMP r/m32, imm8
    CmpEdIb,
    // [REX.W 83 /7 ib] CMP r/m64, imm8
    CmpEqIb,
    // [38 /r] CMP r/m8, r8
    // [REX 38 /r] CMP r/m8, r8
    CmpEbGb,
    // [39 /r] CMP r/m16, r16
    CmpEwGw,
    // [39 /r] CMP r/m32, r32
    CmpEdGd,
    // [REX.W 39 /r] CMP r/m64, r64
    CmpEqGq,
    // [3A /r] CMP r8, r/m8
    // [REX 3A /r] CMP r8, r/m8
    CmpGbEb,
    // [3B /r] CMP r16, r/m16
    CmpGwEw,
    // [3B /r] CMP r32, r/m32
    CmpGdEd,
    // [REX.W 38 /r] CMP r64, r/m64
    CmpGqEq,

    // [66 0F C2 /r ib] CMPPD xmm1, xmm2/m128, imm8
    CmppdVdqWdqIb,
    // [VEX.128.66.0F.WIG C2 /r ib] VCMPPD xmm1, xmm2, xmm3/m128, imm8
    VcmppdVdqHdqWdqIbV128,
    // [VEX.256.66.0F.WIG C2 /r ib] VCMPPD ymm1, ymm2, ymm3/m256, imm8
    VcmppdVqqHqqWqqIbV256,
    // [EVEX.128.66.0F.W1 C2 /r ib] VCMPPD k1 {k2}, xmm2, xmm3/m128/m64bcst, imm8
    VcmppdVdqHdqWdqIbE128,
    // [EVEX.256.66.0F.W1 C2 /r ib] VCMPPD k1 {k2}, ymm2, ymm3/m256/m64bcst, imm8
    VcmppdVqqHqqWqqIbE256,
    // [EVEX.512.66.0F.W1 C2 /r ib] VCMPPD k1 {k2}, zmm2, zmm3/m512/m64bcst{sae}, imm8
    VcmppdVdqqHdqqWdqqIbE512,

    // [NP 0F C2 /r ib] CMPPS xmm1, xmm2/m128, imm8
    CmppsVdqWdqIb,
    // [VEX.128.0F.WIG C2 /r ib] VCMPPS xmm1, xmm2, xmm3/m128, imm8
    VcmppsVdqHdqWdqIbV128,
    // [VEX.256.0F.WIG C2 /r ib] VCMPPS ymm1, ymm2, ymm3/m256, imm8
    VcmppsVqqHqqWqqIbV256,
    // [EVEX.128.0F.W0 C2 /r ib] VCMPPS k1 {k2}, xmm2, xmm3/m128/m32bcst, imm8
    VcmppsVdqHdqWdqIbE128,
    // [EVEX.256.0F.W0 C2 /r ib] VCMPPS k1 {k2}, ymm2, ymm3/m256/m32bcst, imm8
    VcmppsVqqHqqWqqIbE256,
    // [EVEX.512.0F.W0 C2 /r ib] VCMPPS k1 {k2}, zmm2, zmm3/m512/m32bcst{sae}, imm8
    VcmppsVdqqHdqqWdqqIbE512,

    // [A6] CMPS m8, m8
    // [A6] CMPSB m8, m8
    CmpsXbYb,
    // [A7] CMPS m16, m16
    // [A7] CMPSW m16, m16
    CmpsXwYw,
    // [A7] CMPS m32, m32
    // [A7] CMPSD m32, m32
    CmpsXdYd,
    // [REX.W A7] CMPS m64, m64
    // [REX.W A7] CMPSQ m64, m64
    CmpsXqYq,

    // [F2 0F C2 /r ib] CMPSD xmm1, xmm2/m64, imm8
    CmpsdVdqWqIb,
    // [VEX.LIG.F2.0F.WIG C2 /r ib] VCMPSD xmm1, xmm2, xmm3/m64, imm8
    VcmpsdVdqHdqWqIbV,
    // [EVEX.LIG.F2.0F.W1 C2 /r ib] VCMPSD k1 {k2}, xmm2, xmm3/m64{sae}, imm8
    VcmpsdKGqHdqWqIbE,

    // [F3 0F C2 /r ib] CMPSS xmm1, xmm2/m32, imm8
    CmpssVdqWdIb,
    // [VEX.LIG.F3.0F.WIG C2 /r ib] VCMPSS xmm1, xmm2, xmm3/m32, imm8
    VcmpssVdqHdqWdIbV,
    // [EVEX.LIG.F3.0F.W0 C2 /r ib] VCMPSS k1 {k2}, xmm2, xmm3/m32{sae}, imm8
    VcmpssKGqHdqWdIbE,

    // [0F B0 /r] CMPXCHG r/m8, r8
    // [REX 0F B0 /r] CMPXCHG r/m8, r8
    CmpxchgEbGb,
    // [0F B1 /r] CMPXCHG r/m16, r16
    CmpxchgEwGw,
    // [0F B1 /r] CMPXCHG r/m32, r32
    CmpxchgEdGd,
    // [REX.W 0F B1 /r] CMPXCHG r/m64, r64
    CmpxchgEqGq,

    // [0F C7 /1] CMPXCHG8B m64
    Cmpxchg8bMq,
    // [REX.W 0F C7 /1] CMPXCHG m128
    Cmpxchg16bMdq,

    // [66 0F 2F /r] COMISD xmm1, xmm2/m64
    ComisdVdqWq,
    // [VEX.LIG.66.0F.WIG 2F /r] VCOMISD xmm1, xmm2/m64
    VcomisdVdqWqV,
    // [EVEX.LIG.66.0F.W1 2F /r] VCOMISD xmm1, xmm2/m64{sae}
    VcomisdVdqWqE,

    // [NP 0F 2F /r] COMISS xmm1, xmm2/m32
    ComissVdqWd,
    // [VEX.LIG.0F.WIG 2F /r] VCOMISS xmm1, xmm2/m32
    VcomissVdqWdV,
    // [EVEX.LIG.0F.W0 2F /r] VCOMISS xmm1, xmm2/m32{sae}
    VcomissVdqWdE,

    // [0F A2] CPUID
    Cpuid,

    // [F2 0F 38 F0 /r] CRC32 r32, r/m8
    // [F2 REX 0F 38 F0 /r] CRC32 r32, r/m8
    Crc32GdEb,
    // [F2 0F 38 F1 /r] CRC32 r32, r/m16
    Crc32GdEw,
    // [F2 0F 38 F1 /r] CRC32 r32, r/m32
    Crc32GdEd,
    // [F2 REX.W 0F 38 F0 /r] CRC32 r64, r/m8
    Crc32GqEb,
    // [F2 REX.W 0F 38 F1 /r] CRC32 r64, r/m64
    Crc32GqEq,

    // [F3 0F E6 /r] CVTDQ2PD xmm1, xmm2/m64
    Cvtdq2pdVdqWq,
    // [VEX.128.F3.0F.WIG E6 /r] VCVTDQ2PD xmm1, xmm2/m64
    Vcvtdq2pdVdqWqV128,
    // [VEX.256.F3.0F.WIG E6 /r] VCVTDQ2PD ymm1, xmm2/m128
    Vcvtdq2pdVqqWdqV256,
    // [EVEX.128.F3.0F.W0 E6 /r] VCVTDQ2PD xmm1 {k1}{z}, xmm2/m64/m32bcst
    Vcvtdq2pdVdqWqE128,
    // [EVEX.256.F3.0F.W0 E6 /r] VCVTDQ2PD ymm1 {k1}{z}, xmm2/m128/m32bcst
    Vcvtdq2pdVqqWdqE256,
    // [EVEX.512.F3.0F.W0 E6 /r] VCVTDQ2PD zmm1 {k1}{z}, ymm2/m256/m32bcst
    Vcvtdq2pdVdqqWqqE512,

    // [NP 0F 5B /r] CVTDQ2PD xmm1, xmm2/m128
    Cvtdq2psVdqWdq,
    // [VEX.128.0F.WIG 5B /r] VCVTDQ2PD xmm1, xmm2/m128
    Vcvtdq2psVdqWdqV128,
    // [VEX.256.0F.WIG 5B /r] VCVTDQ2PD ymm1, ymm2/m256
    Vcvtdq2psVqqWqqV256,
    // [EVEX.128.0F.W0 5B /r] VCVTDQ2PD xmm1 {k1}{z}, xmm2/m128/m32bcst
    Vcvtdq2psVdqWdqE128,
    // [EVEX.256.0F.W0 5B /r] VCVTDQ2PD ymm1 {k1}{z}, ymm2/m256/m32bcst
    Vcvtdq2psVqqWqqE256,
    // [EVEX.512.0F.W0 5B /r] VCVTDQ2PD zmm1 {k1}{z}, zmm2/m512/m32bcst{er}
    Vcvtdq2psVdqqWdqqE512,

    // [EVEX.128.F2.0F38.W0 72 /r] VCVTNE2PS2BF16 xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vcvtne2ps2bf16VdqHdqWdqE128,
    // [EVEX.256.F2.0F38.W0 72 /r] VCVTNE2PS2BF16 ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vcvtne2ps2bf16VqqHqqWqqE256,
    // [EVEX.512.F2.0F38.W0 72 /r] VCVTNE2PS2BF16 zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    Vcvtne2ps2bf16VdqqHdqqWdqqE512,

    // [EVEX.128.F3.0F38.W0 72 /r] VCVTNEPS2BF16 xmm1 {k1}{z}, xmm2/m128/m32bcst
    Vcvtneps2bf16VdqWdqE128,
    // [EVEX.256.F3.0F38.W0 72 /r] VCVTNEPS2BF16 ymm1 {k1}{z}, ymm2/m256/m32bcst
    Vcvtneps2bf16VqqWqqE256,
    // [EVEX.512.F3.0F38.W0 72 /r] VCVTNEPS2BF16 zmm1 {k1}{z}, zmm2/m512/m32bcst
    Vcvtneps2bf16VdqqWdqqE512,

    // [F2 0F E6 /r] CVTPD2DQ xmm1, xmm2/m128
    Cvtpd2dqVdqWdq,
    // [VEX.128.F2.0F.WIG E6 /r] VCVTPD2DQ xmm1, xmm2/m128
    Vcvtpd2dqVdqWdqV128,
    // [VEX.256.F2.0F.WIG E6 /r] VCVTPD2DQ xmm1, ymm2/m256
    Vcvtpd2dqVdqWqqV256,
    // [EVEX.128.F2.0F.W1 E6 /r] VCVTPD2DQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    Vcvtpd2dqVdqWdqE128,
    // [EVEX.256.F2.0F.W1 E6 /r] VCVTPD2DQ xmm1 {k1}{z}, ymm2/m256/m64bcst
    Vcvtpd2dqVdqWqqE256,
    // [EVEX.512.F2.0F.W1 E6 /r] VCVTPD2DQ ymm1 {k1}{z}, zmm2/m512/m64bcst{er}
    Vcvtpd2dqVqqWdqqE512,

    // [66 0F 2D /r] CVTPD2PI mm, xmm/m128
    Cvtpd2piPqWdq,

    // [66 0F 5A /r] CVTPD2PS xmm1, xmm2/m128
    Cvtpd2psVdqWdq,
    // [VEX.128.66.0F.WIG 5A /r] VCVTPD2PS xmm1, xmm2/m128
    Vcvtpd2psVdqWdqV128,
    // [VEX.256.66.0F.WIG 5A /r] VCVTPD2PS xmm1, ymm2/m256
    Vcvtpd2psVdqWqqV256,
    // [EVEX.128.66.0F.W1 5A /r] VCVTPD2PS xmm1 {k1}{z}, xmm2/m128/m64bcst
    Vcvtpd2psVdqWdqE128,
    // [EVEX.256.66.0F.W1 5A /r] VCVTPD2PS xmm1 {k1}{z}, ymm2/m256/m64bcst
    Vcvtpd2psVdqWqqE256,
    // [EVEX.512.66.0F.W1 5A /r] VCVTPD2PS ymm1 {k1}{z}, zmm2/m512/m64bcst{er}
    Vcvtpd2psVqqWdqqE512,

    // [66 0F 2A /r] CVTPI2PD xmm, mm/m64
    Cvtpi2pdVdqQq,

    // [NP 0F 2A /r] CVTPI2PS xmm, mm/m64
    Cvtpi2psVdqQq,

    // [66 0F 5B /r] CVTPS2DQ xmm1, xmm2/m128
    Cvtps2dqVdqWdq,
    // [VEX.128.66.0F.WIG 5B /r] VCVTPS2DQ xmm1, xmm2/m128
    Vcvtps2dqVdqWdqV128,
    // [VEX.256.66.0F.WIG 5B /r] VCVTPS2DQ ymm1, ymm2/m256
    Vcvtps2dqVqqWqqV256,
    // [EVEX.128.66.0F.W0 5B /r] VCVTPS2DQ xmm1 {k1}{z}, xmm2/m128/m32bcst
    Vcvtps2dqVdqWdqE128,
    // [EVEX.256.66.0F.W0 5B /r] VCVTPS2DQ ymm1 {k1}{z}, ymm2/m256/m32bcst
    Vcvtps2dqVqqWqqE256,
    // [EVEX.512.66.0F.W0 5B /r] VCVTPS2DQ zmm1 {k1}{z}, zmm2/m512/m32bcst{er}
    Vcvtps2dqVdqqWdqqE512,

    // [NP 0F 5A /r] CVTPS2PD xmm1, xmm2/m64
    Cvtps2pdVdqWq,
    // [VEX.128.0F.WIG 5A /r] VCVTPS2PD xmm1, xmm2/m64
    Vcvtps2pdVdqWqV128,
    // [VEX.256.0F.WIG 5A /r] VCVTPS2PD ymm1, xmm2/m128
    Vcvtps2pdVqqWdqV256,
    // [EVEX.128.0F.W0 5A /r] VCVTPS2PD xmm1 {k1}{z}, xmm2/m64/m32bcst
    Vcvtps2pdVdqWqE128,
    // [EVEX.256.0F.W0 5A /r] VCVTPS2PD ymm1 {k1}{z}, xmm2/m128/m32bcst
    Vcvtps2pdVqqWdqE256,
    // [EVEX.512.0F.W0 5A /r] VCVTPS2PD zmm1 {k1}{z}, ymm2/m256/m32bcst{er}
    Vcvtps2pdVdqqWqqE512,

    // [NP 0F 2D /r] CVTPS2PI mm, xmm/m64
    Cvtps2piPqWq,

    // [F2 0F 2D /r] CVTSD2SI r32, xmm1/m64
    Cvtsd2siGdWq,
    // [F2 REX.W 0F 2D /r] CVTSD2SI r64, xmm1/m64
    Cvtsd2siGqWq,
    // [VEX.LIG.F2.0F.W0 2D /r] VCVTSD2SI r32, xmm1/m64
    Vcvtsd2siGdWqV,
    // [EVEX.LIG.F2.0F.W0 2D /r] VCVTSD2SI r32, xmm1/m64{er}
    Vcvtsd2siGdWqE,
    // [VEX.LIG.F2.0F.W1 2D /r] VCVTSD2SI r64, xmm1/m64
    Vcvtsd2siGqWqV,
    // [EVEX.LIG.F2.0F.W1 2D /r] VCVTSD2SI r64, xmm1/m64{er}
    Vcvtsd2siGqWqE,

    // [F2 0F 5A /r] CVTSD2SS xmm1, xmm2/m64
    Cvtsd2ssVdqWq,
    // [VEX.LIG.F2.0F.WIG 5A /r] VCVTSD2SS xmm1, xmm2, xmm3/m64
    Vcvtsd2ssVdqHdqWqV,
    // [EVEX.LIG.F2.0F.W1 5A /r] VCVTSD2SS xmm1 {k1}{z}, xmm2, xmm3/m64{er}
    Vcvtsd2ssVdqHdqWqE,

    // [F2 0F 2A /r] CVTSI2SD xmm1, r/m32
    Cvtsi2sdVdqEd,
    // [F2 REX.W 0F 2A /r] CVTSI2SD xmm1, r/m64
    Cvtsi2sdVdqEq,
    // [VEX.LIG.F2.0F.W0 2A /r] VCVTSI2SD xmm1, xmm2, r/m32
    Vcvtsi2sdVdqHdqEdV,
    // [EVEX.LIG.F2.0F.W0 2A /r] VCVTSI2SD xmm1, xmm2, r/m32
    Vcvtsi2sdVdqHdqEdE,
    // [VEX.LIG.F2.0F.W1 2A /r] VCVTSI2SD xmm1, xmm2, r/m64
    Vcvtsi2sdVdqHdqEqV,
    // [EVEX.LIG.F2.0F.W1 2A /r] VCVTSI2SD xmm1, xmm2, r/m64{er}
    Vcvtsi2sdVdqHdqEqE,

    // [F3 0F 2A /r] CVTSI2SS xmm1, r/m32
    Cvtsi2ssVdqEd,
    // [F2 REX.W 0F 2A /r] CVTSI2SS xmm1, r/m64
    Cvtsi2ssVdqEq,
    // [VEX.LIG.F3.0F.W0 2A /r] VCVTSI2SS xmm1, xmm2, r/m32
    Vcvtsi2ssVdqHdqEdV,
    // [EVEX.LIG.F3.0F.W0 2A /r] VCVTSI2SS xmm1, xmm2, r/m32
    Vcvtsi2ssVdqHdqEdE,
    // [VEX.LIG.F3.0F.W1 2A /r] VCVTSI2SS xmm1, xmm2, r/m64
    Vcvtsi2ssVdqHdqEqV,
    // [EVEX.LIG.F3.0F.W1 2A /r] VCVTSI2SS xmm1, xmm2, r/m64{er}
    Vcvtsi2ssVdqHdqEqE,

    // [F3 0F 5A /r] CVTSS2SD xmm1, xmm2/m32
    Cvtss2sdVdqWd,
    // [VEX.LIG.F3.0F.WIG 5A /r] VCVTSS2SD xmm1, xmm2, xmm3/m32
    Vcvtss2sdVdqHdqWdV,
    // [EVEX.LIG.F3.0F.W0 5A /r] VCVTSS2SD xmm1 {k1}{z}, xmm2, xmm3/m32{sae}
    Vcvtss2sdVdqHdqWdE,

    // [F3 0F 2D /r] CVTSS2SI r32, xmm1/m32
    Cvtss2siGdWd,
    // [F3 REX.W 0F 2D /r] CVTSS2SI r64, xmm1/m32
    Cvtss2siGqWd,
    // [VEX.LIG.F3.0F.W0 2D /r] VCVTSS2SI r32, xmm1/m32
    Vcvtss2siGdWdV,
    // [EVEX.LIG.F3.0F.W0 2D /r] VCVTSS2SI r32, xmm1/m32{er}
    Vcvtss2siGdWdE,
    // [VEX.LIG.F3.0F.W1 2D /r] VCVTSS2SI r64, xmm1/m32
    Vcvtss2siGqWdV,
    // [EVEX.LIG.F3.0F.W1 2D /r] VCVTSS2SI r64, xmm1/m32{er}
    Vcvtss2siGqWdE,

    // [66 0F E6 /r] CVTTPD2DQ xmm1, xmm2/m128
    Cvttpd2dqVdqWdq,
    // [VEX.128.66.0F.WIG E6 /r] VCVTTPD2DQ xmm1, xmm2/m128
    Vcvttpd2dqVdqWdqV128,
    // [VEX.256.66.0F.WIG E6 /r] VCVTTPD2DQ xmm1, ymm2/m256
    Vcvttpd2dqVdqWqqV256,
    // [EVEX.128.66.0F.W1 E6 /r] VCVTTPD2DQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    Vcvttpd2dqVdqWdqE128,
    // [EVEX.256.66.0F.W1 E6 /r] VCVTTPD2DQ xmm1 {k1}{z}, ymm2/m256/m64bcst
    Vcvttpd2dqVdqWqqE256,
    // [EVEX.512.66.0F.W1 E6 /r] VCVTTPD2DQ ymm1 {k1}{z}, zmm2/m512/m64bcst{sae}
    Vcvttpd2dqVqqWdqqE512,

    // [66 0F 2C /r] CVTTPD2PI mm, xmm/m128
    Cvttpd2piPqWdq,

    // [F3 0F 5B /r] CVTTPS2DQ xmm1, xmm2/m128
    Cvttps2dqVdqWdq,
    // [VEX.128.F3.0F.WIG 5B /r] VCVTTPS2DQ xmm1, xmm2/m128
    Vcvttps2dqVdqWdqV128,
    // [VEX.256.F3.0F.WIG 5B /r] VCVTTPS2DQ ymm1, ymm2/m256
    Vcvttps2dqVqqWqqV256,
    // [EVEX.128.F3.0F.W0 5B /r] VCVTTPS2DQ xmm1 {k1}{z}, xmm2/m128/m32bcst
    Vcvttps2dqVdqWdqE128,
    // [EVEX.256.F3.0F.W0 5B /r] VCVTTPS2DQ ymm1 {k1}{z}, ymm2/m256/m32bcst
    Vcvttps2dqVqqWqqE256,
    // [EVEX.512.F3.0F.W0 5B /r] VCVTTPS2DQ zmm1 {k1}{z}, zmm2/m512/m32bcst
    Vcvttps2dqVdqqWdqqE512,

    // [NP 0F 2C /r] CVTTPS2PI mm, xmm/m64
    Cvttps2piPqWq,

    // [F2 0F 2C /r] CVTTSD2SI r32, xmm1/m64
    Cvttsd2siGdWq,
    // [F2 REX.W 0F 2C /r] CVTTSD2SI r64, xmm1/m64
    Cvttsd2siGqWq,
    // [VEX.LIG.F2.0F.W0 2C /r] VCVTTSD2SI r32, xmm1/m64
    Vcvttsd2siGdWqV,
    // [EVEX.LIG.F2.0F.W0 2C /r] VCVTTSD2SI r32, xmm1/m64{sae}
    Vcvttsd2siGdWqE,
    // [VEX.LIG.F2.0F.W1 2C /r] VCVTTSD2SI r64, xmm1/m64
    Vcvttsd2siGqWqV,
    // [EVEX.LIG.F2.0F.W1 2C /r] VCVTTSD2SI r64, xmm1/m64{sae}
    Vcvttsd2siGqWqE,

    // [F3 0F 2C /r] CVTTSS2SI r32, xmm1/m64
    Cvttss2siGdWq,
    // [F3 REX.W 0F 2C /r] CVTTSS2SI r64, xmm1/m64
    Cvttss2siGqWq,
    // [VEX.LIG.F3.0F.W0 2C /r] VCVTTSS2SI r32, xmm1/m64
    Vcvttss2siGdWqV,
    // [EVEX.LIG.F3.0F.W0 2C /r] VCVTTSS2SI r32, xmm1/m64{sae}
    Vcvttss2siGdWqE,
    // [VEX.LIG.F3.0F.W1 2C /r] VCVTTSS2SI r64, xmm1/m64
    Vcvttss2siGqWqV,
    // [EVEX.LIG.F3.0F.W1 2C /r] VCVTTSS2SI r64, xmm1/m64{sae}
    Vcvttss2siGqWqE,

    // [99] CWD
    Cwd,
    // [99] CDQ
    Cdq,
    // [REX.W 99]
    Cqo,

    // [27] DAA
    Daa,

    // [2F] DAS
    Das,

    // [FE /1] DEC r/m8
    // [REX FE /1] DEC r/m8
    DecEb,
    // [FF /1] DEC r/m16
    DecEw,
    // [FF /1] DEC r/m32
    DecEd,
    // [REX.W FF /1] DEC r/m64
    DecEq,
    // [48+rw] DEC r16
    DecGw,
    // [48+rd] DEC r32
    DecGd,

    // [F6 /6] DIV r/m8
    // [REX F6 /6] DIV r/m8
    DivEb,
    // [F7 /6] DIV r/m16
    DivEw,
    // [F7 /6] DIV r/m32
    DivEd,
    // [REX.W F7 /6] DIV r/m64
    DivEq,

    // [66 0F 5E /r] DIVPD xmm1, xmm2/m128
    DivpdVdqWdq,
    // [VEX.128.66.0F.WIG 5E /r] VDIVPD xmm1, xmm2, xmm3/m128
    VdivpdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 5E /r] VDIVPD ymm1, ymm2, ymm3/m256
    VdivpdVqqHqqWqqV256,
    // [EVEX.128.66.0F.W1 5E /r] VDIVPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VdivpdVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 5E /r] VDIVPD ymm1 {k1}{z}, ymm2, ymm3/m128/m64bcst
    VdivpdVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 5E /r] VDIVPD zmm1 {k1}{z}, zmm2, zmm3/m128/m64bcst{er}
    VdivpdVdqqHdqqWdqqE512,

    // [NP 0F 5E /r] DIVPD xmm1, xmm2/m128
    DivpsVdqWdq,
    // [VEX.128.0F.WIG 5E /r] VDIVPS xmm1, xmm2, xmm3/m128
    VdivpsVdqHdqWdqV128,
    // [VEX.256.0F.WIG 5E /r] VDIVPS ymm1, ymm2, ymm3/m256
    VdivpsVqqHqqWqqV256,
    // [EVEX.128.0F.W1 5E /r] VDIVPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VdivpsVdqHdqWdqE128,
    // [EVEX.256.0F.W1 5E /r] VDIVPS ymm1 {k1}{z}, ymm2, ymm3/m128/m32bcst
    VdivpsVqqHqqWqqE256,
    // [EVEX.512.0F.W1 5E /r] VDIVPS zmm1 {k1}{z}, zmm2, zmm3/m128/m32bcst{er}
    VdivpsVdqqHdqqWdqqE512,

    // [F2 0F 5E /r] DIVSD xmm1, xmm2/m64
    DivsdVdqWq,
    // [VEX.LIG.F2.0F.WIG 5E /r] VDIVSD xmm1, xmm2, xmm3/m64
    VdivsdVdqHdqWqV,
    // [EVEX.LIG.F2.0F.W1 5E /r] VDIVSD xmm1 {k1}{z}, xmm2, xmm3/m64{er}
    VdivsdVqqHqqWqE,

    // [F3 0F 5E /r] DIVSS xmm1, xmm2/m64
    DivssVdqWq,
    // [VEX.LIG.F3.0F.WIG 5E /r] VDIVSS xmm1, xmm2, xmm3/m32
    VdivssVdqHdqWdV,
    // [EVEX.LIG.F3.0F.W1 5E /r] VDIVSS xmm1 {k1}{z}, xmm2, xmm3/m32{er}
    VdivssVdqHdqWdE,

    // [EVEX.128.F3.0F38.W0 52 /r] VDPBF16PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vdpbf16psVdqHdqWdqE128,
    // [EVEX.256.F3.0F38.W0 52 /r] VDPBF16PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vdpbf16psVqqHqqWqqE256,
    // [EVEX.512.F3.0F38.W0 52 /r] VDPBF16PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    Vdpbf16psVdqqHdqqWdqqE512,

    // [66 0F 3A 41 /r ib] DPPD xmm1, xmm2/m128, imm8
    DppdVdqWdqIb,
    // [VEX.128.66.0F3A.WIG 41 /r ib] VDPPD xmm1, xmm2, xmm3/m128, imm8
    VdppdVdqHdqWdqIbV128,

    // [66 0F 3A 40 /r ib] DPPS xmm1, xmm2/m128, imm8
    DppsVdqWdqIb,
    // [VEX.128.66.0F3A.WIG 40 /r ib] VDPPS xmm1, xmm2, xmm3/m128, imm8
    VdppsVdqHdqWdqIbV128,
    // [VEX.256.66.0F3A.WIG 40 /r ib] VDPPS ymm1, ymm2, ymm3/m256, imm8
    VdppsVqqHqqWqqIbV256,

    // [NP 0F 77] EMMS
    Emms,

    // [F3 0F 38 FA 11:rrr:bbb] ENCODEKEY128 r32a, r32b, <XMM0>, <XMM1-2>, <XMM4-6>
    Encodekey128GdEd,

    // [F3 0F 38 FB 11:rrr:bbb] ENCODEKEY256 r32a, r32b, <XMM0-1>, <XMM2-6>
    Encodekey256GdEd,

    // [F3 0F 1E FB] ENDBR32
    Endbr32,

    // [F3 0F 1E FA] ENDBR64
    Endbr64,

    // [C8 iw ib] ENTER imm16, imm8
    EnterIwIb,

    // [66 0F 3A 17 /r ib] EXTRACTPS r/m32, xmm1, imm8
    ExtractpsEdVdqIb,
    // [VEX.128.66.0F3A.WIG 17 /r ib] VEXTRACTPS r/m32, xmm1, imm8
    VextractpsEdVdqIbV128,
    // [EVEX.128.66.0F3A.WIG 17 /r ib] VEXTRACTPS r/m32, xmm1, imm8
    VextractpsEdVdqIbE128,

    // [D9 F0] F2XM1
    F2xm1,

    // [D9 E1] FABS
    Fabs,

    // [D8 /0] FADD m32fp
    FaddMd,
    // [DC /0] FADD m64fp
    FaddMq,
    // [D8 C0+i] FADD ST(0), ST(i)
    FaddST0STi,
    // [DC C0+i] FADD ST(i), ST(0)
    FaddSTiST0,
    // [DE C0+i] FADDP ST(i), ST(0)
    // [DE C1] FADDP <ST(1)>, <ST(0)>
    FaddpSTiST0,
    // [DA /0] FIADD m32int
    FiaddMd,
    // [DE /0] FIADD m16int
    FiaddMw,

    // [DF /4] FBLD m80bcd
    FbldMt,

    // [DF /6] FBSTP m80bcd
    FbstpMt,

    // [D9 E0] FCHS
    Fchs,

    // [9B DB E2] FCLEX
    Fclex,
    // [DB E2] FNCLEX
    Fnclex,

    // [DA C0+i] FCMOVB ST(0), ST(i)
    FcmovbST0STi,
    // [DA C8+i] FCMOVE ST(0), ST(i)
    FcmoveST0STi,
    // [DA D0+i] FCMOVBE ST(0), ST(i)
    FcmovbeST0STi,
    // [DA D8+i] FCMOVU ST(0), ST(i)
    FcmovuST0STi,
    // [DB C0+i] FCMOVNB ST(0), ST(i)
    FcmovnbST0STi,
    // [DB C8+i] FCMOVNE ST(0), ST(i)
    FcmovneST0STi,
    // [DB D0+i] FCMOVNBE ST(0), ST(i)
    FcmovnbeST0STi,
    // [DB D8+i] FCMOVNU ST(0), ST(i)
    FcmovnuST0STi,

    // [D8 /2] FCOM m32fp
    FcomMd,
    // [DC /2] FCOM m64fp
    FcomMq,
    // [D8 D0+i] FCOM ST(i)
    // [D8 D1] FCOM <ST(1)>
    FcomSTi,
    // [D8 /3] FCOMP m32fp
    FcompMd,
    // [DC /3] FCOMP m64fp
    FcompMq,
    // [D8 D8+i] FCOMP ST(i)
    // [D8 D9] FCOMP <ST(1)>
    FcompSTi,
    // [DE D9] FCOMPP
    Fcompp,

    // [DB F0+i] FCOMI ST(0), ST(i)
    FcomiST0STi,
    // [DF F0+i] FCOMIP ST(0), ST(i)
    FcomipST0STi,
    // [DB E8+i] FUCOMI ST(0), ST(i)
    FucomiST0STi,
    // [DF E8+i] FUCOMIP ST(0), ST(i)
    FucomipST0STi,

    // [D9 FF] FCOS
    Fcos,

    // [D9 F6] FDECSTP
    Fdecstp,

    // [D8 /6] FDIV m32fp
    FdivMd,
    // [DC /6] FDIV m64fp
    FdivMq,
    // [D8 F0+i] FDIV ST(0), ST(i)
    FdivST0STi,
    // [DC F8+i] FDIV ST(i), ST(0)
    FdivSTiST0,
    // [DE F8+i] FDIVP ST(i), ST(0)
    // [DE F9] FDIVP <ST(1)>, <ST(0)>
    FdivpSTiST0,
    // [DA /6] FIDIV m32int
    FidivMd,
    // [DE /6] FIDIV m16int
    FidivMw,

    // [D8 /7] FDIVR m32fp
    FdivrMd,
    // [DC /7] FDIVR m64fp
    FdivrMq,
    // [D8 F8+i] FDIVR ST(0), ST(i)
    FdivrST0STi,
    // [DC F0+i] FDIVR ST(i), ST(0)
    FdivrSTiST0,
    // [DE F0+i] FDIVPR ST(i), ST(0)
    // [DE F1] FDIVPR <ST(1)>, <ST(0)>
    FdivprSTiST0,
    // [DA /7] FIDIVR m32int
    FidivrMd,
    // [DE /7] FIDIVR m16int
    FidivrMw,

    // [DD C0+i] FFREE ST(i)
    FfreeSTi,

    // [DE /2] FICOM m16int
    FicomMw,
    // [DA /2] FICOM m32int
    FicomMd,
    // [DE /3] FICOMP m16int
    FicompMw,
    // [DA /3] FICOMP m32int
    FicompMd,

    // [DF /0] FILD m16int
    FildMw,
    // [DB /0] FILD m32int
    FildMd,
    // [DF /5] FILD m64int
    FildMq,

    // [D9 F7] FINCSTP
    Fincstp,

    // [9B DB E3] FINIT
    Finit,
    // [DB E3] FNINIT
    Fninit,

    // [DF /2] FIST m16int
    FistMw,
    // [DB /2] FIST m32int
    FistMd,
    // [DF /3] FISTP m16int
    FistpMw,
    // [DB /3] FISTP m32int
    FistpMd,
    // [DF /7] FISTP m64int
    FistpMq,

    // [DF /1] FISTTP m16int
    FisttpMw,
    // [DB /1] FISTTP m32int
    FisttpMd,
    // [DD /1] FISTTP m64int
    FisttpMq,

    // [D9 /0] FLD m32fp
    FldMd,
    // [DD /0] FLD m64fp
    FldMq,
    // [DB /5] FLD m80fp
    FldMt,
    // [D9 C0+i] FLD ST(i)
    FldSTi,

    // [D9 E8] FLD1
    Fld1,
    // [D9 E9] FLDL2T
    Fldl2t,
    // [D9 EA] FLDL2E
    Fldl2e,
    // [D9 EB] FLDPI
    Fldpi,
    // [D9 EC] FLDLG2
    Fldlg2,
    // [D9 ED] FLDLN2
    Fldln2,
    // [D9 EE] FLDZ
    Fldz,

    // [D9 /5] FLDCW m2byte
    FldcwMw,

    // [D9 /4] FLDENV m14/24byte
    FldenvM,

    // [D8 /1] FMUL m32fp
    FmulMd,
    // [DC /1] FMUL m64fp
    FmulMq,
    // [D8 C8+i] FMUL ST(0), ST(i)
    FmulST0STi,
    // [DC C8+i] FMUL ST(i), ST(0)
    FmulSTiST0,
    // [DE C8+i] FMULP ST(i), ST(0)
    // [DE C9] FMULP <ST(1)>, <ST(0)>
    FmulpSTiST0,
    // [DA /1] FIMUL m32int
    FimulMd,
    // [DE /1] FIMUL m16int
    FimulMw,

    // [D9 D0] FNOP
    Fnop,

    // [D9 F3] FPATAN
    Fpatan,

    // [D9 F8] FPREM
    Fprem,

    // [D9 F5] FPREM1
    Fprem1,

    // [D9 F2] FPTAN
    Fptan,

    // [D9 FC] FRNDINT
    Frndint,

    // [DD /4] FRSTOR m94/108byte
    FrstorM,

    // [9B DD /6] FSAVE m94/108byte
    FsaveM,
    // [DD /6] FNSAVE m94/108byte
    FnsaveM,

    // [D9 FD] FSCALE
    Fscale,

    // [D9 FE] FSIN
    Fsin,

    // [D9 FB] FSINCOS
    Fsincos,

    // [D9 FA] FSQRT
    Fsqrt,

    // [D9 /2] FST m32fp
    FstMd,
    // [DD /2] FST m64fp
    FstMq,
    // [DD D0+i] FST ST(i)
    FstSTi,
    // [D9 /3] FSTP m32fp
    FstpMd,
    // [DD /3] FSTP m64fp
    FstpMq,
    // [DB /7] FSTP m80fp
    FstpMt,
    // [DD D8+i] FSTP ST(i)
    FstpSTi,

    // [9B D9 /7] FSTCW m2byte
    FstcwMw,
    // [D9 /7] FNSTCW m2byte
    FnstcwMw,

    // [9B D9 /6] FSTENV m14/28byte
    FstenvM,
    // [D9 /6] FNSTENV m14/28byte
    FnstenvM,

    // [9B DD /7] FSTSW m2byte
    FstswMw,
    // [9B DF E0] FSTSW AX
    FstswAX,
    // [DD /7] FNSTSW m2byte
    FnstswMw,
    // [DF E0] FNSTSW AX
    FnstswAX,

    // [D8 /4] FSUB m32fp
    FsubMd,
    // [DC /4] FSUB m64fp
    FsubMq,
    // [D8 E0+i] FSUB ST(0), ST(i)
    FsubST0STi,
    // [DC E8+i] FSUB ST(i), ST(0)
    FsubSTiST0,
    // [DE E8+i] FSUBP ST(i), ST(0)
    // [DE E9] FSUBP <ST(1)>, <ST(0)>
    FsubpSTiST0,
    // [DA /4] FISUB m32int
    FisubMd,
    // [DE /4] FISUB m16int
    FisubMw,

    // [D8 /5] FSUBR m32fp
    FsubrMd,
    // [DC /5] FSUBR m64fp
    FsubrMq,
    // [D8 E8+i] FSUBR ST(0), ST(i)
    FsubrST0STi,
    // [DC E0+i] FSUBR ST(i), ST(0)
    FsubrSTiST0,
    // [DE E0+i] FSUBRP ST(i), ST(0)
    // [DE E1] FSUBRP <ST(1)>, <ST(0)>
    FsubrpSTiST0,
    // [DA /5] FISUBR m32int
    FisubrMd,
    // [DE /5] FISUBR m16int
    FisubrMw,

    // [D9 E4] FTST
    Ftst,

    // [DD E0+i] FUCOM ST(i)
    // [DD E1] FUCOM <ST(1)>
    FucomSTi,
    // [DD E8+i] FUCOMP ST(i)
    // [DD E9] FUCOMP <ST(1)>
    FucompSTi,
    // [DA E9] FUCOMPP
    Fucompp,

    // [D9 E5] FXAM
    Fxam,

    // [D9 C8+i] FXCH ST(i)
    // [D9 C9] FXCH <ST(1)>
    FxchSTi,

    // [NP 0F AE /1] FXRSTOR m512byte
    FxrstorM,
    // [NP REX.W 0F AE /1] FXRSTOR64 m512byte
    Fxrstor64M,

    // [NP 0F AE /0] FXSAVE m512byte
    FxsaveM,
    // [NP REX.W 0F AE /0] FXSAVE64 m512byte
    Fxsave64M,

    // [D9 F4] FXTRACT
    Fxtract,

    // [D9 F1] FYL2X
    Fyl2x,

    // [D9 F9] FYL2XP1
    Fyl2xp1,

    // [66 0F 3A CF /r ib] GF2P8AFFINEINVQB xmm1, xmm2/m128, imm8
    Gf2p8affineinvqbVdqWdqIb,
    // [VEX.128.66.0F3A.W1 CF /r ib] VGF2P8AFFINEINVQB xmm1, xmm2, xmm3/m128, imm8
    Vgf2p8affineinvqbVdqHdqWdqIbV128,
    // [VEX.256.66.0F3A.W1 CF /r ib] VGF2P8AFFINEINVQB ymm1, ymm2, ymm3/m256, imm8
    Vgf2p8affineinvqbVqqHqqWqqIbV256,
    // [EVEX.128.66.0F3A.W1 CF /r ib] VGF2P8AFFINEINVQB xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    Vgf2p8affineinvqbVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 CF /r ib] VGF2P8AFFINEINVQB ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    Vgf2p8affineinvqbVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 CF /r ib] VGF2P8AFFINEINVQB zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    Vgf2p8affineinvqbVdqqHdqqWdqqIbE512,

    // [66 0F 3A CE /r ib] GF2P8AFFINEQB xmm1, xmm2/m128, imm8
    Gf2p8affineqbVdqWdqIb,
    // [VEX.128.66.0F3A.W1 CE /r ib] VGF2P8AFFINEQB xmm1, xmm2/m128, imm8
    Vgf2p8affineqbVdqHdqWdqIbV128,
    // [VEX.256.66.0F3A.W1 CE /r ib] VGF2P8AFFINEQB ymm1, ymm2/m256, imm8
    Vgf2p8affineqbVqqHqqWqqIbV256,
    // [EVEX.128.66.0F3A.W1 CE /r ib] VGF2P8AFFINEQB xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    Vgf2p8affineqbVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 CE /r ib] VGF2P8AFFINEQB ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    Vgf2p8affineqbVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 CE /r ib] VGF2P8AFFINEQB zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    Vgf2p8affineqbVdqqHdqqWdqqIbE512,

    // [66 0F 38 CF /r ib] GF2P8MULB xmm1, xmm2/m128
    Gf2p8mulbVdqWdq,
    // [VEX.128.66.0F38.W0 CF /r ib] VGF2P8MULB xmm1, xmm2/m128
    Vgf2p8mulbVdqHdqWdqV128,
    // [VEX.256.66.0F38.W0 CF /r ib] VGF2P8MULB ymm1, ymm2/m256
    Vgf2p8mulbVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W0 CF /r ib] VGF2P8MULB xmm1 {k1}{z}, xmm2, xmm3/m128
    Vgf2p8mulbVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 CF /r ib] VGF2P8MULB ymm1 {k1}{z}, ymm2, ymm3/m256
    Vgf2p8mulbVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 CF /r ib] VGF2P8MULB zmm1 {k1}{z}, zmm2, zmm3/m512
    Vgf2p8mulbVdqqHdqqWdqqE512,

    // [66 0F 7C /r] HADDPD xmm1, xmm2/m128
    HaddpdVdqWdq,
    // [VEX.128.66.0F.WIG 7C /r] VHADDPD xmm1, xmm2, xmm3/m128
    VhaddpdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 7C /r] VHADDPD ymm1, ymm2, ymm3/m256
    VhaddpdVqqHqqWqqV256,

    // [F2 0F 7C /r] HADDPD xmm1, xmm2/m128
    HaddpsVdqWdq,
    // [VEX.128.F2.0F.WIG 7C /r] VHADDPD xmm1, xmm2, xmm3/m128
    VhaddpsVdqHdqWdqV128,
    // [VEX.256.F2.0F.WIG 7C /r] VHADDPD ymm1, ymm2, ymm3/m256
    VhaddpsVqqHqqWqqV256,

    // [F4] HLT
    Hlt,

    // [66 0F 7D /r] HSUBPD xmm1, xmm2/m128
    HsubpdVdqWdq,
    // [VEX.128.66.0F.WIG 7D /r] VHSUBPD xmm1, xmm2, xmm3/m128
    VhsubpdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 7D /r] VHSUBPD ymm1, ymm2, ymm3/m256
    VhsubpdVqqHqqWqqV256,

    // [F2 0F 7D /r] HSUBPS xmm1, xmm2/m128
    HsubpsVdqWdq,
    // [VEX.128.F2.0F.WIG 7D /r] VHSUBPS xmm1, xmm2, xmm3/m128
    VhsubpsVdqHdqWdqV128,
    // [VEX.256.F2.0F.WIG 7D /r] VHSUBPS ymm1, ymm2, ymm3/m256
    VhsubpsVqqHqqWqqV256,

    // [F6 /7] IDIV r/m8
    // [REX F6 /7] IDIV r/m8
    IdivEb,
    // [F7 /7] IDIV r/m16
    IdivEw,
    // [F7 /7] IDIV r/m32
    IdivEd,
    // [REX.W F7 /7] IDIV r/m64
    IdivEq,

    // [F6 /5] IMUL r/m8
    ImulEb,
    // [F7 /5] IMUL r/m16
    ImulEw,
    // [F7 /5] IMUL r/m32
    ImulEd,
    // [REX.W F7 /5] IMUL r/m64
    ImulEq,
    // [0F AF /r] IMUL r16, r/m16
    ImulGbEb,
    // [0F AF /r] IMUL r32, r/m32
    ImulGwEw,
    // [REX.W 0F AF /r] IMUL r64, r/m64
    ImulGqEq,
    // [6B /r ib] IMUL r16, r/m16, imm8
    ImulGwEwIb,
    // [6B /r ib] IMUL r32, r/m32, imm8
    ImulGdEdIb,
    // [REX.W 6B /r ib] IMUL r64, r/m64, imm8
    ImulGqEqIb,
    // [69 /r iw] IMUL r16, r/m16, imm16
    ImulGwEwIw,
    // [69 /r id] IMUL r32, r/m32, imm32
    ImulGdEdId,
    // [REX.W 69 /r id] IMUL r64, r/m64, imm32
    ImulGqEqId,

    // [E4 ib] IN AL, imm8
    InALIb,
    // [E5 ib] IN AX, imm8
    InAXIb,
    // [E5 ib] IN EAX, imm8
    InEAXIb,
    // [EC] IN AL, DX
    InALDX,
    // [ED] IN AX, DX
    InAXDX,
    // [ED] IN EAX, DX
    InEAXDX,

    // [FE /0] INC r/m8
    // [REX FE /0] INC r/m8
    IncEb,
    // [FF /0] INC r/m16
    IncEw,
    // [FF /0] INC r/m32
    IncEd,
    // [REX.W FF /0] INC r/m64
    IncEq,
    // [40+rw] INC r16
    IncGw,
    // [40+rd] INC r32
    IncGd,

    // [F3 0F AE /5] INCSSPD r32
    Incsspd,
    // [F3 REX.W 0F AE /5] INCSSPQ r64
    Incsspq,

    // [6C] INS m8, DX
    // [6C] INSB
    InsYbDX,
    // [6D] INS m16, DX
    // [6D] INSW
    InsYwDX,
    // [6D] INS m32, DX
    // [6D] INSD
    InsYdDX,

    // [66 0F 3A 21 /r ib] INSERTPS xmm1, xmm2/m32, imm8
    InsertpsVdqWdIb,
    // [VEX.128.66.0F3A.WIG 21 /r ib] VINSERTPS xmm1, xmm2, xmm3/m32, imm8
    VinsertpsVdqHdqWdIbV128,
    // [EVEX.128.66.0F3A.W0 21 /r ib] VINSERTPS xmm1, xmm2, xmm3/m32, imm8
    VinsertpsVdqHdqWdIbE128,

    // [CC] INT3,
    Int3,
    // [CD ib] INT imm8
    IntIb,
    // [CE] INTO
    Into,
    // [F1] INT1
    Int1,

    // [0F 08] INVD
    Invd,

    // [0F 01 /7] INVLPG mem
    InvlpgM,

    // [66 0F 38 82 /r] INVPCID r32, m128
    InvpcidGdMdq,
    // [66 0F 38 82 /r] INVPCID r64, m128
    InvpcidGqMdq,

    // [CF] IRET
    Iret,
    // [CF] IRETD
    Iretd,
    // [REX.W CF] IRETQ
    Iretq,

    // [70 cb] JO rel8
    JoJb,
    // [0F 80 cw] JO rel16
    JoJw,
    // [0F 80 cd] JO rel32
    JoJd,
    // [71 cb] JNO rel8
    JnoJb,
    // [0F 81 cw] JNO rel16
    JnoJw,
    // [0F 81 cd] JNO rel32
    JnoJd,
    // [72 cb] JB rel8
    // [72 cb] JC rel8
    // [72 cb] JNAE rel8
    JbJb,
    // [0F 82 cw] JB rel16
    // [0F 82 cw] JC rel16
    // [0F 82 cw] JNAE rel16
    JbJw,
    // [0F 82 cd] JB rel32
    // [0F 82 cd] JC rel32
    // [0F 82 cd] JNAE rel32
    JbJd,
    // [73 cb] JAE rel8
    // [73 cb] JNB rel8
    // [73 cb] JNC rel8
    JaeJb,
    // [0F 83 cw] JAE rel16
    // [0F 83 cw] JNB rel16
    // [0F 83 cw] JNC rel16
    JaeJw,
    // [0F 83 cd] JAE rel32
    // [0F 83 cd] JNB rel32
    // [0F 83 cd] JNC rel32
    JaeJd,
    // [74 cb] JE rel8
    // [74 cb] JZ rel8
    JeJb,
    // [0F 84 cw] JE rel16
    // [0F 84 cw] JZ rel16
    JeJw,
    // [0F 84 cd] JE rel32
    // [0F 84 cd] JZ rel32
    JeJd,
    // [75 cb] JNE rel8
    // [75 cb] JNZ rel8
    JneJb,
    // [0F 85 cw] JNE rel16
    // [0F 85 cw] JNZ rel16
    JneJw,
    // [0F 85 cd] JNE rel32
    // [0F 85 cd] JNZ rel32
    JneJd,
    // [76 cb] JBE rel8
    // [76 cb] JNA rel8
    JbeJb,
    // [0F 86 cw] JBE rel16
    // [0F 86 cw] JNA rel16
    JbeJw,
    // [0F 86 cd] JBE rel32
    // [0F 86 cd] JNA rel32
    JbeJd,
    // [77 cb] JA rel8
    // [77 cb] JNBE rel8
    JaJb,
    // [0F 87 cw] JA rel16
    // [0F 87 cw] JNBE rel16
    JaJw,
    // [0F 87 cd] JA rel32
    // [0F 87 cd] JNBE rel32
    JaJd,
    // [78 cb] JS rel8
    JsJb,
    // [0F 88 cw] JS rel16
    JsJw,
    // [0F 88 cd] JS rel32
    JsJd,
    // [79 cb] JNS rel8
    JnsJb,
    // [0F 89 cw] JNS rel16
    JnsJw,
    // [0F 89 cd] JNS rel32
    JnsJd,
    // [7A cb] JP rel8
    // [7A cb] JPE rel8
    JpJb,
    // [0F 8A cw] JP rel16
    // [0F 8A cw] JPE rel16
    JpJw,
    // [0F 8A cd] JP rel32
    // [0F 8A cd] JPE rel32
    JpJd,
    // [7B cb] JNP rel8
    // [7B cb] JPO rel8
    JnpJb,
    // [0F 8B cw] JNP rel16
    // [0F 8B cw] JPO rel16
    JnpJw,
    // [0F 8B cd] JNP rel32
    // [0F 8B cd] JPO rel32
    JnpJd,
    // [7C cb] JL rel8
    // [7C cb] JNGE rel8
    JlJb,
    // [0F 8C cw] JL rel16
    // [0F 8C cw] JNGE rel16
    JlJw,
    // [0F 8C cd] JL rel32
    // [0F 8C cd] JNGE rel32
    JlJd,
    // [7D cb] JGE rel8
    // [7D cb] JNL rel8
    JgeJb,
    // [0F 8D cw] JGE rel16
    // [0F 8D cw] JNL rel16
    JgeJw,
    // [0F 8D cd] JGE rel32
    // [0F 8D cd] JNL rel32
    JgeJd,
    // [7E cb] JLE rel8
    // [7E cb] JNG rel8
    JleJb,
    // [0F 8E cw] JLE rel16
    // [0F 8E cw] JNG rel16
    JleJw,
    // [0F 8E cd] JLE rel32
    // [0F 8E cd] JNG rel32
    JleJd,
    // [7F cb] JG rel8
    // [7F cb] JNLE rel8
    JgJb,
    // [0F 8F cw] JG rel16
    // [0F 8F cw] JNLE rel16
    JgJw,
    // [0F 8F cd] JG rel32
    // [0F 8F cd] JNLE rel32
    JgJd,
    // [E3 cb] JCXZ rel8
    // [E3 cb] JECXZ rel8
    // [E3 cb] JRCXZ rel8
    JcxzJb,

    // [EB cb] JMP rel8
    JmpJb,
    // [E9 cw] JMP rel16
    JmpJw,
    // [E9 cd] JMP rel32
    JmpJd,
    // [FF /4] JMP r/m16
    JmpEw,
    // [FF /4] JMP r/m32
    JmpEd,
    // [FF /4] JMP r/m64
    JmpEq,
    // [EA cd] JMP ptr16:16
    JmpApOp16,
    // [EA cp] JMP ptr16:32
    JmpApOp32,
    // [FF /5] JMP m16:16
    JmpEpOp16,
    // [FF /5] JMP m16:32
    JmpEpOp32,
    // [FF /5] JMP m16:64
    JmpEpOp64,

    // [VEX.L1.66.0F.W0 4A /r] KADDB k1, k2, k3
    KaddbKGbKHbKEb,
    // [VEX.L1.0F.W0 4A /r] KADDW k1, k2, k3
    KaddwKGwKHwKEw,
    // [VEX.L1.66.0F.W1 4A /r] KADDD k1, k2, k3
    KadddKGdKHdKEd,
    // [VEX.L1.0F.W1 4A /r] KADDQ k1, k2, k3
    KaddqKGqKHqKEq,

    // [VEX.L1.66.0F.W0 41 /r] KANDB k1, k2, k3
    KandbKGbKHbKEb,
    // [VEX.L1.0F.W0 41 /r] KANDW k1, k2, k3
    KandwKGwKHwKEw,
    // [VEX.L1.66.0F.W1 41 /r] KANDD k1, k2, k3
    KanddKGdKHdKEd,
    // [VEX.L1.0F.W1 41 /r] KANDQ k1, k2, k3
    KandqKGqKHqKEq,

    // [VEX.L1.66.0F.W0 42 /r] KANDNB k1, k2, k2
    KandnbKGbKHbKEb,
    // [VEX.L1.0F.W0 42 /r] KANDNW k1, k2, k3
    KandneKGwKHwKEw,
    // [VEX.L1.66.0F.W1 42 /r] KANDND k1, k2, k3
    KandndKGdKHdKEd,
    // [VEX.L1.0F.W1 42 /r] KANDNQ k1, k2, k3
    KandnqKGqKHqKEq,

    // [VEX.L0.66.0F.W0 90 /r] KMOVB k1, k2/m8
    KmovbKGbKEb,
    // [VEX.L0.0F.W0 90 /r] KMOVW k1, k2/m16
    KmovwKGwKEw,
    // [VEX.L0.66.0F.W1 90 /r] KMOVD k1, k2/m32
    KmovdKGdKEd,
    // [VEX.L0.0F.W1 90 /r] KMOVQ k1, k2/m64
    KmovqKGqKEq,
    // [VEX.L0.66.0F.W0 91 /r] KMOVB m8, k1
    KmovbKEbKGb,
    // [VEX.L0.0F.W0 91 /r] KMOVW m16, k1
    KmovwKEwKGw,
    // [VEX.L0.66.0F.W1 91 /r] KMOVD m32, k1
    KmovdKEdKGd,
    // [VEX.L0.0F.W1 91 /r] KMOVQ m64, k1
    KmovqKEqKGq,
    // [VEX.L0.66.0F.W0 92 /r] KMOVB k1, r32
    KmovbKEbGd,
    // [VEX.L0.0F.W0 92 /r] KMOVW k1, r32
    KmovwKEwGd,
    // [VEX.L0.66.0F.W1 92 /r] KMOVD k1, r32
    KmovdKEdGd,
    // [VEX.L0.0F.W1 92 /r] KMOVQ k1, r64
    KmovqKEqGq,
    // [VEX.L0.66.0F.W0 93 /r] KMOVB r32, k1
    KmovbGdKEb,
    // [VEX.L0.0F.W0 93 /r] KMOVW r32, k1
    KmovwGdKEw,
    // [VEX.L0.66.0F.W1 93 /r] KMOVD r32, k1
    KmovdGdKEd,
    // [VEX.L0.0F.W1 93 /r] KMOVQ r64, k1
    KmovqGqKEq,

    // [VEX.L1.66.0F.W0 44 /r] KNOTB k1, k2
    KnotbKGbKEb,
    // [VEX.L1.0F.W0 44 /r] KNOTW k1, k2
    KnotwKGwKEw,
    // [VEX.L1.66.0F.W1 44 /r] KNOTD k1, k2
    KnotdKGdKEd,
    // [VEX.L1.0F.W1 44 /r] KNOTQ k1, k2
    KnotqKGqKEq,

    // [VEX.L1.66.0F.W0 45 /r] KORB k1, k2, k2
    KorbKGbKHbKEb,
    // [VEX.L1.0F.W0 45 /r] KORW k1, k2, k3
    KorwKGwKHwKEw,
    // [VEX.L1.66.0F.W1 45 /r] KORD k1, k2, k3
    KordKGdKHdKEd,
    // [VEX.L1.0F.W1 45 /r] KORQ k1, k2, k3
    KorqKGqKHqKEq,

    // [VEX.L1.66.0F.W0 98 /r] KORTESTB k1, k2
    KortestbKGbKEb,
    // [VEX.L1.0F.W0 98 /r] KORTESTW k1, k2
    KortestwKGwKEw,
    // [VEX.L1.66.0F.W1 98 /r] KORTESTD k1, k2
    KortestdKGdKEd,
    // [VEX.L1.0F.W1 98 /r] KORTESTQ k1, k2
    KortestqKGqKEq,

    // [VEX.L0.66.0F3A.W0 32 /r] KSHIFTLB k1, k2, imm8
    KshiftlbKGbKEbIb,
    // [VEX.L0.66.0F3A.W1 32 /r] KSHIFTLW k1, k2, imm8
    KshiftlwKGwKEwIb,
    // [VEX.L0.66.0F3A.W0 33 /r] KSHIFTLD k1, k2, imm8
    KshiftldKGdKEdIb,
    // [VEX.L0.66.0F3A.W1 33 /r] KSHIFTLQ k1, k2, imm8
    KshiftlqKGqKEqIb,

    // [VEX.L0.66.0F3A.W0 30 /r] KSHIFTRB k1, k2, imm8
    KshiftrbKGbKEbIb,
    // [VEX.L0.66.0F3A.W1 30 /r] KSHIFTRW k1, k2, imm8
    KshiftrwKGwKEwIb,
    // [VEX.L0.66.0F3A.W0 31 /r] KSHIFTRD k1, k2, imm8
    KshiftrdKGdKEdIb,
    // [VEX.L0.66.0F3A.W1 31 /r] KSHIFTRQ k1, k2, imm8
    KshiftrqKGqKEqIb,

    // [VEX.L0.66.0F.W0 99 /r] KTESTB k1, k2
    KtestbKGbKEb,
    // [VEX.L0.0F.W0 99 /r] KTESTW k1, k2
    KtestwKGwKEw,
    // [VEX.L0.66.0F.W1 99 /r] KTESTD k1, k2
    KtestdKGdKEd,
    // [VEX.L0.0F.W1 99 /r] KTESTQ k1, k2
    KtestqKGqKEq,

    // [VEX.L1.66.0F.W0 4B /r] KUNPCKBW k1, k2, k3
    KunpckbwKGwKHbKEb,
    // [VEX.L1.0F.W0 4B /r] KUNPCKWD k1, k2, k3
    KunpckbdKGdKHwKEw,
    // [VEX.L1.0F.W1 4B /r] KUNPCKDQ k1, k2, k3
    KunpckbqKGqKHdKEd,

    // [VEX.L1.66.0F.W0 46 /r] KXNORB k1, k2, k3
    KxnorbKGbKHbKEb,
    // [VEX.L1.0F.W0 46 /r] KXNORW k1, k2, k3
    KxnorwKGwKHwKEw,
    // [VEX.L1.66.0F.W1 46 /r] KXNORD k1, k2, k3
    KxnordKGdKHdKEd,
    // [VEX.L1.0F.W1 46 /r] KXNORQ k1, k2, k3
    KxnorqKGqKHqKEq,

    // [VEX.L1.66.0F.W0 47 /r] KXORB k1, k2, k3
    KxorbKGbKHbKEb,
    // [VEX.L1.0F.W0 47 /r] KXORW k1, k2, k3
    KxorwKGwKHwKEw,
    // [VEX.L1.66.0F.W1 47 /r] KXORD k1, k2, k3
    KxordKGdKHdKEd,
    // [VEX.L1.0F.W1 47 /r] KXORQ k1, k2, k3
    KxorqKGqKHqKEq,

    // [9F] LAHF
    Lahf,

    // [0F 02 /r] LAR r16, r/m16
    LarGwEw,
    // [0F 02 /r] LAR reg, r32/m16
    LarGdEw,

    // [F2 0F F0 /r] LDDQU xmm1, m128
    LddquVdqMdq,
    // [VEX.128.F2.0F.WIG F0 /r] VLDDQU xmm1, m128
    VlddquVdqMdqV128,
    // [VEX.256.F2.0F.WIG F0 /r] VLDDQU ymm1, m256
    VlddquVqqMqqV256,

    // [NP 0F AE /2] LDMXCSR m32
    LdmxcsrMd,
    // [VEX.LZ.0F.WIG AE /2] VLDMXCSR m32
    VldmxcsrMd,

    // [C5 /r] LDS r16, m16:16
    LdsGwMp,
    // [C5 /r] LDS r32, m16:32
    LdsGdMp,
    // [0F B2 /r] LSS r16, m16:16
    LssGwMp,
    // [0F B2 /r] LSS r32, m16:32
    LssGdMp,
    // [REX 0F B2 /r] LSS r64, m16:64
    LssGqMp,
    // [C4 /r] LES r16, m16:16
    LesGwMp,
    // [C4 /r] LES r32, m16:32
    LesGdMp,

    // [0F B4 /r] LFS r16, m16:16
    LfsGwMp,
    // [0F B4 /r] LFS r32, m16:32
    LfsGdMp,
    // [REX 0F B4 /r] LFS r64, m16:64
    LfsGqMp,

    // [0F B5 /r] LGS r16, m16:16
    LgsGwMp,
    // [0F B5 /r] LGS r32, m16:32
    LgsGdMp,
    // [REX 0F B5 /r] LGS r64, m16:64
    LgsGqMp,

    // [8D /r] LEA r16, m
    LeaGwM,
    // [8D /r] LEA r32, m
    LeaGdM,
    // [REX.W 8D /r] LEA r64, m
    LeaGqM,

    // [C9] LEAVE
    Leave,

    // [NP 0F AE E8] LFENCE
    Lfence,

    // [0F 01 /2] GDT m16&32
    LgdtMsOp32,
    // [0F 01 /2] LGDT m16&64
    LgdtMsOp64,
    // [0F 01 /3] IDT m16&32
    LidtMsOp32,
    // [0F 01 /3] LIDT m16&64
    LidtMsOp64,

    // [0F 00 /2] LLDT r/m16
    LldtEw,

    // [0F 01 /6] LMSW r/m16
    LmswEw,

    // [F3 0F 38 DC 11:rrr:bbb] LOADIWKEY xmm1, xmm2, <EAX>, <XMM0>
    LoadiwkeyVdqUdq,

    // [F0] LOCK
    Lock,

    // [AC] LODSB
    Lodsb,
    // [AD] LODSW
    Lodsw,
    // [AD] LODSD
    Lodsd,
    // [REX.W AD] LODSQ
    Lodsq,

    // [E2 cb] LOOP rel8
    LoopJb,
    // [E1 cb] LOOPE rel8
    LoopeJb,
    // [E0 cb] LOOPNE rel8
    LoopneJb,

    // [0F 03 /r] LSL r16, r/m16
    LslGwEw,
    // [0F 03 /r] LSL r32, r32/m16
    LslGdEw,
    // [REX.W 0F 03 /r] LSL r64, r32/m16
    LslGqEw,

    // [0F 00 /3] LTR r/m16
    LtrEw,

    // [F3 0F BD /r] LZCNT r16, r/m16
    LzcntGwEw,
    // [F3 0F BD /r] LZCNT r32, r/m32
    LzcntGdEd,
    // [F3 REX.W 0F BD /r] LZCNT r64, r/m64
    LzcntGqEq,

    // [66 0F F7 /r] MASKMOVDQU xmm1, xmm2
    MaskmovdquVdqUdq,
    // [VEX.128.66.0F.WIG F7 /r] VMASKMOVDQU xmm1, xmm2
    VmaskmovdquVdqUdqV128,

    // [NP 0F F7 /r] MASKMOVQ mm1, mm2
    MaskmovqPqQq,

    // [66 0F 5F /r] MAXPD xmm1, xmm2/m128
    MaxpdVdqWdq,
    // [VEX.128.66.0F.WIG 5F /r] VMAXPD xmm1, xmm2, xmm3/m128
    VmaxpdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 5F /r] VMAXPD ymm1, ymm2, ymm3/m256
    VmaxpdVqqHqqWqqV256,
    // [EVEX.128.66.0F.W1 5F /r] VMAXPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VmaxpdVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 5F /r] VMAXPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VmaxpdVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 5F /r] VMAXPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{sae}
    VmaxpdVdqqHdqqWdqqE512,

    // [NP 0F 5F /r] MAXPS xmm1, xmm2/m128
    MaxpsVdqWdq,
    // [VEX.128.0F.WIG 5F /r] VMAXPS xmm1, xmm2, xmm3/m128
    VmaxpsVdqHdqWdqV128,
    // [VEX.256.0F.WIG 5F /r] VMAXPS ymm1, ymm2, ymm3/m256
    VmaxpsVqqHqqWqqV256,
    // [EVEX.128.0F.W0 5F /r] VMAXPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VmaxpsVdqHdqWdqE128,
    // [EVEX.256.0F.W0 5F /r] VMAXPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VmaxpsVqqHqqWqqE256,
    // [EVEX.512.0F.W0 5F /r] VMAXPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{sae}
    VmaxpsVdqqHdqqWdqqE512,

    // [F2 0F 5F /r] MAXSD xmm1, xmm2/m64
    MaxsdVdqWq,
    // [VEX.LIG.F2.0F.WIG 5F /r] VMAXSD xmm1, xmm2, xmm3/m64
    VmaxsdVdqHdqWqV,
    // [EVEX.LIG.F2.0F.W1 5F /r] VMAXSD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}
    VmaxsdVdqHdqWqE,

    // [F3 0F 5F /r] MAXSS xmm1, xmm2/m32
    MaxssVdqWd,
    // [VEX.LIG.F3.0F.WIG 5F /r] VMAXSS xmm1, xmm2, xmm3/m32
    VmaxssVdqHdqWdV,
    // [EVEX.LIG.F3.0F.W1 5F /r] VMAXSS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}
    VmaxssVdqHdqWdE,

    // [NP 0F AE F0] MFENCE
    Mfence,

    // [66 0F 5D /r] MINPD xmm1, xmm2/m128
    MinpdVdqWdq,
    // [VEX.128.66.0F.WIG 5D /r] VMINPD xmm1, xmm2, xmm3/m128
    VminpdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 5D /r] VMINPD ymm1, ymm2, ymm3/m256
    VminpdVqqHqqWqqV256,
    // [EVEX.128.66.0F.W1 5D /r] VMINPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VminpdVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 5D /r] VMINPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VminpdVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 5D /r] VMINPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{sae}
    VminpdVdqqHdqqWdqqE512,

    // [NP 0F 5D /r] MINPD xmm1, xmm2/m128
    MinpsVdqWdq,
    // [VEX.128.0F.WIG 5D /r] VMINPS xmm1, xmm2, xmm3/m128
    VminpsVdqHdqWdqV128,
    // [VEX.256.0F.WIG 5D /r] VMINPS ymm1, ymm2, ymm3/m256
    VminpsVqqHqqWqqV256,
    // [EVEX.128.0F.W1 5D /r] VMINPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VminpsVdqHdqWdqE128,
    // [EVEX.256.0F.W1 5D /r] VMINPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VminpsVqqHqqWqqE256,
    // [EVEX.512.0F.W1 5D /r] VMINPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{sae}
    VminpsVdqqHdqqWdqqE512,

    // [F2 0F 5D /r] MINSD xmm1, xmm2/m64
    MinsdVdqWq,
    // [VEX.LIG.F2.0F.WIG 5D /r] VMINSD xmm1, xmm2, xmm3/m64
    VminsdVdqHdqWqV,
    // [EVEX.LIG.F2.0F.W1 5D /r] VMINSD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}
    VminsdVdqHdqWqE,

    // [F3 0F 5D /r] MINSS xmm1, xmm2/m32
    MinssVdqWd,
    // [VEX.LIG.F3.0F.WIG 5D /r] VMINSS xmm1, xmm2, xmm3/m32
    VminssVdqHdqWdV,
    // [EVEX.LIG.F3.0F.W0 5D /r] VMINSS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}
    VminssVdqHdqWdE,

    // [0F 01 C8] MONITOR
    Monitor,

    // [88 /r] MOV r/m8, r8
    // [REX 88 /r] MOV r/m8, r8
    MovEbGb,
    // [89 /r] MOV r/m16, r16
    MovEwGw,
    // [89 /r] MOV r/m32, r32
    MovEdGd,
    // [REX.W 89 /r] MOV r/m64, r64
    MovEqGq,
    // [8A /r] MOV r8, r/m8
    // [REX 8A /r] MOV r8, r/m8
    MovGbEb,
    // [8B /r] MOV r16, r/m16
    MovGwEw,
    // [8B /r] MOV r32, r/m32
    MovGdEd,
    // [REX.W 8B /r] MOV r64, r/m64
    MovGqEq,
    // [8C /r] MOV r/m16, sreg
    MovEwSw,
    // [8C /r] MOV r16/r32/m16, sreg
    MovEdSw,
    // [REX.W 8C /r] MOV r64/m16, sreg
    MovEqSw,
    // [8E /r] MOV sreg, r/m16
    // [REX.W 8E /r] MOV sreg, r/m64
    MovSwEw,
    // [A0 ob] MOV AL, moffs8
    // [REX.W A0 ob] MOV AL, moffs8
    MovALOb,
    // [A1 ow] MOV AX, moffs16
    MovAXOw,
    // [A1 od] MOV EAX, moffs32
    MovEAXOd,
    // [REX.W A1 oq] MOV RAX, moffs64
    MovRAXOq,
    // [A2 ob] MOV moffs8, AL
    // [REX.W A2 ob] MOV moffs8, AL
    MovObAL,
    // [A3 ow] MOV moffs16, AX
    MovOwAX,
    // [A3 od] MOV moffs32, EAX
    MovOdEAX,
    // [REX.W A3 oq] MOV moffs64, RAX
    MovOqRAX,
    // [B0+rb ib] MOV r8, imm8
    // [REX B0+rb ib] MOV r8, imm8
    MovGbIb,
    // [B8+rw iw] MOV r16, imm16
    MovGwIb,
    // [B8+rd id] MOV r32, imm32
    MovGdIb,
    // [REX.W B8+rd io] MOV r64, imm64
    MovGqIb,
    // [C6 /0 ib] MOV r/m8, imm8
    // [REX C6 /0 ib] MOV r/m8, imm8
    MovEbIb,
    // [C7 /0 iw] MOV r/m16, imm16
    MovEwIw,
    // [C7 /0 id] MOV r/m32, imm32
    MovEdId,
    // [REX.W C7 /0 id] MOV r/m64, imm32
    MovEqId,

    // [0F 20 /r] MOV r32, CR0-7
    MovRdCd,
    // [0F 20 /r] MOV r64, CR0-7
    MovRqCq,
    // [REX.R 0F 20 /0] MOV r64, CR8
    MovRqCR8,
    // [0F 22 /r] MOV CR0-7, r32
    MovCdRd,
    // [0F 22 /r] MOV CR0-7, r64
    MovCqRq,
    // [REX.R 0F 22 /r] MOV CR8, r64
    MovCR8Rq,

    // [0F 21 /r] MOV r32, DR0-7
    MovRdDd,
    // [0F 21 /r] MOV r64, DR0-7
    MovRqDq,
    // [0F 23 /r] MOV DR0-7, r32
    MovDdRd,
    // [0F 23 /r] MOV DR0-7, r64
    MovDqRq,

    // [66 0F 28 /r] MOVAPD xmm1, xmm2/m128
    MovapdVdqWdq,
    // [66 0F 29 /r] MOVAPD xmm1/m128, xmm2
    MovapdWdqVdq,
    // [VEX.128.66.0F.WIG 28 /r] VMOVAPD xmm1, xmm2/m128
    VmovapdVdqWdqV128,
    // [VEX.128.66.0F.WIG 29 /r] VMOVAPD xmm1/m128, xmm2
    VmovapdWdqVdqV128,
    // [VEX.256.66.0F.WIG 28 /r] VMOVAPD ymm1, ymm2/m256
    VmovapdVqqWqqV256,
    // [VEX.256.66.0F.WIG 29 /r] VMOVAPD ymm1/m256, ymm2
    VmovapdWqqVqqV256,
    // [EVEX.128.66.0F.W1 28 /r] VMOVAPD xmm1 {k1}{z}, xmm2/m128
    VmovapdVdqWdqE128,
    // [EVEX.256.66.0F.W1 28 /r] VMOVAPD ymm1 {k1}{z}, ymm2/m256
    VmovapdVqqWqqE256,
    // [EVEX.512.66.0F.W1 28 /r] VMOVAPD zmm1 {k1}{z}, zmm2/m512
    VmovapdVdqqWdqqE512,
    // [EVEX.128.66.0F.W1 28 /r] VMOVAPD xmm1/m128 {k1}{z}, xmm2
    VmovapdWdqVdqE128,
    // [EVEX.256.66.0F.W1 28 /r] VMOVAPD ymm1/m256 {k1}{z}, ymm2
    VmovapdWqqVqqE256,
    // [EVEX.512.66.0F.W1 28 /r] VMOVAPD zmm1/m512 {k1}{z}, zmm2
    VmovapdWdqqVdqqE512,

    // [NP 0F 28 /r] MOVAPD xmm1, xmm2/m128
    MovapsVdqWdq,
    // [NP 0F 29 /r] MOVAPD xmm1/m128, xmm2
    MovapsWdqVdq,
    // [VEX.128.0F.WIG 28 /r] VMOVAPS xmm1, xmm2/m128
    VmovapsVdqWdqV128,
    // [VEX.128.0F.WIG 29 /r] VMOVAPS xmm1/m128, xmm2
    VmovapsWdqVdqV128,
    // [VEX.256.0F.WIG 28 /r] VMOVAPS ymm1, ymm2/m256
    VmovapsVqqWqqV256,
    // [VEX.256.0F.WIG 29 /r] VMOVAPS ymm1/m256, ymm2
    VmovapsWqqVqqV256,
    // [EVEX.128.0F.W1 28 /r] VMOVAPS xmm1 {k1}{z}, xmm2/m128
    VmovapsVdqWdqE128,
    // [EVEX.256.0F.W1 28 /r] VMOVAPS ymm1 {k1}{z}, ymm2/m256
    VmovapsVqqWqqE256,
    // [EVEX.512.0F.W1 28 /r] VMOVAPS zmm1 {k1}{z}, zmm2/m512
    VmovapsVdqqWdqqE512,
    // [EVEX.128.0F.W1 28 /r] VMOVAPS xmm1/m128 {k1}{z}, xmm2
    VmovapsWdqVdqE128,
    // [EVEX.256.0F.W1 28 /r] VMOVAPS ymm1/m256 {k1}{z}, ymm2
    VmovapsWqqVqqE256,
    // [EVEX.512.0F.W1 28 /r] VMOVAPS zmm1/m512 {k1}{z}, zmm2
    VmovapsWdqqVdqqE512,

    // [0F 38 F0 /r] MOVBE r16, m16
    MovbeGwMw,
    // [0F 38 F0 /r] MOVBE r32, m32
    MovbeGdMd,
    // [REX.W 0F 38 F0 /r] MOVBE r64, m64
    MovbeGqMq,
    // [0F 38 F1 /r] MOVBE m16, r16
    MovbeMwGw,
    // [0F 38 F1 /r] MOVBE m32, r32
    MovbeMdGd,
    // [REX.W 0F 38 F1 /r] MOVBE m64, r64
    MovbeMqGq,

    // [NP 0F 6E /r] MOVD mm, r/m32
    MovdPqEd,
    // [NP REX.W 0F 6E /r] MOVQ mm, r/m64
    MovqPqEq,
    // [NP 0F 7E /r] MOVD r/m32, mm
    MovdEdPq,
    // [NP REX.W 0F 7E /r] MOVQ r/m64, mm
    MovqEqPq,
    // [66 0F 6E /r] MOVD xmm1, r/m32
    MovdVdqEd,
    // [66 REX.W 0F 6E /r] MOVQ xmm1, r/m64
    MovqVdqEq,
    // [66 0F 7E /r] MOVD r/m32, xmm1
    MovdEdVdq,
    // [66 REX.W 0F 7E /r] MOVQ r/m64, xmm1
    MovqVdqEd,
    // [VEX.128.66.0F.W0 6E /r] VMOVD xmm1, r/m32
    VmovdVdqEdV128,
    // [EVEX.128.66.0F.W0 6E /r] VMOVD xmm1, r/m32
    VmovdVdqEdE128,
    // [VEX.128.66.0F.W1 6E /r] VMOVQ xmm1, r/m64
    VmovqVdqEqV128,
    // [EVEX.128.66.0F.Wq 6E /r] VMOVD xmm1, r/m64
    VmovqVdqEqE128,
    // [VEX.128.66.0F.W0 7E /r] VMOVD r/m32, xmm1
    VmovdEdVdqV128,
    // [EVEX.128.66.0F.W0 7E /r] VMOVD r/m32, xmm1
    VmovdEdVdqE128,
    // [VEX.128.66.0F.W1 7E /r] VMOVQ r/m64, xmm1
    VmovqVdqEdV128,
    // [EVEX.128.66.0F.W1 7E /r] VMOVD r/m64, xmm1
    VmovqVdqEdE128,

    // [F2 0F 12 /r] MOVDDUP xmm1, xmm2/m64
    MovddupVdqWq,
    // [VEX.128.F2.0F.WIG 12 /r] VMOVDDUP xmm1, xmm2/m64
    // NOTE: Intel manual says m64; TODO: Is this correct?
    VmovddupVdqWqV128,
    // [VEX.256.F2.0F.WIG 12 /r] VMOVDDUP ymm1, ymm2/m256
    VmovddupVqqWqqV256,
    // [EVEX.128.F2.0F.W1 12 /r] VMOVDDUP xmm1 {k1}{z}, xmm2/m64
    // NOTE: Intel manual says m64; TODO: Is this correct
    VmovddupVdqWqE128,
    // [EVEX.256.F2.0F.W1 12 /r] VMOVDDUP ymm1 {k1}{z}, ymm2/m256
    VmovddupVqqWqqE256,
    // [EVEX.512.F2.0F.W1 12 /r] VMOVDDUP zmm1 {k1}{z}, zmm2/m512
    VmovddupVdqqWdqqE512,

    // [NP 0F 38 F9 /r] MOVDIRI m32, r32
    MovdiriMdGd,
    // [NP REX.W 0F 38 F9 /r] MOVDIRI m64, r64
    MovdiriMqGq,

    // [66 0F 38 F8 /r] MOVDIR64B r16, m512
    Movdir64bGwM,
    // [66 0F 38 F8 /r] MOVDIR64B r32, m512
    Movdir64bGdM,
    // [66 0F 38 F8 /r] MOVDIR64B r64, m512
    Movdir64bGqM,

    // [66 0F 6F /r] MOVDQA xmm1, xmm2/m128
    MovdqaVdqWdq,
    // [66 0F 7F /r] MOVDQA xmm1/m128, xmm2
    MovdqaWdqVdq,
    // [VEX.128.66.0F.WIG 6F /r] VMOVDQA xmm1, xmm2/m128
    VmovdqaVdqWdqV128,
    // [VEX.128.66.0F.WIG 7F /r] VMOVDQA xmm1/m128, xmm2
    VmovdqaWdqVdqV128,
    // [VEX.256.66.0F.WIG 6F /r] VMOVDQA ymm1, ymm2/m256
    VmovdqaVqqWqqV256,
    // [VEX.256.66.0F.WIG 7F /r] VMOVDQA ymm1/m256, ymm2
    VmovdqaWqqVqqV256,
    // [EVEX.128.66.0F.W0 6F /r] VMOVDQA32 xmm1 {k1}{z}, xmm2/m128
    Vmovdqa32VdqWdqE128,
    // [EVEX.256.66.0F.W0 6F /r] VMOVDQA32 ymm1 {k1}{z}, ymm2/m256
    Vmovdqa32VqqWqqE256,
    // [EVEX.512.66.0F.W0 6F /r] VMOVDQA32 zmm1 {k1}{z}, zmm2/m512
    Vmovdqa32VdqqWdqqE512,
    // [EVEX.128.66.0F.W0 7F /r] VMOVDQA32 xmm1/m128 {k1}{z}, xmm2
    Vmovdqa32WdqVdqE128,
    // [EVEX.256.66.0F.W0 7F /r] VMOVDQA32 ymm1/m256 {k1}{z}, ymm2
    Vmovdqa32WqqVqqE256,
    // [EVEX.512.66.0F.W0 7F /r] VMOVDQA32 zmm1/m512 {k1}{z}, zmm2
    Vmovdqa32WdqqVdqqE512,
    // [EVEX.128.66.0F.W1 6F /r] VMOVDQA64 xmm1 {k1}{z}, xmm2/m128
    Vmovdqa64VdqWdqE128,
    // [EVEX.256.66.0F.W1 6F /r] VMOVDQA64 ymm1 {k1}{z}, ymm2/m256
    Vmovdqa64VqqWqqE256,
    // [EVEX.512.66.0F.W1 6F /r] VMOVDQA64 zmm1 {k1}{z}, zmm2/m512
    Vmovdqa64VdqqWdqqE512,
    // [EVEX.128.66.0F.W1 7F /r] VMOVDQA64 xmm1/m128 {k1}{z}, xmm2
    Vmovdqa64WdqVdqE128,
    // [EVEX.256.66.0F.W1 7F /r] VMOVDQA64 ymm1/m256 {k1}{z}, ymm2
    Vmovdqa64WqqVqqE256,
    // [EVEX.512.66.0F.W1 7F /r] VMOVDQA64 zmm1/m512 {k1}{z}, zmm2
    Vmovdqa64WdqqVdqqE512,

    // [F3 0F 6F /r] MOVDQU xmm1, xmm2/m128
    MovdquVdqWdq,
    // [F3 0F 7F /r] MOVDQU xmm1/m128, xmm2
    MovdquWdqVdq,
    // [VEX.128.F3.0F.WIG 6F /r] VMOVDQU xmm1, xmm2/m128
    VmovdquVdqWdqV128,
    // [VEX.128.F3.0F.WIG 7F /r] VMOVDQU xmm1/m128, xmm2
    VmovdquWdqVdqV128,
    // [VEX.256.F3.0F.WIG 6F /r] VMOVDQU ymm1, ymm2/m256
    VmovdquVqqWqqV256,
    // [VEX.256.F3.0F.WIG 7F /r] VMOVDQU ymm1/m256, ymm2
    VmovdquWqqVqqV256,
    // [EVEX.128.F2.0F.W0 6F /r] VMOVDQU8 xmm1 {k1}{z}, xmm2/m128
    Vmovdqu8VdqWdqE128,
    // [EVEX.256.F2.0F.W0 6F /r] VMOVDQU8 ymm1 {k1}{z}, ymm2/m256
    Vmovdqu8VqqWqqE256,
    // [EVEX.512.F2.0F.W0 6F /r] VMOVDQU8 zmm1 {k1}{z}, zmm2/m512
    Vmovdqu8VdqqWdqqE512,
    // [EVEX.128.F2.0F.W0 7F /r] VMOVDQU8 xmm1/m128 {k1}{z}, xmm2
    Vmovdqu8WdqVdqE128,
    // [EVEX.256.F2.0F.W0 7F /r] VMOVDQU8 ymm1/m256 {k1}{z}, ymm2
    Vmovdqu8WqqVqqE256,
    // [EVEX.512.F2.0F.W0 7F /r] VMOVDQU8 zmm1/m512 {k1}{z}, zmm2
    Vmovdqu8WdqqVdqqE512,
    // [EVEX.128.F2.0F.W1 6F /r] VMOVDQU16 xmm1 {k1}{z}, xmm2/m128
    Vmovdqu16VdqWdqE128,
    // [EVEX.256.F2.0F.W1 6F /r] VMOVDQU16 ymm1 {k1}{z}, ymm2/m256
    Vmovdqu16VqqWqqE256,
    // [EVEX.512.F2.0F.W1 6F /r] VMOVDQU16 zmm1 {k1}{z}, zmm2/m512
    Vmovdqu16VdqqWdqqE512,
    // [EVEX.128.F2.0F.W1 7F /r] VMOVDQU16 xmm1/m128 {k1}{z}, xmm2
    Vmovdqu16WdqVdqE128,
    // [EVEX.256.F2.0F.W1 7F /r] VMOVDQU16 ymm1/m256 {k1}{z}, ymm2
    Vmovdqu16WqqVqqE256,
    // [EVEX.512.F2.0F.W1 7F /r] VMOVDQU16 zmm1/m512 {k1}{z}, zmm2
    Vmovdqu16WdqqVdqqE512,
    // [EVEX.128.F3.0F.W0 6F /r] VMOVDQU32 xmm1 {k1}{z}, xmm2/m128
    Vmovdqu32VdqWdqE128,
    // [EVEX.256.F3.0F.W0 6F /r] VMOVDQU32 ymm1 {k1}{z}, ymm2/m256
    Vmovdqu32VqqWqqE256,
    // [EVEX.512.F3.0F.W0 6F /r] VMOVDQU32 zmm1 {k1}{z}, zmm2/m512
    Vmovdqu32VdqqWdqqE512,
    // [EVEX.128.F3.0F.W0 7F /r] VMOVDQU32 xmm1/m128 {k1}{z}, xmm2
    Vmovdqu32WdqVdqE128,
    // [EVEX.256.F3.0F.W0 7F /r] VMOVDQU32 ymm1/m256 {k1}{z}, ymm2
    Vmovdqu32WqqVqqE256,
    // [EVEX.512.F3.0F.W0 7F /r] VMOVDQU32 zmm1/m512 {k1}{z}, zmm2
    Vmovdqu32WdqqVdqqE512,
    // [EVEX.128.F3.0F.W1 6F /r] VMOVDQU64 xmm1 {k1}{z}, xmm2/m128
    Vmovdqu64VdqWdqE128,
    // [EVEX.256.F3.0F.W1 6F /r] VMOVDQU64 ymm1 {k1}{z}, ymm2/m256
    Vmovdqu64VqqWqqE256,
    // [EVEX.512.F3.0F.W1 6F /r] VMOVDQU64 zmm1 {k1}{z}, zmm2/m512
    Vmovdqu64VdqqWdqqE512,
    // [EVEX.128.F3.0F.W1 7F /r] VMOVDQU64 xmm1/m128 {k1}{z}, xmm2
    Vmovdqu64WdqVdqE128,
    // [EVEX.256.F3.0F.W1 7F /r] VMOVDQU64 ymm1/m256 {k1}{z}, ymm2
    Vmovdqu64WqqVqqE256,
    // [EVEX.512.F3.0F.W1 7F /r] VMOVDQU64 zmm1/m512 {k1}{z}, zmm2
    Vmovdqu64WdqqVdqqE512,

    // [F2 0F D6 /r] MOVDQ2Q mm, xmm
    Movdq2qPqUdq,

    // [NP 0F 12 /r] MOVHLPS xmm1, xmm2
    MovhlpsVdqUdq,
    // [VEX.128.0F.WIG 12 /r] VMOVHLPS xmm1, xmm2, xmm3
    VmovhlpsVdqHdqUdqV128,
    // [EVEX.128.0F.W0 12 /r] VMOVHLPS xmm1, xmm2, xmm3
    VmovhlpsVdqHdqUdqE128,

    // [66 0F 16 /r] MOVHPD xmm1, m64
    MovhpdVdqMq,
    // [VEX.128.66.0F.WIG 16 /r] VMOVHPD xmm1, xmm2, m64
    VmovhpdVdqHdqMqV128,
    // [EVEX.128.66.0F.W1 16 /r] VMOVHPD xmm1, xmm2, m64
    VmovhpdVdqHdqMqE128,
    // [66 0F 17 /r] MOVHPD m64, xmm1
    MovhpdMqVdq,
    // [VEX.128.66.0F.WIG 17 /r] VMOVHPD m64, xmm1
    VmovhpdMqVdqV128,
    // [EVEX.128.66.0F.W1 17 /r] VMOVHPD m64, xmm1
    VmovhpdMqVdqE128,

    // [NP 0F 16 /r] MOVHPS xmm1, m64
    MovhpsVdqMq,
    // [VEX.128.0F.WIG 16 /r] VMOVHPS xmm1, xmm2, m64
    VmovhpsVdqHdqMqV128,
    // [EVEX.128.0F.W1 16 /r] VMOVHPS xmm1, xmm2, m64
    VmovhpsVdqHdqMqE128,
    // [NP 0F 17 /r] MOVHPS m64, xmm1
    MovhpsMqVdq,
    // [VEX.128.0F.WIG 17 /r] VMOVHPS m64, xmm1
    VmovhpsMqVdqV128,
    // [EVEX.128.0F.W1 17 /r] VMOVHPS m64, xmm1
    VmovhpsMqVdqE128,

    // [NP 0F 16 /r] MOVLHPS xmm1, xmm2
    MovlhpsVdqWdq,
    // [VEX.128.0F.WIG 16 /r] VMOVLHPS xmm1, xmm2, xmm3
    VmovlhpsVdqHdqWdqV128,
    // [EVEX.128.0F.W0 16 /r] VMOVLHPS xmm1, xmm2, xmm3
    VmovlhpsVdqHdqWdqE128,

    // [66 0F 12 /r] MOVLPD xmm1, m64
    MovlpdVdqMq,
    // [VEX.128.66.0F.WIG 12 /r] VMOVLPD xmm1, xmm2, m64
    VmovlpdVdqHdqMqV128,
    // [EVEX.128.66.0F.W1 12 /r] VMOVLPD xmm1, xmm2, m64
    VmovlpdVdqHdqMqE128,
    // [66 0F 13 /r] MOVLPD m64, xmm1
    MovlpdMqVdq,
    // [VEX.128.66.0F.WIG 13 /r] VMOVLPD m64, xmm1
    VmovlpdMqVdqV128,
    // [EVEX.128.66.0F.W1 13 /r] VMOVLPD m64, xmm1
    VmovlpdMqVdqE128,

    // [NP 0F 12 /r] MOVLPS xmm1, m64
    MovlpsVdqMq,
    // [VEX.128.66.0F.WIG 12 /r] VMOVLPS xmm1, xmm2, m64
    VmovlpsVdqHdqMqV128,
    // [EVEX.128.66.0F.W1 12 /r] VMOVLPS xmm1, xmm2, m64
    VmovlpsVdqHdqMqE128,
    // [NP 0F 13 /r] MOVLPS m64, xmm1
    // NOTE: Intel manual doesn't have `NP`
    MovlpsMqVdq,
    // [VEX.128.0F.WIG 13 /r] VMOVLPS m64, xmm1
    VmovlpsMqVdqV128,
    // [EVEX.128.0F.W1 13 /r] VMOVLPS m64, xmm1
    VmovlpsMqVdqE128,

    // [66 0F 50 /r] MOVMSKPD reg, xmm
    MovmskpdGdUdq,
    // [VEX.128.66.0F.WIG 50 /r] VMOVMSKPD reg, xmm
    VmovmskpdGdUdqV128,
    // [VEX.256.66.0F.WIG 50 /r] VMOVMSKPD reg, ymm
    VmovmskpdGdUqqV256,

    // [NP 0F 50 /r] MOVMSKPS reg, xmm
    MovmskpsGdUdq,
    // [VEX.128.0F.WIG 50 /r] VMOVMSKPS reg, xmm
    VmovmskpsGdUdqV128,
    // [VEX.256.0F.WIG 50 /r] VMOVMSKPS reg, ymm
    VmovmskpsGdUqqV256,

    // [66 0F 38 2A /r] MOVNTDQA xmm1, m128
    MovntdqaVdqMdq,
    // [VEX.128.66.0F38.WIG 2A /r] VMOVNTDQA xmm1, m128
    VmovntdqaVdqMdqV128,
    // [VEX.256.66.0F38.WIG 2A /r] VMOVNTDQA ymm1, m256
    VmovntdqaVqqMqqV256,
    // [EVEX.128.66.0F38.W0 2A /r] VMOVNTDQA xmm1, m128
    VmovntdqaVdqMdqE128,
    // [EVEX.256.66.0F38.W0 2A /r] VMOVNTDQA ymm1, m256
    VmovntdqaVqqMqqE256,
    // [EVEX.512.66.0F38.W0 2A /r] VMOVNTDQA zmm1, m512
    VmovntdqaVdqqMdqqE512,

    // [66 0F E7 /r] MOVNTDQ m128, xmm1
    MovntdqMdqVdq,
    // [VEX.128.66.0F.WIG E7 /r] VMOVNTDQ m128, xmm1
    VmovntdqMdqVdqV128,
    // [VEX.256.66.0F.WIG E7 /r] VMOVNTDQ m256, ymm1
    VmovntdqMqqVqqV256,
    // [EVEX.128.66.0F.W0 E7 /r] VMOVNTDQ m128, xmm1
    VmovntdqMdqVdqE128,
    // [EVEX.256.66.0F.W0 E7 /r] VMOVNTDQ m256, ymm1
    VmovntdqMqqVqqE256,
    // [EVEX.512.66.0F.W0 E7 /r] VMOVNTDQ m512, zmm1
    VmovntdqMdqqVdqqE512,

    // [NP 0F C3 /r] MOVNTI m32, r32
    MovntiMdGd,
    // [NP REX.W 0F C3 /r] MOVNTI m64, r64
    MovntiMqGq,

    // [66 0F 2B /r] MOVNTPD m128, xmm1
    MovntpdMdqVdq,
    // [VEX.128.66.0F.WIG 2B /r] VMOVNTPD m128, xmm1
    VmovntpdMdqVdqV128,
    // [VEX.256.66.0F.WIG 2B /r] VMOVNTPD m256, ymm1
    VmovntpdMqqVqqV256,
    // [EVEX.128.66.0F.W1 2B /r] VMOVNTPD m128, xmm1
    VmovntpdMdqVdqE128,
    // [EVEX.256.66.0F.W1 2B /r] VMOVNTPD m256, ymm1
    VmovntpdMqqVqqE256,
    // [EVEX.512.66.0F.W1 2B /r] VMOVNTPD m512, zmm1
    VmovntpdMdqqVdqqE512,

    // [NP 0F 2B /r] MOVNTPS m128, xmm1
    MovntpsMdqVdq,
    // [VEX.128.0F.WIG 2B /r] VMOVNTPS m128, xmm1
    VmovntpsMdqVdqV128,
    // [VEX.256.0F.WIG 2B /r] VMOVNTPS m256, ymm1
    VmovntpsMqqVqqV256,
    // [EVEX.128.0F.W0 2B /r] VMOVNTPS m128, xmm1
    VmovntpsMdqVdqE128,
    // [EVEX.256.0F.W0 2B /r] VMOVNTPS m256, ymm1
    VmovntpsMqqVqqE256,
    // [EVEX.512.0F.W0 2B /r] VMOVNTPS m512, zmm1
    VmovntpsMdqqVdqqE512,

    // [NP 0F E7 /r] MOVNTQ m64, mm
    MovntqMqPq,

    // [NP 0F 6F /r] MOVQ mm, mm/m64
    MovqPqQq,
    // [NP 0F 7F /r] MOVQ mm/m64, mm
    MovqQqPq,
    // [F3 0F 7E /r] MOVQ xmm1, xmm2/m64
    MovqVdqWq,
    // [VEX.128.F3.0F.WIG 7E /r] VMOVQ xmm1, xmm2/m64
    VmovqVdqWqV128,
    // [EVEX.128.F3.0F.W1 7E /r] VMOVQ xmm1, xmm2/m64
    VmovqVdqWqE128,
    // [66 0F D6 /r] MOVQ xmm1/m64, xmm2
    MovqWqVdq,
    // [VEX.128.66.0F.WIG D6 /r] VMOVQ xmm1/m64, xmm2
    VmovqEqVdqV128,
    // [EVEX.128.66.0F.W1 D6 /r] VMOVQ xmm1/m64, xmm2
    VmovqEqVdqE128,

    // [F3 0F D6 /r] MOVQ2DQ xmm1, mm1
    Movq2dqVdqQq,

    // [A4] MOVS m8, m8
    // [A4] MOVSB
    MovsYbXb,
    // [A5] MOVS m16, m16
    // [A5] MOVSW
    MovsYwXw,
    // [A5] MOVS m32, m32
    // [A5] MOVSD
    MovsYdXd,
    // [REX.W A5] MOVS m64, m64
    // [REX.W A5] MOVSQ
    MovsYqXq,

    // [F2 0F 10 /r] MOVSD xmm1, xmm2
    // [F2 0F 10 /r] MOVSD xmm1, m64
    MovsdVdqWq,
    // [F2 0F 11 /r] MOVSD xmm1/m64, xmm2
    MovsdWqVdq,
    // [VEX.LIG.F2.0F.WIG 10 /r] VMOVSD xmm1, xmm2, xmm3
    // [VEX.LIG.F2.0F.WIG 10 /r] VMOVSD xmm1, m64
    VmovsdVdqHdqWdqV,
    // [VEX.LIG.F2.0F.WIG 11 /r] VMOVSD xmm1, xmm2, xmm3
    // [VEX.LIG.F2.0F.WIG 11 /r] VMOVSD m64, xmm1
    VmovsdWdqHdqVdqV,
    // [EVEX.LIG.F2.0F.W1 10 /r] VMOVSD xmm1 {k1}{z}, xmm2, xmm3
    // [EVEX.LIG.F2.0F.W1 10 /r] VMOVSD xmm1 {k1}{z}, m64
    VmovsdVdqHdqWdqE,
    // [EVEX.LIG.F2.0F.W1 11 /r] VMOVSD xmm1 {k1}{z}, xmm2, xmm3
    // [EVEX.LIG.F2.0F.W1 11 /r] VMOVSD m64 {k1}{z}, xmm1
    VmovsdWdqHdqVdqE,

    // [F3 0F 16 /r] MOVSHDUP xmm1, xmm2/m128
    MovshdupVdqWdq,
    // [VEX.128.F3.0F.WIG 16 /r] VMOVSHDUP xmm1, xmm2/m128
    VmovshdupVdqWdqV128,
    // [VEX.256.F3.0F.WIG 16 /r] VMOVSHDUP ymm1, ymm2/m256
    VmovshdupVqqWqqV256,
    // [EVEX.128.F3.0F.W0 16 /r] VMOVSHDUP xmm1 {k1}{z}, xmm2/m128
    VmovshdupVdqWdqE128,
    // [EVEX.256.F3.0F.W0 16 /r] VMOVSHDUP ymm1 {k1}{z}, ymm2/m256
    VmovshdupVqqWqqE256,
    // [EVEX.512.F3.0F.W0 16 /r] VMOVSHDUP zmm1 {k1}{z}, zmm2/m512
    VmovshdupVdqqWdqqE512,

    // [F3 0F 12 /r] MOVSLDUP xmm1, xmm2/m128
    MovsldupVdqWdq,
    // [VEX.128.F3.0F.WIG 12 /r] VMOVSLDUP xmm1, xmm2/m128
    VmovsldupVdqWdqV128,
    // [VEX.256.F3.0F.WIG 12 /r] VMOVSLDUP ymm1, ymm2/m256
    VmovsldupVqqWqqV256,
    // [EVEX.128.F3.0F.W0 12 /r] VMOVSLDUP xmm1 {k1}{z}, xmm2/m128
    VmovsldupVdqWdqE128,
    // [EVEX.256.F3.0F.W0 12 /r] VMOVSLDUP ymm1 {k1}{z}, ymm2/m256
    VmovsldupVqqWqqE256,
    // [EVEX.512.F3.0F.W0 12 /r] VMOVSLDUP zmm1 {k1}{z}, zmm2/m512
    VmovsldupVdqqWdqqE512,

    // [F3 0F 10 /r] MOVSS xmm1, xmm2
    // [F3 0F 10 /r] MOVSS xmm1, m32
    MovssVdqWd,
    // [VEX.LIG.F3.0F.WIG 10 /r] VMOVSS xmm1, xmm2, xmm3
    // [VEX.LIG.F3.0F.WIG 10 /r] VMOVSS xmm1, m32
    VmovssVdqHdqWdqV,
    // [F3 0F 11 /r] MOVSS xmm1/m32, xmm1
    MovssWdVdq,
    // [VEX.LIG.F3.0F.WIG 11 /r] VMOVSS xmm1, xmm2, xmm3
    // [VEX.LIG.F3.0F.WIG 11 /r] VMOVSS m32, xmm1
    VmovssWdqHdqVdqV,
    // [EVEX.LIG.F3.0F.W0 10 /r] VMOVSS xmm1 {k1}{z}, xmm2, xmm3
    // [EVEX.LIG.F3.0F.W0 10 /r] VMOVSS xmm1 {k1}{z}, m32
    VmovssVdqHdqWdqE,
    // [EVEX.LIG.F3.0F.W0 11 /r] VMOVSS xmm1 {k1}{z}, xmm2, xmm3
    // [EVEX.LIG.F3.0F.W0 11 /r] VMOVSS m32 {k1}{z}, xmm1
    VmovssWdqHdqVdqE,

    // [0F BE /r] MOVSX r16, r/m8
    MovsxGwEb,
    // [0F BE /r] MOVSX r32, r/m8
    MovsxGdEb,
    // [REX.W 0F BE /r] MOVSX r64, r/m8
    MovsxGqEb,
    // [0F BF /r] MOVSX r32, r/m16
    MovsxGdEw,
    // [REX.W 0F BF /r] MOVSX r64, r/m16
    MovsxGqEw,
    // [63 /r] MOVSXD r16, r/m16
    MovsxdGwEw,
    // [63 /r] MOVSXD r32, r/m32
    MovsxdGdEd,
    // [REX.W 63 /r] MOVSXD r64, r/m32
    MovsxdGqEd,

    // [66 0F 10 /r] MOVUPD xmm1, xmm2/m128
    MovupdVdqWdq,
    // [66 0F 11 /r] MOVUPD xmm1/m128, xmm2
    MovupdWdqVdq,
    // [VEX.128.66.0F.WIG 10 /r] VMOVUPD xmm1, xmm2/m128
    VmovupdVdqWdqV128,
    // [VEX.128.66.0F.WIG 11 /r] VMOVUPD xmm1/m128, xmm2
    VmovupdWdqVdqV128,
    // [VEX.256.66.0F.WIG 10 /r] VMOVUPD ymm1, ymm2/m256
    VmovupdVqqWqqV256,
    // [VEX.256.66.0F.WIG 11 /r] VMOVUPD ymm1/m256, ymm2
    VmovupdWqqVqqV256,
    // [EVEX.128.66.0F.W1 10 /r] VMOVUPD xmm1 {k1}{z}, xmm2/m128
    VmovupdVdqWdqE128,
    // [EVEX.128.66.0F.W1 11 /r] VMOVUPD xmm1/m128 {k1}{z}, xmm2
    VmovupdWdqVdqE128,
    // [EVEX.256.66.0F.W1 10 /r] VMOVUPD ymm1 {k1}{z}, ymm2/m256
    VmovupdVqqWqqE256,
    // [EVEX.256.66.0F.W1 11 /r] VMOVUPD ymm1/m256 {k1}{z}, ymm2
    VmovupdWqqVqqE256,
    // [EVEX.512.66.0F.W1 10 /r] VMOVUPD zmm1 {k1}{z}, zmm2/m512
    VmovupdVdqqWdqqE512,
    // [EVEX.512.66.0F.W1 11 /r] VMOVUPD zmm1/m512 {k1}{z}, zmm2
    VmovupdWdqqVdqqE512,

    // [NP 0F 10 /r] MOVUPS xmm1, xmm2/m128
    MovupsVdqWdq,
    // [NP 0F 11 /r] MOVUPS xmm1/m128, xmm2
    MovupsWdqVdq,
    // [VEX.128.0F.WIG 10 /r] VMOVUPS xmm1, xmm2/m128
    VmovupsVdqWdqV128,
    // [VEX.128.0F.WIG 11 /r] VMOVUPS xmm1/m128, xmm2
    VmovupsWdqVdqV128,
    // [VEX.256.0F.WIG 10 /r] VMOVUPS ymm1, ymm2/m256
    VmovupsVqqWqqV256,
    // [VEX.256.0F.WIG 11 /r] VMOVUPS ymm1/m256, ymm2
    VmovupsWqqVqqV256,
    // [EVEX.128.0F.W1 10 /r] VMOVUPS xmm1 {k1}{z}, xmm2/m128
    VmovupsVdqWdqE128,
    // [EVEX.256.0F.W1 10 /r] VMOVUPS ymm1 {k1}{z}, ymm2/m256
    VmovupsVqqWqqE256,
    // [EVEX.512.0F.W1 10 /r] VMOVUPS zmm1 {k1}{z}, zmm2/m512
    VmovupsVdqqWdqqE512,
    // [EVEX.128.0F.W1 11 /r] VMOVUPS xmm1/m128 {k1}{z}, xmm2
    VmovupsWdqVdqE128,
    // [EVEX.256.0F.W1 11 /r] VMOVUPS ymm1/m256 {k1}{z}, ymm2
    VmovupsWqqVqqE256,
    // [EVEX.512.0F.W1 11 /r] VMOVUPS zmm1/m512 {k1}{z}, zmm2
    VmovupsWdqqVdqqE512,

    // [0F B6 /r] MOVZX r16, r/m8
    MovzxGwEb,
    // [0F B6 /r] MOVZX r32, r/m8
    MovzxGdEb,
    // [REX.W 0F B6 /r] MOVZX r64, r/m8
    MovzxGqEb,
    // [0F B7 /r] MOVZX r32, r/m16
    MovzwGdEw,
    // [REX.W 0F B7 /r] MOVZX r64, r/m16
    MovzwGqEw,

    // [66 0F 3A 42 /r ib] MPSADBW xmm1, xmm2/m128, imm8
    MpsadbwVdqWdqIb,
    // [VEX.128.66.0F3A.WIG 42 /r ib] VMPSADBW xmm1, xmm2, xmm3/m128, imm8
    VmpsadbwVdqHdqWdqIbV128,
    // [VEX.256.66.0F3A.WIG 42 /r ib] VMPSADBW ymm1, ymm2, ymm3/m256, imm8
    VmpsadbwVqqHqqWqqIbV256,

    // [F6 /4] MUL r/m8
    // [REX F6 /4] MUL r/m8
    MulEb,
    // [F7 /4] MUL r/m16
    MulEw,
    // [F7 /4] MUL r/m32
    MulEd,
    // [REX.W F7 /4] MUL r/m64
    MulEq,

    // [66 0F 59 /r] MULPD xmm1, xmm2/m128
    MulpdVdqWdq,
    // [VEX.128.66.0F.WIG 59 /r] VMULPD xmm1, xmm2, xmm3/m128
    VmulpdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 59 /r] VMULPD ymm1, ymm2, ymm3/m256
    VmulpdVqqHqqWqqV256,
    // [EVEX.128.66.0F.W1 59 /r] VMULPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VmulpdVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 59 /r] VMULPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VmulpdVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 59 /r] VMULPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VmulpdVdqqHdqqWdqqE512,

    // [NP 0F 59 /r] MULPS xmm1, xmm2/m128
    MulpsVdqWdq,
    // [VEX.128.0F.WIG 59 /r] VMULPS xmm1, xmm2, xmm3/m128
    VmulpsVdqHdqWdqV128,
    // [VEX.256.0F.WIG 59 /r] VMULPS ymm1, ymm2, ymm3/m256
    VmulpsVqqHqqWqqV256,
    // [EVEX.128.0F.W1 59 /r] VMULPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VmulpsVdqHdqWdqE128,
    // [EVEX.256.0F.W1 59 /r] VMULPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VmulpsVqqHqqWqqE256,
    // [EVEX.512.0F.W1 59 /r] VMULPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VmulpsVdqqHdqqWdqqE512,

    // [F2 0F 59 /r] MULSD xmm1, xmm2/m64
    MulsdVdqWq,
    // [VEX.LIG.F2.0F.WIG 59 /r] VMULSD xmm1, xmm2, xmm3/m64
    VmulsdVdqHdqWqV,
    // [EVEX.LIG.F2.0F.W1 59 /r] VMULSD xmm1 {k1}{z}, xmm2, xmm3/m64{er}
    VmulsdVdqHdqWqE,

    // [F3 0F 59 /r] MULSS xmm1, xmm2/m32
    MulssVdqWd,
    // [VEX.LIG.F3.0F.WIG 59 /r] VMULSS xmm1, xmm2, xmm3/m32
    VmulssVdqHdqWdV,
    // [EVEX.LIG.F3.0F.W1 59 /r] VMULSS xmm1 {k1}{z}, xmm2, xmm3/m32{er}
    VmulssVdqHdqWdE,

    // [VEX.LZ.F2.0F38.W0 F6 /r] MULX r32a, r32b, r/m32
    MulxGdBdEd,
    // [VEX.LZ.F2.0F38.W1 F6 /r] MULX r64a, r64b, r/m64
    MulxGqBqEq,

    // [0F 01 C9] MWAIT
    Mwait,

    // [F6 /3] NEG r/m8
    // [REX F6 /3] NEG r/m8
    NegEb,
    // [F7 /3] NEG r/m16
    NegEw,
    // [F7 /3] NEG r/m32
    NegEd,
    // [REX.W F7 /3] NEG r/m64
    NegEq,

    // [NP 90] NOP
    Nop,
    // [NP 0F 1F /0] NOP r/m16
    NopEw,
    // [NP 0F 1F /0] NOP r/m32
    NopEd,

    // [F6 /2] NOT r/m8
    // [REX F6 /2] NOT r/m8
    NotEb,
    // [F7 /2] NOT r/m16
    NotEw,
    // [F7 /2] NOT r/m32
    NotEd,
    // [REX.W F7 /2] NOT r/m64
    NotEq,

    // [0C ib] OR AL, imm8
    OrALIb,
    // [0D iw] OR AX, imm16
    OrAXIw,
    // [0D id] OR EAX, imm32
    OrEAXId,
    // [REX.W 0D id] OR RAX, imm32
    OrRAXId,
    // [80 /1 ib] OR r/m8, imm8
    // [REX 80 /1 ib] OR r/m8, imm8
    OrEbIb,
    // [81 /1 iw] OR r/m16, imm16
    OrEwIw,
    // [81 /1 id] OR r/m32, imm32
    OrEdId,
    // [REX.W 81 /1 id] OR r/m64, imm32
    OrEqId,
    // [83 /1 ib] OR r/m16, imm8
    OrEwIb,
    // [83 /1 ib] OR r/m32, imm8
    OrEdIb,
    // [REX.W 83 /1 ib] OR r/m64, imm8
    OrEqIb,
    // [08 /r] OR r/m8, r8
    // [REX 08 /r] OR r/m8, r8
    OrEbGb,
    // [09 /r] OR r/m16, r16
    OrEwGw,
    // [09 /r] OR r/m32, r32
    OrEdGd,
    // [REX.W 09 /r] OR r/m64, r64
    OrEqGq,
    // [0A /r] OR r8, r/m8
    // [REX 0A /r] OR r8, r/m8
    OrGbEb,
    // [0B /r] OR r16, r/m16
    OrGwEw,
    // [0B /r] OR r32, r/m32
    OrGdEd,
    // [REX.W 0B /r] OR r64, r/m64
    OrGqEq,

    // [66 0F 56 /r] ORPD xmm1, xmm2/m128
    OrpdVdqWdq,
    // [VEX.128.66.0F 56 /r] VORPD xmm1, xmm2, xmm3/m128
    VorpdVdqHdqWdqV128,
    // [VEX.256.66.0F 56 /r] VORPD ymm1, ymm2, ymm3/m256
    VorpdVqqHqqWqqV256,
    // [EVEX.128.66.0F.W1 56 /r] VORPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VorpdVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 56 /r] VORPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VorpdVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 56 /r] VORPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VorpdVdqqHdqqWdqqE512,

    // [NP 0F 56 /r] ORPS xmm1, xmm2/m128
    OrpsVdqWdq,
    // [VEX.128.0F 56 /r] VORPS xmm1, xmm2, xmm3/m128
    VorpsVdqHdqWdqV128,
    // [VEX.256.0F 56 /r] VORPS ymm1, ymm2, ymm3/m256
    VorpsVqqHqqWqqV256,
    // [EVEX.128.0F.W1 56 /r] VORPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VorpsVdqHdqWdqE128,
    // [EVEX.256.0F.W1 56 /r] VORPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VorpsVqqHqqWqqE256,
    // [EVEX.512.0F.W1 56 /r] VORPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VorpsVdqqHdqqWdqqE512,

    // [E6 ib] OUT imm8, AL
    OutIbAL,
    // [E7 ib] OUT imm8, AX
    OutIbAX,
    // [E7 ib] OUT imm8, EAX
    OutIbEAX,
    // [EE] OUT DX, AL
    OutDXAL,
    // [EF] OUT DX, AX
    OutDXAX,
    // [EF] OUT DX, EAX
    OutDXEAX,

    // [6E] OUTS m8, DX
    // [6E] OUTSB
    Outsb,
    // [6F] OUTS m16, DX
    // [6F] OUTSW,
    Outsw,
    // [6F] OUTS m32, DX
    // [6F] OUTSD
    Outsd,

    // [EVEX.NDS.128.F2.0F38.W0 68 /r] VP2INTERSECTD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vp2intersectdVdqHdqWdqE128,
    // [EVEX.NDS.256.F2.0F38.W0 68 /r] VP2INTERSECTD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vp2intersectdVqqHqqWqqE256,
    // [EVEX.NDS.512.F2.0F38.W0 68 /r] VP2INTERSECTD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    Vp2intersectdVdqqHdqqWdqqE512,
    // [EVEX.NDS.128.F2.0F38.W1 68 /r] VP2INTERSECTQ xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vp2intersectqVdqHdqWdqE128,
    // [EVEX.NDS.256.F2.0F38.W1 68 /r] VP2INTERSECTQ ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vp2intersectqVqqHqqWqqE256,
    // [EVEX.NDS.512.F2.0F38.W1 68 /r] VP2INTERSECTQ zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    Vp2intersectqVdqqHdqqWdqqE512,

    // [NP 0F 38 1C /r] PABSB mm1, mm2/m64
    PabsbPqQq,
    // [66 0F 38 1C /r] PABSB xmm1, xmm2/m128
    PabsbVdqWdq,
    // [NP 0F 38 1D /r] PABSW mm1, mm2/m64
    PabswPqQq,
    // [66 0F 38 1D /r] PABSW xmm1, xmm2/m128
    PabswVdqWdp,
    // [NP 0F 38 1E /r] PABSD mm1, mm2/m64
    PabsdPqQq,
    // [66 0F 38 1E /r] PABSD xmm1, xmm2/m128
    PabsdVdqWdq,
    // [VEX.128.66.0F38.WIG 1C /r] VPABSB xmm1, xmm2/m128
    VpabsbVdqWdqV128,
    // [VEX.128.66.0F38.WIG 1D /r] VPABSW xmm1, xmm2/m128
    VpabswVdqWdqV128,
    // [VEX.128.66.0F38.WIG 1E /r] VPABSD xmm1, xmm2/m128
    VpabsdVdqWdqV128,
    // [VEX.256.66.0F38.WIG 1C /r] VPABSB ymm1, ymm2/m256
    VpabsbVqqWqqV256,
    // [VEX.256.66.0F38.WIG 1D /r] VPABSW ymm1, ymm2/m256
    VpabswVqqWqqV256,
    // [VEX.256.66.0F38.WIG 1E /r] VPABSD ymm1, ymm2/m256
    VpabsdVqqWqqV256,
    // [EVEX.128.66.0F38.WIG 1C /r] VPABSB xmm1 {k1}{z}, xmm2/m128
    VpabsbVdqWdqE128,
    // [EVEX.256.66.0F38.WIG 1C /r] VPABSB ymm1 {k1}{z}, ymm2/m256
    VpabsbVqqWqqE256,
    // [EVEX.512.66.0F38.WIG 1C /r] VPABSB zmm1 {k1}{z}, zmm2/m512
    VpabsbVdqqWdqqE512,
    // [EVEX.128.66.0F38.WIG 1D /r] VPABSW xmm1 {k1}{z}, xmm2/m128
    VpabswVdqWdqE128,
    // [EVEX.256.66.0F38.WIG 1D /r] VPABSW ymm1 {k1}{z}, ymm2/m256
    VpabswVqqWqqE256,
    // [EVEX.512.66.0F38.WIG 1D /r] VPABSW zmm1 {k1}{z}, zmm2/m512
    VpabswVdqqWdqqE512,
    // [EVEX.128.66.0F38.WIG 1E /r] VPABSD xmm1 {k1}{z}, xmm2/m128
    VpabsdVdqWdqE128,
    // [EVEX.256.66.0F38.WIG 1E /r] VPABSD ymm1 {k1}{z}, ymm2/m256
    VpabsdVqqWqqE256,
    // [EVEX.512.66.0F38.WIG 1E /r] VPABSD zmm1 {k1}{z}, zmm2/m512
    VpabsdVdqqWdqqE512,

    // [NP 0F 63 /r] PACKSSWB mm1, mm2/m64
    PacksswbPqQq,
    // [66 0F 63 /r] PACKSSWB xmm1, xmm2/m128
    PacksswbVdqWdq,
    // [NP 0F 6B /r] PACKSSDW mm1, mm2/m64
    PackssdwPqQq,
    // [66 0F 6B /r] PACKSSDW xmm1, xmm2/m128
    PackssdwVdqWdq,
    // [VEX.128.66.0F.WIG 63 /r] VPACKSSWB xmm1, xmm2, xmm3/m128
    VpacksswbVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 6B /r] VPACKSSDW xmm1, xmm2, xmm3/m128
    VpackssdwVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 63 /r] VPACKSSWB ymm1, ymm2, ymm3/m256
    VpacksswbVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG 6B /r] VPACKSSDW ymm1, ymm2, ymm3/m256
    VpackssdwVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG 63 /r] VPACKSSWB xmm1 {k1}{z}, xmm2, xmm3/m128
    VpacksswbVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG 63 /r] VPACKSSWB ymm1 {k1}{z}, ymm2, ymm3/m256
    VpacksswbVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG 63 /r] VPACKSSWB zmm1 {k1}{z}, zmm2, xmm3/m512
    VpacksswbVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F.W0 6B /r] VPACKSSDW xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpackssdwVdqHdqWdqE128,
    // [EVEX.256.66.0F.W0 6B /r] VPACKSSDW ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpackssdwVqqHqqWqqE256,
    // [EVEX.512.66.0F.W0 6B /r] VPACKSSDW zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpackssdwVdqqHdqqWdqqE512,

    // [66 0F 38 2B /r] PACKUSDW xmm1, xmm2/m128
    PackusdwVdqWdq,
    // [VEX.128.66.0F38.WIG 2B /r] VPACKUSDW xmm1, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VpackusdwVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 2B /r] VPACKUSDW ymm1, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VpackusdwVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W0 2B /r] VPACKUSDW xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpackusdwVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 2B /r] VPACKUSDW ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpackusdwVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 2B /r] VPACKUSDW zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpackusdwVdqqHdqqWdqqE512,

    // [NP 0F 67 /r] PACKUSWB mm1, mm2/m64
    PackuswbPqQq,
    // [66 0F 67 /r] PACKUSWB xmm1, xmm2/m128
    PackuswbVdqWdq,
    // [VEX.128.66.0F.WIG 67 /r] VPACKUSWB xmm1, xmm2, xmm3/m128
    VpackuswbVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 67 /r] VPACKUSWB ymm1, ymm2, ymm3/m128
    VpackuswbVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG 67 /r] VPACKUSWB xmm1 {k1}{z}, xmm2, xmm3/m128
    VpackuswbVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG 67 /r] VPACKUSWB ymm1 {k1}{z}, ymm2, ymm3/m256
    VpackuswbVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG 67 /r] VPACKUSWB zmm1 {k1}{z}, zmm2, zmm3/m512
    VpackuswbVdqqHdqqWdqqE512,

    // [NP 0F FC /r] PADDB mm1, mm2/m64
    PaddbPqQq,
    // [NP 0F FD /r] PADDW mm1, mm2/m64
    PaddwPqQq,
    // [NP 0F FE /r] PADDD mm1, mm2/m64
    PadddPqQq,
    // [NP 0F D4 /r] PADDQ mm1, mm2/m64
    PaddqPqQq,
    // [66 0F FC /r] PADDB xmm1, xmm2/m128
    PaddbVdqWdq,
    // [66 0F FD /r] PADDW xmm1, xmm2/m128
    PaddwVdqWdq,
    // [66 0F FE /r] PADDD xmm1, xmm2/m128
    PadddVdqWdq,
    // [66 0F D4 /r] PADDQ xmm1, xmm2/m128
    PaddqVdqWdq,
    // [VEX.128.66.0F.WIG FC /r] VPADDB xmm1, xmm2, xmm3/m128
    VpaddbVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG FD /r] VPADDW xmm1, xmm2, xmm3/m128
    VpaddwVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG FE /r] VPADDD xmm1, xmm2, xmm3/m128
    VpadddVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG D4 /r] VPADDQ xmm1, xmm2, xmm3/m128
    VpaddqVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG FC /r] VPADDB ymm1, ymm2, ymm3/m256
    VpaddbVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG FD /r] VPADDW ymm1, ymm2, ymm3/m256
    VpaddwVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG FE /r] VPADDD ymm1, ymm2, ymm3/m256
    VpadddVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG D4 /r] VPADDQ ymm1, ymm2, ymm3/m256
    VpaddqVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG FC /r] VPADDB xmm1 {k1}{z}, xmm2, xmm3/m128
    VpaddbVdqHdqWdqE128,
    // [EVEX.128.66.0F.WIG FD /r] VPADDW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpaddwVdqHdqWdqE128,
    // [EVEX.128.66.0F.W0 FE /r] VPADDD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpadddVdqHdqWdqE128,
    // [EVEX.128.66.0F.W1 D4 /r] VPADDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpaddqVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG FC /r] VPADDB ymm1 {k1}{z}, ymm2, ymm3/m256
    VpaddbVqqHqqWqqE256,
    // [EVEX.256.66.0F.WIG FD /r] VPADDW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpaddwVqqHqqWqqE256,
    // [EVEX.256.66.0F.W0 FE /r] VPADDD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpadddVqqHqqWqqE256,
    // [EVEX.256.66.0F.W1 D4 /r] VPADDQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpaddqVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG FC /r] VPADDB zmm1 {k1}{z}, zmm2, zmm3/m512
    VpaddbVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F.WIG FD /r] VPADDW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpaddwVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F.W0 FE /r] VPADDD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpadddVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F.W1 D4 /r] VPADDQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpaddqVdqqHdqqWdqqE512,

    // [NP 0F EC /r] PADDSB mm1, mm2/m64
    PaddsbPqQq,
    // [66 0F EC /r] PADDSB xmm1, xmm2/m128
    PaddsbVdqWdq,
    // [NP 0F ED /r] PADDSW mm1, mm2/m64
    PaddswPqQq,
    // [66 0F ED /r] PADDSW xmm1, xmm2/m128
    PaddswVdqWdq,
    // [VEX.128.66.0F.WIG EC /r] VPADDSB xmm1, xmm2, xmm3/m128
    VpaddsbVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG ED /r] VPADDSW xmm1, xmm2, xmm3/m128
    VpaddswVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG EC /r] VPADDSB ymm1, ymm2, ymm3/m256
    VpaddsbVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG ED /r] VPADDSW ymm1, ymm2, ymm3/m256
    VpaddswVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG EC /r] VPADDSB xmm1 {k1}{z}, xmm2, xmm3/m128
    VpaddsbVdqHdqWdqE128,
    // [EVEX.128.66.0F.WIG ED /r] VPADDSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpaddswVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG EC /r] VPADDSB ymm1 {k1}{z}, ymm2, ymm3/m256
    VpaddsbVqqHqqWqqE256,
    // [EVEX.256.66.0F.WIG ED /r] VPADDSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpaddswVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG EC /r] VPADDSB zmm1 {k1}{z}, zmm2, zmm3/m512
    VpaddsbVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F.WIG ED /r] VPADDSW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpaddswVdqqHdqqWdqqE512,

    // [NP 0F DC /r] PADDUSB mm1, mm2/m64
    PaddusbPqQq,
    // [66 0F DC /r] PADDUSB xmm1, xmm2/m128
    PaddusbVdqWdq,
    // [NP 0F DD /r] PADDUSW mm1, mm2/m64
    PadduswPqQq,
    // [66 0F DD /r] PADDUSW xmm1, xmm2/m128
    PadduswVdqWdq,
    // [VEX.128.66.0F.WIG DC /r] VPADDUSB xmm1, xmm2, xmm3/m128
    VpaddusbVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG DD /r] VPADDUSW xmm1, xmm2, xmm3/m128
    VpadduswVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG DC /r] VPADDUSB ymm1, ymm2, ymm3/m256
    VpaddusbVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG DD /r] VPADDUSW ymm1, ymm2, ymm3/m256
    VpadduswVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG DC /r] VPADDUSB xmm1 {k1}{z}, xmm2, xmm3/m128
    VpaddusbVdqHdqWdqE128,
    // [EVEX.128.66.0F.WIG DD /r] VPADDUSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpadduswVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG DC /r] VPADDUSB ymm1 {k1}{z}, ymm2, ymm3/m256
    VpaddusbVqqHqqWqqE256,
    // [EVEX.256.66.0F.WIG DD /r] VPADDUSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpadduswVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG DC /r] VPADDUSB zmm1 {k1}{z}, zmm2, zmm3/m512
    VpaddusbVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F.WIG DD /r] VPADDUSW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpadduswVdqqHdqqWdqqE512,

    // [NP 0F 3A 0F /r ib] PALIGNR mm1, mm2/m64, imm8
    PalignrPqQqIb,
    // [66 0F 3A 0F /r ib] PALIGNR xmm1, xmm2/m128, imm8
    PalignrVdqWdqIb,
    // [VEX.128.66.0F3A.WIG 0F /r ib] VPALIGNR xmm1, xmm2, xmm3/m128, imm8
    VpalignrVdqHdqWdqIbV128,
    // [VEX.256.66.0F3A.WIG 0F /r ib] VPALIGNR ymm1, ymm2, ymm3/m256, imm8
    VpalignrVqqHqqWqqIbV256,
    // [EVEX.128.66.0F3A.WIG 0F /r ib] VPALIGNR xmm1 {k1}{z}, xmm2, xmm3/m128, imm8
    VpalignrVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.WIG 0F /r ib] VPALIGNR ymm1 {k1}{z}, ymm2, ymm3/m256, imm8
    VpalignrVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.WIG 0F /r ib] VPALIGNR zmm1 {k1}{z}, zmm2, zmm3/m512, imm8
    VpalignrVdqqHdqqWdqqIbE512,

    // [NP 0F DB /r] PAND mm1, mm2/m64
    PandPqQq,
    // [66 0F DB /r] PAND xmm1, xmm2/m128
    PandVdqWdq,
    // [VEX.128.66.0F.WIG DB /r] VPAND xmm1, xmm2, xmm3/m128
    PandVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG DB /r] VPAND ymm1, ymm2, ymm3/m256
    PandVqqHqqWqqV256,
    // [EVEX.128.66.0F.W0 DB /r] VPANDD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpanddVdqHdqWdqE128,
    // [EVEX.256.66.0F.W0 DB /r] VPANDD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpanddVqqHqqWqqE256,
    // [EVEX.512.66.0F.W0 DB /r] VPANDD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpanddVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F.W1 DB /r] VPANDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpandqVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 DB /r] VPANDQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpandqVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 DB /r] VPANDQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpandqVdqqHdqqWdqqE512,

    // [NP 0F DF /r] PANDN mm1, mm2/m64
    PandnPqQq,
    // [66 0F DF /r] PANDN xmm1, xmm2/m128
    PandnVdqWdq,
    // [VEX.128.66.0F.WIG DF /r] VPANDN xmm1, xmm2, xmm3/m128
    PandnVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG DF /r] VPANDN ymm1, ymm2, ymm3/m256
    PandnVqqHqqWqqV256,
    // [EVEX.128.66.0F.W0 DF /r] VPANDND xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpandndVdqHdqWdqE128,
    // [EVEX.256.66.0F.W0 DF /r] VPANDND ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpandndVqqHqqWqqE256,
    // [EVEX.512.66.0F.W0 DF /r] VPANDND zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpandndVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F.W1 DF /r] VPANDNQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpandnqVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 DF /r] VPANDNQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpandnqVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 DF /r] VPANDNQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpandnqVdqqHdqqWdqqE512,

    // [F3 90] PAUSE
    Pause,

    // [NP 0F E0 /r] PAVGB mm1, mm2/m64
    PavgbPqQq,
    // [66 0F E0 /r] PAVGB xmm1, xmm2/m128
    PavgbVdqWdq,
    // [NP 0F E3 /r] PAVGW mm1, mm2/m64
    PavgwPqQq,
    // [66 0F E3 /r] PAVGW xmm1, xmm2/m128
    PavgwVdqWdq,
    // [VEX.128.66.0F.WIG E0 /r] VPAVGB xmm1, xmm2, xmm3/m128
    VpavgbVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG E3 /r] VPAVGW xmm1, xmm2, xmm3/m128
    VpavgwVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG E0 /r] VPAVGB ymm1, ymm2, ymm3/m256
    VpavgbVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG E3 /r] VPAVGW ymm1, ymm2, ymm3/m256
    VpavgwVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG E0 /r] VPAVGB xmm1 {k1}{z}, xmm2, xmm3/m128
    VpavgbVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG E0 /r] VPAVGB ymm1 {k1}{z}, ymm2, ymm3/m256
    VpavgbVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG E0 /r] VPAVGB zmm1 {k1}{z}, zmm2, zmm3/m512
    VpavgbVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F.WIG E3 /r] VPAVGW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpavgwVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG E3 /r] VPAVGW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpavgwVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG E3 /r] VPAVGW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpavgwVdqqHdqqWdqqE512,

    // [66 0F 38 10 /r] PBLENDVB xmm1, xmm2/m128, <XMM0>
    PblendvbVdqWdq,
    // [VEX.128.66.0F3A.W0 4C /r /is4] VPBLENDVB xmm1, xmm2, xmm3/m128, xmm4
    VpblendvbVdqHdqWdqIbV128,
    // [VEX.256.66.0F3A.W0 4C /r /is4] VPBLENDVB ymm1, ymm2, ymm3/m256, ymm4
    VpblendvbVqqHqqWqqIbV256,

    // [66 0F 38 0E /r] PBLENDVW xmm1, xmm2/m128, imm8
    PblendvwVdqWdqIb,
    // [VEX.128.66.0F3A.W0 0E /r ib] VPBLENDVW xmm1, xmm2, xmm3/m128, imm8
    VpblendvwVdqHdqWdqIbV128,
    // [VEX.256.66.0F3A.W0 0E /r ib] VPBLENDVW ymm1, ymm2, ymm3/m256, imm8
    VpblendvwVqqHqqWqqIbV256,

    // [66 0F 3A 44 /r ib] PCLMULQDQ xmm1, xmm2/m128, imm8
    PclmulqdqVdqWdqIb,
    // [VEX.128.66.0F3A.WIG 44 /r ib] VPCLMULQDQ xmm1, xmm2, xmm3/m128, imm8
    VpclmulqdqVdqHdqWdqIbV128,
    // [VEX.256.66.0F3A.WIG 44 /r ib] VPCLMULQDQ ymm1, ymm2, ymm3/m256, imm8
    VpclmulqdqVqqHqqWqqIbV256,
    // [EVEX.128.66.0F3A.WIG 44 /r ib] VPCLMULQDQ xmm1, xmm2, xmm3/m128, imm8
    VpclmulqdqVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.WIG 44 /r ib] VPCLMULQDQ ymm1, ymm2, ymm3/m256, imm8
    VpclmulqdqVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.WIG 44 /r ib] VPCLMULQDQ zmm1, zmm2, zmm3/m512, imm8
    VpclmulqdqVdqqHdqqWdqqIbE512,

    // [NP 0F 74 /r] PCMPEQB mm1, mm2/m64
    PcmpeqbPqQq,
    // [66 0F 74 /r] PCMPEQB xmm1, xmm2/m128
    PcmpeqbVdqWdq,
    // [NP 0F 75 /r] PCMPEQW mm1, mm2/m64
    PcmpeqwPqQq,
    // [66 0F 75 /r] PCMPEQW xmm1, xmm2/m128
    PcmpeqwVdqWdq,
    // [NP 0F 76 /r] PCMPEQD mm1, mm2/m64
    PcmpeqdPqQq,
    // [66 0F 76 /r] PCMPEQD xmm1, xmm2/m128
    PcmpeqdVdqWdq,
    // [VEX.128.66.0F.WIG 74 /r] VCMPEQB xmm1, xmm2, xmm3/m128
    VcmpeqbVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 75 /r] VCMPEQW xmm1, xmm2, xmm3/m128
    VcmpeqwVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 76 /r] VCMPEQD xmm1, xmm2, xmm3/m128
    VcmpeqdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 74 /r] VCMPEQB ymm1, ymm2, ymm3/m256
    VcmpeqbVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG 75 /r] VCMPEQW ymm1, ymm2, ymm3/m256
    VcmpeqwVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG 76 /r] VCMPEQD ymm1, ymm2, ymm3/m256
    VcmpeqdVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG 74 /r] VCMPEQB k1 {k2}, xmm2, xmm3/m128
    VpcmpeqbKGqHdqWdqE128,
    // [EVEX.256.66.0F.WIG 74 /r] VCMPEQB k1 {k2}, ymm2, ymm3/m256
    VpcmpeqbKGqHqqWqqE256,
    // [EVEX.512.66.0F.WIG 74 /r] VCMPEQB k1 {k2}, zmm2, zmm3/m512
    VpcmpeqbKGqHdqqWdqqE512,
    // [EVEX.128.66.0F.WIG 75 /r] VCMPEQW k1 {k2}, xmm2, xmm3/m128
    VpcmpeqwKGqHdqWdqE128,
    // [EVEX.256.66.0F.WIG 75 /r] VCMPEQW k1 {k2}, ymm2, ymm3/m256
    VpcmpeqwKGqHqqWqqE256,
    // [EVEX.512.66.0F.WIG 75 /r] VCMPEQW k1 {k2}, zmm2, zmm3/m512
    VpcmpeqwKGqHdqqWdqqE512,
    // [EVEX.128.66.0F.WIG 76 /r] VCMPEQD k1 {k2}, xmm2, xmm3/m128/m32bcst
    VpcmpeqdKGqHdqWdqE128,
    // [EVEX.256.66.0F.WIG 76 /r] VCMPEQD k1 {k2}, ymm2, ymm3/m256/m32bcst
    VpcmpeqdKGqHqqWqqE256,
    // [EVEX.512.66.0F.WIG 76 /r] VCMPEQD k1 {k2}, zmm2, zmm3/m512/m32bcst
    VpcmpeqdKGqHdqqWdqqE512,

    // [66 0F 38 29 /r] PCMPEQQ xmm1, xmm2/m128
    PcmpeqqVdqWdq,
    // [VEX.128.66.0F38.WIG 29 /r] VPCMPEQQ xmm1, xmm2, xmm3/m128
    VpcmpeqqVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 29 /r] VPCMPEQQ ymm1, ymm2, ymm3/m256
    VpcmpeqqVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W1 29 /r] VPCMPEQQ k1 {k2}, xmm2, xmm3/m128/m64bcst
    VpcmpeqqKgqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 29 /r] VPCMPEQQ k1 {k2}, ymm2, ymm3/m256/m64bcst
    VpcmpeqqKgqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 29 /r] VPCMPEQQ k1 {k2}, zmm2, zmm3/m512/m64bcst
    VpcmpeqqKgqHdqqWdqqE512,

    // [66 0F 3A 61 /r ib] PCMPESTRI xmm1, xmm2/m128, imm8
    PcmpestriVdqWdqIb,
    // [VEX.128.66.0F3A 61 /r ib] VPCMPESTRI xmm1, xmm2/m128, imm8
    VpcmpestriVdqHdqWdqIbV128,

    // [66 0F 3A 60 /r ib] PCMPESTRM xmm1, xmm2/m128, imm8
    PcmpestrmVdqWdqIb,
    // [VEX.128.66.0F3A 60 /r ib] VPCMPESTRM xmm1, xmm2/m128, imm8
    VpcmpestrmVdqHdqWdqIbV128,

    // [NP 0F 64 /r] PCMPGTB mm1, mm2/m64
    PcmpgtbPqQq,
    // [66 0F 64 /r] PCMPGTB xmm1, xmm2/m128
    PcmpgtbVdqWdq,
    // [NP 0F 65 /r] PCMPGTB mm1, mm2/m64
    PcmpgtwPqQq,
    // [66 0F 65 /r] PCMPGTB xmm1, xmm2/m128
    PcmpgtwVdqWdq,
    // [NP 0F 66 /r] PCMPGTB mm1, mm2/m64
    PcmpgtdPqQq,
    // [66 0F 66 /r] PCMPGTB xmm1, xmm2/m128
    PcmpgtdVdqWdq,
    // [VEX.128.66.0F.WIG 64 /r] VCMPGTB xmm1, xmm2, xmm3/m128
    VpcmpgtbVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 65 /r] VCMPGTW xmm1, xmm2, xmm3/m128
    VpcmpgtwVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 66 /r] VCMPGTD xmm1, xmm2, xmm3/m128
    VpcmpgtdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 64 /r] VCMPGTB ymm1, ymm2, ymm3/m256
    VpcmpgtbVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG 65 /r] VCMPGTW ymm1, ymm2, ymm3/m256
    VpcmpgtwVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG 66 /r] VCMPGTD ymm1, ymm2, ymm3/m256
    VpcmpgtdVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG 64 /r] VCMPPGTB k1 {k2}, xmm2, xmm3/m128
    VpcmpgtbKGqHdqWdqE128,
    // [EVEX.256.66.0F.WIG 64 /r] VCMPPGTB k1 {k2}, ymm2, ymm3/m256
    VpcmpgtbKGqHqqWqqE256,
    // [EVEX.512.66.0F.WIG 64 /r] VCMPPGTB k1 {k2}, zmm2, zmm3/m512
    VpcmpgtbKGqHdqqWdqqE512,
    // [EVEX.128.66.0F.WIG 65 /r] VCMPPGTW k1 {k2}, xmm2, xmm3/m128
    VpcmpgtwKGqHdqWdqE128,
    // [EVEX.256.66.0F.WIG 65 /r] VCMPPGTW k1 {k2}, ymm2, ymm3/m256
    VpcmpgtwKGqHqqWqqE256,
    // [EVEX.512.66.0F.WIG 65 /r] VCMPPGTW k1 {k2}, zmm2, zmm3/m512
    VpcmpgtwKGqHdqqWdqqE512,
    // [EVEX.128.66.0F.WIG 66 /r] VCMPPGTD k1 {k2}, xmm2, xmm3/m128/m32bcst
    VpcmpgtdKGqHdqWdqE128,
    // [EVEX.256.66.0F.WIG 66 /r] VCMPPGTD k1 {k2}, ymm2, ymm3/m256/m32bcst
    VpcmpgtdKGqHqqWqqE256,
    // [EVEX.512.66.0F.WIG 66 /r] VCMPPGTD k1 {k2}, zmm2, zmm3/m512/m32bcst
    VpcmpgtdKGqHdqqWdqqE512,

    // [66 0F 38 37 /r] PCMPGTQ xmm1, xmm2/m128
    PcmpgtqVdqWdq,
    // [VEX.128.66.0F38.WIG 37 /r] VCMPGTQ xmm1, xmm2, xmm3/m128
    VpcmpgtqVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 37 /r] VCMPGTQ ymm1, ymm2, ymm3/m256
    VpcmpgtqVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W1 37 /r] VCMPGTQ k1 {k2}, xmm2, xmm3/m128/m64bcst
    VpcmpgtqVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 37 /r] VCMPGTQ k1 {k2}, ymm2, ymm3/m256/m64bcst
    VpcmpgtqVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 37 /r] VCMPGTQ k1 {k2}, zmm2, zmm3/m512/m64bcst
    VpcmpgtqVdqqHdqqWdqqE512,

    // [66 0F 3A 63 /r ib] PCMPISTRI xmm1, xmm2/m128, imm8
    PcmpistriVdqWdqIb,
    // [VEX.128.66.0F3A.WIG 63 /r ib] VPCMPISTRI xmm1, xmm2/m128, imm8
    VpcmpistriVdqHdqWdqIb,

    // [66 0F 3A 62 /r ib] PCMPISTRM xmm1, xmm2/m128, imm8
    PcmpistrmVdqWdqIb,
    // [VEX.128.66.0F3A.WIG 62 /r ib] VPCMPISTRM xmm1, xmm2/m128, imm8
    VpcmpistrmVdqHdqWdqIb,

    // [NP 0F 01 C5] PCONFIG
    Pconfig,

    // [VEX.LZ.F2.0F38.W0 F5 /r] PDEP r32a, r32b, r/m32
    PdepGdBdEd,
    // [VEX.LZ.F2.0F38.W1 F5 /r] PDEP r64a, r64b, r/m64
    PdepGqBqEq,

    // [VEX.LZ.F3.0F38.W0 F5 /r] PEXT r32a, r32b, r/m32
    PextGdBdEd,
    // [VEX.LZ.F3.0F38.W1 F5 /r] PEXT r64a, r64b, r/m64
    PextGqBqEq,

    // [66 0F 3A 14 /r ib] PEXTRB r/m8, xmm1, imm8
    PextrbEdVdqIb,
    // [66 0F 3A 16 /r ib] PEXTRD r/m32, xmm1, imm8
    PextrdEdVdqIb,
    // [66 REX.W 0F 3A 16 /r ib] PEXTRQ r/m64, xmm1, imm8
    PextrqEqVdqIb,
    // [VEX.128.66.0F3A.W0 14 /r ib] VPEXTRB r/m8, xmm1, imm8
    VpextrbEbVdqIbV128,
    // [VEX.128.66.0F3A.W0 16 /r ib] VPEXTRD r/m32, xmm1, imm8
    VpextrdEdVdqIbV128,
    // [VEX.128.66.0F3A.W1 16 /r ib] VPEXTRQ r/m64, xmm1, imm8
    VpextrqEqVdqIbV128,
    // [EVEX.128.66.0F3A.WIG 14 /r ib] VPEXTRB r/m8, xmm1, imm8
    VpextrbEbVdqIbE128,
    // [EVEX.128.66.0F3A.W0 16 /r ib] VPEXTRD r/m32, xmm1, imm8
    VpextrdEdVdqIbE128,
    // [EVEX.128.66.0F3A.W1 16 /r ib] VPEXTRQ r/m64, xmm1, imm8
    VpextrqEqVdqIbE128,

    // [NP 0F C5 /r ib] PEXTRW reg, mm1, imm8
    PextrwGwNqIb,
    // [66 0F C5 /r ib] PEXTRW reg, xmm1, imm8
    PextrwGwUdqIb,
    // [66 0F 3A 15 /r ib] PEXTRW r/m16, xmm1, imm8
    PextrwEwVdqIb,
    // [VEX.128.66.0F.W0 C5 /r ib] VPEXTRW reg, xmm1, imm8
    VpextrwGwUdqIbV128,
    // [VEX.128.66.0F3A.W0 15 /r ib] VPEXTRW r/m16, xmm1, imm8
    VpextrwEwVdqIbV128,
    // [EVEX.128.66.0F.WIG C5 /r ib] VPEXTRW reg, xmm1, imm8
    VpextrwGwUdqIbE128,
    // [EVEX.128.66.0F3A.WIG 15 /r ib] VPEXTRW r/m16, xmm1, imm8
    VpextrwEwVdqIbE128,

    // [NP 0F 38 01 /r] PHADDW mm1, mm2/m64
    PhaddwPqQq,
    // [66 0F 38 01 /r] PHADDW xmm1, xmm2/m128
    PhaddwVdqWdq,
    // [NP 0F 38 02 /r] PHADDD mm1, mm2/m64
    PhadddPqQq,
    // [66 0F 38 02 /r] PHADDD xmm1, xmm2/m128
    PhadddVdqWdq,
    // [VEX.128.66.0F38.WIG 01 /r] VPHADDW xmm1, xmm2, xmm3/m128
    VphaddwVdqHdqWdqV128,
    // [VEX.128.66.0F38.WIG 02 /r] VPHADDD xmm1, xmm2, xmm3/m128
    VphadddVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 01 /r] VPHADDW ymm1, ymm2, ymm3/m256
    VphaddwVqqHqqWqqV256,
    // [VEX.256.66.0F38.WIG 02 /r] VPHADDD ymm1, ymm2, ymm3/m256
    VphadddVqqHqqWqqV256,

    // [NP 0F 38 03 /r] PHADDSW mm1, mm2/m64
    PhaddswPqQq,
    // [66 0F 38 03 /r] PHADDSW xmm1, xmm2/m128
    PhaddswVdqWdq,
    // [VEX.128.66.0F38.WIG 03 /r] VPHADDSW xmm1, xmm2, xmm3/m128
    VphaddswVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 03 /r] VPHADDSW ymm1, ymm2, ymm3/m256
    VphaddswVqqHqqWqqV256,

    // [66 0F 38 41 /r] PHMINPOSUW xmm1, xmm2/m128
    PhminposuwVdqWdq,
    // [VEX.128.66.0F38.WIG 41 /r] VPHMINPOSUW xmm1, xmm2/m128
    VphminposuwVdqWdqV128,

    // [NP 0F 38 05 /r] PHSUBW mm1, mm2/m64
    PhsubwPqQq,
    // [66 0F 38 05 /r] PHSUBW xmm1, xmm2/m128
    PhsubwVdqWdq,
    // [NP 0F 38 06 /r] PHSUBD mm1, mm2/m64
    PhsubdPqQq,
    // [66 0F 38 06 /r] PHSUBD xmm1, xmm2/m128
    PhsubdVdqWdq,
    // [VEX.128.66.0F38.WIG 05 /r] VPHSUBW xmm1, xmm2, xmm3/m128
    VphsubwVdqHdqWdqV128,
    // [VEX.128.66.0F38.WIG 06 /r] VPHSUBD xmm1, xmm2, xmm3/m128
    VphsubdVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 05 /r] VPHSUBW ymm1, ymm2, ymm3/m256
    VphsubwVqqHqqWqqV256,
    // [VEX.256.66.0F38.WIG 06 /r] VPHSUBD ymm1, ymm2, ymm3/m256
    VphsubdVqqHqqWqqV256,

    // [NP 0F 38 07 /r] PHSUBSW mm1, mm2/m64
    PhsubswPqQq,
    // [66 0F 38 07 /r] PHSUBSW xmm1, xmm2/m128
    PhsubswVdqWdq,
    // [VEX.128.66.0F38.WIG 07 /r] VPHSUBSW xmm1, xmm2, xmm3/m128
    VphsubswVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 07 /r] VPHSUBSW ymm1, ymm2, ymm3/m256
    VphsubswVqqHqqWqqV256,

    // [66 0F 3A 20 /r ib] PINSRB xmm1, r32/m8, imm8
    PinsrbVdqEbIb,
    // [66 0F 3A 22 /r ib] PINSRD xmm1, r/m32, imm8
    PinsrdVdqEdIb,
    // [66 REX.W 0F 3A 22 /r ib] PINSRQ xmm1, r/m64, imm8
    PinsrqVdqEqIb,
    // [VEX.128.66.0F3A.W0 20 /r ib] VPINSRB xmm1, xmm2, r32/m8, imm8
    VpinsrbVdqHdqEbIbV128,
    // [VEX.128.66.0F3A.W0 22 /r ib] VPINSRD xmm1, xmm2, r/m32, imm8
    VpinsrdVdqHdqEdIbV128,
    // [VEX.128.66.0F3A.W1 22 /r ib] VPINSRQ xmm1, xmm2, r/m64, imm8
    VpinsrqVdqHdqEqIbV128,
    // [EVEX.128.66.0F3A.WIG 20 /r ib] VPINSRB xmm1, xmm2, r32/m8, imm8
    VpinsrbVdqHdqEbIbE128,
    // [EVEX.128.66.0F3A.W0 22 /r ib] VPINSRD xmm1, xmm2, r/m32, imm8
    VpinsrdVdqHdqEdIbE128,
    // [EVEX.128.66.0F3A.W1 22 /r ib] VPINSRQ xmm1, xmm2, r/m64, imm8
    VpinsrqVdqHdqEqIbE128,

    // [NP 0F C4 /r ib] PINSRW mm1, r32/m16, imm8
    PinsrwPqEwIb,
    // [66 0F C4 /r ib] PINSRW xmm1, r32/m16, imm8
    PinsrwVdqEwIb,
    // [VEX.128.66.0F.W0 C4 /r ib] VPINSRW xmm1, xmm2, r32/m16, imm8
    VpinsrwVdqHdqEwIbV128,
    // [EVEX.128.66.0F.WIG C4 /r ib] VPINSRW xmm1, xmm2, r32/m16, imm8
    VpinsrwVdqHdqEWIbE128,

    // [NP 0F 38 04 /r] PMADDUBSW mm1, mm2/m64
    PmaddubswPqQq,
    // [66 0F 38 04 /r] PMADDUBSW xmm1, xmm2/m128
    PmaddubswVdqWdq,
    // [VEX.128.66.0F38.WIG 04 /r] VPMADDUBSW xmm1, xmm2, xmm3/m128
    VpmaddubswVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 04 /r] VPMADDUBSW ymm1, ymm2, ymm3/m256
    VpmaddubswVqqHqqWqqV256,
    // [EVEX.128.66.0F38.WIG 04 /r] VPMADDUBSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpmaddubswVdqHdqWdqE128,
    // [EVEX.256.66.0F38.WIG 04 /r] VPMADDUBSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpmaddubswVqqHqqWqqE256,
    // [EVEX.512.66.0F38.WIG 04 /r] VPMADDUBSW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpmaddubswVdqqHdqqWdqqE512,

    // [NP 0F F5 /r] PMADDWD mm1, mm2/m64
    PmaddwdPqQq,
    // [66 0F F5 /r] PMADDWD xmm1, xmm2/m128
    PmaddwdVdqWdq,
    // [VEX.128.66.0F.WIG F5 /r] VPMADDWD xmm1, xmm2, xmm3/m128
    VpmaddwdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG F5 /r] VPMADDWD ymm1, ymm2, ymm3/m256
    VpmaddwdVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG F5 /r] VPMADDWD xmm1 {k1}{z}, xmm2, xmm3/m128
    VpmaddwdVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG F5 /r] VPMADDWD ymm1 {k1}{z}, ymm2, ymm3/m256
    VpmaddwdVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG F5 /r] VPMADDWD zmm1 {k1}{z}, zmm2, zmm3/m512
    VpmaddwdVdqqHdqqWdqqE512,

    // [NP 0F EE /r] PMAXSW mm1, mm2/m64
    PmaxswPqQq,
    // [66 0F 38 3C /r] PMAXSB xmm1, xmm2/m128
    PmaxsbVdqWdq,
    // [66 0F EE /r] PMAXSW xmm1, xmm2/m128
    PmaxswVdqWdq,
    // [66 0F 38 3D /r] PMAXSD xmm1, xmm2/m128
    PmaxsdVdqWdq,
    // [VEX.128.66.0F38.WIG 3C /r] VPMAXSB xmm1, xmm2, xmm3/m128
    VpmaxsbVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG EE /r] VPMAXSW xmm1, xmm2, xmm3/m128
    VpmaxswVdqHdqWdqV128,
    // [VEX.128.66.0F38.WIG 3D /r] VPMAXSD xmm1, xmm2, xmm3/m128
    VpmaxsdVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 3C /r] VPMAXSB ymm1, ymm2, ymm3/m256
    VpmaxsbVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG EE /r] VPMAXSW ymm1, ymm2, ymm3/m256
    VpmaxswVqqHqqWqqV256,
    // [VEX.256.66.0F38.WIG 3D /r] VPMAXSD ymm1, ymm2, ymm3/m256
    VpmaxsdVqqHqqWqqV256,
    // [EVEX.128.66.0F38.WIG 3C /r] VPMAXSB xmm1 {k1}{z}, xmm2, xmm3/m128
    VpmaxsbVdqHdqWdqE128,
    // [EVEX.256.66.0F38.WIG 3C /r] VPMAXSB ymm1 {k1}{z}, ymm2, ymm3/m256
    VpmaxsbVqqHqqWqqE256,
    // [EVEX.512.66.0F38.WIG 3C /r] VPMAXSB zmm1 {k1}{z}, zmm2, zmm3/m512
    VpmaxsbVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F.WIG EE /r] VPMAXSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpmaxswVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG EE /r] VPMAXSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpmaxswVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG EE /r] VPMAXSW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpmaxswVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W0 3D /r] VPMAXSD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpmaxsdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 3D /r] VPMAXSD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpmaxsdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 3D /r] VPMAXSD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpmaxsdVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 3D /r] VPMAXSQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpmaxsqVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 3D /r] VPMAXSQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpmaxsqVdqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 3D /r] VPMAXSQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpmaxsqVdqqHdqqWdqqE512,

    // [NP 0F DE /r] PMAXUB mm1, mm2/m64
    PmaxubPqQq,
    // [66 0F DE /r] PMAXUB xmm1, xmm2/m128
    PmaxubVdqWdq,
    // [66 0F 38 3E /r] PMAXUW xmm1, xmm2/m128
    PmaxuwVdqWdq,
    // [VEX.128.66.0F.WIG DE /r] VPMAXUB xmm1, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VpmaxubVdqHdqWdqV128,
    // [VEX.128.66.0F38.WIG 3E /r] VPMAXUW xmm1, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VpmaxuwVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG DE /r] VPMAXUB ymm1, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VpmaxubVqqHqqWqqV256,
    // [VEX.256.66.0F38.WIG 3E /r] VPMAXUW ymm1, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VpmaxuwVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG DE /r] VPMAXUB xmm1 {k1}{z}, xmm2, xmm3/m128
    VpmaxubVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG DE /r] VPMAXUB ymm1 {k1}{z}, ymm2, ymm3/m256
    VpmaxubVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG DE /r] VPMAXUB zmm1 {k1}{z}, zmm2, zmm3/m512
    VpmaxubVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.WIG 3E /r] VPMAXUW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpmaxuwVdqHdqWdqE128,
    // [EVEX.256.66.0F38.WIG 3E /r] VPMAXUW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpmaxuwVqqHqqWqqE256,
    // [EVEX.512.66.0F38.WIG 3E /r] VPMAXUW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpmaxuwVdqqHdqqWdqqE512,

    // [66 0F 38 3F /r] PMAXUD xmm1, xmm2/m128
    PmaxudVdqWdq,
    // [VEX.128.66.0F38.WIG 3F /r] VPMAXUD xmm1, xmm2, xmm3/m128
    VpmaxudVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 3F /r] VPMAXUD ymm1, ymm2, ymm3/m256
    VpmaxudVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W0 3F /r] VPMAXUD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpmaxudVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 3F /r] VPMAXUD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpmaxudVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 3F /r] VPMAXUD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpmaxudVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 3F /r] VPMAXUQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpmaxuqVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 3F /r] VPMAXUQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpmaxuqVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 3F /r] VPMAXUQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpmaxuqVdqqHdqqWdqqE512,

    // [NP 0F EA /r] PMINSW mm1, mm2/m64
    PminswPqQq,
    // [66 0F 38 38 /r] PMINSB xmm1, xmm2/m128
    PminsbVdqWdq,
    // [66 0F EA /r] PMINSW xmm1, xmm2/m128
    PminswVdqWdq,
    // [VEX.128.66.0F38.WIG 38 /r] VPMINSB xmm1, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VpminsbVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG EA /r] VPMINSW xmm1, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VpminswVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 38 /r] VPMINSB ymm1, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VpminsbVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG EA /r] VPMINSW ymm1, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VpminswVqqHqqWqqV256,
    // [EVEX.128.66.0F38.WIG 38 /r] VPMINSB xmm1 {k1}{z}, xmm2, xmm3/m128
    VpminsbVdqHdqWdqE128,
    // [EVEX.256.66.0F38.WIG 38 /r] VPMINSB ymm1 {k1}{z}, ymm2, ymm3/m256
    VpminsbVqqHqqWqqE256,
    // [EVEX.512.66.0F38.WIG 38 /r] VPMINSB zmm1 {k1}{z}, zmm2, zmm3/m512
    VpminsbVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F.WIG EA /r] VPMINSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpminswVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG EA /r] VPMINSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpminswVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG EA /r] VPMINSW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpminswVdqqHdqqWdqqE512,

    // [66 0F 38 39 /r] PMINSD xmm1, xmm2/m128
    PminsdVdqWdq,
    // [VEX.128.66.0F38.WIG 39 /r] VPMINSD xmm1, xmm2, xmm3/m128
    VpminsdVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 39 /r] VPMINSD ymm1, ymm2, ymm3/m256
    VpminsdVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W0 39 /r] VPMINSD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpminsdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 39 /r] VPMINSD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpminsdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 39 /r] VPMINSD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpminsdVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 39 /r] VPMINSQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpminsqVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 39 /r] VPMINSQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpminsqVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 39 /r] VPMINSQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpminsqVdqqHdqqWdqqE512,

    // [NP 0F DA /r] PMINUB mm1, mm2/m64
    PminubPqQq,
    // [66 0F DA /r] PMINUB xmm1, xmm2/m128
    PminubVdqWdq,
    // [66 0F 38 3A /r] PMINUW xmm1, xmm2/m128
    PminuwVdqWdq,
    // [VEX.128.66.0F.WIG DA /r] VPMINUB xmm1, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VpminubVdqHdqWdqV128,
    // [VEX.128.66.0F38.WIG 3A/r] VPMINUW xmm1, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VpminuwVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG DA /r] VPMINUB ymm1, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VpminubVdqHdqWdqV256,
    // [VEX.256.66.0F38.WIG 3A /r] VPMINUW ymm1, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VpminuwVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG DA /r] VPMINUB xmm1 {k1}{z}, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VpminubVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG DA /r] VPMINUB ymm1 {k1}{z}, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VpminubVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG DA /r] VPMINUB zmm1 {k1}{z}, zmm2, zmm3/m512
    // NOTE: Intel manual doesn't mention `WIG`
    VpminubVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.WIG 3A /r] VPMINUW xmm1 {k1}{z}, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VpminuwVdqHdqWdqE128,
    // [EVEX.256.66.0F38.WIG 3A /r] VPMINUW ymm1 {k1}{z}, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VpminuwVqqHqqWqqE256,
    // [EVEX.512.66.0F38.WIG 3A /r] VPMINUW zmm1 {k1}{z}, zmm2, zmm3/m512
    // NOTE: Intel manual doesn't mention `WIG`
    VpminuwVdqqHdqqWdqqE512,

    // [66 0F 38 3B /r] PMINUD xmm1, xmm2/m128
    PminudVdqWdq,
    // [VEX.128.66.0F38.WIG 3B /r] VPMINUD xmm1, xmm2, xmm3/m128
    VpminudVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 3B /r] VPMINUD ymm1, ymm2, ymm3/m256
    VpminudVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W0 3B /r] VPMINUD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpminudVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 3B /r] VPMINUD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpminudVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 3B /r] VPMINUD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpminudVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 3B /r] VPMINUQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpminuqVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 3B /r] VPMINUQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpminuqVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 3B /r] VPMINUQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpminuqVdqqHdqqWdqqE512,

    // [NP 0F D7 /r] PMOVMSKB reg, mm1
    PmovmskbGdNq,
    // [66 0F D7 /r] PMOVMSKB reg, xmm1
    PmovmskbGdUdq,
    // [VEX.128.66.0F.WIG D7 /r] VPMOVMSKB reg, xmm1
    VpmovmskbGdUdqV128,
    // [VEX.256.66.0F.WIG D7 /r] VPMOVMSKB reg, ymm1
    VpmovmskbGdUqqV256,

    // [66 0F 38 20 /r] PMOVSXBW xmm1, xmm2/m64
    PmovsxbwVdqWq,
    // [66 0F 38 21 /r] PMOVSXBD xmm1, xmm2/m32
    PmovsxbdVdqWd,
    // [66 0F 38 22 /r] PMOVSXBQ xmm1, xmm2/m16
    PmovsxbqVdqWw,
    // [66 0F 38 23 /r] PMOVSXWD xmm1, xmm2/m64
    PmovsxwdVdqWq,
    // [66 0F 38 24 /r] PMOVSXWQ xmm1, xmm2/m32
    PmovsxwqVdqWd,
    // [66 0F 38 25 /r] PMOVSXDQ xmm1, xmm2/m64
    PmovsxdqVdqWq,
    // [VEX.128.66.0F38.WIG 20 /r] VPMOVSXBW xmm1, xmm2/m64
    VpmovsxbwVdqWqV128,
    // [VEX.128.66.0F38.WIG 21 /r] VPMOVSXBD xmm1, xmm2/m32
    VpmovsxbdVdqWdV128,
    // [VEX.128.66.0F38.WIG 22 /r] VPMOVSXBQ xmm1, xmm2/m16
    VpmovsxbqVdqWwV128,
    // [VEX.128.66.0F38.WIG 23 /r] VPMOVSXWD xmm1, xmm2/m64
    VpmovsxwdVdqWqV128,
    // [VEX.128.66.0F38.WIG 24 /r] VPMOVSXWQ xmm1, xmm2/m32
    VpmovsxwqVdqWdV128,
    // [VEX.128.66.0F38.WIG 25 /r] VPMOVSXDQ xmm1, xmm2/m64
    VpmovsxdqVdqWqV128,
    // [VEX.256.66.0F38.WIG 20 /r] VPMOVSXBW ymm1, xmm2/m128
    VpmovsxbwVqqWdqV256,
    // [VEX.256.66.0F38.WIG 21 /r] VPMOVSXBD ymm1, ymm2/m64
    VpmovsxbdVqqWqV256,
    // [VEX.256.66.0F38.WIG 22 /r] VPMOVSXBQ ymm1, ymm2/m32
    VpmovsxbqVqqWdV256,
    // [VEX.256.66.0F38.WIG 23 /r] VPMOVSXWD ymm1, ymm2/m128
    VpmovsxwdVqqWdqV256,
    // [VEX.256.66.0F38.WIG 24 /r] VPMOVSXWQ ymm1, ymm2/m64
    VpmovsxwqVqqWqV256,
    // [VEX.256.66.0F38.WIG 25 /r] VPMOVSXDQ ymm1, ymm2/m128
    VpmovsxdqVqqWdqV256,
    // [EVEX.128.66.0F38.WIG 20 /r] VPMOVSXBW xmm1 {k1}{z}, xmm2/m64
    VpmovsxbwVdqWqE128,
    // [EVEX.256.66.0F38.WIG 20 /r] VPMOVSXBW ymm1 {k1}{z}, xmm2/m128
    VpmovsxbwVqqWdqE256,
    // [EVEX.512.66.0F38.WIG 20 /r] VPMOVSXBW zmm1 {k1}{z}, ymm2/m256
    VpmovsxbwVdqqWqqE512,
    // [EVEX.128.66.0F38.WIG 21 /r] VPMOVSXBD xmm1 {k1}{z}, xmm2/m32
    VpmovsxbdVdqWdE128,
    // [EVEX.256.66.0F38.WIG 21 /r] VPMOVSXBD ymm1 {k1}{z}, xmm2/m64
    VpmovsxbdVqqWqE256,
    // [EVEX.512.66.0F38.WIG 21 /r] VPMOVSXBD zmm1 {k1}{z}, xmm2/m128
    VpmovsxbdVdqqWdqE512,
    // [EVEX.128.66.0F38.WIG 22 /r] VPMOVSXBQ xmm1 {k1}{z}, xmm2/m16
    VpmovsxbqVdqWwE128,
    // [EVEX.256.66.0F38.WIG 22 /r] VPMOVSXBQ ymm1 {k1}{z}, xmm2/m32
    VpmovsxbqVqqWdE256,
    // [EVEX.512.66.0F38.WIG 22 /r] VPMOVSXBQ zmm1 {k1}{z}, xmm2/m64
    VpmovsxbqVdqqWqE512,
    // [EVEX.128.66.0F38.WIG 23 /r] VPMOVSXWD xmm1 {k1}{z}, xmm2/m64
    VpmovsxwdVdqWqE128,
    // [EVEX.256.66.0F38.WIG 23 /r] VPMOVSXWD ymm1 {k1}{z}, xmm2/m128
    VpmovsxwdVqqWdqE256,
    // [EVEX.512.66.0F38.WIG 23 /r] VPMOVSXWD zmm1 {k1}{z}, ymm2/m256
    VpmovsxwdVdqqWqqE512,
    // [EVEX.128.66.0F38.WIG 24 /r] VPMOVSXWQ xmm1 {k1}{z}, xmm2/m32
    VpmovsxwqVdqWdE128,
    // [EVEX.256.66.0F38.WIG 24 /r] VPMOVSXWQ ymm1 {k1}{z}, xmm2/m64
    VpmovsxwqVqqWqE256,
    // [EVEX.512.66.0F38.WIG 24 /r] VPMOVSXWQ zmm1 {k1}{z}, xmm2/m128
    VpmovsxwqVdqqWdqE512,
    // [EVEX.128.66.0F38.W0 25 /r] VPMOVSXDQ xmm1 {k1}{z}, xmm2/m64
    VpmovsxdqVdqWqE128,
    // [EVEX.256.66.0F38.W0 25 /r] VPMOVSXDQ ymm1 {k1}{z}, xmm2/m128
    VpmovsxdqVqqWdqE256,
    // [EVEX.512.66.0F38.W0 25 /r] VPMOVSXDQ zmm1 {k1}{z}, ymm2/m256
    VpmovsxdqVdqqWqqE512,

    // [66 0F 38 30 /r] PMOVZXBW xmm1, xmm2/m64
    PmovzxbwVdqWq,
    // [66 0F 38 31 /r] PMOVZXBD xmm1, xmm2/m32
    PmovzxbdVdqWd,
    // [66 0F 38 32 /r] PMOVZXBQ xmm1, xmm2/m16
    PmovzxbqVdqWw,
    // [66 0F 38 33 /r] PMOVZXWD xmm1, xmm2/m64
    PmovzxwdVdqWq,
    // [66 0F 38 34 /r] PMOVZXWQ xmm1, xmm2/m32
    PmovzxwqVdqWd,
    // [66 0F 38 35 /r] PMOVZXDQ xmm1, xmm2/m64
    PmovzxdqVdqWq,
    // [VEX.128.66.0F38.WIG 30 /r] VPMOVZXBW xmm1, xmm2/m64
    VpmovzxbwVdqWqV128,
    // [VEX.128.66.0F38.WIG 31 /r] VPMOVZXBD xmm1, xmm2/m32
    VpmovzxbdVdqWdV128,
    // [VEX.128.66.0F38.WIG 32 /r] VPMOVZXBQ xmm1, xmm2/m16
    VpmovzxbqVdqWwV128,
    // [VEX.128.66.0F38.WIG 33 /r] VPMOVZXWD xmm1, xmm2/m64
    VpmovzxwdVdqWqV128,
    // [VEX.128.66.0F38.WIG 34 /r] VPMOVZXWQ xmm1, xmm2/m32
    VpmovzxwqVdqWdV128,
    // [VEX.128.66.0F38.WIG 35 /r] VPMOVZXDQ xmm1, xmm2/m64
    VpmovzxdqVdqWqV128,
    // [VEX.256.66.0F38.WIG 30 /r] VPMOVZXBW ymm1, ymm2/m128
    VpmovzxbwVqqWqqV256,
    // [VEX.256.66.0F38.WIG 31 /r] VPMOVZXBD ymm1, ymm2/m64
    VpmovzxbdVqqWqV256,
    // [VEX.256.66.0F38.WIG 32 /r] VPMOVZXBQ ymm1, ymm2/m32
    VpmovzxbqVqqWdV256,
    // [VEX.256.66.0F38.WIG 33 /r] VPMOVZXWD ymm1, ymm2/m128
    VpmovzxwdVqqWdqV256,
    // [VEX.256.66.0F38.WIG 34 /r] VPMOVZXWQ ymm1, ymm2/m64
    VpmovzxwqVqqWqV256,
    // [VEX.256.66.0F38.WIG 35 /r] VPMOVZXDQ ymm1, ymm2/m128
    VpmovzxdqVqqWdqV256,
    // [EVEX.128.66.0F38.WIG 30 /r] VPMOVZXBW xmm1 {k1}{z}, xmm2/m64
    VpmovzxbwVdqWqE128,
    // [EVEX.256.66.0F38.WIG 30 /r] VPMOVZXBW ymm1 {k1}{z}, xmm2/m128
    VpmovzxbwVdqWdqE256,
    // [EVEX.512.66.0F38.WIG 30 /r] VPMOVZXBW zmm1 {k1}{z}, ymm2/m256
    VpmovzxbwVdqqWqqE512,
    // [EVEX.128.66.0F38.WIG 31 /r] VPMOVZXBD xmm1 {k1}{z}, xmm2/m32
    VpmovzxbdVdqWdE128,
    // [EVEX.256.66.0F38.WIG 31 /r] VPMOVZXBD ymm1 {k1}{z}, xmm2/m64
    VpmovzxbdVqqWqE256,
    // [EVEX.512.66.0F38.WIG 31 /r] VPMOVZXBD zmm1 {k1}{z}, xmm2/m128
    VpmovzxbdVdqqWdqE512,
    // [EVEX.128.66.0F38.WIG 32 /r] VPMOVZXBQ xmm1 {k1}{z}, xmm2/m16
    VpmovzxbqVdqWwE128,
    // [EVEX.256.66.0F38.WIG 32 /r] VPMOVZXBQ ymm1 {k1}{z}, xmm2/m32
    VpmovzxbqVqqWdE256,
    // [EVEX.512.66.0F38.WIG 32 /r] VPMOVZXBQ zmm1 {k1}{z}, xmm2/m64
    VpmovzxbqVdqqWqE512,
    // [EVEX.128.66.0F38.WIG 33 /r] VPMOVZXWD xmm1 {k1}{z}, xmm2/m64
    VpmovzxwdVdqWqE128,
    // [EVEX.256.66.0F38.WIG 33 /r] VPMOVZXWD ymm1 {k1}{z}, xmm2/m128
    VpmovzxwdVqqWdqE256,
    // [EVEX.512.66.0F38.WIG 33 /r] VPMOVZXWD zmm1 {k1}{z}, ymm2/m256
    VpmovzxwdVdqqWqqE512,
    // [EVEX.128.66.0F38.WIG 34 /r] VPMOVZXWQ xmm1 {k1}{z}, xmm2/m32
    VpmovzxwqVdqWdE128,
    // [EVEX.256.66.0F38.WIG 34 /r] VPMOVZXWQ ymm1 {k1}{z}, xmm2/m64
    VpmovzxwqVqqWqE256,
    // [EVEX.512.66.0F38.WIG 34 /r] VPMOVZXWQ zmm1 {k1}{z}, xmm2/m128
    VpmovzxwqVdqqWdqE512,
    // [EVEX.128.66.0F38.W0 35 /r] VPMOVZXDQ xmm1 {k1}{z}, xmm2/m64
    VpmovzxdqVdqWqE128,
    // [EVEX.256.66.0F38.W0 35 /r] VPMOVZXDQ ymm1 {k1}{z}, xmm2/m128
    VpmovzxdqVqqWdqE256,
    // [EVEX.512.66.0F38.W0 35 /r] VPMOVZXDQ zmm1 {k1}{z}, ymm2/m256
    VpmovzxdqVdqqWqqE512,

    // [66 0F 38 28 /r] PMULDQ xmm1, xmm2/m128
    PmuldqVdqWdq,
    // [VEX.128.66.0F38.WIG 28 /r] VPMULDQ xmm1, xmm2, xmm3/m128
    VpmuldqVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 28 /r] VPMULDQ ymm1, ymm2, ymm3/m256
    VpmuldqVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W1 28 /r] VPMULDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpmuldqVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 28 /r] VPMULDQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpmuldqVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 28 /r] VPMULDQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpmuldqVdqqHdqqWdqqE512,

    // [NP 0F 38 0B /r] PMULHRSW mm1, mm2/m64
    PmulhrswPqQq,
    // [66 0F 38 0B /r] PMULHRSW xmm1, xmm2/m128
    PmulhrswVdqWdq,
    // [VEX.128.66.0F38.WIG 0B /r] VPMULHRSW xmm1, xmm2, xmm3/m128
    VpmulhrswVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 0B /r] VPMULHRSW ymm1, ymm2, ymm3/m256
    VpmulhrswVqqHqqWqqV256,
    // [EVEX.128.66.0F38.WIG 0B /r] VPMULHRSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpmulhrswVdqHdqWdqE128,
    // [EVEX.256.66.0F38.WIG 0B /r] VPMULHRSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpmulhrswVqqHqqWqqE256,
    // [EVEX.512.66.0F38.WIG 0B /r] VPMULHRSW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpmulhrswVdqqHdqqWdqqE512,

    // [NP 0F E4 /r] PMULHUW mm1, mm2/m64
    PmulhuwPqQq,
    // [66 0F E4 /r] PMULHUW xmm1, xmm2/m128
    PmulhuwVdqWdq,
    // [VEX.128.66.0F.WIG E4 /r] VPMULHUW xmm1, xmm2, xmm3/m128
    VpmulhuwVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG E4 /r] VPMULHUW ymm1, ymm2, ymm3/m256
    VpmulhuwVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG E4 /r] VPMULHUW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpmulhuwVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG E4 /r] VPMULHUW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpmulhuwVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG E4 /r] VPMULHUW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpmulhuwVdqqHdqqWdqqE512,

    // [NP 0F E5 /r] PMULHW mm1, mm2/m64
    PmulhwPqQq,
    // [66 0F E5 /r] PMULHW xmm1, xmm2/m128
    PmulhwVdqWdq,
    // [VEX.128.66.0F.WIG E5 /r] VPMULHW xmm1, xmm2, xmm3/m128
    VpmulhwVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG E5 /r] VPMULHW ymm1, ymm2, ymm3/m256
    VpmulhwVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG E5 /r] VPMULHW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpmulhwVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG E5 /r] VPMULHW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpmulhwVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG E5 /r] VPMULHW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpmulhwVdqqHdqqWdqqE512,

    // [66 0F 38 40 /r] PMULLD xmm1, xmm2/m128
    PmulldVdqWdq,
    // [VEX.128.66.0F38.WIG 40 /r] VPMULLD xmm1, xmm2, xmm3/m128
    VpmulldVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 40 /r] VPMULLD ymm1, ymm2, ymm3/m256
    VpmulldVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W0 40 /r] VPMULLD xmm1, xmm2, xmm3/m128/m32bcst
    VpmulldVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 40 /r] VPMULLD ymm1, ymm2, ymm3/m256/m32bcst
    VpmulldVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 40 /r] VPMULLD zmm1, zmm2, zmm3/m512/m32bcst
    VpmulldVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 40 /r] VPMULLD xmm1, xmm2, xmm3/m128/m64bcst
    VpmullqVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 40 /r] VPMULLD ymm1, ymm2, ymm3/m256/m64bcst
    VpmullqVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 40 /r] VPMULLD zmm1, zmm2, zmm3/m512/m64bcst
    VpmullqVdqqHdqqWdqqE512,

    // [NP 0F D5 /r] PMULLW mm1, mm/m64
    PmullwPqQq,
    // [66 0F D5 /r] PMULLW xmm1, xmm2/m128
    PmullwVdqWdq,
    // [VEX.128.66.0F.WIG D5 /r] VPMULLW xmm1, xmm2, xmm3/m128
    VpmullwVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG D5 /r] VPMULLW ymm1, ymm2, ymm3/m256
    VpmullwVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG D5 /r] VPMULLW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpmullwVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG D5 /r] VPMULLW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpmullwVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG D5 /r] VPMULLW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpmullwVdqqHdqqWdqqE512,

    // [NP 0F F4 /r] PMULUDQ mm1, mm2/m64
    PmuludqPqQq,
    // [66 0F F4 /r] PMULUDQ xmm1, xmm2/m128
    PmuludqVdqWdq,
    // [VEX.128.66.0F.WIG F4 /r] VPMULUDQ xmm1, xmm2, xmm3/m128
    VpmuludqVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG F4 /r] VPMULUDQ ymm1, ymm2, ymm3/m256
    VpmuludqVqqHqqWqqV256,
    // [EVEX.128.66.0F.W1 F4 /r] VPMULUDQ xmm1, xmm2, xmm3/m128/m64bcst
    VpmuludqVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 F4 /r] VPMULUDQ ymm1, ymm2, ymm3/m256/m64bcst
    VpmuludqVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 F4 /r] VPMULUDQ zmm1, zmm2, zmm3/m512/m64bcst
    VpmuludqVdqqHdqqWdqqE512,

    // [8F /0] POP r/m16
    PopEw,
    // [8F /0] POP r/m32
    PopEd,
    // [8F /0] POP r/m64
    PopEq,
    // [58+rw] POP r16
    PopGw,
    // [58+rd] POP r32
    PopGd,
    // [REX.W 58+rq] POP r64
    // NOTE: Intel manual doesn't mention REX.W
    PopGq,
    // [0F] POP CS
    // NOTE: Not valid on 80186 or newer
    PopCS,
    // [1F] POP DS
    PopDS,
    // [17] POP SS
    PopSS,
    // [OF A1] POP FS
    PopFS,
    // [0F A9] POP GS
    PopGS,

    // [61] POPA
    Popa,
    // [61] POPAD
    Popad,

    // [F3 0F B8 /r] POPCNT r16, r/m16
    PopcntEwGw,
    // [F3 0F B8 /r] POPCNT r32, r/m32
    PopcntEdGd,
    // [F3 REX.W 0F B8 /r] POPCNT r64, r/m64
    PopcntEqGq,

    // [9D] POPF
    Popf,
    // [9D] POPFD
    Popfd,
    // [9D] POPFQ
    Popfq,

    // [NP 0F EB /r] POR mm1, mm2/m64
    PorPqQq,
    // [66 0F EB /r] POR xmm1, xmm2/m128
    PorVdqWdq,
    // [VEX.128.66.0F.WIG EB /r] VPOR xmm1, xmm2, xmm3/m128
    VporVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG EB /r] VPOR ymm1, ymm2, ymm3/m256
    VporVqqHqqWqqV256,
    // [EVEX.128.66.0F.W0 EB /r] VPORD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpordVdqHdqWdqE128,
    // [EVEX.256.66.0F.W0 EB /r] VPORD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpordVqqHqqWqqE256,
    // [EVEX.512.66.0F.W0 EB /r] VPORD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpordVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F.W1 EB /r] VPORQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VporqVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 EB /r] VPORQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VporqVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 EB /r] VPORQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VporqVdqqHdqqWdqqE512,

    // [0F 18 /1] PREFETCH0 m8
    Prefetch0Mb,
    // [0F 18 /2] PREFETCH1 m8
    Prefetch1Mb,
    // [0F 18 /3] PREFETCH2 m8
    Prefetch2Mb,
    // [0F 18 /0] PREFETCHNTA m8
    PrefetchntaMb,

    // [0F 0D /1] PREFETCHW m8
    PrefetchwMb,

    // [NP 0F F6 /r] PSADBW mm1, mm2/m64
    PsadbwPqQq,
    // [66 0F F6 /r] PSADBW xmm1, xmm2/m128
    PsadbwVdqWdq,
    // [VEX.128.66.0F.WIG F6 /r] VPSADBW xmm1, xmm2, xmm3/m128
    VpsadbwVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG F6 /r] VPSADBW ymm1, ymm2, ymm3/m256
    VpsadbwVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG F6 /r] VPSADBW xmm1, xmm2, xmm3/m128
    VpsadbwVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG F6 /r] VPSADBW ymm1, ymm2, ymm3/m256
    VpsadbwVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG F6 /r] VPSADBW zmm1, zmm2, zmm3/m512
    VpsadbwVdqqHdqqWdqqE512,

    // [NP 0F 38 00 /r] PSHUFB mm1, mm2/m64
    PshufbPqQq,
    // [66 0F 38 00 /r] PSHUFB xmm1, xmm2/m128
    PshufbVdqWdq,
    // [VEX.128.66.0F38.WIG 00 /r] VPSHUFB xmm1, xmm2, xmm3/m128
    VpshufbVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 00 /r] VPSHUFB ymm1, ymm2, ymm3/m256
    VpshufbVqqHqqWqqV256,
    // [EVEX.128.66.0F38.WIG 00 /r] VPSHUFB xmm1 {k1}{z}, xmm2, xmm3/m128
    VpshufbVdqHdqWdqE128,
    // [EVEX.256.66.0F38.WIG 00 /r] VPSHUFB xmm1 {k1}{z}, xmm2, xmm3/m256
    VpshufbVqqHqqWqqE256,
    // [EVEX.512.66.0F38.WIG 00 /r] VPSHUFB xmm1 {k1}{z}, xmm2, xmm3/m512
    VpshufbVdqqHdqqWdqqE512,

    // [66 0F 70 /r ib] PSHUFD xmm1, xmm2/m128, imm8
    PshufdVdqWdqIb,
    // [VEX.128.66.0F.WIG 70 /r ib] VPSHUFD xmm1, xmm2/m128, imm8
    VpshufdVdqWdqIbV128,
    // [VEX.256.66.0F.WIG 70 /r ib] VPSHUFD ymm1, ymm2/m256, imm8
    VpshufdVqqWqqIbV256,
    // [EVEX.128.66.0F.WIG 70 /r ib] VPSHUFD xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VpshufdVdqWdqIbE128,
    // [EVEX.256.66.0F.WIG 70 /r ib] VPSHUFD ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VpshufdVqqWqqIbE256,
    // [EVEX.512.66.0F.WIG 70 /r ib] VPSHUFD zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VpshufdVdqqWdqqIbE512,

    // [F3 0F 70 /r ib] PSHUFHW xmm1, xmm2/m128, imm8
    PshufhwVdqWdqIb,
    // [VEX.128.F3.0F.WIG 70 /r ib] VPSHUFHW xmm1, xmm2/m128, imm8
    VpshufhwVdqWdqIbV128,
    // [VEX.256.F3.0F.WIG 70 /r ib] VPSHUFHW ymm1, ymm2/m256, imm8
    VpshufhwVqqWqqIbV256,
    // [EVEX.128.F3.0F.WIG 70 /r ib] VPSHUFHW xmm1 {k1}{z}, xmm2/m128, imm8
    VpshufhwVdqWdqIbE128,
    // [EVEX.256.F3.0F.WIG 70 /r ib] VPSHUFHW ymm1 {k1}{z}, ymm2/m256, imm8
    VpshufhwVqqWqqIbE256,
    // [EVEX.512.F3.0F.WIG 70 /r ib] VPSHUFHW zmm1 {k1}{z}, zmm2/m512, imm8
    VpshufhwVdqqWdqqIbE512,

    // [F2 0F 70 /r ib] PSHUFLW xmm1, xmm2/m128, imm8
    PshuflwVdqWdqIb,
    // [VEX.128.F2.0F.WIG 70 /r ib] VPSHUFLW xmm1, xmm2/m128, imm8
    VpshuflwVdqWdqIbV128,
    // [VEX.256.F2.0F.WIG 70 /r ib] VPSHUFLW ymm1, ymm2/m256, imm8
    VpshuflwVqqWqqIbV256,
    // [EVEX.128.F2.0F.WIG 70 /r ib] VPSHUFLW xmm1 {k1}{z}, xmm2/m128, imm8
    VpshuflwVdqWdqIbE128,
    // [EVEX.256.F2.0F.WIG 70 /r ib] VPSHUFLW ymm1 {k1}{z}, ymm2/m256, imm8
    VpshuflwVqqWqqIbE256,
    // [EVEX.512.F2.0F.WIG 70 /r ib] VPSHUFLW zmm1 {k1}{z}, zmm2/m512, imm8
    VpshuflwVdqqWdqqIbE512,

    // [NP 0F 70 /r ib] PSHUFW mm1, mm2/m64, imm8
    PshufwPqQqIb,

    // [NP 0F 38 08 /r] PSIGNB mm1, mm2/m64
    PsignbPqQq,
    // [66 0F 38 08 /r] PSIGNB xmm1, xmm2/m128
    PsignbVdqWdq,
    // [NP 0F 38 09 /r] PSIGNW mm1, mm2/m64
    PsignwPqQq,
    // [66 0F 38 09 /r] PSIGNW xmm1, xmm2/m128
    PsignwVdqWdq,
    // [NP 0F 38 0A /r] PSIGND mm1, mm2/m64
    PsigndPqQq,
    // [66 0F 38 0A /r] PSIGND xmm1, xmm2/m128
    PsigndVdqWdq,
    // [VEX.128.66.0F38.WIG 08 /r] VPSIGNB xmm1, xmm2, xmm3/m128
    VpsignbVdqHdqWdqV128,
    // [VEX.128.66.0F38.WIG 09 /r] VPSIGNW xmm1, xmm2, xmm3/m128
    VpsignwVdqHdqWdqV128,
    // [VEX.128.66.0F38.WIG 0A /r] VPSIGND xmm1, xmm2, xmm3/m128
    VpsigndVdqHdqWdqV128,
    // [VEX.256.66.0F38.WIG 08 /r] VPSIGNB ymm1, ymm2, ymm3/m256
    VpsignbVqqHqqWqqV256,
    // [VEX.256.66.0F38.WIG 09 /r] VPSIGNW ymm1, ymm2, ymm3/m256
    VpsignwVqqHqqWqqV256,
    // [VEX.256.66.0F38.WIG 0A /r] VPSIGND ymm1, ymm2, ymm3/m256
    VpsigndVqqHqqWqqV256,

    // [66 0F 73 /7 ib] PSLLDQ xmm1, imm8
    PslldqUdqIb,
    // [VEX.128.66.0F.WIG 73 /7 ib] VPSLLDQ xmm1, xmm2, imm8
    VpslldqHdqUdqIbV128,
    // [VEX.256.66.0F.WIG 73 /7 ib] VPSLLDQ ymm1, ymm2, imm8
    VpslldqHqqUqqIbV256,
    // [EVEX.128.66.0F.WIG 73 /7 ib] VPSLLDQ xmm1, xmm2/m128, imm8
    VpslldqHdqUdqIbE128,
    // [EVEX.256.66.0F.WIG 73 /7 ib] VPSLLDQ ymm1, ymm2/m256, imm8
    VpslldqHqqUqqIbE256,
    // [EVEX.512.66.0F.WIG 73 /7 ib] VPSLLDQ zmm1, zmm2/m512, imm8
    VpslldqHdqqUdqqIbE512,

    // [NP 0F F1 /r] PSLLW mm1, mm2/m64
    PsllwPqQq,
    // [66 0F F1 /r] PSLLW xmm1, xmm2/m128
    PsllwVdqWdq,
    // [NP 0F 71 /6 ib] PSLLW mm, imm8
    PsllwNqIb,
    // [66 0F 71 /6 ib] PSLLW xmm1, xmm2/m128
    PsllwUdqIb,
    // [NP 0F F2 /r] PSLLD mm, mm2/m64
    PslldPqQq,
    // [66 0F F2 /r] PSLLD xmm1, xmm2/m128
    PslldVdqWdq,
    // [NP 0F 72 /6 ib] PSLLD mm, imm8
    PslldNqIb,
    // [66 0F 72 /6 ib] PSLLD xmm, imm8
    PslldUdqIb,
    // [NP 0F F3 /r] PSLLQ mm1, mm2/m64
    PsllqPqQq,
    // [66 0F F3 /r] PSLLQ xmm1, xmm2/m128
    PsllqVdqWdq,
    // [VEX.128.66.0F.WIG F1 /r] VPSLLW xmm1, xmm2, xmm3/m128
    VpsllwVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 71 /6 ib] VPSLLW xmm1, xmm2, imm8
    VpsllwHdqUdqIbV128,
    // [VEX.128.66.0F.WIG F2 /r] VPSLLD xmm1, xmm2, xmm3/m128
    VpslldVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 72 /6 ib] VPSLLD xmm1, xmm2, imm8
    VpslldHdqUdqIbV128,
    // [VEX.128.66.0F.WIG F3 /r] VPSLLQ xmm1, xmm2, xmm3/m128
    VpsllqVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 73 /6 ib] VPSLLQ xmm1, xmm2, imm8
    VpsllqHdqUdqIbV128,
    // [VEX.256.66.0F.WIG F1 /r] VPSLLW ymm1, ymm2, ymm3/m128
    VpsllwVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG 71 /6 ib] VPSLLW ymm1, ymm2, imm8
    VpsllwHqqUqqIbV256,
    // [VEX.256.66.0F.WIG F2 /r] VPSLLD ymm1, ymm2, ymm3/m128
    VpslldVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG 72 /6 ib] VPSLLD ymm1, ymm2, imm8
    VpslldHqqUqqIbV256,
    // [VEX.256.66.0F.WIG F3 /r] VPSLLQ ymm1, ymm2, ymm3/m128
    VpsllqVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG 73 /6 ib] VPSLLQ ymm1, ymm2, imm8
    VpsllqHqqUqqIbV256,
    // [EVEX.128.66.0F.WIG F1 /r] VPSLLW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsllwVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG F1 /r] VPSLLW ymm1 {k1}{z}, ymm2, xmm3/m128
    VpsllwVqqHqqWdqE256,
    // [EVEX.256.66.0F.WIG F1 /r] VPSLLW ymm1 {k1}{z}, ymm2, xmm3/m128
    VpsllwVdqqHdqqWdqE512,
    // [EVEX.128.66.0F.WIG 71 /6 ib] VPSLLW xmm1 {k1}{z}, xmm2/m128, imm8
    VpsllwHdqUdqIbE128,
    // [EVEX.256.66.0F.WIG 71 /6 ib] VPSLLW ymm1 {k1}{z}, ymm2/m256, imm8
    VpsllwHqqUqqIbE256,
    // [EVEX.512.66.0F.WIG 71 /6 ib] VPSLLW zmm1 {k1}{z}, zmm2/m512, imm8
    VpsllwHdqqUdqqIbE512,
    // [EVEX.128.66.0F.W0 F2 /r] VPSLLD xmm1 {k1}{z}, xmm2, xmm3/m128
    VpslldVdqHdqWdqE128,
    // [EVEX.256.66.0F.W0 F2 /r] VPSLLD ymm1 {k1}{z}, ymm2, xmm3/m128
    VpslldVqqHqqWdqE256,
    // [EVEX.512.66.0F.W0 F2 /r] VPSLLD zmm1 {k1}{z}, zmm2, xmm3/m128
    VpslldVdqqHdqqWdqE512,
    // [EVEX.128.66.0F.W0 72 /6 ib] VPSLLD xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VpslldHdqUdqIbE128,
    // [EVEX.256.66.0F.W0 72 /6 ib] VPSLLD ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VpslldHqqUqqIbE256,
    // [EVEX.512.66.0F.W0 72 /6 ib] VPSLLD zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VpslldHdqqUdqqIbE512,
    // [EVEX.128.66.0F.W1 F3 /r] VPSLLQ xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsllqVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 F3 /r] VPSLLQ ymm1 {k1}{z}, ymm2, xmm3/m128
    VpsllqVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 F3 /r] VPSLLQ zmm1 {k1}{z}, zmm2, xmm3/m128
    VpsllqVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F.W1 73 /6 ib] VPSLLQ xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VpsllqHdqUdqIbE128,
    // [EVEX.256.66.0F.W1 73 /6 ib] VPSLLQ ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VpsllqHqqUqqIbE256,
    // [EVEX.512.66.0F.W1 73 /6 ib] VPSLLQ zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VpsllqHdqqUdqqIbE512,

    // [NP 0F E1 /r] PSRAW mm1, mm2/m64
    PsrawPqQq,
    // [66 0F E1 /r] PSRAW xmm1, xmm2/m128
    PsrawVdqWdq,
    // [NP 0F 71 /4 ib] PSRAW mm, imm8
    PsrawNqIb,
    // [66 0F 71 /4 ib] PSRAW xmm, imm8
    PsrawUdqIb,
    // [NP 0F E2 /r] PSRAD mm1, mm2/m64
    PsradPqQq,
    // [66 0F E2 /r] PSRAD xmm1, xmm2/m128
    PsradVdqWdq,
    // [NP 0F 72 /4 ib] PSRAD mm, imm8
    PsradNqIb,
    // [66 0F 72 /4 ib] PSRAD xmm, imm8
    PsradUdqIb,
    // [VEX.128.66.0F.WIG E1 /r] VPSRAW xmm1, xmm2, xmm3/m128
    VpsrawVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 71 /4 ib] VPSRAW xmm1, xmm2, imm8
    VpsrawHdqUdqIbV128,
    // [VEX.128.66.0F.WIG E2 /r] VPSRAD xmm1, xmm2, xmm3/m128
    VpsradVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 72 /4 ib] VPSRAD xmm1, xmm2, imm8
    VpsradHdqUdqIbV128,
    // [VEX.256.66.0F.WIG E1 /r] VPSRAW ymm1, ymm2, xmm3/m128
    VpsrawVqqHqqWdqV256,
    // [VEX.256.66.0F.WIG 71 /4 ib] VPSRAW ymm1, ymm2, imm8
    VpsrawHqqUqqIbV256,
    // [VEX.256.66.0F.WIG E2 /r] VPSRAD ymm1, ymm2, xmm3/m128
    VpsradVqqHqqWdqV256,
    // [VEX.256.66.0F.WIG 72 /4 ib] VPSRAD ymm1, ymm2, imm8
    VpsradHqqUqqIbV256,
    // [EVEX.128.66.0F.WIG E1 /r] VPSRAW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsrawVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG E1 /r] VPSRAW ymm1 {k1}{z}, ymm2, xmm3/m128
    VpsrawVqqHqqWdqE256,
    // [EVEX.512.66.0F.WIG E1 /r] VPSRAW zmm1 {k1}{z}, zmm2, xmm3/m128
    VpsrawVdqqHdqqWdqE512,
    // [EVEX.128.66.0F.WIG 71 /4 ib] VPSRAW xmm1 {k1}{z}, xmm2/m128, imm8
    VpsrawHdqUdqIbE128,
    // [EVEX.256.66.0F.WIG 71 /4 ib] VPSRAW ymm1 {k1}{z}, ymm2/m128, imm8
    VpsrawHqqUqqIbE256,
    // [EVEX.512.66.0F.WIG 71 /4 ib] VPSRAW zmm1 {k1}{z}, zmm2/m128, imm8
    VpsrawHdqqUdqqIbE512,
    // [EVEX.128.66.0F.W0 E2 /r] VPSRAD xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsradVdqHdqWdqE128,
    // [EVEX.256.66.0F.W0 E2 /r] VPSRAD ymm1 {k1}{z}, ymm2, xmm3/m128
    VpsradVqqHqqWdqE256,
    // [EVEX.512.66.0F.W0 E2 /r] VPSRAD zmm1 {k1}{z}, zmm2, xmm3/m128
    VpsradVdqqHdqqWdqE512,
    // [EVEX.128.66.0F.W0 72 /4 ib] VPSRAD xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VpsradHdqUdqIbE128,
    // [EVEX.256.66.0F.W0 72 /4 ib] VPSRAD ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VpsradHqqUqqIbE256,
    // [EVEX.512.66.0F.W0 72 /4 ib] VPSRAD zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VpsradHdqqUdqqIbE512,
    // [EVEX.128.66.0F.W1 E2 /r] VPSRAQ xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsraqVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 E2 /r] VPSRAQ ymm1 {k1}{z}, ymm2, xmm3/m128
    VpsraqVqqHqqWdqE256,
    // [EVEX.512.66.0F.W1 E2 /r] VPSRAQ zmm1 {k1}{z}, zmm2, xmm3/m128
    VpsraqVdqqHdqqWdqE512,
    // [EVEX.128.66.0F.W1 72 /4 ib] VPSRAQ xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VpsraqHdqUdqIbE128,
    // [EVEX.256.66.0F.W1 72 /4 ib] VPSRAQ ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VpsraqHqqUqqIbE256,
    // [EVEX.512.66.0F.W1 72 /4 ib] VPSRAQ zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VpsraqHdqqUdqqIbE512,

    // [66 0F 73 /3 ib] PSRLDQ xmm1, imm8
    PsrldqUdqIb,
    // [VEX.128.66.0F.WIG 73 /3 ib] VPSRLDQ xmm1, xmm2, imm8
    VpsrldqHdqUdqIbV128,
    // [VEX.256.66.0F.WIG 73 /3 ib] VPSRLDQ ymm1, ymm2, imm8
    VpsrldqHqqUqqIbV256,
    // [EVEX.128.66.0F.WIG 73 /3 ib] VPSRLDQ xmm1, xmm2/m128, imm8
    VpsrldqHdqUdqIbE128,
    // [EVEX.256.66.0F.WIG 73 /3 ib] VPSRLDQ ymm1, ymm2/m256, imm8
    VpsrldqHqqUqqIbE256,
    // [EVEX.512.66.0F.WIG 73 /3 ib] VPSRLDQ zmm1, zmm2/m512, imm8
    VpsrldqHdqqUdqqIbE512,

    // [NP 0F D1 /r] PSRLW mm1, mm2/m64
    PsrlwPqQq,
    // [66 0F D1 /r] PSRLW xmm1, xmm2/m128
    PsrlwVdqWdq,
    // [NP 0F 71 /2 ib] PSRLW mm, imm8
    PsrlwNqIb,
    // [66 0F 71 /2 ib] PSRLW xmm1, imm8
    PsrlwUdqIb,
    // [NP 0F D2 /r] PSRLD mm1, mm2/m64
    PsrldPqQq,
    // [66 0F D2 /r] PSRLD xmm1, xmm2/m128
    PsrldVdqWdq,
    // [NP 0F 72 /2 ib] PSRLD mm, imm8
    PsrldNqIb,
    // [66 0F 72 /2 ib] PSRLD xmm, imm8
    PsrldUdqIb,
    // [NP 0F D3 /r] PSRLQ mm1, mm2/m64
    PsrlqPqQq,
    // [66 0F D3 /r] PSRLQ xmm1, xmm2/m128
    PsrlqVdqWdq,
    // [NP 0F 73 /2 ib] PSRLQ mm, imm8
    PsrlqNqIb,
    // [66 0F 73 /2 ib] PSRLQ xmm, imm8
    PsrlqUdqIb,
    // [VEX.128.66.0F.WIG D1 /r] VPSRLW xmm1, xmm2, xmm3/m128
    VpsrlwVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 71 /2 ib] VPSRLW xmm1, xmm2, imm8
    VpsrlwHdqUdqIbV128,
    // [VEX.128.66.0F.WIG D2 /r] VPSRLD xmm1, xmm2, xmm3/m128
    VpsrldVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 72 /2 ib] VPSRLD xmm1, xmm2, imm8
    VpsrldHdqUdqIbV128,
    // [VEX.128.66.0F.WIG D3 /r] VPSRLQ xmm1, xmm2, xmm3/m128
    VpsrlqVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 73 /2 ib] VPSRLQ xmm1, xmm2, imm8
    VpsrlqHdqUdqIbV128,
    // [VEX.256.66.0F.WIG D1 /r] VPSRLW ymm1, ymm2, xmm3/m128
    VpsrlwVqqHqqWdqV128,
    // [VEX.256.66.0F.WIG 71 /2 ib] VPSRLW ymm1, ymm2, imm8
    VpsrlwHqqUdqIbV128,
    // [VEX.256.66.0F.WIG D2 /r] VPSRLD ymm1, ymm2, xmm3/m128
    VpsrldVqqHqqWdqV128,
    // [VEX.256.66.0F.WIG 72 /2 ib] VPSRLD ymm1, ymm2, imm8
    VpsrldHqqUdqIbV128,
    // [VEX.256.66.0F.WIG D3 /r] VPSRLQ ymm1, ymm2, xmm3/m128
    VpsrlqVqqHqqWdqV128,
    // [VEX.256.66.0F.WIG 73 /2 ib] VPSRLQ ymm1, ymm2, imm8
    VpsrlqHqqUdqIbV128,
    // [EVEX.128.66.0F.WIG D1 /r] VPSRLW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsrlwVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG D1 /r] VPSRLW ymm1 {k1}{z}, ymm2, xmm3/m128
    VpsrlwVqqHqqWdqE256,
    // [EVEX.512.66.0F.WIG D1 /r] VPSRLW zmm1 {k1}{z}, zmm2, xmm3/m128
    VpsrlwVdqqHdqqWdqE512,
    // [EVEX.128.66.0F.WIG 71 /2 ib] VPSRLW xmm1 {k1}{z}, xmm2/m128, imm8
    VpsrlwHdqUdqIbE128,
    // [EVEX.256.66.0F.WIG 71 /2 ib] VPSRLW ymm1 {k1}{z}, ymm2/m256, imm8
    VpsrlwHdqUdqIbE256,
    // [EVEX.512.66.0F.WIG 71 /2 ib] VPSRLW zmm1 {k1}{z}, zmm2/m512, imm8
    VpsrlwHdqUdqIbE512,
    // [EVEX.128.66.0F.WIG D2 /r] VPSRLD xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsrldVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG D2 /r] VPSRLD ymm1 {k1}{z}, ymm2, xmm3/m128
    VpsrldVqqHqqWdqE256,
    // [EVEX.512.66.0F.WIG D2 /r] VPSRLD zmm1 {k1}{z}, zmm2, xmm3/m128
    VpsrldVdqqHdqqWdqE512,
    // [EVEX.128.66.0F.WIG 72 /2 ib] VPSRLD xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VpsrldHdqUdqIbE128,
    // [EVEX.256.66.0F.WIG 72 /2 ib] VPSRLD ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VpsrldHdqUdqIbE256,
    // [EVEX.512.66.0F.WIG 72 /2 ib] VPSRLD zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VpsrldHdqUdqIbE512,
    // [EVEX.128.66.0F.WIG D3 /r] VPSRLQ xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsrlqVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG D3 /r] VPSRLQ ymm1 {k1}{z}, ymm2, xmm3/m128
    VpsrlqVqqHqqWdqE256,
    // [EVEX.512.66.0F.WIG D3 /r] VPSRLQ zmm1 {k1}{z}, zmm2, xmm3/m128
    VpsrlqVdqqHdqqWdqE512,
    // [EVEX.128.66.0F.WIG 73 /2 ib] VPSRLQ xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VprslwHdqUdqIbE128,
    // [EVEX.256.66.0F.WIG 73 /2 ib] VPSRLQ ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VprslwHdqUdqIbE256,
    // [EVEX.512.66.0F.WIG 73 /2 ib] VPSRLQ zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VprslwHdqUdqIbE512,

    // [NP 0F F8 /r] PSUBB mm1, mm2/m64
    PsubbPqQq,
    // [66 0F F8 /r] PSUBB xmm1, xmm2/m128
    PsubbVdqWdq,
    // [NP 0F F9 /r] PSUBW mm1, mm2/m64
    PsubwPqQq,
    // [66 0F F9 /r] PSUBW xmm1, xmm2/m128
    PsubwVdqWdq,
    // [NP 0F FA /r] PSUBD mm1, mm2/m64
    PsubdPqQq,
    // [66 0F FA /r] PSUBD xmm1, xmm2/m128
    PsubdVdqWdq,
    // [VEX.128.66.0F.WIG F8 /r] VSUBB xmm1, xmm2, xmm3/m128
    VpsubbVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG F9 /r] VSUBW xmm1, xmm2, xmm3/m128
    VpsubwVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG FA /r] VSUBD xmm1, xmm2, xmm3/m128
    VpsubdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG F8 /r] VSUBB ymm1, ymm2, ymm3/m256
    VpsubbVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG F9 /r] VSUBW ymm1, ymm2, ymm3/m256
    VpsubwVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG FA /r] VSUBD ymm1, ymm2, ymm3/m256
    VpsubdVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG F8 /r] VSUBB xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsubbVdqHdqWdqE128,
    // [EVEX.128.66.0F.WIG F9 /r] VSUBW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsubwVdqHdqWdqE128,
    // [EVEX.128.66.0F.WIG FA /r] VSUBD xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsubdVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG F8 /r] VSUBB ymm1 {k1}{z}, ymm2, ymm3/m256
    VpsubbVqqHqqWqqE256,
    // [EVEX.256.66.0F.WIG F9 /r] VSUBW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpsubwVqqHqqWqqE256,
    // [EVEX.256.66.0F.WIG FA /r] VSUBD ymm1 {k1}{z}, ymm2, ymm3/m256
    VpsubdVqqHqqWqqE256,
    // [EVEX.512.66.0F.W0 F8 /r] VSUBB zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpsubbVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F.W0 F9 /r] VSUBW zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpsubwVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F.W0 FA /r] VSUBD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpsubdVdqqHdqqWdqqE512,

    // [NP 0F FB /r] PSUBQ mm1, mm2/m64
    PsubqPqQq,
    // [66 0F FB /r] PSUBQ xmm1, xmm2/m128
    PsubqVdqWdq,
    // [VEX.128.66.0F.WIG FB /r] VPSUBQ xmm1, xmm2, xmm3/m128
    VpsubqVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG FB /r] VPSUBQ ymm1, ymm2, ymm3/m256
    VpsubqVqqHqqWqqV256,
    // [EVEX.128.66.0F.W1 FB /r] VPSUBQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpsubqVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 FB /r] VPSUBQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpsubqVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 FB /r] VPSUBQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpsubqVdqqHdqqWdqqE512,

    // [NP 0F E8 /r] PSUBSB mm1, mm2/m64
    PsubsbPqQq,
    // [66 0F E8 /r] PSUBSB xmm1, xmm2/m128
    PsubsbVdqWdq,
    // [NP 0F E9 /r] PSUBSW mm1, mm2/m64
    PsubswPqQq,
    // [66 0F E9 /r] PSUBSW xmm1, xmm2/m128
    PsubswVdqWdq,
    // [VEX.128.66.0F.WIG E8 /r] VPSUBSB xmm1, xmm2, xmm3/m128
    VpsubsbVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG E9 /r] VPSUBSW xmm1, xmm2, xmm3/m128
    VpsubswVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG E8 /r] VPSUBSB ymm1, ymm2, ymm3/m256
    VpsubsbVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG E9 /r] VPSUBSW ymm1, ymm2, ymm3/m256
    VpsubswVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG E8 /r] VPSUBSB xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsubsbVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG E8 /r] VPSUBSB ymm1 {k1}{z}, ymm2, ymm3/m256
    VpsubsbVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG E8 /r] VPSUBSB zmm1 {k1}{z}, xmm2, zmm3/m512
    VpsubsbVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F.WIG E9 /r] VPSUBSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsubswVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG E9 /r] VPSUBSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpsubswVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG E9 /r] VPSUBSW zmm1 {k1}{z}, xmm2, zmm3/m512
    VpsubswVdqqHdqqWdqqE512,

    // [NP 0F D8 /r] PSUBUSB mm1, mm2/m64
    PsubusbPqQq,
    // [66 0F D8 /r] PSUBUSB xmm1, xmm2/m128
    PsubusbVdqWdq,
    // [NP 0F D9 /r] PSUBUSW mm1, mm2/m64
    PsubuswPqQq,
    // [66 0F D9 /r] PSUBUSW xmm1, xmm2/m128
    PsubuswVdqWdq,
    // [VEX.128.66.0F.WIG D8 /r] VPSUBUSB xmm1, xmm2, xmm3/m128
    VpsubusbVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG D9 /r] VPSUBUSW xmm1, xmm2, xmm3/m128
    VpsubuswVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG D8 /r] VPSUBUSB ymm1, ymm2, ymm3/m256
    VpsubusbVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG D9 /r] VPSUBUSW ymm1, ymm2, ymm3/m256
    VpsubuswVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG D8 /r] VPSUBUSB xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsubusbVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG D8 /r] VPSUBUSB ymm1 {k1}{z}, ymm2, ymm3/m256
    VpsubusbVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG D8 /r] VPSUBUSB zmm1 {k1}{z}, xmm2, zmm3/m512
    VpsubusbVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F.WIG D9 /r] VPSUBUSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsubuswVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG D9 /r] VPSUBUSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpsubuswVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG D9 /r] VPSUBUSW zmm1 {k1}{z}, xmm2, zmm3/m512
    VpsubuswVdqqHdqqWdqqE512,

    // [66 0F 38 17 /r] PTEST xmm1, xmm2/m128
    PtestVdqWdq,
    // [VEX.128.66.0F38.WIG 17 /r] VPTEST xmm1, xmm2/m128
    VptestVdqWdqV128,
    // [VEX.256.66.0F38.WIG 17 /r] VPTEST ymm1, ymm2/m256
    VptestVqqWqqV256,

    // [F3 0F AE /4] PTWRITE r/m32
    PtwriteEd,
    // [F3 REX.W 0F AE /4] PTWRITE r/m64
    PtwriteEq,

    // [NP 0F 68 /r] PUNPCKHBW mm1, mm2/m64
    PunpckhbwPqQq,
    // [66 0F 68 /r] PUNPCKHBW xmm1, xmm2/m128
    PunpckhbwVdqWdq,
    // [NP 0F 69 /r] PUNPCKHWD mm1, mm2/m64
    PunpckhwdPqQq,
    // [66 0F 69 /r] PUNPCKHWD xmm1, xmm2/m128
    PunpckhwdVdqWdq,
    // [NP 0F 6A /r] PUNPCKHDQ mm1, mm2/m64
    PunpckhdqPqQq,
    // [66 0F 6A /r] PUNPCKHDQ xmm1, xmm2/m128
    PunpckhdqVdqWdq,
    // [66 0F 6D /r] PUNPCKHQDQ xmm1, xmm2/m128
    PunpckhqdqVdqWdq,
    // [VEX.128.66.0F.WIG 68 /r] VPUNPCKHBW xmm1, xmm2, xmm3/m128
    VpunpckhbwVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 69 /r] VPUNPCKHWD xmm1, xmm2, xmm3/m128
    VpunpckhwdVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 6A /r] VPUNPCKHDQ xmm1, xmm2, xmm3/m128
    VpunpckhdqVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 6D /r] VPUNPCKHQDQ xmm1, xmm2, xmm3/m128
    VpunpckhqdqVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 68 /r] VPUNPCKHBW ymm1, ymm2, ymm3/m256
    VpunpckhbwVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG 69 /r] VPUNPCKHWD ymm1, ymm2, ymm3/m256
    VpunpckhwdVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG 6A /r] VPUNPCKHDQ ymm1, ymm2, ymm3/m256
    VpunpckhdqVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG 6D /r] VPUNPCKHQDQ ymm1, ymm2, ymm3/m256
    VpunpckhqdqVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG 68 /r] VPUNPCKHBW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpunpckhbwVdqHdqWdqE128,
    // [EVEX.128.66.0F.WIG 69 /r] VPUNPCKHWD xmm1 {k1}{z}, xmm2, xmm3/m128
    VpunpckhwdVdqHdqWdqE128,
    // [EVEX.128.66.0F.WIG 6A /r] VPUNPCKHDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpunpckhdqVdqHdqWdqE128,
    // [EVEX.128.66.0F.WIG 6D /r] VPUNPCKHQDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpunpckhqdqVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG 68 /r] VPUNPCKHBW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpunpckhbwVqqHqqWqqE256,
    // [EVEX.256.66.0F.WIG 69 /r] VPUNPCKHWD ymm1 {k1}{z}, ymm2, ymm3/m256
    VpunpckhwdVqqHqqWqqE256,
    // [EVEX.256.66.0F.WIG 6A /r] VPUNPCKHDQ ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpunpckhdqVqqHqqWqqE256,
    // [EVEX.256.66.0F.WIG 6D /r] VPUNPCKHQDQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpunpckhqdqVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG 68 /r] VPUNPCKHBW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpunpckhbwVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F.WIG 69 /r] VPUNPCKHWD zmm1 {k1}{z}, zmm2, zmm3/m512
    VpunpckhwdVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F.WIG 6A /r] VPUNPCKHDQ zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpunpckhdqVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F.WIG 6D /r] VPUNPCKHQDQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpunpckhqdqVdqqHdqqWdqqE512,

    // [NP 0F 60 /r] PUNPCKLBW mm1, mm2/m64
    PunpcklbwPqQq,
    // [66 0F 60 /r] PUNPCKLBW xmm1, xmm2/m128
    PunpcklbwVdqWdq,
    // [NP 0F 61 /r] PUNPCKLWD mm1, mm2/m64
    PunpcklwdPqQq,
    // [66 0F 61 /r] PUNPCKLWD xmm1, xmm2/m128
    PunpcklwdVdqWdq,
    // [NP 0F 62 /r] PUNPCKLDQ mm1, mm2/m64
    PunpckldqPqQq,
    // [66 0F 62 /r] PUNPCKLDQ xmm1, xmm2/m128
    PunpckldqVdqWdq,
    // [66 0F 6C /r] PUNPCKLQDQ xmm1, xmm2/m128
    PunpcklqdqVdqWdq,
    // [VEX.128.66.0F.WIG 60 /r] VPUNPCKLBW xmm1, xmm2, xmm3/m128
    VpunpcklbwVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 61 /r] VPUNPCKLWD xmm1, xmm2, xmm3/m128
    VpunpcklwdVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 62 /r] VPUNPCKLDQ xmm1, xmm2, xmm3/m128
    VpunpckldqVdqHdqWdqV128,
    // [VEX.128.66.0F.WIG 6C /r] VPUNPCKLQDQ xmm1, xmm2, xmm3/m128
    VpunpcklqdqVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 60 /r] VPUNPCKLBW ymm1, ymm2, ymm3/m256
    VpunpcklbwVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG 61 /r] VPUNPCKLWD ymm1, ymm2, ymm3/m256
    VpunpcklwdVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG 62 /r] VPUNPCKLDQ ymm1, ymm2, ymm3/m256
    VpunpckldqVqqHqqWqqV256,
    // [VEX.256.66.0F.WIG 6C /r] VPUNPCKLQDQ ymm1, ymm2, ymm3/m256
    VpunpcklqdqVqqHqqWqqV256,
    // [EVEX.128.66.0F.WIG 60 /r] VPUNPCKLBW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpunpcklbwVdqHdqWdqE128,
    // [EVEX.128.66.0F.WIG 61 /r] VPUNPCKLWD xmm1 {k1}{z}, xmm2, xmm3/m128
    VpunpcklwdVdqHdqWdqE128,
    // [EVEX.128.66.0F.WIG 62 /r] VPUNPCKLDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpunpckldqVdqHdqWdqE128,
    // [EVEX.128.66.0F.WIG 6C /r] VPUNPCKLQDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpunpcklqdqVdqHdqWdqE128,
    // [EVEX.256.66.0F.WIG 60 /r] VPUNPCKLBW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpunpcklbwVqqHqqWqqE256,
    // [EVEX.256.66.0F.WIG 61 /r] VPUNPCKLWD ymm1 {k1}{z}, ymm2, ymm3/m256
    VpunpcklwdVqqHqqWqqE256,
    // [EVEX.256.66.0F.WIG 62 /r] VPUNPCKLDQ ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpunpckldqVqqHqqWqqE256,
    // [EVEX.256.66.0F.WIG 6C /r] VPUNPCKLQDQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpunpcklqdqVqqHqqWqqE256,
    // [EVEX.512.66.0F.WIG 60 /r] VPUNPCKLBW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpunpcklbwVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F.WIG 61 /r] VPUNPCKLWD zmm1 {k1}{z}, zmm2, zmm3/m512
    VpunpcklwdVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F.WIG 62 /r] VPUNPCKLDQ zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpunpckldqVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F.WIG 6C /r] VPUNPCKLQDQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpunpcklqdqVdqqHdqqWdqqE512,

    // [FF /6] PUSH r/m16
    PushEw,
    // [FF /6] PUSH r/m32
    PushEd,
    // [FF /6] PUSH r/m64
    PushEq,
    // [50+rw] PUSH r16
    PushGw,
    // [50+rd] PUSH r32
    PushGd,
    // [50+rd] PUSH r64
    PushGq,
    // [6A ib] PUSH imm8
    PushIb,
    // [68 iw] PUSH imm16
    PushIw,
    // [68 iw] PUSH imm32
    PushId,
    // [0E] PUSH CS
    PushCS,
    // [16] PUSH SS
    PushSS,
    // [1E] PUSH DS
    PushDS,
    // [06] PUSH ES
    PushES,
    // [0F A0] PUSH FS
    PushFS,
    // [0F A8] PUSH GS
    PushGS,

    // [60] PUSHA
    Pusha,
    // [60] PUSHAD
    Pushad,

    // [9C] PUSHF
    Pushf,
    // [9C] PUSHFD
    Pushfd,
    // [9C] PUSHFQ
    Pushfq,

    // [NP 0F EF /r] PXOR mm1, mm2/m64
    PxorPqQq,
    // [66 0F EF /r] PXOR xmm1, xmm2/m128
    PxorVdqWdq,
    // [VEX.128.66.0F.WIG EF /r] VPXOR xmm1, xmm2, xmm3/m128
    VpxorVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG EF /r] VPXOR ymm1, ymm2, ymm3/m256
    VpxorVqqHqqWqqV256,
    // [EVEX.128.66.0F.W0 EF /r] VPXORD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpxordVdqHdqWdqE128,
    // [EVEX.256.66.0F.W0 EF /r] VPXORD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpxordVqqHqqWqqE256,
    // [EVEX.512.66.0F.W0 EF /r] VPXORD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpxordVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F.W1 EF /r] VPXORQ xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpxorqVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 EF /r] VPXORQ ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpxorqVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 EF /r] VPXORQ zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpxorqVdqqHdqqWdqqE512,

    // [D0 /2] RCL r/m8, 1
    // [REX D0 /2] RCL r/m8, 1
    RclEb1,
    // [D2 /2] RCL r/m8, CL
    // [REX D2 /2] RCL r/m8, CL
    RclEbCL,
    // [C0 /2 ib] RCL r/m8, imm8
    // [REX C0 /2 ib] RCL r/m8, imm8
    RclEbIb,
    // [D1 /2] RCL r/m16, 1
    RclEw1,
    // [D3 /2] RCL r/m16, CL
    RclEwCL,
    // [C1 /2 ib] RCL r/m16, imm8
    RclEwIb,
    // [D1 /2] RCL r/m32, 1
    RclEd1,
    // [REX.W D1 /2] RCL r/m64, 1
    RclEq1,
    // [D3 /2] RCL r/m32, CL
    RclEdCL,
    // [REX.W D3 /2] RCL r/m64, CL
    RclEqCL,
    // [C1 /2 ib] RCL r/m32, imm8
    RclEdIb,
    // [REX.W C1 /2 ib] RCL r/m64, imm8
    RclEqIb,
    // [D0 /3] RCR r/m8, 1
    // [REX D0 /3] RCR r/m8, 1
    RcrEb1,
    // [D2 /3] RCR r/m8, CL
    // [REX D2 /3] RCR r/m8, CL
    RcrEbCL,
    // [C0 /3 ib] RCR r/m8, imm8
    // [REX C0 /3 ib] RCR r/m8, imm8
    RcrEbIb,
    // [D1 /3] RCR r/m16, 1
    RcrEw1,
    // [D3 /3] RCR r/m16, CL
    RcrEwCL,
    // [C1 /3 ib] RCR r/m16, imm8
    RcrEwIb,
    // [D1 /3] RCR r/m32, 1
    RcrEd1,
    // [REX.W D1 /3] RCR r/m64, 1
    RcrEq1,
    // [D3 /3] RCR r/m32, CL
    RcrEdCL,
    // [REX.W D3 /3] RCR r/m64, CL
    RcrEqCL,
    // [C1 /3 ib] RCR r/m32, imm8
    RcrEdIb,
    // [REX.W C1 /3 ib] RCR r/m64, imm8
    RcrEqIb,
    // [D0 /0] ROL r/m8, 1
    // [REX D0 /0] ROL r/m8, 1
    RolEb1,
    // [D2 /0] ROL r/m8, CL
    // [REX D2 /0] ROL r/m8, CL
    RolEbCL,
    // [C0 /0 ib] ROL r/m8, imm8
    // [REX C0 /0 ib] ROL r/m8, imm8
    RolEbIb,
    // [D1 /0] ROL r/m16, 1
    RolEw1,
    // [D3 /0] ROL r/m16, CL
    RolEwCL,
    // [C1 /0 ib] ROL r/m16, imm8
    RolEwIb,
    // [D1 /0] ROL r/m32, 1
    RolEd1,
    // [REX.W D1 /0] ROL r/m64, 1
    RolEq1,
    // [D3 /0] ROL r/m32, CL
    RolEdCL,
    // [REX.W D3 /0] ROL r/m64, CL
    RolEqCL,
    // [C1 /0 ib] ROL r/m32, imm8
    RolEdIb,
    // [REX.W C1 /0 ib] ROL r/m64, imm8
    RolEqIb,
    // [D0 /1] ROR r/m8, 1
    // [REX D0 /1] ROR r/m8, 1
    RorEb1,
    // [D2 /1] ROR r/m8, CL
    // [REX D2 /1] ROR r/m8, CL
    RorEbCL,
    // [C0 /1 ib] ROR r/m8, imm8
    // [REX C0 /1 ib] ROR r/m8, imm8
    RorEbIb,
    // [D1 /1] ROR r/m16, 1
    RorEw1,
    // [D3 /1] ROR r/m16, CL
    RorEwCL,
    // [C1 /1 ib] ROR r/m16, imm8
    RorEwIb,
    // [D1 /1] ROR r/m32, 1
    RorEd1,
    // [REX.W D1 /1] ROR r/m64, 1
    RorEq1,
    // [D3 /1] ROR r/m32, CL
    RorEdCL,
    // [REX.W D3 /1] ROR r/m64, CL
    RorEqCL,
    // [C1 /1 ib] ROR r/m32, imm8
    RorEdIb,
    // [REX.W C1 /1 ib] ROR r/m64, imm8
    RorEqIb,

    // [NP 0F 53 /r] RCPPS xmm1, xmm2/m128
    RcppsVdqWdq,
    // [VEX.128.0F.WIG 53 /r] VRCPPS xmm1, xmm2/m128
    VrcppsVdqWdqV128,
    // [VEX.256.0F.WIG 53 /r] VRCPPS ymm1, ymm2/m256
    VrcppsVqqWqqV256,

    // [F3 0F 53 /r] RCPPS xmm1, xmm2/m32
    RcpssVdqWd,
    // [VEX.128.F3.0F.WIG 53 /r] VRCPSS xmm1, xmm2, xmm3/m32
    VrcpssVdqHdqWdV128,

    // [F3 0F AE /0] RDFSBASE r32
    RdfsbaseGd,
    // [F3 REX.W 0F AE /0] RDFSBASE r64
    RdfsbaseGq,
    // [F3 0F AE /1] RDGSBASE r32
    RdgsbaseGd,
    // [F3 REX.W 0F AE /1] RDGSBASE r64
    RdgsbaseGq,

    // [0F 32] RDMSR
    Rdmsr,

    // [F3 0F C7 /7] RDPID r32
    RdpidGd,
    // [F3 0F C7 /7] RDPID r64
    RdpidGq,

    // [NP 0F 01 EE] RDPKRU
    Rdpkru,

    // [0F 33] RDPMC
    Rdpmc,

    // [NFx 0F C7 /6] RDRAND r16
    RdrandGw,
    // [NFx 0F C7 /6] RDRAND r32
    RdrandGd,
    // [NFx REX.W 0F C7 /6] RDRAND r64
    RdrandGq,

    // [NFx 0F C7 /7] RDSEED r16
    RdseedGw,
    // [NFx 0F C7 /7] RDSEED r32
    RdseedGd,
    // [NFx REX.W 0F C7 /7] RDSEED r64
    RdseedGq,

    // [F3 0F 1E /1 (mod=11)] RDSSPD r32
    RdsspdGd,
    // [F3 REX.W 0F 1E /1 (mod=11)] RDSSPQ r64
    RdsspqGq,

    // [0F 31] RDTSC
    Rdtsc,

    // [0F 01 F9] RDTSCP
    Rdtscp,

    // [F3 6C] REP INS m8, DX
    RepInsYbDX,
    // [F3 6D] REP INS m16, DX
    RepInsYwDX,
    // [F3 6D] REP INS m32, DX
    RepInsYDX,
    // [F3 6D] REP INS r/m32, DX
    RepInsYdDX,
    // [F3 A4] REP MOVS m8, m8
    // [F3 REX.W A4] REP MOVS m8, m8
    RepMovsYbXb,
    // [F3 A5] REP MOVS m16, m16
    RepMovsYwXw,
    // [F3 A5] REP MOVS m32, m32
    RepMovsYdXd,
    // [F3 REX.W A5] REP MOVS m64, m64
    RepMovsYqXq,
    // [F3 6E] REP OUTS DX, r/m8
    // [F3 REX.W 6E] REP OUTS DX, r/m8
    RepOutsDXYb,
    // [F3 6F] REP OUTS DX, r/m16
    RepOutsDXYw,
    // [F3 6F] REP OUTS DX, r/m32
    // [F3 REX.W 6F] REP OUTS DX, r/m32
    RepOutsDXYd,
    // [F3 AC] REP LODS AL
    // [F3 REX.W AC] REP LODS AL
    RepLodsALXb,
    // [F3 AD] REP LODS AX
    RepLodsAXXw,
    // [F3 AD] REP LODS EAX
    RepLodsEAXXd,
    // [F3 REX.W AD] REP LODS RAX
    RepLodsRAXXq,
    // [F3 AA] REP STOS m8
    // [F3 REX.W AA] REP STOS m8
    RepStosYbAL,
    // [F3 AB] REP STOS m16
    RepStosYwAX,
    // [F3 AB] REP STOS m32
    RepStosYdEAX,
    // [F3 REX.W AB] REP STOS m64
    RepStosYqRAX,
    // [F3 A6] REPE CMPS m8, m8
    // [F3 REX.W A6] REPE CMPS m8, m8
    RepCmpsXbYb,
    // [F3 A7] REPE CMPS m16, m16
    RepCmpsXwYw,
    // [F3 A7] REPE CMPS m32, m32
    RepCmpsXdYd,
    // [F3 REX.W A7] REPE CMPS m64, m64
    RepCmpsXqYq,
    // [F3 AE] REPE SCAS m8
    // [F3 REX.W AE] REPE SCAS m8
    RepeScasALYb,
    // [F3 AF] REPE SCAS m16
    RepeScasAXYw,
    // [F3 AF] REPE SCAS m32
    RepeScasEAXYd,
    // [F3 REX.W AF] REPE SCAS m64
    RepeScasRAXYq,
    // [F2 A6] REPNE CMPS m8, m8
    // [F2 REX.W A6] REPNE CMPS m8, m8
    RepneCmpsXbYb,
    // [F2 A7] REPNE CMPS m16, m16
    RepneCmpsXwYw,
    // [F2 A7] REPNE CMPS m32, m32
    RepneCmpsXdYd,
    // [F2 REX.W A7] REPNE CMPS m64, m64
    RepneCmpsXqYq,
    // [F2 AE] REPNE SCAS m8
    // [F2 REX.W AE] REPNE SCAS m8
    RepneScasALYb,
    // [F2 AF] REPNE SCAS m16
    RepneScasAXYw,
    // [F2 AF] REPNE SCAS m32
    RepneScasEAXYd,
    // [F2 REX.W AF] REPNE SCAS m64
    RepneScasRAXYq,

    // [C3] RET
    Ret,
    // [CB] RET
    Retf,
    // [C2 iw] RET imm16
    RetIw,
    // [CA iw] RET imm16
    RetfIw,

    // [VEX.LZ.F2.0F3A.W0 F0 /r ib] RORX r32, r/m32, imm8
    RorxGdEdIbV,
    // [VEX.LZ.F2.0F3A.W1 F0 /r ib] RORX r64, r/m64, imm8
    RorxGqEqIbV,

    // [66 0F 3A 09 /r ib] ROUNDPD xmm1, xmm2/m128, imm8
    RoundpdVdqWdqIb,
    // [VEX.128.66.0F3A.WIG 09 /r ib] VROUNDPD xmm1, xmm2/m128, imm8
    VroundpdVdqWdqIbV128,
    // [VEX.256.66.0F3A.WIG 09 /r ib] VROUNDPD ymm1, ymm2/m256, imm8
    VroundpdVqqWqqIbV256,

    // [66 0F 3A 08 /r ib] ROUNDPS xmm1, xmm2/m128, imm8
    RoundpsVdqWdqIb,
    // [VEX.128.66.0F3A.WIG 08 /r ib] VROUNDPS xmm1, xmm2/m128, imm8
    VroundpsVdqWdqIbV128,
    // [VEX.256.66.0F3A.WIG 08 /r ib] VROUNDPS ymm1, ymm2/m256, imm8
    VroundpsVqqWqqIbV256,

    // [66 0F 3A 0B /r ib] ROUNDSD xmm1, xmm2/m64, imm8
    RoundsdVdqWq,
    // [VEX.LIG.66.0F3A.WIG 0B /r ib] VROUNDSD xmm1, xmm2, xmm3/m64, imm8
    VroundsdVdqHdqWqIbV,

    // [66 0F 3A 0B /r ib] ROUNDSS xmm1, xmm2/m32, imm8
    RoundssVdqWd,
    // [VEX.LIG.66.0F3A.WIG 0B /r ib] VROUNDSS xmm1, xmm2, xmm3/m32, imm8
    VroundssVdqHdqWdIbV,

    // [0F AA] RSM
    Rsm,

    // [NP 0F 52 /r] RSQRTPS xmm1, xmm2/m128
    RsqrtpsVdqWdq,
    // [VEX.128.0F.WIG 52 /r] VRSQRTPS xmm1, xmm2/m128
    VrsqrtpsVdqWdqV128,
    // [VEX.256.0F.WIG 52 /r] VRSQRTPS ymm1, ymm2/m256
    VrsqrtpsVqqWqqV256,

    // [F3 0F 52 /r] RSQRTSS xmm1, xmm2/m32
    RsqrtssVdqWd,
    // [VEX.LIG.F3.0F.WIG 52 /r] VRSQRTSS xmm1, xmm2, xmm3/m32
    VrsqrtssVdqHdqWdV,

    // [F3 0F 01 /5 (mod!=11, /5, mem-only)] RSTORSSP m64
    RstorsspMq,

    // [9E] SAHF
    Sahf,

    // [D0 /4] SAL r/m8, 1
    // [REX D0 /4] SAL r/m8, 1
    // [D0 /4] SHL r/m8, 1
    // [REX D0 /4] SHL r/m8, 1
    SalEb1,
    // [D2 /4] SAL r/m8, CL
    // [REX D2 /4] SAL r/m8, CL
    // [D2 /4] SHL r/m8, CL
    // [REX D2 /4] SHL r/m8, CL
    SalEbCL,
    // [C0 /4 ib] SAL r/m8, imm8
    // [REX C0 /4 ib] SAL r/m8, imm8
    // [C0 /4 ib] SHL r/m8, imm8
    // [REX C0 /4 ib] SHL r/m8, imm8
    SalEbIb,
    // [D1 /4] SAL r/m16, 1
    // [D1 /4] SAL r/m16, 1
    SalEw1,
    // [D3 /4] SAL r/m16, CL
    // [D3 /4] SHL r/m16, CL
    SalEwCL,
    // [C1 /4 ib] SAL r/m32, imm8
    // [C1 /4 ib] SHL r/m32, imm8
    SalEdIb,
    // [D1 /4] SAL r/m32, 1
    // [D1 /4] SHL r/m32, 1
    SalEd1,
    // [REX.W D1 /4] SAL r/m64, 1
    // [REX.W D1 /4] SHL r/m64, 1
    SalEq1,
    // [D3 /4] SAL r/m32, CL
    // [D3 /4] SHL r/m32, CL
    SalEdCL,
    // [REX.W D3 /4] SAL r/m64, CL
    // [REX.W D3 /4] SHL r/m64, CL
    SalEqCL,
    // [REX.W C1 /4 ib] SAL r/m64, imm8
    // [REX.W C1 /4 ib] SHL r/m64, imm8
    SalEqIb,
    // [D0 /7] SAR r/m8, 1
    SarEb1,
    // [D2 /7] SAR r/m8, CL
    // [REX D2 /7] SAR r/m8, CL
    SarEbCL,
    // [C0 /7 ib] SAR r/m8, imm8
    // [REX C0 /7 ib] SAR r/m8, imm8
    SarEbIb,
    // [D1 /7] SAR r/m16, 1
    SarEw1,
    // [D3 /7] SAR r/m16, CL
    SarEwCL,
    // [C1 /7 ib] SAR r/m32, imm8
    SarEdIb,
    // [D1 /7] SAR r/m32, 1
    SarEd1,
    // [REX.W D1 /7] SAR r/m64, 1
    SarEq1,
    // [D3 /7] SAR r/m32, CL
    SarEdCL,
    // [REX.W D3 /7] SAR r/m64, CL
    SarEqCL,
    // [REX.W C1 /7 ib] SAR r/m64, imm8
    SarEqIb,
    // [D0 /5] SHR r/m8, 1
    // [REX D0 /5] SHR r/m8, 1
    ShrEb1,
    // [D2 /5] SHR r/m8, CL
    // [REX D2 /5] SHR r/m8, CL
    ShrEbCL,
    // [C0 /5 ib] SHR r/m8, imm8
    // [REX C0 /5 ib] SHR r/m8, imm8
    ShrEbIb,
    // [D1 /5] SHR r/m16, 1
    ShrEw1,
    // [D3 /5] SHR r/m16, CL
    ShrEwCL,
    // [C1 /5 ib] SHR r/m32, imm8
    ShrEdIb,
    // [D1 /5] SHR r/m32, 1
    ShrEd1,
    // [REX.W D1 /5] SHR r/m64, 1
    ShrEq1,
    // [D3 /5] SHR r/m32, CL
    ShrEdCL,
    // [REX.W D3 /5] SHR r/m64, CL
    ShrEqCL,
    // [REX.W C1 /5 ib] SHR r/m64, imm8
    ShrEqIb,

    // [VEX.LZ.F3.0F38.W0 F7 /r] SARX r32a, r/m32, r32b
    SarxGdEdBd,
    // [VEX.LZ.66.0F38.W0 F7 /r] SHLX r32a, r/m32, r32b
    ShlxGdEdBd,
    // [VEX.LZ.F2.0F38.W0 F7 /r] SHRX r32a, r/m32, r32b
    ShrxGdEdBd,
    // [VEX.LZ.F3.0F38.W1 F7 /r] SARX r64a, r/m64, r64b
    SarxGqEqBq,
    // [VEX.LZ.66.0F38.W1 F7 /r] SHLX r64a, r/m64, r64b
    ShlxGqEqBq,
    // [VEX.LZ.F2.0F38.W1 F7 /r] SHRX r64a, r/m64, r64b
    ShrxGqEqBq,

    // [F3 0F 01 EA (mod!=11, /5, rm=010)] SAVEPREVSSP
    Saveprevssp,

    // [1C ib] SBB AL, imm8
    SbbALIb,
    // [1D iw] SBB AX, imm16
    SbbAXIw,
    // [1D id] SBB EAX, imm32
    SbbEAXId,
    // [REX.W 1D id] SBB RAX, imm32
    SbbRAXId,
    // [80 /3 ib] SBB r/m8, imm8
    // [REX 80 /3 ib] SBB r/m8, imm8
    SbbEbIb,
    // [81 /3 iw] SBB r/m16, imm16
    SbbEwIw,
    // [81 /3 id] SBB r/m32, imm32
    SbbEdId,
    // [REX.W 81 /3 id] SBB r/m64, imm32
    SbbEqId,
    // [83 /3 ib] SBB r/m16, imm8
    SbbEwIb,
    // [83 /3 ib] SBB r/m32, imm8
    SbbEdIb,
    // [REX.W 83 /3 ib] SBB r/m64, imm8
    SbbEqIb,
    // [18 /r] SBB r/m8, r8
    SbbEbGb,
    // [19 /r] SBB r/m16, r16
    SbbEwGw,
    // [19 /r] SBB r/m32, r32
    SbbEdGd,
    // [REX.W 19 /r] SBB r/m64, r64
    SbbEqGq,
    // [1A /r] SBB r8, r/m8
    // [REX 1A /r] SBB r8, r/m8
    SbbGbEb,
    // [1B /r] SBB r16, r/m16
    SbbGwEw,
    // [1B /r] SBB r32, r/m32
    SbbGdEd,
    // [REX.W 1B /r] SBB r64, r/m64
    SbbGqEq,

    // [AE] SCAS m8
    // [AE] SCASB
    ScasALYb,
    // [AF] SCAS m16
    // [AF] SCASW
    ScasAXYw,
    // [AF] SCAS m32
    // [AF] SCASD
    ScasEAXYd,
    // [REX.W AF] SCAS m64
    // [REX.W AF] SCASQ
    ScasRAXYq,

    // [0F 90] SETO r/m8
    // [REX 0F 90] SETO r/m8
    SetoEb,
    // [0F 91] SETNO r/m8
    // [REX 0F 91] SETNO r/m8
    SetnoEb,
    // [0F 92] SETB r/m8
    // [REX 0F 92] SETB r/m8
    // [0F 92] SETC r/m8
    // [REX 0F 92] SETC r/m8
    // [0F 92] SETAE r/m8
    // [REX 0F 92] SETAE r/m8
    SetbEb,
    // [0F 93] SETAE r/m8
    // [REX 0F 93] SETAE r/m8
    // [0F 93] SETNB r/m8
    // [REX 0F 93] SETNB r/m8
    // [0F 93] SETNC r/m8
    // [REX 0F 93] SETNC r/m8
    SetaeEb,
    // [0F 94] SETE r/m8
    // [REX 0F 94] SETE r/m8
    // [0F 94] SETZ r/m8
    // [REX 0F 94] SETZ r/m8
    SeteEb,
    // [0F 95] SETNE r/m8
    // [REX 0F 95] SETNE r/m8
    // [0F 95] SETNZ r/m8
    // [REX 0F 95] SETNZ r/m8
    SetneEb,
    // [0F 96] SETBE r/m8
    // [REX 0F 96] SETBE r/m8
    // [0F 96] SETNA r/m8
    // [REX 0F 96] SETNA r/m8
    SetbeEb,
    // [0F 97] SETA r/m8
    // [REX 0F 97] SETA r/m8
    // [0F 97] SETNBE r/m8
    // [REX 0F 97] SETNBE r/m8
    SetaEb,
    // [0F 98] SETS r/m8
    // [REX 0F 98] SETS r/m8
    SetsEb,
    // [0F 99] SETNS r/m8
    // [REX 0F 99] SETNS r/m8
    SetnsEb,
    // [0F 9A] SETP r/m8
    // [REX 0F 9A] SETP r/m8
    // [0F 9A] SETPE r/m8
    // [REX 0F 9A] SETPE r/m8
    SetpEb,
    // [0F 9B] SETNP r/m8
    // [REX 0F 9B] SETNP r/m8
    // [0F 9B] SETPO r/m8
    // [REX 0F 9B] SETPO r/m8
    SetnpEb,
    // [0F 9C] SETL r/m8
    // [REX 0F 9C] SETL r/m8
    // [0F 9C] SETNGE r/m8
    // [REX 0F 9C] SETNGE r/m8
    SetlEb,
    // [0F 9D] SETGE r/m8
    // [REX 0F 9D] SETGE r/m8
    // [0F 9D] SETNL r/m8
    // [REX 0F 9D] SETNL r/m8
    SetgeEb,
    // [0F 9E] SETLE r/m8
    // [REX 0F 9E] SETLE r/m8
    // [0F 9E] SETNG r/m8
    // [REX 0F 9E] SETNG r/m8
    SetleEb,
    // [0F 9F] SETG r/m8
    // [REX 0F 9F] SETG r/m8
    // [0F 9F] SETNLE r/m8
    // [REX 0F 9F] SETNLE r/m8
    SetgEb,

    // [F3 0F 01 E8] SETSSBSY
    Setssbsy,

    // [NP 0F AE F8] SFENCE
    Sfence,

    // [0F 01 /0] SGDT mem
    SgdtM,

    // [NP 0F 3A CC /r ib] SHA1RNDS4 xmm1, xmm2/m128, imm8
    Sha1rnds4VdqWdqIb,

    // [NP 0F 38 C8 /r] SHA1NEXTE xmm1, xmm2/m128
    Sha1nexteVdqWdq,

    // [NP 0F 38 C9 /r] SHA1MSG1 xmm1, xmm2/m128
    Sha1msg1VdqWdq,

    // [NP 0F 38 CA /r] SHA1MSG2 xmm1, xmm2/m128
    Sha1msg2VdqWdq,

    // [NP 0F 38 CB /r] SHA256RNDS2 xmm1, xmm2/m128, <XMM0>
    Sha256rnds2VdqWdq,

    // [NP 0F 38 CC /r] SHA256MSG1 xmm1, xmm2/m128
    Sha256msg1VdqWdq,

    // [NP 0F 38 CD /r] SHA256MSG2 xmm1, xmm2/m128
    Sha256msg2VdqWdq,

    // [0F A4 /r ib] SHLD r/m16, r16, imm8
    ShldEwGwIb,
    // [0F A5 /r] SHLD r/m16, r16, CL
    ShldEwGwCL,
    // [0F A4 /r ib] SHLD r/m32, r32, imm8
    ShldEdGdIb,
    // [0F A5 /r] SHLD r/m32, r32, CL
    ShldEdGdCL,
    // [REX.W 0F A4 /r ib] SHLD r/m64, r64, imm8
    ShldEqGqIb,
    // [REX.W 0F A5 /r] SHLD r/m64, r64, CL
    ShldEqGqCL,

    // [0F AC /r ib] SHRD r/m16, r16, imm8
    ShrdEwGwIb,
    // [0F AD /r] SHRD r/m16, r16, CL
    ShrdEwGwCL,
    // [0F AC /r ib] SHRD r/m32, r32, imm8
    ShrdEdGdIb,
    // [0F AD /r] SHRD r/m32, r32, CL
    ShrdEdGdCL,
    // [REX.W 0F AC /r ib] SHRD r/m64, r64, imm8
    ShrdEqGqIb,
    // [REX.W 0F AD /r] SHRD r/m64, r64, CL
    ShrdEqGqCL,

    // [66 0F C6 /r ib] SHUFPD xmm1, xmm2/m128, imm8
    ShufpdVdqWdqIb,
    // [VEX.128.66.0F.WIG C6 /r ib] VSHUFPD xmm1, xmm2, xmm3/m128, imm8
    VshufpdVdqHdqWdqIbV128,
    // [VEX.256.66.0F.WIG C6 /r ib] VSHUFPD ymm1, ymm2, ymm3/m256, imm8
    VshufpdVqqHqqWqqIbV256,
    // [EVEX.128.66.0F.W1 C6 /r ib] VSHUFPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    VshufpdVdqHdqWdqIbE128,
    // [EVEX.256.66.0F.W1 C6 /r ib] VSHUFPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    VshufpdVqqHqqWqqIbE256,
    // [EVEX.512.66.0F.W1 C6 /r ib] VSHUFPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    VshufpdVdqqHdqqWdqqIbE512,

    // [NP 0F C6 /r ib] SHUFPS xmm1, xmm2/m128, imm8
    ShufpsVdqWdqIb,
    // [VEX.128.0F.WIG C6 /r ib] VSHUFPS xmm1, xmm2, xmm3/m128, imm8
    VshufpsVdqHdqWdqIbV128,
    // [VEX.256.0F.WIG C6 /r ib] VSHUFPS ymm1, ymm2, ymm3/m256, imm8
    VshufpsVqqHqqWqqIbV256,
    // [EVEX.128.0F.W0 C6 /r ib] VSHUFPS xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    VshufpsVdqHdqWdqIbE128,
    // [EVEX.256.0F.W0 C6 /r ib] VSHUFPS ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    VshufpsVqqHqqWqqIbE256,
    // [EVEX.512.0F.W0 C6 /r ib] VSHUFPS zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    VshufpsVdqqHdqqWdqqIbE512,

    // [0F 01 /1] SIDT mem
    SidtM,

    // [0F 00 /0] SLDT r/m16
    SldtEw,
    // [REX.W 0F 00 /0] SLDT r64/m16
    SldtEq,

    // [0F 01 /4] SMSW r/m16
    SmswEw,
    // [0F 01 /4] SMSW r32/m16
    SmswEd,
    // [REX.W 0F 01 /4] SMSW r64/m16
    SmswEq,

    // [66 0F 51 /r] SQRTPD xmm1, xmm2/m128
    SqrtpdVdqWdq,
    // [VEX.128.66.0F.WIG 51 /r] VSQRTPD xmm1, xmm2/m128
    VsqrtpdVdqWdqV128,
    // [VEX.256.66.0F.WIG 51 /r] VSQRTPD ymm1, ymm2/m256
    VsqrtpdVqqWqqV256,
    // [EVEX.128.66.0F.W1 51 /r] VSQRTPD xmm1 {k1}{z}, xmm2/m128/m64bcst
    VsqrtpdVdqWdqE128,
    // [EVEX.256.66.0F.W1 51 /r] VSQRTPD ymm1 {k1}{z}, ymm2/m256/m64bcst
    VsqrtpdVqqWqqE256,
    // [EVEX.512.66.0F.W1 51 /r] VSQRTPD zmm1 {k1}{z}, zmm2/m512/m64bcst{er}
    VsqrtpdVdqqWdqqE512,

    // [NP 0F 51 /r] SQRTPS xmm1, xmm2/m128
    SqrtpsVdqWdq,
    // [VEX.128.0F.WIG 51 /r] VSQRTPS xmm1, xmm2/m128
    VsqrtpsVdqWdqV128,
    // [VEX.256.0F.WIG 51 /r] VSQRTPS ymm1, ymm2/m256
    VsqrtpsVqqWqqV256,
    // [EVEX.128.0F.W1 51 /r] VSQRTPS xmm1 {k1}{z}, xmm2/m128/m32bcst
    VsqrtpsVdqWdqE128,
    // [EVEX.256.0F.W1 51 /r] VSQRTPS ymm1 {k1}{z}, ymm2/m256/m32bcst
    VsqrtpsVqqWqqE256,
    // [EVEX.512.0F.W1 51 /r] VSQRTPS zmm1 {k1}{z}, zmm2/m512/m32bcst{er}
    VsqrtpsVdqqWdqqE512,

    // [F2 0F 51 /r] SQRTSD xmm1, xmm2/m64
    SqrtsdVdqWq,
    // [VEX.LIG.F2.0F.WIG 51 /r] VSQRTSD xmm1, xmm2, xmm3/m64
    VsqrtsdVdqHdqWqV,
    // [EVEX.LIG.F2.0F.W1 51 /r] VSQRTSD xmm1 {k1}{z}, xmm2, xmm3/m64{er}
    VsqrtsdVdqHdqWqE,

    // [F2 0F 51 /r] SQRTSS xmm1, xmm2/m64
    SqrtssVdqWq,
    // [VEX.LIG.F2.0F.WIG 51 /r] VSQRTSS xmm1, xmm2, xmm3/m32
    VsqrtssVdqHdqWdV,
    // [EVEX.LIG.F2.0F.W1 51 /r] VSQRTSS xmm1 {k1}{z}, xmm2, xmm3/m32{er}
    VsqrtssVdqHdqWdE,

    // [NP 0F 01 CB] STAC
    Stac,

    // [F9] STC
    Stc,

    // [FD] STD
    Std,

    // [FB] STI
    Sti,

    // [NP 0F AE /3] STMXCSR m32
    StmxcsrMd,
    // [VEX.LZ.0F.WIG AE /3] VSTMXCSR m32
    VstmxcsrMdV,

    // [AA] STOS m8
    // [AA] STOSB
    StosYbAL,
    // [AB] STOS m16
    // [AB] STOSW
    StosYwAX,
    // [AB] STOS m32
    // [AB] STOSD
    StosYdEAX,
    // [REX.W AB] STOS m64
    // [REX.W AB] STOSQ
    StosYqRAX,

    // [0F 00 /1] STR r/m16
    StrEw,

    // [2C ib] SUB AL, imm8
    SubALIb,
    // [2D iw] SUB AX, imm16
    SubAXIw,
    // [2D id] SUB EAX, imm32
    SubEAXId,
    // [REX.W 2D id] SUB RAX, imm32
    SubRAXId,
    // [80 /5 ib] SUB r/m8, imm8
    // [REX 80 /5 ib] SUB r/m8, imm8
    SubEbIb,
    // [81 /5 iw] SUB r/m16, imm16
    SubEwIw,
    // [81 /5 id] SUB r/m32, imm32
    SubEdId,
    // [REX.W 81 /5 id] SUB r/m64, imm32
    SubEqId,
    // [83 /5 ib] SUB r/m16, imm8
    SubEwIb,
    // [83 /5 ib] SUB r/m32, imm8
    SubEdIb,
    // [REX.W 83 /5 ib] SUB r/m64, imm8
    SubEqIb,
    // [28 /r] SUB r/m8, r8
    // [REX 28 /r] SUB r/m8, r8
    SubEbGb,
    // [29 /r] SUB r/m16, r16
    SubEwGw,
    // [29 /r] SUB r/m32, r32
    SubEdGd,
    // [REX.W 29 /r] SUB r/m64, r64
    SubEqGq,
    // [2A /r] SUB r8, r/m8
    // [REX 2A /r] SUB r8, r/m8
    SubGbEb,
    // [2B /r] SUB r16, r/m16
    SubGwEw,
    // [2B /r] SUB r32, r/m32
    SubGdEd,
    // [REX.W 2B /r] SUB r64, r/m64
    SubGqEq,

    // [66 0F 5C /r] SUBPD xmm1, xmm2/m128
    SubpdVdqWdq,
    // [VEX.128.66.0F.WIG 5C /r] VSUBPD xmm1, xmm2, xmm3/m128
    VsubpdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 5C /r] VSUBPD ymm1, ymm2, ymm3/m256
    VsubpdVqqHqqWqqV256,
    // [EVEX.128.66.0F.W1 5C /r] VSUBPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VsubpdVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 5C /r] VSUBPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VsubpdVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 5C /r] VSUBPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VsubpdVdqqHdqqWdqqE512,

    // [NP 0F 5C /r] SUBPS xmm1, xmm2/m128
    SubpsVdqWdq,
    // [VEX.128.0F.WIG 5C /r] VSUBPS xmm1, xmm2, xmm3/m128
    VsubpsVdqHdqWdqV128,
    // [VEX.256.0F.WIG 5C /r] VSUBPS ymm1, ymm2, ymm3/m256
    VsubpsVqqHqqWqqV256,
    // [EVEX.128.0F.W1 5C /r] VSUBPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VsubpsVdqHdqWdqE128,
    // [EVEX.256.0F.W1 5C /r] VSUBPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VsubpsVqqHqqWqqE256,
    // [EVEX.512.0F.W1 5C /r] VSUBPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VsubpsVdqqHdqqWdqqE512,

    // [F2 0F 5C /r] SUBSD xmm1, xmm2/m64
    SubsdVdqWq,
    // [VEX.LIG.F2.0F.WIG 5C /r] VSUBSD xmm1, xmm2, xmm3/m64
    VsubsdVdqHdqWqV,
    // [EVEX.LIG.F2.0F.W1 5C /r] VSUBSD xmm1 {k1}{z}, xmm2, xmm3/m64{er}
    VsubsdVdqHdqWqE,

    // [F3 0F 5C /r] SUBSS xmm1, xmm2/m32
    SubssVdqWd,
    // [VEX.LIG.F2.0F.WIG 5C /r] VSUBSS xmm1, xmm2, xmm3/m32
    VsubssVdqHdqWdV,
    // [EVEX.LIG.F2.0F.W1 5C /r] VSUBSS xmm1 {k1}{z}, xmm2, xmm3/m32{er}
    VsubssVdqHdqWdE,

    // [0F 01 F8] SWAPGS
    Swapgs,

    // [0F 05] SYSCALL
    Syscall,

    // [0F 34] SYSENTER
    Sysenter,

    // [0F 35] SYSEXIT
    // [REX.W 0F 35] SYSEXIT
    Sysexit,

    // [0F 07] SYSRET
    // [REX.W 0F 07] SYSRET
    Sysret,

    // [A8 ib] TEST AL, imm8
    TestALIb,
    // [A9 iw] TEST AX, imm16
    TestAXIw,
    // [A9 id] TEST EAX, imm32
    TestEAXId,
    // [REX.W A9 id] TEST RAX, imm32
    TestRAXId,
    // [F6 /0 ib] TEST r/m8, imm8
    // [REX F6 /0 ib] TEST r/m8, imm8
    TestEbIb,
    // [F7 /0 iw] TEST r/m16, imm16
    TestEwIw,
    // [F7 /0 id] TEST r/m32, imm32
    TestEdId,
    // [REX.W F7 /0 id] TEST r/m64, imm32
    TestEqId,
    // [84 /r] TEST r/m8, r8
    // [REX 84 /r] TEST r/m8, r8
    TestEbGb,
    // [85 /r] TEST r/m16, r16
    TestEwGw,
    // [85 /r] TEST r/m32, r32
    TestEdGd,
    // [REX.W 85 /r] TEST r/m64, r64
    TestEqGq,

    // [66 0F AE /6] TPAUSE r32
    TpauseGd,

    // [F3 0F BC /r] TZCNT r16, r/m16
    TzcntGwEw,
    // [F3 0F BC /r] TZCNT r32, r/m32
    TzcntGdEd,
    // [F3 REX.W 0F BC /r] TZCNT r64, r/m64
    TzcntGqEq,

    // [66 0F 2E /r] UCOMISD xmm1, xmm2/m64
    UcomisdVdqWq,
    // [VEX.LIG.66.0F.WIG 2E /r] VUCOMISD xmm1, xmm2/m64
    VucomisdVdqWqV,
    // [EVEX.LIG.66.0F.W1 2E /r] VUCOMISD xmm1, xmm2/m64{sae}
    VucomisdVdqWqE,

    // [NP 0F 2E /r] UCOMISS xmm1, xmm2/m32
    UcomissVdqWd,
    // [VEX.LIG.0F.WIG 2E /r] VUCOMISS xmm1, xmm2/m32
    VucomissVdqWdV,
    // [EVEX.LIG.0F.W1 2E /r] VUCOMISS xmm1, xmm2/m32{sae}
    VucomissVdqWdE,

    // [0F FF /r] UD0 r32, r/m32
    Ud0GdEd,
    // [0F B9 /r] UD1 r32, r/m32
    Ud1GdEd,
    // [0F 0B] UD2
    Ud2,

    // [F3 0F AE /6] UMONITOR r16/r32/r64
    UmonitorE,

    // [F2 0F AE /6] UMWAIT r32
    UmwaitGd,

    // [66 0F 15 /r] UNPCKHPD xmm1, xmm2/m128
    UnpckhpdVdqWdq,
    // [VEX.128.66.0F.WIG 15 /r] VUNPCKHPD xmm1, xmm2, xmm3/m128
    VunpckhpdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 15 /r] VUNPCKHPD ymm1, ymm2, ymm3/m256
    VunpckhpdVqqHqqWqqV256,
    // [EVEX.128.66.0F.W1 15 /r] VUNPCKHPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VunpckhpdVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 15 /r] VUNPCKHPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VunpckhpdVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 15 /r] VUNPCKHPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VunpckhpdVdqqHdqqWdqqE512,

    // [NP 0F 15 /r] UNPCKHPS xmm1, xmm2/m128
    UnpckhpsVdqWdq,
    // [VEX.128.0F.WIG 15 /r] VUNPCKHPS xmm1, xmm2, xmm3/m128
    VunpckhpsVdqHdqWdqV128,
    // [VEX.256.0F.WIG 15 /r] VUNPCKHPS ymm1, ymm2, ymm3/m256
    VunpckhpsVqqHqqWqqV256,
    // [EVEX.128.0F.W1 15 /r] VUNPCKHPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VunpckhpsVdqHdqWdqE128,
    // [EVEX.256.0F.W1 15 /r] VUNPCKHPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VunpckhpsVqqHqqWqqE256,
    // [EVEX.512.0F.W1 15 /r] VUNPCKHPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VunpckhpsVdqqHdqqWdqqE512,

    // [66 0F 14 /r] UNPCKLPD xmm1, xmm2/m128
    UnpcklpdVdqWdq,
    // [VEX.128.66.0F.WIG 14 /r] VUNPCKLPD xmm1, xmm2, xmm3/m128
    VunpcklpdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 14 /r] VUNPCKLPD ymm1, ymm2, ymm3/m256
    VunpcklpdVqqHqqWqqV256,
    // [EVEX.128.66.0F.W1 14 /r] VUNPCKLPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VunpcklpdVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 14 /r] VUNPCKLPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VunpcklpdVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 14 /r] VUNPCKLPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VunpcklpdVdqqHdqqWdqqE512,

    // [NP 0F 14 /r] UNPCKLPS xmm1, xmm2/m128
    UnpcklpsVdqWdq,
    // [VEX.128.0F.WIG 14 /r] VUNPCKLPS xmm1, xmm2, xmm3/m128
    VunpcklpsVdqHdqWdqV128,
    // [VEX.256.0F.WIG 14 /r] VUNPCKLPS ymm1, ymm2, ymm3/m256
    VunpcklpsVqqHqqWqqV256,
    // [EVEX.128.0F.W1 14 /r] VUNPCKLPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VunpcklpsVdqHdqWdqE128,
    // [EVEX.256.0F.W1 14 /r] VUNPCKLPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VunpcklpsVqqHqqWqqE256,
    // [EVEX.512.0F.W1 14 /r] VUNPCKLPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VunpcklpsVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F3A.W0 03 /r ib] VALIGND xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst, imm8
    ValigndVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W0 03 /r ib] VALIGND ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst, imm8
    ValigndVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 03 /r ib] VALIGND zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst, imm8
    ValigndVdqqHdqqWdqqIbE256,
    // [EVEX.128.66.0F3A.W1 03 /r ib] VALIGNQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    ValignqVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 03 /r ib] VALIGNQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    ValignqVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 03 /r ib] VALIGNQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    ValignqVdqqHdqqWdqqIbE256,

    // [EVEX.128.66.0F38.W1 65 /r] VBLENDMPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VblendmpdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 65 /r] VBLENDMPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VblendmpdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 65 /r] VBLENDMPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VblendmpdVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W0 65 /r] VBLENDMPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VblendmpsVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 65 /r] VBLENDMPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VblendmpsVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 65 /r] VBLENDMPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VblendmpsVdqqHdqqWdqqE512,

    // [VEX.128.66.0F38.W0 18 /r] VBROADCASTSS xmm1, m32
    // [VEX.128.66.0F38.W0 18 /r] VBROADCASTSS xmm1, xmm2
    VbroadcastssVdqWdV128,
    // [VEX.256.66.0F38.W0 18 /r] VBROADCASTSS ymm1, m32
    // [VEX.128.66.0F38.W0 18 /r] VBROADCASTSS ymm1, xmm2
    VbroadcastssVqqWdV256,
    // [VEX.256.66.0F38.W0 19 /r] VBROADCASTSD ymm1, m64
    // [VEX.256.66.0F38.W0 19 /r] VBROADCASTSD ymm1, xmm2
    VbroadcastsdVqqWqV256,
    // [VEX.256.66.0F38.W0 1A /r] VBROADCASTF128 ymm1, m128
    Vbroadcastf128VqqMdqV256,
    // [EVEX.256.66.0F38.W1 19 /r] VBROADCASTSD ymm1 {k1}{z}, xmm2/m64
    VbroadcastsdVqqWqE256,
    // [EVEX.512.66.0F38.W1 19 /r] VBROADCASTSD zmm1 {k1}{z}, xmm2/m64
    VbroadcastsdVdqqWqE512,
    // [EVEX.256.66.0F38.W0 19 /r] VBROADCASTF32X2 ymm1 {k1}{z}, xmm2/m64
    Vbroadcastf32x2VqqWqE256,
    // [EVEX.512.66.0F38.W0 19 /r] VBROADCASTF32X2 zmm1 {k1}{z}, xmm2/m64
    Vbroadcastf32x2VdqqWqE512,
    // [EVEX.128.66.0F38.W0 18 /r] VBROADCASTSS xmm1 {k1}{z}, xmm2/m32
    VbroadcastssVdqWdE128,
    // [EVEX.256.66.0F38.W0 18 /r] VBROADCASTSS ymm1 {k1}{z}, xmm2/m32
    VbroadcastssVqqWdE256,
    // [EVEX.512.66.0F38.W0 18 /r] VBROADCASTSS zmm1 {k1}{z}, xmm2/m32
    VbroadcastssVdqqWdE512,
    // [EVEX.256.66.0F38.W0 1A /r] VBROADCASTF32X4 ymm1 {k1}{z}, m128
    Vbroadcastf32x4VqqWdqE256,
    // [EVEX.512.66.0F38.W0 1A /r] VBROADCASTF32X4 zmm1 {k1}{z}, m128
    Vbroadcastf32x4VdqqWdqE512,
    // [EVEX.256.66.0F38.W1 1A /r] VBROADCASTF64X2 ymm1 {k1}{z}, m128
    Vbroadcastf64x2VqqWdqE256,
    // [EVEX.512.66.0F38.W1 1A /r] VBROADCASTF64X2 zmm1 {k1}{z}, m128
    Vbroadcastf64x2VdqqWdqE512,
    // [EVEX.512.66.0F38.W0 1B /r] VBROADCASTF32X8 zmm1 {k1}{z}, m256
    Vbroadcastf32x8VdqqWqqE512,
    // [EVEX.512.66.0F38.W1 1B /r] VBROADCASTF64X4 zmm1 {k1}{z}, m256
    Vbroadcastf64x4VdqqWqqE512,

    // [EVEX.128.66.0F38.W1 8A /r] VCOMPRESSPD xmm1/m128 {k1}{z}, xmm2
    VcompresspdWdqVdqE128,
    // [EVEX.256.66.0F38.W1 8A /r] VCOMPRESSPD ymm1/m256 {k1}{z}, xmm2
    VcompresspdWqqVqqE256,
    // [EVEX.512.66.0F38.W1 8A /r] VCOMPRESSPD zmm1/m512 {k1}{z}, xmm2
    VcompresspdWdqqVdqqE512,

    // [EVEX.128.66.0F38.W0 8A /r] VCOMPRESSPS xmm1/m128 {k1}{z}, xmm2
    VcompresspsWdqVdqE128,
    // [EVEX.256.66.0F38.W0 8A /r] VCOMPRESSPS ymm1/m256 {k1}{z}, xmm2
    VcompresspsWqqVqqE256,
    // [EVEX.512.66.0F38.W0 8A /r] VCOMPRESSPS zmm1/m512 {k1}{z}, xmm2
    VcompresspsWdqqVdqqE512,

    // [EVEX.128.66.0F.W1 7B /r] VCVTPD2QQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    Vcvtpd2qqVdqWdqE128,
    // [EVEX.256.66.0F.W1 7B /r] VCVTPD2QQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    Vcvtpd2qqVqqWqqE256,
    // [EVEX.512.66.0F.W1 7B /r] VCVTPD2QQ zmm1 {k1}{z}, zmm2/m512/m64bcst
    Vcvtpd2qqVdqqWdqqE512,

    // [EVEX.128.0F.W1 79 /r] VCVTPD2UDQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    Vcvtpd2udqVdqWdqE128,
    // [EVEX.256.0F.W1 79 /r] VCVTPD2UDQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    Vcvtpd2udqVqqWqqE256,
    // [EVEX.512.0F.W1 79 /r] VCVTPD2UDQ zmm1 {k1}{z}, zmm2/m512/m64bcst
    Vcvtpd2udqVdqqWdqqE512,

    // [EVEX.128.66.0F.W1 79 /r] VCVTPD2UQQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    Vcvtpd2uqqVdqWdqE128,
    // [EVEX.256.66.0F.W1 79 /r] VCVTPD2UQQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    Vcvtpd2uqqVqqWqqE256,
    // [EVEX.512.66.0F.W1 79 /r] VCVTPD2UQQ zmm1 {k1}{z}, zmm2/m512/m64bcst
    Vcvtpd2uqqVdqqWdqqE512,

    // [VEX.128.66.0F38.W0 13 /r] VCVTPH2PS xmm1, xmm2/m64
    Vcvtph2psVdqWqV128,
    // [VEX.256.66.0F38.W0 13 /r] VCVTPH2PS ymm1, xmm2/m128
    Vcvtph2psVqqWdqV256,
    // [EVEX.128.66.0F38.W0 13 /r] VCVTPH2PS xmm1 {k1}{z}, xmm2/m64
    Vcvtph2psVdqWqE128,
    // [EVEX.256.66.0F38.W0 13 /r] VCVTPH2PS ymm1 {k1}{z}, xmm2/m128
    Vcvtph2psVqqWdqE256,
    // [EVEX.512.66.0F38.W0 13 /r] VCVTPH2PS zmm1 {k1}{z}, ymm2/m256{sae}
    Vcvtph2psVdqqWqqE512,

    // [VEX.128.66.0F3A.W0 1D /r ib] VCVTPS2PH xmm1/m64, xmm2, imm8
    Vcvtps2phWqVdqIbV128,
    // [VEX.256.66.0F3A.W0 1D /r ib] VCVTPS2PH xmm1/m128, ymm2, imm8
    Vcvtps2phWdqVqqIbV256,
    // [EVEX.128.66.0F3A.W0 1D /r ib] VCVTPS2PH xmm1/m64 {k1}{z}, xmm2, imm8
    Vcvtps2phWqVdqIbE128,
    // [EVEX.256.66.0F3A.W0 1D /r ib] VCVTPS2PH xmm1/m128 {k1}{z}, ymm2, imm8
    Vcvtps2phWdqVqqIbE256,
    // [EVEX.512.66.0F3A.W0 1D /r ib] VCVTPS2PH ymm1/m256 {k1}{z}, zmm2{sae}, imm8
    Vcvtps2phWqqVdqqIbE512,

    // [EVEX.128.0F.W0 79 /r] VCVTPS2UDQ xmm1 {k1}{z}, xmm2/m128/m32bcst
    Vcvtps2udqVdqWdqE128,
    // [EVEX.256.0F.W0 79 /r] VCVTPS2UDQ ymm1 {k1}{z}, ymm2/m256/m32bcst
    Vcvtps2udqVqqWqqE256,
    // [EVEX.512.0F.W0 79 /r] VCVTPS2UDQ zmm1 {k1}{z}, zmm2/m512/m32bcst{er}
    Vcvtps2udqVdqqWdqqE512,

    // [EVEX.128.66.0F.W0 7B /r] VCVTPS2QQ xmm1 {k1}{z}, xmm2/m64/m32bcst
    Vcvtps2qqVdqWqE128,
    // [EVEX.256.66.0F.W0 7B /r] VCVTPS2QQ ymm1 {k1}{z}, xmm2/m128/m32bcst
    Vcvtps2qqVqqWdqE256,
    // [EVEX.512.66.0F.W0 7B /r] VCVTPS2QQ zmm1 {k1}{z}, ymm2/m256/m32bcst{er}
    Vcvtps2qqVdqqWqqE512,

    // [EVEX.128.66.0F.W0 79 /r] VCVTPS2UQQ xmm1 {k1}{z}, xmm2/m64/m32bcst
    Vcvtps2uqqVdqWqE128,
    // [EVEX.256.66.0F.W0 79 /r] VCVTPS2UQQ ymm1 {k1}{z}, xmm2/m128/m32bcst
    Vcvtps2uqqVqqWdqE256,
    // [EVEX.512.66.0F.W0 79 /r] VCVTPS2UQQ zmm1 {k1}{z}, ymm2/m256/m32bcst{er}
    Vcvtps2uqqVdqqWqqE512,

    // [EVEX.128.F3.0F.W1 E6 /r] VCVTQQ2PD xmm1 {k1}{z}, xmm2/m128/m64bcst
    Vcvtqq2pdVdqWdqE128,
    // [EVEX.256.F3.0F.W1 E6 /r] VCVTQQ2PD ymm1 {k1}{z}, ymm2/m256/m64bcst
    Vcvtqq2pdVqqWqqE256,
    // [EVEX.512.F3.0F.W1 E6 /r] VCVTQQ2PD zmm1 {k1}{z}, zmm2/m512/m64bcst{er}
    Vcvtqq2pdVdqqWdqqE512,

    // [EVEX.128.0F.W1 5B /r] VCVTQQ2PS xmm1 {k1}{z}, xmm2/m128/m64bcst
    Vcvtqq2psVdqWdqE128,
    // [EVEX.256.0F.W1 5B /r] VCVTQQ2PS ymm1 {k1}{z}, ymm2/m256/m64bcst
    Vcvtqq2psVqqWqqE256,
    // [EVEX.512.0F.W1 5B /r] VCVTQQ2PS zmm1 {k1}{z}, zmm2/m512/m64bcst{er}
    Vcvtqq2psVdqqWdqqE512,

    // [EVEX.LIG.F2.0F.W0 79 /r] VCVTSD2USI r32, xmm1/m64{er}
    Vcvtsd2usiGdVqE,
    // [EVEX.LIG.F2.0F.W1 79 /r] VCVTSD2USI r64, xmm1/m64{er}
    Vcvtsd2usiGqVqE,

    // [EVEX.LIG.F3.0F.W0 79 /r] VCVTSS2USI r32, xmm1/m32{er}
    Vcvtss2usiGdVdE,
    // [EVEX.LIG.F3.0F.W1 79 /r] VCVTSS2USI r64, xmm1/m32{er}
    Vcvtss2usiGqVdE,

    // [EVEX.128.66.0F.W1 7A /r] VCVTTPD2QQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    Vcvttpd2qqVdqWdqE128,
    // [EVEX.256.66.0F.W1 7A /r] VCVTTPD2QQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    Vcvttpd2qqVqqWqqE256,
    // [EVEX.512.66.0F.W1 7A /r] VCVTTPD2QQ zmm1 {k1}{z}, zmm2/m512/m64bcst{sae}
    Vcvttpd2qqVdqqWdqqE512,

    // [EVEX.128.0F.W1 78 /r] VCVTTPD2UDQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    Vcvttpd2udqVdqWdqE128,
    // [EVEX.256.0F.W1 78 /r] VCVTTPD2UDQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    // NOTE: Intel manual lists the opcode as `... 78 02 /r`
    Vcvttpd2udqVqqWqqE256,
    // [EVEX.512.0F.W1 78 /r] VCVTTPD2UDQ zmm1 {k1}{z}, zmm2/m512/m64bcst{sae}
    Vcvttpd2udqVdqqWdqqE512,

    // [EVEX.128.66.0F.W1 78 /r] VCVTTPD2UQQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    Vcvttpd2uqqVdqWdqE128,
    // [EVEX.256.66.0F.W1 78 /r] VCVTTPD2UQQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    Vcvttpd2uqqVqqWqqE256,
    // [EVEX.512.66.0F.W1 78 /r] VCVTTPD2UQQ zmm1 {k1}{z}, zmm2/m512/m64bcst{sae}
    Vcvttpd2uqqVdqqWdqqE512,

    // [EVEX.128.66.0F.W1 7A /r] VCVTTPS2QQ xmm1 {k1}{z}, xmm2/m64/m32bcst
    Vcvttps2qqVdqWqE128,
    // [EVEX.256.66.0F.W1 7A /r] VCVTTPS2QQ ymm1 {k1}{z}, ymm2/m128/m32bcst
    Vcvttps2qqVqqWdqE256,
    // [EVEX.512.66.0F.W1 7A /r] VCVTTPS2QQ zmm1 {k1}{z}, zmm2/m256/m32bcst{sae}
    Vcvttps2qqVdqqWqqE512,

    // [EVEX.128.66.0F.W1 78 /r] VCVTTPS2UQQ xmm1 {k1}{z}, xmm2/m64/m32bcst
    Vcvttps2uqqVdqWqE128,
    // [EVEX.256.66.0F.W1 78 /r] VCVTTPS2UQQ ymm1 {k1}{z}, ymm2/m128/m32bcst
    Vcvttps2uqqVqqWdqE256,
    // [EVEX.512.66.0F.W1 78 /r] VCVTTPS2UQQ zmm1 {k1}{z}, zmm2/m256/m32bcst{sae}
    Vcvttps2uqqVdqqWqqE512,

    // [EVEX.LIG.F2.0F.W0 78 /r] VCVTTSD2USI r32, xmm1/m64{sae}
    Vcvttsd2usiGdWqE,
    // [EVEX.LIG.F2.0F.W1 78 /r] VCVTTSD2USI r64, xmm1/m64{sae}
    Vcvttsd2usiGqWqE,

    // [EVEX.LIG.F3.0F.W0 78 /r] VCVTTSS2USI r32, xmm1/m64{sae}
    Vcvttss2usiGdWqE,
    // [EVEX.LIG.F3.0F.W1 78 /r] VCVTTSS2USI r64, xmm1/m64{sae}
    Vcvttss2usiGqWqE,

    // [EVEX.128.F3.0F.W0 7A /r] VCVTUDQ2PD xmm1 {k1}{z}, xmm2/m64/m32bcst
    Vcvtudq2pdVdqWqE128,
    // [EVEX.256.F3.0F.W0 7A /r] VCVTUDQ2PD ymm1 {k1}{z}, xmm2/m128/m32bcst
    Vcvtudq2pdVqqWdqE256,
    // [EVEX.512.F3.0F.W0 7A /r] VCVTUDQ2PD zmm1 {k1}{z}, ymm2/m256/m32bcst
    Vcvtudq2pdVdqqWqqE512,

    // [EVEX.128.F2.0F.W0 7A /r] VCVTUDQ2PS xmm1 {k1}{z}, xmm2/m128/m32bcst
    Vcvtudq2psVdqWdqE128,
    // [EVEX.256.F2.0F.W0 7A /r] VCVTUDQ2PS ymm1 {k1}{z}, xmm2/m256/m32bcst
    Vcvtudq2psVqqWqqE256,
    // [EVEX.512.F2.0F.W0 7A /r] VCVTUDQ2PS zmm1 {k1}{z}, ymm2/m512/m32bcst{er}
    Vcvtudq2psVdqqWdqqE512,

    // [EVEX.128.F3.0F.W1 7A /r] VCVTUQQ2PD xmm1 {k1}{z}, xmm2/m128/m64bcst
    Vcvtuqq2pdVdqWdqE128,
    // [EVEX.256.F3.0F.W1 7A /r] VCVTUQQ2PD ymm1 {k1}{z}, xmm2/m256/m64bcst
    Vcvtuqq2pdVqqWqqE256,
    // [EVEX.512.F3.0F.W1 7A /r] VCVTUQQ2PD zmm1 {k1}{z}, ymm2/m512/m64bcst{er]
    Vcvtuqq2pdVdqqWdqqE512,

    // [EVEX.128.F2.0F.W1 7A /r] VCVTUQQ2PS xmm1 {k1}{z}, xmm2/m128/m64bcst
    Vcvtuqq2psVdqWdqE128,
    // [EVEX.256.F2.0F.W1 7A /r] VCVTUQQ2PS ymm1 {k1}{z}, xmm2/m256/m64bcst
    Vcvtuqq2psVqqWqqE256,
    // [EVEX.512.F2.0F.W1 7A /r] VCVTUQQ2PS zmm1 {k1}{z}, ymm2/m512/m64bcst[er}
    Vcvtuqq2psVdqqWdqqE512,

    // [EVEX.LIG.F2.0F.W0 7B /r] VCVTUSI2SD xmm1, xmm2, r/m32
    Vcvtusi2sdVdqHdqWdE,
    // [EVEX.LIG.F2.0F.W1 7B /r] VCVTUSI2SD xmm1, xmm2, r/m64{er}
    Vcvtusi2sdVdqHdqWqE,

    // [EVEX.LIG.F3.0F.W0 7B /r] VCVTUSI2SS xmm1, xmm2, r/m32
    Vcvtusi2ssVdqHdqWdE,
    // [EVEX.LIG.F3.0F.W1 7B /r] VCVTUSI2SS xmm1, xmm2, r/m64{er}
    Vcvtusi2ssVdqHdqWqE,

    // [EVEX.128.66.0F3A.W0 42 /r ib] VDBPSADBW xmm1 {k1}{z}, xmm2, xmm3/m128, imm8
    VdbpsadbwVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W0 42 /r ib] VDBPSADBW ymm1 {k1}{z}, ymm2, ymm3/m256, imm8
    VdbpsadbwVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 42 /r ib] VDBPSADBW zmm1 {k1}{z}, zmm2, zmm3/m512, imm8
    VdbpsadbwVdqqHdqqWdqqIbE512,

    // [EVEX.128.66.0F38.W1 88 /r] VEXPANDPD xmm1 {k1}{z}, xmm2/m128
    VexpandpdVdqWdqE128,
    // [EVEX.256.66.0F38.W1 88 /r] VEXPANDPD ymm1 {k1}{z}, ymm2/m256
    VexpandpdVqqWqqE256,
    // [EVEX.512.66.0F38.W1 88 /r] VEXPANDPD zmm1 {k1}{z}, zmm2/m512
    VexpandpdVdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 88 /r] VEXPANDPS xmm1 {k1}{z}, xmm2/m128
    VexpandpsVdqWdqE128,
    // [EVEX.256.66.0F38.W0 88 /r] VEXPANDPS ymm1 {k1}{z}, ymm2/m256
    VexpandpsVqqWqqE256,
    // [EVEX.512.66.0F38.W0 88 /r] VEXPANDPS zmm1 {k1}{z}, zmm2/m512
    VexpandpsVdqqWdqqE512,

    // [0F 00 /4] VERR r/m16
    VerrEw,
    // [0F 00 /5] VERW r/m16
    VerwEw,

    // [VEX.256.66.0F3A.W0 19 /r ib] VEXTRACTF128 xmm1/m128, ymm2, imm8
    Vextractf128WdqVqqIbV256,
    // [EVEX.256.66.0F3A.W0 19 /r ib] VEXTRACTF32X4 xmm1/m128 {k1}{z}, ymm2, imm8
    Vextractf32x4WdqVqqIbE256,
    // [EVEX.512.66.0F3A.W0 19 /r ib] VEXTRACTF32X4 xmm1/m128 {k1}{z}, zmm2, imm8
    Vextractf32x4WdqWdqqIbE512,
    // [EVEX.256.66.0F3A.W1 19 /r ib] VEXTRACTF64X2 xmm1/m128 {k1}{z}, ymm2, imm8
    Vextractf64x2WdqVqqIbE256,
    // [EVEX.512.66.0F3A.W1 19 /r ib] VEXTRACTF64X2 xmm1/m128 {k1}{z}, zmm2, imm8
    Vextractf64x2WdqVdqqIbE512,
    // [EVEX.512.66.0F3A.W0 1B /r ib] VEXTRACTF32X8 ymm1/m256 {k1}{z}, zmm2, imm8
    Vextractf32x8WqqVdqqIbE512,
    // [EVEX.512.66.0F3A.W1 1B /r ib] VEXTRACTF64X4 ymm1/m256 {k1}{z}, zmm2, imm8
    Vextractf64x4WqqVdqqIbE512,

    // [VEX.256.66.0F3A.W0 39 /r ib] VEXTRACTI128 xmm1/m128, ymm2, imm8
    Vextracti128WdqVqqIbV256,
    // [EVEX.256.66.0F3A.W0 39 /r ib] VEXTRACTI32X4 xmm1/m128 {k1}{z}, ymm2, imm8
    Vextracti32x4WdqVqqIbE256,
    // [EVEX.512.66.0F3A.W0 39 /r ib] VEXTRACTI32X4 xmm1/m128 {k1}{z}, zmm2, imm8
    Vextracti32x4WdqWdqqIbE512,
    // [EVEX.256.66.0F3A.W1 39 /r ib] VEXTRACTI64X2 xmm1/m128 {k1}{z}, ymm2, imm8
    Vextracti64x2WdqVqqIbE256,
    // [EVEX.512.66.0F3A.W1 39 /r ib] VEXTRACTI64X2 xmm1/m128 {k1}{z}, zmm2, imm8
    Vextracti64x2WdqVdqqIbE512,
    // [EVEX.512.66.0F3A.W0 3B /r ib] VEXTRACTI32X8 ymm1/m256 {k1}{z}, zmm2, imm8
    Vextracti32x8WqqVdqqIbE512,
    // [EVEX.512.66.0F3A.W1 3B /r ib] VEXTRACTI64X4 ymm1/m256 {k1}{z}, zmm2, imm8
    Vextracti64x4WqqVdqqIbE512,

    // [EVEX.128.66.0F3A.W1 54 /r ib] VFIXUPIMMPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    VfixupimmpdVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 54 /r ib] VFIXUPIMMPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    VfixupimmpdVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 54 /r ib] VFIXUPIMMPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{sae}, imm8
    VfixupimmpdVdqqHdqqWdqqIbE512,

    // [EVEX.128.66.0F3A.W0 54 /r ib] VFIXUPIMMPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst, imm8
    VfixupimmpsVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W0 54 /r ib] VFIXUPIMMPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst, imm8
    VfixupimmpsVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 54 /r ib] VFIXUPIMMPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{sae}, imm8
    VfixupimmpsVdqqHdqqWdqqIbE512,

    // [EVEX.LIG.66.0F3A.W1 55 /r ib] VFIXUPIMMSD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}, imm8
    VfixupimmsdVdqHdqWqIbE,

    // [EVEX.LIG.66.0F3A.W0 55 /r ib] VFIXUPIMMSS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}, imm8
    VfixupimmssVdqHdqWdIbE,

    // [VEX.128.66.0F38.W1 98 /r] VFMADD132PD xmm1, xmm2, xmm3/m128
    Vfmadd132pdVdqHdqWdqV128,
    // [VEX.128.66.0F38.W1 A8 /r] VFMADD213PD xmm1, xmm2, xmm3/m128
    Vfmadd213pdVdqHdqWdqV128,
    // [VEX.128.66.0F38.W1 B8 /r] VFMADD231PD xmm1, xmm2, xmm3/m128
    Vfmadd231pdVdqHdqWdqV128,
    // [VEX.256.66.0F38.W1 98 /r] VFMADD132PD ymm1, ymm2, ymm3/m256
    Vfmadd132pdVqqHqqWqqV256,
    // [VEX.256.66.0F38.W1 A8 /r] VFMADD213PD ymm1, ymm2, ymm3/m256
    Vfmadd213pdVqqHqqWqqV256,
    // [VEX.256.66.0F38.W1 B8 /r] VFMADD231PD ymm1, ymm2, ymm3/m256
    Vfmadd231pdVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W1 98 /r] VFMADD132PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfmadd132pdVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W1 A8 /r] VFMADD213PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfmadd213pdVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W1 B8 /r] VFMADD231PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfmadd231pdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 98 /r] VFMADD132PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfmadd132pdVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W1 A8 /r] VFMADD213PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfmadd213pdVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W1 B8 /r] VFMADD231PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfmadd231pdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 98 /r] VFMADD132PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfmadd132pdVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W1 A8 /r] VFMADD213PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfmadd213pdVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W1 B8 /r] VFMADD231PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfmadd231pdVdqqHdqqWdqqE512,

    // [VEX.128.66.0F38.W0 98 /r] VFMADD132PS xmm1, xmm2, xmm3/m128
    Vfmadd132psVdqHdqWdqV128,
    // [VEX.128.66.0F38.W0 A8 /r] VFMADD213PS xmm1, xmm2, xmm3/m128
    Vfmadd213psVdqHdqWdqV128,
    // [VEX.128.66.0F38.W0 B8 /r] VFMADD231PS xmm1, xmm2, xmm3/m128
    Vfmadd231psVdqHdqWdqV128,
    // [VEX.256.66.0F38.W0 98 /r] VFMADD132PS ymm1, ymm2, ymm3/m256
    Vfmadd132psVqqHqqWqqV256,
    // [VEX.256.66.0F38.W0 A8 /r] VFMADD213PS ymm1, ymm2, ymm3/m256
    Vfmadd213psVqqHqqWqqV256,
    // [VEX.256.66.0F38.W0 B8 /r] VFMADD231PS ymm1, ymm2, ymm3/m256
    Vfmadd231psVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W0 98 /r] VFMADD132PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfmadd132psVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 A8 /r] VFMADD213PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfmadd213psVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 B8 /r] VFMADD231PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfmadd231psVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 98 /r] VFMADD132PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfmadd132psVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W0 A8 /r] VFMADD213PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfmadd213psVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W0 B8 /r] VFMADD231PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfmadd231psVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 98 /r] VFMADD132PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfmadd132psVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W0 A8 /r] VFMADD213PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfmadd213psVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W0 B8 /r] VFMADD231PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfmadd231psVdqqHdqqWdqqE512,

    // [VEX.LIG.66.0F38.W1 99 /r] VFMADD132SD xmm1, xmm2, xmm3/m64
    Vfmadd132sdVdqHdqWqV,
    // [VEX.LIG.66.0F38.W1 A9 /r] VFMADD213SD xmm1, xmm2, xmm3/m64
    Vfmadd213sdVdqHdqWqV,
    // [VEX.LIG.66.0F38.W1 B9 /r] VFMADD231SD xmm1, xmm2, xmm3/m64
    Vfmadd231sdVdqHdqWqV,
    // [EVEX.LIG.66.0F38.W1 99 /r] VFMADD132SD xmm1, xmm2, xmm3/m64{er}
    Vfmadd132sdVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W1 A9 /r] VFMADD213SD xmm1, xmm2, xmm3/m64{er}
    Vfmadd213sdVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W1 B9 /r] VFMADD231SD xmm1, xmm2, xmm3/m64{er}
    Vfmadd231sdVdqHdqWqE,

    // [VEX.LIG.66.0F38.W0 99 /r] VFMADD132SS xmm1, xmm2, xmm3/m64
    Vfmadd132ssVdqHdqWqV,
    // [VEX.LIG.66.0F38.W0 A9 /r] VFMADD213SS xmm1, xmm2, xmm3/m64
    Vfmadd213ssVdqHdqWqV,
    // [VEX.LIG.66.0F38.W0 B9 /r] VFMADD231SS xmm1, xmm2, xmm3/m64
    Vfmadd231ssVdqHdqWqV,
    // [EVEX.LIG.66.0F38.W0 99 /r] VFMADD132SS xmm1, xmm2, xmm3/m64{er}
    Vfmadd132ssVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W0 A9 /r] VFMADD213SS xmm1, xmm2, xmm3/m64{er}
    Vfmadd213ssVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W0 B9 /r] VFMADD231SS xmm1, xmm2, xmm3/m64{er}
    Vfmadd231ssVdqHdqWqE,

    // [VEX.128.66.0F38.W1 96 /r] VFMADDSUB132PD xmm1, xmm2, xmm3/m128
    Vfmaddsub132pdVdqHdqWdqV128,
    // [VEX.128.66.0F38.W1 A6 /r] VFMADDSUB213PD xmm1, xmm2, xmm3/m128
    Vfmaddsub213pdVdqHdqWdqV128,
    // [VEX.128.66.0F38.W1 B6 /r] VFMADDSUB231PD xmm1, xmm2, xmm3/m128
    Vfmaddsub231pdVdqHdqWdqV128,
    // [VEX.256.66.0F38.W1 96 /r] VFMADDSUB132PD ymm1, ymm2, ymm3/m256
    Vfmaddsub132pdVqqHqqWqqV256,
    // [VEX.256.66.0F38.W1 A6 /r] VFMADDSUB213PD ymm1, ymm2, ymm3/m256
    Vfmaddsub213pdVqqHqqWqqV256,
    // [VEX.256.66.0F38.W1 B6 /r] VFMADDSUB231PD ymm1, ymm2, ymm3/m256
    Vfmaddsub231pdVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W1 96 /r] VFMADDSUB132PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfmaddsub132pdVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W1 A6 /r] VFMADDSUB213PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfmaddsub213pdVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W1 B6 /r] VFMADDSUB231PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfmaddsub231pdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 96 /r] VFMADDSUB132PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfmaddsub132pdVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W1 A6 /r] VFMADDSUB213PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfmaddsub213pdVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W1 B6 /r] VFMADDSUB231PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfmaddsub231pdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 96 /r] VFMADDSUB132PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfmaddsub132pdVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W1 A6 /r] VFMADDSUB213PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfmaddsub213pdVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W1 B6 /r] VFMADDSUB231PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfmaddsub231pdVdqqHdqqWdqqE512,

    // [VEX.128.66.0F38.W0 96 /r] VFMADDSUB132PS xmm1, xmm2, xmm3/m128
    Vfmaddsub132psVdqHdqWdqV128,
    // [VEX.128.66.0F38.W0 A6 /r] VFMADDSUB213PS xmm1, xmm2, xmm3/m128
    Vfmaddsub213psVdqHdqWdqV128,
    // [VEX.128.66.0F38.W0 B6 /r] VFMADDSUB231PS xmm1, xmm2, xmm3/m128
    Vfmaddsub231psVdqHdqWdqV128,
    // [VEX.256.66.0F38.W0 96 /r] VFMADDSUB132PS ymm1, ymm2, ymm3/m256
    Vfmaddsub132psVqqHqqWqqV256,
    // [VEX.256.66.0F38.W0 A6 /r] VFMADDSUB213PS ymm1, ymm2, ymm3/m256
    Vfmaddsub213psVqqHqqWqqV256,
    // [VEX.256.66.0F38.W0 B6 /r] VFMADDSUB231PS ymm1, ymm2, ymm3/m256
    Vfmaddsub231psVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W0 96 /r] VFMADDSUB132PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfmaddsub132psVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 A6 /r] VFMADDSUB213PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfmaddsub213psVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 B6 /r] VFMADDSUB231PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfmaddsub231psVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 96 /r] VFMADDSUB132PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfmaddsub132psVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W0 A6 /r] VFMADDSUB213PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfmaddsub213psVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W0 B6 /r] VFMADDSUB231PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfmaddsub231psVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 96 /r] VFMADDSUB132PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfmaddsub132psVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W0 A6 /r] VFMADDSUB213PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfmaddsub213psVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W0 B6 /r] VFMADDSUB231PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfmaddsub231psVdqqHdqqWdqqE512,

    // [VEX.128.66.0F38.W1 97 /r] VFMSUBADD132PD xmm1, xmm2, xmm3/m128
    Vfmsubadd132pdVdqHdqWdqV128,
    // [VEX.128.66.0F38.W1 A7 /r] VFMSUBADD213PD xmm1, xmm2, xmm3/m128
    Vfmsubadd213pdVdqHdqWdqV128,
    // [VEX.128.66.0F38.W1 B7 /r] VFMSUBADD231PD xmm1, xmm2, xmm3/m128
    Vfmsubadd231pdVdqHdqWdqV128,
    // [VEX.256.66.0F38.W1 97 /r] VFMSUBADD132PD ymm1, ymm2, ymm3/m256
    Vfmsubadd132pdVqqHqqWqqV256,
    // [VEX.256.66.0F38.W1 A7 /r] VFMSUBADD213PD ymm1, ymm2, ymm3/m256
    Vfmsubadd213pdVqqHqqWqqV256,
    // [VEX.256.66.0F38.W1 B7 /r] VFMSUBADD231PD ymm1, ymm2, ymm3/m256
    Vfmsubadd231pdVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W1 97 /r] VFMSUBADD132PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfmsubadd132pdVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W1 A7 /r] VFMSUBADD213PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfmsubadd213pdVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W1 B7 /r] VFMSUBADD231PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfmsubadd231pdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 97 /r] VFMSUBADD132PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfmsubadd132pdVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W1 A7 /r] VFMSUBADD213PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfmsubadd213pdVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W1 B7 /r] VFMSUBADD231PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfmsubadd231pdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 97 /r] VFMSUBADD132PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfmsubadd132pdVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W1 A7 /r] VFMSUBADD213PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfmsubadd213pdVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W1 B7 /r] VFMSUBADD231PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfmsubadd231pdVdqqHdqqWdqqE512,

    // [VEX.128.66.0F38.W0 97 /r] VFMSUBADD132PS xmm1, xmm2, xmm3/m128
    Vfmsubadd132psVdqHdqWdqV128,
    // [VEX.128.66.0F38.W0 A7 /r] VFMSUBADD213PS xmm1, xmm2, xmm3/m128
    Vfmsubadd213psVdqHdqWdqV128,
    // [VEX.128.66.0F38.W0 B7 /r] VFMSUBADD231PS xmm1, xmm2, xmm3/m128
    Vfmsubadd231psVdqHdqWdqV128,
    // [VEX.256.66.0F38.W0 97 /r] VFMSUBADD132PS ymm1, ymm2, ymm3/m256
    Vfmsubadd132psVqqHqqWqqV256,
    // [VEX.256.66.0F38.W0 A7 /r] VFMSUBADD213PS ymm1, ymm2, ymm3/m256
    Vfmsubadd213psVqqHqqWqqV256,
    // [VEX.256.66.0F38.W0 B7 /r] VFMSUBADD231PS ymm1, ymm2, ymm3/m256
    Vfmsubadd231psVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W0 97 /r] VFMSUBADD132PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfmsubadd132psVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 A7 /r] VFMSUBADD213PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfmsubadd213psVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 B7 /r] VFMSUBADD231PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfmsubadd231psVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 97 /r] VFMSUBADD132PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfmsubadd132psVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W0 A7 /r] VFMSUBADD213PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfmsubadd213psVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W0 B7 /r] VFMSUBADD231PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfmsubadd231psVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 97 /r] VFMSUBADD132PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfmsubadd132psVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W0 A7 /r] VFMSUBADD213PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfmsubadd213psVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W0 B7 /r] VFMSUBADD231PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfmsubadd231psVdqqHdqqWdqqE512,

    // [VEX.128.66.0F38.W1 9A /r] VFMSUB132PD xmm1, xmm2, xmm3/m128
    Vfmsub132pdVdqHdqWdqV128,
    // [VEX.128.66.0F38.W1 AA /r] VFMSUB213PD xmm1, xmm2, xmm3/m128
    Vfmsub213pdVdqHdqWdqV128,
    // [VEX.128.66.0F38.W1 BA /r] VFMSUB231PD xmm1, xmm2, xmm3/m128
    Vfmsub231pdVdqHdqWdqV128,
    // [VEX.256.66.0F38.W1 9A /r] VFMSUB132PD ymm1, ymm2, ymm3/m256
    Vfmsub132pdVqqHqqWqqV256,
    // [VEX.256.66.0F38.W1 AA /r] VFMSUB213PD ymm1, ymm2, ymm3/m256
    Vfmsub213pdVqqHqqWqqV256,
    // [VEX.256.66.0F38.W1 BA /r] VFMSUB231PD ymm1, ymm2, ymm3/m256
    Vfmsub231pdVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W1 9A /r] VFMSUB132PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfmsub132pdVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W1 AA /r] VFMSUB213PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfmsub213pdVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W1 BA /r] VFMSUB231PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfmsub231pdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 9A /r] VFMSUB132PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfmsub132pdVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W1 AA /r] VFMSUB213PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfmsub213pdVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W1 BA /r] VFMSUB231PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfmsub231pdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 9A /r] VFMSUB132PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfmsub132pdVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W1 AA /r] VFMSUB213PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfmsub213pdVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W1 BA /r] VFMSUB231PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfmsub231pdVdqqHdqqWdqqE512,

    // [VEX.128.66.0F38.W0 9A /r] VFMSUB132PS xmm1, xmm2, xmm3/m128
    Vfmsub132psVdqHdqWdqV128,
    // [VEX.128.66.0F38.W0 AA /r] VFMSUB213PS xmm1, xmm2, xmm3/m128
    Vfmsub213psVdqHdqWdqV128,
    // [VEX.128.66.0F38.W0 BA /r] VFMSUB231PS xmm1, xmm2, xmm3/m128
    Vfmsub231psVdqHdqWdqV128,
    // [VEX.256.66.0F38.W0 9A /r] VFMSUB132PS ymm1, ymm2, ymm3/m256
    Vfmsub132psVqqHqqWqqV256,
    // [VEX.256.66.0F38.W0 AA /r] VFMSUB213PS ymm1, ymm2, ymm3/m256
    Vfmsub213psVqqHqqWqqV256,
    // [VEX.256.66.0F38.W0 BA /r] VFMSUB231PS ymm1, ymm2, ymm3/m256
    Vfmsub231psVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W0 9A /r] VFMSUB132PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfmsub132psVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 AA /r] VFMSUB213PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfmsub213psVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 BA /r] VFMSUB231PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfmsub231psVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 9A /r] VFMSUB132PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfmsub132psVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W0 AA /r] VFMSUB213PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfmsub213psVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W0 BA /r] VFMSUB231PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfmsub231psVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 9A /r] VFMSUB132PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfmsub132psVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W0 AA /r] VFMSUB213PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfmsub213psVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W0 BA /r] VFMSUB231PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfmsub231psVdqqHdqqWdqqE512,

    // [VEX.LIG.66.0F38.W1 9B /r] VFMSUB132SD xmm1, xmm2, xmm3/m64
    Vfmsub132sdVdqHdqWqV,
    // [VEX.LIG.66.0F38.W1 AB /r] VFMSUB213SD xmm1, xmm2, xmm3/m64
    Vfmsub213sdVdqHdqWqV,
    // [VEX.LIG.66.0F38.W1 BB /r] VFMSUB231SD xmm1, xmm2, xmm3/m64
    Vfmsub231sdVdqHdqWqV,
    // [EVEX.LIG.66.0F38.W1 9B /r] VFMSUB132SD xmm1, xmm2, xmm3/m64{er}
    Vfmsub132sdVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W1 AB /r] VFMSUB213SD xmm1, xmm2, xmm3/m64{er}
    Vfmsub213sdVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W1 BB /r] VFMSUB231SD xmm1, xmm2, xmm3/m64{er}
    Vfmsub231sdVdqHdqWqE,

    // [VEX.LIG.66.0F38.W0 9B /r] VFMSUB132SS xmm1, xmm2, xmm3/m64
    Vfmsub132ssVdqHdqWqV,
    // [VEX.LIG.66.0F38.W0 AB /r] VFMSUB213SS xmm1, xmm2, xmm3/m64
    Vfmsub213ssVdqHdqWqV,
    // [VEX.LIG.66.0F38.W0 BB /r] VFMSUB231SS xmm1, xmm2, xmm3/m64
    Vfmsub231ssVdqHdqWqV,
    // [EVEX.LIG.66.0F38.W0 9B /r] VFMSUB132SS xmm1, xmm2, xmm3/m64{er}
    Vfmsub132ssVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W0 AB /r] VFMSUB213SS xmm1, xmm2, xmm3/m64{er}
    Vfmsub213ssVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W0 BB /r] VFMSUB231SS xmm1, xmm2, xmm3/m64{er}
    Vfmsub231ssVdqHdqWqE,

    // [VEX.128.66.0F38.W1 9C /r] VFNMADD132PD xmm1, xmm2, xmm3/m128
    Vfnmadd132pdVdqHdqWdqV128,
    // [VEX.128.66.0F38.W1 AC /r] VFNMADD213PD xmm1, xmm2, xmm3/m128
    Vfnmadd213pdVdqHdqWdqV128,
    // [VEX.128.66.0F38.W1 BC /r] VFNMADD231PD xmm1, xmm2, xmm3/m128
    Vfnmadd231pdVdqHdqWdqV128,
    // [VEX.256.66.0F38.W1 9C /r] VFNMADD132PD ymm1, ymm2, ymm3/m256
    Vfnmadd132pdVqqHqqWqqV256,
    // [VEX.256.66.0F38.W1 AC /r] VFNMADD213PD ymm1, ymm2, ymm3/m256
    Vfnmadd213pdVqqHqqWqqV256,
    // [VEX.256.66.0F38.W1 BC /r] VFNMADD231PD ymm1, ymm2, ymm3/m256
    Vfnmadd231pdVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W1 9C /r] VFNMADD132PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfnmadd132pdVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W1 AC /r] VFNMADD213PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfnmadd213pdVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W1 BC /r] VFNMADD231PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfnmadd231pdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 9C /r] VFNMADD132PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfnmadd132pdVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W1 AC /r] VFNMADD213PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfnmadd213pdVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W1 BC /r] VFNMADD231PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfnmadd231pdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 9C /r] VFNMADD132PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfnmadd132pdVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W1 AC /r] VFNMADD213PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfnmadd213pdVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W1 BC /r] VFNMADD231PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfnmadd231pdVdqqHdqqWdqqE512,

    // [VEX.128.66.0F38.W0 9C /r] VFNMADD132PS xmm1, xmm2, xmm3/m128
    Vfnmadd132psVdqHdqWdqV128,
    // [VEX.128.66.0F38.W0 AC /r] VFNMADD213PS xmm1, xmm2, xmm3/m128
    Vfnmadd213psVdqHdqWdqV128,
    // [VEX.128.66.0F38.W0 BC /r] VFNMADD231PS xmm1, xmm2, xmm3/m128
    Vfnmadd231psVdqHdqWdqV128,
    // [VEX.256.66.0F38.W0 9C /r] VFNMADD132PS ymm1, ymm2, ymm3/m256
    Vfnmadd132psVqqHqqWqqV256,
    // [VEX.256.66.0F38.W0 AC /r] VFNMADD213PS ymm1, ymm2, ymm3/m256
    Vfnmadd213psVqqHqqWqqV256,
    // [VEX.256.66.0F38.W0 BC /r] VFNMADD231PS ymm1, ymm2, ymm3/m256
    Vfnmadd231psVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W0 9C /r] VFNMADD132PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfnmadd132psVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 AC /r] VFNMADD213PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfnmadd213psVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 BC /r] VFNMADD231PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfnmadd231psVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 9C /r] VFNMADD132PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfnmadd132psVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W0 AC /r] VFNMADD213PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfnmadd213psVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W0 BC /r] VFNMADD231PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfnmadd231psVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 9C /r] VFNMADD132PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfnmadd132psVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W0 AC /r] VFNMADD213PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfnmadd213psVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W0 BC /r] VFNMADD231PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfnmadd231psVdqqHdqqWdqqE512,

    // [VEX.LIG.66.0F38.W1 9D /r] VFNMADD132SD xmm1, xmm2, xmm3/m64
    Vfnmadd132sdVdqHdqWqV,
    // [VEX.LIG.66.0F38.W1 AD /r] VFNMADD213SD xmm1, xmm2, xmm3/m64
    Vfnmadd213sdVdqHdqWqV,
    // [VEX.LIG.66.0F38.W1 BD /r] VFNMADD231SD xmm1, xmm2, xmm3/m64
    Vfnmadd231sdVdqHdqWqV,
    // [EVEX.LIG.66.0F38.W1 9D /r] VFNMADD132SD xmm1, xmm2, xmm3/m64{er}
    Vfnmadd132sdVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W1 AD /r] VFNMADD213SD xmm1, xmm2, xmm3/m64{er}
    Vfnmadd213sdVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W1 BD /r] VFNMADD231SD xmm1, xmm2, xmm3/m64{er}
    Vfnmadd231sdVdqHdqWqE,

    // [VEX.LIG.66.0F38.W0 9D /r] VFNMADD132SS xmm1, xmm2, xmm3/m64
    Vfnmadd132ssVdqHdqWqV,
    // [VEX.LIG.66.0F38.W0 AD /r] VFNMADD213SS xmm1, xmm2, xmm3/m64
    Vfnmadd213ssVdqHdqWqV,
    // [VEX.LIG.66.0F38.W0 BD /r] VFNMADD231SS xmm1, xmm2, xmm3/m64
    Vfnmadd231ssVdqHdqWqV,
    // [EVEX.LIG.66.0F38.W0 9D /r] VFNMADD132SS xmm1, xmm2, xmm3/m64{er}
    Vfnmadd132ssVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W0 AD /r] VFNMADD213SS xmm1, xmm2, xmm3/m64{er}
    Vfnmadd213ssVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W0 BD /r] VFNMADD231SS xmm1, xmm2, xmm3/m64{er}
    Vfnmadd231ssVdqHdqWqE,

    // [VEX.128.66.0F38.W1 9E /r] VFNMSUB132PD xmm1, xmm2, xmm3/m128
    Vfnmsub132pdVdqHdqWdqV128,
    // [VEX.128.66.0F38.W1 AE /r] VFNMSUB213PD xmm1, xmm2, xmm3/m128
    Vfnmsub213pdVdqHdqWdqV128,
    // [VEX.128.66.0F38.W1 BE /r] VFNMSUB231PD xmm1, xmm2, xmm3/m128
    Vfnmsub231pdVdqHdqWdqV128,
    // [VEX.256.66.0F38.W1 9E /r] VFNMSUB132PD ymm1, ymm2, ymm3/m256
    Vfnmsub132pdVqqHqqWqqV256,
    // [VEX.256.66.0F38.W1 AE /r] VFNMSUB213PD ymm1, ymm2, ymm3/m256
    Vfnmsub213pdVqqHqqWqqV256,
    // [VEX.256.66.0F38.W1 BE /r] VFNMSUB231PD ymm1, ymm2, ymm3/m256
    Vfnmsub231pdVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W1 9E /r] VFNMSUB132PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfnmsub132pdVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W1 AE /r] VFNMSUB213PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfnmsub213pdVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W1 BE /r] VFNMSUB231PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vfnmsub231pdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 9E /r] VFNMSUB132PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfnmsub132pdVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W1 AE /r] VFNMSUB213PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfnmsub213pdVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W1 BE /r] VFNMSUB231PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vfnmsub231pdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 9E /r] VFNMSUB132PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfnmsub132pdVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W1 AE /r] VFNMSUB213PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfnmsub213pdVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W1 BE /r] VFNMSUB231PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    Vfnmsub231pdVdqqHdqqWdqqE512,

    // [VEX.128.66.0F38.W0 9E /r] VFNMSUB132PS xmm1, xmm2, xmm3/m128
    Vfnmsub132psVdqHdqWdqV128,
    // [VEX.128.66.0F38.W0 AE /r] VFNMSUB213PS xmm1, xmm2, xmm3/m128
    Vfnmsub213psVdqHdqWdqV128,
    // [VEX.128.66.0F38.W0 BE /r] VFNMSUB231PS xmm1, xmm2, xmm3/m128
    Vfnmsub231psVdqHdqWdqV128,
    // [VEX.256.66.0F38.W0 9E /r] VFNMSUB132PS ymm1, ymm2, ymm3/m256
    Vfnmsub132psVqqHqqWqqV256,
    // [VEX.256.66.0F38.W0 AE /r] VFNMSUB213PS ymm1, ymm2, ymm3/m256
    Vfnmsub213psVqqHqqWqqV256,
    // [VEX.256.66.0F38.W0 BE /r] VFNMSUB231PS ymm1, ymm2, ymm3/m256
    Vfnmsub231psVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W0 9E /r] VFNMSUB132PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfnmsub132psVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 AE /r] VFNMSUB213PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfnmsub213psVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 BE /r] VFNMSUB231PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vfnmsub231psVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 9E /r] VFNMSUB132PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfnmsub132psVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W0 AE /r] VFNMSUB213PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfnmsub213psVqqHqqWqqE256,
    // [EVEX.256.66.0F38.W0 BE /r] VFNMSUB231PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vfnmsub231psVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 9E /r] VFNMSUB132PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfnmsub132psVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W0 AE /r] VFNMSUB213PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfnmsub213psVdqqHdqqWdqqE512,
    // [EVEX.512.66.0F38.W0 BE /r] VFNMSUB231PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    Vfnmsub231psVdqqHdqqWdqqE512,

    // [VEX.LIG.66.0F38.W1 9F /r] VFNMSUB132SD xmm1, xmm2, xmm3/m64
    Vfnmsub132sdVdqHdqWqV,
    // [VEX.LIG.66.0F38.W1 AF /r] VFNMSUB213SD xmm1, xmm2, xmm3/m64
    Vfnmsub213sdVdqHdqWqV,
    // [VEX.LIG.66.0F38.W1 BF /r] VFNMSUB231SD xmm1, xmm2, xmm3/m64
    Vfnmsub231sdVdqHdqWqV,
    // [EVEX.LIG.66.0F38.W1 9F /r] VFNMSUB132SD xmm1, xmm2, xmm3/m64{er}
    Vfnmsub132sdVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W1 AF /r] VFNMSUB213SD xmm1, xmm2, xmm3/m64{er}
    Vfnmsub213sdVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W1 BF /r] VFNMSUB231SD xmm1, xmm2, xmm3/m64{er}
    Vfnmsub231sdVdqHdqWqE,

    // [VEX.LIG.66.0F38.W0 9F /r] VFNMSUB132SS xmm1, xmm2, xmm3/m64
    Vfnmsub132ssVdqHdqWqV,
    // [VEX.LIG.66.0F38.W0 AF /r] VFNMSUB213SS xmm1, xmm2, xmm3/m64
    Vfnmsub213ssVdqHdqWqV,
    // [VEX.LIG.66.0F38.W0 BF /r] VFNMSUB231SS xmm1, xmm2, xmm3/m64
    Vfnmsub231ssVdqHdqWqV,
    // [EVEX.LIG.66.0F38.W0 9F /r] VFNMSUB132SS xmm1, xmm2, xmm3/m64{er}
    Vfnmsub132ssVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W0 AF /r] VFNMSUB213SS xmm1, xmm2, xmm3/m64{er}
    Vfnmsub213ssVdqHdqWqE,
    // [EVEX.LIG.66.0F38.W0 BF /r] VFNMSUB231SS xmm1, xmm2, xmm3/m64{er}
    Vfnmsub231ssVdqHdqWqE,

    // [EVEX.128.66.0F3A.W1 66 /r ib] VFPCLASSPD k2 {k1}, xmm1/m128/m64bcst, imm8
    VfpclasspdKGbWdqIbE128,
    // [EVEX.256.66.0F3A.W1 66 /r ib] VFPCLASSPD k2 {k1}, ymm1/m256/m64bcst, imm8
    VfpclasspdKGbWqqIbE256,
    // [EVEX.512.66.0F3A.W1 66 /r ib] VFPCLASSPD k2 {k1}, zmm1/m512/m64bcst, imm8
    VfpclasspdKGbWdqqIbE512,

    // [EVEX.128.66.0F3A.W0 66 /r ib] VFPCLASSPS k2 {k1}, xmm1/m128/m32bcst, imm8
    VfpclasspsKGbWdqIbE128,
    // [EVEX.256.66.0F3A.W0 66 /r ib] VFPCLASSPS k2 {k1}, ymm1/m256/m32bcst, imm8
    VfpclasspsKGbWqqIbE256,
    // [EVEX.512.66.0F3A.W0 66 /r ib] VFPCLASSPS k2 {k1}, zmm1/m512/m32bcst, imm8
    VfpclasspsKGbWdqqIbE512,

    // [EVEX.LIG.66.0F3A.W1 67 /r ib] VFPCLASSSD k2 {k1}, xmm2/m64, imm8
    VfpclasssdKGbWqIbE,

    // [EVEX.LIG.66.0F3A.W0 67 /r ib] VFPCLASSSS k2 {k1}, xmm2/m32, imm8
    VfpclassssKGbWdIbE,

    // [VEX.128.66.0F38.W1 92 /r] VGATHERDPD xmm1, vm32x, xmm2
    VgatherdpdVdqVMdHdqV128,
    // [VEX.128.66.0F38.W1 93 /r] VGATHERQPD xmm1, vm64x, xmm2
    VgatherqpdVdqVMqHdqV128,
    // [VEX.256.66.0F38.W1 92 /r] VGATHERDPD ymm1, vm32y, ymm2
    VgatherdpdVqqVMdHqqV256,
    // [VEX.256.66.0F38.W1 93 /r] VGATHERQPD ymm1, vm64y, ymm2
    VgatherqpdVqqVMqHqqV256,

    // [VEX.128.66.0F38.W0 92 /r] VGATHERDPS xmm1, vm32x, xmm2
    VgatherdpsVdqVMdHdqV128,
    // [VEX.128.66.0F38.W0 93 /r] VGATHERQPS xmm1, vm64x, xmm2
    VgatherqpsVdqVMqHdqV128,
    // [VEX.256.66.0F38.W0 92 /r] VGATHERDPS ymm1, vm32y, ymm2
    VgatherdpsVqqVMdHqqV256,
    // [VEX.256.66.0F38.W0 93 /r] VGATHERQPS ymm1, vm64y, ymm2
    VgatherqpsVqqVMqHqqV256,

    // [EVEX.128.66.0F38.W0 92 /vsib] VGATHERDPS xmm1 {k1}{z}, vm32x
    VgatherdpsVdqVMdE128,
    // [EVEX.256.66.0F38.W0 92 /vsib] VGATHERDPS ymm1 {k1}{z}, vm32y
    VgatherdpsVqqVMdE256,
    // [EVEX.512.66.0F38.W0 92 /vsib] VGATHERDPS zmm1 {k1}{z}, vm32z
    VgatherdpsVdqqVMdE512,
    // [EVEX.128.66.0F38.W1 92 /vsib] VGATHERDPD xmm1 {k1}{z}, vm32x
    VgatherdpdVdqVMdE128,
    // [EVEX.256.66.0F38.W1 92 /vsib] VGATHERDPD ymm1 {k1}{z}, vm32y
    VgatherdpdVqqVMdE256,
    // [EVEX.512.66.0F38.W1 92 /vsib] VGATHERDPD zmm1 {k1}{z}, vm32z
    VgatherdpdVdqqVMdE512,

    // [EVEX.128.66.0F38.W0 93 /vsib] VGATHERQPS xmm1 {k1}{z}, vm64x
    VgatherqpsVdqVMqE128,
    // [EVEX.256.66.0F38.W0 93 /vsib] VGATHERQPS ymm1 {k1}{z}, vm64y
    VgatherqpsVqqVMqE256,
    // [EVEX.512.66.0F38.W0 93 /vsib] VGATHERQPS zmm1 {k1}{z}, vm64z
    VgatherqpsVdqqVMqE512,
    // [EVEX.128.66.0F38.W1 93 /vsib] VGATHERQPD xmm1 {k1}{z}, vm64x
    VgatherqpdVdqVMqE128,
    // [EVEX.256.66.0F38.W1 93 /vsib] VGATHERQPD ymm1 {k1}{z}, vm64y
    VgatherqpdVqqVMqE256,
    // [EVEX.512.66.0F38.W1 93 /vsib] VGATHERQPD zmm1 {k1}{z}, vm64z
    VgatherqpdVdqqVMqE512,

    // [EVEX.128.66.0F38.W1 42 /r] VGETEXPPD xmm1 {k1}{z}, xmm2/m128/m64bcst
    VgetexppdVdqWdqE128,
    // [EVEX.256.66.0F38.W1 42 /r] VGETEXPPD ymm1 {k1}{z}, ymm2/m256/m64bcst
    VgetexppdVqqWqqE256,
    // [EVEX.512.66.0F38.W1 42 /r] VGETEXPPD zmm1 {k1}{z}, zmm2/m512/m64bcst{er}
    VgetexppdVdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 42 /r] VGETEXPPS xmm1 {k1}{z}, xmm2/m128/m32bcst
    VgetexppsVdqWdqE128,
    // [EVEX.256.66.0F38.W0 42 /r] VGETEXPPS ymm1 {k1}{z}, ymm2/m256/m32bcst
    VgetexppsVqqWqqE256,
    // [EVEX.512.66.0F38.W0 42 /r] VGETEXPPS zmm1 {k1}{z}, zmm2/m512/m32bcst{er}
    VgetexppsVdqqWdqqE512,

    // [EVEX.LIG.66.0F38.W1 43 /r] VGETEXPSD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}
    VgetexpsdVdqHdqWqE,

    // [EVEX.LIG.66.0F38.W0 43 /r] VGETEXPSS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}
    VgetexpssVdqHdqWdE,

    // [EVEX.128.66.0F3A.W1 26 /r ib] VGETMANTPD xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VgetmantpdVdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 26 /r ib] VGETMANTPD ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VgetmantpdVqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 26 /r ib] VGETMANTPD zmm1 {k1}{z}, zmm2/m512/m64bcst{er}, imm8
    VgetmantpdVdqqWdqqIbE512,

    // [EVEX.128.66.0F3A.W0 26 /r ib] VGETMANTPS xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VgetmantpsVdqWdqIbE128,
    // [EVEX.256.66.0F3A.W0 26 /r ib] VGETMANTPS ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VgetmantpsVqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 26 /r ib] VGETMANTPS zmm1 {k1}{z}, zmm2/m512/m64bcst{er}, imm8
    VgetmantpsVdqqWdqqIbE512,

    // [EVEX.LIG.66.0F3A.W1 27 /r ib] VGETMANTSD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}, imm8
    VgetmantsdVdqHdqWqIbE,

    // [EVEX.LIG.66.0F3A.W0 27 /r ib] VGETMANTSS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}, imm8
    VgetmantssVdqHdqWdIbE,

    // [VEX.256.66.0F3A.W0 18 /r ib] VINSERTF128 ymm1, ymm2, xmm3/m128, imm8
    Vinsertf128VqqHqqWdqIbV256,
    // [EVEX.256.66.0F3A.W0 18 /r ib] VINSERTF32X4 ymm1 {k1}{z}, ymm2, xmm3/m128, imm8
    Vinsertf32x4VqqHqqWdqIbE256,
    // [EVEX.512.66.0F3A.W0 18 /r ib] VINSERTF32X4 zmm1 {k1}{z}, zmm2, xmm3/m128, imm8
    Vinsertf32x4VdqqHdqqWdqIbE512,
    // [EVEX.256.66.0F3A.W1 18 /r ib] VINSERTF64X2 ymm1 {k1}{z}, ymm2, xmm3/m128, imm8
    Vinsertf64x2VqqHqqWdqIbE256,
    // [EVEX.512.66.0F3A.W1 18 /r ib] VINSERTF64X2 zmm1 {k1}{z}, zmm2, xmm3/m128, imm8
    Vinsertf64x2VdqqHdqqWdqIbE512,
    // [EVEX.512.66.0F3A.W0 1A /r ib] VINSERTF32X8 zmm1 {k1}{z}, zmm2, ymm3/m256, imm8
    Vinsertf32x8VdqqHdqqWqqIbE512,
    // [EVEX.512.66.0F3A.W1 1A /r ib] VINSERTF64X4 zmm1 {k1}{z}, zmm2, ymm3/m256, imm8
    Vinsertf64x4VdqqHdqqWqqIbE512,

    // [VEX.256.66.0F3A.W0 38 /r ib] VINSERTI128 ymm1, ymm2, xmm3/m128, imm8
    Vinserti128VqqHqqWdqIbV256,
    // [EVEX.256.66.0F3A.W0 38 /r ib] VINSERTI32X4 ymm1 {k1}{z}, ymm2, xmm3/m128, imm8
    Vinserti32x4VqqHqqWdqIbE256,
    // [EVEX.512.66.0F3A.W0 38 /r ib] VINSERTI32X4 zmm1 {k1}{z}, zmm2, xmm3/m128, imm8
    Vinserti32x4VdqqHdqqWdqIbE512,
    // [EVEX.256.66.0F3A.W1 38 /r ib] VINSERTI64X2 ymm1 {k1}{z}, ymm2, xmm3/m128, imm8
    Vinserti64x2VqqHqqWdqIbE256,
    // [EVEX.512.66.0F3A.W1 38 /r ib] VINSERTI64X2 zmm1 {k1}{z}, zmm2, xmm3/m128, imm8
    Vinserti64x2VdqqHdqqWdqIbE512,
    // [EVEX.512.66.0F3A.W0 3A /r ib] VINSERTI32X8 zmm1 {k1}{z}, zmm2, ymm3/m256, imm8
    Vinserti32x8VdqqHdqqWqqIbE512,
    // [EVEX.512.66.0F3A.W1 3A /r ib] VINSERTI64X4 zmm1 {k1}{z}, zmm2, ymm3/m256, imm8
    Vinserti64x4VdqqHdqqWqqIbE512,

    // [VEX.128.66.0F38.W0 2C /r] VMASKMOVPS xmm1, xmm2, m128
    VmaskmovpsVdqHdqWdqV128,
    // [VEX.256.66.0F38.W0 2C /r] VMASKMOVPS ymm1, ymm2, m256
    VmaskmovpsVqqHqqWqqV256,
    // [VEX.128.66.0F38.W0 2D /r] VMASKMOVPD xmm1, xmm2, m128
    VmaskmovpdVdqHdqWdqV128,
    // [VEX.256.66.0F38.W0 2D /r] VMASKMOVPD ymm1, ymm2, m256
    VmaskmovpdVqqHqqWqqV256,
    // [VEX.128.66.0F38.W0 2E /r] VMASKMOVPS m128, xmm1, xmm2
    VmaskmovpsWdqHdqVdqV128,
    // [VEX.256.66.0F38.W0 2E /r] VMASKMOVPS m256, ymm1, ymm2
    VmaskmovpsWqqHqqVqqV256,
    // [VEX.128.66.0F38.W0 2F /r] VMASKMOVPD m128, xmm1, xmm2
    VmaskmovpdWdqHdqVdqV128,
    // [VEX.256.66.0F38.W0 2F /r] VMASKMOVPD m256, ymm1, ymm2
    VmaskmovpdWqqHqqVqqV256,

    // [VEX.128.66.0F3A.W0 02 /r ib] VPBLENDD xmm1, xmm2, xmm3/m128, imm8
    VpblenddVdqHdqWdqIbV128,
    // [VEX.256.66.0F3A.W0 02 /r ib] VPBLENDD ymm1, ymm2, ymm3/m256, imm8
    VpblenddVqqHqqWqqIbV256,

    // [EVEX.128.66.0F38.W0 66 /r] VPBLENDMB xmm1 {k1}{z}, xmm2, xmm3/m128
    VpblendmbVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 66 /r] VPBLENDMB ymm1 {k1}{z}, ymm2, ymm3/m256
    VpblendmbVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 66 /r] VPBLENDMB zmm1 {k1}{z}, zmm2, zmm3/m512
    VpblendmbVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 66 /r] VPBLENDMW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpblendmwVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 66 /r] VPBLENDMW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpblendmwVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 66 /r] VPBLENDMW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpblendmwVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 64 /r] VPBLENDMD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpblendmdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 64 /r] VPBLENDMD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpblendmdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 64 /r] VPBLENDMD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpblendmdVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 64 /r] VPBLENDMQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpblendmqVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 64 /r] VPBLENDMQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpblendmqVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 64 /r] VPBLENDMQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpblendmqVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 7A /r] VPBROADCASTB xmm1 {k1}{z}, r8
    VpbroadcastbVdqEbE128,
    // [EVEX.256.66.0F38.W0 7A /r] VPBROADCASTB ymm1 {k1}{z}, r8
    VpbroadcastbVqqEbE256,
    // [EVEX.512.66.0F38.W0 7A /r] VPBROADCASTB zmm1 {k1}{z}, r8
    VpbroadcastbVdqqEbE512,
    // [EVEX.128.66.0F38.W0 7B /r] VPBROADCASTW xmm1 {k1}{z}, r16
    VpbroadcastwVdqEwE128,
    // [EVEX.256.66.0F38.W0 7B /r] VPBROADCASTW ymm1 {k1}{z}, r16
    VpbroadcastwVqqEwE256,
    // [EVEX.512.66.0F38.W0 7B /r] VPBROADCASTW zmm1 {k1}{z}, r16
    VpbroadcastwVdqqEwE512,
    // [EVEX.128.66.0F38.W0 7C /r] VPBROADCASTD xmm1 {k1}{z}, r32
    VpbroadcastdVdqEdE128,
    // [EVEX.256.66.0F38.W0 7C /r] VPBROADCASTD ymm1 {k1}{z}, r32
    VpbroadcastdVqqEdE256,
    // [EVEX.512.66.0F38.W0 7C /r] VPBROADCASTD zmm1 {k1}{z}, r32
    VpbroadcastdVdqqEdE512,
    // [EVEX.128.66.0F38.W1 7C /r] VPBROADCASTQ xmm1 {k1}{z}, r64
    VpbroadcastqVdqEqE128,
    // [EVEX.256.66.0F38.W1 7C /r] VPBROADCASTQ ymm1 {k1}{z}, r64
    VpbroadcastqVqqEqE256,
    // [EVEX.512.66.0F38.W1 7C /r] VPBROADCASTQ zmm1 {k1}{z}, r64
    VpbroadcastqVdqqEqE512,

    // [VEX.128.66.0F38.W0 78 /r] VPBROADCASTB xmm1, xmm2/m8
    VpbroadcastbVdqWbV128,
    // [VEX.256.66.0F38.W0 78 /r] VPBROADCASTB ymm1, ymm2/m8
    VpbroadcastbVqqWbV256,
    // [EVEX.128.66.0F38.W0 78 /r] VPBROADCASTB xmm1 {k1}{z}, xmm2/m8
    VpbroadcastbVdqWbE128,
    // [EVEX.256.66.0F38.W0 78 /r] VPBROADCASTB ymm1 {k1}{z}, xmm2/m8
    VpbroadcastbVqqWbE256,
    // [EVEX.512.66.0F38.W0 78 /r] VPBROADCASTB zmm1 {k1}{z}, xmm2/m8
    VpbroadcastbVdqqWbE512,
    // [VEX.128.66.0F38.W0 79 /r] VPBROADCASTW xmm1, xmm2/m16
    VpbroadcastwVdqWwV128,
    // [VEX.256.66.0F38.W0 79 /r] VPBROADCASTW ymm1, ymm2/m16
    VpbroadcastwVqqWwV256,
    // [EVEX.128.66.0F38.W0 79 /r] VPBROADCASTW xmm1 {k1}{z}, xmm2/m16
    VpbroadcastwVdqWwE128,
    // [EVEX.256.66.0F38.W0 79 /r] VPBROADCASTW ymm1 {k1}{z}, xmm2/m16
    VpbroadcastwVqqWwE256,
    // [EVEX.512.66.0F38.W0 79 /r] VPBROADCASTW zmm1 {k1}{z}, xmm2/m16
    VpbroadcastwVdqqWwE512,
    // [VEX.128.66.0F38.W0 58 /r] VPBROADCASTD xmm1, xmm2/m32
    VpbroadcastdVdqWdV128,
    // [VEX.256.66.0F38.W0 58 /r] VPBROADCASTD ymm1, ymm2/m32
    VpbroadcastdVqqWdV256,
    // [EVEX.128.66.0F38.W0 58 /r] VPBROADCASTD xmm1 {k1}{z}, xmm2/m32
    VpbroadcastdVdqWdE128,
    // [EVEX.256.66.0F38.W0 58 /r] VPBROADCASTD ymm1 {k1}{z}, xmm2/m32
    VpbroadcastdVqqWdE256,
    // [EVEX.512.66.0F38.W0 58 /r] VPBROADCASTD zmm1 {k1}{z}, xmm2/m32
    VpbroadcastdVdqqWdE512,
    // [VEX.128.66.0F38.W0 59 /r] VPBROADCASTQ xmm1, xmm2/m64
    VpbroadcastqVdqWqV128,
    // [VEX.256.66.0F38.W0 59 /r] VPBROADCASTQ ymm1, ymm2/m64
    VpbroadcastqVqqWqV256,
    // [EVEX.128.66.0F38.W1 59 /r] VPBROADCASTQ xmm1 {k1}{z}, xmm2/m64
    VpbroadcastqVdqWqE128,
    // [EVEX.256.66.0F38.W1 59 /r] VPBROADCASTQ ymm1 {k1}{z}, xmm2/m64
    VpbroadcastqVqqWqE256,
    // [EVEX.512.66.0F38.W1 59 /r] VPBROADCASTQ zmm1 {k1}{z}, xmm2/m64
    VpbroadcastqVdqqWqE512,
    // [EVEX.128.66.0F38.W0 59 /r] VBROADCASTI32X2 xmm1 {k1}{z}, xmm2/m64
    Vbroadcasti32x2VdqWqE128,
    // [EVEX.256.66.0F38.W0 59 /r] VBROADCASTI32X2 ymm1 {k1}{z}, xmm2/m64
    Vbroadcasti32x2VqqWqE256,
    // [EVEX.512.66.0F38.W0 59 /r] VBROADCASTI32X2 zmm1 {k1}{z}, xmm2/m64
    Vbroadcasti32x2VdqqWqE512,
    // [VEX.256.66.0F38.W0 5A /r] VBROADCASTI128 ymm1, m128
    Vbroadcasti128VqqMdqV256,
    // [EVEX.256.66.0F38.W0 5A /r] VBROADCASTI32X4 ymm1 {k1}{z}, m128
    Vbroadcasti32x4VqqMdqE256,
    // [EVEX.512.66.0F38.W0 5A /r] VBROADCASTI32X4 zmm1 {k1}{z}, m128
    Vbroadcasti32x4VdqqMdqE512,
    // [EVEX.256.66.0F38.W1 5A /r] VBROADCASTI64X2 ymm1 {k1}{z}, m128
    Vbroadcasti64x2VqqMdqE256,
    // [EVEX.512.66.0F38.W1 5A /r] VBROADCASTI64X2 zmm1 {k1}{z}, m128
    Vbroadcasti64x2VdqqMdqE512,
    // [EVEX.512.66.0F38.W0 5B /r] VBROADCASTI32X8 zmm1 {k1}{z}, m256
    Vbroadcasti32x8VdqqMqqE512,
    // [EVEX.512.66.0F38.W1 5B /r] VBROADCASTI64X4 zmm1 {k1}{z}, m256
    Vbroadcasti64x4VdqqMqqE512,

    // [EVEX.128.66.0F38.W1 2A /r] VPBROADCASTMB2Q xmm1, k1
    Vpbroadcastmb2qVdqKEbE128,
    // [EVEX.256.66.0F38.W1 2A /r] VPBROADCASTMB2Q ymm1, k1
    Vpbroadcastmb2qVqqKEbE256,
    // [EVEX.512.66.0F38.W1 2A /r] VPBROADCASTMB2Q zmm1, k1
    Vpbroadcastmb2qVdqqKEbE512,
    // [EVEX.128.66.0F38.W0 3A /r] VPBROADCASTMW2D xmm1, k1
    Vpbroadcastmb2qVdqKEwE128,
    // [EVEX.256.66.0F38.W0 3A /r] VPBROADCASTMW2D ymm1, k1
    Vpbroadcastmb2qVqqKEwE256,
    // [EVEX.512.66.0F38.W0 3A /r] VPBROADCASTMW2D zmm1, k1
    Vpbroadcastmb2qVdqqKEwE512,

    // [EVEX.128.66.0F3A.W0 3F /r ib] VPCMPB k1 {k2}, xmm2, xmm3/m128, imm8
    VpcmpbKGqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W0 3F /r ib] VPCMPB k1 {k2}, ymm2, ymm3/m256, imm8
    VpcmpbKGqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 3F /r ib] VPCMPB k1 {k2}, zmm2, zmm3/m512, imm8
    VpcmpbKGqHdqqWdqqIbE512,
    // [EVEX.128.66.0F3A.W0 3E /r ib] VPCMPUB k1 {k2}, xmm2, xmm3/m128, imm8
    VpcmpubKGqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W0 3E /r ib] VPCMPUB k1 {k2}, ymm2, ymm3/m256, imm8
    VpcmpubKGqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 3E /r ib] VPCMPUB k1 {k2}, zmm2, zmm3/m512, imm8
    VpcmpubKGqHdqqWdqqIbE512,

    // [EVEX.128.66.0F3A.W0 1F /r ib] VPCMPD k1 {k2}, xmm2, xmm3/m128/m32bcst, imm8
    VpcmpdKGqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W0 1F /r ib] VPCMPD k1 {k2}, ymm2, ymm3/m256/m32bcst, imm8
    VpcmpdKGqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 1F /r ib] VPCMPD k1 {k2}, zmm2, zmm3/m512/m32bcst, imm8
    VpcmpdKGqHdqqWdqqIbE512,
    // [EVEX.128.66.0F3A.W0 1E /r ib] VPCMPUD k1 {k2}, xmm2, xmm3/m128/m32bcst, imm8
    VpcmpudKGqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W0 1E /r ib] VPCMPUD k1 {k2}, ymm2, ymm3/m256/m32bcst, imm8
    VpcmpudKGqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 1E /r ib] VPCMPUD k1 {k2}, zmm2, zmm3/m512/m32bcst, imm8
    VpcmpudKGqHdqqWdqqIbE512,

    // [EVEX.128.66.0F3A.W1 1F /r ib] VPCMPQ k1 {k2}, xmm2, xmm3/m128/m64bcst, imm8
    VpcmpqKGqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 1F /r ib] VPCMPQ k1 {k2}, ymm2, ymm3/m256/m64bcst, imm8
    VpcmpqKGqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 1F /r ib] VPCMPQ k1 {k2}, zmm2, zmm3/m512/m64bcst, imm8
    VpcmpqKGqHdqqWdqqIbE512,
    // [EVEX.128.66.0F3A.W1 1E /r ib] VPCMPUQ k1 {k2}, xmm2, xmm3/m128/m64bcst, imm8
    VpcmpuqKGqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 1E /r ib] VPCMPUQ k1 {k2}, ymm2, ymm3/m256/m64bcst, imm8
    VpcmpuqKGqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 1E /r ib] VPCMPUQ k1 {k2}, zmm2, zmm3/m512/m64bcst, imm8
    VpcmpuqKGqHdqqWdqqIbE512,

    // [EVEX.128.66.0F3A.W1 3F /r ib] VPCMPW k1 {k2}, xmm2, xmm3/m128, imm8
    VpcmpwKGqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 3F /r ib] VPCMPW k1 {k2}, ymm2, ymm3/m256, imm8
    VpcmpwKGqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 3F /r ib] VPCMPW k1 {k2}, zmm2, zmm3/m512, imm8
    VpcmpwKGqHdqqWdqqIbE512,
    // [EVEX.128.66.0F3A.W1 3E /r ib] VPCMPUW k1 {k2}, xmm2, xmm3/m128, imm8
    VpcmpuwKGqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 3E /r ib] VPCMPUW k1 {k2}, ymm2, ymm3/m256, imm8
    VpcmpuwKGqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 3E /r ib] VPCMPUW k1 {k2}, zmm2, zmm3/m512, imm8
    VpcmpuwKGqHdqqWdqqIbE512,

    // [EVEX.128.66.0F38.W0 63 /r] VPCOMPRESSB m128 {k1}, xmm1
    // [EVEX.128.66.0F38.W0 63 /r] VPCOMPRESSB xmm1 {k1}{z}, xmm2
    VpcompressbWdqVdqE128,
    // [EVEX.256.66.0F38.W0 63 /r] VPCOMPRESSB m256 {k1}, ymm1
    // [EVEX.256.66.0F38.W0 63 /r] VPCOMPRESSB ymm1 {k1}{z}, ymm2
    VpcompressbWqqVqqE256,
    // [EVEX.512.66.0F38.W0 63 /r] VPCOMPRESSB m512 {k1}, zmm1
    // [EVEX.512.66.0F38.W0 63 /r] VPCOMPRESSB zmm1 {k1}{z}, zmm2
    VpcompressbWdqqVdqqE512,
    // [EVEX.128.66.0F38.W1 63 /r] VPCOMPRESSW m128 {k1}, xmm1
    // [EVEX.128.66.0F38.W1 63 /r] VPCOMPRESSW xmm1 {k1}{z}, xmm2
    VpcompresswWdqVdqE128,
    // [EVEX.256.66.0F38.W1 63 /r] VPCOMPRESSW m256 {k1}, ymm1
    // [EVEX.256.66.0F38.W1 63 /r] VPCOMPRESSW ymm1 {k1}{z}, ymm2
    VpcompresswWqqVqqE256,
    // [EVEX.512.66.0F38.W1 63 /r] VPCOMPRESSW m512 {k1}, zmm1
    // [EVEX.512.66.0F38.W1 63 /r] VPCOMPRESSW zmm1 {k1}{z}, zmm2
    VpcompresswWdqqVdqqE512,

    // [EVEX.128.66.0F38.W0 8B /r] VPCOMPRESSD xmm1/m128 {k1}{z}, xmm2
    VpcompressdWdqVdqE128,
    // [EVEX.256.66.0F38.W0 8B /r] VPCOMPRESSD ymm1/m256 {k1}{z}, ymm2
    VpcompressdWqqVqqE256,
    // [EVEX.512.66.0F38.W0 8B /r] VPCOMPRESSD zmm1/m512 {k1}{z}, zmm2
    VpcompressdWdqqVdqqE512,

    // [EVEX.128.66.0F38.W1 8B /r] VPCOMPRESSQ xmm1/m128 {k1}{z}, xmm2
    VpcompressqWdqVdqE128,
    // [EVEX.256.66.0F38.W1 8B /r] VPCOMPRESSQ ymm1/m256 {k1}{z}, ymm2
    VpcompressqWqqVqqE256,
    // [EVEX.512.66.0F38.W1 8B /r] VPCOMPRESSQ zmm1/m512 {k1}{z}, zmm2
    VpcompressqWdqqVdqqE512,

    // [EVEX.128.66.0F38.W0 C4 /r] VPCONFLICTD xmm1 {k1}{z}, xmm2/m128/m32bcst
    VpconflictdVdqWdqE128,
    // [EVEX.256.66.0F38.W0 C4 /r] VPCONFLICTD ymm1 {k1}{z}, ymm2/m256/m32bcst
    VpconflictdVqqWqqE256,
    // [EVEX.512.66.0F38.W0 C4 /r] VPCONFLICTD zmm1 {k1}{z}, zmm2/m512/m32bcst
    VpconflictdVdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 C4 /r] VPCONFLICTQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    VpconflictqVdqWdqE128,
    // [EVEX.256.66.0F38.W1 C4 /r] VPCONFLICTQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    VpconflictqVqqWqqE256,
    // [EVEX.512.66.0F38.W1 C4 /r] VPCONFLICTQ zmm1 {k1}{z}, zmm2/m512/m64bcst
    VpconflictqVdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 50 /r] VPDPBUSD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpdpbusdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 50 /r] VPDPBUSD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpdpbusdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 50 /r] VPDPBUSD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpdpbusdVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 51 /r] VPDPBUSDS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpdpbusdsVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 51 /r] VPDPBUSDS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpdpbusdsVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 51 /r] VPDPBUSDS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpdpbusdsVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 52 /r] VPDPWSSD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpdpwssdVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 52 /r] VPDPWSSD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpdpwssdVqqHqqWqqE256,
    // [EVEX.128.66.0F38.W0 52 /r] VPDPWSSD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpdpwssdVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 53 /r] VPDPWSSDS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpdpwssdsVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 53 /r] VPDPWSSDS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpdpwssdsVqqHqqWqqE256,
    // [EVEX.128.66.0F38.W0 53 /r] VPDPWSSDS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpdpwssdsVdqqHdqqWdqqE512,

    // [VEX.256.66.0F3A.W0 06 /r ib] VPERM2F128 ymm1, ymm2, ymm3/m256, imm8
    Vperm2f128VqqHqqWqqIbV256,

    // [VEX.256.66.0F3A.W0 46 /r ib] VPERM2I128 ymm1, ymm2, ymm3/m256, imm8
    Vperm2i128VqqHqqWqqIbV256,

    // [EVEX.128.66.0F38.W0 8D /r] VPERMB xmm1 {k1}{z}, xmm2, xmm3/m128
    VpermbVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 8D /r] VPERMB ymm1 {k1}{z}, ymm2, ymm3/m256
    VpermbVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 8D /r] VPERMB zmm1 {k1}{z}, zmm2, zmm3/m512
    VpermbVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 36 /r] VPERMD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpermdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 36 /r] VPERMD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpermdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 36 /r] VPERMD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpermdVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 8D /r] VPERMW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpermwVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 8D /r] VPERMW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpermwVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 8D /r] VPERMW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpermwVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 75 /r] VPERMI2B xmm1 {k1}{z}, xmm2, xmm3/m128
    Vpermi2bVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 75 /r] VPERMI2B ymm1 {k1}{z}, ymm2, ymm3/m256
    Vpermi2bVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 75 /r] VPERMI2B zmm1 {k1}{z}, zmm2, zmm3/m512
    Vpermi2bVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F38.W1 75 /r] VPERMI2W xmm1 {k1}{z}, xmm2, xmm3/m128
    Vpermi2wVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 75 /r] VPERMI2W ymm1 {k1}{z}, ymm2, ymm3/m256
    Vpermi2wVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 75 /r] VPERMI2W zmm1 {k1}{z}, zmm2, zmm3/m512
    Vpermi2wVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W0 76 /r] VPERMI2D xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vpermi2dVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 76 /r] VPERMI2D ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vpermi2dVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 76 /r] VPERMI2D zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    Vpermi2dVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 76 /r] VPERMI2Q xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vpermi2qVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 76 /r] VPERMI2Q ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vpermi2qVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 76 /r] VPERMI2Q zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    Vpermi2qVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W0 77 /r] VPERMI2PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vpermi2psVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 77 /r] VPERMI2PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vpermi2psVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 77 /r] VPERMI2PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    Vpermi2psVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 77 /r] VPERMI2PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vpermi2pdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 77 /r] VPERMI2PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vpermi2pdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 77 /r] VPERMI2PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    Vpermi2pdVdqqHdqqWdqqE512,

    // [VEX.128.66.0F38.W0 0D /r] VPERMILPD xmm1, xmm2, xmm3/m128
    VpermilpdVdqHdqWdqV128,
    // [VEX.256.66.0F38.W0 0D /r] VPERMILPD ymm1, ymm2, ymm3/m256
    VpermilpdVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W1 0D /r] VPERMILPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpermilpdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 0D /r] VPERMILPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpermilpdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 0D /r] VPERMILPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpermilpdVdqqHdqqWdqqE512,
    // [VEX.128.66.0F3A.W0 05 /r ib] VPERMILPD xmm1, xmm2/m128, imm8
    VpermilpdVdqWdqIbV128,
    // [VEX.256.66.0F3A.W0 05 /r ib] VPERMILPD ymm1, ymm2/m256, imm8
    VpermilpdVqqWqqIbV128,
    // [EVEX.128.66.0F3A.W1 05 /r ib] VPERMILPD xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VpermilpdVdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 05 /r ib] VPERMILPD ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VpermilpdVqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 05 /r ib] VPERMILPD zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VpermilpdVdqqWdqqIbE512,

    // [VEX.128.66.0F38.W0 0C /r] VPERMILPS xmm1, xmm2, xmm3/m128
    VpermilpsVdqHdqWdqV128,
    // [VEX.128.66.0F3A.W0 04 /r ib] VPERMILPS xmm1, xmm2/m128, imm8
    VpermilpsVdqWdqIbV128,
    // [VEX.256.66.0F38.W0 0C /r] VPERMILPS ymm1, ymm2, ymm3/m256
    VpermilpsVqqHqqWqqV256,
    // [VEX.256.66.0F3A.W0 04 /r ib] VPERMILPS ymm1, ymm2/m256, imm8
    VpermilpsVqqWqqIbV128,
    // [EVEX.128.66.0F38.W0 0C /r] VPERMILPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpermilpsVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 0C /r] VPERMILPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpermilpsVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 0C /r] VPERMILPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpermilpsVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F3A.W0 04 /r ib] VPERMILPS xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VpermilpsVdqWdqIbE128,
    // [EVEX.256.66.0F3A.W0 04 /r ib] VPERMILPS ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VpermilpsVqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 04 /r ib] VPERMILPS zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VpermilpsVdqqWdqqIbE512,

    // [VEX.256.66.0F3A.W1 01 /r ib] VPERMPD ymm1, ymm2/m256, imm8
    VpermpdVqqWqqIbV256,
    // [EVEX.256.66.0F3A.W1 01 /r ib] VPERMPD ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VpermpdVqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 01 /r ib] VPERMPD zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VpermpdVdqqWdqqIbE512,
    // [EVEX.256.66.0F3A.W1 16 /r ib] VPERMPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpermpdVqqHqqWqqE256,
    // [EVEX.512.66.0F3A.W1 16 /r ib] VPERMPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpermpdVdqqHdqqWdqqE512,

    // [VEX.256.66.0F38.W0 16 /r] VPERMPS ymm1, ymm2, ymm3/m256
    VpermpsVqqHqqWqqV256,
    // [EVEX.256.66.0F38.W0 16 /r] VPERMPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpermpsVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 16 /r] VPERMPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpermpsVdqqHdqqWdqqE512,

    // [VEX.256.66.0F3A.W1 00 /r ib] VPERMQ ymm1, ymm2/m256, imm8
    VpermqVqqWqqIbV256,
    // [EVEX.256.66.0F3A.W1 00 /r ib] VPERMQ ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VpermqVqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 00 /r ib] VPERMQ zmm1 {k1}{z}, xmm2/m512/m64bcst, imm8
    VpermqVdqqWdqqIbE512,
    // [EVEX.256.66.0F38.W1 36 /r] VPERMQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpermqVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 36 /r] VPERMQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpermqVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 7D /r] VPERMT2B xmm1 {k1}{z}, xmm2, xmm3/m128
    Vpermt2bVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 7D /r] VPERMT2B ymm1 {k1}{z}, ymm2, ymm3/m256
    Vpermt2bVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 7D /r] VPERMT2B zmm1 {k1}{z}, zmm2, zmm3/m512
    Vpermt2bVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F38.W1 7D /r] VPERMT2W xmm1 {k1}{z}, xmm2, xmm3/m128
    Vpermt2wVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 7D /r] VPERMT2W ymm1 {k1}{z}, ymm2, ymm3/m256
    Vpermt2wVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 7D /r] VPERMT2W zmm1 {k1}{z}, zmm2, zmm3/m512
    Vpermt2wVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W0 7E /r] VPERMT2D xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vpermt2dVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 7E /r] VPERMT2D ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vpermt2dVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 7E /r] VPERMT2D zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    Vpermt2dVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 7E /r] VPERMT2Q xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vpermt2qVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 7E /r] VPERMT2Q ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vpermt2qVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 7E /r] VPERMT2Q zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    Vpermt2qVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W0 7F /r] VPERMT2PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    Vpermt2psVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 7F /r] VPERMT2PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    Vpermt2psVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 7F /r] VPERMT2PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    Vpermt2psVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 7F /r] VPERMT2PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vpermt2pdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 7F /r] VPERMT2PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vpermt2pdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 7F /r] VPERMT2PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    Vpermt2pdVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 62 /r] VPEXPANDB xmm1 {k1}{z}, m128
    // [EVEX.128.66.0F38.W0 62 /r] VPEXPANDB xmm1 {k1}{z}, xmm2
    VpexpandbVdqWdqE128,
    // [EVEX.256.66.0F38.W0 62 /r] VPEXPANDB ymm1 {k1}{z}, m256
    // [EVEX.256.66.0F38.W0 62 /r] VPEXPANDB ymm1 {k1}{z}, ymm2
    VpexpandbVqqWqqE256,
    // [EVEX.512.66.0F38.W0 62 /r] VPEXPANDB zmm1 {k1}{z}, m512
    // [EVEX.512.66.0F38.W0 62 /r] VPEXPANDB zmm1 {k1}{z}, zmm2
    VpexpandbVdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 62 /r] VPEXPANDW xmm1 {k1}{z}, m128
    // [EVEX.128.66.0F38.W1 62 /r] VPEXPANDW xmm1 {k1}{z}, xmm2
    VpexpandwVdqWdqE128,
    // [EVEX.256.66.0F38.W1 62 /r] VPEXPANDW ymm1 {k1}{z}, m256
    // [EVEX.256.66.0F38.W1 62 /r] VPEXPANDW ymm1 {k1}{z}, ymm2
    VpexpandwVqqWqqE256,
    // [EVEX.512.66.0F38.W1 62 /r] VPEXPANDW zmm1 {k1}{z}, m512
    // [EVEX.512.66.0F38.W1 62 /r] VPEXPANDW zmm1 {k1}{z}, zmm2
    VpexpandwVdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 89 /r] VPEXPANDD xmm1 {k1}{z}, xmm2/m128
    VpexpanddVdqWdqE128,
    // [EVEX.256.66.0F38.W0 89 /r] VPEXPANDD ymm1 {k1}{z}, ymm2/m256
    VpexpanddVqqWqqE256,
    // [EVEX.512.66.0F38.W0 89 /r] VPEXPANDD zmm1 {k1}{z}, zmm2/m512
    VpexpanddVdqqWdqqE512,

    // [EVEX.128.66.0F38.W1 89 /r] VPEXPANDQ xmm1 {k1}{z}, xmm2/m128
    VpexpandqVdqWdqE128,
    // [EVEX.256.66.0F38.W1 89 /r] VPEXPANDQ ymm1 {k1}{z}, ymm2/m256
    VpexpandqVqqWqqE256,
    // [EVEX.512.66.0F38.W1 89 /r] VPEXPANDQ zmm1 {k1}{z}, zmm2/m512
    VpexpandqVdqqWdqqE512,

    // [VEX.128.66.0F38.W0 90 /r] VPGATHERDD xmm1, vm32x, xmm2
    VpgatherddVdqVMdHdqV128,
    // [VEX.128.66.0F38.W0 91 /r] VPGATHERQD xmm1, vm64x, xmm2
    VpgatherqdVdqVMqHdqV128,
    // [VEX.256.66.0F38.W0 90 /r] VPGATHERDD ymm1, vm32y, ymm2
    VpgatherddVqqVMdHqqV256,
    // [VEX.256.66.0F38.W0 91 /r] VPGATHERQD ymm1, vm64y, ymm2
    VpgatherqdVqqVMqHqqV256,

    // [EVEX.128.66.0F38.W0 90 /vsib] VPGATHERDD xmm1 {k1}, vm32x
    VpgatherddVdqVMdE128,
    // [EVEX.256.66.0F38.W0 90 /vsib] VPGATHERDD ymm1 {k1}, vm32y
    VpgatherddVqqVMdE256,
    // [EVEX.512.66.0F38.W0 90 /vsib] VPGATHERDD zmm1 {k1}, vm32z
    VpgatherddVdqqVMdE512,
    // [EVEX.128.66.0F38.W1 90 /vsib] VPGATHERDQ xmm1 {k1}, vm32x
    VpgatherdqVdqVMdE128,
    // [EVEX.256.66.0F38.W1 90 /vsib] VPGATHERDQ ymm1 {k1}, vm32y
    VpgatherdqVqqVMdE256,
    // [EVEX.512.66.0F38.W1 90 /vsib] VPGATHERDQ zmm1 {k1}, vm32z
    VpgatherdqVdqqVMdE512,

    // [VEX.128.66.0F38.W1 90 /r] VPGATHERDQ xmm1, vm32x, xmm2
    VpgatherdqVdqVMdHdqV128,
    // [VEX.128.66.0F38.W1 91 /r] VPGATHERQQ xmm1, vm64x, xmm2
    VpgatherqqVdqVMqHdqV128,
    // [VEX.256.66.0F38.W1 90 /r] VPGATHERDQ ymm1, vm32x, ymm2
    VpgatherdqVqqVMdHqqV256,
    // [VEX.256.66.0F38.W1 91 /r] VPGATHERQQ ymm1, vm64x, ymm2
    VpgatherqqVqqVMqHqqV256,

    // [EVEX.128.66.0F38.W0 91 /vsib] VPGATHERQD xmm1 {k1}, vm64x
    VpgatherqdVdqVMqE128,
    // [EVEX.256.66.0F38.W0 91 /vsib] VPGATHERQD ymm1 {k1}, vm64y
    VpgatherqdVqqVMqE256,
    // [EVEX.512.66.0F38.W0 91 /vsib] VPGATHERQD zmm1 {k1}, vm64z
    VpgatherqdVdqqVMqE512,
    // [EVEX.128.66.0F38.W1 91 /vsib] VPGATHERQQ xmm1 {k1}, vm64x
    VpgatherqqVdqVMqE128,
    // [EVEX.256.66.0F38.W1 91 /vsib] VPGATHERQQ ymm1 {k1}, vm64y
    VpgatherqqVqqVMqE256,
    // [EVEX.512.66.0F38.W1 91 /vsib] VPGATHERQQ zmm1 {k1}, vm64z
    VpgatherqqVdqqVMqE512,

    // [EVEX.128.66.0F38.W0 44 /r] VPLZCNTD xmm1 {k1}{z}, xmm2/m128/m32bcst
    VplzcntdVdqWdqE128,
    // [EVEX.256.66.0F38.W0 44 /r] VPLZCNTD ymm1 {k1}{z}, ymm2/m256/m32bcst
    VplzcntdVqqWqqE256,
    // [EVEX.512.66.0F38.W0 44 /r] VPLZCNTD zmm1 {k1}{z}, zmm2/m512/m32bcst
    VplzcntdVdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 44 /r] VPLZCNTQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    VplzcntqVdqWdqE128,
    // [EVEX.256.66.0F38.W1 44 /r] VPLZCNTQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    VplzcntqVqqWqqE256,
    // [EVEX.512.66.0F38.W1 44 /r] VPLZCNTQ zmm1 {k1}{z}, zmm2/m512/m64bcst
    VplzcntqVdqqWdqqE512,

    // [EVEX.128.66.0F38.W1 B5 /r] VPMADD52HUQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vpmadd52huqVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 B5 /r] VPMADD52HUQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vpmadd52huqVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 B5 /r] VPMADD52HUQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    Vpmadd52huqVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F38.W1 B4 /r] VPMADD52LUQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    Vpmadd52luqVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 B4 /r] VPMADD52LUQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    Vpmadd52luqVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 B4 /r] VPMADD52LUQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    Vpmadd52luqVdqqHdqqWdqqE512,

    // [VEX.128.66.0F38.W0 8C /r] VPMASKMOVD xmm1, xmm2, m128
    VpmaskmovdVdqHdqMdqV128,
    // [VEX.256.66.0F38.W0 8C /r] VPMASKMOVD ymm1, ymm2, m256
    VpmaskmovdVqqHqqMqqV256,
    // [VEX.128.66.0F38.W1 8C /r] VPMASKMOVQ xmm1, xmm2, m128
    VpmaskmovqVdqHdqMdqV128,
    // [VEX.256.66.0F38.W1 8C /r] VPMASKMOVQ ymm1, ymm2, m256
    VpmaskmovqVqqHqqMqqV256,
    // [VEX.128.66.0F38.W0 8E /r] VPMASKMOVD m128, xmm1, xmm2
    VpmaskmovdMdqHdqVdqV128,
    // [VEX.256.66.0F38.W0 8E /r] VPMASKMOVD m256, ymm1, ymm2
    VpmaskmovdMqqHqqVqqV256,
    // [VEX.128.66.0F38.W1 8E /r] VPMASKMOVQ m128, xmm1, xmm2
    VpmaskmovqMdqHdqVdqV128,
    // [VEX.256.66.0F38.W1 8E /r] VPMASKMOVQ m256, ymm1, ymm2
    VpmaskmovqMqqHqqVqqV256,

    // [EVEX.128.F3.0F38.W0 29 /r] VPMOVB2M k1, xmm1
    Vpmovb2mKGqVdqE128,
    // [EVEX.256.F3.0F38.W0 29 /r] VPMOVB2M k1, ymm1
    Vpmovb2mKGqVqqE256,
    // [EVEX.512.F3.0F38.W0 29 /r] VPMOVB2M k1, zmm1
    Vpmovb2mKGqVdqqE512,
    // [EVEX.128.F3.0F38.W1 29 /r] VPMOVW2M k1, xmm1
    Vpmovw2mKGqVdqE128,
    // [EVEX.256.F3.0F38.W1 29 /r] VPMOVW2M k1, ymm1
    Vpmovw2mKGqVqqE256,
    // [EVEX.512.F3.0F38.W1 29 /r] VPMOVW2M k1, zmm1
    Vpmovw2mKGqVdqqE512,
    // [EVEX.128.F3.0F38.W0 39 /r] VPMOVD2M k1, xmm1
    Vpmovd2mKGqVdqE128,
    // [EVEX.256.F3.0F38.W0 39 /r] VPMOVD2M k1, ymm1
    Vpmovd2mKGqVqqE256,
    // [EVEX.512.F3.0F38.W0 39 /r] VPMOVD2M k1, zmm1
    Vpmovd2mKGqVdqqE512,
    // [EVEX.128.F3.0F38.W1 39 /r] VPMOVQ2M k1, xmm1
    Vpmovq2mKGqVdqE128,
    // [EVEX.256.F3.0F38.W1 39 /r] VPMOVQ2M k1, ymm1
    Vpmovq2mKGqVqqE256,
    // [EVEX.512.F3.0F38.W1 39 /r] VPMOVQ2M k1, zmm1
    Vpmovq2mKGqVdqqE512,

    // [EVEX.128.F3.0F38.W0 31 /r] VPMOVDB xmm1/m32 {k1}{z}, xmm2
    VpmovdbWdVdqE128,
    // [EVEX.128.F3.0F38.W0 21 /r] VPMOVSDB xmm1/m32 {k1}{z}, xmm2
    VpmovsdbWdVdqE128,
    // [EVEX.128.F3.0F38.W0 11 /r] VPMOVUSDB xmm1/m32 {k1}{z}, xmm2
    VpmovusdbWdVdqE128,
    // [EVEX.256.F3.0F38.W0 31 /r] VPMOVDB xmm1/m64 {k1}{z}, ymm2
    VpmovdbWqVqqE256,
    // [EVEX.256.F3.0F38.W0 21 /r] VPMOVSDB xmm1/m64 {k1}{z}, ymm2
    VpmovsdbWqVqqE256,
    // [EVEX.256.F3.0F38.W0 11 /r] VPMOVUSDB xmm1/m64 {k1}{z}, ymm2
    VpmovusdbWqVqqE256,
    // [EVEX.512.F3.0F38.W0 31 /r] VPMOVDB xmm1/m128 {k1}{z}, zmm2
    VpmovdbWdqVdqqE512,
    // [EVEX.512.F3.0F38.W0 21 /r] VPMOVSDB xmm1/m128 {k1}{z}, zmm2
    VpmovsdbWdqVdqqE512,
    // [EVEX.512.F3.0F38.W0 11 /r] VPMOVUSDB xmm1/m128 {k1}{z}, zmm2
    VpmovusdbWdqVdqqE512,

    // [EVEX.128.F3.0F38.W0 33 /r] VPMOVDW xmm1/m64 {k1}{z}, xmm2
    VpmovdwWqVdqE128,
    // [EVEX.128.F3.0F38.W0 23 /r] VPMOVSDW xmm1/m64 {k1}{z}, xmm2
    VpmovsdwWqVdqE128,
    // [EVEX.128.F3.0F38.W0 13 /r] VPMOVUSDW xmm1/m64 {k1}{z}, xmm2
    VpmovusdwWqVdqE128,
    // [EVEX.256.F3.0F38.W0 33 /r] VPMOVDW xmm1/m128 {k1}{z}, ymm2
    VpmovdwWdqVqqE256,
    // [EVEX.256.F3.0F38.W0 23 /r] VPMOVSDW xmm1/m128 {k1}{z}, ymm2
    VpmovsdwWdqVqqE256,
    // [EVEX.256.F3.0F38.W0 13 /r] VPMOVUSDW xmm1/m128 {k1}{z}, ymm2
    VpmovusdwWdqVqqE256,
    // [EVEX.512.F3.0F38.W0 33 /r] VPMOVDW ymm1/m256 {k1}{z}, zmm2
    VpmovdwWqqVdqqE512,
    // [EVEX.512.F3.0F38.W0 23 /r] VPMOVSDW ymm1/m256 {k1}{z}, zmm2
    VpmovsdwWqqVdqqE512,
    // [EVEX.512.F3.0F38.W0 13 /r] VPMOVUSDW ymm1/m256 {k1}{z}, zmm2
    VpmovusdwWqqVdqqE512,

    // [EVEX.128.F3.0F38.W0 28 /r] VPMOVM2B xmm1, k1
    Vpmovm2bVdqKRqE128,
    // [EVEX.256.F3.0F38.W0 28 /r] VPMOVM2B ymm1, k1
    Vpmovm2bVqqKRqE256,
    // [EVEX.512.F3.0F38.W0 28 /r] VPMOVM2B zmm1, k1
    Vpmovm2bVdqqKRqE512,
    // [EVEX.128.F3.0F38.W1 28 /r] VPMOVM2W xmm1, k1
    Vpmovm2wVdqKRqE128,
    // [EVEX.256.F3.0F38.W1 28 /r] VPMOVM2W ymm1, k1
    Vpmovm2wVqqKRqE256,
    // [EVEX.512.F3.0F38.W1 28 /r] VPMOVM2W zmm1, k1
    Vpmovm2wVdqqKRqE512,
    // [EVEX.128.F3.0F38.W0 38 /r] VPMOVM2D xmm1, k1
    Vpmovm2dVdqKRqE128,
    // [EVEX.256.F3.0F38.W0 38 /r] VPMOVM2D ymm1, k1
    Vpmovm2dVqqKRqE256,
    // [EVEX.512.F3.0F38.W0 38 /r] VPMOVM2D zmm1, k1
    Vpmovm2dVdqqKRqE512,
    // [EVEX.128.F3.0F38.W1 38 /r] VPMOVM2Q xmm1, k1
    Vpmovm2qVdqKRqE128,
    // [EVEX.256.F3.0F38.W1 38 /r] VPMOVM2Q ymm1, k1
    Vpmovm2qVqqKRqE256,
    // [EVEX.512.F3.0F38.W1 38 /r] VPMOVM2Q zmm1, k1
    Vpmovm2qVdqqKRqE512,

    // [EVEX.128.F3.0F38.W0 32 /r] VPMOVQB xmm1/m16 {k1}{z}, xmm2
    VpmovqbWwVdqE128,
    // [EVEX.128.F3.0F38.W0 22 /r] VPMOVSQB xmm1/m16 {k1}{z}, xmm2
    VpmovsqbWwVdqE128,
    // [EVEX.128.F3.0F38.W0 12 /r] VPMOVUSQB xmm1/m16 {k1}{z}, xmm2
    VpmovusqbWwVdqE128,
    // [EVEX.256.F3.0F38.W0 32 /r] VPMOVQB xmm1/m32 {k1}{z}, ymm2
    VpmovqbWdVqqE256,
    // [EVEX.256.F3.0F38.W0 22 /r] VPMOVSQB xmm1/m32 {k1}{z}, ymm2
    VpmovsqbWdVqqE256,
    // [EVEX.256.F3.0F38.W0 12 /r] VPMOVUSQB xmm1/m32 {k1}{z}, ymm2
    VpmovusqbWdVqqE256,
    // [EVEX.128.F3.0F38.W0 32 /r] VPMOVQB xmm1/m64 {k1}{z}, zmm2
    VpmovqbWqVdqqE5122,
    // [EVEX.128.F3.0F38.W0 22 /r] VPMOVSQB xmm1/m64 {k1}{z}, zmm2
    VpmovsqbWqVdqqE512,
    // [EVEX.128.F3.0F38.W0 12 /r] VPMOVUSQB xmm1/m64 {k1}{z}, zmm2
    VpmovusqbWqVdqqE512,

    // [EVEX.128.F3.0F38.W0 35 /r] VPMOVQD xmm1/m64 {k1}{z}, xmm2
    // NOTE: Intel manual says m128 and "2 packed double-word integer in xmm1/m128"
    VpmovqdWqVdqE128,
    // [EVEX.128.F3.0F38.W0 5 /r] VPMOVSQD xmm1/m64 {k1}{z}, xmm2
    VpmovsqdWqVdqE128,
    // [EVEX.128.F3.0F38.W0 15 /r] VPMOVUSQD xmm1/m64 {k1}{z}, xmm2
    VpmovusqdWqVdqE128,
    // [EVEX.256.F3.0F38.W0 35 /r] VPMOVQD xmm1/m128 {k1}{z}, ymm2
    VpmovqdWdqVqqE256,
    // [EVEX.256.F3.0F38.W0 25 /r] VPMOVSQD xmm1/m128 {k1}{z}, ymm2
    VpmovsqdWdqVqqE256,
    // [EVEX.256.F3.0F38.W0 15 /r] VPMOVUSQD xmm1/m128 {k1}{z}, ymm2
    VpmovusqdWdqVqqE256,
    // [EVEX.512.F3.0F38.W0 35 /r] VPMOVQD xmm1/m256 {k1}{z}, zmm2
    VpmovqdWqqVdqqE512,
    // [EVEX.512.F3.0F38.W0 25 /r] VPMOVSQD xmm1/m256 {k1}{z}, zmm2
    VpmovsqdWqqVdqqE512,
    // [EVEX.512.F3.0F38.W0 15 /r] VPMOVUSQD xmm1/m256 {k1}{z}, zmm2
    VpmovusqdWqqVdqqE512,

    // [EVEX.128.F3.0F38.W0 34 /r] VPMOVQW xmm1/m32 {k1}{z}, xmm2
    VpmovqwWdVdqE128,
    // [EVEX.128.F3.0F38.W0 24 /r] VPMOVSQW xmm1/m32 {k1}{z}, xmm2
    VpmovsqwWdVdqE128,
    // [EVEX.128.F3.0F38.W0 14 /r] VPMOVUSQW xmm1/m32 {k1}{z}, xmm2
    VpmovusqwWdVdqE128,
    // [EVEX.256.F3.0F38.W0 34 /r] VPMOVQW xmm1/m64 {k1}{z}, ymm2
    VpmovqwWqVqqE256,
    // [EVEX.256.F3.0F38.W0 24 /r] VPMOVSQW xmm1/m64 {k1}{z}, ymm2
    VpmovsqwWqVqqE256,
    // [EVEX.256.F3.0F38.W0 14 /r] VPMOVUSQW xmm1/m64 {k1}{z}, ymm2
    VpmovusqwWqVqqE256,
    // [EVEX.512.F3.0F38.W0 34 /r] VPMOVQW xmm1/m128 {k1}{z}, zmm2
    VpmovqwWdqVdqqE512,
    // [EVEX.512.F3.0F38.W0 24 /r] VPMOVSQW xmm1/m128 {k1}{z}, zmm2
    VpmovsqwWdqVdqqE512,
    // [EVEX.521.F3.0F38.W0 14 /r] VPMOVUSQW xmm1/m128 {k1}{z}, zmm2
    VpmovusqwWdqVdqqE512,

    // [EVEX.128.F3.0F38.W0 30 /r] VPMOVWB xmm1/m64 {k1}{z}, xmm2
    VpmovwbWqVdqE128,
    // [EVEX.128.F3.0F38.W0 20 /r] VPMOVSWB xmm1/m64 {k1}{z}, xmm2
    VpmovswbWqVdqE128,
    // [EVEX.128.F3.0F38.W0 10 /r] VPMOVUSWB xmm1/m64 {k1}{z}, xmm2
    VpmovuswbWqVdqE128,
    // [EVEX.256.F3.0F38.W0 30 /r] VPMOVWB xmm1/m128 {k1}{z}, ymm2
    VpmovwbWdqVqqE256,
    // [EVEX.256.F3.0F38.W0 20 /r] VPMOVSWB xmm1/m128 {k1}{z}, ymm2
    VpmovswbWdqVqqE256,
    // [EVEX.256.F3.0F38.W0 10 /r] VPMOVUSWB xmm1/m128 {k1}{z}, ymm2
    VpmovuswbWdqVqqE256,
    // [EVEX.512.F3.0F38.W0 30 /r] VPMOVWB xmm1/m256 {k1}{z}, zmm2
    VpmovwbWqqVdqqE512,
    // [EVEX.512.F3.0F38.W0 20 /r] VPMOVSWB xmm1/m256 {k1}{z}, zmm2
    VpmovswbWqqVdqqE512,
    // [EVEX.512.F3.0F38.W0 10 /r] VPMOVUSWB xmm1/m256 {k1}{z}, zmm2
    VpmovuswbWqqVdqqE512,

    // [EVEX.128.66.0F38.W1 83 /r] VPMULTISHIFTQB xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpmultishiftqbVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 83 /r] VPMULTISHIFTQB ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpmultishiftqbVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 83 /r] VPMULTISHIFTQB zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpmultishiftqbVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 54 /r] VPOPCNTB xmm1 {k1}{z}, xmm2/m128
    VpopcntbVdqWdqE128,
    // [EVEX.256.66.0F38.W0 54 /r] VPOPCNTB ymm1 {k1}{z}, ymm2/m256
    VpopcntbVqqWqqE256,
    // [EVEX.512.66.0F38.W0 54 /r] VPOPCNTB zmm1 {k1}{z}, zmm2/m512
    VpopcntbVdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 54 /r] VPOPCNTW xmm1 {k1}{z}, xmm2/m128
    VpopcntwVdqWdqE128,
    // [EVEX.256.66.0F38.W1 54 /r] VPOPCNTW ymm1 {k1}{z}, ymm2/m256
    VpopcntwVqqWqqE256,
    // [EVEX.512.66.0F38.W1 54 /r] VPOPCNTW zmm1 {k1}{z}, zmm2/m512
    VpopcntwVdqqWdqqE512,
    // [EVEX.128.66.0F38.W0 55 /r] VPOPCNTD xmm1 {k1}{z}, xmm2/m128/m32bcst
    VpopcntdVdqWdqE128,
    // [EVEX.256.66.0F38.W0 55 /r] VPOPCNTD ymm1 {k1}{z}, ymm2/m256/m32bcst
    VpopcntdVqqWqqE256,
    // [EVEX.512.66.0F38.W0 55 /r] VPOPCNTD zmm1 {k1}{z}, zmm2/m512/m32bcst
    VpopcntdVdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 55 /r] VPOPCNTQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    VpopcntqVdqWdqE128,
    // [EVEX.256.66.0F38.W1 55 /r] VPOPCNTQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    VpopcntqVqqWqqE256,
    // [EVEX.512.66.0F38.W1 55 /r] VPOPCNTQ zmm1 {k1}{z}, zmm2/m512/m64bcst
    VpopcntqVdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 15 /r] VPROLVD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VprolvdVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 72 /1 ib] VPROLD xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VproldHdqWdqIbE128,
    // [EVEX.128.66.0F38.W1 15 /r] VPROLVQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VprolvqVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W1 72 /1 ib] VPROLQ xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VprolqHdqWdqIbE128,
    // [EVEX.256.66.0F38.W0 15 /r] VPROLVD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VprolvdVqqHqqWqqE128,
    // [EVEX.256.66.0F38.W0 72 /1 ib] VPROLD ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VproldHqqWqqIbE128,
    // [EVEX.256.66.0F38.W1 15 /r] VPROLVQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VprolvqVqqHqqWqqE128,
    // [EVEX.256.66.0F38.W1 72 /1 ib] VPROLQ ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VprolqHqqWqqIbE128,
    // [EVEX.512.66.0F38.W0 15 /r] VPROLVD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VprolvdVdqqHdqqWdqqE128,
    // [EVEX.512.66.0F38.W0 72 /1 ib] VPROLD zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VproldHdqqWdqqIbE128,
    // [EVEX.512.66.0F38.W1 15 /r] VPROLVQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VprolvqVdqqHdqqWdqqE128,
    // [EVEX.512.66.0F38.W1 72 /1 ib] VPROLQ zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VprolqHdqqWdqqIbE128,

    // [EVEX.128.66.0F38.W0 14 /r] VPRORVD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VprorvdVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W0 72 /0 ib] VPRORD xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VprordHdqWdqIbE128,
    // [EVEX.128.66.0F38.W1 14 /r] VPRORVQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VprorvqVdqHdqWdqE128,
    // [EVEX.128.66.0F38.W1 72 /0 ib] VPRORQ xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VprorqHdqWdqIbE128,
    // [EVEX.256.66.0F38.W0 14 /r] VPRORVD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VprorvdVqqHqqWqqE128,
    // [EVEX.256.66.0F38.W0 72 /0 ib] VPRORD ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VprordHqqWqqIbE128,
    // [EVEX.256.66.0F38.W1 14 /r] VPRORVQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VprorvqVqqHqqWqqE128,
    // [EVEX.256.66.0F38.W1 72 /0 ib] VPRORQ ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VprorqHqqWqqIbE128,
    // [EVEX.512.66.0F38.W0 14 /r] VPRORVD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VprorvdVdqqHdqqWdqqE128,
    // [EVEX.512.66.0F38.W0 72 /0 ib] VPRORD zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VprordHdqqWdqqIbE128,
    // [EVEX.512.66.0F38.W1 14 /r] VPRORVQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VprorvqVdqqHdqqWdqqE128,
    // [EVEX.512.66.0F38.W1 72 /0 ib] VPRORQ zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VprorqHdqqWdqqIbE128,

    // [EVEX.128.66.0F38.W0 A0 /vsib] VPSCATTERDD vm32x {k1}, xmm1
    VpscatterddVMdVdqE128,
    // [EVEX.256.66.0F38.W0 A0 /vsib] VPSCATTERDD vm32y {k1}, ymm1
    VpscatterddVMdVqqE256,
    // [EVEX.512.66.0F38.W0 A0 /vsib] VPSCATTERDD vm32z {k1}, zmm1
    VpscatterddVMdVdqqE512,
    // [EVEX.128.66.0F38.W1 A0 /vsib] VPSCATTERDQ vm32x {k1}, xmm1
    VpscatterdqVMdVdqE128,
    // [EVEX.256.66.0F38.W1 A0 /vsib] VPSCATTERDQ vm32y {k1}, ymm1
    VpscatterdqVMdVqqE256,
    // [EVEX.512.66.0F38.W1 A0 /vsib] VPSCATTERDQ vm32z {k1}, zmm1
    VpscatterdqVMdVdqqE512,
    // [EVEX.128.66.0F38.W0 A1 /vsib] VPSCATTERQD vm64x {k1}, xmm1
    VpscatterqdVMqVdqE128,
    // [EVEX.256.66.0F38.W0 A1 /vsib] VPSCATTERQD vm64y {k1}, ymm1
    VpscatterqdVMqVqqE256,
    // [EVEX.512.66.0F38.W0 A1 /vsib] VPSCATTERQD vm64z {k1}, zmm1
    VpscatterqdVMqVdqqE512,
    // [EVEX.128.66.0F38.W1 A1 /vsib] VPSCATTERQQ vm64x {k1}, xmm1
    VpscatterqqVMqVdqE128,
    // [EVEX.256.66.0F38.W1 A1 /vsib] VPSCATTERQQ vm64y {k1}, ymm1
    VpscatterqqVMqVqqE256,
    // [EVEX.512.66.0F38.W1 A1 /vsib] VPSCATTERQQ vm64z {k1}, zmm1
    VpscatterqqVMqVdqqE512,

    // [EVEX.128.66.0F3A.W1 70 /r ib] VPSHLDW xmm1 {k1}{z}, xmm2, xmm3/m128, imm8
    VpshldwVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 70 /r ib] VPSHLDW ymm1 {k1}{z}, ymm2, ymm3/m128, imm8
    VpshldwVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 70 /r ib] VPSHLDW zmm1 {k1}{z}, zmm2, zmm3/m128, imm8
    VpshldwVdqqHdqqWdqqIbE512,
    // [EVEX.128.66.0F3A.W0 71 /r ib] VPSHLDD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst, imm8
    VpshlddVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W0 71 /r ib] VPSHLDD ymm1 {k1}{z}, ymm2, ymm3/m128/m32bcst, imm8
    VpshlddVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 71 /r ib] VPSHLDD zmm1 {k1}{z}, zmm2, zmm3/m128/m32bcst, imm8
    VpshlddVdqqHdqqWdqqIbE512,
    // [EVEX.128.66.0F3A.W1 71 /r ib] VPSHLDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    VpshldqVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 71 /r ib] VPSHLDQ ymm1 {k1}{z}, ymm2, ymm3/m128/m64bcst, imm8
    VpshldqVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 71 /r ib] VPSHLDQ zmm1 {k1}{z}, zmm2, zmm3/m128/m64bcst, imm8
    VpshldqVdqqHdqqWdqqIbE512,

    // [EVEX.128.66.0F38.W1 70 /r] VPSHLDVW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpshldvwVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 70 /r] VPSHLDVW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpshldvwVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 70 /r] VPSHLDVW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpshldvwVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W0 71 /r] VPSHLDVW xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpshldvdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 71 /r] VPSHLDVW ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpshldvdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 71 /r] VPSHLDVW zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpshldvdVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 71 /r] VPSHLDVW xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpshldvqVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 71 /r] VPSHLDVW ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpshldvqVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 71 /r] VPSHLDVW zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpshldvqVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F3A.W1 72 /r ib] VPSHRDW xmm1 {k1}{z}, xmm2, xmm3/m128, imm8
    VpshrdwVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 72 /r ib] VPSHRDW ymm1 {k1}{z}, ymm2, ymm3/m128, imm8
    VpshrdwVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 72 /r ib] VPSHRDW zmm1 {k1}{z}, zmm2, zmm3/m128, imm8
    VpshrdwVdqqHdqqWdqqIbE512,
    // [EVEX.128.66.0F3A.W0 73 /r ib] VPSHRDD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst, imm8
    VpshrddVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W0 73 /r ib] VPSHRDD ymm1 {k1}{z}, ymm2, ymm3/m128/m32bcst, imm8
    VpshrddVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 73 /r ib] VPSHRDD zmm1 {k1}{z}, zmm2, zmm3/m128/m32bcst, imm8
    VpshrddVdqqHdqqWdqqIbE512,
    // [EVEX.128.66.0F3A.W1 73 /r ib] VPSHRDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    VpshrdqVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 73 /r ib] VPSHRDQ ymm1 {k1}{z}, ymm2, ymm3/m128/m64bcst, imm8
    VpshrdqVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 73 /r ib] VPSHRDQ zmm1 {k1}{z}, zmm2, zmm3/m128/m64bcst, imm8
    VpshrdqVdqqHdqqWdqqIbE512,

    // [EVEX.128.66.0F38.W1 72 /r] VPSHRDVW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpshrdvwVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 72 /r] VPSHRDVW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpshrdvwVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 72 /r] VPSHRDVW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpshrdvwVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W0 73 /r] VPSHRDVW xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpshrdvdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 73 /r] VPSHRDVW ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpshrdvdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 73 /r] VPSHRDVW zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpshrdvdVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 73 /r] VPSHRDVW xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpshrdvqVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 73 /r] VPSHRDVW ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpshrdvqVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 73 /r] VPSHRDVW zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpshrdvqVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F38.W0 8F /r] VPSHUFBITQMB k1 {k2}, xmm2, xmm3/m128
    VpshufbitqmbKGqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 8F /r] VPSHUFBITQMB k1 {k2}, ymm2, ymm3/m256
    VpshufbitqmbKGqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 8F /r] VPSHUFBITQMB k1 {k2}, zmm2, zmm3/m512
    VpshufbitqmbKGqHdqqWdqqE512,

    // [VEX.128.66.0F38.W0 47 /r] VPSLLVD xmm1, xmm2, xmm3/m128
    VpsllvdVdqHdqWdqV128,
    // [VEX.128.66.0F38.W1 47 /r] VPSLLVQ xmm1, xmm2, xmm3/m128
    VpsllvqVdqHdqWdqV128,
    // [VEX.256.66.0F38.W0 47 /r] VPSLLVD ymm1, ymm2, ymm3/m256
    VpsllvdVqqHqqWqqV256,
    // [VEX.256.66.0F38.W1 47 /r] VPSLLVQ ymm1, ymm2, ymm3/m256
    VpsllvqVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W1 12 /r] VPSLLVW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsllvwVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 12 /r] VPSLLVW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpsllvwVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 12 /r] VPSLLVW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpsllvwVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W0 47 /r] VPSLLVD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpsllvdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 47 /r] VPSLLVD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpsllvdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 47 /r] VPSLLVD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpsllvdVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 47 /r] VPSLLVQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpsllvqVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 47 /r] VPSLLVQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpsllvqVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 47 /r] VPSLLVQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpsllvqVdqqHdqqWdqqE512,

    // [VEX.128.66.0F38.W0 46 /r] VPSRAVD xmm1, xmm2, xmm3/m128
    VpsravdVdqHdqWdqV128,
    // [VEX.256.66.0F38.W0 46 /r] VPSRAVD ymm1, ymm2, ymm3/m256
    VpsravdVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W1 11 /r] VPSRAVW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsravwVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 11 /r] VPSRAVW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpsravwVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 11 /r] VPSRAVW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpsravwVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W0 46 /r] VPSRAVD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpsravdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 46 /r] VPSRAVD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpsravdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 46 /r] VPSRAVD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpsravdVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 46 /r] VPSRAVQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpsravqVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 46 /r] VPSRAVQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpsravqVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 46 /r] VPSRAVQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpsravqVdqqHdqqWdqqE512,

    // [VEX.128.66.0F38.W0 45 /r] VPSRLVD xmm1, xmm2, xmm3/m128
    VpsrlvdVdqHdqWdqV128,
    // [VEX.128.66.0F38.W1 45 /r] VPSRLVQ xmm1, xmm2, xmm3/m128
    VpsrlvqVdqHdqWdqV128,
    // [VEX.256.66.0F38.W0 45 /r] VPSRLVD ymm1, ymm2, ymm3/m256
    VpsrlvdVqqHqqWqqV256,
    // [VEX.256.66.0F38.W1 45 /r] VPSRLV! ymm1, ymm2, ymm3/m256
    VpsrlvqVqqHqqWqqV256,
    // [EVEX.128.66.0F38.W1 10 /r] VPSRLVW xmm1 {k1}{z}, xmm2, xmm3/m128
    VpsrlvwVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 10 /r] VPSRLVW ymm1 {k1}{z}, ymm2, ymm3/m256
    VpsrlvwVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 10 /r] VPSRLVW zmm1 {k1}{z}, zmm2, zmm3/m512
    VpsrlvwVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W0 45 /r] VPSRLVD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VpsrlvdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 45 /r] VPSRLVD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VpsrlvdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 45 /r] VPSRLVD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VpsrlvdVdqqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 45 /r] VPSRLVQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VpsrlvqVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 45 /r] VPSRLVQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VpsrlvqVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 45 /r] VPSRLVQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VpsrlvqVdqqHdqqWdqqE512,

    // [EVEX.128.66.0F3A.W0 25 /r ib] VPTERNLOGD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst, imm8
    VpternlogdVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W0 25 /r ib] VPTERNLOGD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst, imm8
    VpternlogdVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 25 /r ib] VPTERNLOGD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst, imm8
    VpternlogdVdqqHdqqWdqqIbE512,
    // [EVEX.128.66.0F3A.W1 25 /r ib] VPTERNLOGQ xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst, imm8
    VpternlogqVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 25 /r ib] VPTERNLOGQ ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst, imm8
    VpternlogqVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 25 /r ib] VPTERNLOGQ zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst, imm8
    VpternlogqVdqqHdqqWdqqIbE512,

    // [EVEX.128.66.0F38.W0 26 /r] VPTESTMB k2 {k1}, xmm2, xmm3/m128
    VptestmbKGqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 26 /r] VPTESTMB k2 {k1}, ymm2, ymm3/m256
    VptestmbKGqHqqWqqE256,
    // [EVEX.521.66.0F38.W0 26 /r] VPTESTMB k2 {k1}, zmm2, zmm3/m512
    VptestmbKGqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 26 /r] VPTESTMW k2 {k1}, xmm2, xmm3/m128
    VptestmwKGqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 26 /r] VPTESTMW k2 {k1}, ymm2, ymm3/m256
    VptestmwKGqHqqWqqE256,
    // [EVEX.521.66.0F38.W1 26 /r] VPTESTMW k2 {k1}, zmm2, zmm3/m512
    VptestmwKGqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W0 27 /r] VPTESTMD k2 {k1}, xmm2, xmm3/m128/m32bcst
    VptestmdKGqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 27 /r] VPTESTMD k2 {k1}, ymm2, ymm3/m256/m32bcst
    VptestmdKGqHqqWqqE256,
    // [EVEX.521.66.0F38.W0 27 /r] VPTESTMD k2 {k1}, zmm2, zmm3/m512/m32bcst
    VptestmdKGqHdqqWdqqE512,
    // [EVEX.128.66.0F38.W1 27 /r] VPTESTMQ k2 {k1}, xmm2, xmm3/m128/m64bcst
    VptestmqKGqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 27 /r] VPTESTMQ k2 {k1}, ymm2, ymm3/m256/m64bcst
    VptestmqKGqHqqWqqE256,
    // [EVEX.521.66.0F38.W1 27 /r] VPTESTMQ k2 {k1}, zmm2, zmm3/m512/m64bcst
    VptestmqKGqHdqqWdqqE512,

    // [EVEX.128.F3.0F38.W0 26 /r] VPTESTNMB k2 {k1}, xmm2, xmm3/m128
    VptestnmbKGqHdqWdqE128,
    // [EVEX.256.F3.0F38.W0 26 /r] VPTESTNMB k2 {k1}, ymm2, ymm3/m256
    VptestnmbKGqHqqWqqE256,
    // [EVEX.521.F3.0F38.W0 26 /r] VPTESTNMB k2 {k1}, zmm2, zmm3/m512
    VptestnmbKGqHdqqWdqqE512,
    // [EVEX.128.F3.0F38.W1 26 /r] VPTESTNMW k2 {k1}, xmm2, xmm3/m128
    VptestnmwKGqHdqWdqE128,
    // [EVEX.256.F3.0F38.W1 26 /r] VPTESTNMW k2 {k1}, ymm2, ymm3/m256
    VptestnmwKGqHqqWqqE256,
    // [EVEX.521.F3.0F38.W1 26 /r] VPTESTNMW k2 {k1}, zmm2, zmm3/m512
    VptestnmwKGqHdqqWdqqE512,
    // [EVEX.128.F3.0F38.W0 27 /r] VPTESTNMD k2 {k1}, xmm2, xmm3/m128/m32bcst
    VptestnmdKGqHdqWdqE128,
    // [EVEX.256.F3.0F38.W0 27 /r] VPTESTNMD k2 {k1}, ymm2, ymm3/m256/m32bcst
    VptestnmdKGqHqqWqqE256,
    // [EVEX.521.F3.0F38.W0 27 /r] VPTESTNMD k2 {k1}, zmm2, zmm3/m512/m32bcst
    VptestnmdKGqHdqqWdqqE512,
    // [EVEX.128.F3.0F38.W1 27 /r] VPTESTNMQ k2 {k1}, xmm2, xmm3/m128/m64bcst
    VptestnmqKGqHdqWdqE128,
    // [EVEX.256.F3.0F38.W1 27 /r] VPTESTNMQ k2 {k1}, ymm2, ymm3/m256/m64bcst
    VptestnmqKGqHqqWqqE256,
    // [EVEX.521.F3.0F38.W1 27 /r] VPTESTNMQ k2 {k1}, zmm2, zmm3/m512/m64bcst
    VptestnmqKGqHdqqWdqqE512,

    // [EVEX.128.66.0F3A.W1 50 /r ib] VRANGEPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    VrangepdVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 50 /r ib] VRANGEPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    VrangepdVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 50 /r ib] VRANGEPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    VrangepdVdqqHdqqWdqqIbE512,

    // [EVEX.128.66.0F3A.W0 50 /r ib] VRANGEPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst, imm8
    VrangepsVdqHdqWdqIbE128,
    // [EVEX.256.66.0F3A.W0 50 /r ib] VRANGEPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst, imm8
    VrangepsVqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 50 /r ib] VRANGEPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst, imm8
    VrangepsVdqqHdqqWdqqIbE512,

    // [EVEX.LIG.66.0F3A.W1 51 /r ib] VRANGESD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}, imm8
    VrangesdVdqHdqWdqIbE,

    // [EVEX.LIG.66.0F3A.W0 51 /r ib] VRANGESS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}, imm8
    VrangessVdqHdqWdqIbE,

    // [EVEX.128.66.0F38.W1 4C /r] VRCP14PD xmm1 {k1}{z}, xmm2/m128/m64bcst
    Vrcp14pdVdqWdqE128,
    // [EVEX.256.66.0F38.W1 4C /r] VRCP14PD ymm1 {k1}{z}, ymm2/m256/m64bcst
    Vrcp14pdVqqWqqE256,
    // [EVEX.512.66.0F38.W1 4C /r] VRCP14PD zmm1 {k1}{z}, zmm2/m512/m64bcst
    Vrcp14pdVdqqWdqqE512,

    // [EVEX.LIG.66.0F38.W1 4D /r] VRCP14SD xmm1 {k1}{z}, xmm2, xmm3/m64
    Vrcp14sdVdqHdqWqE,

    // [EVEX.128.66.0F38.W0 4C /r] VRCP14PS xmm1 {k1}{z}, xmm2/m128/m32bcst
    Vrcp14psVdqWdqE128,
    // [EVEX.256.66.0F38.W0 4C /r] VRCP14PS ymm1 {k1}{z}, ymm2/m256/m32bcst
    Vrcp14psVqqWqqE256,
    // [EVEX.512.66.0F38.W0 4C /r] VRCP14PS zmm1 {k1}{z}, zmm2/m512/m32bcst
    Vrcp14psVdqqWdqqE512,

    // [EVEX.LIG.66.0F38.W0 4D /r] VRCP14SS xmm1 {k1}{z}, xmm2, xmm3/m32
    Vrcp14ssVdqHdqWdE,

    // [EVEX.128.66.0F3A.W1 56 /r ib] VREDUCEPD xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VreducepdVdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 56 /r ib] VREDUCEPD ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VreducepdVqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 56 /r ib] VREDUCEPD zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VreducepdVdqqWdqqIbE512,

    // [EVEX.LIG.66.0F3A.W1 57 /r ib] VREDUCESD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}, imm8
    VreducesdVdqHdqWqE,

    // [EVEX.128.66.0F3A.W0 56 /r ib] VREDUCEPS xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VreducepsVdqWdqIbE128,
    // [EVEX.256.66.0F3A.W0 56 /r ib] VREDUCEPS ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VreducepsVqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 56 /r ib] VREDUCEPS zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VreducepsVdqqWdqqIbE512,

    // [EVEX.LIG.66.0F3A.W0 57 /r ib] VREDUCESS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}, imm8
    VreducessVdqHdqWdE,

    // [EVEX.128.66.0F3A.W1 09 /r ib] VRNDSCALEPD xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VrndscalepdVdqWdqIbE128,
    // [EVEX.256.66.0F3A.W1 09 /r ib] VRNDSCALEPD ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VrndscalepdVqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 09 /r ib] VRNDSCALEPD zmm1 {k1}{z}, zmm2/m512/m64bcst{sae}, imm8
    VrndscalepdVdqqWdqqIbE512,

    // [EVEX.LIG.66.0F3A.W1 0B /r ib] VRNDSCALESD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}, imm8
    VrndscalesdVdqHdqWqIbE,

    // [EVEX.128.66.0F3A.W0 08 /r ib] VRNDSCALEPS xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VrndscalepsVdqWdqIbE128,
    // [EVEX.256.66.0F3A.W0 08 /r ib] VRNDSCALEPS ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VrndscalepsVqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 08 /r ib] VRNDSCALEPS zmm1 {k1}{z}, zmm2/m512/m32bcst{sae}, imm8
    VrndscalepsVdqqWdqqIbE512,

    // [EVEX.LIG.66.0F3A.W0 0A /r ib] VRNDSCALESS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}, imm8
    VrndscalessVdqHdqWdIbE,

    // [EVEX.128.66.0F38.W1 4E /r] VRSQRT14PD xmm1 {k1}{z}, xmm2/m128/m64bcst
    Vrsqrt14pdVdqWdqE128,
    // [EVEX.256.66.0F38.W1 4E /r] VRSQRT14PD ymm1 {k1}{z}, ymm2/m256/m64bcst
    Vrsqrt14pdVqqWqqE256,
    // [EVEX.512.66.0F38.W1 4E /r] VRSQRT14PD zmm1 {k1}{z}, zmm2/m512/m64bcst
    Vrsqrt14pdVdqqWdqqE512,

    // [EVEX.LIG.66.0F38.W1 4F /r] VRSQRT14SD xmm1 {k1}{z}, xmm2, xmm3/m64
    Vrsqrt14sdVdqHdqWqE,

    // [EVEX.128.66.0F38.W0 4E /r] VRSQRT14PS xmm1 {k1}{z}, xmm2/m128/m32bcst
    Vrsqrt14psVdqWdqE128,
    // [EVEX.256.66.0F38.W0 4E /r] VRSQRT14PS ymm1 {k1}{z}, ymm2/m256/m32bcst
    Vrsqrt14psVqqWqqE256,
    // [EVEX.512.66.0F38.W0 4E /r] VRSQRT14PS zmm1 {k1}{z}, zmm2/m512/m32bcst
    Vrsqrt14psVdqqWdqqE512,

    // [EVEX.LIG.66.0F38.W0 4F /r] VRSQRT14SS xmm1 {k1}{z}, xmm2, xmm3/m32
    Vrsqrt14ssVdqHdqWdE,

    // [EVEX.128.66.0F38.W1 2C /r] VSCALEFPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VscalefpdVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W1 2C /r] VSCALEFPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VscalefpdVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W1 2C /r] VSCALEFPD zmm1 {k1}{z}, zmm2, xmm3/m512/m64bcst
    VscalefpdVdqqHdqqWdqqE512,

    // [EVEX.LIG.66.0F38.W1 2D /r] VSCALEFSD xmm1 {k1}{z}, xmm2, xmm3/m64{er}
    VscalefsdVdqHdqWqE,

    // [EVEX.128.66.0F38.W0 2C /r] VSCALEFPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VscalefpsVdqHdqWdqE128,
    // [EVEX.256.66.0F38.W0 2C /r] VSCALEFPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VscalefpsVqqHqqWqqE256,
    // [EVEX.512.66.0F38.W0 2C /r] VSCALEFPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VscalefpsVdqqHdqqWdqqE512,

    // [EVEX.LIG.66.0F38.W0 2D /r] VSCALEFSS xmm1 {k1}{z}, xmm2, xmm3/m32{er}
    VscalefssVdqHdqWdE,

    // [EVEX.128.66.0F38.W0 A2 /vsib] VSCATTERDPS vm32x {k1}, xmm1
    VscatterdpsVMdVdqE128,
    // [EVEX.256.66.0F38.W0 A2 /vsib] VSCATTERDPS vm32y {k1}, ymm1
    VscatterdpsVMdVqqE256,
    // [EVEX.512.66.0F38.W0 A2 /vsib] VSCATTERDPS vm32z {k1}, zmm1
    VscatterdpsVMdVdqqE512,
    // [EVEX.128.66.0F38.W1 A2 /vsib] VSCATTERDPD vm32x {k1}, xmm1
    VscatterdpdVMdVdqE128,
    // [EVEX.256.66.0F38.W1 A2 /vsib] VSCATTERDPD vm32y {k1}, ymm1
    VscatterdpdVMdVqqE256,
    // [EVEX.512.66.0F38.W1 A2 /vsib] VSCATTERDPD vm32z {k1}, zmm1
    VscatterdpdVMdVdqqE512,
    // [EVEX.128.66.0F38.W0 A3 /vsib] VSCATTERQPS vm64x {k1}, xmm1
    VscatterqpsVMqVdqE128,
    // [EVEX.256.66.0F38.W0 A3 /vsib] VSCATTERQPS vm64y {k1}, ymm1
    VscatterqpsVMqVqqE256,
    // [EVEX.512.66.0F38.W0 A3 /vsib] VSCATTERQPS vm64z {k1}, zmm1
    VscatterqpsVMqVdqqE512,
    // [EVEX.128.66.0F38.W1 A3 /vsib] VSCATTERQPD vm64x {k1}, xmm1
    VscatterqpdVMqVdqE128,
    // [EVEX.256.66.0F38.W1 A3 /vsib] VSCATTERQPD vm64y {k1}, ymm1
    VscatterqpdVMqVqqE256,
    // [EVEX.512.66.0F38.W1 A3 /vsib] VSCATTERQPD vm64z {k1}, zmm1
    VscatterqpdVMqVdqqE512,

    // [EVEX.256.66.0F3A.W0 23 /r ib] VSHUFF32X4 ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst, imm8
    Vshuff32x4VqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 23 /r ib] VSHUFF32X4 zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst, imm8
    Vshuff32x4VdqqHdqqWdqqIbE512,
    // [EVEX.256.66.0F3A.W1 23 /r ib] VSHUFF64X2 ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    Vshuff64x2VqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 23 /r ib] VSHUFF64X2 zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    Vshuff64x2VdqqHdqqWdqqIbE512,
    // [EVEX.256.66.0F3A.W0 43 /r ib] VSHUFI32X4 ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst, imm8
    Vshufi32x4VqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W0 43 /r ib] VSHUFI32X4 zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst, imm8
    Vshufi32x4VdqqHdqqWdqqIbE512,
    // [EVEX.256.66.0F3A.W1 43 /r ib] VSHUFI64X2 ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    Vshufi64x2VqqHqqWqqIbE256,
    // [EVEX.512.66.0F3A.W1 43 /r ib] VSHUFI64X2 zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    Vshufi64x2VdqqHdqqWdqqIbE512,

    // [VEX.128.66.0F38.W0 0E /r] VTESTPS xmm1, xmm2/m128
    VtestpsVdqWdqV128,
    // [VEX.256.66.0F38.W0 0E /r] VTESTPS ymm1, ymm2/m256
    VtestpsVqqWqqV256,
    // [VEX.128.66.0F38.W0 0F /r] VTESTPD xmm1, xmm2/m128
    VtestpdVdqWdqV128,
    // [VEX.256.66.0F38.W0 0F /r] VTESTPD ymm1, ymm2/m256
    VtestpdVqqWqqV256,

    // [VEX.256.0F.WIG 77] VZEROALL
    Vzeroall,

    // [VEX.128.0F.WIG 77] VZEROUPPER
    Vzeroupper,

    // [9B] WAIT
    Wait,

    // [0F 09] WBINVD
    Wbinvd,

    // [F3 0F 09] WBNOINVD
    Wbnoinvd,

    // [F3 0F AE /2] WRFSBASE r32
    WrfsbaseGd,
    // [F3 REX.W 0F AE /2] WRFSBASE r64
    WrfsbaseGq,
    // [F3 0F AE /3] WRGSBASE r32
    WrgsbaseGd,
    // [F3 REX.W 0F AE /3] WRGSBASE r64
    WrgsbaseGq,

    // [0F 30] WRMSR
    Wrmsr,

    // [NP 0F 01 EF] WRPKRU
    Wrpkru,

    // [0F 38 F6] WRSSD r/m32, r32
    WrssdEdGd,
    // [REX.W 0F 38 F6] WRSSQ r/m64, r64
    WrssqEqGq,

    // [66 0F 38 F5] WRUSSD r/m32, r32
    WrussdEdGd,
    // [66 REX.W 0F 38 F5] WRUSSQ r/m64, r64
    WrussqEqGq,

    // [F2] XACQUIRE
    Xacquire,
    // [F3] XRELEASE
    Xrelease,

    // [C6 F8 ib] XABORT imm8
    XabortIb,

    // [0F C0 /r] XADD r/m8, r8
    // [REX 0F C0 /r] XADD r/m8, r8
    XaddEbGb,
    // [0F C1 /r] XADD r/m16, r16
    XaddEwGw,
    // [0F C1 /r] XADD r/m32, r32
    XaddEdGd,
    // [REX.W 0F C1 /r] XADD r/m64, r64
    XaddEqGq,

    // [C7 F8] XBEGIN rel16
    XbeginIw,
    // [C7 F8] XBEGIN rel32
    XbeginId,

    // [90+rw] XCHG AX, r16
    // [90+rw] XCHG r16, AX
    XchgAXGw,
    // [90+rd] XCHG EAX, r32
    // [90+rd] XCHG r32, EAX
    XchgEAXGd,
    // [REX.W 90+rd] XCHG RAX, r64
    // [REX.W 90+rd] XCHG r64, RAX
    XchgRAXGd,
    // [86 /r] XCHG r/m8, r8
    // [REX 86 /r] XCHG r/m8, r8
    // [86 /r] XCHG r8, r/m8
    // [REX 86 /r] XCHG r8, r/m8
    XchgEbGb,
    // [87 /r] XCHG r/m16, r16
    // [87 /r] XCHG r16, r/m16
    XchgEwGw,
    // [87 /r] XCHG r/m32, r32
    // [87 /r] XCHG r32, r/m32
    XchgEdGd,
    // [REX.W 87 /r] XCHG r/m64, r64
    // [REX.W 87 /r] XCHG r64, r/m64
    XchgEqGq,

    // [NP 0F 01 D5] XEND
    Xend,

    // [NP 0F 01 D0] XGETBV
    Xgetbv,

    // [D7] XLAT m8
    // [D7] XLATB
    // [REX.W D7] XLATB
    Xlat,

    // [34 ib] XOR AL, imm8
    XorALIb,
    // [35 iw] XOR AX, imm16
    XorAXIw,
    // [35 id] XOR EAX, imm32
    XorEAXId,
    // [REX.W 35 id] XOR RAX, imm32
    XorRAXId,
    // [80 /6 ib] XOR r/m8, imm8
    // [REX 80 /6 ib] XOR r/m8, imm8
    XorEbIb,
    // [81 /6 iw] XOR r/m16, imm16
    XorEwIw,
    // [81 /6 id] XOR r/m32, imm32
    XorEdId,
    // [REX.W 81 /6 id] XOR r/m64, imm32
    XorEqId,
    // [83 /6 ib] XOR r/m16, imm8
    XorEwIb,
    // [83 /6 ib] XOR r/m32, imm8
    XorEdIb,
    // [REX.W 83 /6 ib] XOR r/m64, imm8
    XorEqIb,
    // [30 /r] XOR r/m8, r8
    // [REX 30 /r] XOR r/m8, r8
    XorEbGb,
    // [31 /r] XOR r/m16, r16
    XorEwGw,
    // [31 /r] XOR r/m32, r32
    XorEdGd,
    // [REX.W 31 /r] XOR r/m64, r64
    XorEqGq,
    // [32 /r] XOR r8, r/m8
    // [REX 32 /r] XOR r8, r/m8
    XorGbEb,
    // [33 /r] XOR r16, r/m16
    XorGwEw,
    // [33 /r] XOR r32, r/m32
    XorGdEd,
    // [REX.W 33 /r] XOR r64, r/m64
    XorGqEq,

    // [66 0F 57 /r] XORPD xmm1, xmm2/m128
    XorpdVdqWdq,
    // [VEX.128.66.0F.WIG 57 /r] VXORPD xmm1, xmm2, xmm3/m128
    VxorpdVdqHdqWdqV128,
    // [VEX.256.66.0F.WIG 57 /r] VXORPD ymm1, ymm2, ymm3/m256
    VxorpdVqqHqqWqqV256,
    // [EVEX.128.66.0F.W1 57 /r] VXORPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VxorpdVdqHdqWdqE128,
    // [EVEX.256.66.0F.W1 57 /r] VXORPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VxorpdVqqHqqWqqE256,
    // [EVEX.512.66.0F.W1 57 /r] VXORPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VxorpdVdqqHdqqWdqqE512,

    // [NP 0F 57 /r] XORPS xmm1, xmm2/m128
    XorpsVdqWdq,
    // [VEX.128.0F.WIG 57 /r] VXORPS xmm1, xmm2, xmm3/m128
    VxorpsVdqHdqWdqV128,
    // [VEX.256.0F.WIG 57 /r] VXORPS ymm1, ymm2, ymm3/m256
    VxorpsVqqHqqWqqV256,
    // [EVEX.128.0F.W1 57 /r] VXORPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VxorpsVdqHdqWdqE128,
    // [EVEX.256.0F.W1 57 /r] VXORPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VxorpsVqqHqqWqqE256,
    // [EVEX.512.0F.W1 57 /r] VXORPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VxorpsVdqqHdqqWdqqE512,

    // [NP 0F AE /5] XRSTOR mem
    Xrstor,
    // [NP REX.W 0F AE /5] XRSTOR64 mem
    Xrstor64,

    // [NP 0F C7 /3] XRSTORS mem
    Xrstors,
    // [NP REX.W 0F C7 /3] XRSTORS64 mem
    Xrstors64,

    // [NP 0F AE /4] XSAVE mem
    Xsave,
    // [NP REX.W 0F AE /4] XSAVE64 mem
    Xsave64,

    // [NP 0F C7 /4] XSAVEC mem
    Xsavec,
    // [NP REX.W 0F C7 /4] XSAVEC64 mem
    Xsavec64,

    // [NP 0F AE /6] XSAVEOPT mem
    Xsaveopt,
    // [NP REX.W 0F AE /6] XSAVEOPT64 mem
    Xsaveopt64,

    // [NP 0F C7 /5] XSAVES mem
    Xsaves,
    // [NP REX.W 0F C7 /5] XSAVES64 mem
    Xsaves64,

    // [NP 0F 01 D1] XSETBV
    Xsetbv,

    // [NP 0F 01 D6] XTEST
    Xtest,
}
