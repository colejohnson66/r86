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
// TODO: fix this
#![allow(non_camel_case_types)]
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
    AAA,

    // [D5 ib] AAD imm8
    AAD_Ib,

    // [D4 ib] AAM imm8
    AAM_Ib,

    // [3F] AAS
    AAS,

    // [14 ib] ADC AL, imm8
    ADC_ALIb,
    // [15 iw] ADC AX, imm16
    ADC_AXIw,
    // [15 id] ADC EAX, imm32
    ADC_EAXId,
    // [REX.W 15 id] ADC RAX, imm32
    ADC_RAXId,
    // [80 /2 ib] ADC r/m8, imm8
    // [REX 80 /2 ib] ADC r/m8, imm8
    ADC_EbIb,
    // [81 /2 iw] ADC r/m16, imm16
    ADC_EwIw,
    // [81 /2 iw] ADC r/m32, imm32
    ADC_EdId,
    // [REX.W 81 /2 id] ADC r/m64, imm32
    ADC_EqId,
    // [83 /2 ib] ADC r/m16, imm8
    ADC_GwIb,
    // [83 /2 ib] ADC r/m32, imm8
    ADC_GdIb,
    // [REX.W 83 /2 ib] ADDC r/m64, imm8
    ADC_GqIb,
    // [10 /r] ADC r/m8, r8
    // [REX 10 /r] ADC r/m8, r8
    ADC_EbGb,
    // [11 /r] ADC r/m16, r16
    ADC_EwGw,
    // [11 /r] ADC r/m32, r32
    ADC_EdGd,
    // [REX.W 11 /r] ADC r/m64, r64
    ADC_EqGq,
    // [12 /r] ADC r8, r/m8
    // [REX 12 /r] ADC r8, r/m8
    ADC_GbEb,
    // [13 /r] ADC r16, r/m16
    ADC_GwEw,
    // [13 /r] ADC r32, r/m32
    ADC_GdEd,
    // [REX.W 13 /r] ADC r64, r/m64
    ADC_GqEq,

    // [66 0F 38 F6 /r] ADCX r32, r/m32
    ADCX_GdEd,
    // [66 REX.W 0F 38 F6 /r] ADCX r64, r/m64
    ADCX_GqEq,

    // [04 ib] ADD AL, imm8
    ADD_ALIb,
    // [05 iw] ADD AX, imm16
    ADD_AXIw,
    // [05 id] ADD EAX, imm32
    ADD_EAXId,
    // [REX.W 05 id] ADD RAX, imm32
    ADD_RAXId,
    // [80 /0 ib] ADD r/m8, imm8
    // [REX 80 /0 ib] ADD r/m8, imm8
    ADD_EbIb,
    // [81 /0 iw] ADD r/m16, imm16
    ADD_EwIw,
    // [81 /0 iw] ADD r/m32, imm32
    ADD_EdId,
    // [REX.W 81 /0 id] ADD r/m64, imm32
    ADD_EqId,
    // [83 /0 ib] ADD r/m16, imm8
    ADD_GwIb,
    // [83 /0 ib] ADD r/m32, imm8
    ADD_GdIb,
    // [REX.W 83 /0 ib] ADDC r/m64, imm8
    ADD_GqIb,
    // [00 /r] ADD r/m8, r8
    // [REX 00 /r] ADD r/m8, r8
    ADD_EbGb,
    // [01 /r] ADD r/m16, r16
    ADD_EwGw,
    // [01 /r] ADD r/m32, r32
    ADD_EdGd,
    // [REX.W 01 /r] ADD r/m64, r64
    ADD_EqGq,
    // [02 /r] ADD r8, r/m8
    // [REX 02 /r] ADD r8, r/m8
    ADD_GbEb,
    // [03 /r] ADD r16, r/m16
    ADD_GwEw,
    // [03 /r] ADD r32, r/m32
    ADD_GdEd,
    // [REX.W 03 /r] ADD r64, r/m64
    ADD_GqEq,

    // [66 0F 58 /r] ADDPD xmm1, xmm2/m128
    ADDPD_VdqWdq,
    // [VEX.128.66.0F.WIG 58 /r] VADDPD xmm1, xmm2, xmm3/m128
    VADDPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 58 /r] VADDPD ymm1, ymm2, ymm3/m256
    VADDPD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W1 58 /r] VADDPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VADDPD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 58 /r] VADDPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VADDPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 58 /r] VADDPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VADDPD_VdqqHdqqWdqq_E512,

    // [NP 0F 58 /r] ADDPS xmm1, xmm2/m128
    ADDPS_VdqWdq,
    // [VEX.128.0F.WIG 58 /r] VADDPS xmm1, xmm2, xmm3/m128
    VADDPS_VdqHdqWdq_V128,
    // [VEX.256.0F.WIG 58 /r] VADDPS ymm1, ymm2, ymm3/m256
    VADDPS_VqqHqqWqq_V256,
    // [EVEX.128.0F.W1 58 /r] VADDPS xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VADDPS_VdqHdqWdq_E128,
    // [EVEX.256.0F.W1 58 /r] VADDPS ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VADDPS_VqqHqqWqq_E256,
    // [EVEX.512.0F.W1 58 /r] VADDPS zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VADDPS_VdqqHdqqWdqq_E512,

    // [F2 0F 58 /r] ADDSD xmm1, xmm2/m64
    ADDSD_VdqWq,
    // [VEX.LIG.F2.OF.WIG 58 /r] VADDSD xmm1, xmm2, xmm3/m64
    VADDSD_VdqHdqWq_V,
    // [EVEX.LIG.F2.0F.W1 58 /r] VADDSD xmm1 {k1}{z}, xmm2, xmm3/m64{er}
    VADDSD_VdqHdqWq_E,

    // [F3 0F 58 /r] ADDSS xmm1, xmm2/m32
    ADDSS_VdqWd,
    // [VEX.LIG.F3.0F.WIG 58 /r] VADDSS xmm1, xmm2, xmm3/m32
    VADDSS_VdqHdqWd_V,
    // [EVEX.LIG.F3.0F.W0 58 /r] VADDSS xmm1 {k1}{z}, xmm2, xmm3/m32{er}
    VADDSS_VdqHdqWd_E,

    // [66 0F D0 /r] ADDSUBPD xmm1, xmm2/m128
    ADDSUBPD_VdqWdq,
    // [VEX.128.66.0F.WIG D0 /r] VADDSUBPD xmm1, xmm2, xmm3/m128
    VADDSUBPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG D0 /r] VADDSUBPD ymm1, ymm2, ymm3/m256
    VADDSUBPD_VqqHqqWqq_V256,

    // [F2 0F D0 /r] ADDSUBPS xmm1, xmm2/m128
    ADDSUBPS_VdqWdq,
    // [VEX.128.F2.0F.WIG D0 /r] VADDSUBPS xmm1, xmm2, xmm3/m128
    VADDSUBPS_VdqHdqWdq_V128,
    // [VEX.256.F2.0F.WIG D0 /r] VADDSUBPS ymm1, ymm2, ymm3/m256
    VADDSUBPS_VqqHqqWqq_V256,

    // [F3 0F 38 F6 /r] ADOX r32, r/m32
    ADOX_GdEd,
    // [F3 REX.W 0F 38 F6 /r] ADOX r64, r/m64
    ADOX_GqEq,

    // [66 0F 38 DE /r] AESDEC xmm1, xmm2/m128
    AESDEC_VdqWdq,
    // [VEX.128.66.0F38.WIG DE /r] VAESDEC xmm1, xmm2, xmm3/m128
    VAESDEC_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG DE /r] VAESDEC ymm1, ymm2, ymm3/m256
    VAESDEC_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.WIG DE /r] VAESDEC xmm1, xmm2, xmm3/m128
    VAESDEC_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.WIG DE /r] VAESDEC ymm1, ymm2, ymm3/m256
    VAESDEC_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.WIG DE /r] VAESDEC zmm1, zmm2, zmm3/m512
    VAESDEC_VdqqHdqqWdqq_E512,

    // [66 0F 38 DF /r] AESDECLAST xmm1, xmm2/m128
    AESDECLAST_VdqWdq,
    // [VEX.128.66.0F38.WIG DF /r] VAESDECLAST xmm1, xmm2, xmm3/m128
    VAESDECLAST_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG DF /r] VAESDECLAST ymm1, ymm2, ymm3/m256
    VAESDECLAST_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.WIG DF /r] VAESDECLAST xmm1, xmm2, xmm3/m128
    VAESDECLAST_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.WIG DF /r] VAESDECLAST ymm1, ymm2, ymm3/m256
    VAESDECLAST_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.WIG DF /r] VAESDECLAST zmm1, zmm2, zmm3/m512
    VAESDECLAST_VdqqHdqqWdqq_E512,

    // [66 0F 38 DC /r] AESENC xmm1, xmm2/m128
    AESENC_VdqWdq,
    // [VEX.128.66.0F38.WIG DC /r] VAESENC xmm1, xmm2, xmm3/m128
    VAESENC_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG DC /r] VAESENC ymm1, ymm2, ymm3/m256
    VAESENC_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.WIG DC /r] VAESENC xmm1, xmm2, xmm3/m128
    VAESENC_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.WIG DC /r] VAESENC ymm1, ymm2, ymm3/m256
    VAESENC_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.WIG DC /r] VAESENC zmm1, zmm2, zmm3/m512
    VAESENC_VdqqHdqqWdqq_E512,

    // [66 0F 38 DD /r] AESENCLAST xmm1, xmm2/m128
    AESENCLAST_VdqWdq,
    // [VEX.128.66.0F38.WIG DD /r] VAESENCLAST xmm1, xmm2, xmm3/m128
    VAESENCLAST_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG DD /r] VAESENCLAST ymm1, ymm2, ymm3/m256
    VAESENCLAST_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.WIG DD /r] VAESENCLAST xmm1, xmm2, xmm3/m128
    VAESENCLAST_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.WIG DD /r] VAESENCLAST ymm1, ymm2, ymm3/m256
    VAESENCLAST_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.WIG DD /r] VAESENCLAST zmm1, zmm2, zmm3/m512
    VAESENCLAST_VdqqHdqqWdqq_E512,

    // [66 0F 38 DB /r] AESIMC xmm1, xmm2/m128
    AESIMC_VdqWdq,
    // [VEX.128.66.0F38.WIG DB /r] VAESIMC xmm1, xmm2/m128
    VAESIMC_VdqWdq_V128,

    // [66 0F 3A DF /r ib] AESKEYGENASSIST xmm1, xmm2/m128, imm8
    AESKEYGENASSIST_VdqWdqIb,
    // [VEX.128.66.0F3A.WIG DF /r ib] AESKEYGENASSIST xmm1, xmm2/m128, imm8
    VAESKEYGENASSIST_VdqWdqIb_V128,

    // [24 ib] AND AL, imm8
    AND_ALIb,
    // [25 iw] AND AX, imm16
    AND_AXIw,
    // [25 id] AND EAX, imm32
    AND_EAXId,
    // [REX.W 25 id] AND RAX, imm32
    AND_RAXId,
    // [80 /4 ib] AND r/m8, imm8
    // [REX 80 /4 ib] AND r/m8, imm8
    AND_EbIb,
    // [81 /4 iw] AND r/m16, imm16
    AND_EwIw,
    // [81 /4 iw] AND r/m32, imm32
    AND_EdId,
    // [REX.W 81 /4 id] AND r/m64, imm32
    AND_EqId,
    // [83 /4 ib] AND r/m16, imm8
    AND_GwIb,
    // [83 /4 ib] AND r/m32, imm8
    AND_GdIb,
    // [REX.W 83 /4 ib] ANDC r/m64, imm8
    AND_GqIb,
    // [20 /r] AND r/m8, r8
    // [REX 20 /r] AND r/m8, r8
    AND_EbGb,
    // [21 /r] AND r/m16, r16
    AND_EwGw,
    // [21 /r] AND r/m32, r32
    AND_EdGd,
    // [REX.W 21 /r] AND r/m64, r64
    AND_EqGq,
    // [22 /r] AND r8, r/m8
    // [REX 22 /r] AND r8, r/m8
    AND_GbEb,
    // [23 /r] AND r16, r/m16
    AND_GwEw,
    // [23 /r] AND r32, r/m32
    AND_GdEd,
    // [REX.W 23 /r] AND r64, r/m64
    AND_GqEq,

    // [VEX.LZ.0F38.W0 F2 /r] ANDN r32a, r32b, r/m32
    ANDN_GdBdEd,
    // [VEX.LZ.0F38.W1 F2 /r] ANDN r64a, r64b, r/m64
    ANDN_GqBqEq,

    // [66 0F 54 /r] ANDPD xmm1, xmm2/m128
    ANDPD_VdqWdq,
    // [VEX.128.66.0F 54 /r] VANDPD xmm1, xmm2, xmm3/m128
    VANDPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F 54 /r] VANDPD ymm1, ymm2, ymm3/m256
    VANDPD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W1 54 /r] VANDPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VANDPD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 54 /r] VANDPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VANDPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 54 /r] VANDPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VANDPD_VdqqHdqqWdqq_E512,

    // [NP 0F 54 /r] ANDPS xmm1, xmm2/m128
    ANDPS_VdqWdq,
    // [VEX.128.0F 54 /r] VANDPS xmm1, xmm2, xmm3/m128
    VANDPS_VdqHdqWdq_V128,
    // [VEX.256.0F 54 /r] VANDPS ymm1, ymm2, ymm3/m256
    VANDPS_VqqHqqWqq_V256,
    // [EVEX.128.0F.W0 54 /r] VANDPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VANDPS_VdqHdqWdq_E128,
    // [EVEX.256.0F.W0 54 /r] VANDPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VANDPS_VqqHqqWqq_E256,
    // [EVEX.512.0F.W0 54 /r] VANDPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VANDPS_VdqqHdqqWdqq_E512,

    // [66 0F 55 /r] ANDNPD xmm1, xmm2/m128
    ANDNPD_VdqWdq,
    // [VEX.128.66.0F 55 /r] VANDNPD xmm1, xmm2, xmm3/m128
    VANDNPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F 55 /r] VANDNPD ymm1, ymm2, ymm3/m256
    VANDNPD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W1 55 /r] VANDNPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VANDNPD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 55 /r] VANDNPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VANDNPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 55 /r] VANDNPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VANDNPD_VdqqHdqqWdqq_E512,

    // [NP 0F 55 /r] ANDNPS xmm1, xmm2/m128
    ANDNPS_VdqWdq,
    // [VEX.128.0F 55 /r] VANDNPS xmm1, xmm2, xmm3/m128
    VANDNPS_VdqHdqWdq_V128,
    // [VEX.256.0F 55 /r] VANDNPS ymm1, ymm2, ymm3/m256
    VANDNPS_VqqHqqWqq_V256,
    // [EVEX.128.0F.W0 55 /r] VANDNPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VANDNPS_VdqHdqWdq_E128,
    // [EVEX.256.0F.W0 55 /r] VANDNPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VANDNPS_VqqHqqWqq_E256,
    // [EVEX.512.0F.W0 55 /r] VANDNPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VANDNPS_VdqqHdqqWdqq_E512,

    // [63 /r] ARPL r/m16, r16
    ARPL_EwGw,

    // [VEX.LZ.0F38.W0 F7 /r] BEXTR r32a, r/m32, r32b
    BEXTR_GdEdBd,
    // [VEX.LZ.0F38.W1 F7 /r] BEXTR r64a, r/m64, r64b
    BEXTR_GqEqBq,

    // [66 0F 3A 0D /r ib] BLENDPD xmm1, xmm2/m128, imm8
    BLENDPD_VdqWdqIb,
    // [VEX.128.66.0F3A.WIG 0D /r ib] VBLENDPD xmm1, xmm2, xmm3/m128, imm8
    VBLENDPD_VdqHdqWdqIb_V128,
    // [VEX.256.66.0F3A.WIG 0D /r ib] VBLENDPD ymm1, ymm2, ymm3/m256, imm8
    VBLENDPD_VqqHqqWqqIb_V256,

    // [66 0F 3A 0C /r ib] BLENDPS xmm1, xmm2/m128, imm8
    BLENDPS_VdqWdqIb,
    // [VEX.128.66.0F3A.WIG 0C /r ib] VBLENDPS xmm1, xmm2, xmm3/m128, imm8
    VBLENDPS_VdqHdqWdqIb_V128,
    // [VEX.256.66.0F3A.WIG 0C /r ib] VBLENDPS ymm1, ymm2, ymm3/m256, imm8
    VBLENDPS_VqqHqqWqqIb_V256,

    // [66 0F 38 15 /r] BLENDVPD xmm1, xmm2/m128, <XMM0>
    BLENDVPD_VdqWdq,
    // [VEX.128.66.0F3A.W0 4B /r /is4] VBLENDVPD xmm1, xmm2, xmm3/m128, xmm4
    VBLENDVPD_VdqHdqWdqIb_V128,
    // [VEX.256.66.0F3A.W0 4B /r /is4] VBLENDVPD ymm1, ymm2, ymm3/m256, ymm4
    VBLENDVPD_VqqHqqWqqIb_V256,

    // [66 0F 38 14 /r] BLENDVPS xmm1, xmm2/m128, <XMM0>
    BLENDVPS_VdqWdq,
    // [VEX.128.66.0F3A.W0 4A /r /is4] VBLENDVPS xmm1, xmm2, xmm3/m128, xmm4
    VBLENDVPS_VdqHdqWdqIb_V128,
    // [VEX.256.66.0F3A.W0 4A /r /is4] VBLENDVPS ymm1, ymm2, ymm3/m256, ymm4
    VBLENDVPS_VqqHqqWqqIb_V256,

    // [VEX.LZ.0F38.W0 F3 /3] BLSI r32, r/m32
    BLSI_BdEd,
    // [VEX.LZ.0F37.W1 F3 /3] BLSI r64, r/m64
    BLSI_BqEq,

    // [VEX.LZ.0F38.W0 F3 /2] BLSMSK r32, r/m32
    BLSMSK_BdEd,
    // [VEX.LZ.0F37.W1 F3 /2] BLSMSK r64, r/m64
    BLSMSK_BqEq,

    // [VEX.LZ.0F38.W0 F3 /1] BLSR r32, r/m32
    BLSR_BdEd,
    // [VEX.LZ.0F37.W1 F3 /1] BLSR r64, r/m64
    BLSR_BqEq,

    // [F3 0F 1A /r] BNDCL bnd, r/m32
    BNDCL_BGdqEd,
    // [F3 0F 1A /r] BNDCL bnd, r/m64
    BNDCL_BGdqEq,

    // [F2 0F 1A /r] BNDCU bnd, r/m32
    BNDCU_BGdqEd,
    // [F2 0F 1A /r] BNDCU bnd, r/m64
    BNDCU_BGdqEq,
    // [F2 0F 1B /r] BNDCN bnd, r/m32
    BNDCN_BGdqEd,
    // [F2 0F 1B /r] BNDCN bnd, r/m64
    BNDCN_BGdqEq,

    // [NP 0F 1A /r] BNDLDX bnd, mib
    BNDLDX_BGdqM,

    // [F3 0F 1B /r] BNDMK bnd, m32
    BNDMK_BGdqMd,
    // [F3 0F 1B /r] BNDMK bnd, m64
    BNDMK_BGdqMq,

    // [66 0F 1A /r] BNDMOV bnd1, bnd2/m64
    BNDMOV_BGdqBEq,
    // [66 0F 1A /r] BNDMOV bnd1, bnd2/m128
    BNDMOV_BGdqBEdq,
    // [66 0F 1B /r] BNDMOV bnd1/m64, bnd2
    BNDMOV_BEqBGdq,
    // [66 0F 1B /r] BNDMOV bnd1/m128, bnd2
    BNDMOV_BEdqBGdq,

    // [NP 0F 1B /r] BNDSTX mib, bnd
    BNDSTX_MBGdq,

    // [62 /r] BOUND r16, m16&16
    BOUND_GwMa,
    // [62 /r] BOUND r32, m32&32
    BOUND_GdMa,

    // [0F BC /r] BSF r16, r/m16
    BSF_GwEw,
    // [0F BC /r] BSF r32, r/m32
    BSF_GdEd,
    // [REX.W 0F BC /r] BSF r64, r/m64
    BSF_GqEq,

    // [0F BD /r] BSR r16, r/m16
    BSR_GwEw,
    // [0F BD /r] BSR r32, r/m32
    BSR_GdEd,
    // [REX.W 0F BD /r] BSR r64, r/m64
    BSR_GqEq,

    // [0F C8+rd] BSWAP r32
    BSWAP_Gd,
    // [REX.W 0F C8+rd] BSWAP r64
    BSWAP_Gq,

    // [0F A3 /r] BT r/m16, r16
    BT_EwGw,
    // [0F A3 /r] BT r/m32, r32
    BT_EdGd,
    // [REX.W 0F A3 /r] BT r/m64, r64
    BT_EqGq,
    // [0F BA /4 ib] BT r/m16, imm8
    BT_EwIb,
    // [0F BA /4 ib] BT r/m32, imm8
    BT_EdIb,
    // [REX.W 0F BA /4 ib] BT r/m64, imm8
    BT_EqIb,

    // [0F BB /r] BTC r/m16, r16
    BTC_EwGw,
    // [0F BB /r] BTC r/m32, r32
    BTC_EdGd,
    // [REX.W 0F BB /r] BTC r/m64, r64
    BTC_EqGq,
    // [0F BA /7 ib] BTC r/m16, imm8
    BTC_EwIb,
    // [0F BA /7 ib] BTC r/m32, imm8
    BTC_EdIb,
    // [REX.W 0F BA /7 ib] BTC r/m64, imm8
    BTC_EqIb,

    // [0F B3 /r] BTR r/m16, r16
    BTR_EwGw,
    // [0F B3 /r] BTR r/m32, r32
    BTR_EdGd,
    // [REX.W 0F B3 /r] BTR r/m64, r64
    BTR_EqGq,
    // [0F BA /6 ib] BTR r/m16, imm8
    BTR_EwIb,
    // [0F BA /6 ib] BTR r/m32, imm8
    BTR_EdIb,
    // [REX.W 0F BA /6 ib] BTR r/m64, imm8
    BTR_EqIb,

    // [0F AB /r] BTS r/m16, r16
    BTS_EwGw,
    // [0F AB /r] BTS r/m32, r32
    BTS_EdGd,
    // [REX.W 0F AB /r] BTS r/m64, r64
    BTS_EqGq,
    // [0F BA /5 ib] BTS r/m16, imm8
    BTS_EwIb,
    // [0F BA /5 ib] BTS r/m32, imm8
    BTS_EdIb,
    // [REX.W 0F BA /5 ib] BTS r/m64, imm8
    BTS_EqIb,

    // [VEX.LZ.0F38.W0 F5 /r] BZHI r32a, r/m32, r32b
    BZHI_GdDdEd,
    // [VEX.LZ.0F38.W1 F5 /r] BZHI r64a, r/m64, r64b
    BZHI_GqBqEq,

    // [E8 cw] CALL rel16
    CALL_Jw,
    // [E8 cw] CALL rel32
    CALL_Jd,
    // [FF /2] CALL r/m16
    CALL_Ew,
    // [FF /2] CALL r/m32
    CALL_Ed,
    // [FF /2] CALL r/m64
    CALL_Eq,
    // [9A cd] CALL ptr16:16
    CALL_Ap_Op16,
    // [9A cp] CALL ptr16:32
    CALL_Ap_Op32,
    // [FF /3] CALL m16:16
    CALL_Ep_Op16,
    // [FF /3] CALL m16:32
    CALL_Ep_Op32,
    // [REX.W FF /3] CALL m16:64
    CALL_Ep_Op64,

    // [98] CBW
    CBW,
    // [98] CWDE
    CWDE,
    // [REX.W 98] CDQE
    CDQE,

    // [NP 0F 01 CA] CLAC
    CLAC,

    // [F8] CLC
    CLC,

    // [FC] CLD
    CLD,

    // [NP 0F 1C /0] CLDEMOTE m8
    CLDEMOTE_Mb,

    // [NP 0F AE /7] CLFLUSH m8
    CLFLUSH_Mb,

    // [NFx 66 0F AE /7] CLFLUSHOPT m8
    CLFLUSHOPT_Mb,

    // [FA] CLI
    CLI,

    // [F3 0F AE /6] CLRSSBSY m64
    CLRSSBSY_Mq,

    // [0F 06] CLTS
    CLTS,

    // [66 0F AE /6] CLWB m8
    CLWB_Mb,

    // [F5] CMC
    CMC,

    // [0F 40 /r] CMOVO r16, r/m16
    CMOVO_GwEw,
    // [0F 40 /r] CMOVO r32, r/m32
    CMOVO_GdEd,
    // [REX.W 0F 40 /r] CMOVO r64, r/m64
    CMOVO_GqEq,
    // [0F 41 /r] CMOVNO r16, r/m16
    CMOVNO_GwEw,
    // [0F 41 /r] CMOVNO r32, r/m32
    CMOVNO_GdEd,
    // [REX.W 0F 41 /r] CMOVNO r64, r/m64
    CMOVNO_GqEq,
    // [0F 42 /r] CMOVB r16, r/m16
    // [0F 42 /r] CMOVC r16, r/m16
    // [0F 42 /r] CMOVNAE r16, r/m16
    CMOVB_GwEw,
    // [0F 42 /r] CMOVB r32, r/m32
    // [0F 42 /r] CMOVC r32, r/m32
    // [0F 42 /r] CMOVNAE r32, r/m32
    CMOVB_GdEd,
    // [REX.W 0F 42 /r] CMOVB r64, r/m64
    // [REX.W 0F 42 /r] CMOVC r64, r/m64
    // [REX.W 0F 42 /r] CMOVNAE r64, r/m64
    CMOVB_GqEq,
    // [0F 43 /r] CMOVAE r16, r/m16
    // [0F 43 /r] CMOVNB r16, r/m16
    CMOVAE_GwEw,
    // [0F 43 /r] CMOVAE r32, r/m32
    // [0F 43 /r] CMOVNB r32, r/m32
    CMOVAE_GdEd,
    // [REX.W 0F 43 /r] CMOVAE r64, r/m64
    // [REX.W 0F 43 /r] CMOVNB r64, r/m64
    CMOVAE_GqEq,
    // [0F 44 /r] CMOVE r16, r/m16
    // [0F 44 /r] CMOVZ r16, r/m16
    CMOVE_GwEw,
    // [0F 44 /r] CMOVE r32, r/m32
    // [0F 44 /r] CMOVZ r32, r/m32
    CMOVE_GdEd,
    // [REX.W 0F 44 /r] CMOVE r64, r/m64
    // [REX.W 0F 44 /r] CMOVZ r64, r/m64
    CMOVE_GqEq,
    // [0F 45 /r] CMOVNE r16, r/m16
    // [0F 45 /r] CMOVNZ r16, r/m16
    CMOVNE_GwEw,
    // [0F 45 /r] CMOVNE r32, r/m32
    // [0F 45 /r] CMOVNZ r32, r/m32
    CMOVNE_GdEd,
    // [REX.W 0F 45 /r] CMOVNE r64, r/m64
    // [REX.W 0F 45 /r] CMOVNZ r64, r/m64
    CMOVNE_GqEq,
    // [0F 46 /r] CMOVBE r16, r/m16
    // [0F 46 /r] CMOVNA r16, r/m16
    CMOVBE_GwEw,
    // [0F 46 /r] CMOVBE r32, r/m32
    // [0F 46 /r] CMOVNA r32, r/m32
    CMOVBE_GdEd,
    // [REX.W 0F 46 /r] CMOVBE r64, r/m64
    // [REX.W 0F 46 /r] CMOVNA r64, r/m64
    CMOVBE_GqEq,
    // [0F 47 /r] CMOVA r16, r/m16
    // [0F 47 /r] CMOVNBE r16, r/m16
    // [0F 47 /r] CMOVNC r16, r/m16
    CMOVA_GwEw,
    // [0F 47 /r] CMOVA r32, r/m32
    // [0F 47 /r] CMOVNBE r32, r/m32
    // [0F 47 /r] CMOVNC r32, r/m32
    CMOVA_GdEd,
    // [REX.W 0F 47 /r] CMOVA r64, r/m64
    // [REX.W 0F 47 /r] CMOVNBE r64, r/m64
    // [REX.W 0F 47 /r] CMOVNC r64, r/m64
    CMOVA_GqEq,
    // [0F 48 /r] CMOVS r16, r/m16
    CMOVS_GwEw,
    // [0F 48 /r] CMOVS r32, r/m32
    CMOVS_GdEd,
    // [REX.W 0F 48 /r] CMOVS r64, r/m64
    CMOVS_GqEq,
    // [0F 49 /r] CMOVNS r16, r/m16
    CMOVNS_GwEw,
    // [0F 49 /r] CMOVNS r32, r/m32
    CMOVNS_GdEd,
    // [REX.W 0F 49 /r] CMOVNS r64, r/m64
    CMOVNS_GqEq,
    // [0F 4A /r] CMOVP r16, r/m16
    // [0F 4A /r] CMOVPE r16, r/m16
    CMOVP_GwEw,
    // [0F 4A /r] CMOVP r32, r/m32
    // [0F 4A /r] CMOVPE r32, r/m32
    CMOVP_GdEd,
    // [REX.W 0F 4A /r] CMOVP r64, r/m64
    // [REX.W 0F 4A /r] CMOVPE r64, r/m64
    CMOVP_GqEq,
    // [0F 4B /r] CMOVNP r16, r/m16
    // [0F 4B /r] CMOVPO r16, r/m16
    CMOVNP_GwEw,
    // [0F 4B /r] CMOVNP r32, r/m32
    // [0F 4B /r] CMOVPO r32, r/m32
    CMOVNP_GdEd,
    // [REX.W 0F 4B /r] CMOVNP r64, r/m64
    // [REX.W 0F 4B /r] CMOVPO r64, r/m64
    CMOVNP_GqEq,
    // [0F 4C /r] CMOVL r16, r/m16
    // [0F 4C /r] CMOVNGE r16, r/m16
    CMOVL_GwEw,
    // [0F 4C /r] CMOVL r32, r/m32
    // [0F 4C /r] CMOVNGE r32, r/m32
    CMOVL_GdEd,
    // [REX.W 0F 4C /r] CMOVL r64, r/m64
    // [REX.W 0F 4C /r] CMOVNGE r64, r/m64
    CMOVL_GqEq,
    // [0F 4D /r] CMOVGE r16, r/m16
    // [0F 4D /r] CMOVNL r16, r/m16
    CMOVGE_GwEw,
    // [0F 4D /r] CMOVGE r32, r/m32
    // [0F 4D /r] CMOVNL r32, r/m32
    CMOVGE_GdEd,
    // [REX.W 0F 4D /r] CMOVGE r64, r/m64
    // [REX.W 0F 4D /r] CMOVNL r64, r/m64
    CMOVGE_GqEq,
    // [0F 4E /r] CMOVLE r16, r/m16
    // [0F 4E /r] CMOVNG r16, r/m16
    CMOVLE_GwEw,
    // [0F 4E /r] CMOVLE r32, r/m32
    // [0F 4E /r] CMOVNG r32, r/m32
    CMOVLE_GdEd,
    // [REX.W 0F 4E /r] CMOVLE r64, r/m64
    // [REX.W 0F 4E /r] CMOVNG r64, r/m64
    CMOVLE_GqEq,
    // [0F 4F /r] CMOVG r16, r/m16
    // [0F 4F /r] CMOVNLE r16, r/m16
    CMOVG_GwEw,
    // [0F 4F /r] CMOVG r32, r/m32
    // [0F 4F /r] CMOVNLE r32, r/m32
    CMOVG_GdEd,
    // [REX.W 0F 4F /r] CMOVG r64, r/m64
    // [REX.W 0F 4F /r] CMOVNLE r64, r/m64
    CMOVG_GqEq,

    // [3C ib] CMP AL, imm8
    CMP_ALIb,
    // [3D iw] CMP AX, imm16
    CMP_AXIw,
    // [3D id] CMP EAX, imm32
    CMP_EAXId,
    // [REX.W 3D id] CMP RAX, imm32
    CMP_RAXId,
    // [80 /7 ib] CMP r/m8, imm8
    // [REX 80 /7 ib] CMP r/m8, imm8
    CMP_EbIb,
    // [81 /7 ib] CMP r/m16, imm16
    CMP_EwIw,
    // [81 /7 ib] CMP r/m32, imm32
    CMP_EdId,
    // [REX.W 81 /7 id] CMP r/m64, imm32
    CMP_EqId,
    // [83 /7 ib] CMP r/m16, imm8
    CMP_EwIb,
    // [83 /7 ib] CMP r/m32, imm8
    CMP_EdIb,
    // [REX.W 83 /7 ib] CMP r/m64, imm8
    CMP_EqIb,
    // [38 /r] CMP r/m8, r8
    // [REX 38 /r] CMP r/m8, r8
    CMP_EbGb,
    // [39 /r] CMP r/m16, r16
    CMP_EwGw,
    // [39 /r] CMP r/m32, r32
    CMP_EdGd,
    // [REX.W 39 /r] CMP r/m64, r64
    CMP_EqGq,
    // [3A /r] CMP r8, r/m8
    // [REX 3A /r] CMP r8, r/m8
    CMP_GbEb,
    // [3B /r] CMP r16, r/m16
    CMP_GwEw,
    // [3B /r] CMP r32, r/m32
    CMP_GdEd,
    // [REX.W 38 /r] CMP r64, r/m64
    CMP_GqEq,

    // [66 0F C2 /r ib] CMPPD xmm1, xmm2/m128, imm8
    CMPPD_VdqWdqIb,
    // [VEX.128.66.0F.WIG C2 /r ib] VCMPPD xmm1, xmm2, xmm3/m128, imm8
    VCMPPD_VdqHdqWdqIb_V128,
    // [VEX.256.66.0F.WIG C2 /r ib] VCMPPD ymm1, ymm2, ymm3/m256, imm8
    VCMPPD_VqqHqqWqqIb_V256,
    // [EVEX.128.66.0F.W1 C2 /r ib] VCMPPD k1 {k2}, xmm2, xmm3/m128/m64bcst, imm8
    VCMPPD_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F.W1 C2 /r ib] VCMPPD k1 {k2}, ymm2, ymm3/m256/m64bcst, imm8
    VCMPPD_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F.W1 C2 /r ib] VCMPPD k1 {k2}, zmm2, zmm3/m512/m64bcst{sae}, imm8
    VCMPPD_VdqqHdqqWdqqIb_E512,

    // [NP 0F C2 /r ib] CMPPS xmm1, xmm2/m128, imm8
    CMPPS_VdqWdqIb,
    // [VEX.128.0F.WIG C2 /r ib] VCMPPS xmm1, xmm2, xmm3/m128, imm8
    VCMPPS_VdqHdqWdqIb_V128,
    // [VEX.256.0F.WIG C2 /r ib] VCMPPS ymm1, ymm2, ymm3/m256, imm8
    VCMPPS_VqqHqqWqqIb_V256,
    // [EVEX.128.0F.W0 C2 /r ib] VCMPPS k1 {k2}, xmm2, xmm3/m128/m32bcst, imm8
    VCMPPS_VdqHdqWdqIb_E128,
    // [EVEX.256.0F.W0 C2 /r ib] VCMPPS k1 {k2}, ymm2, ymm3/m256/m32bcst, imm8
    VCMPPS_VqqHqqWqqIb_E256,
    // [EVEX.512.0F.W0 C2 /r ib] VCMPPS k1 {k2}, zmm2, zmm3/m512/m32bcst{sae}, imm8
    VCMPPS_VdqqHdqqWdqqIb_E512,

    // [A6] CMPS m8, m8
    // [A6] CMPSB m8, m8
    CMPS_XbYb,
    // [A7] CMPS m16, m16
    // [A7] CMPSW m16, m16
    CMPS_XwYw,
    // [A7] CMPS m32, m32
    // [A7] CMPSD m32, m32
    CMPS_XdYd,
    // [REX.W A7] CMPS m64, m64
    // [REX.W A7] CMPSQ m64, m64
    CMPS_XqYq,

    // [F2 0F C2 /r ib] CMPSD xmm1, xmm2/m64, imm8
    CMPSD_VdqWqIb,
    // [VEX.LIG.F2.0F.WIG C2 /r ib] VCMPSD xmm1, xmm2, xmm3/m64, imm8
    VCMPSD_VdqHdqWqIb_V,
    // [EVEX.LIG.F2.0F.W1 C2 /r ib] VCMPSD k1 {k2}, xmm2, xmm3/m64{sae}, imm8
    VCMPSD_KGqHdqWqIb_E,

    // [F3 0F C2 /r ib] CMPSS xmm1, xmm2/m32, imm8
    CMPSS_VdqWdIb,
    // [VEX.LIG.F3.0F.WIG C2 /r ib] VCMPSS xmm1, xmm2, xmm3/m32, imm8
    VCMPSS_VdqHdqWdIb_V,
    // [EVEX.LIG.F3.0F.W0 C2 /r ib] VCMPSS k1 {k2}, xmm2, xmm3/m32{sae}, imm8
    VCMPSS_KGqHdqWdIb_E,

    // [0F B0 /r] CMPXCHG r/m8, r8
    // [REX 0F B0 /r] CMPXCHG r/m8, r8
    CMPXCHG_EbGb,
    // [0F B1 /r] CMPXCHG r/m16, r16
    CMPXCHG_EwGw,
    // [0F B1 /r] CMPXCHG r/m32, r32
    CMPXCHG_EdGd,
    // [REX.W 0F B1 /r] CMPXCHG r/m64, r64
    CMPXCHG_EqGq,

    // [0F C7 /1] CMPXCHG8B m64
    CMPXCHG8B_Mq,
    // [REX.W 0F C7 /1] CMPXCHG m128
    CMPXCHG16B_Mdq,

    // [66 0F 2F /r] COMISD xmm1, xmm2/m64
    COMISD_VdqWq,
    // [VEX.LIG.66.0F.WIG 2F /r] VCOMISD xmm1, xmm2/m64
    VCOMISD_VdqWq_V,
    // [EVEX.LIG.66.0F.W1 2F /r] VCOMISD xmm1, xmm2/m64{sae}
    VCOMISD_VdqWq_E,

    // [NP 0F 2F /r] COMISS xmm1, xmm2/m32
    COMISS_VdqWd,
    // [VEX.LIG.0F.WIG 2F /r] VCOMISS xmm1, xmm2/m32
    VCOMISS_VdqWd_V,
    // [EVEX.LIG.0F.W0 2F /r] VCOMISS xmm1, xmm2/m32{sae}
    VCOMISS_VdqWd_E,

    // [0F A2] CPUID
    CPUID,

    // [F2 0F 38 F0 /r] CRC32 r32, r/m8
    // [F2 REX 0F 38 F0 /r] CRC32 r32, r/m8
    CRC32_GdEb,
    // [F2 0F 38 F1 /r] CRC32 r32, r/m16
    CRC32_GdEw,
    // [F2 0F 38 F1 /r] CRC32 r32, r/m32
    CRC32_GdEd,
    // [F2 REX.W 0F 38 F0 /r] CRC32 r64, r/m8
    CRC32_GqEb,
    // [F2 REX.W 0F 38 F1 /r] CRC32 r64, r/m64
    CRC32_GqEq,

    // [F3 0F E6 /r] CVTDQ2PD xmm1, xmm2/m64
    CVTDQ2PD_VdqWq,
    // [VEX.128.F3.0F.WIG E6 /r] VCVTDQ2PD xmm1, xmm2/m64
    VCVTDQ2PD_VdqWq_V128,
    // [VEX.256.F3.0F.WIG E6 /r] VCVTDQ2PD ymm1, xmm2/m128
    VCVTDQ2PD_VqqWdq_V256,
    // [EVEX.128.F3.0F.W0 E6 /r] VCVTDQ2PD xmm1 {k1}{z}, xmm2/m64/m32bcst
    VCVTDQ2PD_VdqWq_E128,
    // [EVEX.256.F3.0F.W0 E6 /r] VCVTDQ2PD ymm1 {k1}{z}, xmm2/m128/m32bcst
    VCVTDQ2PD_VqqWdq_E256,
    // [EVEX.512.F3.0F.W0 E6 /r] VCVTDQ2PD zmm1 {k1}{z}, ymm2/m256/m32bcst
    VCVTDQ2PD_VdqqWqq_E512,

    // [NP 0F 5B /r] CVTDQ2PD xmm1, xmm2/m128
    CVTDQ2PS_VdqWdq,
    // [VEX.128.0F.WIG 5B /r] VCVTDQ2PD xmm1, xmm2/m128
    VCVTDQ2PS_VdqWdq_V128,
    // [VEX.256.0F.WIG 5B /r] VCVTDQ2PD ymm1, ymm2/m256
    VCVTDQ2PS_VqqWqq_V256,
    // [EVEX.128.0F.W0 5B /r] VCVTDQ2PD xmm1 {k1}{z}, xmm2/m128/m32bcst
    VCVTDQ2PS_VdqWdq_E128,
    // [EVEX.256.0F.W0 5B /r] VCVTDQ2PD ymm1 {k1}{z}, ymm2/m256/m32bcst
    VCVTDQ2PS_VqqWqq_E256,
    // [EVEX.512.0F.W0 5B /r] VCVTDQ2PD zmm1 {k1}{z}, zmm2/m512/m32bcst{er}
    VCVTDQ2PS_VdqqWdqq_E512,

    // [EVEX.128.F2.0F38.W0 72 /r] VCVTNE2PS2BF16 xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VCVTNE2PS2BF16_VdqHdqWdq_E128,
    // [EVEX.256.F2.0F38.W0 72 /r] VCVTNE2PS2BF16 ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VCVTNE2PS2BF16_VqqHqqWqq_E256,
    // [EVEX.512.F2.0F38.W0 72 /r] VCVTNE2PS2BF16 zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VCVTNE2PS2BF16_VdqqHdqqWdqq_E512,

    // [EVEX.128.F3.0F38.W0 72 /r] VCVTNEPS2BF16 xmm1 {k1}{z}, xmm2/m128/m32bcst
    VCVTNEPS2BF16_VdqWdq_E128,
    // [EVEX.256.F3.0F38.W0 72 /r] VCVTNEPS2BF16 ymm1 {k1}{z}, ymm2/m256/m32bcst
    VCVTNEPS2BF16_VqqWqq_E256,
    // [EVEX.512.F3.0F38.W0 72 /r] VCVTNEPS2BF16 zmm1 {k1}{z}, zmm2/m512/m32bcst
    VCVTNEPS2BF16_VdqqWdqq_E512,

    // [F2 0F E6 /r] CVTPD2DQ xmm1, xmm2/m128
    CVTPD2DQ_VdqWdq,
    // [VEX.128.F2.0F.WIG E6 /r] VCVTPD2DQ xmm1, xmm2/m128
    VCVTPD2DQ_VdqWdq_V128,
    // [VEX.256.F2.0F.WIG E6 /r] VCVTPD2DQ xmm1, ymm2/m256
    VCVTPD2DQ_VdqWqq_V256,
    // [EVEX.128.F2.0F.W1 E6 /r] VCVTPD2DQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    VCVTPD2DQ_VdqWdq_E128,
    // [EVEX.256.F2.0F.W1 E6 /r] VCVTPD2DQ xmm1 {k1}{z}, ymm2/m256/m64bcst
    VCVTPD2DQ_VdqWqq_E256,
    // [EVEX.512.F2.0F.W1 E6 /r] VCVTPD2DQ ymm1 {k1}{z}, zmm2/m512/m64bcst{er}
    VCVTPD2DQ_VqqWdqq_E512,

    // [66 0F 2D /r] CVTPD2PI mm, xmm/m128
    CVTPD2PI_PqWdq,

    // [66 0F 5A /r] CVTPD2PS xmm1, xmm2/m128
    CVTPD2PS_VdqWdq,
    // [VEX.128.66.0F.WIG 5A /r] VCVTPD2PS xmm1, xmm2/m128
    VCVTPD2PS_VdqWdq_V128,
    // [VEX.256.66.0F.WIG 5A /r] VCVTPD2PS xmm1, ymm2/m256
    VCVTPD2PS_VdqWqq_V256,
    // [EVEX.128.66.0F.W1 5A /r] VCVTPD2PS xmm1 {k1}{z}, xmm2/m128/m64bcst
    VCVTPD2PS_VdqWdq_E128,
    // [EVEX.256.66.0F.W1 5A /r] VCVTPD2PS xmm1 {k1}{z}, ymm2/m256/m64bcst
    VCVTPD2PS_VdqWqq_E256,
    // [EVEX.512.66.0F.W1 5A /r] VCVTPD2PS ymm1 {k1}{z}, zmm2/m512/m64bcst{er}
    VCVTPD2PS_VqqWdqq_E512,

    // [66 0F 2A /r] CVTPI2PD xmm, mm/m64
    CVTPI2PD_VdqQq,

    // [NP 0F 2A /r] CVTPI2PS xmm, mm/m64
    CVTPI2PS_VdqQq,

    // [66 0F 5B /r] CVTPS2DQ xmm1, xmm2/m128
    CVTPS2DQ_VdqWdq,
    // [VEX.128.66.0F.WIG 5B /r] VCVTPS2DQ xmm1, xmm2/m128
    VCVTPS2DQ_VdqWdq_V128,
    // [VEX.256.66.0F.WIG 5B /r] VCVTPS2DQ ymm1, ymm2/m256
    VCVTPS2DQ_VqqWqq_V256,
    // [EVEX.128.66.0F.W0 5B /r] VCVTPS2DQ xmm1 {k1}{z}, xmm2/m128/m32bcst
    VCVTPS2DQ_VdqWdq_E128,
    // [EVEX.256.66.0F.W0 5B /r] VCVTPS2DQ ymm1 {k1}{z}, ymm2/m256/m32bcst
    VCVTPS2DQ_VqqWqq_E256,
    // [EVEX.512.66.0F.W0 5B /r] VCVTPS2DQ zmm1 {k1}{z}, zmm2/m512/m32bcst{er}
    VCVTPS2DQ_VdqqWdqq_E512,

    // [NP 0F 5A /r] CVTPS2PD xmm1, xmm2/m64
    CVTPS2PD_VdqWq,
    // [VEX.128.0F.WIG 5A /r] VCVTPS2PD xmm1, xmm2/m64
    VCVTPS2PD_VdqWq_V128,
    // [VEX.256.0F.WIG 5A /r] VCVTPS2PD ymm1, xmm2/m128
    VCVTPS2PD_VqqWdq_V256,
    // [EVEX.128.0F.W0 5A /r] VCVTPS2PD xmm1 {k1}{z}, xmm2/m64/m32bcst
    VCVTPS2PD_VdqWq_E128,
    // [EVEX.256.0F.W0 5A /r] VCVTPS2PD ymm1 {k1}{z}, xmm2/m128/m32bcst
    VCVTPS2PD_VqqWdq_E256,
    // [EVEX.512.0F.W0 5A /r] VCVTPS2PD zmm1 {k1}{z}, ymm2/m256/m32bcst{er}
    VCVTPS2PD_VdqqWqq_E512,

    // [NP 0F 2D /r] CVTPS2PI mm, xmm/m64
    CVTPS2PI_PqWq,

    // [F2 0F 2D /r] CVTSD2SI r32, xmm1/m64
    CVTSD2SI_GdWq,
    // [F2 REX.W 0F 2D /r] CVTSD2SI r64, xmm1/m64
    CVTSD2SI_GqWq,
    // [VEX.LIG.F2.0F.W0 2D /r] VCVTSD2SI r32, xmm1/m64
    VCVTSD2SI_GdWq_V,
    // [EVEX.LIG.F2.0F.W0 2D /r] VCVTSD2SI r32, xmm1/m64{er}
    VCVTSD2SI_GdWq_E,
    // [VEX.LIG.F2.0F.W1 2D /r] VCVTSD2SI r64, xmm1/m64
    VCVTSD2SI_GqWq_V,
    // [EVEX.LIG.F2.0F.W1 2D /r] VCVTSD2SI r64, xmm1/m64{er}
    VCVTSD2SI_GqWq_E,

    // [F2 0F 5A /r] CVTSD2SS xmm1, xmm2/m64
    CVTSD2SS_VdqWq,
    // [VEX.LIG.F2.0F.WIG 5A /r] VCVTSD2SS xmm1, xmm2, xmm3/m64
    VCVTSD2SS_VdqHdqWq_V,
    // [EVEX.LIG.F2.0F.W1 5A /r] VCVTSD2SS xmm1 {k1}{z}, xmm2, xmm3/m64{er}
    VCVTSD2SS_VdqHdqWq_E,

    // [F2 0F 2A /r] CVTSI2SD xmm1, r/m32
    CVTSI2SD_VdqEd,
    // [F2 REX.W 0F 2A /r] CVTSI2SD xmm1, r/m64
    CVTSI2SD_VdqEq,
    // [VEX.LIG.F2.0F.W0 2A /r] VCVTSI2SD xmm1, xmm2, r/m32
    VCVTSI2SD_VdqHdqEd_V,
    // [EVEX.LIG.F2.0F.W0 2A /r] VCVTSI2SD xmm1, xmm2, r/m32
    VCVTSI2SD_VdqHdqEd_E,
    // [VEX.LIG.F2.0F.W1 2A /r] VCVTSI2SD xmm1, xmm2, r/m64
    VCVTSI2SD_VdqHdqEq_V,
    // [EVEX.LIG.F2.0F.W1 2A /r] VCVTSI2SD xmm1, xmm2, r/m64{er}
    VCVTSI2SD_VdqHdqEq_E,

    // [F3 0F 2A /r] CVTSI2SS xmm1, r/m32
    CVTSI2SS_VdqEd,
    // [F2 REX.W 0F 2A /r] CVTSI2SS xmm1, r/m64
    CVTSI2SS_VdqEq,
    // [VEX.LIG.F3.0F.W0 2A /r] VCVTSI2SS xmm1, xmm2, r/m32
    VCVTSI2SS_VdqHdqEd_V,
    // [EVEX.LIG.F3.0F.W0 2A /r] VCVTSI2SS xmm1, xmm2, r/m32
    VCVTSI2SS_VdqHdqEd_E,
    // [VEX.LIG.F3.0F.W1 2A /r] VCVTSI2SS xmm1, xmm2, r/m64
    VCVTSI2SS_VdqHdqEq_V,
    // [EVEX.LIG.F3.0F.W1 2A /r] VCVTSI2SS xmm1, xmm2, r/m64{er}
    VCVTSI2SS_VdqHdqEq_E,

    // [F3 0F 5A /r] CVTSS2SD xmm1, xmm2/m32
    CVTSS2SD_VdqWd,
    // [VEX.LIG.F3.0F.WIG 5A /r] VCVTSS2SD xmm1, xmm2, xmm3/m32
    VCVTSS2SD_VdqHdqWd_V,
    // [EVEX.LIG.F3.0F.W0 5A /r] VCVTSS2SD xmm1 {k1}{z}, xmm2, xmm3/m32{sae}
    VCVTSS2SD_VdqHdqWd_E,

    // [F3 0F 2D /r] CVTSS2SI r32, xmm1/m32
    CVTSS2SI_GdWd,
    // [F3 REX.W 0F 2D /r] CVTSS2SI r64, xmm1/m32
    CVTSS2SI_GqWd,
    // [VEX.LIG.F3.0F.W0 2D /r] VCVTSS2SI r32, xmm1/m32
    VCVTSS2SI_GdWd_V,
    // [EVEX.LIG.F3.0F.W0 2D /r] VCVTSS2SI r32, xmm1/m32{er}
    VCVTSS2SI_GdWd_E,
    // [VEX.LIG.F3.0F.W1 2D /r] VCVTSS2SI r64, xmm1/m32
    VCVTSS2SI_GqWd_V,
    // [EVEX.LIG.F3.0F.W1 2D /r] VCVTSS2SI r64, xmm1/m32{er}
    VCVTSS2SI_GqWd_E,

    // [66 0F E6 /r] CVTTPD2DQ xmm1, xmm2/m128
    CVTTPD2DQ_VdqWdq,
    // [VEX.128.66.0F.WIG E6 /r] VCVTTPD2DQ xmm1, xmm2/m128
    VCVTTPD2DQ_VdqWdq_V128,
    // [VEX.256.66.0F.WIG E6 /r] VCVTTPD2DQ xmm1, ymm2/m256
    VCVTTPD2DQ_VdqWqq_V256,
    // [EVEX.128.66.0F.W1 E6 /r] VCVTTPD2DQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    VCVTTPD2DQ_VdqWdq_E128,
    // [EVEX.256.66.0F.W1 E6 /r] VCVTTPD2DQ xmm1 {k1}{z}, ymm2/m256/m64bcst
    VCVTTPD2DQ_VdqWqq_E256,
    // [EVEX.512.66.0F.W1 E6 /r] VCVTTPD2DQ ymm1 {k1}{z}, zmm2/m512/m64bcst{sae}
    VCVTTPD2DQ_VqqWdqq_E512,

    // [66 0F 2C /r] CVTTPD2PI mm, xmm/m128
    CVTTPD2PI_PqWdq,

    // [F3 0F 5B /r] CVTTPS2DQ xmm1, xmm2/m128
    CVTTPS2DQ_VdqWdq,
    // [VEX.128.F3.0F.WIG 5B /r] VCVTTPS2DQ xmm1, xmm2/m128
    VCVTTPS2DQ_VdqWdq_V128,
    // [VEX.256.F3.0F.WIG 5B /r] VCVTTPS2DQ ymm1, ymm2/m256
    VCVTTPS2DQ_VqqWqq_V256,
    // [EVEX.128.F3.0F.W0 5B /r] VCVTTPS2DQ xmm1 {k1}{z}, xmm2/m128/m32bcst
    VCVTTPS2DQ_VdqWdq_E128,
    // [EVEX.256.F3.0F.W0 5B /r] VCVTTPS2DQ ymm1 {k1}{z}, ymm2/m256/m32bcst
    VCVTTPS2DQ_VqqWqq_E256,
    // [EVEX.512.F3.0F.W0 5B /r] VCVTTPS2DQ zmm1 {k1}{z}, zmm2/m512/m32bcst
    VCVTTPS2DQ_VdqqWdqq_E512,

    // [NP 0F 2C /r] CVTTPS2PI mm, xmm/m64
    CVTTPS2PI_PqWq,

    // [F2 0F 2C /r] CVTTSD2SI r32, xmm1/m64
    CVTTSD2SI_GdWq,
    // [F2 REX.W 0F 2C /r] CVTTSD2SI r64, xmm1/m64
    CVTTSD2SI_GqWq,
    // [VEX.LIG.F2.0F.W0 2C /r] VCVTTSD2SI r32, xmm1/m64
    VCVTTSD2SI_GdWq_V,
    // [EVEX.LIG.F2.0F.W0 2C /r] VCVTTSD2SI r32, xmm1/m64{sae}
    VCVTTSD2SI_GdWq_E,
    // [VEX.LIG.F2.0F.W1 2C /r] VCVTTSD2SI r64, xmm1/m64
    VCVTTSD2SI_GqWq_V,
    // [EVEX.LIG.F2.0F.W1 2C /r] VCVTTSD2SI r64, xmm1/m64{sae}
    VCVTTSD2SI_GqWq_E,

    // [F3 0F 2C /r] CVTTSS2SI r32, xmm1/m64
    CVTTSS2SI_GdWq,
    // [F3 REX.W 0F 2C /r] CVTTSS2SI r64, xmm1/m64
    CVTTSS2SI_GqWq,
    // [VEX.LIG.F3.0F.W0 2C /r] VCVTTSS2SI r32, xmm1/m64
    VCVTTSS2SI_GdWq_V,
    // [EVEX.LIG.F3.0F.W0 2C /r] VCVTTSS2SI r32, xmm1/m64{sae}
    VCVTTSS2SI_GdWq_E,
    // [VEX.LIG.F3.0F.W1 2C /r] VCVTTSS2SI r64, xmm1/m64
    VCVTTSS2SI_GqWq_V,
    // [EVEX.LIG.F3.0F.W1 2C /r] VCVTTSS2SI r64, xmm1/m64{sae}
    VCVTTSS2SI_GqWq_E,

    // [99] CWD
    CWD,
    // [99] CDQ
    CDQ,
    // [REX.W 99]
    CQO,

    // [27] DAA
    DAA,

    // [2F] DAS
    DAS,

    // [FE /1] DEC r/m8
    // [REX FE /1] DEC r/m8
    DEC_Eb,
    // [FF /1] DEC r/m16
    DEC_Ew,
    // [FF /1] DEC r/m32
    DEC_Ed,
    // [REX.W FF /1] DEC r/m64
    DEC_Eq,
    // [48+rw] DEC r16
    DEC_Gw,
    // [48+rd] DEC r32
    DEC_Gd,

    // [F6 /6] DIV r/m8
    // [REX F6 /6] DIV r/m8
    DIV_Eb,
    // [F7 /6] DIV r/m16
    DIV_Ew,
    // [F7 /6] DIV r/m32
    DIV_Ed,
    // [REX.W F7 /6] DIV r/m64
    DIV_Eq,

    // [66 0F 5E /r] DIVPD xmm1, xmm2/m128
    DIVPD_VdqWdq,
    // [VEX.128.66.0F.WIG 5E /r] VDIVPD xmm1, xmm2, xmm3/m128
    VDIVPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 5E /r] VDIVPD ymm1, ymm2, ymm3/m256
    VDIVPD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W1 5E /r] VDIVPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VDIVPD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 5E /r] VDIVPD ymm1 {k1}{z}, ymm2, ymm3/m128/m64bcst
    VDIVPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 5E /r] VDIVPD zmm1 {k1}{z}, zmm2, zmm3/m128/m64bcst{er}
    VDIVPD_VdqqHdqqWdqq_E512,

    // [NP 0F 5E /r] DIVPD xmm1, xmm2/m128
    DIVPS_VdqWdq,
    // [VEX.128.0F.WIG 5E /r] VDIVPS xmm1, xmm2, xmm3/m128
    VDIVPS_VdqHdqWdq_V128,
    // [VEX.256.0F.WIG 5E /r] VDIVPS ymm1, ymm2, ymm3/m256
    VDIVPS_VqqHqqWqq_V256,
    // [EVEX.128.0F.W1 5E /r] VDIVPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VDIVPS_VdqHdqWdq_E128,
    // [EVEX.256.0F.W1 5E /r] VDIVPS ymm1 {k1}{z}, ymm2, ymm3/m128/m32bcst
    VDIVPS_VqqHqqWqq_E256,
    // [EVEX.512.0F.W1 5E /r] VDIVPS zmm1 {k1}{z}, zmm2, zmm3/m128/m32bcst{er}
    VDIVPS_VdqqHdqqWdqq_E512,

    // [F2 0F 5E /r] DIVSD xmm1, xmm2/m64
    DIVSD_VdqWq,
    // [VEX.LIG.F2.0F.WIG 5E /r] VDIVSD xmm1, xmm2, xmm3/m64
    VDIVSD_VdqHdqWq_V,
    // [EVEX.LIG.F2.0F.W1 5E /r] VDIVSD xmm1 {k1}{z}, xmm2, xmm3/m64{er}
    VDIVSD_VqqHqqWq_E,

    // [F3 0F 5E /r] DIVSS xmm1, xmm2/m64
    DIVSS_VdqWq,
    // [VEX.LIG.F3.0F.WIG 5E /r] VDIVSS xmm1, xmm2, xmm3/m32
    VDIVSS_VdqHdqWd_V,
    // [EVEX.LIG.F3.0F.W1 5E /r] VDIVSS xmm1 {k1}{z}, xmm2, xmm3/m32{er}
    VDIVSS_VdqHdqWd_E,

    // [EVEX.128.F3.0F38.W0 52 /r] VDPBF16PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VDPBF16PS_VdqHdqWdq_E128,
    // [EVEX.256.F3.0F38.W0 52 /r] VDPBF16PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VDPBF16PS_VqqHqqWqq_E256,
    // [EVEX.512.F3.0F38.W0 52 /r] VDPBF16PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VDPBF16PS_VdqqHdqqWdqq_E512,

    // [66 0F 3A 41 /r ib] DPPD xmm1, xmm2/m128, imm8
    DPPD_VdqWdqIb,
    // [VEX.128.66.0F3A.WIG 41 /r ib] VDPPD xmm1, xmm2, xmm3/m128, imm8
    VDPPD_VdqHdqWdqIb_V128,

    // [66 0F 3A 40 /r ib] DPPS xmm1, xmm2/m128, imm8
    DPPS_VdqWdqIb,
    // [VEX.128.66.0F3A.WIG 40 /r ib] VDPPS xmm1, xmm2, xmm3/m128, imm8
    VDPPS_VdqHdqWdqIb_V128,
    // [VEX.256.66.0F3A.WIG 40 /r ib] VDPPS ymm1, ymm2, ymm3/m256, imm8
    VDPPS_VqqHqqWqqIb_V256,

    // [NP 0F 77] EMMS
    EMMS,

    // [F3 0F 1E FB] ENDBR32
    ENDBR32,

    // [F3 0F 1E FA] ENDBR64
    ENDBR64,

    // [C8 iw ib] ENTER imm16, imm8
    ENTER_IwIb,

    // [66 0F 3A 17 /r ib] EXTRACTPS r/m32, xmm1, imm8
    EXTRACTPS_EdVdqIb,
    // [VEX.128.66.0F3A.WIG 17 /r ib] VEXTRACTPS r/m32, xmm1, imm8
    VEXTRACTPS_EdVdqIb_V128,
    // [EVEX.128.66.0F3A.WIG 17 /r ib] VEXTRACTPS r/m32, xmm1, imm8
    VEXTRACTPS_EdVdqIb_E128,

    // [D9 F0] F2XM1
    F2XM1,

    // [D9 E1] FABS
    FABS,

    // [D8 /0] FADD m32fp
    FADD_Md,
    // [DC /0] FADD m64fp
    FADD_Mq,
    // [D8 C0+i] FADD ST(0), ST(i)
    FADD_ST0STi,
    // [DC C0+i] FADD ST(i), ST(0)
    FADD_STiST0,
    // [DE C0+i] FADDP ST(i), ST(0)
    // [DE C1] FADDP <ST(1)>, <ST(0)>
    FADDP_STiST0,
    // [DA /0] FIADD m32int
    FIADD_Md,
    // [DE /0] FIADD m16int
    FIADD_Mw,

    // [DF /4] FBLD m80bcd
    FBLD_Mt,

    // [DF /6] FBSTP m80bcd
    FBSTP_Mt,

    // [D9 E0] FCHS
    FCHS,

    // [9B DB E2] FCLEX
    FCLEX,
    // [DB E2] FNCLEX
    FNCLEX,

    // [DA C0+i] FCMOVB ST(0), ST(i)
    FCMOVB_ST0STi,
    // [DA C8+i] FCMOVE ST(0), ST(i)
    FCMOVE_ST0STi,
    // [DA D0+i] FCMOVBE ST(0), ST(i)
    FCMOVBE_ST0STi,
    // [DA D8+i] FCMOVU ST(0), ST(i)
    FCMOVU_ST0STi,
    // [DB C0+i] FCMOVNB ST(0), ST(i)
    FCMOVNB_ST0STi,
    // [DB C8+i] FCMOVNE ST(0), ST(i)
    FCMOVNE_ST0STi,
    // [DB D0+i] FCMOVNBE ST(0), ST(i)
    FCMOVNBE_ST0STi,
    // [DB D8+i] FCMOVNU ST(0), ST(i)
    FCMOVNU_ST0STi,

    // [D8 /2] FCOM m32fp
    FCOM_Md,
    // [DC /2] FCOM m64fp
    FCOM_Mq,
    // [D8 D0+i] FCOM ST(i)
    // [D8 D1] FCOM <ST(1)>
    FCOM_STi,
    // [D8 /3] FCOMP m32fp
    FCOMP_Md,
    // [DC /3] FCOMP m64fp
    FCOMP_Mq,
    // [D8 D8+i] FCOMP ST(i)
    // [D8 D9] FCOMP <ST(1)>
    FCOMP_STi,
    // [DE D9] FCOMPP
    FCOMPP,

    // [DB F0+i] FCOMI ST(0), ST(i)
    FCOMI_ST0STi,
    // [DF F0+i] FCOMIP ST(0), ST(i)
    FCOMIP_ST0STi,
    // [DB E8+i] FUCOMI ST(0), ST(i)
    FUCOMI_ST0STi,
    // [DF E8+i] FUCOMIP ST(0), ST(i)
    FUCOMIP_ST0STi,

    // [D9 FF] FCOS
    FCOS,

    // [D9 F6] FDECSTP
    FDECSTP,

    // [D8 /6] FDIV m32fp
    FDIV_Md,
    // [DC /6] FDIV m64fp
    FDIV_Mq,
    // [D8 F0+i] FDIV ST(0), ST(i)
    FDIV_ST0STi,
    // [DC F8+i] FDIV ST(i), ST(0)
    FDIV_STiST0,
    // [DE F8+i] FDIVP ST(i), ST(0)
    // [DE F9] FDIVP <ST(1)>, <ST(0)>
    FDIVP_STiST0,
    // [DA /6] FIDIV m32int
    FIDIV_Md,
    // [DE /6] FIDIV m16int
    FIDIV_Mw,

    // [D8 /7] FDIVR m32fp
    FDIVR_Md,
    // [DC /7] FDIVR m64fp
    FDIVR_Mq,
    // [D8 F8+i] FDIVR ST(0), ST(i)
    FDIVR_ST0STi,
    // [DC F0+i] FDIVR ST(i), ST(0)
    FDIVR_STiST0,
    // [DE F0+i] FDIVPR ST(i), ST(0)
    // [DE F1] FDIVPR <ST(1)>, <ST(0)>
    FDIVPR_STiST0,
    // [DA /7] FIDIVR m32int
    FIDIVR_Md,
    // [DE /7] FIDIVR m16int
    FIDIVR_Mw,

    // [DD C0+i] FFREE ST(i)
    FFREE_STi,

    // [DE /2] FICOM m16int
    FICOM_Mw,
    // [DA /2] FICOM m32int
    FICOM_Md,
    // [DE /3] FICOMP m16int
    FICOMP_Mw,
    // [DA /3] FICOMP m32int
    FICOMP_Md,

    // [DF /0] FILD m16int
    FILD_Mw,
    // [DB /0] FILD m32int
    FILD_Md,
    // [DF /5] FILD m64int
    FILD_Mq,

    // [D9 F7] FINCSTP
    FINCSTP,

    // [9B DB E3] FINIT
    FINIT,
    // [DB E3] FNINIT
    FNINIT,

    // [DF /2] FIST m16int
    FIST_Mw,
    // [DB /2] FIST m32int
    FIST_Md,
    // [DF /3] FISTP m16int
    FISTP_Mw,
    // [DB /3] FISTP m32int
    FISTP_Md,
    // [DF /7] FISTP m64int
    FISTP_Mq,

    // [DF /1] FISTTP m16int
    FISTTP_Mw,
    // [DB /1] FISTTP m32int
    FISTTP_Md,
    // [DD /1] FISTTP m64int
    FISTTP_Mq,

    // [D9 /0] FLD m32fp
    FLD_Md,
    // [DD /0] FLD m64fp
    FLD_Mq,
    // [DB /5] FLD m80fp
    FLD_Mt,
    // [D9 C0+i] FLD ST(i)
    FLD_STi,

    // [D9 E8] FLD1
    FLD1,
    // [D9 E9] FLDL2T
    FLDL2T,
    // [D9 EA] FLDL2E
    FLDL2E,
    // [D9 EB] FLDPI
    FLDPI,
    // [D9 EC] FLDLG2
    FLDLG2,
    // [D9 ED] FLDLN2
    FLDLN2,
    // [D9 EE] FLDZ
    FLDZ,

    // [D9 /5] FLDCW m2byte
    FLDCW_Mw,

    // [D9 /4] FLDENV m14/24byte
    FLDENV_M,

    // [D8 /1] FMUL m32fp
    FMUL_Md,
    // [DC /1] FMUL m64fp
    FMUL_Mq,
    // [D8 C8+i] FMUL ST(0), ST(i)
    FMUL_ST0STi,
    // [DC C8+i] FMUL ST(i), ST(0)
    FMUL_STiST0,
    // [DE C8+i] FMULP ST(i), ST(0)
    // [DE C9] FMULP <ST(1)>, <ST(0)>
    FMULP_STiST0,
    // [DA /1] FIMUL m32int
    FIMUL_Md,
    // [DE /1] FIMUL m16int
    FIMUL_Mw,

    // [D9 D0] FNOP
    FNOP,

    // [D9 F3] FPATAN
    FPATAN,

    // [D9 F8] FPREM
    FPREM,

    // [D9 F5] FPREM1
    FPREM1,

    // [D9 F2] FPTAN
    FPTAN,

    // [D9 FC] FRNDINT
    FRNDINT,

    // [DD /4] FRSTOR m94/108byte
    FRSTOR_M,

    // [9B DD /6] FSAVE m94/108byte
    FSAVE_M,
    // [DD /6] FNSAVE m94/108byte
    FNSAVE_M,

    // [D9 FD] FSCALE
    FSCALE,

    // [D9 FE] FSIN
    FSIN,

    // [D9 FB] FSINCOS
    FSINCOS,

    // [D9 FA] FSQRT
    FSQRT,

    // [D9 /2] FST m32fp
    FST_Md,
    // [DD /2] FST m64fp
    FST_Mq,
    // [DD D0+i] FST ST(i)
    FST_STi,
    // [D9 /3] FSTP m32fp
    FSTP_Md,
    // [DD /3] FSTP m64fp
    FSTP_Mq,
    // [DB /7] FSTP m80fp
    FSTP_Mt,
    // [DD D8+i] FSTP ST(i)
    FSTP_STi,

    // [9B D9 /7] FSTCW m2byte
    FSTCW_Mw,
    // [D9 /7] FNSTCW m2byte
    FNSTCW_Mw,

    // [9B D9 /6] FSTENV m14/28byte
    FSTENV_M,
    // [D9 /6] FNSTENV m14/28byte
    FNSTENV_M,

    // [9B DD /7] FSTSW m2byte
    FSTSW_Mw,
    // [9B DF E0] FSTSW AX
    FSTSW_AX,
    // [DD /7] FNSTSW m2byte
    FNSTSW_Mw,
    // [DF E0] FNSTSW AX
    FNSTSW_AX,

    // [D8 /4] FSUB m32fp
    FSUB_Md,
    // [DC /4] FSUB m64fp
    FSUB_Mq,
    // [D8 E0+i] FSUB ST(0), ST(i)
    FSUB_ST0STi,
    // [DC E8+i] FSUB ST(i), ST(0)
    FSUB_STiST0,
    // [DE E8+i] FSUBP ST(i), ST(0)
    // [DE E9] FSUBP <ST(1)>, <ST(0)>
    FSUBP_STiST0,
    // [DA /4] FISUB m32int
    FISUB_Md,
    // [DE /4] FISUB m16int
    FISUB_Mw,

    // [D8 /5] FSUBR m32fp
    FSUBR_Md,
    // [DC /5] FSUBR m64fp
    FSUBR_Mq,
    // [D8 E8+i] FSUBR ST(0), ST(i)
    FSUBR_ST0STi,
    // [DC E0+i] FSUBR ST(i), ST(0)
    FSUBR_STiST0,
    // [DE E0+i] FSUBRP ST(i), ST(0)
    // [DE E1] FSUBRP <ST(1)>, <ST(0)>
    FSUBRP_STiST0,
    // [DA /5] FISUBR m32int
    FISUBR_Md,
    // [DE /5] FISUBR m16int
    FISUBR_Mw,

    // [D9 E4] FTST
    FTST,

    // [DD E0+i] FUCOM ST(i)
    // [DD E1] FUCOM <ST(1)>
    FUCOM_STi,
    // [DD E8+i] FUCOMP ST(i)
    // [DD E9] FUCOMP <ST(1)>
    FUCOMP_STi,
    // [DA E9] FUCOMPP
    FUCOMPP,

    // [D9 E5] FXAM
    FXAM,

    // [D9 C8+i] FXCH ST(i)
    // [D9 C9] FXCH <ST(1)>
    FXCH_STi,

    // [NP 0F AE /1] FXRSTOR m512byte
    FXRSTOR_M,
    // [NP REX.W 0F AE /1] FXRSTOR64 m512byte
    FXRSTOR64_M,

    // [NP 0F AE /0] FXSAVE m512byte
    FXSAVE_M,
    // [NP REX.W 0F AE /0] FXSAVE64 m512byte
    FXSAVE64_M,

    // [D9 F4] FXTRACT
    FXTRACT,

    // [D9 F1] FYL2X
    FYL2X,

    // [D9 F9] FYL2XP1
    FYL2XP1,

    // [66 0F 3A CF /r ib] GF2P8AFFINEINVQB xmm1, xmm2/m128, imm8
    GF2P8AFFINEINVQB_VdqWdqIb,
    // [VEX.128.66.0F3A.W1 CF /r ib] VGF2P8AFFINEINVQB xmm1, xmm2, xmm3/m128, imm8
    VGF2P8AFFINEINVQB_VdqHdqWdqIb_V128,
    // [VEX.256.66.0F3A.W1 CF /r ib] VGF2P8AFFINEINVQB ymm1, ymm2, ymm3/m256, imm8
    VGF2P8AFFINEINVQB_VqqHqqWqqIb_V256,
    // [EVEX.128.66.0F3A.W1 CF /r ib] VGF2P8AFFINEINVQB xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    VGF2P8AFFINEINVQB_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 CF /r ib] VGF2P8AFFINEINVQB ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    VGF2P8AFFINEINVQB_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 CF /r ib] VGF2P8AFFINEINVQB zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    VGF2P8AFFINEINVQB_VdqqHdqqWdqqIb_E512,

    // [66 0F 3A CE /r ib] GF2P8AFFINEQB xmm1, xmm2/m128, imm8
    GF2P8AFFINEQB_VdqWdqIb,
    // [VEX.128.66.0F3A.W1 CE /r ib] VGF2P8AFFINEQB xmm1, xmm2/m128, imm8
    VGF2P8AFFINEQB_VdqHdqWdqIb_V128,
    // [VEX.256.66.0F3A.W1 CE /r ib] VGF2P8AFFINEQB ymm1, ymm2/m256, imm8
    VGF2P8AFFINEQB_VqqHqqWqqIb_V256,
    // [EVEX.128.66.0F3A.W1 CE /r ib] VGF2P8AFFINEQB xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    VGF2P8AFFINEQB_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 CE /r ib] VGF2P8AFFINEQB ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    VGF2P8AFFINEQB_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 CE /r ib] VGF2P8AFFINEQB zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    VGF2P8AFFINEQB_VdqqHdqqWdqqIb_E512,

    // [66 0F 38 CF /r ib] GF2P8MULB xmm1, xmm2/m128
    GF2P8MULB_VdqWdq,
    // [VEX.128.66.0F38.W0 CF /r ib] VGF2P8MULB xmm1, xmm2/m128
    VGF2P8MULB_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W0 CF /r ib] VGF2P8MULB ymm1, ymm2/m256
    VGF2P8MULB_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W0 CF /r ib] VGF2P8MULB xmm1 {k1}{z}, xmm2, xmm3/m128
    VGF2P8MULB_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 CF /r ib] VGF2P8MULB ymm1 {k1}{z}, ymm2, ymm3/m256
    VGF2P8MULB_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 CF /r ib] VGF2P8MULB zmm1 {k1}{z}, zmm2, zmm3/m512
    VGF2P8MULB_VdqqHdqqWdqq_E512,

    // [66 0F 7C /r] HADDPD xmm1, xmm2/m128
    HADDPD_VdqWdq,
    // [VEX.128.66.0F.WIG 7C /r] VHADDPD xmm1, xmm2, xmm3/m128
    VHADDPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 7C /r] VHADDPD ymm1, ymm2, ymm3/m256
    VHADDPD_VqqHqqWqq_V256,

    // [F2 0F 7C /r] HADDPD xmm1, xmm2/m128
    HADDPS_VdqWdq,
    // [VEX.128.F2.0F.WIG 7C /r] VHADDPD xmm1, xmm2, xmm3/m128
    VHADDPS_VdqHdqWdq_V128,
    // [VEX.256.F2.0F.WIG 7C /r] VHADDPD ymm1, ymm2, ymm3/m256
    VHADDPS_VqqHqqWqq_V256,

    // [F4] HLT
    HLT,

    // [66 0F 7D /r] HSUBPD xmm1, xmm2/m128
    HSUBPD_VdqWdq,
    // [VEX.128.66.0F.WIG 7D /r] VHSUBPD xmm1, xmm2, xmm3/m128
    VHSUBPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 7D /r] VHSUBPD ymm1, ymm2, ymm3/m256
    VHSUBPD_VqqHqqWqq_V256,

    // [F2 0F 7D /r] HSUBPS xmm1, xmm2/m128
    HSUBPS_VdqWdq,
    // [VEX.128.F2.0F.WIG 7D /r] VHSUBPS xmm1, xmm2, xmm3/m128
    VHSUBPS_VdqHdqWdq_V128,
    // [VEX.256.F2.0F.WIG 7D /r] VHSUBPS ymm1, ymm2, ymm3/m256
    VHSUBPS_VqqHqqWqq_V256,

    // [F6 /7] IDIV r/m8
    // [REX F6 /7] IDIV r/m8
    IDIV_Eb,
    // [F7 /7] IDIV r/m16
    IDIV_Ew,
    // [F7 /7] IDIV r/m32
    IDIV_Ed,
    // [REX.W F7 /7] IDIV r/m64
    IDIV_Eq,

    // [F6 /5] IMUL r/m8
    IMUL_Eb,
    // [F7 /5] IMUL r/m16
    IMUL_Ew,
    // [F7 /5] IMUL r/m32
    IMUL_Ed,
    // [REX.W F7 /5] IMUL r/m64
    IMUL_Eq,
    // [0F AF /r] IMUL r16, r/m16
    IMUL_GbEb,
    // [0F AF /r] IMUL r32, r/m32
    IMUL_GwEw,
    // [REX.W 0F AF /r] IMUL r64, r/m64
    IMUL_GqEq,
    // [6B /r ib] IMUL r16, r/m16, imm8
    IMUL_GwEwIb,
    // [6B /r ib] IMUL r32, r/m32, imm8
    IMUL_GdEdIb,
    // [REX.W 6B /r ib] IMUL r64, r/m64, imm8
    IMUL_GqEqIb,
    // [69 /r iw] IMUL r16, r/m16, imm16
    IMUL_GwEwIw,
    // [69 /r id] IMUL r32, r/m32, imm32
    IMUL_GdEdId,
    // [REX.W 69 /r id] IMUL r64, r/m64, imm32
    IMUL_GqEqId,

    // [E4 ib] IN AL, imm8
    IN_ALIb,
    // [E5 ib] IN AX, imm8
    IN_AXIb,
    // [E5 ib] IN EAX, imm8
    IN_EAXIb,
    // [EC] IN AL, DX
    IN_ALDX,
    // [ED] IN AX, DX
    IN_AXDX,
    // [ED] IN EAX, DX
    IN_EAXDX,

    // [FE /0] INC r/m8
    // [REX FE /0] INC r/m8
    INC_Eb,
    // [FF /0] INC r/m16
    INC_Ew,
    // [FF /0] INC r/m32
    INC_Ed,
    // [REX.W FF /0] INC r/m64
    INC_Eq,
    // [40+rw] INC r16
    INC_Gw,
    // [40+rd] INC r32
    INC_Gd,

    // [F3 0F AE /5] INCSSPD r32
    INCSSPD,
    // [F3 REX.W 0F AE /5] INCSSPQ r64
    INCSSPQ,

    // [6C] INS m8, DX
    // [6C] INSB
    INS_YbDX,
    // [6D] INS m16, DX
    // [6D] INSW
    INS_YwDX,
    // [6D] INS m32, DX
    // [6D] INSD
    INS_YdDX,

    // [66 0F 3A 21 /r ib] INSERTPS xmm1, xmm2/m32, imm8
    INSERTPS_VdqWdIb,
    // [VEX.128.66.0F3A.WIG 21 /r ib] VINSERTPS xmm1, xmm2, xmm3/m32, imm8
    VINSERTPS_VdqHdqWdIb_V128,
    // [EVEX.128.66.0F3A.W0 21 /r ib] VINSERTPS xmm1, xmm2, xmm3/m32, imm8
    VINSERTPS_VdqHdqWdIb_E128,

    // [CC] INT3,
    INT3,
    // [CD ib] INT imm8
    INT_Ib,
    // [CE] INTO
    INTO,
    // [F1] INT1
    INT1,

    // [0F 08] INVD
    INVD,

    // [0F 01 /7] INVLPG mem
    INVLPG_M,

    // [66 0F 38 82 /r] INVPCID r32, m128
    INVPCID_GdMdq,
    // [66 0F 38 82 /r] INVPCID r64, m128
    INVPCID_GqMdq,

    // [CF] IRET
    IRET,
    // [CF] IRETD
    IRETD,
    // [REX.W CF] IRETQ
    IRETQ,

    // [70 cb] JO rel8
    JO_Jb,
    // [0F 80 cw] JO rel16
    JO_Jw,
    // [0F 80 cd] JO rel32
    JO_Jd,
    // [71 cb] JNO rel8
    JNO_Jb,
    // [0F 81 cw] JNO rel16
    JNO_Jw,
    // [0F 81 cd] JNO rel32
    JNO_Jd,
    // [72 cb] JB rel8
    // [72 cb] JC rel8
    // [72 cb] JNAE rel8
    JB_Jb,
    // [0F 82 cw] JB rel16
    // [0F 82 cw] JC rel16
    // [0F 82 cw] JNAE rel16
    JB_Jw,
    // [0F 82 cd] JB rel32
    // [0F 82 cd] JC rel32
    // [0F 82 cd] JNAE rel32
    JB_Jd,
    // [73 cb] JAE rel8
    // [73 cb] JNB rel8
    // [73 cb] JNC rel8
    JAE_Jb,
    // [0F 83 cw] JAE rel16
    // [0F 83 cw] JNB rel16
    // [0F 83 cw] JNC rel16
    JAE_Jw,
    // [0F 83 cd] JAE rel32
    // [0F 83 cd] JNB rel32
    // [0F 83 cd] JNC rel32
    JAE_Jd,
    // [74 cb] JE rel8
    // [74 cb] JZ rel8
    JE_Jb,
    // [0F 84 cw] JE rel16
    // [0F 84 cw] JZ rel16
    JE_Jw,
    // [0F 84 cd] JE rel32
    // [0F 84 cd] JZ rel32
    JE_Jd,
    // [75 cb] JNE rel8
    // [75 cb] JNZ rel8
    JNE_Jb,
    // [0F 85 cw] JNE rel16
    // [0F 85 cw] JNZ rel16
    JNE_Jw,
    // [0F 85 cd] JNE rel32
    // [0F 85 cd] JNZ rel32
    JNE_Jd,
    // [76 cb] JBE rel8
    // [76 cb] JNA rel8
    JBE_Jb,
    // [0F 86 cw] JBE rel16
    // [0F 86 cw] JNA rel16
    JBE_Jw,
    // [0F 86 cd] JBE rel32
    // [0F 86 cd] JNA rel32
    JBE_Jd,
    // [77 cb] JA rel8
    // [77 cb] JNBE rel8
    JA_Jb,
    // [0F 87 cw] JA rel16
    // [0F 87 cw] JNBE rel16
    JA_Jw,
    // [0F 87 cd] JA rel32
    // [0F 87 cd] JNBE rel32
    JA_Jd,
    // [78 cb] JS rel8
    JS_Jb,
    // [0F 88 cw] JS rel16
    JS_Jw,
    // [0F 88 cd] JS rel32
    JS_Jd,
    // [79 cb] JNS rel8
    JNS_Jb,
    // [0F 89 cw] JNS rel16
    JNS_Jw,
    // [0F 89 cd] JNS rel32
    JNS_Jd,
    // [7A cb] JP rel8
    // [7A cb] JPE rel8
    JP_Jb,
    // [0F 8A cw] JP rel16
    // [0F 8A cw] JPE rel16
    JP_Jw,
    // [0F 8A cd] JP rel32
    // [0F 8A cd] JPE rel32
    JP_Jd,
    // [7B cb] JNP rel8
    // [7B cb] JPO rel8
    JNP_Jb,
    // [0F 8B cw] JNP rel16
    // [0F 8B cw] JPO rel16
    JNP_Jw,
    // [0F 8B cd] JNP rel32
    // [0F 8B cd] JPO rel32
    JNP_Jd,
    // [7C cb] JL rel8
    // [7C cb] JNGE rel8
    JL_Jb,
    // [0F 8C cw] JL rel16
    // [0F 8C cw] JNGE rel16
    JL_Jw,
    // [0F 8C cd] JL rel32
    // [0F 8C cd] JNGE rel32
    JL_Jd,
    // [7D cb] JGE rel8
    // [7D cb] JNL rel8
    JGE_Jb,
    // [0F 8D cw] JGE rel16
    // [0F 8D cw] JNL rel16
    JGE_Jw,
    // [0F 8D cd] JGE rel32
    // [0F 8D cd] JNL rel32
    JGE_Jd,
    // [7E cb] JLE rel8
    // [7E cb] JNG rel8
    JLE_Jb,
    // [0F 8E cw] JLE rel16
    // [0F 8E cw] JNG rel16
    JLE_Jw,
    // [0F 8E cd] JLE rel32
    // [0F 8E cd] JNG rel32
    JLE_Jd,
    // [7F cb] JG rel8
    // [7F cb] JNLE rel8
    JG_Jb,
    // [0F 8F cw] JG rel16
    // [0F 8F cw] JNLE rel16
    JG_Jw,
    // [0F 8F cd] JG rel32
    // [0F 8F cd] JNLE rel32
    JG_Jd,
    // [E3 cb] JCXZ rel8
    // [E3 cb] JECXZ rel8
    // [E3 cb] JRCXZ rel8
    JCXZ_Jb,

    // [EB cb] JMP rel8
    JMP_Jb,
    // [E9 cw] JMP rel16
    JMP_Jw,
    // [E9 cd] JMP rel32
    JMP_Jd,
    // [FF /4] JMP r/m16
    JMP_Ew,
    // [FF /4] JMP r/m32
    JMP_Ed,
    // [FF /4] JMP r/m64
    JMP_Eq,
    // [EA cd] JMP ptr16:16
    JMP_Ap_Op16,
    // [EA cp] JMP ptr16:32
    JMP_Ap_Op32,
    // [FF /5] JMP m16:16
    JMP_Ep_Op16,
    // [FF /5] JMP m16:32
    JMP_Ep_Op32,
    // [FF /5] JMP m16:64
    JMP_Ep_Op64,

    // [VEX.L1.66.0F.W0 4A /r] KADDB k1, k2, k3
    KADDB_KGbKHbKEb,
    // [VEX.L1.0F.W0 4A /r] KADDW k1, k2, k3
    KADDW_KGwKHwKEw,
    // [VEX.L1.66.0F.W1 4A /r] KADDD k1, k2, k3
    KADDD_KGdKHdKEd,
    // [VEX.L1.0F.W1 4A /r] KADDQ k1, k2, k3
    KADDQ_KGqKHqKEq,

    // [VEX.L1.66.0F.W0 41 /r] KANDB k1, k2, k3
    KANDB_KGbKHbKEb,
    // [VEX.L1.0F.W0 41 /r] KANDW k1, k2, k3
    KANDW_KGwKHwKEw,
    // [VEX.L1.66.0F.W1 41 /r] KANDD k1, k2, k3
    KANDD_KGdKHdKEd,
    // [VEX.L1.0F.W1 41 /r] KANDQ k1, k2, k3
    KANDQ_KGqKHqKEq,

    // [VEX.L1.66.0F.W0 42 /r] KANDNB k1, k2, k2
    KANDNB_KGbKHbKEb,
    // [VEX.L1.0F.W0 42 /r] KANDNW k1, k2, k3
    KANDNE_KGwKHwKEw,
    // [VEX.L1.66.0F.W1 42 /r] KANDND k1, k2, k3
    KANDND_KGdKHdKEd,
    // [VEX.L1.0F.W1 42 /r] KANDNQ k1, k2, k3
    KANDNQ_KGqKHqKEq,

    // [VEX.L0.66.0F.W0 90 /r] KMOVB k1, k2/m8
    KMOVB_KGbKEb,
    // [VEX.L0.0F.W0 90 /r] KMOVW k1, k2/m16
    KMOVW_KGwKEw,
    // [VEX.L0.66.0F.W1 90 /r] KMOVD k1, k2/m32
    KMOVD_KGdKEd,
    // [VEX.L0.0F.W1 90 /r] KMOVQ k1, k2/m64
    KMOVQ_KGqKEq,
    // [VEX.L0.66.0F.W0 91 /r] KMOVB m8, k1
    KMOVB_KEbKGb,
    // [VEX.L0.0F.W0 91 /r] KMOVW m16, k1
    KMOVW_KEwKGw,
    // [VEX.L0.66.0F.W1 91 /r] KMOVD m32, k1
    KMOVD_KEdKGd,
    // [VEX.L0.0F.W1 91 /r] KMOVQ m64, k1
    KMOVQ_KEqKGq,
    // [VEX.L0.66.0F.W0 92 /r] KMOVB k1, r32
    KMOVB_KEbGd,
    // [VEX.L0.0F.W0 92 /r] KMOVW k1, r32
    KMOVW_KEwGd,
    // [VEX.L0.66.0F.W1 92 /r] KMOVD k1, r32
    KMOVD_KEdGd,
    // [VEX.L0.0F.W1 92 /r] KMOVQ k1, r64
    KMOVQ_KEqGq,
    // [VEX.L0.66.0F.W0 93 /r] KMOVB r32, k1
    KMOVB_GdKEb,
    // [VEX.L0.0F.W0 93 /r] KMOVW r32, k1
    KMOVW_GdKEw,
    // [VEX.L0.66.0F.W1 93 /r] KMOVD r32, k1
    KMOVD_GdKEd,
    // [VEX.L0.0F.W1 93 /r] KMOVQ r64, k1
    KMOVQ_GqKEq,

    // [VEX.L1.66.0F.W0 44 /r] KNOTB k1, k2
    KNOTB_KGbKEb,
    // [VEX.L1.0F.W0 44 /r] KNOTW k1, k2
    KNOTW_KGwKEw,
    // [VEX.L1.66.0F.W1 44 /r] KNOTD k1, k2
    KNOTD_KGdKEd,
    // [VEX.L1.0F.W1 44 /r] KNOTQ k1, k2
    KNOTQ_KGqKEq,

    // [VEX.L1.66.0F.W0 45 /r] KORB k1, k2, k2
    KORB_KGbKHbKEb,
    // [VEX.L1.0F.W0 45 /r] KORW k1, k2, k3
    KORW_KGwKHwKEw,
    // [VEX.L1.66.0F.W1 45 /r] KORD k1, k2, k3
    KORD_KGdKHdKEd,
    // [VEX.L1.0F.W1 45 /r] KORQ k1, k2, k3
    KORQ_KGqKHqKEq,

    // [VEX.L1.66.0F.W0 98 /r] KORTESTB k1, k2
    KORTESTB_KGbKEb,
    // [VEX.L1.0F.W0 98 /r] KORTESTW k1, k2
    KORTESTW_KGwKEw,
    // [VEX.L1.66.0F.W1 98 /r] KORTESTD k1, k2
    KORTESTD_KGdKEd,
    // [VEX.L1.0F.W1 98 /r] KORTESTQ k1, k2
    KORTESTQ_KGqKEq,

    // [VEX.L0.66.0F3A.W0 32 /r] KSHIFTLB k1, k2, imm8
    KSHIFTLB_KGbKEbIb,
    // [VEX.L0.66.0F3A.W1 32 /r] KSHIFTLW k1, k2, imm8
    KSHIFTLW_KGwKEwIb,
    // [VEX.L0.66.0F3A.W0 33 /r] KSHIFTLD k1, k2, imm8
    KSHIFTLD_KGdKEdIb,
    // [VEX.L0.66.0F3A.W1 33 /r] KSHIFTLQ k1, k2, imm8
    KSHIFTLQ_KGqKEqIb,

    // [VEX.L0.66.0F3A.W0 30 /r] KSHIFTRB k1, k2, imm8
    KSHIFTRB_KGbKEbIb,
    // [VEX.L0.66.0F3A.W1 30 /r] KSHIFTRW k1, k2, imm8
    KSHIFTRW_KGwKEwIb,
    // [VEX.L0.66.0F3A.W0 31 /r] KSHIFTRD k1, k2, imm8
    KSHIFTRD_KGdKEdIb,
    // [VEX.L0.66.0F3A.W1 31 /r] KSHIFTRQ k1, k2, imm8
    KSHIFTRQ_KGqKEqIb,

    // [VEX.L0.66.0F.W0 99 /r] KTESTB k1, k2
    KTESTB_KGbKEb,
    // [VEX.L0.0F.W0 99 /r] KTESTW k1, k2
    KTESTW_KGwKEw,
    // [VEX.L0.66.0F.W1 99 /r] KTESTD k1, k2
    KTESTD_KGdKEd,
    // [VEX.L0.0F.W1 99 /r] KTESTQ k1, k2
    KTESTQ_KGqKEq,

    // [VEX.L1.66.0F.W0 4B /r] KUNPCKBW k1, k2, k3
    KUNPCKBW_KGwKHbKEb,
    // [VEX.L1.0F.W0 4B /r] KUNPCKWD k1, k2, k3
    KUNPCKBD_KGdKHwKEw,
    // [VEX.L1.0F.W1 4B /r] KUNPCKDQ k1, k2, k3
    KUNPCKBQ_KGqKHdKEd,

    // [VEX.L1.66.0F.W0 46 /r] KXNORB k1, k2, k3
    KXNORB_KGbKHbKEb,
    // [VEX.L1.0F.W0 46 /r] KXNORW k1, k2, k3
    KXNORW_KGwKHwKEw,
    // [VEX.L1.66.0F.W1 46 /r] KXNORD k1, k2, k3
    KXNORD_KGdKHdKEd,
    // [VEX.L1.0F.W1 46 /r] KXNORQ k1, k2, k3
    KXNORQ_KGqKHqKEq,

    // [VEX.L1.66.0F.W0 47 /r] KXORB k1, k2, k3
    KXORB_KGbKHbKEb,
    // [VEX.L1.0F.W0 47 /r] KXORW k1, k2, k3
    KXORW_KGwKHwKEw,
    // [VEX.L1.66.0F.W1 47 /r] KXORD k1, k2, k3
    KXORD_KGdKHdKEd,
    // [VEX.L1.0F.W1 47 /r] KXORQ k1, k2, k3
    KXORQ_KGqKHqKEq,

    // [9F] LAHF
    LAHF,

    // [0F 02 /r] LAR r16, r/m16
    LAR_GwEw,
    // [0F 02 /r] LAR reg, r32/m16
    LAR_GdEw,

    // [F2 0F F0 /r] LDDQU xmm1, m128
    LDDQU_VdqMdq,
    // [VEX.128.F2.0F.WIG F0 /r] VLDDQU xmm1, m128
    VLDDQU_VdqMdq_V128,
    // [VEX.256.F2.0F.WIG F0 /r] VLDDQU ymm1, m256
    VLDDQU_VqqMqq_V256,

    // [NP 0F AE /2] LDMXCSR m32
    LDMXCSR_Md,
    // [VEX.LZ.0F.WIG AE /2] VLDMXCSR m32
    VLDMXCSR_Md,

    // [C5 /r] LDS r16, m16:16
    LDS_GwMp,
    // [C5 /r] LDS r32, m16:32
    LDS_GdMp,
    // [0F B2 /r] LSS r16, m16:16
    LSS_GwMp,
    // [0F B2 /r] LSS r32, m16:32
    LSS_GdMp,
    // [REX 0F B2 /r] LSS r64, m16:64
    LSS_GqMp,
    // [C4 /r] LES r16, m16:16
    LES_GwMp,
    // [C4 /r] LES r32, m16:32
    LES_GdMp,

    // [0F B4 /r] LFS r16, m16:16
    LFS_GwMp,
    // [0F B4 /r] LFS r32, m16:32
    LFS_GdMp,
    // [REX 0F B4 /r] LFS r64, m16:64
    LFS_GqMp,

    // [0F B5 /r] LGS r16, m16:16
    LGS_GwMp,
    // [0F B5 /r] LGS r32, m16:32
    LGS_GdMp,
    // [REX 0F B5 /r] LGS r64, m16:64
    LGS_GqMp,

    // [8D /r] LEA r16, m
    LEA_GwM,
    // [8D /r] LEA r32, m
    LEA_GdM,
    // [REX.W 8D /r] LEA r64, m
    LEA_GqM,

    // [C9] LEAVE
    LEAVE,

    // [NP 0F AE E8] LFENCE
    LFENCE,

    // [0F 01 /2] GDT m16&32
    LGDT_Ms_Op32,
    // [0F 01 /2] LGDT m16&64
    LGDT_Ms_Op64,
    // [0F 01 /3] IDT m16&32
    LIDT_Ms_Op32,
    // [0F 01 /3] LIDT m16&64
    LIDT_Ms_Op64,

    // [0F 00 /2] LLDT r/m16
    LLDT_Ew,

    // [0F 01 /6] LMSW r/m16
    LMSW_Ew,

    // [F0] LOCK
    LOCK,

    // [AC] LODSB
    LODSB,
    // [AD] LODSW
    LODSW,
    // [AD] LODSD
    LODSD,
    // [REX.W AD] LODSQ
    LODSQ,

    // [E2 cb] LOOP rel8
    LOOP_Jb,
    // [E1 cb] LOOPE rel8
    LOOPE_Jb,
    // [E0 cb] LOOPNE rel8
    LOOPNE_Jb,

    // [0F 03 /r] LSL r16, r/m16
    LSL_GwEw,
    // [0F 03 /r] LSL r32, r32/m16
    LSL_GdEw,
    // [REX.W 0F 03 /r] LSL r64, r32/m16
    LSL_GqEw,

    // [0F 00 /3] LTR r/m16
    LTR_Ew,

    // [F3 0F BD /r] LZCNT r16, r/m16
    LZCNT_GwEw,
    // [F3 0F BD /r] LZCNT r32, r/m32
    LZCNT_GdEd,
    // [F3 REX.W 0F BD /r] LZCNT r64, r/m64
    LZCNT_GqEq,

    // [66 0F F7 /r] MASKMOVDQU xmm1, xmm2
    MASKMOVDQU_VdqUdq,
    // [VEX.128.66.0F.WIG F7 /r] VMASKMOVDQU xmm1, xmm2
    VMASKMOVDQU_VdqUdq_V128,

    // [NP 0F F7 /r] MASKMOVQ mm1, mm2
    MASKMOVQ_PqQq,

    // [66 0F 5F /r] MAXPD xmm1, xmm2/m128
    MAXPD_VdqWdq,
    // [VEX.128.66.0F.WIG 5F /r] VMAXPD xmm1, xmm2, xmm3/m128
    VMAXPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 5F /r] VMAXPD ymm1, ymm2, ymm3/m256
    VMAXPD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W1 5F /r] VMAXPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VMAXPD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 5F /r] VMAXPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VMAXPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 5F /r] VMAXPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{sae}
    VMAXPD_VdqqHdqqWdqq_E512,

    // [NP 0F 5F /r] MAXPS xmm1, xmm2/m128
    MAXPS_VdqWdq,
    // [VEX.128.0F.WIG 5F /r] VMAXPS xmm1, xmm2, xmm3/m128
    VMAXPS_VdqHdqWdq_V128,
    // [VEX.256.0F.WIG 5F /r] VMAXPS ymm1, ymm2, ymm3/m256
    VMAXPS_VqqHqqWqq_V256,
    // [EVEX.128.0F.W0 5F /r] VMAXPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VMAXPS_VdqHdqWdq_E128,
    // [EVEX.256.0F.W0 5F /r] VMAXPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VMAXPS_VqqHqqWqq_E256,
    // [EVEX.512.0F.W0 5F /r] VMAXPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{sae}
    VMAXPS_VdqqHdqqWdqq_E512,

    // [F2 0F 5F /r] MAXSD xmm1, xmm2/m64
    MAXSD_VdqWq,
    // [VEX.LIG.F2.0F.WIG 5F /r] VMAXSD xmm1, xmm2, xmm3/m64
    VMAXSD_VdqHdqWq_V,
    // [EVEX.LIG.F2.0F.W1 5F /r] VMAXSD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}
    VMAXSD_VdqHdqWq_E,

    // [F3 0F 5F /r] MAXSS xmm1, xmm2/m32
    MAXSS_VdqWd,
    // [VEX.LIG.F3.0F.WIG 5F /r] VMAXSS xmm1, xmm2, xmm3/m32
    VMAXSS_VdqHdqWd_V,
    // [EVEX.LIG.F3.0F.W1 5F /r] VMAXSS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}
    VMAXSS_VdqHdqWd_E,

    // [NP 0F AE F0] MFENCE
    MFENCE,

    // [66 0F 5D /r] MINPD xmm1, xmm2/m128
    MINPD_VdqWdq,
    // [VEX.128.66.0F.WIG 5D /r] VMINPD xmm1, xmm2, xmm3/m128
    VMINPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 5D /r] VMINPD ymm1, ymm2, ymm3/m256
    VMINPD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W1 5D /r] VMINPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VMINPD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 5D /r] VMINPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VMINPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 5D /r] VMINPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{sae}
    VMINPD_VdqqHdqqWdqq_E512,

    // [NP 0F 5D /r] MINPD xmm1, xmm2/m128
    MINPS_VdqWdq,
    // [VEX.128.0F.WIG 5D /r] VMINPS xmm1, xmm2, xmm3/m128
    VMINPS_VdqHdqWdq_V128,
    // [VEX.256.0F.WIG 5D /r] VMINPS ymm1, ymm2, ymm3/m256
    VMINPS_VqqHqqWqq_V256,
    // [EVEX.128.0F.W1 5D /r] VMINPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VMINPS_VdqHdqWdq_E128,
    // [EVEX.256.0F.W1 5D /r] VMINPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VMINPS_VqqHqqWqq_E256,
    // [EVEX.512.0F.W1 5D /r] VMINPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{sae}
    VMINPS_VdqqHdqqWdqq_E512,

    // [F2 0F 5D /r] MINSD xmm1, xmm2/m64
    MINSD_VdqWq,
    // [VEX.LIG.F2.0F.WIG 5D /r] VMINSD xmm1, xmm2, xmm3/m64
    VMINSD_VdqHdqWq_V,
    // [EVEX.LIG.F2.0F.W1 5D /r] VMINSD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}
    VMINSD_VdqHdqWq_E,

    // [F3 0F 5D /r] MINSS xmm1, xmm2/m32
    MINSS_VdqWd,
    // [VEX.LIG.F3.0F.WIG 5D /r] VMINSS xmm1, xmm2, xmm3/m32
    VMINSS_VdqHdqWd_V,
    // [EVEX.LIG.F3.0F.W0 5D /r] VMINSS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}
    VMINSS_VdqHdqWd_E,

    // [0F 01 C8] MONITOR
    MONITOR,

    // [88 /r] MOV r/m8, r8
    // [REX 88 /r] MOV r/m8, r8
    MOV_EbGb,
    // [89 /r] MOV r/m16, r16
    MOV_EwGw,
    // [89 /r] MOV r/m32, r32
    MOV_EdGd,
    // [REX.W 89 /r] MOV r/m64, r64
    MOV_EqGq,
    // [8A /r] MOV r8, r/m8
    // [REX 8A /r] MOV r8, r/m8
    MOV_GbEb,
    // [8B /r] MOV r16, r/m16
    MOV_GwEw,
    // [8B /r] MOV r32, r/m32
    MOV_GdEd,
    // [REX.W 8B /r] MOV r64, r/m64
    MOV_GqEq,
    // [8C /r] MOV r/m16, sreg
    MOV_EwSw,
    // [8C /r] MOV r16/r32/m16, sreg
    MOV_EdSw,
    // [REX.W 8C /r] MOV r64/m16, sreg
    MOV_EqSw,
    // [8E /r] MOV sreg, r/m16
    // [REX.W 8E /r] MOV sreg, r/m64
    MOV_SwEw,
    // [A0 ob] MOV AL, moffs8
    // [REX.W A0 ob] MOV AL, moffs8
    MOV_ALOb,
    // [A1 ow] MOV AX, moffs16
    MOV_AXOw,
    // [A1 od] MOV EAX, moffs32
    MOV_EAXOd,
    // [REX.W A1 oq] MOV RAX, moffs64
    MOV_RAXOq,
    // [A2 ob] MOV moffs8, AL
    // [REX.W A2 ob] MOV moffs8, AL
    MOV_ObAL,
    // [A3 ow] MOV moffs16, AX
    MOV_OwAX,
    // [A3 od] MOV moffs32, EAX
    MOV_OdEAX,
    // [REX.W A3 oq] MOV moffs64, RAX
    MOV_OqRAX,
    // [B0+rb ib] MOV r8, imm8
    // [REX B0+rb ib] MOV r8, imm8
    MOV_GbIb,
    // [B8+rw iw] MOV r16, imm16
    MOV_GwIb,
    // [B8+rd id] MOV r32, imm32
    MOV_GdIb,
    // [REX.W B8+rd io] MOV r64, imm64
    MOV_GqIb,
    // [C6 /0 ib] MOV r/m8, imm8
    // [REX C6 /0 ib] MOV r/m8, imm8
    MOV_EbIb,
    // [C7 /0 iw] MOV r/m16, imm16
    MOV_EwIw,
    // [C7 /0 id] MOV r/m32, imm32
    MOV_EdId,
    // [REX.W C7 /0 id] MOV r/m64, imm32
    MOV_EqId,

    // [0F 20 /r] MOV r32, CR0-7
    MOV_RdCd,
    // [0F 20 /r] MOV r64, CR0-7
    MOV_RqCq,
    // [REX.R 0F 20 /0] MOV r64, CR8
    MOV_RqCR8,
    // [0F 22 /r] MOV CR0-7, r32
    MOV_CdRd,
    // [0F 22 /r] MOV CR0-7, r64
    MOV_CqRq,
    // [REX.R 0F 22 /r] MOV CR8, r64
    MOV_CR8Rq,

    // [0F 21 /r] MOV r32, DR0-7
    MOV_RdDd,
    // [0F 21 /r] MOV r64, DR0-7
    MOV_RqDq,
    // [0F 23 /r] MOV DR0-7, r32
    MOV_DdRd,
    // [0F 23 /r] MOV DR0-7, r64
    MOV_DqRq,

    // [66 0F 28 /r] MOVAPD xmm1, xmm2/m128
    MOVAPD_VdqWdq,
    // [66 0F 29 /r] MOVAPD xmm1/m128, xmm2
    MOVAPD_WdqVdq,
    // [VEX.128.66.0F.WIG 28 /r] VMOVAPD xmm1, xmm2/m128
    VMOVAPD_VdqWdq_V128,
    // [VEX.128.66.0F.WIG 29 /r] VMOVAPD xmm1/m128, xmm2
    VMOVAPD_WdqVdq_V128,
    // [VEX.256.66.0F.WIG 28 /r] VMOVAPD ymm1, ymm2/m256
    VMOVAPD_VqqWqq_V256,
    // [VEX.256.66.0F.WIG 29 /r] VMOVAPD ymm1/m256, ymm2
    VMOVAPD_WqqVqq_V256,
    // [EVEX.128.66.0F.W1 28 /r] VMOVAPD xmm1 {k1}{z}, xmm2/m128
    VMOVAPD_VdqWdq_E128,
    // [EVEX.256.66.0F.W1 28 /r] VMOVAPD ymm1 {k1}{z}, ymm2/m256
    VMOVAPD_VqqWqq_E256,
    // [EVEX.512.66.0F.W1 28 /r] VMOVAPD zmm1 {k1}{z}, zmm2/m512
    VMOVAPD_VdqqWdqq_E512,
    // [EVEX.128.66.0F.W1 28 /r] VMOVAPD xmm1/m128 {k1}{z}, xmm2
    VMOVAPD_WdqVdq_E128,
    // [EVEX.256.66.0F.W1 28 /r] VMOVAPD ymm1/m256 {k1}{z}, ymm2
    VMOVAPD_WqqVqq_E256,
    // [EVEX.512.66.0F.W1 28 /r] VMOVAPD zmm1/m512 {k1}{z}, zmm2
    VMOVAPD_WdqqVdqq_E512,

    // [NP 0F 28 /r] MOVAPD xmm1, xmm2/m128
    MOVAPS_VdqWdq,
    // [NP 0F 29 /r] MOVAPD xmm1/m128, xmm2
    MOVAPS_WdqVdq,
    // [VEX.128.0F.WIG 28 /r] VMOVAPS xmm1, xmm2/m128
    VMOVAPS_VdqWdq_V128,
    // [VEX.128.0F.WIG 29 /r] VMOVAPS xmm1/m128, xmm2
    VMOVAPS_WdqVdq_V128,
    // [VEX.256.0F.WIG 28 /r] VMOVAPS ymm1, ymm2/m256
    VMOVAPS_VqqWqq_V256,
    // [VEX.256.0F.WIG 29 /r] VMOVAPS ymm1/m256, ymm2
    VMOVAPS_WqqVqq_V256,
    // [EVEX.128.0F.W1 28 /r] VMOVAPS xmm1 {k1}{z}, xmm2/m128
    VMOVAPS_VdqWdq_E128,
    // [EVEX.256.0F.W1 28 /r] VMOVAPS ymm1 {k1}{z}, ymm2/m256
    VMOVAPS_VqqWqq_E256,
    // [EVEX.512.0F.W1 28 /r] VMOVAPS zmm1 {k1}{z}, zmm2/m512
    VMOVAPS_VdqqWdqq_E512,
    // [EVEX.128.0F.W1 28 /r] VMOVAPS xmm1/m128 {k1}{z}, xmm2
    VMOVAPS_WdqVdq_E128,
    // [EVEX.256.0F.W1 28 /r] VMOVAPS ymm1/m256 {k1}{z}, ymm2
    VMOVAPS_WqqVqq_E256,
    // [EVEX.512.0F.W1 28 /r] VMOVAPS zmm1/m512 {k1}{z}, zmm2
    VMOVAPS_WdqqVdqq_E512,

    // [0F 38 F0 /r] MOVBE r16, m16
    MOVBE_GwMw,
    // [0F 38 F0 /r] MOVBE r32, m32
    MOVBE_GdMd,
    // [REX.W 0F 38 F0 /r] MOVBE r64, m64
    MOVBE_GqMq,
    // [0F 38 F1 /r] MOVBE m16, r16
    MOVBE_MwGw,
    // [0F 38 F1 /r] MOVBE m32, r32
    MOVBE_MdGd,
    // [REX.W 0F 38 F1 /r] MOVBE m64, r64
    MOVBE_MqGq,

    // [NP 0F 6E /r] MOVD mm, r/m32
    MOVD_PqEd,
    // [NP REX.W 0F 6E /r] MOVQ mm, r/m64
    MOVQ_PqEq,
    // [NP 0F 7E /r] MOVD r/m32, mm
    MOVD_EdPq,
    // [NP REX.W 0F 7E /r] MOVQ r/m64, mm
    MOVQ_EqPq,
    // [66 0F 6E /r] MOVD xmm1, r/m32
    MOVD_VdqEd,
    // [66 REX.W 0F 6E /r] MOVQ xmm1, r/m64
    MOVQ_VdqEq,
    // [66 0F 7E /r] MOVD r/m32, xmm1
    MOVD_EdVdq,
    // [66 REX.W 0F 7E /r] MOVQ r/m64, xmm1
    MOVQ_VdqEd,
    // [VEX.128.66.0F.W0 6E /r] VMOVD xmm1, r/m32
    VMOVD_VdqEd_V128,
    // [EVEX.128.66.0F.W0 6E /r] VMOVD xmm1, r/m32
    VMOVD_VdqEd_E128,
    // [VEX.128.66.0F.W1 6E /r] VMOVQ xmm1, r/m64
    VMOVQ_VdqEq_V128,
    // [EVEX.128.66.0F.Wq 6E /r] VMOVD xmm1, r/m64
    VMOVQ_VdqEq_E128,
    // [VEX.128.66.0F.W0 7E /r] VMOVD r/m32, xmm1
    VMOVD_EdVdq_V128,
    // [EVEX.128.66.0F.W0 7E /r] VMOVD r/m32, xmm1
    VMOVD_EdVdq_E128,
    // [VEX.128.66.0F.W1 7E /r] VMOVQ r/m64, xmm1
    VMOVQ_VdqEd_V128,
    // [EVEX.128.66.0F.W1 7E /r] VMOVD r/m64, xmm1
    VMOVQ_VdqEd_E128,

    // [F2 0F 12 /r] MOVDDUP xmm1, xmm2/m64
    MOVDDUP_VdqWq,
    // [VEX.128.F2.0F.WIG 12 /r] VMOVDDUP xmm1, xmm2/m64
    // NOTE: Intel manual says m64; TODO: Is this correct?
    VMOVDDUP_VdqWq_V128,
    // [VEX.256.F2.0F.WIG 12 /r] VMOVDDUP ymm1, ymm2/m256
    VMOVDDUP_VqqWqq_V256,
    // [EVEX.128.F2.0F.W1 12 /r] VMOVDDUP xmm1 {k1}{z}, xmm2/m64
    // NOTE: Intel manual says m64; TODO: Is this correct
    VMOVDDUP_VdqWq_E128,
    // [EVEX.256.F2.0F.W1 12 /r] VMOVDDUP ymm1 {k1}{z}, ymm2/m256
    VMOVDDUP_VqqWqq_E256,
    // [EVEX.512.F2.0F.W1 12 /r] VMOVDDUP zmm1 {k1}{z}, zmm2/m512
    VMOVDDUP_VdqqWdqq_E512,

    // [NP 0F 38 F9 /r] MOVDIRI m32, r32
    MOVDIRI_MdGd,
    // [NP REX.W 0F 38 F9 /r] MOVDIRI m64, r64
    MOVDIRI_MqGq,

    // [66 0F 38 F8 /r] MOVDIR64B r16, m512
    MOVDIR64B_GwM,
    // [66 0F 38 F8 /r] MOVDIR64B r32, m512
    MOVDIR64B_GdM,
    // [66 0F 38 F8 /r] MOVDIR64B r64, m512
    MOVDIR64B_GqM,

    // [66 0F 6F /r] MOVDQA xmm1, xmm2/m128
    MOVDQA_VdqWdq,
    // [66 0F 7F /r] MOVDQA xmm1/m128, xmm2
    MOVDQA_WdqVdq,
    // [VEX.128.66.0F.WIG 6F /r] VMOVDQA xmm1, xmm2/m128
    VMOVDQA_VdqWdq_V128,
    // [VEX.128.66.0F.WIG 7F /r] VMOVDQA xmm1/m128, xmm2
    VMOVDQA_WdqVdq_V128,
    // [VEX.256.66.0F.WIG 6F /r] VMOVDQA ymm1, ymm2/m256
    VMOVDQA_VqqWqq_V256,
    // [VEX.256.66.0F.WIG 7F /r] VMOVDQA ymm1/m256, ymm2
    VMOVDQA_WqqVqq_V256,
    // [EVEX.128.66.0F.W0 6F /r] VMOVDQA32 xmm1 {k1}{z}, xmm2/m128
    VMOVDQA32_VdqWdq_E128,
    // [EVEX.256.66.0F.W0 6F /r] VMOVDQA32 ymm1 {k1}{z}, ymm2/m256
    VMOVDQA32_VqqWqq_E256,
    // [EVEX.512.66.0F.W0 6F /r] VMOVDQA32 zmm1 {k1}{z}, zmm2/m512
    VMOVDQA32_VdqqWdqq_E512,
    // [EVEX.128.66.0F.W0 7F /r] VMOVDQA32 xmm1/m128 {k1}{z}, xmm2
    VMOVDQA32_WdqVdq_E128,
    // [EVEX.256.66.0F.W0 7F /r] VMOVDQA32 ymm1/m256 {k1}{z}, ymm2
    VMOVDQA32_WqqVqq_E256,
    // [EVEX.512.66.0F.W0 7F /r] VMOVDQA32 zmm1/m512 {k1}{z}, zmm2
    VMOVDQA32_WdqqVdqq_E512,
    // [EVEX.128.66.0F.W1 6F /r] VMOVDQA64 xmm1 {k1}{z}, xmm2/m128
    VMOVDQA64_VdqWdq_E128,
    // [EVEX.256.66.0F.W1 6F /r] VMOVDQA64 ymm1 {k1}{z}, ymm2/m256
    VMOVDQA64_VqqWqq_E256,
    // [EVEX.512.66.0F.W1 6F /r] VMOVDQA64 zmm1 {k1}{z}, zmm2/m512
    VMOVDQA64_VdqqWdqq_E512,
    // [EVEX.128.66.0F.W1 7F /r] VMOVDQA64 xmm1/m128 {k1}{z}, xmm2
    VMOVDQA64_WdqVdq_E128,
    // [EVEX.256.66.0F.W1 7F /r] VMOVDQA64 ymm1/m256 {k1}{z}, ymm2
    VMOVDQA64_WqqVqq_E256,
    // [EVEX.512.66.0F.W1 7F /r] VMOVDQA64 zmm1/m512 {k1}{z}, zmm2
    VMOVDQA64_WdqqVdqq_E512,

    // [F3 0F 6F /r] MOVDQU xmm1, xmm2/m128
    MOVDQU_VdqWdq,
    // [F3 0F 7F /r] MOVDQU xmm1/m128, xmm2
    MOVDQU_WdqVdq,
    // [VEX.128.F3.0F.WIG 6F /r] VMOVDQU xmm1, xmm2/m128
    VMOVDQU_VdqWdq_V128,
    // [VEX.128.F3.0F.WIG 7F /r] VMOVDQU xmm1/m128, xmm2
    VMOVDQU_WdqVdq_V128,
    // [VEX.256.F3.0F.WIG 6F /r] VMOVDQU ymm1, ymm2/m256
    VMOVDQU_VqqWqq_V256,
    // [VEX.256.F3.0F.WIG 7F /r] VMOVDQU ymm1/m256, ymm2
    VMOVDQU_WqqVqq_V256,
    // [EVEX.128.F2.0F.W0 6F /r] VMOVDQU8 xmm1 {k1}{z}, xmm2/m128
    VMOVDQU8_VdqWdq_E128,
    // [EVEX.256.F2.0F.W0 6F /r] VMOVDQU8 ymm1 {k1}{z}, ymm2/m256
    VMOVDQU8_VqqWqq_E256,
    // [EVEX.512.F2.0F.W0 6F /r] VMOVDQU8 zmm1 {k1}{z}, zmm2/m512
    VMOVDQU8_VdqqWdqq_E512,
    // [EVEX.128.F2.0F.W0 7F /r] VMOVDQU8 xmm1/m128 {k1}{z}, xmm2
    VMOVDQU8_WdqVdq_E128,
    // [EVEX.256.F2.0F.W0 7F /r] VMOVDQU8 ymm1/m256 {k1}{z}, ymm2
    VMOVDQU8_WqqVqq_E256,
    // [EVEX.512.F2.0F.W0 7F /r] VMOVDQU8 zmm1/m512 {k1}{z}, zmm2
    VMOVDQU8_WdqqVdqq_E512,
    // [EVEX.128.F2.0F.W1 6F /r] VMOVDQU16 xmm1 {k1}{z}, xmm2/m128
    VMOVDQU16_VdqWdq_E128,
    // [EVEX.256.F2.0F.W1 6F /r] VMOVDQU16 ymm1 {k1}{z}, ymm2/m256
    VMOVDQU16_VqqWqq_E256,
    // [EVEX.512.F2.0F.W1 6F /r] VMOVDQU16 zmm1 {k1}{z}, zmm2/m512
    VMOVDQU16_VdqqWdqq_E512,
    // [EVEX.128.F2.0F.W1 7F /r] VMOVDQU16 xmm1/m128 {k1}{z}, xmm2
    VMOVDQU16_WdqVdq_E128,
    // [EVEX.256.F2.0F.W1 7F /r] VMOVDQU16 ymm1/m256 {k1}{z}, ymm2
    VMOVDQU16_WqqVqq_E256,
    // [EVEX.512.F2.0F.W1 7F /r] VMOVDQU16 zmm1/m512 {k1}{z}, zmm2
    VMOVDQU16_WdqqVdqq_E512,
    // [EVEX.128.F3.0F.W0 6F /r] VMOVDQU32 xmm1 {k1}{z}, xmm2/m128
    VMOVDQU32_VdqWdq_E128,
    // [EVEX.256.F3.0F.W0 6F /r] VMOVDQU32 ymm1 {k1}{z}, ymm2/m256
    VMOVDQU32_VqqWqq_E256,
    // [EVEX.512.F3.0F.W0 6F /r] VMOVDQU32 zmm1 {k1}{z}, zmm2/m512
    VMOVDQU32_VdqqWdqq_E512,
    // [EVEX.128.F3.0F.W0 7F /r] VMOVDQU32 xmm1/m128 {k1}{z}, xmm2
    VMOVDQU32_WdqVdq_E128,
    // [EVEX.256.F3.0F.W0 7F /r] VMOVDQU32 ymm1/m256 {k1}{z}, ymm2
    VMOVDQU32_WqqVqq_E256,
    // [EVEX.512.F3.0F.W0 7F /r] VMOVDQU32 zmm1/m512 {k1}{z}, zmm2
    VMOVDQU32_WdqqVdqq_E512,
    // [EVEX.128.F3.0F.W1 6F /r] VMOVDQU64 xmm1 {k1}{z}, xmm2/m128
    VMOVDQU64_VdqWdq_E128,
    // [EVEX.256.F3.0F.W1 6F /r] VMOVDQU64 ymm1 {k1}{z}, ymm2/m256
    VMOVDQU64_VqqWqq_E256,
    // [EVEX.512.F3.0F.W1 6F /r] VMOVDQU64 zmm1 {k1}{z}, zmm2/m512
    VMOVDQU64_VdqqWdqq_E512,
    // [EVEX.128.F3.0F.W1 7F /r] VMOVDQU64 xmm1/m128 {k1}{z}, xmm2
    VMOVDQU64_WdqVdq_E128,
    // [EVEX.256.F3.0F.W1 7F /r] VMOVDQU64 ymm1/m256 {k1}{z}, ymm2
    VMOVDQU64_WqqVqq_E256,
    // [EVEX.512.F3.0F.W1 7F /r] VMOVDQU64 zmm1/m512 {k1}{z}, zmm2
    VMOVDQU64_WdqqVdqq_E512,

    // [F2 0F D6 /r] MOVDQ2Q mm, xmm
    MOVDQ2Q_PqUdq,

    // [NP 0F 12 /r] MOVHLPS xmm1, xmm2
    MOVHLPS_VdqUdq,
    // [VEX.128.0F.WIG 12 /r] VMOVHLPS xmm1, xmm2, xmm3
    VMOVHLPS_VdqHdqUdq_V128,
    // [EVEX.128.0F.W0 12 /r] VMOVHLPS xmm1, xmm2, xmm3
    VMOVHLPS_VdqHdqUdq_E128,

    // [66 0F 16 /r] MOVHPD xmm1, m64
    MOVHPD_VdqMq,
    // [VEX.128.66.0F.WIG 16 /r] VMOVHPD xmm1, xmm2, m64
    VMOVHPD_VdqHdqMq_V128,
    // [EVEX.128.66.0F.W1 16 /r] VMOVHPD xmm1, xmm2, m64
    VMOVHPD_VdqHdqMq_E128,
    // [66 0F 17 /r] MOVHPD m64, xmm1
    MOVHPD_MqVdq,
    // [VEX.128.66.0F.WIG 17 /r] VMOVHPD m64, xmm1
    VMOVHPD_MqVdq_V128,
    // [EVEX.128.66.0F.W1 17 /r] VMOVHPD m64, xmm1
    VMOVHPD_MqVdq_E128,

    // [NP 0F 16 /r] MOVHPS xmm1, m64
    MOVHPS_VdqMq,
    // [VEX.128.0F.WIG 16 /r] VMOVHPS xmm1, xmm2, m64
    VMOVHPS_VdqHdqMq_V128,
    // [EVEX.128.0F.W1 16 /r] VMOVHPS xmm1, xmm2, m64
    VMOVHPS_VdqHdqMq_E128,
    // [NP 0F 17 /r] MOVHPS m64, xmm1
    MOVHPS_MqVdq,
    // [VEX.128.0F.WIG 17 /r] VMOVHPS m64, xmm1
    VMOVHPS_MqVdq_V128,
    // [EVEX.128.0F.W1 17 /r] VMOVHPS m64, xmm1
    VMOVHPS_MqVdq_E128,

    // [NP 0F 16 /r] MOVLHPS xmm1, xmm2
    MOVLHPS_VdqWdq,
    // [VEX.128.0F.WIG 16 /r] VMOVLHPS xmm1, xmm2, xmm3
    VMOVLHPS_VdqHdqWdq_V128,
    // [EVEX.128.0F.W0 16 /r] VMOVLHPS xmm1, xmm2, xmm3
    VMOVLHPS_VdqHdqWdq_E128,

    // [66 0F 12 /r] MOVLPD xmm1, m64
    MOVLPD_VdqMq,
    // [VEX.128.66.0F.WIG 12 /r] VMOVLPD xmm1, xmm2, m64
    VMOVLPD_VdqHdqMq_V128,
    // [EVEX.128.66.0F.W1 12 /r] VMOVLPD xmm1, xmm2, m64
    VMOVLPD_VdqHdqMq_E128,
    // [66 0F 13 /r] MOVLPD m64, xmm1
    MOVLPD_MqVdq,
    // [VEX.128.66.0F.WIG 13 /r] VMOVLPD m64, xmm1
    VMOVLPD_MqVdq_V128,
    // [EVEX.128.66.0F.W1 13 /r] VMOVLPD m64, xmm1
    VMOVLPD_MqVdq_E128,

    // [NP 0F 12 /r] MOVLPS xmm1, m64
    MOVLPS_VdqMq,
    // [VEX.128.66.0F.WIG 12 /r] VMOVLPS xmm1, xmm2, m64
    VMOVLPS_VdqHdqMq_V128,
    // [EVEX.128.66.0F.W1 12 /r] VMOVLPS xmm1, xmm2, m64
    VMOVLPS_VdqHdqMq_E128,
    // [NP 0F 13 /r] MOVLPS m64, xmm1
    // NOTE: Intel manual doesn't have `NP`
    MOVLPS_MqVdq,
    // [VEX.128.0F.WIG 13 /r] VMOVLPS m64, xmm1
    VMOVLPS_MqVdq_V128,
    // [EVEX.128.0F.W1 13 /r] VMOVLPS m64, xmm1
    VMOVLPS_MqVdq_E128,

    // [66 0F 50 /r] MOVMSKPD reg, xmm
    MOVMSKPD_GdUdq,
    // [VEX.128.66.0F.WIG 50 /r] VMOVMSKPD reg, xmm
    VMOVMSKPD_GdUdq_V128,
    // [VEX.256.66.0F.WIG 50 /r] VMOVMSKPD reg, ymm
    VMOVMSKPD_GdUqq_V256,

    // [NP 0F 50 /r] MOVMSKPS reg, xmm
    MOVMSKPS_GdUdq,
    // [VEX.128.0F.WIG 50 /r] VMOVMSKPS reg, xmm
    VMOVMSKPS_GdUdq_V128,
    // [VEX.256.0F.WIG 50 /r] VMOVMSKPS reg, ymm
    VMOVMSKPS_GdUqq_V256,

    // [66 0F 38 2A /r] MOVNTDQA xmm1, m128
    MOVNTDQA_VdqMdq,
    // [VEX.128.66.0F38.WIG 2A /r] VMOVNTDQA xmm1, m128
    VMOVNTDQA_VdqMdq_V128,
    // [VEX.256.66.0F38.WIG 2A /r] VMOVNTDQA ymm1, m256
    VMOVNTDQA_VqqMqq_V256,
    // [EVEX.128.66.0F38.W0 2A /r] VMOVNTDQA xmm1, m128
    VMOVNTDQA_VdqMdq_E128,
    // [EVEX.256.66.0F38.W0 2A /r] VMOVNTDQA ymm1, m256
    VMOVNTDQA_VqqMqq_E256,
    // [EVEX.512.66.0F38.W0 2A /r] VMOVNTDQA zmm1, m512
    VMOVNTDQA_VdqqMdqq_E512,

    // [66 0F E7 /r] MOVNTDQ m128, xmm1
    MOVNTDQ_MdqVdq,
    // [VEX.128.66.0F.WIG E7 /r] VMOVNTDQ m128, xmm1
    VMOVNTDQ_MdqVdq_V128,
    // [VEX.256.66.0F.WIG E7 /r] VMOVNTDQ m256, ymm1
    VMOVNTDQ_MqqVqq_V256,
    // [EVEX.128.66.0F.W0 E7 /r] VMOVNTDQ m128, xmm1
    VMOVNTDQ_MdqVdq_E128,
    // [EVEX.256.66.0F.W0 E7 /r] VMOVNTDQ m256, ymm1
    VMOVNTDQ_MqqVqq_E256,
    // [EVEX.512.66.0F.W0 E7 /r] VMOVNTDQ m512, zmm1
    VMOVNTDQ_MdqqVdqq_E512,

    // [NP 0F C3 /r] MOVNTI m32, r32
    MOVNTI_MdGd,
    // [NP REX.W 0F C3 /r] MOVNTI m64, r64
    MOVNTI_MqGq,

    // [66 0F 2B /r] MOVNTPD m128, xmm1
    MOVNTPD_MdqVdq,
    // [VEX.128.66.0F.WIG 2B /r] VMOVNTPD m128, xmm1
    VMOVNTPD_MdqVdq_V128,
    // [VEX.256.66.0F.WIG 2B /r] VMOVNTPD m256, ymm1
    VMOVNTPD_MqqVqq_V256,
    // [EVEX.128.66.0F.W1 2B /r] VMOVNTPD m128, xmm1
    VMOVNTPD_MdqVdq_E128,
    // [EVEX.256.66.0F.W1 2B /r] VMOVNTPD m256, ymm1
    VMOVNTPD_MqqVqq_E256,
    // [EVEX.512.66.0F.W1 2B /r] VMOVNTPD m512, zmm1
    VMOVNTPD_MdqqVdqq_E512,

    // [NP 0F 2B /r] MOVNTPS m128, xmm1
    MOVNTPS_MdqVdq,
    // [VEX.128.0F.WIG 2B /r] VMOVNTPS m128, xmm1
    VMOVNTPS_MdqVdq_V128,
    // [VEX.256.0F.WIG 2B /r] VMOVNTPS m256, ymm1
    VMOVNTPS_MqqVqq_V256,
    // [EVEX.128.0F.W0 2B /r] VMOVNTPS m128, xmm1
    VMOVNTPS_MdqVdq_E128,
    // [EVEX.256.0F.W0 2B /r] VMOVNTPS m256, ymm1
    VMOVNTPS_MqqVqq_E256,
    // [EVEX.512.0F.W0 2B /r] VMOVNTPS m512, zmm1
    VMOVNTPS_MdqqVdqq_E512,

    // [NP 0F E7 /r] MOVNTQ m64, mm
    MOVNTQ_MqPq,

    // [NP 0F 6F /r] MOVQ mm, mm/m64
    MOVQ_PqQq,
    // [NP 0F 7F /r] MOVQ mm/m64, mm
    MOVQ_QqPq,
    // [F3 0F 7E /r] MOVQ xmm1, xmm2/m64
    MOVQ_VdqWq,
    // [VEX.128.F3.0F.WIG 7E /r] VMOVQ xmm1, xmm2/m64
    VMOVQ_VdqWq_V128,
    // [EVEX.128.F3.0F.W1 7E /r] VMOVQ xmm1, xmm2/m64
    VMOVQ_VdqWq_E128,
    // [66 0F D6 /r] MOVQ xmm1/m64, xmm2
    MOVQ_WqVdq,
    // [VEX.128.66.0F.WIG D6 /r] VMOVQ xmm1/m64, xmm2
    VMOVQ_EqVdq_V128,
    // [EVEX.128.66.0F.W1 D6 /r] VMOVQ xmm1/m64, xmm2
    VMOVQ_EqVdq_E128,

    // [F3 0F D6 /r] MOVQ2DQ xmm1, mm1
    MOVQ2DQ_VdqQq,

    // [A4] MOVS m8, m8
    // [A4] MOVSB
    MOVS_YbXb,
    // [A5] MOVS m16, m16
    // [A5] MOVSW
    MOVS_YwXw,
    // [A5] MOVS m32, m32
    // [A5] MOVSD
    MOVS_YdXd,
    // [REX.W A5] MOVS m64, m64
    // [REX.W A5] MOVSQ
    MOVS_YqXq,

    // [F2 0F 10 /r] MOVSD xmm1, xmm2
    // [F2 0F 10 /r] MOVSD xmm1, m64
    MOVSD_VdqWq,
    // [F2 0F 11 /r] MOVSD xmm1/m64, xmm2
    MOVSD_WqVdq,
    // [VEX.LIG.F2.0F.WIG 10 /r] VMOVSD xmm1, xmm2, xmm3
    // [VEX.LIG.F2.0F.WIG 10 /r] VMOVSD xmm1, m64
    VMOVSD_VdqHdqWdq_V,
    // [VEX.LIG.F2.0F.WIG 11 /r] VMOVSD xmm1, xmm2, xmm3
    // [VEX.LIG.F2.0F.WIG 11 /r] VMOVSD m64, xmm1
    VMOVSD_WdqHdqVdq_V,
    // [EVEX.LIG.F2.0F.W1 10 /r] VMOVSD xmm1 {k1}{z}, xmm2, xmm3
    // [EVEX.LIG.F2.0F.W1 10 /r] VMOVSD xmm1 {k1}{z}, m64
    VMOVSD_VdqHdqWdq_E,
    // [EVEX.LIG.F2.0F.W1 11 /r] VMOVSD xmm1 {k1}{z}, xmm2, xmm3
    // [EVEX.LIG.F2.0F.W1 11 /r] VMOVSD m64 {k1}{z}, xmm1
    VMOVSD_WdqHdqVdq_E,

    // [F3 0F 16 /r] MOVSHDUP xmm1, xmm2/m128
    MOVSHDUP_VdqWdq,
    // [VEX.128.F3.0F.WIG 16 /r] VMOVSHDUP xmm1, xmm2/m128
    VMOVSHDUP_VdqWdq_V128,
    // [VEX.256.F3.0F.WIG 16 /r] VMOVSHDUP ymm1, ymm2/m256
    VMOVSHDUP_VqqWqq_V256,
    // [EVEX.128.F3.0F.W0 16 /r] VMOVSHDUP xmm1 {k1}{z}, xmm2/m128
    VMOVSHDUP_VdqWdq_E128,
    // [EVEX.256.F3.0F.W0 16 /r] VMOVSHDUP ymm1 {k1}{z}, ymm2/m256
    VMOVSHDUP_VqqWqq_E256,
    // [EVEX.512.F3.0F.W0 16 /r] VMOVSHDUP zmm1 {k1}{z}, zmm2/m512
    VMOVSHDUP_VdqqWdqq_E512,

    // [F3 0F 12 /r] MOVSLDUP xmm1, xmm2/m128
    MOVSLDUP_VdqWdq,
    // [VEX.128.F3.0F.WIG 12 /r] VMOVSLDUP xmm1, xmm2/m128
    VMOVSLDUP_VdqWdq_V128,
    // [VEX.256.F3.0F.WIG 12 /r] VMOVSLDUP ymm1, ymm2/m256
    VMOVSLDUP_VqqWqq_V256,
    // [EVEX.128.F3.0F.W0 12 /r] VMOVSLDUP xmm1 {k1}{z}, xmm2/m128
    VMOVSLDUP_VdqWdq_E128,
    // [EVEX.256.F3.0F.W0 12 /r] VMOVSLDUP ymm1 {k1}{z}, ymm2/m256
    VMOVSLDUP_VqqWqq_E256,
    // [EVEX.512.F3.0F.W0 12 /r] VMOVSLDUP zmm1 {k1}{z}, zmm2/m512
    VMOVSLDUP_VdqqWdqq_E512,

    // [F3 0F 10 /r] MOVSS xmm1, xmm2
    // [F3 0F 10 /r] MOVSS xmm1, m32
    MOVSS_VdqWd,
    // [VEX.LIG.F3.0F.WIG 10 /r] VMOVSS xmm1, xmm2, xmm3
    // [VEX.LIG.F3.0F.WIG 10 /r] VMOVSS xmm1, m32
    VMOVSS_VdqHdqWdq_V,
    // [F3 0F 11 /r] MOVSS xmm1/m32, xmm1
    MOVSS_WdVdq,
    // [VEX.LIG.F3.0F.WIG 11 /r] VMOVSS xmm1, xmm2, xmm3
    // [VEX.LIG.F3.0F.WIG 11 /r] VMOVSS m32, xmm1
    VMOVSS_WdqHdqVdq_V,
    // [EVEX.LIG.F3.0F.W0 10 /r] VMOVSS xmm1 {k1}{z}, xmm2, xmm3
    // [EVEX.LIG.F3.0F.W0 10 /r] VMOVSS xmm1 {k1}{z}, m32
    VMOVSS_VdqHdqWdq_E,
    // [EVEX.LIG.F3.0F.W0 11 /r] VMOVSS xmm1 {k1}{z}, xmm2, xmm3
    // [EVEX.LIG.F3.0F.W0 11 /r] VMOVSS m32 {k1}{z}, xmm1
    VMOVSS_WdqHdqVdq_E,

    // [0F BE /r] MOVSX r16, r/m8
    MOVSX_GwEb,
    // [0F BE /r] MOVSX r32, r/m8
    MOVSX_GdEb,
    // [REX.W 0F BE /r] MOVSX r64, r/m8
    MOVSX_GqEb,
    // [0F BF /r] MOVSX r32, r/m16
    MOVSX_GdEw,
    // [REX.W 0F BF /r] MOVSX r64, r/m16
    MOVSX_GqEw,
    // [63 /r] MOVSXD r16, r/m16
    MOVSXD_GwEw,
    // [63 /r] MOVSXD r32, r/m32
    MOVSXD_GdEd,
    // [REX.W 63 /r] MOVSXD r64, r/m32
    MOVSXD_GqEd,

    // [66 0F 10 /r] MOVUPD xmm1, xmm2/m128
    MOVUPD_VdqWdq,
    // [66 0F 11 /r] MOVUPD xmm1/m128, xmm2
    MOVUPD_WdqVdq,
    // [VEX.128.66.0F.WIG 10 /r] VMOVUPD xmm1, xmm2/m128
    VMOVUPD_VdqWdq_V128,
    // [VEX.128.66.0F.WIG 11 /r] VMOVUPD xmm1/m128, xmm2
    VMOVUPD_WdqVdq_V128,
    // [VEX.256.66.0F.WIG 10 /r] VMOVUPD ymm1, ymm2/m256
    VMOVUPD_VqqWqq_V256,
    // [VEX.256.66.0F.WIG 11 /r] VMOVUPD ymm1/m256, ymm2
    VMOVUPD_WqqVqq_V256,
    // [EVEX.128.66.0F.W1 10 /r] VMOVUPD xmm1 {k1}{z}, xmm2/m128
    VMOVUPD_VdqWdq_E128,
    // [EVEX.128.66.0F.W1 11 /r] VMOVUPD xmm1/m128 {k1}{z}, xmm2
    VMOVUPD_WdqVdq_E128,
    // [EVEX.256.66.0F.W1 10 /r] VMOVUPD ymm1 {k1}{z}, ymm2/m256
    VMOVUPD_VqqWqq_E256,
    // [EVEX.256.66.0F.W1 11 /r] VMOVUPD ymm1/m256 {k1}{z}, ymm2
    VMOVUPD_WqqVqq_E256,
    // [EVEX.512.66.0F.W1 10 /r] VMOVUPD zmm1 {k1}{z}, zmm2/m512
    VMOVUPD_VdqqWdqq_E512,
    // [EVEX.512.66.0F.W1 11 /r] VMOVUPD zmm1/m512 {k1}{z}, zmm2
    VMOVUPD_WdqqVdqq_E512,

    // [NP 0F 10 /r] MOVUPS xmm1, xmm2/m128
    MOVUPS_VdqWdq,
    // [NP 0F 11 /r] MOVUPS xmm1/m128, xmm2
    MOVUPS_WdqVdq,
    // [VEX.128.0F.WIG 10 /r] VMOVUPS xmm1, xmm2/m128
    VMOVUPS_VdqWdq_V128,
    // [VEX.128.0F.WIG 11 /r] VMOVUPS xmm1/m128, xmm2
    VMOVUPS_WdqVdq_V128,
    // [VEX.256.0F.WIG 10 /r] VMOVUPS ymm1, ymm2/m256
    VMOVUPS_VqqWqq_V256,
    // [VEX.256.0F.WIG 11 /r] VMOVUPS ymm1/m256, ymm2
    VMOVUPS_WqqVqq_V256,
    // [EVEX.128.0F.W1 10 /r] VMOVUPS xmm1 {k1}{z}, xmm2/m128
    VMOVUPS_VdqWdq_E128,
    // [EVEX.256.0F.W1 10 /r] VMOVUPS ymm1 {k1}{z}, ymm2/m256
    VMOVUPS_VqqWqq_E256,
    // [EVEX.512.0F.W1 10 /r] VMOVUPS zmm1 {k1}{z}, zmm2/m512
    VMOVUPS_VdqqWdqq_E512,
    // [EVEX.128.0F.W1 11 /r] VMOVUPS xmm1/m128 {k1}{z}, xmm2
    VMOVUPS_WdqVdq_E128,
    // [EVEX.256.0F.W1 11 /r] VMOVUPS ymm1/m256 {k1}{z}, ymm2
    VMOVUPS_WqqVqq_E256,
    // [EVEX.512.0F.W1 11 /r] VMOVUPS zmm1/m512 {k1}{z}, zmm2
    VMOVUPS_WdqqVdqq_E512,

    // [0F B6 /r] MOVZX r16, r/m8
    MOVZX_GwEb,
    // [0F B6 /r] MOVZX r32, r/m8
    MOVZX_GdEb,
    // [REX.W 0F B6 /r] MOVZX r64, r/m8
    MOVZX_GqEb,
    // [0F B7 /r] MOVZX r32, r/m16
    MOVZW_GdEw,
    // [REX.W 0F B7 /r] MOVZX r64, r/m16
    MOVZW_GqEw,

    // [66 0F 3A 42 /r ib] MPSADBW xmm1, xmm2/m128, imm8
    MPSADBW_VdqWdqIb,
    // [VEX.128.66.0F3A.WIG 42 /r ib] VMPSADBW xmm1, xmm2, xmm3/m128, imm8
    VMPSADBW_VdqHdqWdqIb_V128,
    // [VEX.256.66.0F3A.WIG 42 /r ib] VMPSADBW ymm1, ymm2, ymm3/m256, imm8
    VMPSADBW_VqqHqqWqqIb_V256,

    // [F6 /4] MUL r/m8
    // [REX F6 /4] MUL r/m8
    MUL_Eb,
    // [F7 /4] MUL r/m16
    MUL_Ew,
    // [F7 /4] MUL r/m32
    MUL_Ed,
    // [REX.W F7 /4] MUL r/m64
    MUL_Eq,

    // [66 0F 59 /r] MULPD xmm1, xmm2/m128
    MULPD_VdqWdq,
    // [VEX.128.66.0F.WIG 59 /r] VMULPD xmm1, xmm2, xmm3/m128
    VMULPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 59 /r] VMULPD ymm1, ymm2, ymm3/m256
    VMULPD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W1 59 /r] VMULPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VMULPD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 59 /r] VMULPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VMULPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 59 /r] VMULPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VMULPD_VdqqHdqqWdqq_E512,

    // [NP 0F 59 /r] MULPS xmm1, xmm2/m128
    MULPS_VdqWdq,
    // [VEX.128.0F.WIG 59 /r] VMULPS xmm1, xmm2, xmm3/m128
    VMULPS_VdqHdqWdq_V128,
    // [VEX.256.0F.WIG 59 /r] VMULPS ymm1, ymm2, ymm3/m256
    VMULPS_VqqHqqWqq_V256,
    // [EVEX.128.0F.W1 59 /r] VMULPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VMULPS_VdqHdqWdq_E128,
    // [EVEX.256.0F.W1 59 /r] VMULPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VMULPS_VqqHqqWqq_E256,
    // [EVEX.512.0F.W1 59 /r] VMULPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VMULPS_VdqqHdqqWdqq_E512,

    // [F2 0F 59 /r] MULSD xmm1, xmm2/m64
    MULSD_VdqWq,
    // [VEX.LIG.F2.0F.WIG 59 /r] VMULSD xmm1, xmm2, xmm3/m64
    VMULSD_VdqHdqWq_V,
    // [EVEX.LIG.F2.0F.W1 59 /r] VMULSD xmm1 {k1}{z}, xmm2, xmm3/m64{er}
    VMULSD_VdqHdqWq_E,

    // [F3 0F 59 /r] MULSS xmm1, xmm2/m32
    MULSS_VdqWd,
    // [VEX.LIG.F3.0F.WIG 59 /r] VMULSS xmm1, xmm2, xmm3/m32
    VMULSS_VdqHdqWd_V,
    // [EVEX.LIG.F3.0F.W1 59 /r] VMULSS xmm1 {k1}{z}, xmm2, xmm3/m32{er}
    VMULSS_VdqHdqWd_E,

    // [VEX.LZ.F2.0F38.W0 F6 /r] MULX r32a, r32b, r/m32
    MULX_GdBdEd,
    // [VEX.LZ.F2.0F38.W1 F6 /r] MULX r64a, r64b, r/m64
    MULX_GqBqEq,

    // [0F 01 C9] MWAIT
    MWAIT,

    // [F6 /3] NEG r/m8
    // [REX F6 /3] NEG r/m8
    NEG_Eb,
    // [F7 /3] NEG r/m16
    NEG_Ew,
    // [F7 /3] NEG r/m32
    NEG_Ed,
    // [REX.W F7 /3] NEG r/m64
    NEG_Eq,

    // [NP 90] NOP
    NOP,
    // [NP 0F 1F /0] NOP r/m16
    NOP_Ew,
    // [NP 0F 1F /0] NOP r/m32
    NOP_Ed,

    // [F6 /2] NOT r/m8
    // [REX F6 /2] NOT r/m8
    NOT_Eb,
    // [F7 /2] NOT r/m16
    NOT_Ew,
    // [F7 /2] NOT r/m32
    NOT_Ed,
    // [REX.W F7 /2] NOT r/m64
    NOT_Eq,

    // [0C ib] OR AL, imm8
    OR_ALIb,
    // [0D iw] OR AX, imm16
    OR_AXIw,
    // [0D id] OR EAX, imm32
    OR_EAXId,
    // [REX.W 0D id] OR RAX, imm32
    OR_RAXId,
    // [80 /1 ib] OR r/m8, imm8
    // [REX 80 /1 ib] OR r/m8, imm8
    OR_EbIb,
    // [81 /1 iw] OR r/m16, imm16
    OR_EwIw,
    // [81 /1 id] OR r/m32, imm32
    OR_EdId,
    // [REX.W 81 /1 id] OR r/m64, imm32
    OR_EqId,
    // [83 /1 ib] OR r/m16, imm8
    OR_EwIb,
    // [83 /1 ib] OR r/m32, imm8
    OR_EdIb,
    // [REX.W 83 /1 ib] OR r/m64, imm8
    OR_EqIb,
    // [08 /r] OR r/m8, r8
    // [REX 08 /r] OR r/m8, r8
    OR_EbGb,
    // [09 /r] OR r/m16, r16
    OR_EwGw,
    // [09 /r] OR r/m32, r32
    OR_EdGd,
    // [REX.W 09 /r] OR r/m64, r64
    OR_EqGq,
    // [0A /r] OR r8, r/m8
    // [REX 0A /r] OR r8, r/m8
    OR_GbEb,
    // [0B /r] OR r16, r/m16
    OR_GwEw,
    // [0B /r] OR r32, r/m32
    OR_GdEd,
    // [REX.W 0B /r] OR r64, r/m64
    OR_GqEq,

    // [66 0F 56 /r] ORPD xmm1, xmm2/m128
    ORPD_VdqWdq,
    // [VEX.128.66.0F 56 /r] VORPD xmm1, xmm2, xmm3/m128
    VORPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F 56 /r] VORPD ymm1, ymm2, ymm3/m256
    VORPD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W1 56 /r] VORPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VORPD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 56 /r] VORPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VORPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 56 /r] VORPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VORPD_VdqqHdqqWdqq_E512,

    // [NP 0F 56 /r] ORPS xmm1, xmm2/m128
    ORPS_VdqWdq,
    // [VEX.128.0F 56 /r] VORPS xmm1, xmm2, xmm3/m128
    VORPS_VdqHdqWdq_V128,
    // [VEX.256.0F 56 /r] VORPS ymm1, ymm2, ymm3/m256
    VORPS_VqqHqqWqq_V256,
    // [EVEX.128.0F.W1 56 /r] VORPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VORPS_VdqHdqWdq_E128,
    // [EVEX.256.0F.W1 56 /r] VORPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VORPS_VqqHqqWqq_E256,
    // [EVEX.512.0F.W1 56 /r] VORPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VORPS_VdqqHdqqWdqq_E512,

    // [E6 ib] OUT imm8, AL
    OUT_IbAL,
    // [E7 ib] OUT imm8, AX
    OUT_IbAX,
    // [E7 ib] OUT imm8, EAX
    OUT_IbEAX,
    // [EE] OUT DX, AL
    OUT_DXAL,
    // [EF] OUT DX, AX
    OUT_DXAX,
    // [EF] OUT DX, EAX
    OUT_DXEAX,

    // [6E] OUTS m8, DX
    // [6E] OUTSB
    OUTSB,
    // [6F] OUTS m16, DX
    // [6F] OUTSW,
    OUTSW,
    // [6F] OUTS m32, DX
    // [6F] OUTSD
    OUTSD,

    // [EVEX.NDS.128.F2.0F38.W0 68 /r] VP2INTERSECTD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VP2INTERSECTD_VdqHdqWdq_E128,
    // [EVEX.NDS.256.F2.0F38.W0 68 /r] VP2INTERSECTD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VP2INTERSECTD_VqqHqqWqq_E256,
    // [EVEX.NDS.512.F2.0F38.W0 68 /r] VP2INTERSECTD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VP2INTERSECTD_VdqqHdqqWdqq_E512,
    // [EVEX.NDS.128.F2.0F38.W1 68 /r] VP2INTERSECTQ xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VP2INTERSECTQ_VdqHdqWdq_E128,
    // [EVEX.NDS.256.F2.0F38.W1 68 /r] VP2INTERSECTQ ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VP2INTERSECTQ_VqqHqqWqq_E256,
    // [EVEX.NDS.512.F2.0F38.W1 68 /r] VP2INTERSECTQ zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VP2INTERSECTQ_VdqqHdqqWdqq_E512,

    // [NP 0F 38 1C /r] PABSB mm1, mm2/m64
    PABSB_PqQq,
    // [66 0F 38 1C /r] PABSB xmm1, xmm2/m128
    PABSB_VdqWdq,
    // [NP 0F 38 1D /r] PABSW mm1, mm2/m64
    PABSW_PqQq,
    // [66 0F 38 1D /r] PABSW xmm1, xmm2/m128
    PABSW_VdqWdp,
    // [NP 0F 38 1E /r] PABSD mm1, mm2/m64
    PABSD_PqQq,
    // [66 0F 38 1E /r] PABSD xmm1, xmm2/m128
    PABSD_VdqWdq,
    // [VEX.128.66.0F38.WIG 1C /r] VPABSB xmm1, xmm2/m128
    VPABSB_VdqWdq_V128,
    // [VEX.128.66.0F38.WIG 1D /r] VPABSW xmm1, xmm2/m128
    VPABSW_VdqWdq_V128,
    // [VEX.128.66.0F38.WIG 1E /r] VPABSD xmm1, xmm2/m128
    VPABSD_VdqWdq_V128,
    // [VEX.256.66.0F38.WIG 1C /r] VPABSB ymm1, ymm2/m256
    VPABSB_VqqWqq_V256,
    // [VEX.256.66.0F38.WIG 1D /r] VPABSW ymm1, ymm2/m256
    VPABSW_VqqWqq_V256,
    // [VEX.256.66.0F38.WIG 1E /r] VPABSD ymm1, ymm2/m256
    VPABSD_VqqWqq_V256,
    // [EVEX.128.66.0F38.WIG 1C /r] VPABSB xmm1 {k1}{z}, xmm2/m128
    VPABSB_VdqWdq_E128,
    // [EVEX.256.66.0F38.WIG 1C /r] VPABSB ymm1 {k1}{z}, ymm2/m256
    VPABSB_VqqWqq_E256,
    // [EVEX.512.66.0F38.WIG 1C /r] VPABSB zmm1 {k1}{z}, zmm2/m512
    VPABSB_VdqqWdqq_E512,
    // [EVEX.128.66.0F38.WIG 1D /r] VPABSW xmm1 {k1}{z}, xmm2/m128
    VPABSW_VdqWdq_E128,
    // [EVEX.256.66.0F38.WIG 1D /r] VPABSW ymm1 {k1}{z}, ymm2/m256
    VPABSW_VqqWqq_E256,
    // [EVEX.512.66.0F38.WIG 1D /r] VPABSW zmm1 {k1}{z}, zmm2/m512
    VPABSW_VdqqWdqq_E512,
    // [EVEX.128.66.0F38.WIG 1E /r] VPABSD xmm1 {k1}{z}, xmm2/m128
    VPABSD_VdqWdq_E128,
    // [EVEX.256.66.0F38.WIG 1E /r] VPABSD ymm1 {k1}{z}, ymm2/m256
    VPABSD_VqqWqq_E256,
    // [EVEX.512.66.0F38.WIG 1E /r] VPABSD zmm1 {k1}{z}, zmm2/m512
    VPABSD_VdqqWdqq_E512,

    // [NP 0F 63 /r] PACKSSWB mm1, mm2/m64
    PACKSSWB_PqQq,
    // [66 0F 63 /r] PACKSSWB xmm1, xmm2/m128
    PACKSSWB_VdqWdq,
    // [NP 0F 6B /r] PACKSSDW mm1, mm2/m64
    PACKSSDW_PqQq,
    // [66 0F 6B /r] PACKSSDW xmm1, xmm2/m128
    PACKSSDW_VdqWdq,
    // [VEX.128.66.0F.WIG 63 /r] VPACKSSWB xmm1, xmm2, xmm3/m128
    VPACKSSWB_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 6B /r] VPACKSSDW xmm1, xmm2, xmm3/m128
    VPACKSSDW_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 63 /r] VPACKSSWB ymm1, ymm2, ymm3/m256
    VPACKSSWB_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG 6B /r] VPACKSSDW ymm1, ymm2, ymm3/m256
    VPACKSSDW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG 63 /r] VPACKSSWB xmm1 {k1}{z}, xmm2, xmm3/m128
    VPACKSSWB_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG 63 /r] VPACKSSWB ymm1 {k1}{z}, ymm2, ymm3/m256
    VPACKSSWB_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG 63 /r] VPACKSSWB zmm1 {k1}{z}, zmm2, xmm3/m512
    VPACKSSWB_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F.W0 6B /r] VPACKSSDW xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPACKSSDW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W0 6B /r] VPACKSSDW ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPACKSSDW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W0 6B /r] VPACKSSDW zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPACKSSDW_VdqqHdqqWdqq_E512,

    // [66 0F 38 2B /r] PACKUSDW xmm1, xmm2/m128
    PACKUSDW_VdqWdq,
    // [VEX.128.66.0F38.WIG 2B /r] VPACKUSDW xmm1, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VPACKUSDW_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 2B /r] VPACKUSDW ymm1, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VPACKUSDW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W0 2B /r] VPACKUSDW xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPACKUSDW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 2B /r] VPACKUSDW ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPACKUSDW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 2B /r] VPACKUSDW zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPACKUSDW_VdqqHdqqWdqq_E512,

    // [NP 0F 67 /r] PACKUSWB mm1, mm2/m64
    PACKUSWB_PqQq,
    // [66 0F 67 /r] PACKUSWB xmm1, xmm2/m128
    PACKUSWB_VdqWdq,
    // [VEX.128.66.0F.WIG 67 /r] VPACKUSWB xmm1, xmm2, xmm3/m128
    VPACKUSWB_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 67 /r] VPACKUSWB ymm1, ymm2, ymm3/m128
    VPACKUSWB_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG 67 /r] VPACKUSWB xmm1 {k1}{z}, xmm2, xmm3/m128
    VPACKUSWB_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG 67 /r] VPACKUSWB ymm1 {k1}{z}, ymm2, ymm3/m256
    VPACKUSWB_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG 67 /r] VPACKUSWB zmm1 {k1}{z}, zmm2, zmm3/m512
    VPACKUSWB_VdqqHdqqWdqq_E512,

    // [NP 0F FC /r] PADDB mm1, mm2/m64
    PADDB_PqQq,
    // [NP 0F FD /r] PADDW mm1, mm2/m64
    PADDW_PqQq,
    // [NP 0F FE /r] PADDD mm1, mm2/m64
    PADDD_PqQq,
    // [NP 0F D4 /r] PADDQ mm1, mm2/m64
    PADDQ_PqQq,
    // [66 0F FC /r] PADDB xmm1, xmm2/m128
    PADDB_VdqWdq,
    // [66 0F FD /r] PADDW xmm1, xmm2/m128
    PADDW_VdqWdq,
    // [66 0F FE /r] PADDD xmm1, xmm2/m128
    PADDD_VdqWdq,
    // [66 0F D4 /r] PADDQ xmm1, xmm2/m128
    PADDQ_VdqWdq,
    // [VEX.128.66.0F.WIG FC /r] VPADDB xmm1, xmm2, xmm3/m128
    VPADDB_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG FD /r] VPADDW xmm1, xmm2, xmm3/m128
    VPADDW_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG FE /r] VPADDD xmm1, xmm2, xmm3/m128
    VPADDD_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG D4 /r] VPADDQ xmm1, xmm2, xmm3/m128
    VPADDQ_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG FC /r] VPADDB ymm1, ymm2, ymm3/m256
    VPADDB_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG FD /r] VPADDW ymm1, ymm2, ymm3/m256
    VPADDW_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG FE /r] VPADDD ymm1, ymm2, ymm3/m256
    VPADDD_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG D4 /r] VPADDQ ymm1, ymm2, ymm3/m256
    VPADDQ_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG FC /r] VPADDB xmm1 {k1}{z}, xmm2, xmm3/m128
    VPADDB_VdqHdqWdq_E128,
    // [EVEX.128.66.0F.WIG FD /r] VPADDW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPADDW_VdqHdqWdq_E128,
    // [EVEX.128.66.0F.W0 FE /r] VPADDD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPADDD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F.W1 D4 /r] VPADDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPADDQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG FC /r] VPADDB ymm1 {k1}{z}, ymm2, ymm3/m256
    VPADDB_VqqHqqWqq_E256,
    // [EVEX.256.66.0F.WIG FD /r] VPADDW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPADDW_VqqHqqWqq_E256,
    // [EVEX.256.66.0F.W0 FE /r] VPADDD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPADDD_VqqHqqWqq_E256,
    // [EVEX.256.66.0F.W1 D4 /r] VPADDQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPADDQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG FC /r] VPADDB zmm1 {k1}{z}, zmm2, zmm3/m512
    VPADDB_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F.WIG FD /r] VPADDW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPADDW_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F.W0 FE /r] VPADDD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPADDD_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F.W1 D4 /r] VPADDQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPADDQ_VdqqHdqqWdqq_E512,

    // [NP 0F EC /r] PADDSB mm1, mm2/m64
    PADDSB_PqQq,
    // [66 0F EC /r] PADDSB xmm1, xmm2/m128
    PADDSB_VdqWdq,
    // [NP 0F ED /r] PADDSW mm1, mm2/m64
    PADDSW_PqQq,
    // [66 0F ED /r] PADDSW xmm1, xmm2/m128
    PADDSW_VdqWdq,
    // [VEX.128.66.0F.WIG EC /r] VPADDSB xmm1, xmm2, xmm3/m128
    VPADDSB_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG ED /r] VPADDSW xmm1, xmm2, xmm3/m128
    VPADDSW_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG EC /r] VPADDSB ymm1, ymm2, ymm3/m256
    VPADDSB_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG ED /r] VPADDSW ymm1, ymm2, ymm3/m256
    VPADDSW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG EC /r] VPADDSB xmm1 {k1}{z}, xmm2, xmm3/m128
    VPADDSB_VdqHdqWdq_E128,
    // [EVEX.128.66.0F.WIG ED /r] VPADDSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPADDSW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG EC /r] VPADDSB ymm1 {k1}{z}, ymm2, ymm3/m256
    VPADDSB_VqqHqqWqq_E256,
    // [EVEX.256.66.0F.WIG ED /r] VPADDSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPADDSW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG EC /r] VPADDSB zmm1 {k1}{z}, zmm2, zmm3/m512
    VPADDSB_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F.WIG ED /r] VPADDSW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPADDSW_VdqqHdqqWdqq_E512,

    // [NP 0F DC /r] PADDUSB mm1, mm2/m64
    PADDUSB_PqQq,
    // [66 0F DC /r] PADDUSB xmm1, xmm2/m128
    PADDUSB_VdqWdq,
    // [NP 0F DD /r] PADDUSW mm1, mm2/m64
    PADDUSW_PqQq,
    // [66 0F DD /r] PADDUSW xmm1, xmm2/m128
    PADDUSW_VdqWdq,
    // [VEX.128.66.0F.WIG DC /r] VPADDUSB xmm1, xmm2, xmm3/m128
    VPADDUSB_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG DD /r] VPADDUSW xmm1, xmm2, xmm3/m128
    VPADDUSW_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG DC /r] VPADDUSB ymm1, ymm2, ymm3/m256
    VPADDUSB_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG DD /r] VPADDUSW ymm1, ymm2, ymm3/m256
    VPADDUSW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG DC /r] VPADDUSB xmm1 {k1}{z}, xmm2, xmm3/m128
    VPADDUSB_VdqHdqWdq_E128,
    // [EVEX.128.66.0F.WIG DD /r] VPADDUSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPADDUSW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG DC /r] VPADDUSB ymm1 {k1}{z}, ymm2, ymm3/m256
    VPADDUSB_VqqHqqWqq_E256,
    // [EVEX.256.66.0F.WIG DD /r] VPADDUSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPADDUSW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG DC /r] VPADDUSB zmm1 {k1}{z}, zmm2, zmm3/m512
    VPADDUSB_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F.WIG DD /r] VPADDUSW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPADDUSW_VdqqHdqqWdqq_E512,

    // [NP 0F 3A 0F /r ib] PALIGNR mm1, mm2/m64, imm8
    PALIGNR_PqQqIb,
    // [66 0F 3A 0F /r ib] PALIGNR xmm1, xmm2/m128, imm8
    PALIGNR_VdqWdqIb,
    // [VEX.128.66.0F3A.WIG 0F /r ib] VPALIGNR xmm1, xmm2, xmm3/m128, imm8
    VPALIGNR_VdqHdqWdqIb_V128,
    // [VEX.256.66.0F3A.WIG 0F /r ib] VPALIGNR ymm1, ymm2, ymm3/m256, imm8
    VPALIGNR_VqqHqqWqqIb_V256,
    // [EVEX.128.66.0F3A.WIG 0F /r ib] VPALIGNR xmm1 {k1}{z}, xmm2, xmm3/m128, imm8
    VPALIGNR_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.WIG 0F /r ib] VPALIGNR ymm1 {k1}{z}, ymm2, ymm3/m256, imm8
    VPALIGNR_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.WIG 0F /r ib] VPALIGNR zmm1 {k1}{z}, zmm2, zmm3/m512, imm8
    VPALIGNR_VdqqHdqqWdqqIb_E512,

    // [NP 0F DB /r] PAND mm1, mm2/m64
    PAND_PqQq,
    // [66 0F DB /r] PAND xmm1, xmm2/m128
    PAND_VdqWdq,
    // [VEX.128.66.0F.WIG DB /r] VPAND xmm1, xmm2, xmm3/m128
    PAND_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG DB /r] VPAND ymm1, ymm2, ymm3/m256
    PAND_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W0 DB /r] VPANDD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPANDD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W0 DB /r] VPANDD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPANDD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W0 DB /r] VPANDD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPANDD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F.W1 DB /r] VPANDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPANDQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 DB /r] VPANDQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPANDQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 DB /r] VPANDQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPANDQ_VdqqHdqqWdqq_E512,

    // [NP 0F DF /r] PANDN mm1, mm2/m64
    PANDN_PqQq,
    // [66 0F DF /r] PANDN xmm1, xmm2/m128
    PANDN_VdqWdq,
    // [VEX.128.66.0F.WIG DF /r] VPANDN xmm1, xmm2, xmm3/m128
    PANDN_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG DF /r] VPANDN ymm1, ymm2, ymm3/m256
    PANDN_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W0 DF /r] VPANDND xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPANDND_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W0 DF /r] VPANDND ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPANDND_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W0 DF /r] VPANDND zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPANDND_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F.W1 DF /r] VPANDNQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPANDNQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 DF /r] VPANDNQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPANDNQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 DF /r] VPANDNQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPANDNQ_VdqqHdqqWdqq_E512,

    // [F3 90] PAUSE
    PAUSE,

    // [NP 0F E0 /r] PAVGB mm1, mm2/m64
    PAVGB_PqQq,
    // [66 0F E0 /r] PAVGB xmm1, xmm2/m128
    PAVGB_VdqWdq,
    // [NP 0F E3 /r] PAVGW mm1, mm2/m64
    PAVGW_PqQq,
    // [66 0F E3 /r] PAVGW xmm1, xmm2/m128
    PAVGW_VdqWdq,
    // [VEX.128.66.0F.WIG E0 /r] VPAVGB xmm1, xmm2, xmm3/m128
    VPAVGB_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG E3 /r] VPAVGW xmm1, xmm2, xmm3/m128
    VPAVGW_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG E0 /r] VPAVGB ymm1, ymm2, ymm3/m256
    VPAVGB_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG E3 /r] VPAVGW ymm1, ymm2, ymm3/m256
    VPAVGW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG E0 /r] VPAVGB xmm1 {k1}{z}, xmm2, xmm3/m128
    VPAVGB_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG E0 /r] VPAVGB ymm1 {k1}{z}, ymm2, ymm3/m256
    VPAVGB_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG E0 /r] VPAVGB zmm1 {k1}{z}, zmm2, zmm3/m512
    VPAVGB_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F.WIG E3 /r] VPAVGW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPAVGW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG E3 /r] VPAVGW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPAVGW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG E3 /r] VPAVGW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPAVGW_VdqqHdqqWdqq_E512,

    // [66 0F 38 10 /r] PBLENDVB xmm1, xmm2/m128, <XMM0>
    PBLENDVB_VdqWdq,
    // [VEX.128.66.0F3A.W0 4C /r /is4] VPBLENDVB xmm1, xmm2, xmm3/m128, xmm4
    VPBLENDVB_VdqHdqWdqIb_V128,
    // [VEX.256.66.0F3A.W0 4C /r /is4] VPBLENDVB ymm1, ymm2, ymm3/m256, ymm4
    VPBLENDVB_VqqHqqWqqIb_V256,

    // [66 0F 38 0E /r] PBLENDVW xmm1, xmm2/m128, imm8
    PBLENDVW_VdqWdqIb,
    // [VEX.128.66.0F3A.W0 0E /r ib] VPBLENDVW xmm1, xmm2, xmm3/m128, imm8
    VPBLENDVW_VdqHdqWdqIb_V128,
    // [VEX.256.66.0F3A.W0 0E /r ib] VPBLENDVW ymm1, ymm2, ymm3/m256, imm8
    VPBLENDVW_VqqHqqWqqIb_V256,

    // [66 0F 3A 44 /r ib] PCLMULQDQ xmm1, xmm2/m128, imm8
    PCLMULQDQ_VdqWdqIb,
    // [VEX.128.66.0F3A.WIG 44 /r ib] VPCLMULQDQ xmm1, xmm2, xmm3/m128, imm8
    VPCLMULQDQ_VdqHdqWdqIb_V128,
    // [VEX.256.66.0F3A.WIG 44 /r ib] VPCLMULQDQ ymm1, ymm2, ymm3/m256, imm8
    VPCLMULQDQ_VqqHqqWqqIb_V256,
    // [EVEX.128.66.0F3A.WIG 44 /r ib] VPCLMULQDQ xmm1, xmm2, xmm3/m128, imm8
    VPCLMULQDQ_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.WIG 44 /r ib] VPCLMULQDQ ymm1, ymm2, ymm3/m256, imm8
    VPCLMULQDQ_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.WIG 44 /r ib] VPCLMULQDQ zmm1, zmm2, zmm3/m512, imm8
    VPCLMULQDQ_VdqqHdqqWdqqIb_E512,

    // [NP 0F 74 /r] PCMPEQB mm1, mm2/m64
    PCMPEQB_PqQq,
    // [66 0F 74 /r] PCMPEQB xmm1, xmm2/m128
    PCMPEQB_VdqWdq,
    // [NP 0F 75 /r] PCMPEQW mm1, mm2/m64
    PCMPEQW_PqQq,
    // [66 0F 75 /r] PCMPEQW xmm1, xmm2/m128
    PCMPEQW_VdqWdq,
    // [NP 0F 76 /r] PCMPEQD mm1, mm2/m64
    PCMPEQD_PqQq,
    // [66 0F 76 /r] PCMPEQD xmm1, xmm2/m128
    PCMPEQD_VdqWdq,
    // [VEX.128.66.0F.WIG 74 /r] VCMPEQB xmm1, xmm2, xmm3/m128
    VCMPEQB_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 75 /r] VCMPEQW xmm1, xmm2, xmm3/m128
    VCMPEQW_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 76 /r] VCMPEQD xmm1, xmm2, xmm3/m128
    VCMPEQD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 74 /r] VCMPEQB ymm1, ymm2, ymm3/m256
    VCMPEQB_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG 75 /r] VCMPEQW ymm1, ymm2, ymm3/m256
    VCMPEQW_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG 76 /r] VCMPEQD ymm1, ymm2, ymm3/m256
    VCMPEQD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG 74 /r] VCMPEQB k1 {k2}, xmm2, xmm3/m128
    VPCMPEQB_KGqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG 74 /r] VCMPEQB k1 {k2}, ymm2, ymm3/m256
    VPCMPEQB_KGqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG 74 /r] VCMPEQB k1 {k2}, zmm2, zmm3/m512
    VPCMPEQB_KGqHdqqWdqq_E512,
    // [EVEX.128.66.0F.WIG 75 /r] VCMPEQW k1 {k2}, xmm2, xmm3/m128
    VPCMPEQW_KGqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG 75 /r] VCMPEQW k1 {k2}, ymm2, ymm3/m256
    VPCMPEQW_KGqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG 75 /r] VCMPEQW k1 {k2}, zmm2, zmm3/m512
    VPCMPEQW_KGqHdqqWdqq_E512,
    // [EVEX.128.66.0F.WIG 76 /r] VCMPEQD k1 {k2}, xmm2, xmm3/m128/m32bcst
    VPCMPEQD_KGqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG 76 /r] VCMPEQD k1 {k2}, ymm2, ymm3/m256/m32bcst
    VPCMPEQD_KGqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG 76 /r] VCMPEQD k1 {k2}, zmm2, zmm3/m512/m32bcst
    VPCMPEQD_KGqHdqqWdqq_E512,

    // [66 0F 38 29 /r] PCMPEQQ xmm1, xmm2/m128
    PCMPEQQ_VdqWdq,
    // [VEX.128.66.0F38.WIG 29 /r] VPCMPEQQ xmm1, xmm2, xmm3/m128
    VPCMPEQQ_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 29 /r] VPCMPEQQ ymm1, ymm2, ymm3/m256
    VPCMPEQQ_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W1 29 /r] VPCMPEQQ k1 {k2}, xmm2, xmm3/m128/m64bcst
    VPCMPEQQ_KgqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 29 /r] VPCMPEQQ k1 {k2}, ymm2, ymm3/m256/m64bcst
    VPCMPEQQ_KgqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 29 /r] VPCMPEQQ k1 {k2}, zmm2, zmm3/m512/m64bcst
    VPCMPEQQ_KgqHdqqWdqq_E512,

    // [66 0F 3A 61 /r ib] PCMPESTRI xmm1, xmm2/m128, imm8
    PCMPESTRI_VdqWdqIb,
    // [VEX.128.66.0F3A 61 /r ib] VPCMPESTRI xmm1, xmm2/m128, imm8
    VPCMPESTRI_VdqHdqWdqIb_V128,

    // [66 0F 3A 60 /r ib] PCMPESTRM xmm1, xmm2/m128, imm8
    PCMPESTRM_VdqWdqIb,
    // [VEX.128.66.0F3A 60 /r ib] VPCMPESTRM xmm1, xmm2/m128, imm8
    VPCMPESTRM_VdqHdqWdqIb_V128,

    // [NP 0F 64 /r] PCMPGTB mm1, mm2/m64
    PCMPGTB_PqQq,
    // [66 0F 64 /r] PCMPGTB xmm1, xmm2/m128
    PCMPGTB_VdqWdq,
    // [NP 0F 65 /r] PCMPGTB mm1, mm2/m64
    PCMPGTW_PqQq,
    // [66 0F 65 /r] PCMPGTB xmm1, xmm2/m128
    PCMPGTW_VdqWdq,
    // [NP 0F 66 /r] PCMPGTB mm1, mm2/m64
    PCMPGTD_PqQq,
    // [66 0F 66 /r] PCMPGTB xmm1, xmm2/m128
    PCMPGTD_VdqWdq,
    // [VEX.128.66.0F.WIG 64 /r] VCMPGTB xmm1, xmm2, xmm3/m128
    VPCMPGTB_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 65 /r] VCMPGTW xmm1, xmm2, xmm3/m128
    VPCMPGTW_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 66 /r] VCMPGTD xmm1, xmm2, xmm3/m128
    VPCMPGTD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 64 /r] VCMPGTB ymm1, ymm2, ymm3/m256
    VPCMPGTB_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG 65 /r] VCMPGTW ymm1, ymm2, ymm3/m256
    VPCMPGTW_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG 66 /r] VCMPGTD ymm1, ymm2, ymm3/m256
    VPCMPGTD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG 64 /r] VCMPPGTB k1 {k2}, xmm2, xmm3/m128
    VPCMPGTB_KGqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG 64 /r] VCMPPGTB k1 {k2}, ymm2, ymm3/m256
    VPCMPGTB_KGqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG 64 /r] VCMPPGTB k1 {k2}, zmm2, zmm3/m512
    VPCMPGTB_KGqHdqqWdqq_E512,
    // [EVEX.128.66.0F.WIG 65 /r] VCMPPGTW k1 {k2}, xmm2, xmm3/m128
    VPCMPGTW_KGqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG 65 /r] VCMPPGTW k1 {k2}, ymm2, ymm3/m256
    VPCMPGTW_KGqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG 65 /r] VCMPPGTW k1 {k2}, zmm2, zmm3/m512
    VPCMPGTW_KGqHdqqWdqq_E512,
    // [EVEX.128.66.0F.WIG 66 /r] VCMPPGTD k1 {k2}, xmm2, xmm3/m128/m32bcst
    VPCMPGTD_KGqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG 66 /r] VCMPPGTD k1 {k2}, ymm2, ymm3/m256/m32bcst
    VPCMPGTD_KGqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG 66 /r] VCMPPGTD k1 {k2}, zmm2, zmm3/m512/m32bcst
    VPCMPGTD_KGqHdqqWdqq_E512,

    // [66 0F 38 37 /r] PCMPGTQ xmm1, xmm2/m128
    PCMPGTQ_VdqWdq,
    // [VEX.128.66.0F38.WIG 37 /r] VCMPGTQ xmm1, xmm2, xmm3/m128
    VPCMPGTQ_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 37 /r] VCMPGTQ ymm1, ymm2, ymm3/m256
    VPCMPGTQ_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W1 37 /r] VCMPGTQ k1 {k2}, xmm2, xmm3/m128/m64bcst
    VPCMPGTQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 37 /r] VCMPGTQ k1 {k2}, ymm2, ymm3/m256/m64bcst
    VPCMPGTQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 37 /r] VCMPGTQ k1 {k2}, zmm2, zmm3/m512/m64bcst
    VPCMPGTQ_VdqqHdqqWdqq_E512,

    // [66 0F 3A 63 /r ib] PCMPISTRI xmm1, xmm2/m128, imm8
    PCMPISTRI_VdqWdqIb,
    // [VEX.128.66.0F3A.WIG 63 /r ib] VPCMPISTRI xmm1, xmm2/m128, imm8
    VPCMPISTRI_VdqHdqWdqIb,

    // [66 0F 3A 62 /r ib] PCMPISTRM xmm1, xmm2/m128, imm8
    PCMPISTRM_VdqWdqIb,
    // [VEX.128.66.0F3A.WIG 62 /r ib] VPCMPISTRM xmm1, xmm2/m128, imm8
    VPCMPISTRM_VdqHdqWdqIb,

    // [VEX.LZ.F2.0F38.W0 F5 /r] PDEP r32a, r32b, r/m32
    PDEP_GdBdEd,
    // [VEX.LZ.F2.0F38.W1 F5 /r] PDEP r64a, r64b, r/m64
    PDEP_GqBqEq,

    // [VEX.LZ.F3.0F38.W0 F5 /r] PEXT r32a, r32b, r/m32
    PEXT_GdBdEd,
    // [VEX.LZ.F3.0F38.W1 F5 /r] PEXT r64a, r64b, r/m64
    PEXT_GqBqEq,

    // [66 0F 3A 14 /r ib] PEXTRB r/m8, xmm1, imm8
    PEXTRB_EdVdqIb,
    // [66 0F 3A 16 /r ib] PEXTRD r/m32, xmm1, imm8
    PEXTRD_EdVdqIb,
    // [66 REX.W 0F 3A 16 /r ib] PEXTRQ r/m64, xmm1, imm8
    PEXTRQ_EqVdqIb,
    // [VEX.128.66.0F3A.W0 14 /r ib] VPEXTRB r/m8, xmm1, imm8
    VPEXTRB_EbVdqIb_V128,
    // [VEX.128.66.0F3A.W0 16 /r ib] VPEXTRD r/m32, xmm1, imm8
    VPEXTRD_EdVdqIb_V128,
    // [VEX.128.66.0F3A.W1 16 /r ib] VPEXTRQ r/m64, xmm1, imm8
    VPEXTRQ_EqVdqIb_V128,
    // [EVEX.128.66.0F3A.WIG 14 /r ib] VPEXTRB r/m8, xmm1, imm8
    VPEXTRB_EbVdqIb_E128,
    // [EVEX.128.66.0F3A.W0 16 /r ib] VPEXTRD r/m32, xmm1, imm8
    VPEXTRD_EdVdqIb_E128,
    // [EVEX.128.66.0F3A.W1 16 /r ib] VPEXTRQ r/m64, xmm1, imm8
    VPEXTRQ_EqVdqIb_E128,

    // [NP 0F C5 /r ib] PEXTRW reg, mm1, imm8
    PEXTRW_GwNqIb,
    // [66 0F C5 /r ib] PEXTRW reg, xmm1, imm8
    PEXTRW_GwUdqIb,
    // [66 0F 3A 15 /r ib] PEXTRW r/m16, xmm1, imm8
    PEXTRW_EwVdqIb,
    // [VEX.128.66.0F.W0 C5 /r ib] VPEXTRW reg, xmm1, imm8
    VPEXTRW_GwUdqIb_V128,
    // [VEX.128.66.0F3A.W0 15 /r ib] VPEXTRW r/m16, xmm1, imm8
    VPEXTRW_EwVdqIb_V128,
    // [EVEX.128.66.0F.WIG C5 /r ib] VPEXTRW reg, xmm1, imm8
    VPEXTRW_GwUdqIb_E128,
    // [EVEX.128.66.0F3A.WIG 15 /r ib] VPEXTRW r/m16, xmm1, imm8
    VPEXTRW_EwVdqIb_E128,

    // [NP 0F 38 01 /r] PHADDW mm1, mm2/m64
    PHADDW_PqQq,
    // [66 0F 38 01 /r] PHADDW xmm1, xmm2/m128
    PHADDW_VdqWdq,
    // [NP 0F 38 02 /r] PHADDD mm1, mm2/m64
    PHADDD_PqQq,
    // [66 0F 38 02 /r] PHADDD xmm1, xmm2/m128
    PHADDD_VdqWdq,
    // [VEX.128.66.0F38.WIG 01 /r] VPHADDW xmm1, xmm2, xmm3/m128
    VPHADDW_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.WIG 02 /r] VPHADDD xmm1, xmm2, xmm3/m128
    VPHADDD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 01 /r] VPHADDW ymm1, ymm2, ymm3/m256
    VPHADDW_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.WIG 02 /r] VPHADDD ymm1, ymm2, ymm3/m256
    VPHADDD_VqqHqqWqq_V256,

    // [NP 0F 38 03 /r] PHADDSW mm1, mm2/m64
    PHADDSW_PqQq,
    // [66 0F 38 03 /r] PHADDSW xmm1, xmm2/m128
    PHADDSW_VdqWdq,
    // [VEX.128.66.0F38.WIG 03 /r] VPHADDSW xmm1, xmm2, xmm3/m128
    VPHADDSW_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 03 /r] VPHADDSW ymm1, ymm2, ymm3/m256
    VPHADDSW_VqqHqqWqq_V256,

    // [66 0F 38 41 /r] PHMINPOSUW xmm1, xmm2/m128
    PHMINPOSUW_VdqWdq,
    // [VEX.128.66.0F38.WIG 41 /r] VPHMINPOSUW xmm1, xmm2/m128
    VPHMINPOSUW_VdqWdq_V128,

    // [NP 0F 38 05 /r] PHSUBW mm1, mm2/m64
    PHSUBW_PqQq,
    // [66 0F 38 05 /r] PHSUBW xmm1, xmm2/m128
    PHSUBW_VdqWdq,
    // [NP 0F 38 06 /r] PHSUBD mm1, mm2/m64
    PHSUBD_PqQq,
    // [66 0F 38 06 /r] PHSUBD xmm1, xmm2/m128
    PHSUBD_VdqWdq,
    // [VEX.128.66.0F38.WIG 05 /r] VPHSUBW xmm1, xmm2, xmm3/m128
    VPHSUBW_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.WIG 06 /r] VPHSUBD xmm1, xmm2, xmm3/m128
    VPHSUBD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 05 /r] VPHSUBW ymm1, ymm2, ymm3/m256
    VPHSUBW_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.WIG 06 /r] VPHSUBD ymm1, ymm2, ymm3/m256
    VPHSUBD_VqqHqqWqq_V256,

    // [NP 0F 38 07 /r] PHSUBSW mm1, mm2/m64
    PHSUBSW_PqQq,
    // [66 0F 38 07 /r] PHSUBSW xmm1, xmm2/m128
    PHSUBSW_VdqWdq,
    // [VEX.128.66.0F38.WIG 07 /r] VPHSUBSW xmm1, xmm2, xmm3/m128
    VPHSUBSW_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 07 /r] VPHSUBSW ymm1, ymm2, ymm3/m256
    VPHSUBSW_VqqHqqWqq_V256,

    // [66 0F 3A 20 /r ib] PINSRB xmm1, r32/m8, imm8
    PINSRB_VdqEbIb,
    // [66 0F 3A 22 /r ib] PINSRD xmm1, r/m32, imm8
    PINSRD_VdqEdIb,
    // [66 REX.W 0F 3A 22 /r ib] PINSRQ xmm1, r/m64, imm8
    PINSRQ_VdqEqIb,
    // [VEX.128.66.0F3A.W0 20 /r ib] VPINSRB xmm1, xmm2, r32/m8, imm8
    VPINSRB_VdqHdqEbIb_V128,
    // [VEX.128.66.0F3A.W0 22 /r ib] VPINSRD xmm1, xmm2, r/m32, imm8
    VPINSRD_VdqHdqEdIb_V128,
    // [VEX.128.66.0F3A.W1 22 /r ib] VPINSRQ xmm1, xmm2, r/m64, imm8
    VPINSRQ_VdqHdqEqIb_V128,
    // [EVEX.128.66.0F3A.WIG 20 /r ib] VPINSRB xmm1, xmm2, r32/m8, imm8
    VPINSRB_VdqHdqEbIb_E128,
    // [EVEX.128.66.0F3A.W0 22 /r ib] VPINSRD xmm1, xmm2, r/m32, imm8
    VPINSRD_VdqHdqEdIb_E128,
    // [EVEX.128.66.0F3A.W1 22 /r ib] VPINSRQ xmm1, xmm2, r/m64, imm8
    VPINSRQ_VdqHdqEqIb_E128,

    // [NP 0F C4 /r ib] PINSRW mm1, r32/m16, imm8
    PINSRW_PqEwIb,
    // [66 0F C4 /r ib] PINSRW xmm1, r32/m16, imm8
    PINSRW_VdqEwIb,
    // [VEX.128.66.0F.W0 C4 /r ib] VPINSRW xmm1, xmm2, r32/m16, imm8
    VPINSRW_VdqHdqEwIb_V128,
    // [EVEX.128.66.0F.WIG C4 /r ib] VPINSRW xmm1, xmm2, r32/m16, imm8
    VPINSRW_VdqHdqEWIb_E128,

    // [NP 0F 38 04 /r] PMADDUBSW mm1, mm2/m64
    PMADDUBSW_PqQq,
    // [66 0F 38 04 /r] PMADDUBSW xmm1, xmm2/m128
    PMADDUBSW_VdqWdq,
    // [VEX.128.66.0F38.WIG 04 /r] VPMADDUBSW xmm1, xmm2, xmm3/m128
    VPMADDUBSW_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 04 /r] VPMADDUBSW ymm1, ymm2, ymm3/m256
    VPMADDUBSW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.WIG 04 /r] VPMADDUBSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPMADDUBSW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.WIG 04 /r] VPMADDUBSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPMADDUBSW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.WIG 04 /r] VPMADDUBSW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPMADDUBSW_VdqqHdqqWdqq_E512,

    // [NP 0F F5 /r] PMADDWD mm1, mm2/m64
    PMADDWD_PqQq,
    // [66 0F F5 /r] PMADDWD xmm1, xmm2/m128
    PMADDWD_VdqWdq,
    // [VEX.128.66.0F.WIG F5 /r] VPMADDWD xmm1, xmm2, xmm3/m128
    VPMADDWD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG F5 /r] VPMADDWD ymm1, ymm2, ymm3/m256
    VPMADDWD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG F5 /r] VPMADDWD xmm1 {k1}{z}, xmm2, xmm3/m128
    VPMADDWD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG F5 /r] VPMADDWD ymm1 {k1}{z}, ymm2, ymm3/m256
    VPMADDWD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG F5 /r] VPMADDWD zmm1 {k1}{z}, zmm2, zmm3/m512
    VPMADDWD_VdqqHdqqWdqq_E512,

    // [NP 0F EE /r] PMAXSW mm1, mm2/m64
    PMAXSW_PqQq,
    // [66 0F 38 3C /r] PMAXSB xmm1, xmm2/m128
    PMAXSB_VdqWdq,
    // [66 0F EE /r] PMAXSW xmm1, xmm2/m128
    PMAXSW_VdqWdq,
    // [66 0F 38 3D /r] PMAXSD xmm1, xmm2/m128
    PMAXSD_VdqWdq,
    // [VEX.128.66.0F38.WIG 3C /r] VPMAXSB xmm1, xmm2, xmm3/m128
    VPMAXSB_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG EE /r] VPMAXSW xmm1, xmm2, xmm3/m128
    VPMAXSW_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.WIG 3D /r] VPMAXSD xmm1, xmm2, xmm3/m128
    VPMAXSD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 3C /r] VPMAXSB ymm1, ymm2, ymm3/m256
    VPMAXSB_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG EE /r] VPMAXSW ymm1, ymm2, ymm3/m256
    VPMAXSW_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.WIG 3D /r] VPMAXSD ymm1, ymm2, ymm3/m256
    VPMAXSD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.WIG 3C /r] VPMAXSB xmm1 {k1}{z}, xmm2, xmm3/m128
    VPMAXSB_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.WIG 3C /r] VPMAXSB ymm1 {k1}{z}, ymm2, ymm3/m256
    VPMAXSB_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.WIG 3C /r] VPMAXSB zmm1 {k1}{z}, zmm2, zmm3/m512
    VPMAXSB_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F.WIG EE /r] VPMAXSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPMAXSW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG EE /r] VPMAXSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPMAXSW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG EE /r] VPMAXSW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPMAXSW_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W0 3D /r] VPMAXSD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPMAXSD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 3D /r] VPMAXSD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPMAXSD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 3D /r] VPMAXSD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPMAXSD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 3D /r] VPMAXSQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPMAXSQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 3D /r] VPMAXSQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPMAXSQ_VdqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 3D /r] VPMAXSQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPMAXSQ_VdqqHdqqWdqq_E512,

    // [NP 0F DE /r] PMAXUB mm1, mm2/m64
    PMAXUB_PqQq,
    // [66 0F DE /r] PMAXUB xmm1, xmm2/m128
    PMAXUB_VdqWdq,
    // [66 0F 38 3E /r] PMAXUW xmm1, xmm2/m128
    PMAXUW_VdqWdq,
    // [VEX.128.66.0F.WIG DE /r] VPMAXUB xmm1, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VPMAXUB_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.WIG 3E /r] VPMAXUW xmm1, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VPMAXUW_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG DE /r] VPMAXUB ymm1, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VPMAXUB_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.WIG 3E /r] VPMAXUW ymm1, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VPMAXUW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG DE /r] VPMAXUB xmm1 {k1}{z}, xmm2, xmm3/m128
    VPMAXUB_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG DE /r] VPMAXUB ymm1 {k1}{z}, ymm2, ymm3/m256
    VPMAXUB_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG DE /r] VPMAXUB zmm1 {k1}{z}, zmm2, zmm3/m512
    VPMAXUB_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.WIG 3E /r] VPMAXUW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPMAXUW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.WIG 3E /r] VPMAXUW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPMAXUW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.WIG 3E /r] VPMAXUW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPMAXUW_VdqqHdqqWdqq_E512,

    // [66 0F 38 3F /r] PMAXUD xmm1, xmm2/m128
    PMAXUD_VdqWdq,
    // [VEX.128.66.0F38.WIG 3F /r] VPMAXUD xmm1, xmm2, xmm3/m128
    VPMAXUD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 3F /r] VPMAXUD ymm1, ymm2, ymm3/m256
    VPMAXUD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W0 3F /r] VPMAXUD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPMAXUD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 3F /r] VPMAXUD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPMAXUD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 3F /r] VPMAXUD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPMAXUD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 3F /r] VPMAXUQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPMAXUQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 3F /r] VPMAXUQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPMAXUQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 3F /r] VPMAXUQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPMAXUQ_VdqqHdqqWdqq_E512,

    // [NP 0F EA /r] PMINSW mm1, mm2/m64
    PMINSW_PqQq,
    // [66 0F 38 38 /r] PMINSB xmm1, xmm2/m128
    PMINSB_VdqWdq,
    // [66 0F EA /r] PMINSW xmm1, xmm2/m128
    PMINSW_VdqWdq,
    // [VEX.128.66.0F38.WIG 38 /r] VPMINSB xmm1, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VPMINSB_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG EA /r] VPMINSW xmm1, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VPMINSW_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 38 /r] VPMINSB ymm1, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VPMINSB_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG EA /r] VPMINSW ymm1, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VPMINSW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.WIG 38 /r] VPMINSB xmm1 {k1}{z}, xmm2, xmm3/m128
    VPMINSB_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.WIG 38 /r] VPMINSB ymm1 {k1}{z}, ymm2, ymm3/m256
    VPMINSB_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.WIG 38 /r] VPMINSB zmm1 {k1}{z}, zmm2, zmm3/m512
    VPMINSB_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F.WIG EA /r] VPMINSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPMINSW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG EA /r] VPMINSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPMINSW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG EA /r] VPMINSW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPMINSW_VdqqHdqqWdqq_E512,

    // [66 0F 38 39 /r] PMINSD xmm1, xmm2/m128
    PMINSD_VdqWdq,
    // [VEX.128.66.0F38.WIG 39 /r] VPMINSD xmm1, xmm2, xmm3/m128
    VPMINSD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 39 /r] VPMINSD ymm1, ymm2, ymm3/m256
    VPMINSD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W0 39 /r] VPMINSD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPMINSD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 39 /r] VPMINSD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPMINSD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 39 /r] VPMINSD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPMINSD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 39 /r] VPMINSQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPMINSQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 39 /r] VPMINSQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPMINSQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 39 /r] VPMINSQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPMINSQ_VdqqHdqqWdqq_E512,

    // [NP 0F DA /r] PMINUB mm1, mm2/m64
    PMINUB_PqQq,
    // [66 0F DA /r] PMINUB xmm1, xmm2/m128
    PMINUB_VdqWdq,
    // [66 0F 38 3A /r] PMINUW xmm1, xmm2/m128
    PMINUW_VdqWdq,
    // [VEX.128.66.0F.WIG DA /r] VPMINUB xmm1, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VPMINUB_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.WIG 3A/r] VPMINUW xmm1, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VPMINUW_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG DA /r] VPMINUB ymm1, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VPMINUB_VdqHdqWdq_V256,
    // [VEX.256.66.0F38.WIG 3A /r] VPMINUW ymm1, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VPMINUW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG DA /r] VPMINUB xmm1 {k1}{z}, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VPMINUB_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG DA /r] VPMINUB ymm1 {k1}{z}, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VPMINUB_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG DA /r] VPMINUB zmm1 {k1}{z}, zmm2, zmm3/m512
    // NOTE: Intel manual doesn't mention `WIG`
    VPMINUB_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.WIG 3A /r] VPMINUW xmm1 {k1}{z}, xmm2, xmm3/m128
    // NOTE: Intel manual doesn't mention `WIG`
    VPMINUW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.WIG 3A /r] VPMINUW ymm1 {k1}{z}, ymm2, ymm3/m256
    // NOTE: Intel manual doesn't mention `WIG`
    VPMINUW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.WIG 3A /r] VPMINUW zmm1 {k1}{z}, zmm2, zmm3/m512
    // NOTE: Intel manual doesn't mention `WIG`
    VPMINUW_VdqqHdqqWdqq_E512,

    // [66 0F 38 3B /r] PMINUD xmm1, xmm2/m128
    PMINUD_VdqWdq,
    // [VEX.128.66.0F38.WIG 3B /r] VPMINUD xmm1, xmm2, xmm3/m128
    VPMINUD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 3B /r] VPMINUD ymm1, ymm2, ymm3/m256
    VPMINUD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W0 3B /r] VPMINUD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPMINUD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 3B /r] VPMINUD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPMINUD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 3B /r] VPMINUD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPMINUD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 3B /r] VPMINUQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPMINUQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 3B /r] VPMINUQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPMINUQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 3B /r] VPMINUQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPMINUQ_VdqqHdqqWdqq_E512,

    // [NP 0F D7 /r] PMOVMSKB reg, mm1
    PMOVMSKB_GdNq,
    // [66 0F D7 /r] PMOVMSKB reg, xmm1
    PMOVMSKB_GdUdq,
    // [VEX.128.66.0F.WIG D7 /r] VPMOVMSKB reg, xmm1
    VPMOVMSKB_GdUdq_V128,
    // [VEX.256.66.0F.WIG D7 /r] VPMOVMSKB reg, ymm1
    VPMOVMSKB_GdUqq_V256,

    // [66 0F 38 20 /r] PMOVSXBW xmm1, xmm2/m64
    PMOVSXBW_VdqWq,
    // [66 0F 38 21 /r] PMOVSXBD xmm1, xmm2/m32
    PMOVSXBD_VdqWd,
    // [66 0F 38 22 /r] PMOVSXBQ xmm1, xmm2/m16
    PMOVSXBQ_VdqWw,
    // [66 0F 38 23 /r] PMOVSXWD xmm1, xmm2/m64
    PMOVSXWD_VdqWq,
    // [66 0F 38 24 /r] PMOVSXWQ xmm1, xmm2/m32
    PMOVSXWQ_VdqWd,
    // [66 0F 38 25 /r] PMOVSXDQ xmm1, xmm2/m64
    PMOVSXDQ_VdqWq,
    // [VEX.128.66.0F38.WIG 20 /r] VPMOVSXBW xmm1, xmm2/m64
    VPMOVSXBW_VdqWq_V128,
    // [VEX.128.66.0F38.WIG 21 /r] VPMOVSXBD xmm1, xmm2/m32
    VPMOVSXBD_VdqWd_V128,
    // [VEX.128.66.0F38.WIG 22 /r] VPMOVSXBQ xmm1, xmm2/m16
    VPMOVSXBQ_VdqWw_V128,
    // [VEX.128.66.0F38.WIG 23 /r] VPMOVSXWD xmm1, xmm2/m64
    VPMOVSXWD_VdqWq_V128,
    // [VEX.128.66.0F38.WIG 24 /r] VPMOVSXWQ xmm1, xmm2/m32
    VPMOVSXWQ_VdqWd_V128,
    // [VEX.128.66.0F38.WIG 25 /r] VPMOVSXDQ xmm1, xmm2/m64
    VPMOVSXDQ_VdqWq_V128,
    // [VEX.256.66.0F38.WIG 20 /r] VPMOVSXBW ymm1, xmm2/m128
    VPMOVSXBW_VqqWdq_V256,
    // [VEX.256.66.0F38.WIG 21 /r] VPMOVSXBD ymm1, ymm2/m64
    VPMOVSXBD_VqqWq_V256,
    // [VEX.256.66.0F38.WIG 22 /r] VPMOVSXBQ ymm1, ymm2/m32
    VPMOVSXBQ_VqqWd_V256,
    // [VEX.256.66.0F38.WIG 23 /r] VPMOVSXWD ymm1, ymm2/m128
    VPMOVSXWD_VqqWdq_V256,
    // [VEX.256.66.0F38.WIG 24 /r] VPMOVSXWQ ymm1, ymm2/m64
    VPMOVSXWQ_VqqWq_V256,
    // [VEX.256.66.0F38.WIG 25 /r] VPMOVSXDQ ymm1, ymm2/m128
    VPMOVSXDQ_VqqWdq_V256,
    // [EVEX.128.66.0F38.WIG 20 /r] VPMOVSXBW xmm1 {k1}{z}, xmm2/m64
    VPMOVSXBW_VdqWq_E128,
    // [EVEX.256.66.0F38.WIG 20 /r] VPMOVSXBW ymm1 {k1}{z}, xmm2/m128
    VPMOVSXBW_VqqWdq_E256,
    // [EVEX.512.66.0F38.WIG 20 /r] VPMOVSXBW zmm1 {k1}{z}, ymm2/m256
    VPMOVSXBW_VdqqWqq_E512,
    // [EVEX.128.66.0F38.WIG 21 /r] VPMOVSXBD xmm1 {k1}{z}, xmm2/m32
    VPMOVSXBD_VdqWd_E128,
    // [EVEX.256.66.0F38.WIG 21 /r] VPMOVSXBD ymm1 {k1}{z}, xmm2/m64
    VPMOVSXBD_VqqWq_E256,
    // [EVEX.512.66.0F38.WIG 21 /r] VPMOVSXBD zmm1 {k1}{z}, xmm2/m128
    VPMOVSXBD_VdqqWdq_E512,
    // [EVEX.128.66.0F38.WIG 22 /r] VPMOVSXBQ xmm1 {k1}{z}, xmm2/m16
    VPMOVSXBQ_VdqWw_E128,
    // [EVEX.256.66.0F38.WIG 22 /r] VPMOVSXBQ ymm1 {k1}{z}, xmm2/m32
    VPMOVSXBQ_VqqWd_E256,
    // [EVEX.512.66.0F38.WIG 22 /r] VPMOVSXBQ zmm1 {k1}{z}, xmm2/m64
    VPMOVSXBQ_VdqqWq_E512,
    // [EVEX.128.66.0F38.WIG 23 /r] VPMOVSXWD xmm1 {k1}{z}, xmm2/m64
    VPMOVSXWD_VdqWq_E128,
    // [EVEX.256.66.0F38.WIG 23 /r] VPMOVSXWD ymm1 {k1}{z}, xmm2/m128
    VPMOVSXWD_VqqWdq_E256,
    // [EVEX.512.66.0F38.WIG 23 /r] VPMOVSXWD zmm1 {k1}{z}, ymm2/m256
    VPMOVSXWD_VdqqWqq_E512,
    // [EVEX.128.66.0F38.WIG 24 /r] VPMOVSXWQ xmm1 {k1}{z}, xmm2/m32
    VPMOVSXWQ_VdqWd_E128,
    // [EVEX.256.66.0F38.WIG 24 /r] VPMOVSXWQ ymm1 {k1}{z}, xmm2/m64
    VPMOVSXWQ_VqqWq_E256,
    // [EVEX.512.66.0F38.WIG 24 /r] VPMOVSXWQ zmm1 {k1}{z}, xmm2/m128
    VPMOVSXWQ_VdqqWdq_E512,
    // [EVEX.128.66.0F38.W0 25 /r] VPMOVSXDQ xmm1 {k1}{z}, xmm2/m64
    VPMOVSXDQ_VdqWq_E128,
    // [EVEX.256.66.0F38.W0 25 /r] VPMOVSXDQ ymm1 {k1}{z}, xmm2/m128
    VPMOVSXDQ_VqqWdq_E256,
    // [EVEX.512.66.0F38.W0 25 /r] VPMOVSXDQ zmm1 {k1}{z}, ymm2/m256
    VPMOVSXDQ_VdqqWqq_E512,

    // [66 0F 38 30 /r] PMOVZXBW xmm1, xmm2/m64
    PMOVZXBW_VdqWq,
    // [66 0F 38 31 /r] PMOVZXBD xmm1, xmm2/m32
    PMOVZXBD_VdqWd,
    // [66 0F 38 32 /r] PMOVZXBQ xmm1, xmm2/m16
    PMOVZXBQ_VdqWw,
    // [66 0F 38 33 /r] PMOVZXWD xmm1, xmm2/m64
    PMOVZXWD_VdqWq,
    // [66 0F 38 34 /r] PMOVZXWQ xmm1, xmm2/m32
    PMOVZXWQ_VdqWd,
    // [66 0F 38 35 /r] PMOVZXDQ xmm1, xmm2/m64
    PMOVZXDQ_VdqWq,
    // [VEX.128.66.0F38.WIG 30 /r] VPMOVZXBW xmm1, xmm2/m64
    VPMOVZXBW_VdqWq_V128,
    // [VEX.128.66.0F38.WIG 31 /r] VPMOVZXBD xmm1, xmm2/m32
    VPMOVZXBD_VdqWd_V128,
    // [VEX.128.66.0F38.WIG 32 /r] VPMOVZXBQ xmm1, xmm2/m16
    VPMOVZXBQ_VdqWw_V128,
    // [VEX.128.66.0F38.WIG 33 /r] VPMOVZXWD xmm1, xmm2/m64
    VPMOVZXWD_VdqWq_V128,
    // [VEX.128.66.0F38.WIG 34 /r] VPMOVZXWQ xmm1, xmm2/m32
    VPMOVZXWQ_VdqWd_V128,
    // [VEX.128.66.0F38.WIG 35 /r] VPMOVZXDQ xmm1, xmm2/m64
    VPMOVZXDQ_VdqWq_V128,
    // [VEX.256.66.0F38.WIG 30 /r] VPMOVZXBW ymm1, ymm2/m128
    VPMOVZXBW_VqqWqq_V256,
    // [VEX.256.66.0F38.WIG 31 /r] VPMOVZXBD ymm1, ymm2/m64
    VPMOVZXBD_VqqWq_V256,
    // [VEX.256.66.0F38.WIG 32 /r] VPMOVZXBQ ymm1, ymm2/m32
    VPMOVZXBQ_VqqWd_V256,
    // [VEX.256.66.0F38.WIG 33 /r] VPMOVZXWD ymm1, ymm2/m128
    VPMOVZXWD_VqqWdq_V256,
    // [VEX.256.66.0F38.WIG 34 /r] VPMOVZXWQ ymm1, ymm2/m64
    VPMOVZXWQ_VqqWq_V256,
    // [VEX.256.66.0F38.WIG 35 /r] VPMOVZXDQ ymm1, ymm2/m128
    VPMOVZXDQ_VqqWdq_V256,
    // [EVEX.128.66.0F38.WIG 30 /r] VPMOVZXBW xmm1 {k1}{z}, xmm2/m64
    VPMOVZXBW_VdqWq_E128,
    // [EVEX.256.66.0F38.WIG 30 /r] VPMOVZXBW ymm1 {k1}{z}, xmm2/m128
    VPMOVZXBW_VdqWdq_E256,
    // [EVEX.512.66.0F38.WIG 30 /r] VPMOVZXBW zmm1 {k1}{z}, ymm2/m256
    VPMOVZXBW_VdqqWqq_E512,
    // [EVEX.128.66.0F38.WIG 31 /r] VPMOVZXBD xmm1 {k1}{z}, xmm2/m32
    VPMOVZXBD_VdqWd_E128,
    // [EVEX.256.66.0F38.WIG 31 /r] VPMOVZXBD ymm1 {k1}{z}, xmm2/m64
    VPMOVZXBD_VqqWq_E256,
    // [EVEX.512.66.0F38.WIG 31 /r] VPMOVZXBD zmm1 {k1}{z}, xmm2/m128
    VPMOVZXBD_VdqqWdq_E512,
    // [EVEX.128.66.0F38.WIG 32 /r] VPMOVZXBQ xmm1 {k1}{z}, xmm2/m16
    VPMOVZXBQ_VdqWw_E128,
    // [EVEX.256.66.0F38.WIG 32 /r] VPMOVZXBQ ymm1 {k1}{z}, xmm2/m32
    VPMOVZXBQ_VqqWd_E256,
    // [EVEX.512.66.0F38.WIG 32 /r] VPMOVZXBQ zmm1 {k1}{z}, xmm2/m64
    VPMOVZXBQ_VdqqWq_E512,
    // [EVEX.128.66.0F38.WIG 33 /r] VPMOVZXWD xmm1 {k1}{z}, xmm2/m64
    VPMOVZXWD_VdqWq_E128,
    // [EVEX.256.66.0F38.WIG 33 /r] VPMOVZXWD ymm1 {k1}{z}, xmm2/m128
    VPMOVZXWD_VqqWdq_E256,
    // [EVEX.512.66.0F38.WIG 33 /r] VPMOVZXWD zmm1 {k1}{z}, ymm2/m256
    VPMOVZXWD_VdqqWqq_E512,
    // [EVEX.128.66.0F38.WIG 34 /r] VPMOVZXWQ xmm1 {k1}{z}, xmm2/m32
    VPMOVZXWQ_VdqWd_E128,
    // [EVEX.256.66.0F38.WIG 34 /r] VPMOVZXWQ ymm1 {k1}{z}, xmm2/m64
    VPMOVZXWQ_VqqWq_E256,
    // [EVEX.512.66.0F38.WIG 34 /r] VPMOVZXWQ zmm1 {k1}{z}, xmm2/m128
    VPMOVZXWQ_VdqqWdq_E512,
    // [EVEX.128.66.0F38.W0 35 /r] VPMOVZXDQ xmm1 {k1}{z}, xmm2/m64
    VPMOVZXDQ_VdqWq_E128,
    // [EVEX.256.66.0F38.W0 35 /r] VPMOVZXDQ ymm1 {k1}{z}, xmm2/m128
    VPMOVZXDQ_VqqWdq_E256,
    // [EVEX.512.66.0F38.W0 35 /r] VPMOVZXDQ zmm1 {k1}{z}, ymm2/m256
    VPMOVZXDQ_VdqqWqq_E512,

    // [66 0F 38 28 /r] PMULDQ xmm1, xmm2/m128
    PMULDQ_VdqWdq,
    // [VEX.128.66.0F38.WIG 28 /r] VPMULDQ xmm1, xmm2, xmm3/m128
    VPMULDQ_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 28 /r] VPMULDQ ymm1, ymm2, ymm3/m256
    VPMULDQ_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W1 28 /r] VPMULDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPMULDQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 28 /r] VPMULDQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPMULDQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 28 /r] VPMULDQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPMULDQ_VdqqHdqqWdqq_E512,

    // [NP 0F 38 0B /r] PMULHRSW mm1, mm2/m64
    PMULHRSW_PqQq,
    // [66 0F 38 0B /r] PMULHRSW xmm1, xmm2/m128
    PMULHRSW_VdqWdq,
    // [VEX.128.66.0F38.WIG 0B /r] VPMULHRSW xmm1, xmm2, xmm3/m128
    VPMULHRSW_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 0B /r] VPMULHRSW ymm1, ymm2, ymm3/m256
    VPMULHRSW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.WIG 0B /r] VPMULHRSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPMULHRSW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.WIG 0B /r] VPMULHRSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPMULHRSW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.WIG 0B /r] VPMULHRSW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPMULHRSW_VdqqHdqqWdqq_E512,

    // [NP 0F E4 /r] PMULHUW mm1, mm2/m64
    PMULHUW_PqQq,
    // [66 0F E4 /r] PMULHUW xmm1, xmm2/m128
    PMULHUW_VdqWdq,
    // [VEX.128.66.0F.WIG E4 /r] VPMULHUW xmm1, xmm2, xmm3/m128
    VPMULHUW_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG E4 /r] VPMULHUW ymm1, ymm2, ymm3/m256
    VPMULHUW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG E4 /r] VPMULHUW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPMULHUW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG E4 /r] VPMULHUW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPMULHUW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG E4 /r] VPMULHUW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPMULHUW_VdqqHdqqWdqq_E512,

    // [NP 0F E5 /r] PMULHW mm1, mm2/m64
    PMULHW_PqQq,
    // [66 0F E5 /r] PMULHW xmm1, xmm2/m128
    PMULHW_VdqWdq,
    // [VEX.128.66.0F.WIG E5 /r] VPMULHW xmm1, xmm2, xmm3/m128
    VPMULHW_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG E5 /r] VPMULHW ymm1, ymm2, ymm3/m256
    VPMULHW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG E5 /r] VPMULHW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPMULHW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG E5 /r] VPMULHW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPMULHW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG E5 /r] VPMULHW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPMULHW_VdqqHdqqWdqq_E512,

    // [66 0F 38 40 /r] PMULLD xmm1, xmm2/m128
    PMULLD_VdqWdq,
    // [VEX.128.66.0F38.WIG 40 /r] VPMULLD xmm1, xmm2, xmm3/m128
    VPMULLD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 40 /r] VPMULLD ymm1, ymm2, ymm3/m256
    VPMULLD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W0 40 /r] VPMULLD xmm1, xmm2, xmm3/m128/m32bcst
    VPMULLD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 40 /r] VPMULLD ymm1, ymm2, ymm3/m256/m32bcst
    VPMULLD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 40 /r] VPMULLD zmm1, zmm2, zmm3/m512/m32bcst
    VPMULLD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 40 /r] VPMULLD xmm1, xmm2, xmm3/m128/m64bcst
    VPMULLQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 40 /r] VPMULLD ymm1, ymm2, ymm3/m256/m64bcst
    VPMULLQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 40 /r] VPMULLD zmm1, zmm2, zmm3/m512/m64bcst
    VPMULLQ_VdqqHdqqWdqq_E512,

    // [NP 0F D5 /r] PMULLW mm1, mm/m64
    PMULLW_PqQq,
    // [66 0F D5 /r] PMULLW xmm1, xmm2/m128
    PMULLW_VdqWdq,
    // [VEX.128.66.0F.WIG D5 /r] VPMULLW xmm1, xmm2, xmm3/m128
    VPMULLW_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG D5 /r] VPMULLW ymm1, ymm2, ymm3/m256
    VPMULLW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG D5 /r] VPMULLW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPMULLW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG D5 /r] VPMULLW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPMULLW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG D5 /r] VPMULLW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPMULLW_VdqqHdqqWdqq_E512,

    // [NP 0F F4 /r] PMULUDQ mm1, mm2/m64
    PMULUDQ_PqQq,
    // [66 0F F4 /r] PMULUDQ xmm1, xmm2/m128
    PMULUDQ_VdqWdq,
    // [VEX.128.66.0F.WIG F4 /r] VPMULUDQ xmm1, xmm2, xmm3/m128
    VPMULUDQ_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG F4 /r] VPMULUDQ ymm1, ymm2, ymm3/m256
    VPMULUDQ_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W1 F4 /r] VPMULUDQ xmm1, xmm2, xmm3/m128/m64bcst
    VPMULUDQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 F4 /r] VPMULUDQ ymm1, ymm2, ymm3/m256/m64bcst
    VPMULUDQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 F4 /r] VPMULUDQ zmm1, zmm2, zmm3/m512/m64bcst
    VPMULUDQ_VdqqHdqqWdqq_E512,

    // [8F /0] POP r/m16
    POP_Ew,
    // [8F /0] POP r/m32
    POP_Ed,
    // [8F /0] POP r/m64
    POP_Eq,
    // [58+rw] POP r16
    POP_Gw,
    // [58+rd] POP r32
    POP_Gd,
    // [REX.W 58+rq] POP r64
    // NOTE: Intel manual doesn't mention REX.W
    POP_Gq,
    // [0F] POP CS
    // NOTE: Not valid on 80186 or newer
    POP_CS,
    // [1F] POP DS
    POP_DS,
    // [17] POP SS
    POP_SS,
    // [OF A1] POP FS
    POP_FS,
    // [0F A9] POP GS
    POP_GS,

    // [61] POPA
    POPA,
    // [61] POPAD
    POPAD,

    // [F3 0F B8 /r] POPCNT r16, r/m16
    POPCNT_EwGw,
    // [F3 0F B8 /r] POPCNT r32, r/m32
    POPCNT_EdGd,
    // [F3 REX.W 0F B8 /r] POPCNT r64, r/m64
    POPCNT_EqGq,

    // [9D] POPF
    POPF,
    // [9D] POPFD
    POPFD,
    // [9D] POPFQ
    POPFQ,

    // [NP 0F EB /r] POR mm1, mm2/m64
    POR_PqQq,
    // [66 0F EB /r] POR xmm1, xmm2/m128
    POR_VdqWdq,
    // [VEX.128.66.0F.WIG EB /r] VPOR xmm1, xmm2, xmm3/m128
    VPOR_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG EB /r] VPOR ymm1, ymm2, ymm3/m256
    VPOR_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W0 EB /r] VPORD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPORD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W0 EB /r] VPORD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPORD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W0 EB /r] VPORD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPORD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F.W1 EB /r] VPORQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPORQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 EB /r] VPORQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPORQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 EB /r] VPORQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPORQ_VdqqHdqqWdqq_E512,

    // [0F 18 /1] PREFETCH0 m8
    PREFETCH0_Mb,
    // [0F 18 /2] PREFETCH1 m8
    PREFETCH1_Mb,
    // [0F 18 /3] PREFETCH2 m8
    PREFETCH2_Mb,
    // [0F 18 /0] PREFETCHNTA m8
    PREFETCHNTA_Mb,

    // [0F 0D /1] PREFETCHW m8
    PREFETCHW_Mb,

    // [NP 0F F6 /r] PSADBW mm1, mm2/m64
    PSADBW_PqQq,
    // [66 0F F6 /r] PSADBW xmm1, xmm2/m128
    PSADBW_VdqWdq,
    // [VEX.128.66.0F.WIG F6 /r] VPSADBW xmm1, xmm2, xmm3/m128
    VPSADBW_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG F6 /r] VPSADBW ymm1, ymm2, ymm3/m256
    VPSADBW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG F6 /r] VPSADBW xmm1, xmm2, xmm3/m128
    VPSADBW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG F6 /r] VPSADBW ymm1, ymm2, ymm3/m256
    VPSADBW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG F6 /r] VPSADBW zmm1, zmm2, zmm3/m512
    VPSADBW_VdqqHdqqWdqq_E512,

    // [NP 0F 38 00 /r] PSHUFB mm1, mm2/m64
    PSHUFB_PqQq,
    // [66 0F 38 00 /r] PSHUFB xmm1, xmm2/m128
    PSHUFB_VdqWdq,
    // [VEX.128.66.0F38.WIG 00 /r] VPSHUFB xmm1, xmm2, xmm3/m128
    VPSHUFB_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 00 /r] VPSHUFB ymm1, ymm2, ymm3/m256
    VPSHUFB_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.WIG 00 /r] VPSHUFB xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSHUFB_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.WIG 00 /r] VPSHUFB xmm1 {k1}{z}, xmm2, xmm3/m256
    VPSHUFB_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.WIG 00 /r] VPSHUFB xmm1 {k1}{z}, xmm2, xmm3/m512
    VPSHUFB_VdqqHdqqWdqq_E512,

    // [66 0F 70 /r ib] PSHUFD xmm1, xmm2/m128, imm8
    PSHUFD_VdqWdqIb,
    // [VEX.128.66.0F.WIG 70 /r ib] VPSHUFD xmm1, xmm2/m128, imm8
    VPSHUFD_VdqWdqIb_V128,
    // [VEX.256.66.0F.WIG 70 /r ib] VPSHUFD ymm1, ymm2/m256, imm8
    VPSHUFD_VqqWqqIb_V256,
    // [EVEX.128.66.0F.WIG 70 /r ib] VPSHUFD xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VPSHUFD_VdqWdqIb_E128,
    // [EVEX.256.66.0F.WIG 70 /r ib] VPSHUFD ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VPSHUFD_VqqWqqIb_E256,
    // [EVEX.512.66.0F.WIG 70 /r ib] VPSHUFD zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VPSHUFD_VdqqWdqqIb_E512,

    // [F3 0F 70 /r ib] PSHUFHW xmm1, xmm2/m128, imm8
    PSHUFHW_VdqWdqIb,
    // [VEX.128.F3.0F.WIG 70 /r ib] VPSHUFHW xmm1, xmm2/m128, imm8
    VPSHUFHW_VdqWdqIb_V128,
    // [VEX.256.F3.0F.WIG 70 /r ib] VPSHUFHW ymm1, ymm2/m256, imm8
    VPSHUFHW_VqqWqqIb_V256,
    // [EVEX.128.F3.0F.WIG 70 /r ib] VPSHUFHW xmm1 {k1}{z}, xmm2/m128, imm8
    VPSHUFHW_VdqWdqIb_E128,
    // [EVEX.256.F3.0F.WIG 70 /r ib] VPSHUFHW ymm1 {k1}{z}, ymm2/m256, imm8
    VPSHUFHW_VqqWqqIb_E256,
    // [EVEX.512.F3.0F.WIG 70 /r ib] VPSHUFHW zmm1 {k1}{z}, zmm2/m512, imm8
    VPSHUFHW_VdqqWdqqIb_E512,

    // [F2 0F 70 /r ib] PSHUFLW xmm1, xmm2/m128, imm8
    PSHUFLW_VdqWdqIb,
    // [VEX.128.F2.0F.WIG 70 /r ib] VPSHUFLW xmm1, xmm2/m128, imm8
    VPSHUFLW_VdqWdqIb_V128,
    // [VEX.256.F2.0F.WIG 70 /r ib] VPSHUFLW ymm1, ymm2/m256, imm8
    VPSHUFLW_VqqWqqIb_V256,
    // [EVEX.128.F2.0F.WIG 70 /r ib] VPSHUFLW xmm1 {k1}{z}, xmm2/m128, imm8
    VPSHUFLW_VdqWdqIb_E128,
    // [EVEX.256.F2.0F.WIG 70 /r ib] VPSHUFLW ymm1 {k1}{z}, ymm2/m256, imm8
    VPSHUFLW_VqqWqqIb_E256,
    // [EVEX.512.F2.0F.WIG 70 /r ib] VPSHUFLW zmm1 {k1}{z}, zmm2/m512, imm8
    VPSHUFLW_VdqqWdqqIb_E512,

    // [NP 0F 70 /r ib] PSHUFW mm1, mm2/m64, imm8
    PSHUFW_PqQqIb,

    // [NP 0F 38 08 /r] PSIGNB mm1, mm2/m64
    PSIGNB_PqQq,
    // [66 0F 38 08 /r] PSIGNB xmm1, xmm2/m128
    PSIGNB_VdqWdq,
    // [NP 0F 38 09 /r] PSIGNW mm1, mm2/m64
    PSIGNW_PqQq,
    // [66 0F 38 09 /r] PSIGNW xmm1, xmm2/m128
    PSIGNW_VdqWdq,
    // [NP 0F 38 0A /r] PSIGND mm1, mm2/m64
    PSIGND_PqQq,
    // [66 0F 38 0A /r] PSIGND xmm1, xmm2/m128
    PSIGND_VdqWdq,
    // [VEX.128.66.0F38.WIG 08 /r] VPSIGNB xmm1, xmm2, xmm3/m128
    VPSIGNB_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.WIG 09 /r] VPSIGNW xmm1, xmm2, xmm3/m128
    VPSIGNW_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.WIG 0A /r] VPSIGND xmm1, xmm2, xmm3/m128
    VPSIGND_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.WIG 08 /r] VPSIGNB ymm1, ymm2, ymm3/m256
    VPSIGNB_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.WIG 09 /r] VPSIGNW ymm1, ymm2, ymm3/m256
    VPSIGNW_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.WIG 0A /r] VPSIGND ymm1, ymm2, ymm3/m256
    VPSIGND_VqqHqqWqq_V256,

    // [66 0F 73 /7 ib] PSLLDQ xmm1, imm8
    PSLLDQ_UdqIb,
    // [VEX.128.66.0F.WIG 73 /7 ib] VPSLLDQ xmm1, xmm2, imm8
    VPSLLDQ_HdqUdqIb_V128,
    // [VEX.256.66.0F.WIG 73 /7 ib] VPSLLDQ ymm1, ymm2, imm8
    VPSLLDQ_HqqUqqIb_V256,
    // [EVEX.128.66.0F.WIG 73 /7 ib] VPSLLDQ xmm1, xmm2/m128, imm8
    VPSLLDQ_HdqUdqIb_E128,
    // [EVEX.256.66.0F.WIG 73 /7 ib] VPSLLDQ ymm1, ymm2/m256, imm8
    VPSLLDQ_HqqUqqIb_E256,
    // [EVEX.512.66.0F.WIG 73 /7 ib] VPSLLDQ zmm1, zmm2/m512, imm8
    VPSLLDQ_HdqqUdqqIb_E512,

    // [NP 0F F1 /r] PSLLW mm1, mm2/m64
    PSLLW_PqQq,
    // [66 0F F1 /r] PSLLW xmm1, xmm2/m128
    PSLLW_VdqWdq,
    // [NP 0F 71 /6 ib] PSLLW mm, imm8
    PSLLW_NqIb,
    // [66 0F 71 /6 ib] PSLLW xmm1, xmm2/m128
    PSLLW_UdqIb,
    // [NP 0F F2 /r] PSLLD mm, mm2/m64
    PSLLD_PqQq,
    // [66 0F F2 /r] PSLLD xmm1, xmm2/m128
    PSLLD_VdqWdq,
    // [NP 0F 72 /6 ib] PSLLD mm, imm8
    PSLLD_NqIb,
    // [66 0F 72 /6 ib] PSLLD xmm, imm8
    PSLLD_UdqIb,
    // [NP 0F F3 /r] PSLLQ mm1, mm2/m64
    PSLLQ_PqQq,
    // [66 0F F3 /r] PSLLQ xmm1, xmm2/m128
    PSLLQ_VdqWdq,
    // [VEX.128.66.0F.WIG F1 /r] VPSLLW xmm1, xmm2, xmm3/m128
    VPSLLW_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 71 /6 ib] VPSLLW xmm1, xmm2, imm8
    VPSLLW_HdqUdqIb_V128,
    // [VEX.128.66.0F.WIG F2 /r] VPSLLD xmm1, xmm2, xmm3/m128
    VPSLLD_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 72 /6 ib] VPSLLD xmm1, xmm2, imm8
    VPSLLD_HdqUdqIb_V128,
    // [VEX.128.66.0F.WIG F3 /r] VPSLLQ xmm1, xmm2, xmm3/m128
    VPSLLQ_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 73 /6 ib] VPSLLQ xmm1, xmm2, imm8
    VPSLLQ_HdqUdqIb_V128,
    // [VEX.256.66.0F.WIG F1 /r] VPSLLW ymm1, ymm2, ymm3/m128
    VPSLLW_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG 71 /6 ib] VPSLLW ymm1, ymm2, imm8
    VPSLLW_HqqUqqIb_V256,
    // [VEX.256.66.0F.WIG F2 /r] VPSLLD ymm1, ymm2, ymm3/m128
    VPSLLD_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG 72 /6 ib] VPSLLD ymm1, ymm2, imm8
    VPSLLD_HqqUqqIb_V256,
    // [VEX.256.66.0F.WIG F3 /r] VPSLLQ ymm1, ymm2, ymm3/m128
    VPSLLQ_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG 73 /6 ib] VPSLLQ ymm1, ymm2, imm8
    VPSLLQ_HqqUqqIb_V256,
    // [EVEX.128.66.0F.WIG F1 /r] VPSLLW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSLLW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG F1 /r] VPSLLW ymm1 {k1}{z}, ymm2, xmm3/m128
    VPSLLW_VqqHqqWdq_E256,
    // [EVEX.256.66.0F.WIG F1 /r] VPSLLW ymm1 {k1}{z}, ymm2, xmm3/m128
    VPSLLW_VdqqHdqqWdq_E512,
    // [EVEX.128.66.0F.WIG 71 /6 ib] VPSLLW xmm1 {k1}{z}, xmm2/m128, imm8
    VPSLLW_HdqUdqIb_E128,
    // [EVEX.256.66.0F.WIG 71 /6 ib] VPSLLW ymm1 {k1}{z}, ymm2/m256, imm8
    VPSLLW_HqqUqqIb_E256,
    // [EVEX.512.66.0F.WIG 71 /6 ib] VPSLLW zmm1 {k1}{z}, zmm2/m512, imm8
    VPSLLW_HdqqUdqqIb_E512,
    // [EVEX.128.66.0F.W0 F2 /r] VPSLLD xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSLLD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W0 F2 /r] VPSLLD ymm1 {k1}{z}, ymm2, xmm3/m128
    VPSLLD_VqqHqqWdq_E256,
    // [EVEX.512.66.0F.W0 F2 /r] VPSLLD zmm1 {k1}{z}, zmm2, xmm3/m128
    VPSLLD_VdqqHdqqWdq_E512,
    // [EVEX.128.66.0F.W0 72 /6 ib] VPSLLD xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VPSLLD_HdqUdqIb_E128,
    // [EVEX.256.66.0F.W0 72 /6 ib] VPSLLD ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VPSLLD_HqqUqqIb_E256,
    // [EVEX.512.66.0F.W0 72 /6 ib] VPSLLD zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VPSLLD_HdqqUdqqIb_E512,
    // [EVEX.128.66.0F.W1 F3 /r] VPSLLQ xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSLLQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 F3 /r] VPSLLQ ymm1 {k1}{z}, ymm2, xmm3/m128
    VPSLLQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 F3 /r] VPSLLQ zmm1 {k1}{z}, zmm2, xmm3/m128
    VPSLLQ_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F.W1 73 /6 ib] VPSLLQ xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VPSLLQ_HdqUdqIb_E128,
    // [EVEX.256.66.0F.W1 73 /6 ib] VPSLLQ ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VPSLLQ_HqqUqqIb_E256,
    // [EVEX.512.66.0F.W1 73 /6 ib] VPSLLQ zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VPSLLQ_HdqqUdqqIb_E512,

    // [NP 0F E1 /r] PSRAW mm1, mm2/m64
    PSRAW_PqQq,
    // [66 0F E1 /r] PSRAW xmm1, xmm2/m128
    PSRAW_VdqWdq,
    // [NP 0F 71 /4 ib] PSRAW mm, imm8
    PSRAW_NqIb,
    // [66 0F 71 /4 ib] PSRAW xmm, imm8
    PSRAW_UdqIb,
    // [NP 0F E2 /r] PSRAD mm1, mm2/m64
    PSRAD_PqQq,
    // [66 0F E2 /r] PSRAD xmm1, xmm2/m128
    PSRAD_VdqWdq,
    // [NP 0F 72 /4 ib] PSRAD mm, imm8
    PSRAD_NqIb,
    // [66 0F 72 /4 ib] PSRAD xmm, imm8
    PSRAD_UdqIb,
    // [VEX.128.66.0F.WIG E1 /r] VPSRAW xmm1, xmm2, xmm3/m128
    VPSRAW_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 71 /4 ib] VPSRAW xmm1, xmm2, imm8
    VPSRAW_HdqUdqIb_V128,
    // [VEX.128.66.0F.WIG E2 /r] VPSRAD xmm1, xmm2, xmm3/m128
    VPSRAD_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 72 /4 ib] VPSRAD xmm1, xmm2, imm8
    VPSRAD_HdqUdqIb_V128,
    // [VEX.256.66.0F.WIG E1 /r] VPSRAW ymm1, ymm2, xmm3/m128
    VPSRAW_VqqHqqWdq_V256,
    // [VEX.256.66.0F.WIG 71 /4 ib] VPSRAW ymm1, ymm2, imm8
    VPSRAW_HqqUqqIb_V256,
    // [VEX.256.66.0F.WIG E2 /r] VPSRAD ymm1, ymm2, xmm3/m128
    VPSRAD_VqqHqqWdq_V256,
    // [VEX.256.66.0F.WIG 72 /4 ib] VPSRAD ymm1, ymm2, imm8
    VPSRAD_HqqUqqIb_V256,
    // [EVEX.128.66.0F.WIG E1 /r] VPSRAW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSRAW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG E1 /r] VPSRAW ymm1 {k1}{z}, ymm2, xmm3/m128
    VPSRAW_VqqHqqWdq_E256,
    // [EVEX.512.66.0F.WIG E1 /r] VPSRAW zmm1 {k1}{z}, zmm2, xmm3/m128
    VPSRAW_VdqqHdqqWdq_E512,
    // [EVEX.128.66.0F.WIG 71 /4 ib] VPSRAW xmm1 {k1}{z}, xmm2/m128, imm8
    VPSRAW_HdqUdqIb_E128,
    // [EVEX.256.66.0F.WIG 71 /4 ib] VPSRAW ymm1 {k1}{z}, ymm2/m128, imm8
    VPSRAW_HqqUqqIb_E256,
    // [EVEX.512.66.0F.WIG 71 /4 ib] VPSRAW zmm1 {k1}{z}, zmm2/m128, imm8
    VPSRAW_HdqqUdqqIb_E512,
    // [EVEX.128.66.0F.W0 E2 /r] VPSRAD xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSRAD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W0 E2 /r] VPSRAD ymm1 {k1}{z}, ymm2, xmm3/m128
    VPSRAD_VqqHqqWdq_E256,
    // [EVEX.512.66.0F.W0 E2 /r] VPSRAD zmm1 {k1}{z}, zmm2, xmm3/m128
    VPSRAD_VdqqHdqqWdq_E512,
    // [EVEX.128.66.0F.W0 72 /4 ib] VPSRAD xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VPSRAD_HdqUdqIb_E128,
    // [EVEX.256.66.0F.W0 72 /4 ib] VPSRAD ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VPSRAD_HqqUqqIb_E256,
    // [EVEX.512.66.0F.W0 72 /4 ib] VPSRAD zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VPSRAD_HdqqUdqqIb_E512,
    // [EVEX.128.66.0F.W1 E2 /r] VPSRAQ xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSRAQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 E2 /r] VPSRAQ ymm1 {k1}{z}, ymm2, xmm3/m128
    VPSRAQ_VqqHqqWdq_E256,
    // [EVEX.512.66.0F.W1 E2 /r] VPSRAQ zmm1 {k1}{z}, zmm2, xmm3/m128
    VPSRAQ_VdqqHdqqWdq_E512,
    // [EVEX.128.66.0F.W1 72 /4 ib] VPSRAQ xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VPSRAQ_HdqUdqIb_E128,
    // [EVEX.256.66.0F.W1 72 /4 ib] VPSRAQ ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VPSRAQ_HqqUqqIb_E256,
    // [EVEX.512.66.0F.W1 72 /4 ib] VPSRAQ zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VPSRAQ_HdqqUdqqIb_E512,

    // [66 0F 73 /3 ib] PSRLDQ xmm1, imm8
    PSRLDQ_UdqIb,
    // [VEX.128.66.0F.WIG 73 /3 ib] VPSRLDQ xmm1, xmm2, imm8
    VPSRLDQ_HdqUdqIb_V128,
    // [VEX.256.66.0F.WIG 73 /3 ib] VPSRLDQ ymm1, ymm2, imm8
    VPSRLDQ_HqqUqqIb_V256,
    // [EVEX.128.66.0F.WIG 73 /3 ib] VPSRLDQ xmm1, xmm2/m128, imm8
    VPSRLDQ_HdqUdqIb_E128,
    // [EVEX.256.66.0F.WIG 73 /3 ib] VPSRLDQ ymm1, ymm2/m256, imm8
    VPSRLDQ_HqqUqqIb_E256,
    // [EVEX.512.66.0F.WIG 73 /3 ib] VPSRLDQ zmm1, zmm2/m512, imm8
    VPSRLDQ_HdqqUdqqIb_E512,

    // [NP 0F D1 /r] PSRLW mm1, mm2/m64
    PSRLW_PqQq,
    // [66 0F D1 /r] PSRLW xmm1, xmm2/m128
    PSRLW_VdqWdq,
    // [NP 0F 71 /2 ib] PSRLW mm, imm8
    PSRLW_NqIb,
    // [66 0F 71 /2 ib] PSRLW xmm1, imm8
    PSRLW_UdqIb,
    // [NP 0F D2 /r] PSRLD mm1, mm2/m64
    PSRLD_PqQq,
    // [66 0F D2 /r] PSRLD xmm1, xmm2/m128
    PSRLD_VdqWdq,
    // [NP 0F 72 /2 ib] PSRLD mm, imm8
    PSRLD_NqIb,
    // [66 0F 72 /2 ib] PSRLD xmm, imm8
    PSRLD_UdqIb,
    // [NP 0F D3 /r] PSRLQ mm1, mm2/m64
    PSRLQ_PqQq,
    // [66 0F D3 /r] PSRLQ xmm1, xmm2/m128
    PSRLQ_VdqWdq,
    // [NP 0F 73 /2 ib] PSRLQ mm, imm8
    PSRLQ_NqIb,
    // [66 0F 73 /2 ib] PSRLQ xmm, imm8
    PSRLQ_UdqIb,
    // [VEX.128.66.0F.WIG D1 /r] VPSRLW xmm1, xmm2, xmm3/m128
    VPSRLW_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 71 /2 ib] VPSRLW xmm1, xmm2, imm8
    VPSRLW_HdqUdqIb_V128,
    // [VEX.128.66.0F.WIG D2 /r] VPSRLD xmm1, xmm2, xmm3/m128
    VPSRLD_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 72 /2 ib] VPSRLD xmm1, xmm2, imm8
    VPSRLD_HdqUdqIb_V128,
    // [VEX.128.66.0F.WIG D3 /r] VPSRLQ xmm1, xmm2, xmm3/m128
    VPSRLQ_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 73 /2 ib] VPSRLQ xmm1, xmm2, imm8
    VPSRLQ_HdqUdqIb_V128,
    // [VEX.256.66.0F.WIG D1 /r] VPSRLW ymm1, ymm2, xmm3/m128
    VPSRLW_VqqHqqWdq_V128,
    // [VEX.256.66.0F.WIG 71 /2 ib] VPSRLW ymm1, ymm2, imm8
    VPSRLW_HqqUdqIb_V128,
    // [VEX.256.66.0F.WIG D2 /r] VPSRLD ymm1, ymm2, xmm3/m128
    VPSRLD_VqqHqqWdq_V128,
    // [VEX.256.66.0F.WIG 72 /2 ib] VPSRLD ymm1, ymm2, imm8
    VPSRLD_HqqUdqIb_V128,
    // [VEX.256.66.0F.WIG D3 /r] VPSRLQ ymm1, ymm2, xmm3/m128
    VPSRLQ_VqqHqqWdq_V128,
    // [VEX.256.66.0F.WIG 73 /2 ib] VPSRLQ ymm1, ymm2, imm8
    VPSRLQ_HqqUdqIb_V128,
    // [EVEX.128.66.0F.WIG D1 /r] VPSRLW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSRLW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG D1 /r] VPSRLW ymm1 {k1}{z}, ymm2, xmm3/m128
    VPSRLW_VqqHqqWdq_E256,
    // [EVEX.512.66.0F.WIG D1 /r] VPSRLW zmm1 {k1}{z}, zmm2, xmm3/m128
    VPSRLW_VdqqHdqqWdq_E512,
    // [EVEX.128.66.0F.WIG 71 /2 ib] VPSRLW xmm1 {k1}{z}, xmm2/m128, imm8
    VPSRLW_HdqUdqIb_E128,
    // [EVEX.256.66.0F.WIG 71 /2 ib] VPSRLW ymm1 {k1}{z}, ymm2/m256, imm8
    VPSRLW_HdqUdqIb_E256,
    // [EVEX.512.66.0F.WIG 71 /2 ib] VPSRLW zmm1 {k1}{z}, zmm2/m512, imm8
    VPSRLW_HdqUdqIb_E512,
    // [EVEX.128.66.0F.WIG D2 /r] VPSRLD xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSRLD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG D2 /r] VPSRLD ymm1 {k1}{z}, ymm2, xmm3/m128
    VPSRLD_VqqHqqWdq_E256,
    // [EVEX.512.66.0F.WIG D2 /r] VPSRLD zmm1 {k1}{z}, zmm2, xmm3/m128
    VPSRLD_VdqqHdqqWdq_E512,
    // [EVEX.128.66.0F.WIG 72 /2 ib] VPSRLD xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VPSRLD_HdqUdqIb_E128,
    // [EVEX.256.66.0F.WIG 72 /2 ib] VPSRLD ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VPSRLD_HdqUdqIb_E256,
    // [EVEX.512.66.0F.WIG 72 /2 ib] VPSRLD zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VPSRLD_HdqUdqIb_E512,
    // [EVEX.128.66.0F.WIG D3 /r] VPSRLQ xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSRLQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG D3 /r] VPSRLQ ymm1 {k1}{z}, ymm2, xmm3/m128
    VPSRLQ_VqqHqqWdq_E256,
    // [EVEX.512.66.0F.WIG D3 /r] VPSRLQ zmm1 {k1}{z}, zmm2, xmm3/m128
    VPSRLQ_VdqqHdqqWdq_E512,
    // [EVEX.128.66.0F.WIG 73 /2 ib] VPSRLQ xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VPRSLW_HdqUdqIb_E128,
    // [EVEX.256.66.0F.WIG 73 /2 ib] VPSRLQ ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VPRSLW_HdqUdqIb_E256,
    // [EVEX.512.66.0F.WIG 73 /2 ib] VPSRLQ zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VPRSLW_HdqUdqIb_E512,

    // [NP 0F F8 /r] PSUBB mm1, mm2/m64
    PSUBB_PqQq,
    // [66 0F F8 /r] PSUBB xmm1, xmm2/m128
    PSUBB_VdqWdq,
    // [NP 0F F9 /r] PSUBW mm1, mm2/m64
    PSUBW_PqQq,
    // [66 0F F9 /r] PSUBW xmm1, xmm2/m128
    PSUBW_VdqWdq,
    // [NP 0F FA /r] PSUBD mm1, mm2/m64
    PSUBD_PqQq,
    // [66 0F FA /r] PSUBD xmm1, xmm2/m128
    PSUBD_VdqWdq,
    // [VEX.128.66.0F.WIG F8 /r] VSUBB xmm1, xmm2, xmm3/m128
    VPSUBB_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG F9 /r] VSUBW xmm1, xmm2, xmm3/m128
    VPSUBW_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG FA /r] VSUBD xmm1, xmm2, xmm3/m128
    VPSUBD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG F8 /r] VSUBB ymm1, ymm2, ymm3/m256
    VPSUBB_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG F9 /r] VSUBW ymm1, ymm2, ymm3/m256
    VPSUBW_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG FA /r] VSUBD ymm1, ymm2, ymm3/m256
    VPSUBD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG F8 /r] VSUBB xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSUBB_VdqHdqWdq_E128,
    // [EVEX.128.66.0F.WIG F9 /r] VSUBW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSUBW_VdqHdqWdq_E128,
    // [EVEX.128.66.0F.WIG FA /r] VSUBD xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSUBD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG F8 /r] VSUBB ymm1 {k1}{z}, ymm2, ymm3/m256
    VPSUBB_VqqHqqWqq_E256,
    // [EVEX.256.66.0F.WIG F9 /r] VSUBW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPSUBW_VqqHqqWqq_E256,
    // [EVEX.256.66.0F.WIG FA /r] VSUBD ymm1 {k1}{z}, ymm2, ymm3/m256
    VPSUBD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W0 F8 /r] VSUBB zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPSUBB_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F.W0 F9 /r] VSUBW zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPSUBW_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F.W0 FA /r] VSUBD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPSUBD_VdqqHdqqWdqq_E512,

    // [NP 0F FB /r] PSUBQ mm1, mm2/m64
    PSUBQ_PqQq,
    // [66 0F FB /r] PSUBQ xmm1, xmm2/m128
    PSUBQ_VdqWdq,
    // [VEX.128.66.0F.WIG FB /r] VPSUBQ xmm1, xmm2, xmm3/m128
    VPSUBQ_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG FB /r] VPSUBQ ymm1, ymm2, ymm3/m256
    VPSUBQ_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W1 FB /r] VPSUBQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPSUBQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 FB /r] VPSUBQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPSUBQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 FB /r] VPSUBQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPSUBQ_VdqqHdqqWdqq_E512,

    // [NP 0F E8 /r] PSUBSB mm1, mm2/m64
    PSUBSB_PqQq,
    // [66 0F E8 /r] PSUBSB xmm1, xmm2/m128
    PSUBSB_VdqWdq,
    // [NP 0F E9 /r] PSUBSW mm1, mm2/m64
    PSUBSW_PqQq,
    // [66 0F E9 /r] PSUBSW xmm1, xmm2/m128
    PSUBSW_VdqWdq,
    // [VEX.128.66.0F.WIG E8 /r] VPSUBSB xmm1, xmm2, xmm3/m128
    VPSUBSB_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG E9 /r] VPSUBSW xmm1, xmm2, xmm3/m128
    VPSUBSW_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG E8 /r] VPSUBSB ymm1, ymm2, ymm3/m256
    VPSUBSB_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG E9 /r] VPSUBSW ymm1, ymm2, ymm3/m256
    VPSUBSW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG E8 /r] VPSUBSB xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSUBSB_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG E8 /r] VPSUBSB ymm1 {k1}{z}, ymm2, ymm3/m256
    VPSUBSB_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG E8 /r] VPSUBSB zmm1 {k1}{z}, xmm2, zmm3/m512
    VPSUBSB_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F.WIG E9 /r] VPSUBSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSUBSW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG E9 /r] VPSUBSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPSUBSW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG E9 /r] VPSUBSW zmm1 {k1}{z}, xmm2, zmm3/m512
    VPSUBSW_VdqqHdqqWdqq_E512,

    // [NP 0F D8 /r] PSUBUSB mm1, mm2/m64
    PSUBUSB_PqQq,
    // [66 0F D8 /r] PSUBUSB xmm1, xmm2/m128
    PSUBUSB_VdqWdq,
    // [NP 0F D9 /r] PSUBUSW mm1, mm2/m64
    PSUBUSW_PqQq,
    // [66 0F D9 /r] PSUBUSW xmm1, xmm2/m128
    PSUBUSW_VdqWdq,
    // [VEX.128.66.0F.WIG D8 /r] VPSUBUSB xmm1, xmm2, xmm3/m128
    VPSUBUSB_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG D9 /r] VPSUBUSW xmm1, xmm2, xmm3/m128
    VPSUBUSW_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG D8 /r] VPSUBUSB ymm1, ymm2, ymm3/m256
    VPSUBUSB_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG D9 /r] VPSUBUSW ymm1, ymm2, ymm3/m256
    VPSUBUSW_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG D8 /r] VPSUBUSB xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSUBUSB_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG D8 /r] VPSUBUSB ymm1 {k1}{z}, ymm2, ymm3/m256
    VPSUBUSB_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG D8 /r] VPSUBUSB zmm1 {k1}{z}, xmm2, zmm3/m512
    VPSUBUSB_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F.WIG D9 /r] VPSUBUSW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSUBUSW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG D9 /r] VPSUBUSW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPSUBUSW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG D9 /r] VPSUBUSW zmm1 {k1}{z}, xmm2, zmm3/m512
    VPSUBUSW_VdqqHdqqWdqq_E512,

    // [66 0F 38 17 /r] PTEST xmm1, xmm2/m128
    PTEST_VdqWdq,
    // [VEX.128.66.0F38.WIG 17 /r] VPTEST xmm1, xmm2/m128
    VPTEST_VdqWdq_V128,
    // [VEX.256.66.0F38.WIG 17 /r] VPTEST ymm1, ymm2/m256
    VPTEST_VqqWqq_V256,

    // [F3 0F AE /4] PTWRITE r/m32
    PTWRITE_Ed,
    // [F3 REX.W 0F AE /4] PTWRITE r/m64
    PTWRITE_Eq,

    // [NP 0F 68 /r] PUNPCKHBW mm1, mm2/m64
    PUNPCKHBW_PqQq,
    // [66 0F 68 /r] PUNPCKHBW xmm1, xmm2/m128
    PUNPCKHBW_VdqWdq,
    // [NP 0F 69 /r] PUNPCKHWD mm1, mm2/m64
    PUNPCKHWD_PqQq,
    // [66 0F 69 /r] PUNPCKHWD xmm1, xmm2/m128
    PUNPCKHWD_VdqWdq,
    // [NP 0F 6A /r] PUNPCKHDQ mm1, mm2/m64
    PUNPCKHDQ_PqQq,
    // [66 0F 6A /r] PUNPCKHDQ xmm1, xmm2/m128
    PUNPCKHDQ_VdqWdq,
    // [66 0F 6D /r] PUNPCKHQDQ xmm1, xmm2/m128
    PUNPCKHQDQ_VdqWdq,
    // [VEX.128.66.0F.WIG 68 /r] VPUNPCKHBW xmm1, xmm2, xmm3/m128
    VPUNPCKHBW_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 69 /r] VPUNPCKHWD xmm1, xmm2, xmm3/m128
    VPUNPCKHWD_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 6A /r] VPUNPCKHDQ xmm1, xmm2, xmm3/m128
    VPUNPCKHDQ_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 6D /r] VPUNPCKHQDQ xmm1, xmm2, xmm3/m128
    VPUNPCKHQDQ_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 68 /r] VPUNPCKHBW ymm1, ymm2, ymm3/m256
    VPUNPCKHBW_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG 69 /r] VPUNPCKHWD ymm1, ymm2, ymm3/m256
    VPUNPCKHWD_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG 6A /r] VPUNPCKHDQ ymm1, ymm2, ymm3/m256
    VPUNPCKHDQ_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG 6D /r] VPUNPCKHQDQ ymm1, ymm2, ymm3/m256
    VPUNPCKHQDQ_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG 68 /r] VPUNPCKHBW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPUNPCKHBW_VdqHdqWdq_E128,
    // [EVEX.128.66.0F.WIG 69 /r] VPUNPCKHWD xmm1 {k1}{z}, xmm2, xmm3/m128
    VPUNPCKHWD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F.WIG 6A /r] VPUNPCKHDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPUNPCKHDQ_VdqHdqWdq_E128,
    // [EVEX.128.66.0F.WIG 6D /r] VPUNPCKHQDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPUNPCKHQDQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG 68 /r] VPUNPCKHBW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPUNPCKHBW_VqqHqqWqq_E256,
    // [EVEX.256.66.0F.WIG 69 /r] VPUNPCKHWD ymm1 {k1}{z}, ymm2, ymm3/m256
    VPUNPCKHWD_VqqHqqWqq_E256,
    // [EVEX.256.66.0F.WIG 6A /r] VPUNPCKHDQ ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPUNPCKHDQ_VqqHqqWqq_E256,
    // [EVEX.256.66.0F.WIG 6D /r] VPUNPCKHQDQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPUNPCKHQDQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG 68 /r] VPUNPCKHBW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPUNPCKHBW_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F.WIG 69 /r] VPUNPCKHWD zmm1 {k1}{z}, zmm2, zmm3/m512
    VPUNPCKHWD_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F.WIG 6A /r] VPUNPCKHDQ zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPUNPCKHDQ_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F.WIG 6D /r] VPUNPCKHQDQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPUNPCKHQDQ_VdqqHdqqWdqq_E512,

    // [NP 0F 60 /r] PUNPCKLBW mm1, mm2/m64
    PUNPCKLBW_PqQq,
    // [66 0F 60 /r] PUNPCKLBW xmm1, xmm2/m128
    PUNPCKLBW_VdqWdq,
    // [NP 0F 61 /r] PUNPCKLWD mm1, mm2/m64
    PUNPCKLWD_PqQq,
    // [66 0F 61 /r] PUNPCKLWD xmm1, xmm2/m128
    PUNPCKLWD_VdqWdq,
    // [NP 0F 62 /r] PUNPCKLDQ mm1, mm2/m64
    PUNPCKLDQ_PqQq,
    // [66 0F 62 /r] PUNPCKLDQ xmm1, xmm2/m128
    PUNPCKLDQ_VdqWdq,
    // [66 0F 6C /r] PUNPCKLQDQ xmm1, xmm2/m128
    PUNPCKLQDQ_VdqWdq,
    // [VEX.128.66.0F.WIG 60 /r] VPUNPCKLBW xmm1, xmm2, xmm3/m128
    VPUNPCKLBW_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 61 /r] VPUNPCKLWD xmm1, xmm2, xmm3/m128
    VPUNPCKLWD_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 62 /r] VPUNPCKLDQ xmm1, xmm2, xmm3/m128
    VPUNPCKLDQ_VdqHdqWdq_V128,
    // [VEX.128.66.0F.WIG 6C /r] VPUNPCKLQDQ xmm1, xmm2, xmm3/m128
    VPUNPCKLQDQ_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 60 /r] VPUNPCKLBW ymm1, ymm2, ymm3/m256
    VPUNPCKLBW_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG 61 /r] VPUNPCKLWD ymm1, ymm2, ymm3/m256
    VPUNPCKLWD_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG 62 /r] VPUNPCKLDQ ymm1, ymm2, ymm3/m256
    VPUNPCKLDQ_VqqHqqWqq_V256,
    // [VEX.256.66.0F.WIG 6C /r] VPUNPCKLQDQ ymm1, ymm2, ymm3/m256
    VPUNPCKLQDQ_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.WIG 60 /r] VPUNPCKLBW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPUNPCKLBW_VdqHdqWdq_E128,
    // [EVEX.128.66.0F.WIG 61 /r] VPUNPCKLWD xmm1 {k1}{z}, xmm2, xmm3/m128
    VPUNPCKLWD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F.WIG 62 /r] VPUNPCKLDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPUNPCKLDQ_VdqHdqWdq_E128,
    // [EVEX.128.66.0F.WIG 6C /r] VPUNPCKLQDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPUNPCKLQDQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.WIG 60 /r] VPUNPCKLBW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPUNPCKLBW_VqqHqqWqq_E256,
    // [EVEX.256.66.0F.WIG 61 /r] VPUNPCKLWD ymm1 {k1}{z}, ymm2, ymm3/m256
    VPUNPCKLWD_VqqHqqWqq_E256,
    // [EVEX.256.66.0F.WIG 62 /r] VPUNPCKLDQ ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPUNPCKLDQ_VqqHqqWqq_E256,
    // [EVEX.256.66.0F.WIG 6C /r] VPUNPCKLQDQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPUNPCKLQDQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.WIG 60 /r] VPUNPCKLBW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPUNPCKLBW_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F.WIG 61 /r] VPUNPCKLWD zmm1 {k1}{z}, zmm2, zmm3/m512
    VPUNPCKLWD_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F.WIG 62 /r] VPUNPCKLDQ zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPUNPCKLDQ_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F.WIG 6C /r] VPUNPCKLQDQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPUNPCKLQDQ_VdqqHdqqWdqq_E512,

    // [FF /6] PUSH r/m16
    PUSH_Ew,
    // [FF /6] PUSH r/m32
    PUSH_Ed,
    // [FF /6] PUSH r/m64
    PUSH_Eq,
    // [50+rw] PUSH r16
    PUSH_Gw,
    // [50+rd] PUSH r32
    PUSH_Gd,
    // [50+rd] PUSH r64
    PUSH_Gq,
    // [6A ib] PUSH imm8
    PUSH_Ib,
    // [68 iw] PUSH imm16
    PUSH_Iw,
    // [68 iw] PUSH imm32
    PUSH_Id,
    // [0E] PUSH CS
    PUSH_CS,
    // [16] PUSH SS
    PUSH_SS,
    // [1E] PUSH DS
    PUSH_DS,
    // [06] PUSH ES
    PUSH_ES,
    // [0F A0] PUSH FS
    PUSH_FS,
    // [0F A8] PUSH GS
    PUSH_GS,

    // [60] PUSHA
    PUSHA,
    // [60] PUSHAD
    PUSHAD,

    // [9C] PUSHF
    PUSHF,
    // [9C] PUSHFD
    PUSHFD,
    // [9C] PUSHFQ
    PUSHFQ,

    // [NP 0F EF /r] PXOR mm1, mm2/m64
    PXOR_PqQq,
    // [66 0F EF /r] PXOR xmm1, xmm2/m128
    PXOR_VdqWdq,
    // [VEX.128.66.0F.WIG EF /r] VPXOR xmm1, xmm2, xmm3/m128
    VPXOR_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG EF /r] VPXOR ymm1, ymm2, ymm3/m256
    VPXOR_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W0 EF /r] VPXORD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPXORD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W0 EF /r] VPXORD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPXORD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W0 EF /r] VPXORD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPXORD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F.W1 EF /r] VPXORQ xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPXORQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 EF /r] VPXORQ ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPXORQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 EF /r] VPXORQ zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPXORQ_VdqqHdqqWdqq_E512,

    // [D0 /2] RCL r/m8, 1
    // [REX D0 /2] RCL r/m8, 1
    RCL_Eb1,
    // [D2 /2] RCL r/m8, CL
    // [REX D2 /2] RCL r/m8, CL
    RCL_EbCL,
    // [C0 /2 ib] RCL r/m8, imm8
    // [REX C0 /2 ib] RCL r/m8, imm8
    RCL_EbIb,
    // [D1 /2] RCL r/m16, 1
    RCL_Ew1,
    // [D3 /2] RCL r/m16, CL
    RCL_EwCL,
    // [C1 /2 ib] RCL r/m16, imm8
    RCL_EwIb,
    // [D1 /2] RCL r/m32, 1
    RCL_Ed1,
    // [REX.W D1 /2] RCL r/m64, 1
    RCL_Eq1,
    // [D3 /2] RCL r/m32, CL
    RCL_EdCL,
    // [REX.W D3 /2] RCL r/m64, CL
    RCL_EqCL,
    // [C1 /2 ib] RCL r/m32, imm8
    RCL_EdIb,
    // [REX.W C1 /2 ib] RCL r/m64, imm8
    RCL_EqIb,
    // [D0 /3] RCR r/m8, 1
    // [REX D0 /3] RCR r/m8, 1
    RCR_Eb1,
    // [D2 /3] RCR r/m8, CL
    // [REX D2 /3] RCR r/m8, CL
    RCR_EbCL,
    // [C0 /3 ib] RCR r/m8, imm8
    // [REX C0 /3 ib] RCR r/m8, imm8
    RCR_EbIb,
    // [D1 /3] RCR r/m16, 1
    RCR_Ew1,
    // [D3 /3] RCR r/m16, CL
    RCR_EwCL,
    // [C1 /3 ib] RCR r/m16, imm8
    RCR_EwIb,
    // [D1 /3] RCR r/m32, 1
    RCR_Ed1,
    // [REX.W D1 /3] RCR r/m64, 1
    RCR_Eq1,
    // [D3 /3] RCR r/m32, CL
    RCR_EdCL,
    // [REX.W D3 /3] RCR r/m64, CL
    RCR_EqCL,
    // [C1 /3 ib] RCR r/m32, imm8
    RCR_EdIb,
    // [REX.W C1 /3 ib] RCR r/m64, imm8
    RCR_EqIb,
    // [D0 /0] ROL r/m8, 1
    // [REX D0 /0] ROL r/m8, 1
    ROL_Eb1,
    // [D2 /0] ROL r/m8, CL
    // [REX D2 /0] ROL r/m8, CL
    ROL_EbCL,
    // [C0 /0 ib] ROL r/m8, imm8
    // [REX C0 /0 ib] ROL r/m8, imm8
    ROL_EbIb,
    // [D1 /0] ROL r/m16, 1
    ROL_Ew1,
    // [D3 /0] ROL r/m16, CL
    ROL_EwCL,
    // [C1 /0 ib] ROL r/m16, imm8
    ROL_EwIb,
    // [D1 /0] ROL r/m32, 1
    ROL_Ed1,
    // [REX.W D1 /0] ROL r/m64, 1
    ROL_Eq1,
    // [D3 /0] ROL r/m32, CL
    ROL_EdCL,
    // [REX.W D3 /0] ROL r/m64, CL
    ROL_EqCL,
    // [C1 /0 ib] ROL r/m32, imm8
    ROL_EdIb,
    // [REX.W C1 /0 ib] ROL r/m64, imm8
    ROL_EqIb,
    // [D0 /1] ROR r/m8, 1
    // [REX D0 /1] ROR r/m8, 1
    ROR_Eb1,
    // [D2 /1] ROR r/m8, CL
    // [REX D2 /1] ROR r/m8, CL
    ROR_EbCL,
    // [C0 /1 ib] ROR r/m8, imm8
    // [REX C0 /1 ib] ROR r/m8, imm8
    ROR_EbIb,
    // [D1 /1] ROR r/m16, 1
    ROR_Ew1,
    // [D3 /1] ROR r/m16, CL
    ROR_EwCL,
    // [C1 /1 ib] ROR r/m16, imm8
    ROR_EwIb,
    // [D1 /1] ROR r/m32, 1
    ROR_Ed1,
    // [REX.W D1 /1] ROR r/m64, 1
    ROR_Eq1,
    // [D3 /1] ROR r/m32, CL
    ROR_EdCL,
    // [REX.W D3 /1] ROR r/m64, CL
    ROR_EqCL,
    // [C1 /1 ib] ROR r/m32, imm8
    ROR_EdIb,
    // [REX.W C1 /1 ib] ROR r/m64, imm8
    ROR_EqIb,

    // [NP 0F 53 /r] RCPPS xmm1, xmm2/m128
    RCPPS_VdqWdq,
    // [VEX.128.0F.WIG 53 /r] VRCPPS xmm1, xmm2/m128
    VRCPPS_VdqWdq_V128,
    // [VEX.256.0F.WIG 53 /r] VRCPPS ymm1, ymm2/m256
    VRCPPS_VqqWqq_V256,

    // [F3 0F 53 /r] RCPPS xmm1, xmm2/m32
    RCPSS_VdqWd,
    // [VEX.128.F3.0F.WIG 53 /r] VRCPSS xmm1, xmm2, xmm3/m32
    VRCPSS_VdqHdqWd_V128,

    // [F3 0F AE /0] RDFSBASE r32
    RDFSBASE_Gd,
    // [F3 REX.W 0F AE /0] RDFSBASE r64
    RDFSBASE_Gq,
    // [F3 0F AE /1] RDGSBASE r32
    RDGSBASE_Gd,
    // [F3 REX.W 0F AE /1] RDGSBASE r64
    RDGSBASE_Gq,

    // [0F 32] RDMSR
    RDMSR,

    // [F3 0F C7 /7] RDPID r32
    RDPID_Gd,
    // [F3 0F C7 /7] RDPID r64
    RDPID_Gq,

    // [NP 0F 01 EE] RDPKRU
    RDPKRU,

    // [0F 33] RDPMC
    RDPMC,

    // [NFx 0F C7 /6] RDRAND r16
    RDRAND_Gw,
    // [NFx 0F C7 /6] RDRAND r32
    RDRAND_Gd,
    // [NFx REX.W 0F C7 /6] RDRAND r64
    RDRAND_Gq,

    // [NFx 0F C7 /7] RDSEED r16
    RDSEED_Gw,
    // [NFx 0F C7 /7] RDSEED r32
    RDSEED_Gd,
    // [NFx REX.W 0F C7 /7] RDSEED r64
    RDSEED_Gq,

    // [F3 0F 1E /1 (mod=11)] RDSSPD r32
    RDSSPD_Gd,
    // [F3 REX.W 0F 1E /1 (mod=11)] RDSSPQ r64
    RDSSPQ_Gq,

    // [0F 31] RDTSC
    RDTSC,

    // [0F 01 F9] RDTSCP
    RDTSCP,

    // [F3 6C] REP INS m8, DX
    REP_INS_YbDX,
    // [F3 6D] REP INS m16, DX
    REP_INS_YwDX,
    // [F3 6D] REP INS m32, DX
    REP_INS_YDX,
    // [F3 6D] REP INS r/m32, DX
    REP_INS_YdDX,
    // [F3 A4] REP MOVS m8, m8
    // [F3 REX.W A4] REP MOVS m8, m8
    REP_MOVS_YbXb,
    // [F3 A5] REP MOVS m16, m16
    REP_MOVS_YwXw,
    // [F3 A5] REP MOVS m32, m32
    REP_MOVS_YdXd,
    // [F3 REX.W A5] REP MOVS m64, m64
    REP_MOVS_YqXq,
    // [F3 6E] REP OUTS DX, r/m8
    // [F3 REX.W 6E] REP OUTS DX, r/m8
    REP_OUTS_DXYb,
    // [F3 6F] REP OUTS DX, r/m16
    REP_OUTS_DXYw,
    // [F3 6F] REP OUTS DX, r/m32
    // [F3 REX.W 6F] REP OUTS DX, r/m32
    REP_OUTS_DXYd,
    // [F3 AC] REP LODS AL
    // [F3 REX.W AC] REP LODS AL
    REP_LODS_ALXb,
    // [F3 AD] REP LODS AX
    REP_LODS_AXXw,
    // [F3 AD] REP LODS EAX
    REP_LODS_EAXXd,
    // [F3 REX.W AD] REP LODS RAX
    REP_LODS_RAXXq,
    // [F3 AA] REP STOS m8
    // [F3 REX.W AA] REP STOS m8
    REP_STOS_YbAL,
    // [F3 AB] REP STOS m16
    REP_STOS_YwAX,
    // [F3 AB] REP STOS m32
    REP_STOS_YdEAX,
    // [F3 REX.W AB] REP STOS m64
    REP_STOS_YqRAX,
    // [F3 A6] REPE CMPS m8, m8
    // [F3 REX.W A6] REPE CMPS m8, m8
    REP_CMPS_XbYb,
    // [F3 A7] REPE CMPS m16, m16
    REP_CMPS_XwYw,
    // [F3 A7] REPE CMPS m32, m32
    REP_CMPS_XdYd,
    // [F3 REX.W A7] REPE CMPS m64, m64
    REP_CMPS_XqYq,
    // [F3 AE] REPE SCAS m8
    // [F3 REX.W AE] REPE SCAS m8
    REPE_SCAS_ALYb,
    // [F3 AF] REPE SCAS m16
    REPE_SCAS_AXYw,
    // [F3 AF] REPE SCAS m32
    REPE_SCAS_EAXYd,
    // [F3 REX.W AF] REPE SCAS m64
    REPE_SCAS_RAXYq,
    // [F2 A6] REPNE CMPS m8, m8
    // [F2 REX.W A6] REPNE CMPS m8, m8
    REPNE_CMPS_XbYb,
    // [F2 A7] REPNE CMPS m16, m16
    REPNE_CMPS_XwYw,
    // [F2 A7] REPNE CMPS m32, m32
    REPNE_CMPS_XdYd,
    // [F2 REX.W A7] REPNE CMPS m64, m64
    REPNE_CMPS_XqYq,
    // [F2 AE] REPNE SCAS m8
    // [F2 REX.W AE] REPNE SCAS m8
    REPNE_SCAS_ALYb,
    // [F2 AF] REPNE SCAS m16
    REPNE_SCAS_AXYw,
    // [F2 AF] REPNE SCAS m32
    REPNE_SCAS_EAXYd,
    // [F2 REX.W AF] REPNE SCAS m64
    REPNE_SCAS_RAXYq,

    // [C3] RET
    RET,
    // [CB] RET
    RETF,
    // [C2 iw] RET imm16
    RET_Iw,
    // [CA iw] RET imm16
    RETF_Iw,

    // [VEX.LZ.F2.0F3A.W0 F0 /r ib] RORX r32, r/m32, imm8
    RORX_GdEdIb_V,
    // [VEX.LZ.F2.0F3A.W1 F0 /r ib] RORX r64, r/m64, imm8
    RORX_GqEqIb_V,

    // [66 0F 3A 09 /r ib] ROUNDPD xmm1, xmm2/m128, imm8
    ROUNDPD_VdqWdqIb,
    // [VEX.128.66.0F3A.WIG 09 /r ib] VROUNDPD xmm1, xmm2/m128, imm8
    VROUNDPD_VdqWdqIb_V128,
    // [VEX.256.66.0F3A.WIG 09 /r ib] VROUNDPD ymm1, ymm2/m256, imm8
    VROUNDPD_VqqWqqIb_V256,

    // [66 0F 3A 08 /r ib] ROUNDPS xmm1, xmm2/m128, imm8
    ROUNDPS_VdqWdqIb,
    // [VEX.128.66.0F3A.WIG 08 /r ib] VROUNDPS xmm1, xmm2/m128, imm8
    VROUNDPS_VdqWdqIb_V128,
    // [VEX.256.66.0F3A.WIG 08 /r ib] VROUNDPS ymm1, ymm2/m256, imm8
    VROUNDPS_VqqWqqIb_V256,

    // [66 0F 3A 0B /r ib] ROUNDSD xmm1, xmm2/m64, imm8
    ROUNDSD_VdqWq,
    // [VEX.LIG.66.0F3A.WIG 0B /r ib] VROUNDSD xmm1, xmm2, xmm3/m64, imm8
    VROUNDSD_VdqHdqWqIb_V,

    // [66 0F 3A 0B /r ib] ROUNDSS xmm1, xmm2/m32, imm8
    ROUNDSS_VdqWd,
    // [VEX.LIG.66.0F3A.WIG 0B /r ib] VROUNDSS xmm1, xmm2, xmm3/m32, imm8
    VROUNDSS_VdqHdqWdIb_V,

    // [0F AA] RSM
    RSM,

    // [NP 0F 52 /r] RSQRTPS xmm1, xmm2/m128
    RSQRTPS_VdqWdq,
    // [VEX.128.0F.WIG 52 /r] VRSQRTPS xmm1, xmm2/m128
    VRSQRTPS_VdqWdq_V128,
    // [VEX.256.0F.WIG 52 /r] VRSQRTPS ymm1, ymm2/m256
    VRSQRTPS_VqqWqq_V256,

    // [F3 0F 52 /r] RSQRTSS xmm1, xmm2/m32
    RSQRTSS_VdqWd,
    // [VEX.LIG.F3.0F.WIG 52 /r] VRSQRTSS xmm1, xmm2, xmm3/m32
    VRSQRTSS_VdqHdqWd_V,

    // [F3 0F 01 /5 (mod!=11, /5, mem-only)] RSTORSSP m64
    RSTORSSP_Mq,

    // [9E] SAHF
    SAHF,

    // [D0 /4] SAL r/m8, 1
    // [REX D0 /4] SAL r/m8, 1
    // [D0 /4] SHL r/m8, 1
    // [REX D0 /4] SHL r/m8, 1
    SAL_Eb1,
    // [D2 /4] SAL r/m8, CL
    // [REX D2 /4] SAL r/m8, CL
    // [D2 /4] SHL r/m8, CL
    // [REX D2 /4] SHL r/m8, CL
    SAL_EbCL,
    // [C0 /4 ib] SAL r/m8, imm8
    // [REX C0 /4 ib] SAL r/m8, imm8
    // [C0 /4 ib] SHL r/m8, imm8
    // [REX C0 /4 ib] SHL r/m8, imm8
    SAL_EbIb,
    // [D1 /4] SAL r/m16, 1
    // [D1 /4] SAL r/m16, 1
    SAL_Ew1,
    // [D3 /4] SAL r/m16, CL
    // [D3 /4] SHL r/m16, CL
    SAL_EwCL,
    // [C1 /4 ib] SAL r/m32, imm8
    // [C1 /4 ib] SHL r/m32, imm8
    SAL_EdIb,
    // [D1 /4] SAL r/m32, 1
    // [D1 /4] SHL r/m32, 1
    SAL_Ed1,
    // [REX.W D1 /4] SAL r/m64, 1
    // [REX.W D1 /4] SHL r/m64, 1
    SAL_Eq1,
    // [D3 /4] SAL r/m32, CL
    // [D3 /4] SHL r/m32, CL
    SAL_EdCL,
    // [REX.W D3 /4] SAL r/m64, CL
    // [REX.W D3 /4] SHL r/m64, CL
    SAL_EqCL,
    // [REX.W C1 /4 ib] SAL r/m64, imm8
    // [REX.W C1 /4 ib] SHL r/m64, imm8
    SAL_EqIb,
    // [D0 /7] SAR r/m8, 1
    SAR_Eb1,
    // [D2 /7] SAR r/m8, CL
    // [REX D2 /7] SAR r/m8, CL
    SAR_EbCL,
    // [C0 /7 ib] SAR r/m8, imm8
    // [REX C0 /7 ib] SAR r/m8, imm8
    SAR_EbIb,
    // [D1 /7] SAR r/m16, 1
    SAR_Ew1,
    // [D3 /7] SAR r/m16, CL
    SAR_EwCL,
    // [C1 /7 ib] SAR r/m32, imm8
    SAR_EdIb,
    // [D1 /7] SAR r/m32, 1
    SAR_Ed1,
    // [REX.W D1 /7] SAR r/m64, 1
    SAR_Eq1,
    // [D3 /7] SAR r/m32, CL
    SAR_EdCL,
    // [REX.W D3 /7] SAR r/m64, CL
    SAR_EqCL,
    // [REX.W C1 /7 ib] SAR r/m64, imm8
    SAR_EqIb,
    // [D0 /5] SHR r/m8, 1
    // [REX D0 /5] SHR r/m8, 1
    SHR_Eb1,
    // [D2 /5] SHR r/m8, CL
    // [REX D2 /5] SHR r/m8, CL
    SHR_EbCL,
    // [C0 /5 ib] SHR r/m8, imm8
    // [REX C0 /5 ib] SHR r/m8, imm8
    SHR_EbIb,
    // [D1 /5] SHR r/m16, 1
    SHR_Ew1,
    // [D3 /5] SHR r/m16, CL
    SHR_EwCL,
    // [C1 /5 ib] SHR r/m32, imm8
    SHR_EdIb,
    // [D1 /5] SHR r/m32, 1
    SHR_Ed1,
    // [REX.W D1 /5] SHR r/m64, 1
    SHR_Eq1,
    // [D3 /5] SHR r/m32, CL
    SHR_EdCL,
    // [REX.W D3 /5] SHR r/m64, CL
    SHR_EqCL,
    // [REX.W C1 /5 ib] SHR r/m64, imm8
    SHR_EqIb,

    // [VEX.LZ.F3.0F38.W0 F7 /r] SARX r32a, r/m32, r32b
    SARX_GdEdBd,
    // [VEX.LZ.66.0F38.W0 F7 /r] SHLX r32a, r/m32, r32b
    SHLX_GdEdBd,
    // [VEX.LZ.F2.0F38.W0 F7 /r] SHRX r32a, r/m32, r32b
    SHRX_GdEdBd,
    // [VEX.LZ.F3.0F38.W1 F7 /r] SARX r64a, r/m64, r64b
    SARX_GqEqBq,
    // [VEX.LZ.66.0F38.W1 F7 /r] SHLX r64a, r/m64, r64b
    SHLX_GqEqBq,
    // [VEX.LZ.F2.0F38.W1 F7 /r] SHRX r64a, r/m64, r64b
    SHRX_GqEqBq,

    // [F3 0F 01 EA (mod!=11, /5, rm=010)] SAVEPREVSSP
    SAVEPREVSSP,

    // [1C ib] SBB AL, imm8
    SBB_ALIb,
    // [1D iw] SBB AX, imm16
    SBB_AXIw,
    // [1D id] SBB EAX, imm32
    SBB_EAXId,
    // [REX.W 1D id] SBB RAX, imm32
    SBB_RAXId,
    // [80 /3 ib] SBB r/m8, imm8
    // [REX 80 /3 ib] SBB r/m8, imm8
    SBB_EbIb,
    // [81 /3 iw] SBB r/m16, imm16
    SBB_EwIw,
    // [81 /3 id] SBB r/m32, imm32
    SBB_EdId,
    // [REX.W 81 /3 id] SBB r/m64, imm32
    SBB_EqId,
    // [83 /3 ib] SBB r/m16, imm8
    SBB_EwIb,
    // [83 /3 ib] SBB r/m32, imm8
    SBB_EdIb,
    // [REX.W 83 /3 ib] SBB r/m64, imm8
    SBB_EqIb,
    // [18 /r] SBB r/m8, r8
    SBB_EbGb,
    // [19 /r] SBB r/m16, r16
    SBB_EwGw,
    // [19 /r] SBB r/m32, r32
    SBB_EdGd,
    // [REX.W 19 /r] SBB r/m64, r64
    SBB_EqGq,
    // [1A /r] SBB r8, r/m8
    // [REX 1A /r] SBB r8, r/m8
    SBB_GbEb,
    // [1B /r] SBB r16, r/m16
    SBB_GwEw,
    // [1B /r] SBB r32, r/m32
    SBB_GdEd,
    // [REX.W 1B /r] SBB r64, r/m64
    SBB_GqEq,

    // [AE] SCAS m8
    // [AE] SCASB
    SCAS_ALYb,
    // [AF] SCAS m16
    // [AF] SCASW
    SCAS_AXYw,
    // [AF] SCAS m32
    // [AF] SCASD
    SCAS_EAXYd,
    // [REX.W AF] SCAS m64
    // [REX.W AF] SCASQ
    SCAS_RAXYq,

    // [0F 90] SETO r/m8
    // [REX 0F 90] SETO r/m8
    SETO_Eb,
    // [0F 91] SETNO r/m8
    // [REX 0F 91] SETNO r/m8
    SETNO_Eb,
    // [0F 92] SETB r/m8
    // [REX 0F 92] SETB r/m8
    // [0F 92] SETC r/m8
    // [REX 0F 92] SETC r/m8
    // [0F 92] SETAE r/m8
    // [REX 0F 92] SETAE r/m8
    SETB_Eb,
    // [0F 93] SETAE r/m8
    // [REX 0F 93] SETAE r/m8
    // [0F 93] SETNB r/m8
    // [REX 0F 93] SETNB r/m8
    // [0F 93] SETNC r/m8
    // [REX 0F 93] SETNC r/m8
    SETAE_Eb,
    // [0F 94] SETE r/m8
    // [REX 0F 94] SETE r/m8
    // [0F 94] SETZ r/m8
    // [REX 0F 94] SETZ r/m8
    SETE_Eb,
    // [0F 95] SETNE r/m8
    // [REX 0F 95] SETNE r/m8
    // [0F 95] SETNZ r/m8
    // [REX 0F 95] SETNZ r/m8
    SETNE_Eb,
    // [0F 96] SETBE r/m8
    // [REX 0F 96] SETBE r/m8
    // [0F 96] SETNA r/m8
    // [REX 0F 96] SETNA r/m8
    SETBE_Eb,
    // [0F 97] SETA r/m8
    // [REX 0F 97] SETA r/m8
    // [0F 97] SETNBE r/m8
    // [REX 0F 97] SETNBE r/m8
    SETA_Eb,
    // [0F 98] SETS r/m8
    // [REX 0F 98] SETS r/m8
    SETS_Eb,
    // [0F 99] SETNS r/m8
    // [REX 0F 99] SETNS r/m8
    SETNS_Eb,
    // [0F 9A] SETP r/m8
    // [REX 0F 9A] SETP r/m8
    // [0F 9A] SETPE r/m8
    // [REX 0F 9A] SETPE r/m8
    SETP_Eb,
    // [0F 9B] SETNP r/m8
    // [REX 0F 9B] SETNP r/m8
    // [0F 9B] SETPO r/m8
    // [REX 0F 9B] SETPO r/m8
    SETNP_Eb,
    // [0F 9C] SETL r/m8
    // [REX 0F 9C] SETL r/m8
    // [0F 9C] SETNGE r/m8
    // [REX 0F 9C] SETNGE r/m8
    SETL_Eb,
    // [0F 9D] SETGE r/m8
    // [REX 0F 9D] SETGE r/m8
    // [0F 9D] SETNL r/m8
    // [REX 0F 9D] SETNL r/m8
    SETGE_Eb,
    // [0F 9E] SETLE r/m8
    // [REX 0F 9E] SETLE r/m8
    // [0F 9E] SETNG r/m8
    // [REX 0F 9E] SETNG r/m8
    SETLE_Eb,
    // [0F 9F] SETG r/m8
    // [REX 0F 9F] SETG r/m8
    // [0F 9F] SETNLE r/m8
    // [REX 0F 9F] SETNLE r/m8
    SETG_Eb,

    // [F3 0F 01 E8] SETSSBSY
    SETSSBSY,

    // [NP 0F AE F8] SFENCE
    SFENCE,

    // [0F 01 /0] SGDT mem
    SGDT_M,

    // [NP 0F 3A CC /r ib] SHA1RNDS4 xmm1, xmm2/m128, imm8
    SHA1RNDS4_VdqWdqIb,

    // [NP 0F 38 C8 /r] SHA1NEXTE xmm1, xmm2/m128
    SHA1NEXTE_VdqWdq,

    // [NP 0F 38 C9 /r] SHA1MSG1 xmm1, xmm2/m128
    SHA1MSG1_VdqWdq,

    // [NP 0F 38 CA /r] SHA1MSG2 xmm1, xmm2/m128
    SHA1MSG2_VdqWdq,

    // [NP 0F 38 CB /r] SHA256RNDS2 xmm1, xmm2/m128, <XMM0>
    SHA256RNDS2_VdqWdq,

    // [NP 0F 38 CC /r] SHA256MSG1 xmm1, xmm2/m128
    SHA256MSG1_VdqWdq,

    // [NP 0F 38 CD /r] SHA256MSG2 xmm1, xmm2/m128
    SHA256MSG2_VdqWdq,

    // [0F A4 /r ib] SHLD r/m16, r16, imm8
    SHLD_EwGwIb,
    // [0F A5 /r] SHLD r/m16, r16, CL
    SHLD_EwGwCL,
    // [0F A4 /r ib] SHLD r/m32, r32, imm8
    SHLD_EdGdIb,
    // [0F A5 /r] SHLD r/m32, r32, CL
    SHLD_EdGdCL,
    // [REX.W 0F A4 /r ib] SHLD r/m64, r64, imm8
    SHLD_EqGqIb,
    // [REX.W 0F A5 /r] SHLD r/m64, r64, CL
    SHLD_EqGqCL,

    // [0F AC /r ib] SHRD r/m16, r16, imm8
    SHRD_EwGwIb,
    // [0F AD /r] SHRD r/m16, r16, CL
    SHRD_EwGwCL,
    // [0F AC /r ib] SHRD r/m32, r32, imm8
    SHRD_EdGdIb,
    // [0F AD /r] SHRD r/m32, r32, CL
    SHRD_EdGdCL,
    // [REX.W 0F AC /r ib] SHRD r/m64, r64, imm8
    SHRD_EqGqIb,
    // [REX.W 0F AD /r] SHRD r/m64, r64, CL
    SHRD_EqGqCL,

    // [66 0F C6 /r ib] SHUFPD xmm1, xmm2/m128, imm8
    SHUFPD_VdqWdqIb,
    // [VEX.128.66.0F.WIG C6 /r ib] VSHUFPD xmm1, xmm2, xmm3/m128, imm8
    VSHUFPD_VdqHdqWdqIb_V128,
    // [VEX.256.66.0F.WIG C6 /r ib] VSHUFPD ymm1, ymm2, ymm3/m256, imm8
    VSHUFPD_VqqHqqWqqIb_V256,
    // [EVEX.128.66.0F.W1 C6 /r ib] VSHUFPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    VSHUFPD_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F.W1 C6 /r ib] VSHUFPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    VSHUFPD_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F.W1 C6 /r ib] VSHUFPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    VSHUFPD_VdqqHdqqWdqqIb_E512,

    // [NP 0F C6 /r ib] SHUFPS xmm1, xmm2/m128, imm8
    SHUFPS_VdqWdqIb,
    // [VEX.128.0F.WIG C6 /r ib] VSHUFPS xmm1, xmm2, xmm3/m128, imm8
    VSHUFPS_VdqHdqWdqIb_V128,
    // [VEX.256.0F.WIG C6 /r ib] VSHUFPS ymm1, ymm2, ymm3/m256, imm8
    VSHUFPS_VqqHqqWqqIb_V256,
    // [EVEX.128.0F.W0 C6 /r ib] VSHUFPS xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    VSHUFPS_VdqHdqWdqIb_E128,
    // [EVEX.256.0F.W0 C6 /r ib] VSHUFPS ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    VSHUFPS_VqqHqqWqqIb_E256,
    // [EVEX.512.0F.W0 C6 /r ib] VSHUFPS zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    VSHUFPS_VdqqHdqqWdqqIb_E512,

    // [0F 01 /1] SIDT mem
    SIDT_M,

    // [0F 00 /0] SLDT r/m16
    SLDT_Ew,
    // [REX.W 0F 00 /0] SLDT r64/m16
    SLDT_Eq,

    // [0F 01 /4] SMSW r/m16
    SMSW_Ew,
    // [0F 01 /4] SMSW r32/m16
    SMSW_Ed,
    // [REX.W 0F 01 /4] SMSW r64/m16
    SMSW_Eq,

    // [66 0F 51 /r] SQRTPD xmm1, xmm2/m128
    SQRTPD_VdqWdq,
    // [VEX.128.66.0F.WIG 51 /r] VSQRTPD xmm1, xmm2/m128
    VSQRTPD_VdqWdq_V128,
    // [VEX.256.66.0F.WIG 51 /r] VSQRTPD ymm1, ymm2/m256
    VSQRTPD_VqqWqq_V256,
    // [EVEX.128.66.0F.W1 51 /r] VSQRTPD xmm1 {k1}{z}, xmm2/m128/m64bcst
    VSQRTPD_VdqWdq_E128,
    // [EVEX.256.66.0F.W1 51 /r] VSQRTPD ymm1 {k1}{z}, ymm2/m256/m64bcst
    VSQRTPD_VqqWqq_E256,
    // [EVEX.512.66.0F.W1 51 /r] VSQRTPD zmm1 {k1}{z}, zmm2/m512/m64bcst{er}
    VSQRTPD_VdqqWdqq_E512,

    // [NP 0F 51 /r] SQRTPS xmm1, xmm2/m128
    SQRTPS_VdqWdq,
    // [VEX.128.0F.WIG 51 /r] VSQRTPS xmm1, xmm2/m128
    VSQRTPS_VdqWdq_V128,
    // [VEX.256.0F.WIG 51 /r] VSQRTPS ymm1, ymm2/m256
    VSQRTPS_VqqWqq_V256,
    // [EVEX.128.0F.W1 51 /r] VSQRTPS xmm1 {k1}{z}, xmm2/m128/m32bcst
    VSQRTPS_VdqWdq_E128,
    // [EVEX.256.0F.W1 51 /r] VSQRTPS ymm1 {k1}{z}, ymm2/m256/m32bcst
    VSQRTPS_VqqWqq_E256,
    // [EVEX.512.0F.W1 51 /r] VSQRTPS zmm1 {k1}{z}, zmm2/m512/m32bcst{er}
    VSQRTPS_VdqqWdqq_E512,

    // [F2 0F 51 /r] SQRTSD xmm1, xmm2/m64
    SQRTSD_VdqWq,
    // [VEX.LIG.F2.0F.WIG 51 /r] VSQRTSD xmm1, xmm2, xmm3/m64
    VSQRTSD_VdqHdqWq_V,
    // [EVEX.LIG.F2.0F.W1 51 /r] VSQRTSD xmm1 {k1}{z}, xmm2, xmm3/m64{er}
    VSQRTSD_VdqHdqWq_E,

    // [F2 0F 51 /r] SQRTSS xmm1, xmm2/m64
    SQRTSS_VdqWq,
    // [VEX.LIG.F2.0F.WIG 51 /r] VSQRTSS xmm1, xmm2, xmm3/m32
    VSQRTSS_VdqHdqWd_V,
    // [EVEX.LIG.F2.0F.W1 51 /r] VSQRTSS xmm1 {k1}{z}, xmm2, xmm3/m32{er}
    VSQRTSS_VdqHdqWd_E,

    // [NP 0F 01 CB] STAC
    STAC,

    // [F9] STC
    STC,

    // [FD] STD
    STD,

    // [FB] STI
    STI,

    // [NP 0F AE /3] STMXCSR m32
    STMXCSR_Md,
    // [VEX.LZ.0F.WIG AE /3] VSTMXCSR m32
    VSTMXCSR_Md_V,

    // [AA] STOS m8
    // [AA] STOSB
    STOS_YbAL,
    // [AB] STOS m16
    // [AB] STOSW
    STOS_YwAX,
    // [AB] STOS m32
    // [AB] STOSD
    STOS_YdEAX,
    // [REX.W AB] STOS m64
    // [REX.W AB] STOSQ
    STOS_YqRAX,

    // [0F 00 /1] STR r/m16
    STR_Ew,

    // [2C ib] SUB AL, imm8
    SUB_ALIb,
    // [2D iw] SUB AX, imm16
    SUB_AXIw,
    // [2D id] SUB EAX, imm32
    SUB_EAXId,
    // [REX.W 2D id] SUB RAX, imm32
    SUB_RAXId,
    // [80 /5 ib] SUB r/m8, imm8
    // [REX 80 /5 ib] SUB r/m8, imm8
    SUB_EbIb,
    // [81 /5 iw] SUB r/m16, imm16
    SUB_EwIw,
    // [81 /5 id] SUB r/m32, imm32
    SUB_EdId,
    // [REX.W 81 /5 id] SUB r/m64, imm32
    SUB_EqId,
    // [83 /5 ib] SUB r/m16, imm8
    SUB_EwIb,
    // [83 /5 ib] SUB r/m32, imm8
    SUB_EdIb,
    // [REX.W 83 /5 ib] SUB r/m64, imm8
    SUB_EqIb,
    // [28 /r] SUB r/m8, r8
    // [REX 28 /r] SUB r/m8, r8
    SUB_EbGb,
    // [29 /r] SUB r/m16, r16
    SUB_EwGw,
    // [29 /r] SUB r/m32, r32
    SUB_EdGd,
    // [REX.W 29 /r] SUB r/m64, r64
    SUB_EqGq,
    // [2A /r] SUB r8, r/m8
    // [REX 2A /r] SUB r8, r/m8
    SUB_GbEb,
    // [2B /r] SUB r16, r/m16
    SUB_GwEw,
    // [2B /r] SUB r32, r/m32
    SUB_GdEd,
    // [REX.W 2B /r] SUB r64, r/m64
    SUB_GqEq,

    // [66 0F 5C /r] SUBPD xmm1, xmm2/m128
    SUBPD_VdqWdq,
    // [VEX.128.66.0F.WIG 5C /r] VSUBPD xmm1, xmm2, xmm3/m128
    VSUBPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 5C /r] VSUBPD ymm1, ymm2, ymm3/m256
    VSUBPD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W1 5C /r] VSUBPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VSUBPD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 5C /r] VSUBPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VSUBPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 5C /r] VSUBPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VSUBPD_VdqqHdqqWdqq_E512,

    // [NP 0F 5C /r] SUBPS xmm1, xmm2/m128
    SUBPS_VdqWdq,
    // [VEX.128.0F.WIG 5C /r] VSUBPS xmm1, xmm2, xmm3/m128
    VSUBPS_VdqHdqWdq_V128,
    // [VEX.256.0F.WIG 5C /r] VSUBPS ymm1, ymm2, ymm3/m256
    VSUBPS_VqqHqqWqq_V256,
    // [EVEX.128.0F.W1 5C /r] VSUBPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VSUBPS_VdqHdqWdq_E128,
    // [EVEX.256.0F.W1 5C /r] VSUBPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VSUBPS_VqqHqqWqq_E256,
    // [EVEX.512.0F.W1 5C /r] VSUBPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VSUBPS_VdqqHdqqWdqq_E512,

    // [F2 0F 5C /r] SUBSD xmm1, xmm2/m64
    SUBSD_VdqWq,
    // [VEX.LIG.F2.0F.WIG 5C /r] VSUBSD xmm1, xmm2, xmm3/m64
    VSUBSD_VdqHdqWq_V,
    // [EVEX.LIG.F2.0F.W1 5C /r] VSUBSD xmm1 {k1}{z}, xmm2, xmm3/m64{er}
    VSUBSD_VdqHdqWq_E,

    // [F3 0F 5C /r] SUBSS xmm1, xmm2/m32
    SUBSS_VdqWd,
    // [VEX.LIG.F2.0F.WIG 5C /r] VSUBSS xmm1, xmm2, xmm3/m32
    VSUBSS_VdqHdqWd_V,
    // [EVEX.LIG.F2.0F.W1 5C /r] VSUBSS xmm1 {k1}{z}, xmm2, xmm3/m32{er}
    VSUBSS_VdqHdqWd_E,

    // [0F 01 F8] SWAPGS
    SWAPGS,

    // [0F 05] SYSCALL
    SYSCALL,

    // [0F 34] SYSENTER
    SYSENTER,

    // [0F 35] SYSEXIT
    // [REX.W 0F 35] SYSEXIT
    SYSEXIT,

    // [0F 07] SYSRET
    // [REX.W 0F 07] SYSRET
    SYSRET,

    // [A8 ib] TEST AL, imm8
    TEST_ALIb,
    // [A9 iw] TEST AX, imm16
    TEST_AXIw,
    // [A9 id] TEST EAX, imm32
    TEST_EAXId,
    // [REX.W A9 id] TEST RAX, imm32
    TEST_RAXId,
    // [F6 /0 ib] TEST r/m8, imm8
    // [REX F6 /0 ib] TEST r/m8, imm8
    TEST_EbIb,
    // [F7 /0 iw] TEST r/m16, imm16
    TEST_EwIw,
    // [F7 /0 id] TEST r/m32, imm32
    TEST_EdId,
    // [REX.W F7 /0 id] TEST r/m64, imm32
    TEST_EqId,
    // [84 /r] TEST r/m8, r8
    // [REX 84 /r] TEST r/m8, r8
    TEST_EbGb,
    // [85 /r] TEST r/m16, r16
    TEST_EwGw,
    // [85 /r] TEST r/m32, r32
    TEST_EdGd,
    // [REX.W 85 /r] TEST r/m64, r64
    TEST_EqGq,

    // [66 0F AE /6] TPAUSE r32
    TPAUSE_Gd,

    // [F3 0F BC /r] TZCNT r16, r/m16
    TZCNT_GwEw,
    // [F3 0F BC /r] TZCNT r32, r/m32
    TZCNT_GdEd,
    // [F3 REX.W 0F BC /r] TZCNT r64, r/m64
    TZCNT_GqEq,

    // [66 0F 2E /r] UCOMISD xmm1, xmm2/m64
    UCOMISD_VdqWq,
    // [VEX.LIG.66.0F.WIG 2E /r] VUCOMISD xmm1, xmm2/m64
    VUCOMISD_VdqWq_V,
    // [EVEX.LIG.66.0F.W1 2E /r] VUCOMISD xmm1, xmm2/m64{sae}
    VUCOMISD_VdqWq_E,

    // [NP 0F 2E /r] UCOMISS xmm1, xmm2/m32
    UCOMISS_VdqWd,
    // [VEX.LIG.0F.WIG 2E /r] VUCOMISS xmm1, xmm2/m32
    VUCOMISS_VdqWd_V,
    // [EVEX.LIG.0F.W1 2E /r] VUCOMISS xmm1, xmm2/m32{sae}
    VUCOMISS_VdqWd_E,

    // [0F FF /r] UD0 r32, r/m32
    UD0_GdEd,
    // [0F B9 /r] UD1 r32, r/m32
    UD1_GdEd,
    // [0F 0B] UD2
    UD2,

    // [F3 0F AE /6] UMONITOR r16/r32/r64
    UMONITOR_E,

    // [F2 0F AE /6] UMWAIT r32
    UMWAIT_Gd,

    // [66 0F 15 /r] UNPCKHPD xmm1, xmm2/m128
    UNPCKHPD_VdqWdq,
    // [VEX.128.66.0F.WIG 15 /r] VUNPCKHPD xmm1, xmm2, xmm3/m128
    VUNPCKHPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 15 /r] VUNPCKHPD ymm1, ymm2, ymm3/m256
    VUNPCKHPD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W1 15 /r] VUNPCKHPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VUNPCKHPD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 15 /r] VUNPCKHPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VUNPCKHPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 15 /r] VUNPCKHPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VUNPCKHPD_VdqqHdqqWdqq_E512,

    // [NP 0F 15 /r] UNPCKHPS xmm1, xmm2/m128
    UNPCKHPS_VdqWdq,
    // [VEX.128.0F.WIG 15 /r] VUNPCKHPS xmm1, xmm2, xmm3/m128
    VUNPCKHPS_VdqHdqWdq_V128,
    // [VEX.256.0F.WIG 15 /r] VUNPCKHPS ymm1, ymm2, ymm3/m256
    VUNPCKHPS_VqqHqqWqq_V256,
    // [EVEX.128.0F.W1 15 /r] VUNPCKHPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VUNPCKHPS_VdqHdqWdq_E128,
    // [EVEX.256.0F.W1 15 /r] VUNPCKHPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VUNPCKHPS_VqqHqqWqq_E256,
    // [EVEX.512.0F.W1 15 /r] VUNPCKHPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VUNPCKHPS_VdqqHdqqWdqq_E512,

    // [66 0F 14 /r] UNPCKLPD xmm1, xmm2/m128
    UNPCKLPD_VdqWdq,
    // [VEX.128.66.0F.WIG 14 /r] VUNPCKLPD xmm1, xmm2, xmm3/m128
    VUNPCKLPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 14 /r] VUNPCKLPD ymm1, ymm2, ymm3/m256
    VUNPCKLPD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W1 14 /r] VUNPCKLPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VUNPCKLPD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 14 /r] VUNPCKLPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VUNPCKLPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 14 /r] VUNPCKLPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VUNPCKLPD_VdqqHdqqWdqq_E512,

    // [NP 0F 14 /r] UNPCKLPS xmm1, xmm2/m128
    UNPCKLPS_VdqWdq,
    // [VEX.128.0F.WIG 14 /r] VUNPCKLPS xmm1, xmm2, xmm3/m128
    VUNPCKLPS_VdqHdqWdq_V128,
    // [VEX.256.0F.WIG 14 /r] VUNPCKLPS ymm1, ymm2, ymm3/m256
    VUNPCKLPS_VqqHqqWqq_V256,
    // [EVEX.128.0F.W1 14 /r] VUNPCKLPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VUNPCKLPS_VdqHdqWdq_E128,
    // [EVEX.256.0F.W1 14 /r] VUNPCKLPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VUNPCKLPS_VqqHqqWqq_E256,
    // [EVEX.512.0F.W1 14 /r] VUNPCKLPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VUNPCKLPS_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F3A.W0 03 /r ib] VALIGND xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst, imm8
    VALIGND_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 03 /r ib] VALIGND ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst, imm8
    VALIGND_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 03 /r ib] VALIGND zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst, imm8
    VALIGND_VdqqHdqqWdqqIb_E256,
    // [EVEX.128.66.0F3A.W1 03 /r ib] VALIGNQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    VALIGNQ_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 03 /r ib] VALIGNQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    VALIGNQ_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 03 /r ib] VALIGNQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    VALIGNQ_VdqqHdqqWdqqIb_E256,

    // [EVEX.128.66.0F38.W1 65 /r] VBLENDMPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VBLENDMPD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 65 /r] VBLENDMPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VBLENDMPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 65 /r] VBLENDMPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VBLENDMPD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W0 65 /r] VBLENDMPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VBLENDMPS_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 65 /r] VBLENDMPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VBLENDMPS_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 65 /r] VBLENDMPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VBLENDMPS_VdqqHdqqWdqq_E512,

    // [VEX.128.66.0F38.W0 18 /r] VBROADCASTSS xmm1, m32
    // [VEX.128.66.0F38.W0 18 /r] VBROADCASTSS xmm1, xmm2
    VBROADCASTSS_VdqWd_V128,
    // [VEX.256.66.0F38.W0 18 /r] VBROADCASTSS ymm1, m32
    // [VEX.128.66.0F38.W0 18 /r] VBROADCASTSS ymm1, xmm2
    VBROADCASTSS_VqqWd_V256,
    // [VEX.256.66.0F38.W0 19 /r] VBROADCASTSD ymm1, m64
    // [VEX.256.66.0F38.W0 19 /r] VBROADCASTSD ymm1, xmm2
    VBROADCASTSD_VqqWq_V256,
    // [VEX.256.66.0F38.W0 1A /r] VBROADCASTF128 ymm1, m128
    VBROADCASTF128_VqqMdq_V256,
    // [EVEX.256.66.0F38.W1 19 /r] VBROADCASTSD ymm1 {k1}{z}, xmm2/m64
    VBROADCASTSD_VqqWq_E256,
    // [EVEX.512.66.0F38.W1 19 /r] VBROADCASTSD zmm1 {k1}{z}, xmm2/m64
    VBROADCASTSD_VdqqWq_E512,
    // [EVEX.256.66.0F38.W0 19 /r] VBROADCASTF32X2 ymm1 {k1}{z}, xmm2/m64
    VBROADCASTF32X2_VqqWq_E256,
    // [EVEX.512.66.0F38.W0 19 /r] VBROADCASTF32X2 zmm1 {k1}{z}, xmm2/m64
    VBROADCASTF32X2_VdqqWq_E512,
    // [EVEX.128.66.0F38.W0 18 /r] VBROADCASTSS xmm1 {k1}{z}, xmm2/m32
    VBROADCASTSS_VdqWd_E128,
    // [EVEX.256.66.0F38.W0 18 /r] VBROADCASTSS ymm1 {k1}{z}, xmm2/m32
    VBROADCASTSS_VqqWd_E256,
    // [EVEX.512.66.0F38.W0 18 /r] VBROADCASTSS zmm1 {k1}{z}, xmm2/m32
    VBROADCASTSS_VdqqWd_E512,
    // [EVEX.256.66.0F38.W0 1A /r] VBROADCASTF32X4 ymm1 {k1}{z}, m128
    VBROADCASTF32X4_VqqWdq_E256,
    // [EVEX.512.66.0F38.W0 1A /r] VBROADCASTF32X4 zmm1 {k1}{z}, m128
    VBROADCASTF32X4_VdqqWdq_E512,
    // [EVEX.256.66.0F38.W1 1A /r] VBROADCASTF64X2 ymm1 {k1}{z}, m128
    VBROADCASTF64X2_VqqWdq_E256,
    // [EVEX.512.66.0F38.W1 1A /r] VBROADCASTF64X2 zmm1 {k1}{z}, m128
    VBROADCASTF64X2_VdqqWdq_E512,
    // [EVEX.512.66.0F38.W0 1B /r] VBROADCASTF32X8 zmm1 {k1}{z}, m256
    VBROADCASTF32X8_VdqqWqq_E512,
    // [EVEX.512.66.0F38.W1 1B /r] VBROADCASTF64X4 zmm1 {k1}{z}, m256
    VBROADCASTF64X4_VdqqWqq_E512,

    // [EVEX.128.66.0F38.W1 8A /r] VCOMPRESSPD xmm1/m128 {k1}{z}, xmm2
    VCOMPRESSPD_WdqVdq_E128,
    // [EVEX.256.66.0F38.W1 8A /r] VCOMPRESSPD ymm1/m256 {k1}{z}, xmm2
    VCOMPRESSPD_WqqVqq_E256,
    // [EVEX.512.66.0F38.W1 8A /r] VCOMPRESSPD zmm1/m512 {k1}{z}, xmm2
    VCOMPRESSPD_WdqqVdqq_E512,

    // [EVEX.128.66.0F38.W0 8A /r] VCOMPRESSPS xmm1/m128 {k1}{z}, xmm2
    VCOMPRESSPS_WdqVdq_E128,
    // [EVEX.256.66.0F38.W0 8A /r] VCOMPRESSPS ymm1/m256 {k1}{z}, xmm2
    VCOMPRESSPS_WqqVqq_E256,
    // [EVEX.512.66.0F38.W0 8A /r] VCOMPRESSPS zmm1/m512 {k1}{z}, xmm2
    VCOMPRESSPS_WdqqVdqq_E512,

    // [EVEX.128.66.0F.W1 7B /r] VCVTPD2QQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    VCVTPD2QQ_VdqWdq_E128,
    // [EVEX.256.66.0F.W1 7B /r] VCVTPD2QQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    VCVTPD2QQ_VqqWqq_E256,
    // [EVEX.512.66.0F.W1 7B /r] VCVTPD2QQ zmm1 {k1}{z}, zmm2/m512/m64bcst
    VCVTPD2QQ_VdqqWdqq_E512,

    // [EVEX.128.0F.W1 79 /r] VCVTPD2UDQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    VCVTPD2UDQ_VdqWdq_E128,
    // [EVEX.256.0F.W1 79 /r] VCVTPD2UDQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    VCVTPD2UDQ_VqqWqq_E256,
    // [EVEX.512.0F.W1 79 /r] VCVTPD2UDQ zmm1 {k1}{z}, zmm2/m512/m64bcst
    VCVTPD2UDQ_VdqqWdqq_E512,

    // [EVEX.128.66.0F.W1 79 /r] VCVTPD2UQQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    VCVTPD2UQQ_VdqWdq_E128,
    // [EVEX.256.66.0F.W1 79 /r] VCVTPD2UQQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    VCVTPD2UQQ_VqqWqq_E256,
    // [EVEX.512.66.0F.W1 79 /r] VCVTPD2UQQ zmm1 {k1}{z}, zmm2/m512/m64bcst
    VCVTPD2UQQ_VdqqWdqq_E512,

    // [VEX.128.66.0F38.W0 13 /r] VCVTPH2PS xmm1, xmm2/m64
    VCVTPH2PS_VdqWq_V128,
    // [VEX.256.66.0F38.W0 13 /r] VCVTPH2PS ymm1, xmm2/m128
    VCVTPH2PS_VqqWdq_V256,
    // [EVEX.128.66.0F38.W0 13 /r] VCVTPH2PS xmm1 {k1}{z}, xmm2/m64
    VCVTPH2PS_VdqWq_E128,
    // [EVEX.256.66.0F38.W0 13 /r] VCVTPH2PS ymm1 {k1}{z}, xmm2/m128
    VCVTPH2PS_VqqWdq_E256,
    // [EVEX.512.66.0F38.W0 13 /r] VCVTPH2PS zmm1 {k1}{z}, ymm2/m256{sae}
    VCVTPH2PS_VdqqWqq_E512,

    // [VEX.128.66.0F3A.W0 1D /r ib] VCVTPS2PH xmm1/m64, xmm2, imm8
    VCVTPS2PH_WqVdqIb_V128,
    // [VEX.256.66.0F3A.W0 1D /r ib] VCVTPS2PH xmm1/m128, ymm2, imm8
    VCVTPS2PH_WdqVqqIb_V256,
    // [EVEX.128.66.0F3A.W0 1D /r ib] VCVTPS2PH xmm1/m64 {k1}{z}, xmm2, imm8
    VCVTPS2PH_WqVdqIb_E128,
    // [EVEX.256.66.0F3A.W0 1D /r ib] VCVTPS2PH xmm1/m128 {k1}{z}, ymm2, imm8
    VCVTPS2PH_WdqVqqIb_E256,
    // [EVEX.512.66.0F3A.W0 1D /r ib] VCVTPS2PH ymm1/m256 {k1}{z}, zmm2{sae}, imm8
    VCVTPS2PH_WqqVdqqIb_E512,

    // [EVEX.128.0F.W0 79 /r] VCVTPS2UDQ xmm1 {k1}{z}, xmm2/m128/m32bcst
    VCVTPS2UDQ_VdqWdq_E128,
    // [EVEX.256.0F.W0 79 /r] VCVTPS2UDQ ymm1 {k1}{z}, ymm2/m256/m32bcst
    VCVTPS2UDQ_VqqWqq_E256,
    // [EVEX.512.0F.W0 79 /r] VCVTPS2UDQ zmm1 {k1}{z}, zmm2/m512/m32bcst{er}
    VCVTPS2UDQ_VdqqWdqq_E512,

    // [EVEX.128.66.0F.W0 7B /r] VCVTPS2QQ xmm1 {k1}{z}, xmm2/m64/m32bcst
    VCVTPS2QQ_VdqWq_E128,
    // [EVEX.256.66.0F.W0 7B /r] VCVTPS2QQ ymm1 {k1}{z}, xmm2/m128/m32bcst
    VCVTPS2QQ_VqqWdq_E256,
    // [EVEX.512.66.0F.W0 7B /r] VCVTPS2QQ zmm1 {k1}{z}, ymm2/m256/m32bcst{er}
    VCVTPS2QQ_VdqqWqq_E512,

    // [EVEX.128.66.0F.W0 79 /r] VCVTPS2UQQ xmm1 {k1}{z}, xmm2/m64/m32bcst
    VCVTPS2UQQ_VdqWq_E128,
    // [EVEX.256.66.0F.W0 79 /r] VCVTPS2UQQ ymm1 {k1}{z}, xmm2/m128/m32bcst
    VCVTPS2UQQ_VqqWdq_E256,
    // [EVEX.512.66.0F.W0 79 /r] VCVTPS2UQQ zmm1 {k1}{z}, ymm2/m256/m32bcst{er}
    VCVTPS2UQQ_VdqqWqq_E512,

    // [EVEX.128.F3.0F.W1 E6 /r] VCVTQQ2PD xmm1 {k1}{z}, xmm2/m128/m64bcst
    VCVTQQ2PD_VdqWdq_E128,
    // [EVEX.256.F3.0F.W1 E6 /r] VCVTQQ2PD ymm1 {k1}{z}, ymm2/m256/m64bcst
    VCVTQQ2PD_VqqWqq_E256,
    // [EVEX.512.F3.0F.W1 E6 /r] VCVTQQ2PD zmm1 {k1}{z}, zmm2/m512/m64bcst{er}
    VCVTQQ2PD_VdqqWdqq_E512,

    // [EVEX.128.0F.W1 5B /r] VCVTQQ2PS xmm1 {k1}{z}, xmm2/m128/m64bcst
    VCVTQQ2PS_VdqWdq_E128,
    // [EVEX.256.0F.W1 5B /r] VCVTQQ2PS ymm1 {k1}{z}, ymm2/m256/m64bcst
    VCVTQQ2PS_VqqWqq_E256,
    // [EVEX.512.0F.W1 5B /r] VCVTQQ2PS zmm1 {k1}{z}, zmm2/m512/m64bcst{er}
    VCVTQQ2PS_VdqqWdqq_E512,

    // [EVEX.LIG.F2.0F.W0 79 /r] VCVTSD2USI r32, xmm1/m64{er}
    VCVTSD2USI_GdVq_E,
    // [EVEX.LIG.F2.0F.W1 79 /r] VCVTSD2USI r64, xmm1/m64{er}
    VCVTSD2USI_GqVq_E,

    // [EVEX.LIG.F3.0F.W0 79 /r] VCVTSS2USI r32, xmm1/m32{er}
    VCVTSS2USI_GdVd_E,
    // [EVEX.LIG.F3.0F.W1 79 /r] VCVTSS2USI r64, xmm1/m32{er}
    VCVTSS2USI_GqVd_E,

    // [EVEX.128.66.0F.W1 7A /r] VCVTTPD2QQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    VCVTTPD2QQ_VdqWdq_E128,
    // [EVEX.256.66.0F.W1 7A /r] VCVTTPD2QQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    VCVTTPD2QQ_VqqWqq_E256,
    // [EVEX.512.66.0F.W1 7A /r] VCVTTPD2QQ zmm1 {k1}{z}, zmm2/m512/m64bcst{sae}
    VCVTTPD2QQ_VdqqWdqq_E512,

    // [EVEX.128.0F.W1 78 /r] VCVTTPD2UDQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    VCVTTPD2UDQ_VdqWdq_E128,
    // [EVEX.256.0F.W1 78 /r] VCVTTPD2UDQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    // NOTE: Intel manual lists the opcode as `... 78 02 /r`
    VCVTTPD2UDQ_VqqWqq_E256,
    // [EVEX.512.0F.W1 78 /r] VCVTTPD2UDQ zmm1 {k1}{z}, zmm2/m512/m64bcst{sae}
    VCVTTPD2UDQ_VdqqWdqq_E512,

    // [EVEX.128.66.0F.W1 78 /r] VCVTTPD2UQQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    VCVTTPD2UQQ_VdqWdq_E128,
    // [EVEX.256.66.0F.W1 78 /r] VCVTTPD2UQQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    VCVTTPD2UQQ_VqqWqq_E256,
    // [EVEX.512.66.0F.W1 78 /r] VCVTTPD2UQQ zmm1 {k1}{z}, zmm2/m512/m64bcst{sae}
    VCVTTPD2UQQ_VdqqWdqq_E512,

    // [EVEX.128.66.0F.W1 7A /r] VCVTTPS2QQ xmm1 {k1}{z}, xmm2/m64/m32bcst
    VCVTTPS2QQ_VdqWq_E128,
    // [EVEX.256.66.0F.W1 7A /r] VCVTTPS2QQ ymm1 {k1}{z}, ymm2/m128/m32bcst
    VCVTTPS2QQ_VqqWdq_E256,
    // [EVEX.512.66.0F.W1 7A /r] VCVTTPS2QQ zmm1 {k1}{z}, zmm2/m256/m32bcst{sae}
    VCVTTPS2QQ_VdqqWqq_E512,

    // [EVEX.128.66.0F.W1 78 /r] VCVTTPS2UQQ xmm1 {k1}{z}, xmm2/m64/m32bcst
    VCVTTPS2UQQ_VdqWq_E128,
    // [EVEX.256.66.0F.W1 78 /r] VCVTTPS2UQQ ymm1 {k1}{z}, ymm2/m128/m32bcst
    VCVTTPS2UQQ_VqqWdq_E256,
    // [EVEX.512.66.0F.W1 78 /r] VCVTTPS2UQQ zmm1 {k1}{z}, zmm2/m256/m32bcst{sae}
    VCVTTPS2UQQ_VdqqWqq_E512,

    // [EVEX.LIG.F2.0F.W0 78 /r] VCVTTSD2USI r32, xmm1/m64{sae}
    VCVTTSD2USI_GdWq_E,
    // [EVEX.LIG.F2.0F.W1 78 /r] VCVTTSD2USI r64, xmm1/m64{sae}
    VCVTTSD2USI_GqWq_E,

    // [EVEX.LIG.F3.0F.W0 78 /r] VCVTTSS2USI r32, xmm1/m64{sae}
    VCVTTSS2USI_GdWq_E,
    // [EVEX.LIG.F3.0F.W1 78 /r] VCVTTSS2USI r64, xmm1/m64{sae}
    VCVTTSS2USI_GqWq_E,

    // [EVEX.128.F3.0F.W0 7A /r] VCVTUDQ2PD xmm1 {k1}{z}, xmm2/m64/m32bcst
    VCVTUDQ2PD_VdqWq_E128,
    // [EVEX.256.F3.0F.W0 7A /r] VCVTUDQ2PD ymm1 {k1}{z}, xmm2/m128/m32bcst
    VCVTUDQ2PD_VqqWdq_E256,
    // [EVEX.512.F3.0F.W0 7A /r] VCVTUDQ2PD zmm1 {k1}{z}, ymm2/m256/m32bcst
    VCVTUDQ2PD_VdqqWqq_E512,

    // [EVEX.128.F2.0F.W0 7A /r] VCVTUDQ2PS xmm1 {k1}{z}, xmm2/m128/m32bcst
    VCVTUDQ2PS_VdqWdq_E128,
    // [EVEX.256.F2.0F.W0 7A /r] VCVTUDQ2PS ymm1 {k1}{z}, xmm2/m256/m32bcst
    VCVTUDQ2PS_VqqWqq_E256,
    // [EVEX.512.F2.0F.W0 7A /r] VCVTUDQ2PS zmm1 {k1}{z}, ymm2/m512/m32bcst{er}
    VCVTUDQ2PS_VdqqWdqq_E512,

    // [EVEX.128.F3.0F.W1 7A /r] VCVTUQQ2PD xmm1 {k1}{z}, xmm2/m128/m64bcst
    VCVTUQQ2PD_VdqWdq_E128,
    // [EVEX.256.F3.0F.W1 7A /r] VCVTUQQ2PD ymm1 {k1}{z}, xmm2/m256/m64bcst
    VCVTUQQ2PD_VqqWqq_E256,
    // [EVEX.512.F3.0F.W1 7A /r] VCVTUQQ2PD zmm1 {k1}{z}, ymm2/m512/m64bcst{er]
    VCVTUQQ2PD_VdqqWdqq_E512,

    // [EVEX.128.F2.0F.W1 7A /r] VCVTUQQ2PS xmm1 {k1}{z}, xmm2/m128/m64bcst
    VCVTUQQ2PS_VdqWdq_E128,
    // [EVEX.256.F2.0F.W1 7A /r] VCVTUQQ2PS ymm1 {k1}{z}, xmm2/m256/m64bcst
    VCVTUQQ2PS_VqqWqq_E256,
    // [EVEX.512.F2.0F.W1 7A /r] VCVTUQQ2PS zmm1 {k1}{z}, ymm2/m512/m64bcst[er}
    VCVTUQQ2PS_VdqqWdqq_E512,

    // [EVEX.LIG.F2.0F.W0 7B /r] VCVTUSI2SD xmm1, xmm2, r/m32
    VCVTUSI2SD_VdqHdqWd_E,
    // [EVEX.LIG.F2.0F.W1 7B /r] VCVTUSI2SD xmm1, xmm2, r/m64{er}
    VCVTUSI2SD_VdqHdqWq_E,

    // [EVEX.LIG.F3.0F.W0 7B /r] VCVTUSI2SS xmm1, xmm2, r/m32
    VCVTUSI2SS_VdqHdqWd_E,
    // [EVEX.LIG.F3.0F.W1 7B /r] VCVTUSI2SS xmm1, xmm2, r/m64{er}
    VCVTUSI2SS_VdqHdqWq_E,

    // [EVEX.128.66.0F3A.W0 42 /r ib] VDBPSADBW xmm1 {k1}{z}, xmm2, xmm3/m128, imm8
    VDBPSADBW_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 42 /r ib] VDBPSADBW ymm1 {k1}{z}, ymm2, ymm3/m256, imm8
    VDBPSADBW_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 42 /r ib] VDBPSADBW zmm1 {k1}{z}, zmm2, zmm3/m512, imm8
    VDBPSADBW_VdqqHdqqWdqqIb_E512,

    // [EVEX.128.66.0F38.W1 88 /r] VEXPANDPD xmm1 {k1}{z}, xmm2/m128
    VEXPANDPD_VdqWdq_E128,
    // [EVEX.256.66.0F38.W1 88 /r] VEXPANDPD ymm1 {k1}{z}, ymm2/m256
    VEXPANDPD_VqqWqq_E256,
    // [EVEX.512.66.0F38.W1 88 /r] VEXPANDPD zmm1 {k1}{z}, zmm2/m512
    VEXPANDPD_VdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 88 /r] VEXPANDPS xmm1 {k1}{z}, xmm2/m128
    VEXPANDPS_VdqWdq_E128,
    // [EVEX.256.66.0F38.W0 88 /r] VEXPANDPS ymm1 {k1}{z}, ymm2/m256
    VEXPANDPS_VqqWqq_E256,
    // [EVEX.512.66.0F38.W0 88 /r] VEXPANDPS zmm1 {k1}{z}, zmm2/m512
    VEXPANDPS_VdqqWdqq_E512,

    // [0F 00 /4] VERR r/m16
    VERR_Ew,
    // [0F 00 /5] VERW r/m16
    VERW_Ew,

    // [VEX.256.66.0F3A.W0 19 /r ib] VEXTRACTF128 xmm1/m128, ymm2, imm8
    VEXTRACTF128_WdqVqqIb_V256,
    // [EVEX.256.66.0F3A.W0 19 /r ib] VEXTRACTF32X4 xmm1/m128 {k1}{z}, ymm2, imm8
    VEXTRACTF32X4_WdqVqqIb_E256,
    // [EVEX.512.66.0F3A.W0 19 /r ib] VEXTRACTF32X4 xmm1/m128 {k1}{z}, zmm2, imm8
    VEXTRACTF32X4_WdqWdqqIb_E512,
    // [EVEX.256.66.0F3A.W1 19 /r ib] VEXTRACTF64X2 xmm1/m128 {k1}{z}, ymm2, imm8
    VEXTRACTF64X2_WdqVqqIb_E256,
    // [EVEX.512.66.0F3A.W1 19 /r ib] VEXTRACTF64X2 xmm1/m128 {k1}{z}, zmm2, imm8
    VEXTRACTF64X2_WdqVdqqIb_E512,
    // [EVEX.512.66.0F3A.W0 1B /r ib] VEXTRACTF32X8 ymm1/m256 {k1}{z}, zmm2, imm8
    VEXTRACTF32X8_WqqVdqqIb_E512,
    // [EVEX.512.66.0F3A.W1 1B /r ib] VEXTRACTF64X4 ymm1/m256 {k1}{z}, zmm2, imm8
    VEXTRACTF64X4_WqqVdqqIb_E512,

    // [VEX.256.66.0F3A.W0 39 /r ib] VEXTRACTI128 xmm1/m128, ymm2, imm8
    VEXTRACTI128_WdqVqqIb_V256,
    // [EVEX.256.66.0F3A.W0 39 /r ib] VEXTRACTI32X4 xmm1/m128 {k1}{z}, ymm2, imm8
    VEXTRACTI32X4_WdqVqqIb_E256,
    // [EVEX.512.66.0F3A.W0 39 /r ib] VEXTRACTI32X4 xmm1/m128 {k1}{z}, zmm2, imm8
    VEXTRACTI32X4_WdqWdqqIb_E512,
    // [EVEX.256.66.0F3A.W1 39 /r ib] VEXTRACTI64X2 xmm1/m128 {k1}{z}, ymm2, imm8
    VEXTRACTI64X2_WdqVqqIb_E256,
    // [EVEX.512.66.0F3A.W1 39 /r ib] VEXTRACTI64X2 xmm1/m128 {k1}{z}, zmm2, imm8
    VEXTRACTI64X2_WdqVdqqIb_E512,
    // [EVEX.512.66.0F3A.W0 3B /r ib] VEXTRACTI32X8 ymm1/m256 {k1}{z}, zmm2, imm8
    VEXTRACTI32X8_WqqVdqqIb_E512,
    // [EVEX.512.66.0F3A.W1 3B /r ib] VEXTRACTI64X4 ymm1/m256 {k1}{z}, zmm2, imm8
    VEXTRACTI64X4_WqqVdqqIb_E512,

    // [EVEX.128.66.0F3A.W1 54 /r ib] VFIXUPIMMPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    VFIXUPIMMPD_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 54 /r ib] VFIXUPIMMPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    VFIXUPIMMPD_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 54 /r ib] VFIXUPIMMPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{sae}, imm8
    VFIXUPIMMPD_VdqqHdqqWdqqIb_E512,

    // [EVEX.128.66.0F3A.W0 54 /r ib] VFIXUPIMMPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst, imm8
    VFIXUPIMMPS_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 54 /r ib] VFIXUPIMMPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst, imm8
    VFIXUPIMMPS_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 54 /r ib] VFIXUPIMMPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{sae}, imm8
    VFIXUPIMMPS_VdqqHdqqWdqqIb_E512,

    // [EVEX.LIG.66.0F3A.W1 55 /r ib] VFIXUPIMMSD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}, imm8
    VFIXUPIMMSD_VdqHdqWqIb_E,

    // [EVEX.LIG.66.0F3A.W0 55 /r ib] VFIXUPIMMSS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}, imm8
    VFIXUPIMMSS_VdqHdqWdIb_E,

    // [VEX.128.66.0F38.W1 98 /r] VFMADD132PD xmm1, xmm2, xmm3/m128
    VFMADD132PD_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W1 A8 /r] VFMADD213PD xmm1, xmm2, xmm3/m128
    VFMADD213PD_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W1 B8 /r] VFMADD231PD xmm1, xmm2, xmm3/m128
    VFMADD231PD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W1 98 /r] VFMADD132PD ymm1, ymm2, ymm3/m256
    VFMADD132PD_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W1 A8 /r] VFMADD213PD ymm1, ymm2, ymm3/m256
    VFMADD213PD_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W1 B8 /r] VFMADD231PD ymm1, ymm2, ymm3/m256
    VFMADD231PD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W1 98 /r] VFMADD132PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFMADD132PD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W1 A8 /r] VFMADD213PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFMADD213PD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W1 B8 /r] VFMADD231PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFMADD231PD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 98 /r] VFMADD132PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFMADD132PD_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W1 A8 /r] VFMADD213PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFMADD213PD_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W1 B8 /r] VFMADD231PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFMADD231PD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 98 /r] VFMADD132PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFMADD132PD_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W1 A8 /r] VFMADD213PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFMADD213PD_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W1 B8 /r] VFMADD231PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFMADD231PD_VdqqHdqqWdqq_E512,

    // [VEX.128.66.0F38.W0 98 /r] VFMADD132PS xmm1, xmm2, xmm3/m128
    VFMADD132PS_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W0 A8 /r] VFMADD213PS xmm1, xmm2, xmm3/m128
    VFMADD213PS_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W0 B8 /r] VFMADD231PS xmm1, xmm2, xmm3/m128
    VFMADD231PS_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W0 98 /r] VFMADD132PS ymm1, ymm2, ymm3/m256
    VFMADD132PS_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W0 A8 /r] VFMADD213PS ymm1, ymm2, ymm3/m256
    VFMADD213PS_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W0 B8 /r] VFMADD231PS ymm1, ymm2, ymm3/m256
    VFMADD231PS_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W0 98 /r] VFMADD132PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFMADD132PS_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 A8 /r] VFMADD213PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFMADD213PS_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 B8 /r] VFMADD231PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFMADD231PS_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 98 /r] VFMADD132PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFMADD132PS_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W0 A8 /r] VFMADD213PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFMADD213PS_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W0 B8 /r] VFMADD231PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFMADD231PS_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 98 /r] VFMADD132PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFMADD132PS_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W0 A8 /r] VFMADD213PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFMADD213PS_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W0 B8 /r] VFMADD231PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFMADD231PS_VdqqHdqqWdqq_E512,

    // [VEX.LIG.66.0F38.W1 99 /r] VFMADD132SD xmm1, xmm2, xmm3/m64
    VFMADD132SD_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W1 A9 /r] VFMADD213SD xmm1, xmm2, xmm3/m64
    VFMADD213SD_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W1 B9 /r] VFMADD231SD xmm1, xmm2, xmm3/m64
    VFMADD231SD_VdqHdqWq_V,
    // [EVEX.LIG.66.0F38.W1 99 /r] VFMADD132SD xmm1, xmm2, xmm3/m64{er}
    VFMADD132SD_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W1 A9 /r] VFMADD213SD xmm1, xmm2, xmm3/m64{er}
    VFMADD213SD_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W1 B9 /r] VFMADD231SD xmm1, xmm2, xmm3/m64{er}
    VFMADD231SD_VdqHdqWq_E,

    // [VEX.LIG.66.0F38.W0 99 /r] VFMADD132SS xmm1, xmm2, xmm3/m64
    VFMADD132SS_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W0 A9 /r] VFMADD213SS xmm1, xmm2, xmm3/m64
    VFMADD213SS_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W0 B9 /r] VFMADD231SS xmm1, xmm2, xmm3/m64
    VFMADD231SS_VdqHdqWq_V,
    // [EVEX.LIG.66.0F38.W0 99 /r] VFMADD132SS xmm1, xmm2, xmm3/m64{er}
    VFMADD132SS_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W0 A9 /r] VFMADD213SS xmm1, xmm2, xmm3/m64{er}
    VFMADD213SS_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W0 B9 /r] VFMADD231SS xmm1, xmm2, xmm3/m64{er}
    VFMADD231SS_VdqHdqWq_E,

    // [VEX.128.66.0F38.W1 96 /r] VFMADDSUB132PD xmm1, xmm2, xmm3/m128
    VFMADDSUB132PD_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W1 A6 /r] VFMADDSUB213PD xmm1, xmm2, xmm3/m128
    VFMADDSUB213PD_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W1 B6 /r] VFMADDSUB231PD xmm1, xmm2, xmm3/m128
    VFMADDSUB231PD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W1 96 /r] VFMADDSUB132PD ymm1, ymm2, ymm3/m256
    VFMADDSUB132PD_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W1 A6 /r] VFMADDSUB213PD ymm1, ymm2, ymm3/m256
    VFMADDSUB213PD_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W1 B6 /r] VFMADDSUB231PD ymm1, ymm2, ymm3/m256
    VFMADDSUB231PD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W1 96 /r] VFMADDSUB132PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFMADDSUB132PD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W1 A6 /r] VFMADDSUB213PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFMADDSUB213PD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W1 B6 /r] VFMADDSUB231PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFMADDSUB231PD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 96 /r] VFMADDSUB132PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFMADDSUB132PD_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W1 A6 /r] VFMADDSUB213PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFMADDSUB213PD_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W1 B6 /r] VFMADDSUB231PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFMADDSUB231PD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 96 /r] VFMADDSUB132PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFMADDSUB132PD_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W1 A6 /r] VFMADDSUB213PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFMADDSUB213PD_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W1 B6 /r] VFMADDSUB231PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFMADDSUB231PD_VdqqHdqqWdqq_E512,

    // [VEX.128.66.0F38.W0 96 /r] VFMADDSUB132PS xmm1, xmm2, xmm3/m128
    VFMADDSUB132PS_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W0 A6 /r] VFMADDSUB213PS xmm1, xmm2, xmm3/m128
    VFMADDSUB213PS_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W0 B6 /r] VFMADDSUB231PS xmm1, xmm2, xmm3/m128
    VFMADDSUB231PS_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W0 96 /r] VFMADDSUB132PS ymm1, ymm2, ymm3/m256
    VFMADDSUB132PS_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W0 A6 /r] VFMADDSUB213PS ymm1, ymm2, ymm3/m256
    VFMADDSUB213PS_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W0 B6 /r] VFMADDSUB231PS ymm1, ymm2, ymm3/m256
    VFMADDSUB231PS_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W0 96 /r] VFMADDSUB132PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFMADDSUB132PS_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 A6 /r] VFMADDSUB213PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFMADDSUB213PS_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 B6 /r] VFMADDSUB231PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFMADDSUB231PS_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 96 /r] VFMADDSUB132PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFMADDSUB132PS_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W0 A6 /r] VFMADDSUB213PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFMADDSUB213PS_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W0 B6 /r] VFMADDSUB231PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFMADDSUB231PS_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 96 /r] VFMADDSUB132PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFMADDSUB132PS_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W0 A6 /r] VFMADDSUB213PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFMADDSUB213PS_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W0 B6 /r] VFMADDSUB231PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFMADDSUB231PS_VdqqHdqqWdqq_E512,

    // [VEX.128.66.0F38.W1 97 /r] VFMSUBADD132PD xmm1, xmm2, xmm3/m128
    VFMSUBADD132PD_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W1 A7 /r] VFMSUBADD213PD xmm1, xmm2, xmm3/m128
    VFMSUBADD213PD_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W1 B7 /r] VFMSUBADD231PD xmm1, xmm2, xmm3/m128
    VFMSUBADD231PD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W1 97 /r] VFMSUBADD132PD ymm1, ymm2, ymm3/m256
    VFMSUBADD132PD_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W1 A7 /r] VFMSUBADD213PD ymm1, ymm2, ymm3/m256
    VFMSUBADD213PD_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W1 B7 /r] VFMSUBADD231PD ymm1, ymm2, ymm3/m256
    VFMSUBADD231PD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W1 97 /r] VFMSUBADD132PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFMSUBADD132PD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W1 A7 /r] VFMSUBADD213PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFMSUBADD213PD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W1 B7 /r] VFMSUBADD231PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFMSUBADD231PD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 97 /r] VFMSUBADD132PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFMSUBADD132PD_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W1 A7 /r] VFMSUBADD213PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFMSUBADD213PD_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W1 B7 /r] VFMSUBADD231PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFMSUBADD231PD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 97 /r] VFMSUBADD132PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFMSUBADD132PD_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W1 A7 /r] VFMSUBADD213PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFMSUBADD213PD_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W1 B7 /r] VFMSUBADD231PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFMSUBADD231PD_VdqqHdqqWdqq_E512,

    // [VEX.128.66.0F38.W0 97 /r] VFMSUBADD132PS xmm1, xmm2, xmm3/m128
    VFMSUBADD132PS_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W0 A7 /r] VFMSUBADD213PS xmm1, xmm2, xmm3/m128
    VFMSUBADD213PS_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W0 B7 /r] VFMSUBADD231PS xmm1, xmm2, xmm3/m128
    VFMSUBADD231PS_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W0 97 /r] VFMSUBADD132PS ymm1, ymm2, ymm3/m256
    VFMSUBADD132PS_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W0 A7 /r] VFMSUBADD213PS ymm1, ymm2, ymm3/m256
    VFMSUBADD213PS_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W0 B7 /r] VFMSUBADD231PS ymm1, ymm2, ymm3/m256
    VFMSUBADD231PS_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W0 97 /r] VFMSUBADD132PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFMSUBADD132PS_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 A7 /r] VFMSUBADD213PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFMSUBADD213PS_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 B7 /r] VFMSUBADD231PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFMSUBADD231PS_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 97 /r] VFMSUBADD132PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFMSUBADD132PS_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W0 A7 /r] VFMSUBADD213PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFMSUBADD213PS_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W0 B7 /r] VFMSUBADD231PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFMSUBADD231PS_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 97 /r] VFMSUBADD132PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFMSUBADD132PS_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W0 A7 /r] VFMSUBADD213PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFMSUBADD213PS_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W0 B7 /r] VFMSUBADD231PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFMSUBADD231PS_VdqqHdqqWdqq_E512,

    // [VEX.128.66.0F38.W1 9A /r] VFMSUB132PD xmm1, xmm2, xmm3/m128
    VFMSUB132PD_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W1 AA /r] VFMSUB213PD xmm1, xmm2, xmm3/m128
    VFMSUB213PD_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W1 BA /r] VFMSUB231PD xmm1, xmm2, xmm3/m128
    VFMSUB231PD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W1 9A /r] VFMSUB132PD ymm1, ymm2, ymm3/m256
    VFMSUB132PD_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W1 AA /r] VFMSUB213PD ymm1, ymm2, ymm3/m256
    VFMSUB213PD_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W1 BA /r] VFMSUB231PD ymm1, ymm2, ymm3/m256
    VFMSUB231PD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W1 9A /r] VFMSUB132PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFMSUB132PD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W1 AA /r] VFMSUB213PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFMSUB213PD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W1 BA /r] VFMSUB231PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFMSUB231PD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 9A /r] VFMSUB132PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFMSUB132PD_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W1 AA /r] VFMSUB213PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFMSUB213PD_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W1 BA /r] VFMSUB231PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFMSUB231PD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 9A /r] VFMSUB132PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFMSUB132PD_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W1 AA /r] VFMSUB213PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFMSUB213PD_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W1 BA /r] VFMSUB231PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFMSUB231PD_VdqqHdqqWdqq_E512,

    // [VEX.128.66.0F38.W0 9A /r] VFMSUB132PS xmm1, xmm2, xmm3/m128
    VFMSUB132PS_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W0 AA /r] VFMSUB213PS xmm1, xmm2, xmm3/m128
    VFMSUB213PS_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W0 BA /r] VFMSUB231PS xmm1, xmm2, xmm3/m128
    VFMSUB231PS_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W0 9A /r] VFMSUB132PS ymm1, ymm2, ymm3/m256
    VFMSUB132PS_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W0 AA /r] VFMSUB213PS ymm1, ymm2, ymm3/m256
    VFMSUB213PS_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W0 BA /r] VFMSUB231PS ymm1, ymm2, ymm3/m256
    VFMSUB231PS_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W0 9A /r] VFMSUB132PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFMSUB132PS_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 AA /r] VFMSUB213PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFMSUB213PS_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 BA /r] VFMSUB231PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFMSUB231PS_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 9A /r] VFMSUB132PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFMSUB132PS_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W0 AA /r] VFMSUB213PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFMSUB213PS_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W0 BA /r] VFMSUB231PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFMSUB231PS_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 9A /r] VFMSUB132PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFMSUB132PS_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W0 AA /r] VFMSUB213PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFMSUB213PS_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W0 BA /r] VFMSUB231PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFMSUB231PS_VdqqHdqqWdqq_E512,

    // [VEX.LIG.66.0F38.W1 9B /r] VFMSUB132SD xmm1, xmm2, xmm3/m64
    VFMSUB132SD_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W1 AB /r] VFMSUB213SD xmm1, xmm2, xmm3/m64
    VFMSUB213SD_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W1 BB /r] VFMSUB231SD xmm1, xmm2, xmm3/m64
    VFMSUB231SD_VdqHdqWq_V,
    // [EVEX.LIG.66.0F38.W1 9B /r] VFMSUB132SD xmm1, xmm2, xmm3/m64{er}
    VFMSUB132SD_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W1 AB /r] VFMSUB213SD xmm1, xmm2, xmm3/m64{er}
    VFMSUB213SD_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W1 BB /r] VFMSUB231SD xmm1, xmm2, xmm3/m64{er}
    VFMSUB231SD_VdqHdqWq_E,

    // [VEX.LIG.66.0F38.W0 9B /r] VFMSUB132SS xmm1, xmm2, xmm3/m64
    VFMSUB132SS_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W0 AB /r] VFMSUB213SS xmm1, xmm2, xmm3/m64
    VFMSUB213SS_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W0 BB /r] VFMSUB231SS xmm1, xmm2, xmm3/m64
    VFMSUB231SS_VdqHdqWq_V,
    // [EVEX.LIG.66.0F38.W0 9B /r] VFMSUB132SS xmm1, xmm2, xmm3/m64{er}
    VFMSUB132SS_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W0 AB /r] VFMSUB213SS xmm1, xmm2, xmm3/m64{er}
    VFMSUB213SS_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W0 BB /r] VFMSUB231SS xmm1, xmm2, xmm3/m64{er}
    VFMSUB231SS_VdqHdqWq_E,

    // [VEX.128.66.0F38.W1 9C /r] VFNMADD132PD xmm1, xmm2, xmm3/m128
    VFNMADD132PD_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W1 AC /r] VFNMADD213PD xmm1, xmm2, xmm3/m128
    VFNMADD213PD_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W1 BC /r] VFNMADD231PD xmm1, xmm2, xmm3/m128
    VFNMADD231PD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W1 9C /r] VFNMADD132PD ymm1, ymm2, ymm3/m256
    VFNMADD132PD_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W1 AC /r] VFNMADD213PD ymm1, ymm2, ymm3/m256
    VFNMADD213PD_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W1 BC /r] VFNMADD231PD ymm1, ymm2, ymm3/m256
    VFNMADD231PD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W1 9C /r] VFNMADD132PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFNMADD132PD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W1 AC /r] VFNMADD213PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFNMADD213PD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W1 BC /r] VFNMADD231PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFNMADD231PD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 9C /r] VFNMADD132PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFNMADD132PD_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W1 AC /r] VFNMADD213PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFNMADD213PD_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W1 BC /r] VFNMADD231PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFNMADD231PD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 9C /r] VFNMADD132PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFNMADD132PD_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W1 AC /r] VFNMADD213PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFNMADD213PD_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W1 BC /r] VFNMADD231PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFNMADD231PD_VdqqHdqqWdqq_E512,

    // [VEX.128.66.0F38.W0 9C /r] VFNMADD132PS xmm1, xmm2, xmm3/m128
    VFNMADD132PS_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W0 AC /r] VFNMADD213PS xmm1, xmm2, xmm3/m128
    VFNMADD213PS_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W0 BC /r] VFNMADD231PS xmm1, xmm2, xmm3/m128
    VFNMADD231PS_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W0 9C /r] VFNMADD132PS ymm1, ymm2, ymm3/m256
    VFNMADD132PS_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W0 AC /r] VFNMADD213PS ymm1, ymm2, ymm3/m256
    VFNMADD213PS_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W0 BC /r] VFNMADD231PS ymm1, ymm2, ymm3/m256
    VFNMADD231PS_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W0 9C /r] VFNMADD132PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFNMADD132PS_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 AC /r] VFNMADD213PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFNMADD213PS_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 BC /r] VFNMADD231PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFNMADD231PS_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 9C /r] VFNMADD132PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFNMADD132PS_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W0 AC /r] VFNMADD213PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFNMADD213PS_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W0 BC /r] VFNMADD231PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFNMADD231PS_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 9C /r] VFNMADD132PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFNMADD132PS_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W0 AC /r] VFNMADD213PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFNMADD213PS_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W0 BC /r] VFNMADD231PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFNMADD231PS_VdqqHdqqWdqq_E512,

    // [VEX.LIG.66.0F38.W1 9D /r] VFNMADD132SD xmm1, xmm2, xmm3/m64
    VFNMADD132SD_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W1 AD /r] VFNMADD213SD xmm1, xmm2, xmm3/m64
    VFNMADD213SD_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W1 BD /r] VFNMADD231SD xmm1, xmm2, xmm3/m64
    VFNMADD231SD_VdqHdqWq_V,
    // [EVEX.LIG.66.0F38.W1 9D /r] VFNMADD132SD xmm1, xmm2, xmm3/m64{er}
    VFNMADD132SD_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W1 AD /r] VFNMADD213SD xmm1, xmm2, xmm3/m64{er}
    VFNMADD213SD_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W1 BD /r] VFNMADD231SD xmm1, xmm2, xmm3/m64{er}
    VFNMADD231SD_VdqHdqWq_E,

    // [VEX.LIG.66.0F38.W0 9D /r] VFNMADD132SS xmm1, xmm2, xmm3/m64
    VFNMADD132SS_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W0 AD /r] VFNMADD213SS xmm1, xmm2, xmm3/m64
    VFNMADD213SS_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W0 BD /r] VFNMADD231SS xmm1, xmm2, xmm3/m64
    VFNMADD231SS_VdqHdqWq_V,
    // [EVEX.LIG.66.0F38.W0 9D /r] VFNMADD132SS xmm1, xmm2, xmm3/m64{er}
    VFNMADD132SS_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W0 AD /r] VFNMADD213SS xmm1, xmm2, xmm3/m64{er}
    VFNMADD213SS_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W0 BD /r] VFNMADD231SS xmm1, xmm2, xmm3/m64{er}
    VFNMADD231SS_VdqHdqWq_E,

    // [VEX.128.66.0F38.W1 9E /r] VFNMSUB132PD xmm1, xmm2, xmm3/m128
    VFNMSUB132PD_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W1 AE /r] VFNMSUB213PD xmm1, xmm2, xmm3/m128
    VFNMSUB213PD_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W1 BE /r] VFNMSUB231PD xmm1, xmm2, xmm3/m128
    VFNMSUB231PD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W1 9E /r] VFNMSUB132PD ymm1, ymm2, ymm3/m256
    VFNMSUB132PD_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W1 AE /r] VFNMSUB213PD ymm1, ymm2, ymm3/m256
    VFNMSUB213PD_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W1 BE /r] VFNMSUB231PD ymm1, ymm2, ymm3/m256
    VFNMSUB231PD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W1 9E /r] VFNMSUB132PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFNMSUB132PD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W1 AE /r] VFNMSUB213PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFNMSUB213PD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W1 BE /r] VFNMSUB231PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VFNMSUB231PD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 9E /r] VFNMSUB132PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFNMSUB132PD_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W1 AE /r] VFNMSUB213PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFNMSUB213PD_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W1 BE /r] VFNMSUB231PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VFNMSUB231PD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 9E /r] VFNMSUB132PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFNMSUB132PD_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W1 AE /r] VFNMSUB213PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFNMSUB213PD_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W1 BE /r] VFNMSUB231PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst{er}
    VFNMSUB231PD_VdqqHdqqWdqq_E512,

    // [VEX.128.66.0F38.W0 9E /r] VFNMSUB132PS xmm1, xmm2, xmm3/m128
    VFNMSUB132PS_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W0 AE /r] VFNMSUB213PS xmm1, xmm2, xmm3/m128
    VFNMSUB213PS_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W0 BE /r] VFNMSUB231PS xmm1, xmm2, xmm3/m128
    VFNMSUB231PS_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W0 9E /r] VFNMSUB132PS ymm1, ymm2, ymm3/m256
    VFNMSUB132PS_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W0 AE /r] VFNMSUB213PS ymm1, ymm2, ymm3/m256
    VFNMSUB213PS_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W0 BE /r] VFNMSUB231PS ymm1, ymm2, ymm3/m256
    VFNMSUB231PS_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W0 9E /r] VFNMSUB132PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFNMSUB132PS_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 AE /r] VFNMSUB213PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFNMSUB213PS_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 BE /r] VFNMSUB231PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VFNMSUB231PS_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 9E /r] VFNMSUB132PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFNMSUB132PS_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W0 AE /r] VFNMSUB213PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFNMSUB213PS_VqqHqqWqq_E256,
    // [EVEX.256.66.0F38.W0 BE /r] VFNMSUB231PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VFNMSUB231PS_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 9E /r] VFNMSUB132PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFNMSUB132PS_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W0 AE /r] VFNMSUB213PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFNMSUB213PS_VdqqHdqqWdqq_E512,
    // [EVEX.512.66.0F38.W0 BE /r] VFNMSUB231PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VFNMSUB231PS_VdqqHdqqWdqq_E512,

    // [VEX.LIG.66.0F38.W1 9F /r] VFNMSUB132SD xmm1, xmm2, xmm3/m64
    VFNMSUB132SD_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W1 AF /r] VFNMSUB213SD xmm1, xmm2, xmm3/m64
    VFNMSUB213SD_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W1 BF /r] VFNMSUB231SD xmm1, xmm2, xmm3/m64
    VFNMSUB231SD_VdqHdqWq_V,
    // [EVEX.LIG.66.0F38.W1 9F /r] VFNMSUB132SD xmm1, xmm2, xmm3/m64{er}
    VFNMSUB132SD_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W1 AF /r] VFNMSUB213SD xmm1, xmm2, xmm3/m64{er}
    VFNMSUB213SD_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W1 BF /r] VFNMSUB231SD xmm1, xmm2, xmm3/m64{er}
    VFNMSUB231SD_VdqHdqWq_E,

    // [VEX.LIG.66.0F38.W0 9F /r] VFNMSUB132SS xmm1, xmm2, xmm3/m64
    VFNMSUB132SS_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W0 AF /r] VFNMSUB213SS xmm1, xmm2, xmm3/m64
    VFNMSUB213SS_VdqHdqWq_V,
    // [VEX.LIG.66.0F38.W0 BF /r] VFNMSUB231SS xmm1, xmm2, xmm3/m64
    VFNMSUB231SS_VdqHdqWq_V,
    // [EVEX.LIG.66.0F38.W0 9F /r] VFNMSUB132SS xmm1, xmm2, xmm3/m64{er}
    VFNMSUB132SS_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W0 AF /r] VFNMSUB213SS xmm1, xmm2, xmm3/m64{er}
    VFNMSUB213SS_VdqHdqWq_E,
    // [EVEX.LIG.66.0F38.W0 BF /r] VFNMSUB231SS xmm1, xmm2, xmm3/m64{er}
    VFNMSUB231SS_VdqHdqWq_E,

    // [EVEX.128.66.0F3A.W1 66 /r ib] VFPCLASSPD k2 {k1}, xmm1/m128/m64bcst, imm8
    VFPCLASSPD_KGbWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 66 /r ib] VFPCLASSPD k2 {k1}, ymm1/m256/m64bcst, imm8
    VFPCLASSPD_KGbWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 66 /r ib] VFPCLASSPD k2 {k1}, zmm1/m512/m64bcst, imm8
    VFPCLASSPD_KGbWdqqIb_E512,

    // [EVEX.128.66.0F3A.W0 66 /r ib] VFPCLASSPS k2 {k1}, xmm1/m128/m32bcst, imm8
    VFPCLASSPS_KGbWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 66 /r ib] VFPCLASSPS k2 {k1}, ymm1/m256/m32bcst, imm8
    VFPCLASSPS_KGbWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 66 /r ib] VFPCLASSPS k2 {k1}, zmm1/m512/m32bcst, imm8
    VFPCLASSPS_KGbWdqqIb_E512,

    // [EVEX.LIG.66.0F3A.W1 67 /r ib] VFPCLASSSD k2 {k1}, xmm2/m64, imm8
    VFPCLASSSD_KGbWqIb_E,

    // [EVEX.LIG.66.0F3A.W0 67 /r ib] VFPCLASSSS k2 {k1}, xmm2/m32, imm8
    VFPCLASSSS_KGbWdIb_E,

    // [VEX.128.66.0F38.W1 92 /r] VGATHERDPD xmm1, vm32x, xmm2
    VGATHERDPD_VdqVMdHdq_V128,
    // [VEX.128.66.0F38.W1 93 /r] VGATHERQPD xmm1, vm64x, xmm2
    VGATHERQPD_VdqVMqHdq_V128,
    // [VEX.256.66.0F38.W1 92 /r] VGATHERDPD ymm1, vm32y, ymm2
    VGATHERDPD_VqqVMdHqq_V256,
    // [VEX.256.66.0F38.W1 93 /r] VGATHERQPD ymm1, vm64y, ymm2
    VGATHERQPD_VqqVMqHqq_V256,

    // [VEX.128.66.0F38.W0 92 /r] VGATHERDPS xmm1, vm32x, xmm2
    VGATHERDPS_VdqVMdHdq_V128,
    // [VEX.128.66.0F38.W0 93 /r] VGATHERQPS xmm1, vm64x, xmm2
    VGATHERQPS_VdqVMqHdq_V128,
    // [VEX.256.66.0F38.W0 92 /r] VGATHERDPS ymm1, vm32y, ymm2
    VGATHERDPS_VqqVMdHqq_V256,
    // [VEX.256.66.0F38.W0 93 /r] VGATHERQPS ymm1, vm64y, ymm2
    VGATHERQPS_VqqVMqHqq_V256,

    // [EVEX.128.66.0F38.W0 92 /vsib] VGATHERDPS xmm1 {k1}{z}, vm32x
    VGATHERDPS_VdqVMd_E128,
    // [EVEX.256.66.0F38.W0 92 /vsib] VGATHERDPS ymm1 {k1}{z}, vm32y
    VGATHERDPS_VqqVMd_E256,
    // [EVEX.512.66.0F38.W0 92 /vsib] VGATHERDPS zmm1 {k1}{z}, vm32z
    VGATHERDPS_VdqqVMd_E512,
    // [EVEX.128.66.0F38.W1 92 /vsib] VGATHERDPD xmm1 {k1}{z}, vm32x
    VGATHERDPD_VdqVMd_E128,
    // [EVEX.256.66.0F38.W1 92 /vsib] VGATHERDPD ymm1 {k1}{z}, vm32y
    VGATHERDPD_VqqVMd_E256,
    // [EVEX.512.66.0F38.W1 92 /vsib] VGATHERDPD zmm1 {k1}{z}, vm32z
    VGATHERDPD_VdqqVMd_E512,

    // [EVEX.128.66.0F38.W0 93 /vsib] VGATHERQPS xmm1 {k1}{z}, vm64x
    VGATHERQPS_VdqVMq_E128,
    // [EVEX.256.66.0F38.W0 93 /vsib] VGATHERQPS ymm1 {k1}{z}, vm64y
    VGATHERQPS_VqqVMq_E256,
    // [EVEX.512.66.0F38.W0 93 /vsib] VGATHERQPS zmm1 {k1}{z}, vm64z
    VGATHERQPS_VdqqVMq_E512,
    // [EVEX.128.66.0F38.W1 93 /vsib] VGATHERQPD xmm1 {k1}{z}, vm64x
    VGATHERQPD_VdqVMq_E128,
    // [EVEX.256.66.0F38.W1 93 /vsib] VGATHERQPD ymm1 {k1}{z}, vm64y
    VGATHERQPD_VqqVMq_E256,
    // [EVEX.512.66.0F38.W1 93 /vsib] VGATHERQPD zmm1 {k1}{z}, vm64z
    VGATHERQPD_VdqqVMq_E512,

    // [EVEX.128.66.0F38.W1 42 /r] VGETEXPPD xmm1 {k1}{z}, xmm2/m128/m64bcst
    VGETEXPPD_VdqWdq_E128,
    // [EVEX.256.66.0F38.W1 42 /r] VGETEXPPD ymm1 {k1}{z}, ymm2/m256/m64bcst
    VGETEXPPD_VqqWqq_E256,
    // [EVEX.512.66.0F38.W1 42 /r] VGETEXPPD zmm1 {k1}{z}, zmm2/m512/m64bcst{er}
    VGETEXPPD_VdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 42 /r] VGETEXPPS xmm1 {k1}{z}, xmm2/m128/m32bcst
    VGETEXPPS_VdqWdq_E128,
    // [EVEX.256.66.0F38.W0 42 /r] VGETEXPPS ymm1 {k1}{z}, ymm2/m256/m32bcst
    VGETEXPPS_VqqWqq_E256,
    // [EVEX.512.66.0F38.W0 42 /r] VGETEXPPS zmm1 {k1}{z}, zmm2/m512/m32bcst{er}
    VGETEXPPS_VdqqWdqq_E512,

    // [EVEX.LIG.66.0F38.W1 43 /r] VGETEXPSD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}
    VGETEXPSD_VdqHdqWq_E,

    // [EVEX.LIG.66.0F38.W0 43 /r] VGETEXPSS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}
    VGETEXPSS_VdqHdqWd_E,

    // [EVEX.128.66.0F3A.W1 26 /r ib] VGETMANTPD xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VGETMANTPD_VdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 26 /r ib] VGETMANTPD ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VGETMANTPD_VqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 26 /r ib] VGETMANTPD zmm1 {k1}{z}, zmm2/m512/m64bcst{er}, imm8
    VGETMANTPD_VdqqWdqqIb_E512,

    // [EVEX.128.66.0F3A.W0 26 /r ib] VGETMANTPS xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VGETMANTPS_VdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 26 /r ib] VGETMANTPS ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VGETMANTPS_VqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 26 /r ib] VGETMANTPS zmm1 {k1}{z}, zmm2/m512/m64bcst{er}, imm8
    VGETMANTPS_VdqqWdqqIb_E512,

    // [EVEX.LIG.66.0F3A.W1 27 /r ib] VGETMANTSD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}, imm8
    VGETMANTSD_VdqHdqWqIb_E,

    // [EVEX.LIG.66.0F3A.W0 27 /r ib] VGETMANTSS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}, imm8
    VGETMANTSS_VdqHdqWdIb_E,

    // [VEX.256.66.0F3A.W0 18 /r ib] VINSERTF128 ymm1, ymm2, xmm3/m128, imm8
    VINSERTF128_VqqHqqWdqIb_V256,
    // [EVEX.256.66.0F3A.W0 18 /r ib] VINSERTF32X4 ymm1 {k1}{z}, ymm2, xmm3/m128, imm8
    VINSERTF32X4_VqqHqqWdqIb_E256,
    // [EVEX.512.66.0F3A.W0 18 /r ib] VINSERTF32X4 zmm1 {k1}{z}, zmm2, xmm3/m128, imm8
    VINSERTF32X4_VdqqHdqqWdqIb_E512,
    // [EVEX.256.66.0F3A.W1 18 /r ib] VINSERTF64X2 ymm1 {k1}{z}, ymm2, xmm3/m128, imm8
    VINSERTF64X2_VqqHqqWdqIb_E256,
    // [EVEX.512.66.0F3A.W1 18 /r ib] VINSERTF64X2 zmm1 {k1}{z}, zmm2, xmm3/m128, imm8
    VINSERTF64X2_VdqqHdqqWdqIb_E512,
    // [EVEX.512.66.0F3A.W0 1A /r ib] VINSERTF32X8 zmm1 {k1}{z}, zmm2, ymm3/m256, imm8
    VINSERTF32X8_VdqqHdqqWqqIb_E512,
    // [EVEX.512.66.0F3A.W1 1A /r ib] VINSERTF64X4 zmm1 {k1}{z}, zmm2, ymm3/m256, imm8
    VINSERTF64X4_VdqqHdqqWqqIb_E512,

    // [VEX.256.66.0F3A.W0 38 /r ib] VINSERTI128 ymm1, ymm2, xmm3/m128, imm8
    VINSERTI128_VqqHqqWdqIb_V256,
    // [EVEX.256.66.0F3A.W0 38 /r ib] VINSERTI32X4 ymm1 {k1}{z}, ymm2, xmm3/m128, imm8
    VINSERTI32X4_VqqHqqWdqIb_E256,
    // [EVEX.512.66.0F3A.W0 38 /r ib] VINSERTI32X4 zmm1 {k1}{z}, zmm2, xmm3/m128, imm8
    VINSERTI32X4_VdqqHdqqWdqIb_E512,
    // [EVEX.256.66.0F3A.W1 38 /r ib] VINSERTI64X2 ymm1 {k1}{z}, ymm2, xmm3/m128, imm8
    VINSERTI64X2_VqqHqqWdqIb_E256,
    // [EVEX.512.66.0F3A.W1 38 /r ib] VINSERTI64X2 zmm1 {k1}{z}, zmm2, xmm3/m128, imm8
    VINSERTI64X2_VdqqHdqqWdqIb_E512,
    // [EVEX.512.66.0F3A.W0 3A /r ib] VINSERTI32X8 zmm1 {k1}{z}, zmm2, ymm3/m256, imm8
    VINSERTI32X8_VdqqHdqqWqqIb_E512,
    // [EVEX.512.66.0F3A.W1 3A /r ib] VINSERTI64X4 zmm1 {k1}{z}, zmm2, ymm3/m256, imm8
    VINSERTI64X4_VdqqHdqqWqqIb_E512,

    // [VEX.128.66.0F38.W0 2C /r] VMASKMOVPS xmm1, xmm2, m128
    VMASKMOVPS_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W0 2C /r] VMASKMOVPS ymm1, ymm2, m256
    VMASKMOVPS_VqqHqqWqq_V256,
    // [VEX.128.66.0F38.W0 2D /r] VMASKMOVPD xmm1, xmm2, m128
    VMASKMOVPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W0 2D /r] VMASKMOVPD ymm1, ymm2, m256
    VMASKMOVPD_VqqHqqWqq_V256,
    // [VEX.128.66.0F38.W0 2E /r] VMASKMOVPS m128, xmm1, xmm2
    VMASKMOVPS_WdqHdqVdq_V128,
    // [VEX.256.66.0F38.W0 2E /r] VMASKMOVPS m256, ymm1, ymm2
    VMASKMOVPS_WqqHqqVqq_V256,
    // [VEX.128.66.0F38.W0 2F /r] VMASKMOVPD m128, xmm1, xmm2
    VMASKMOVPD_WdqHdqVdq_V128,
    // [VEX.256.66.0F38.W0 2F /r] VMASKMOVPD m256, ymm1, ymm2
    VMASKMOVPD_WqqHqqVqq_V256,

    // [VEX.128.66.0F3A.W0 02 /r ib] VPBLENDD xmm1, xmm2, xmm3/m128, imm8
    VPBLENDD_VdqHdqWdqIb_V128,
    // [VEX.256.66.0F3A.W0 02 /r ib] VPBLENDD ymm1, ymm2, ymm3/m256, imm8
    VPBLENDD_VqqHqqWqqIb_V256,

    // [EVEX.128.66.0F38.W0 66 /r] VPBLENDMB xmm1 {k1}{z}, xmm2, xmm3/m128
    VPBLENDMB_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 66 /r] VPBLENDMB ymm1 {k1}{z}, ymm2, ymm3/m256
    VPBLENDMB_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 66 /r] VPBLENDMB zmm1 {k1}{z}, zmm2, zmm3/m512
    VPBLENDMB_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 66 /r] VPBLENDMW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPBLENDMW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 66 /r] VPBLENDMW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPBLENDMW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 66 /r] VPBLENDMW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPBLENDMW_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 64 /r] VPBLENDMD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPBLENDMD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 64 /r] VPBLENDMD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPBLENDMD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 64 /r] VPBLENDMD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPBLENDMD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 64 /r] VPBLENDMQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPBLENDMQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 64 /r] VPBLENDMQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPBLENDMQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 64 /r] VPBLENDMQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPBLENDMQ_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 7A /r] VPBROADCASTB xmm1 {k1}{z}, r8
    VPBROADCASTB_VdqEb_E128,
    // [EVEX.256.66.0F38.W0 7A /r] VPBROADCASTB ymm1 {k1}{z}, r8
    VPBROADCASTB_VqqEb_E256,
    // [EVEX.512.66.0F38.W0 7A /r] VPBROADCASTB zmm1 {k1}{z}, r8
    VPBROADCASTB_VdqqEb_E512,
    // [EVEX.128.66.0F38.W0 7B /r] VPBROADCASTW xmm1 {k1}{z}, r16
    VPBROADCASTW_VdqEw_E128,
    // [EVEX.256.66.0F38.W0 7B /r] VPBROADCASTW ymm1 {k1}{z}, r16
    VPBROADCASTW_VqqEw_E256,
    // [EVEX.512.66.0F38.W0 7B /r] VPBROADCASTW zmm1 {k1}{z}, r16
    VPBROADCASTW_VdqqEw_E512,
    // [EVEX.128.66.0F38.W0 7C /r] VPBROADCASTD xmm1 {k1}{z}, r32
    VPBROADCASTD_VdqEd_E128,
    // [EVEX.256.66.0F38.W0 7C /r] VPBROADCASTD ymm1 {k1}{z}, r32
    VPBROADCASTD_VqqEd_E256,
    // [EVEX.512.66.0F38.W0 7C /r] VPBROADCASTD zmm1 {k1}{z}, r32
    VPBROADCASTD_VdqqEd_E512,
    // [EVEX.128.66.0F38.W1 7C /r] VPBROADCASTQ xmm1 {k1}{z}, r64
    VPBROADCASTQ_VdqEq_E128,
    // [EVEX.256.66.0F38.W1 7C /r] VPBROADCASTQ ymm1 {k1}{z}, r64
    VPBROADCASTQ_VqqEq_E256,
    // [EVEX.512.66.0F38.W1 7C /r] VPBROADCASTQ zmm1 {k1}{z}, r64
    VPBROADCASTQ_VdqqEq_E512,

    // [VEX.128.66.0F38.W0 78 /r] VPBROADCASTB xmm1, xmm2/m8
    VPBROADCASTB_VdqWb_V128,
    // [VEX.256.66.0F38.W0 78 /r] VPBROADCASTB ymm1, ymm2/m8
    VPBROADCASTB_VqqWb_V256,
    // [EVEX.128.66.0F38.W0 78 /r] VPBROADCASTB xmm1 {k1}{z}, xmm2/m8
    VPBROADCASTB_VdqWb_E128,
    // [EVEX.256.66.0F38.W0 78 /r] VPBROADCASTB ymm1 {k1}{z}, xmm2/m8
    VPBROADCASTB_VqqWb_E256,
    // [EVEX.512.66.0F38.W0 78 /r] VPBROADCASTB zmm1 {k1}{z}, xmm2/m8
    VPBROADCASTB_VdqqWb_E512,
    // [VEX.128.66.0F38.W0 79 /r] VPBROADCASTW xmm1, xmm2/m16
    VPBROADCASTW_VdqWw_V128,
    // [VEX.256.66.0F38.W0 79 /r] VPBROADCASTW ymm1, ymm2/m16
    VPBROADCASTW_VqqWw_V256,
    // [EVEX.128.66.0F38.W0 79 /r] VPBROADCASTW xmm1 {k1}{z}, xmm2/m16
    VPBROADCASTW_VdqWw_E128,
    // [EVEX.256.66.0F38.W0 79 /r] VPBROADCASTW ymm1 {k1}{z}, xmm2/m16
    VPBROADCASTW_VqqWw_E256,
    // [EVEX.512.66.0F38.W0 79 /r] VPBROADCASTW zmm1 {k1}{z}, xmm2/m16
    VPBROADCASTW_VdqqWw_E512,
    // [VEX.128.66.0F38.W0 58 /r] VPBROADCASTD xmm1, xmm2/m32
    VPBROADCASTD_VdqWd_V128,
    // [VEX.256.66.0F38.W0 58 /r] VPBROADCASTD ymm1, ymm2/m32
    VPBROADCASTD_VqqWd_V256,
    // [EVEX.128.66.0F38.W0 58 /r] VPBROADCASTD xmm1 {k1}{z}, xmm2/m32
    VPBROADCASTD_VdqWd_E128,
    // [EVEX.256.66.0F38.W0 58 /r] VPBROADCASTD ymm1 {k1}{z}, xmm2/m32
    VPBROADCASTD_VqqWd_E256,
    // [EVEX.512.66.0F38.W0 58 /r] VPBROADCASTD zmm1 {k1}{z}, xmm2/m32
    VPBROADCASTD_VdqqWd_E512,
    // [VEX.128.66.0F38.W0 59 /r] VPBROADCASTQ xmm1, xmm2/m64
    VPBROADCASTQ_VdqWq_V128,
    // [VEX.256.66.0F38.W0 59 /r] VPBROADCASTQ ymm1, ymm2/m64
    VPBROADCASTQ_VqqWq_V256,
    // [EVEX.128.66.0F38.W1 59 /r] VPBROADCASTQ xmm1 {k1}{z}, xmm2/m64
    VPBROADCASTQ_VdqWq_E128,
    // [EVEX.256.66.0F38.W1 59 /r] VPBROADCASTQ ymm1 {k1}{z}, xmm2/m64
    VPBROADCASTQ_VqqWq_E256,
    // [EVEX.512.66.0F38.W1 59 /r] VPBROADCASTQ zmm1 {k1}{z}, xmm2/m64
    VPBROADCASTQ_VdqqWq_E512,
    // [EVEX.128.66.0F38.W0 59 /r] VBROADCASTI32X2 xmm1 {k1}{z}, xmm2/m64
    VBROADCASTI32X2_VdqWq_E128,
    // [EVEX.256.66.0F38.W0 59 /r] VBROADCASTI32X2 ymm1 {k1}{z}, xmm2/m64
    VBROADCASTI32X2_VqqWq_E256,
    // [EVEX.512.66.0F38.W0 59 /r] VBROADCASTI32X2 zmm1 {k1}{z}, xmm2/m64
    VBROADCASTI32X2_VdqqWq_E512,
    // [VEX.256.66.0F38.W0 5A /r] VBROADCASTI128 ymm1, m128
    VBROADCASTI128_VqqMdq_V256,
    // [EVEX.256.66.0F38.W0 5A /r] VBROADCASTI32X4 ymm1 {k1}{z}, m128
    VBROADCASTI32X4_VqqMdq_E256,
    // [EVEX.512.66.0F38.W0 5A /r] VBROADCASTI32X4 zmm1 {k1}{z}, m128
    VBROADCASTI32X4_VdqqMdq_E512,
    // [EVEX.256.66.0F38.W1 5A /r] VBROADCASTI64X2 ymm1 {k1}{z}, m128
    VBROADCASTI64X2_VqqMdq_E256,
    // [EVEX.512.66.0F38.W1 5A /r] VBROADCASTI64X2 zmm1 {k1}{z}, m128
    VBROADCASTI64X2_VdqqMdq_E512,
    // [EVEX.512.66.0F38.W0 5B /r] VBROADCASTI32X8 zmm1 {k1}{z}, m256
    VBROADCASTI32X8_VdqqMqq_E512,
    // [EVEX.512.66.0F38.W1 5B /r] VBROADCASTI64X4 zmm1 {k1}{z}, m256
    VBROADCASTI64X4_VdqqMqq_E512,

    // [EVEX.128.66.0F38.W1 2A /r] VPBROADCASTMB2Q xmm1, k1
    VPBROADCASTMB2Q_VdqKEb_E128,
    // [EVEX.256.66.0F38.W1 2A /r] VPBROADCASTMB2Q ymm1, k1
    VPBROADCASTMB2Q_VqqKEb_E256,
    // [EVEX.512.66.0F38.W1 2A /r] VPBROADCASTMB2Q zmm1, k1
    VPBROADCASTMB2Q_VdqqKEb_E512,
    // [EVEX.128.66.0F38.W0 3A /r] VPBROADCASTMW2D xmm1, k1
    VPBROADCASTMB2Q_VdqKEw_E128,
    // [EVEX.256.66.0F38.W0 3A /r] VPBROADCASTMW2D ymm1, k1
    VPBROADCASTMB2Q_VqqKEw_E256,
    // [EVEX.512.66.0F38.W0 3A /r] VPBROADCASTMW2D zmm1, k1
    VPBROADCASTMB2Q_VdqqKEw_E512,

    // [EVEX.128.66.0F3A.W0 3F /r ib] VPCMPB k1 {k2}, xmm2, xmm3/m128, imm8
    VPCMPB_KGqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 3F /r ib] VPCMPB k1 {k2}, ymm2, ymm3/m256, imm8
    VPCMPB_KGqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 3F /r ib] VPCMPB k1 {k2}, zmm2, zmm3/m512, imm8
    VPCMPB_KGqHdqqWdqqIb_E512,
    // [EVEX.128.66.0F3A.W0 3E /r ib] VPCMPUB k1 {k2}, xmm2, xmm3/m128, imm8
    VPCMPUB_KGqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 3E /r ib] VPCMPUB k1 {k2}, ymm2, ymm3/m256, imm8
    VPCMPUB_KGqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 3E /r ib] VPCMPUB k1 {k2}, zmm2, zmm3/m512, imm8
    VPCMPUB_KGqHdqqWdqqIb_E512,

    // [EVEX.128.66.0F3A.W0 1F /r ib] VPCMPD k1 {k2}, xmm2, xmm3/m128/m32bcst, imm8
    VPCMPD_KGqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 1F /r ib] VPCMPD k1 {k2}, ymm2, ymm3/m256/m32bcst, imm8
    VPCMPD_KGqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 1F /r ib] VPCMPD k1 {k2}, zmm2, zmm3/m512/m32bcst, imm8
    VPCMPD_KGqHdqqWdqqIb_E512,
    // [EVEX.128.66.0F3A.W0 1E /r ib] VPCMPUD k1 {k2}, xmm2, xmm3/m128/m32bcst, imm8
    VPCMPUD_KGqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 1E /r ib] VPCMPUD k1 {k2}, ymm2, ymm3/m256/m32bcst, imm8
    VPCMPUD_KGqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 1E /r ib] VPCMPUD k1 {k2}, zmm2, zmm3/m512/m32bcst, imm8
    VPCMPUD_KGqHdqqWdqqIb_E512,

    // [EVEX.128.66.0F3A.W1 1F /r ib] VPCMPQ k1 {k2}, xmm2, xmm3/m128/m64bcst, imm8
    VPCMPQ_KGqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 1F /r ib] VPCMPQ k1 {k2}, ymm2, ymm3/m256/m64bcst, imm8
    VPCMPQ_KGqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 1F /r ib] VPCMPQ k1 {k2}, zmm2, zmm3/m512/m64bcst, imm8
    VPCMPQ_KGqHdqqWdqqIb_E512,
    // [EVEX.128.66.0F3A.W1 1E /r ib] VPCMPUQ k1 {k2}, xmm2, xmm3/m128/m64bcst, imm8
    VPCMPUQ_KGqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 1E /r ib] VPCMPUQ k1 {k2}, ymm2, ymm3/m256/m64bcst, imm8
    VPCMPUQ_KGqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 1E /r ib] VPCMPUQ k1 {k2}, zmm2, zmm3/m512/m64bcst, imm8
    VPCMPUQ_KGqHdqqWdqqIb_E512,

    // [EVEX.128.66.0F3A.W1 3F /r ib] VPCMPW k1 {k2}, xmm2, xmm3/m128, imm8
    VPCMPW_KGqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 3F /r ib] VPCMPW k1 {k2}, ymm2, ymm3/m256, imm8
    VPCMPW_KGqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 3F /r ib] VPCMPW k1 {k2}, zmm2, zmm3/m512, imm8
    VPCMPW_KGqHdqqWdqqIb_E512,
    // [EVEX.128.66.0F3A.W1 3E /r ib] VPCMPUW k1 {k2}, xmm2, xmm3/m128, imm8
    VPCMPUW_KGqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 3E /r ib] VPCMPUW k1 {k2}, ymm2, ymm3/m256, imm8
    VPCMPUW_KGqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 3E /r ib] VPCMPUW k1 {k2}, zmm2, zmm3/m512, imm8
    VPCMPUW_KGqHdqqWdqqIb_E512,

    // [EVEX.128.66.0F38.W0 63 /r] VPCOMPRESSB m128 {k1}, xmm1
    // [EVEX.128.66.0F38.W0 63 /r] VPCOMPRESSB xmm1 {k1}{z}, xmm2
    VPCOMPRESSB_WdqVdq_E128,
    // [EVEX.256.66.0F38.W0 63 /r] VPCOMPRESSB m256 {k1}, ymm1
    // [EVEX.256.66.0F38.W0 63 /r] VPCOMPRESSB ymm1 {k1}{z}, ymm2
    VPCOMPRESSB_WqqVqq_E256,
    // [EVEX.512.66.0F38.W0 63 /r] VPCOMPRESSB m512 {k1}, zmm1
    // [EVEX.512.66.0F38.W0 63 /r] VPCOMPRESSB zmm1 {k1}{z}, zmm2
    VPCOMPRESSB_WdqqVdqq_E512,
    // [EVEX.128.66.0F38.W1 63 /r] VPCOMPRESSW m128 {k1}, xmm1
    // [EVEX.128.66.0F38.W1 63 /r] VPCOMPRESSW xmm1 {k1}{z}, xmm2
    VPCOMPRESSW_WdqVdq_E128,
    // [EVEX.256.66.0F38.W1 63 /r] VPCOMPRESSW m256 {k1}, ymm1
    // [EVEX.256.66.0F38.W1 63 /r] VPCOMPRESSW ymm1 {k1}{z}, ymm2
    VPCOMPRESSW_WqqVqq_E256,
    // [EVEX.512.66.0F38.W1 63 /r] VPCOMPRESSW m512 {k1}, zmm1
    // [EVEX.512.66.0F38.W1 63 /r] VPCOMPRESSW zmm1 {k1}{z}, zmm2
    VPCOMPRESSW_WdqqVdqq_E512,

    // [EVEX.128.66.0F38.W0 8B /r] VPCOMPRESSD xmm1/m128 {k1}{z}, xmm2
    VPCOMPRESSD_WdqVdq_E128,
    // [EVEX.256.66.0F38.W0 8B /r] VPCOMPRESSD ymm1/m256 {k1}{z}, ymm2
    VPCOMPRESSD_WqqVqq_E256,
    // [EVEX.512.66.0F38.W0 8B /r] VPCOMPRESSD zmm1/m512 {k1}{z}, zmm2
    VPCOMPRESSD_WdqqVdqq_E512,

    // [EVEX.128.66.0F38.W1 8B /r] VPCOMPRESSQ xmm1/m128 {k1}{z}, xmm2
    VPCOMPRESSQ_WdqVdq_E128,
    // [EVEX.256.66.0F38.W1 8B /r] VPCOMPRESSQ ymm1/m256 {k1}{z}, ymm2
    VPCOMPRESSQ_WqqVqq_E256,
    // [EVEX.512.66.0F38.W1 8B /r] VPCOMPRESSQ zmm1/m512 {k1}{z}, zmm2
    VPCOMPRESSQ_WdqqVdqq_E512,

    // [EVEX.128.66.0F38.W0 C4 /r] VPCONFLICTD xmm1 {k1}{z}, xmm2/m128/m32bcst
    VPCONFLICTD_VdqWdq_E128,
    // [EVEX.256.66.0F38.W0 C4 /r] VPCONFLICTD ymm1 {k1}{z}, ymm2/m256/m32bcst
    VPCONFLICTD_VqqWqq_E256,
    // [EVEX.512.66.0F38.W0 C4 /r] VPCONFLICTD zmm1 {k1}{z}, zmm2/m512/m32bcst
    VPCONFLICTD_VdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 C4 /r] VPCONFLICTQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    VPCONFLICTQ_VdqWdq_E128,
    // [EVEX.256.66.0F38.W1 C4 /r] VPCONFLICTQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    VPCONFLICTQ_VqqWqq_E256,
    // [EVEX.512.66.0F38.W1 C4 /r] VPCONFLICTQ zmm1 {k1}{z}, zmm2/m512/m64bcst
    VPCONFLICTQ_VdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 50 /r] VPDPBUSD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPDPBUSD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 50 /r] VPDPBUSD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPDPBUSD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 50 /r] VPDPBUSD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPDPBUSD_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 51 /r] VPDPBUSDS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPDPBUSDS_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 51 /r] VPDPBUSDS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPDPBUSDS_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 51 /r] VPDPBUSDS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPDPBUSDS_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 52 /r] VPDPWSSD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPDPWSSD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 52 /r] VPDPWSSD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPDPWSSD_VqqHqqWqq_E256,
    // [EVEX.128.66.0F38.W0 52 /r] VPDPWSSD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPDPWSSD_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 53 /r] VPDPWSSDS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPDPWSSDS_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 53 /r] VPDPWSSDS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPDPWSSDS_VqqHqqWqq_E256,
    // [EVEX.128.66.0F38.W0 53 /r] VPDPWSSDS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPDPWSSDS_VdqqHdqqWdqq_E512,

    // [VEX.256.66.0F3A.W0 06 /r ib] VPERM2F128 ymm1, ymm2, ymm3/m256, imm8
    VPERM2F128_VqqHqqWqqIb_V256,

    // [VEX.256.66.0F3A.W0 46 /r ib] VPERM2I128 ymm1, ymm2, ymm3/m256, imm8
    VPERM2I128_VqqHqqWqqIb_V256,

    // [EVEX.128.66.0F38.W0 8D /r] VPERMB xmm1 {k1}{z}, xmm2, xmm3/m128
    VPERMB_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 8D /r] VPERMB ymm1 {k1}{z}, ymm2, ymm3/m256
    VPERMB_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 8D /r] VPERMB zmm1 {k1}{z}, zmm2, zmm3/m512
    VPERMB_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 36 /r] VPERMD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPERMD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 36 /r] VPERMD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPERMD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 36 /r] VPERMD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPERMD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 8D /r] VPERMW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPERMW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 8D /r] VPERMW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPERMW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 8D /r] VPERMW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPERMW_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 75 /r] VPERMI2B xmm1 {k1}{z}, xmm2, xmm3/m128
    VPERMI2B_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 75 /r] VPERMI2B ymm1 {k1}{z}, ymm2, ymm3/m256
    VPERMI2B_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 75 /r] VPERMI2B zmm1 {k1}{z}, zmm2, zmm3/m512
    VPERMI2B_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F38.W1 75 /r] VPERMI2W xmm1 {k1}{z}, xmm2, xmm3/m128
    VPERMI2W_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 75 /r] VPERMI2W ymm1 {k1}{z}, ymm2, ymm3/m256
    VPERMI2W_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 75 /r] VPERMI2W zmm1 {k1}{z}, zmm2, zmm3/m512
    VPERMI2W_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W0 76 /r] VPERMI2D xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPERMI2D_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 76 /r] VPERMI2D ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPERMI2D_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 76 /r] VPERMI2D zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPERMI2D_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 76 /r] VPERMI2Q xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPERMI2Q_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 76 /r] VPERMI2Q ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPERMI2Q_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 76 /r] VPERMI2Q zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPERMI2Q_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W0 77 /r] VPERMI2PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPERMI2PS_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 77 /r] VPERMI2PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPERMI2PS_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 77 /r] VPERMI2PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPERMI2PS_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 77 /r] VPERMI2PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPERMI2PD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 77 /r] VPERMI2PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPERMI2PD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 77 /r] VPERMI2PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPERMI2PD_VdqqHdqqWdqq_E512,

    // [VEX.128.66.0F38.W0 0D /r] VPERMILPD xmm1, xmm2, xmm3/m128
    VPERMILPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W0 0D /r] VPERMILPD ymm1, ymm2, ymm3/m256
    VPERMILPD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W1 0D /r] VPERMILPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPERMILPD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 0D /r] VPERMILPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPERMILPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 0D /r] VPERMILPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPERMILPD_VdqqHdqqWdqq_E512,
    // [VEX.128.66.0F3A.W0 05 /r ib] VPERMILPD xmm1, xmm2/m128, imm8
    VPERMILPD_VdqWdqIb_V128,
    // [VEX.256.66.0F3A.W0 05 /r ib] VPERMILPD ymm1, ymm2/m256, imm8
    VPERMILPD_VqqWqqIb_V128,
    // [EVEX.128.66.0F3A.W1 05 /r ib] VPERMILPD xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VPERMILPD_VdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 05 /r ib] VPERMILPD ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VPERMILPD_VqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 05 /r ib] VPERMILPD zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VPERMILPD_VdqqWdqqIb_E512,

    // [VEX.128.66.0F38.W0 0C /r] VPERMILPS xmm1, xmm2, xmm3/m128
    VPERMILPS_VdqHdqWdq_V128,
    // [VEX.128.66.0F3A.W0 04 /r ib] VPERMILPS xmm1, xmm2/m128, imm8
    VPERMILPS_VdqWdqIb_V128,
    // [VEX.256.66.0F38.W0 0C /r] VPERMILPS ymm1, ymm2, ymm3/m256
    VPERMILPS_VqqHqqWqq_V256,
    // [VEX.256.66.0F3A.W0 04 /r ib] VPERMILPS ymm1, ymm2/m256, imm8
    VPERMILPS_VqqWqqIb_V128,
    // [EVEX.128.66.0F38.W0 0C /r] VPERMILPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPERMILPS_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 0C /r] VPERMILPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPERMILPS_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 0C /r] VPERMILPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPERMILPS_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F3A.W0 04 /r ib] VPERMILPS xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VPERMILPS_VdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 04 /r ib] VPERMILPS ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VPERMILPS_VqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 04 /r ib] VPERMILPS zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VPERMILPS_VdqqWdqqIb_E512,

    // [VEX.256.66.0F3A.W1 01 /r ib] VPERMPD ymm1, ymm2/m256, imm8
    VPERMPD_VqqWqqIb_V256,
    // [EVEX.256.66.0F3A.W1 01 /r ib] VPERMPD ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VPERMPD_VqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 01 /r ib] VPERMPD zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VPERMPD_VdqqWdqqIb_E512,
    // [EVEX.256.66.0F3A.W1 16 /r ib] VPERMPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPERMPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F3A.W1 16 /r ib] VPERMPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPERMPD_VdqqHdqqWdqq_E512,

    // [VEX.256.66.0F38.W0 16 /r] VPERMPS ymm1, ymm2, ymm3/m256
    VPERMPS_VqqHqqWqq_V256,
    // [EVEX.256.66.0F38.W0 16 /r] VPERMPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPERMPS_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 16 /r] VPERMPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPERMPS_VdqqHdqqWdqq_E512,

    // [VEX.256.66.0F3A.W1 00 /r ib] VPERMQ ymm1, ymm2/m256, imm8
    VPERMQ_VqqWqqIb_V256,
    // [EVEX.256.66.0F3A.W1 00 /r ib] VPERMQ ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VPERMQ_VqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 00 /r ib] VPERMQ zmm1 {k1}{z}, xmm2/m512/m64bcst, imm8
    VPERMQ_VdqqWdqqIb_E512,
    // [EVEX.256.66.0F38.W1 36 /r] VPERMQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPERMQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 36 /r] VPERMQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPERMQ_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 7D /r] VPERMT2B xmm1 {k1}{z}, xmm2, xmm3/m128
    VPERMT2B_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 7D /r] VPERMT2B ymm1 {k1}{z}, ymm2, ymm3/m256
    VPERMT2B_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 7D /r] VPERMT2B zmm1 {k1}{z}, zmm2, zmm3/m512
    VPERMT2B_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F38.W1 7D /r] VPERMT2W xmm1 {k1}{z}, xmm2, xmm3/m128
    VPERMT2W_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 7D /r] VPERMT2W ymm1 {k1}{z}, ymm2, ymm3/m256
    VPERMT2W_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 7D /r] VPERMT2W zmm1 {k1}{z}, zmm2, zmm3/m512
    VPERMT2W_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W0 7E /r] VPERMT2D xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPERMT2D_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 7E /r] VPERMT2D ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPERMT2D_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 7E /r] VPERMT2D zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPERMT2D_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 7E /r] VPERMT2Q xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPERMT2Q_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 7E /r] VPERMT2Q ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPERMT2Q_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 7E /r] VPERMT2Q zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPERMT2Q_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W0 7F /r] VPERMT2PS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPERMT2PS_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 7F /r] VPERMT2PS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPERMT2PS_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 7F /r] VPERMT2PS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPERMT2PS_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 7F /r] VPERMT2PD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPERMT2PD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 7F /r] VPERMT2PD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPERMT2PD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 7F /r] VPERMT2PD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPERMT2PD_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 62 /r] VPEXPANDB xmm1 {k1}{z}, m128
    // [EVEX.128.66.0F38.W0 62 /r] VPEXPANDB xmm1 {k1}{z}, xmm2
    VPEXPANDB_VdqWdq_E128,
    // [EVEX.256.66.0F38.W0 62 /r] VPEXPANDB ymm1 {k1}{z}, m256
    // [EVEX.256.66.0F38.W0 62 /r] VPEXPANDB ymm1 {k1}{z}, ymm2
    VPEXPANDB_VqqWqq_E256,
    // [EVEX.512.66.0F38.W0 62 /r] VPEXPANDB zmm1 {k1}{z}, m512
    // [EVEX.512.66.0F38.W0 62 /r] VPEXPANDB zmm1 {k1}{z}, zmm2
    VPEXPANDB_VdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 62 /r] VPEXPANDW xmm1 {k1}{z}, m128
    // [EVEX.128.66.0F38.W1 62 /r] VPEXPANDW xmm1 {k1}{z}, xmm2
    VPEXPANDW_VdqWdq_E128,
    // [EVEX.256.66.0F38.W1 62 /r] VPEXPANDW ymm1 {k1}{z}, m256
    // [EVEX.256.66.0F38.W1 62 /r] VPEXPANDW ymm1 {k1}{z}, ymm2
    VPEXPANDW_VqqWqq_E256,
    // [EVEX.512.66.0F38.W1 62 /r] VPEXPANDW zmm1 {k1}{z}, m512
    // [EVEX.512.66.0F38.W1 62 /r] VPEXPANDW zmm1 {k1}{z}, zmm2
    VPEXPANDW_VdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 89 /r] VPEXPANDD xmm1 {k1}{z}, xmm2/m128
    VPEXPANDD_VdqWdq_E128,
    // [EVEX.256.66.0F38.W0 89 /r] VPEXPANDD ymm1 {k1}{z}, ymm2/m256
    VPEXPANDD_VqqWqq_E256,
    // [EVEX.512.66.0F38.W0 89 /r] VPEXPANDD zmm1 {k1}{z}, zmm2/m512
    VPEXPANDD_VdqqWdqq_E512,

    // [EVEX.128.66.0F38.W1 89 /r] VPEXPANDQ xmm1 {k1}{z}, xmm2/m128
    VPEXPANDQ_VdqWdq_E128,
    // [EVEX.256.66.0F38.W1 89 /r] VPEXPANDQ ymm1 {k1}{z}, ymm2/m256
    VPEXPANDQ_VqqWqq_E256,
    // [EVEX.512.66.0F38.W1 89 /r] VPEXPANDQ zmm1 {k1}{z}, zmm2/m512
    VPEXPANDQ_VdqqWdqq_E512,

    // [VEX.128.66.0F38.W0 90 /r] VPGATHERDD xmm1, vm32x, xmm2
    VPGATHERDD_VdqVMdHdq_V128,
    // [VEX.128.66.0F38.W0 91 /r] VPGATHERQD xmm1, vm64x, xmm2
    VPGATHERQD_VdqVMqHdq_V128,
    // [VEX.256.66.0F38.W0 90 /r] VPGATHERDD ymm1, vm32y, ymm2
    VPGATHERDD_VqqVMdHqq_V256,
    // [VEX.256.66.0F38.W0 91 /r] VPGATHERQD ymm1, vm64y, ymm2
    VPGATHERQD_VqqVMqHqq_V256,

    // [EVEX.128.66.0F38.W0 90 /vsib] VPGATHERDD xmm1 {k1}, vm32x
    VPGATHERDD_VdqVMd_E128,
    // [EVEX.256.66.0F38.W0 90 /vsib] VPGATHERDD ymm1 {k1}, vm32y
    VPGATHERDD_VqqVMd_E256,
    // [EVEX.512.66.0F38.W0 90 /vsib] VPGATHERDD zmm1 {k1}, vm32z
    VPGATHERDD_VdqqVMd_E512,
    // [EVEX.128.66.0F38.W1 90 /vsib] VPGATHERDQ xmm1 {k1}, vm32x
    VPGATHERDQ_VdqVMd_E128,
    // [EVEX.256.66.0F38.W1 90 /vsib] VPGATHERDQ ymm1 {k1}, vm32y
    VPGATHERDQ_VqqVMd_E256,
    // [EVEX.512.66.0F38.W1 90 /vsib] VPGATHERDQ zmm1 {k1}, vm32z
    VPGATHERDQ_VdqqVMd_E512,

    // [VEX.128.66.0F38.W1 90 /r] VPGATHERDQ xmm1, vm32x, xmm2
    VPGATHERDQ_VdqVMdHdq_V128,
    // [VEX.128.66.0F38.W1 91 /r] VPGATHERQQ xmm1, vm64x, xmm2
    VPGATHERQQ_VdqVMqHdq_V128,
    // [VEX.256.66.0F38.W1 90 /r] VPGATHERDQ ymm1, vm32x, ymm2
    VPGATHERDQ_VqqVMdHqq_V256,
    // [VEX.256.66.0F38.W1 91 /r] VPGATHERQQ ymm1, vm64x, ymm2
    VPGATHERQQ_VqqVMqHqq_V256,

    // [EVEX.128.66.0F38.W0 91 /vsib] VPGATHERQD xmm1 {k1}, vm64x
    VPGATHERQD_VdqVMq_E128,
    // [EVEX.256.66.0F38.W0 91 /vsib] VPGATHERQD ymm1 {k1}, vm64y
    VPGATHERQD_VqqVMq_E256,
    // [EVEX.512.66.0F38.W0 91 /vsib] VPGATHERQD zmm1 {k1}, vm64z
    VPGATHERQD_VdqqVMq_E512,
    // [EVEX.128.66.0F38.W1 91 /vsib] VPGATHERQQ xmm1 {k1}, vm64x
    VPGATHERQQ_VdqVMq_E128,
    // [EVEX.256.66.0F38.W1 91 /vsib] VPGATHERQQ ymm1 {k1}, vm64y
    VPGATHERQQ_VqqVMq_E256,
    // [EVEX.512.66.0F38.W1 91 /vsib] VPGATHERQQ zmm1 {k1}, vm64z
    VPGATHERQQ_VdqqVMq_E512,

    // [EVEX.128.66.0F38.W0 44 /r] VPLZCNTD xmm1 {k1}{z}, xmm2/m128/m32bcst
    VPLZCNTD_VdqWdq_E128,
    // [EVEX.256.66.0F38.W0 44 /r] VPLZCNTD ymm1 {k1}{z}, ymm2/m256/m32bcst
    VPLZCNTD_VqqWqq_E256,
    // [EVEX.512.66.0F38.W0 44 /r] VPLZCNTD zmm1 {k1}{z}, zmm2/m512/m32bcst
    VPLZCNTD_VdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 44 /r] VPLZCNTQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    VPLZCNTQ_VdqWdq_E128,
    // [EVEX.256.66.0F38.W1 44 /r] VPLZCNTQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    VPLZCNTQ_VqqWqq_E256,
    // [EVEX.512.66.0F38.W1 44 /r] VPLZCNTQ zmm1 {k1}{z}, zmm2/m512/m64bcst
    VPLZCNTQ_VdqqWdqq_E512,

    // [EVEX.128.66.0F38.W1 B5 /r] VPMADD52HUQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPMADD52HUQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 B5 /r] VPMADD52HUQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPMADD52HUQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 B5 /r] VPMADD52HUQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPMADD52HUQ_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F38.W1 B4 /r] VPMADD52LUQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPMADD52LUQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 B4 /r] VPMADD52LUQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPMADD52LUQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 B4 /r] VPMADD52LUQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPMADD52LUQ_VdqqHdqqWdqq_E512,

    // [VEX.128.66.0F38.W0 8C /r] VPMASKMOVD xmm1, xmm2, m128
    VPMASKMOVD_VdqHdqMdq_V128,
    // [VEX.256.66.0F38.W0 8C /r] VPMASKMOVD ymm1, ymm2, m256
    VPMASKMOVD_VqqHqqMqq_V256,
    // [VEX.128.66.0F38.W1 8C /r] VPMASKMOVQ xmm1, xmm2, m128
    VPMASKMOVQ_VdqHdqMdq_V128,
    // [VEX.256.66.0F38.W1 8C /r] VPMASKMOVQ ymm1, ymm2, m256
    VPMASKMOVQ_VqqHqqMqq_V256,
    // [VEX.128.66.0F38.W0 8E /r] VPMASKMOVD m128, xmm1, xmm2
    VPMASKMOVD_MdqHdqVdq_V128,
    // [VEX.256.66.0F38.W0 8E /r] VPMASKMOVD m256, ymm1, ymm2
    VPMASKMOVD_MqqHqqVqq_V256,
    // [VEX.128.66.0F38.W1 8E /r] VPMASKMOVQ m128, xmm1, xmm2
    VPMASKMOVQ_MdqHdqVdq_V128,
    // [VEX.256.66.0F38.W1 8E /r] VPMASKMOVQ m256, ymm1, ymm2
    VPMASKMOVQ_MqqHqqVqq_V256,

    // [EVEX.128.F3.0F38.W0 29 /r] VPMOVB2M k1, xmm1
    VPMOVB2M_KGqVdq_E128,
    // [EVEX.256.F3.0F38.W0 29 /r] VPMOVB2M k1, ymm1
    VPMOVB2M_KGqVqq_E256,
    // [EVEX.512.F3.0F38.W0 29 /r] VPMOVB2M k1, zmm1
    VPMOVB2M_KGqVdqq_E512,
    // [EVEX.128.F3.0F38.W1 29 /r] VPMOVW2M k1, xmm1
    VPMOVW2M_KGqVdq_E128,
    // [EVEX.256.F3.0F38.W1 29 /r] VPMOVW2M k1, ymm1
    VPMOVW2M_KGqVqq_E256,
    // [EVEX.512.F3.0F38.W1 29 /r] VPMOVW2M k1, zmm1
    VPMOVW2M_KGqVdqq_E512,
    // [EVEX.128.F3.0F38.W0 39 /r] VPMOVD2M k1, xmm1
    VPMOVD2M_KGqVdq_E128,
    // [EVEX.256.F3.0F38.W0 39 /r] VPMOVD2M k1, ymm1
    VPMOVD2M_KGqVqq_E256,
    // [EVEX.512.F3.0F38.W0 39 /r] VPMOVD2M k1, zmm1
    VPMOVD2M_KGqVdqq_E512,
    // [EVEX.128.F3.0F38.W1 39 /r] VPMOVQ2M k1, xmm1
    VPMOVQ2M_KGqVdq_E128,
    // [EVEX.256.F3.0F38.W1 39 /r] VPMOVQ2M k1, ymm1
    VPMOVQ2M_KGqVqq_E256,
    // [EVEX.512.F3.0F38.W1 39 /r] VPMOVQ2M k1, zmm1
    VPMOVQ2M_KGqVdqq_E512,

    // [EVEX.128.F3.0F38.W0 31 /r] VPMOVDB xmm1/m32 {k1}{z}, xmm2
    VPMOVDB_WdVdq_E128,
    // [EVEX.128.F3.0F38.W0 21 /r] VPMOVSDB xmm1/m32 {k1}{z}, xmm2
    VPMOVSDB_WdVdq_E128,
    // [EVEX.128.F3.0F38.W0 11 /r] VPMOVUSDB xmm1/m32 {k1}{z}, xmm2
    VPMOVUSDB_WdVdq_E128,
    // [EVEX.256.F3.0F38.W0 31 /r] VPMOVDB xmm1/m64 {k1}{z}, ymm2
    VPMOVDB_WqVqq_E256,
    // [EVEX.256.F3.0F38.W0 21 /r] VPMOVSDB xmm1/m64 {k1}{z}, ymm2
    VPMOVSDB_WqVqq_E256,
    // [EVEX.256.F3.0F38.W0 11 /r] VPMOVUSDB xmm1/m64 {k1}{z}, ymm2
    VPMOVUSDB_WqVqq_E256,
    // [EVEX.512.F3.0F38.W0 31 /r] VPMOVDB xmm1/m128 {k1}{z}, zmm2
    VPMOVDB_WdqVdqq_E512,
    // [EVEX.512.F3.0F38.W0 21 /r] VPMOVSDB xmm1/m128 {k1}{z}, zmm2
    VPMOVSDB_WdqVdqq_E512,
    // [EVEX.512.F3.0F38.W0 11 /r] VPMOVUSDB xmm1/m128 {k1}{z}, zmm2
    VPMOVUSDB_WdqVdqq_E512,

    // [EVEX.128.F3.0F38.W0 33 /r] VPMOVDW xmm1/m64 {k1}{z}, xmm2
    VPMOVDW_WqVdq_E128,
    // [EVEX.128.F3.0F38.W0 23 /r] VPMOVSDW xmm1/m64 {k1}{z}, xmm2
    VPMOVSDW_WqVdq_E128,
    // [EVEX.128.F3.0F38.W0 13 /r] VPMOVUSDW xmm1/m64 {k1}{z}, xmm2
    VPMOVUSDW_WqVdq_E128,
    // [EVEX.256.F3.0F38.W0 33 /r] VPMOVDW xmm1/m128 {k1}{z}, ymm2
    VPMOVDW_WdqVqq_E256,
    // [EVEX.256.F3.0F38.W0 23 /r] VPMOVSDW xmm1/m128 {k1}{z}, ymm2
    VPMOVSDW_WdqVqq_E256,
    // [EVEX.256.F3.0F38.W0 13 /r] VPMOVUSDW xmm1/m128 {k1}{z}, ymm2
    VPMOVUSDW_WdqVqq_E256,
    // [EVEX.512.F3.0F38.W0 33 /r] VPMOVDW ymm1/m256 {k1}{z}, zmm2
    VPMOVDW_WqqVdqq_E512,
    // [EVEX.512.F3.0F38.W0 23 /r] VPMOVSDW ymm1/m256 {k1}{z}, zmm2
    VPMOVSDW_WqqVdqq_E512,
    // [EVEX.512.F3.0F38.W0 13 /r] VPMOVUSDW ymm1/m256 {k1}{z}, zmm2
    VPMOVUSDW_WqqVdqq_E512,

    // [EVEX.128.F3.0F38.W0 28 /r] VPMOVM2B xmm1, k1
    VPMOVM2B_VdqKRq_E128,
    // [EVEX.256.F3.0F38.W0 28 /r] VPMOVM2B ymm1, k1
    VPMOVM2B_VqqKRq_E256,
    // [EVEX.512.F3.0F38.W0 28 /r] VPMOVM2B zmm1, k1
    VPMOVM2B_VdqqKRq_E512,
    // [EVEX.128.F3.0F38.W1 28 /r] VPMOVM2W xmm1, k1
    VPMOVM2W_VdqKRq_E128,
    // [EVEX.256.F3.0F38.W1 28 /r] VPMOVM2W ymm1, k1
    VPMOVM2W_VqqKRq_E256,
    // [EVEX.512.F3.0F38.W1 28 /r] VPMOVM2W zmm1, k1
    VPMOVM2W_VdqqKRq_E512,
    // [EVEX.128.F3.0F38.W0 38 /r] VPMOVM2D xmm1, k1
    VPMOVM2D_VdqKRq_E128,
    // [EVEX.256.F3.0F38.W0 38 /r] VPMOVM2D ymm1, k1
    VPMOVM2D_VqqKRq_E256,
    // [EVEX.512.F3.0F38.W0 38 /r] VPMOVM2D zmm1, k1
    VPMOVM2D_VdqqKRq_E512,
    // [EVEX.128.F3.0F38.W1 38 /r] VPMOVM2Q xmm1, k1
    VPMOVM2Q_VdqKRq_E128,
    // [EVEX.256.F3.0F38.W1 38 /r] VPMOVM2Q ymm1, k1
    VPMOVM2Q_VqqKRq_E256,
    // [EVEX.512.F3.0F38.W1 38 /r] VPMOVM2Q zmm1, k1
    VPMOVM2Q_VdqqKRq_E512,

    // [EVEX.128.F3.0F38.W0 32 /r] VPMOVQB xmm1/m16 {k1}{z}, xmm2
    VPMOVQB_WwVdq_E128,
    // [EVEX.128.F3.0F38.W0 22 /r] VPMOVSQB xmm1/m16 {k1}{z}, xmm2
    VPMOVSQB_WwVdq_E128,
    // [EVEX.128.F3.0F38.W0 12 /r] VPMOVUSQB xmm1/m16 {k1}{z}, xmm2
    VPMOVUSQB_WwVdq_E128,
    // [EVEX.256.F3.0F38.W0 32 /r] VPMOVQB xmm1/m32 {k1}{z}, ymm2
    VPMOVQB_WdVqq_E256,
    // [EVEX.256.F3.0F38.W0 22 /r] VPMOVSQB xmm1/m32 {k1}{z}, ymm2
    VPMOVSQB_WdVqq_E256,
    // [EVEX.256.F3.0F38.W0 12 /r] VPMOVUSQB xmm1/m32 {k1}{z}, ymm2
    VPMOVUSQB_WdVqq_E256,
    // [EVEX.128.F3.0F38.W0 32 /r] VPMOVQB xmm1/m64 {k1}{z}, zmm2
    VPMOVQB_WqVdqq_E5122,
    // [EVEX.128.F3.0F38.W0 22 /r] VPMOVSQB xmm1/m64 {k1}{z}, zmm2
    VPMOVSQB_WqVdqq_E512,
    // [EVEX.128.F3.0F38.W0 12 /r] VPMOVUSQB xmm1/m64 {k1}{z}, zmm2
    VPMOVUSQB_WqVdqq_E512,

    // [EVEX.128.F3.0F38.W0 35 /r] VPMOVQD xmm1/m64 {k1}{z}, xmm2
    // NOTE: Intel manual says m128 and "2 packed double-word integer in xmm1/m128"
    VPMOVQD_WqVdq_E128,
    // [EVEX.128.F3.0F38.W0 5 /r] VPMOVSQD xmm1/m64 {k1}{z}, xmm2
    VPMOVSQD_WqVdq_E128,
    // [EVEX.128.F3.0F38.W0 15 /r] VPMOVUSQD xmm1/m64 {k1}{z}, xmm2
    VPMOVUSQD_WqVdq_E128,
    // [EVEX.256.F3.0F38.W0 35 /r] VPMOVQD xmm1/m128 {k1}{z}, ymm2
    VPMOVQD_WdqVqq_E256,
    // [EVEX.256.F3.0F38.W0 25 /r] VPMOVSQD xmm1/m128 {k1}{z}, ymm2
    VPMOVSQD_WdqVqq_E256,
    // [EVEX.256.F3.0F38.W0 15 /r] VPMOVUSQD xmm1/m128 {k1}{z}, ymm2
    VPMOVUSQD_WdqVqq_E256,
    // [EVEX.512.F3.0F38.W0 35 /r] VPMOVQD xmm1/m256 {k1}{z}, zmm2
    VPMOVQD_WqqVdqq_E512,
    // [EVEX.512.F3.0F38.W0 25 /r] VPMOVSQD xmm1/m256 {k1}{z}, zmm2
    VPMOVSQD_WqqVdqq_E512,
    // [EVEX.512.F3.0F38.W0 15 /r] VPMOVUSQD xmm1/m256 {k1}{z}, zmm2
    VPMOVUSQD_WqqVdqq_E512,

    // [EVEX.128.F3.0F38.W0 34 /r] VPMOVQW xmm1/m32 {k1}{z}, xmm2
    VPMOVQW_WdVdq_E128,
    // [EVEX.128.F3.0F38.W0 24 /r] VPMOVSQW xmm1/m32 {k1}{z}, xmm2
    VPMOVSQW_WdVdq_E128,
    // [EVEX.128.F3.0F38.W0 14 /r] VPMOVUSQW xmm1/m32 {k1}{z}, xmm2
    VPMOVUSQW_WdVdq_E128,
    // [EVEX.256.F3.0F38.W0 34 /r] VPMOVQW xmm1/m64 {k1}{z}, ymm2
    VPMOVQW_WqVqq_E256,
    // [EVEX.256.F3.0F38.W0 24 /r] VPMOVSQW xmm1/m64 {k1}{z}, ymm2
    VPMOVSQW_WqVqq_E256,
    // [EVEX.256.F3.0F38.W0 14 /r] VPMOVUSQW xmm1/m64 {k1}{z}, ymm2
    VPMOVUSQW_WqVqq_E256,
    // [EVEX.512.F3.0F38.W0 34 /r] VPMOVQW xmm1/m128 {k1}{z}, zmm2
    VPMOVQW_WdqVdqq_E512,
    // [EVEX.512.F3.0F38.W0 24 /r] VPMOVSQW xmm1/m128 {k1}{z}, zmm2
    VPMOVSQW_WdqVdqq_E512,
    // [EVEX.521.F3.0F38.W0 14 /r] VPMOVUSQW xmm1/m128 {k1}{z}, zmm2
    VPMOVUSQW_WdqVdqq_E512,

    // [EVEX.128.F3.0F38.W0 30 /r] VPMOVWB xmm1/m64 {k1}{z}, xmm2
    VPMOVWB_WqVdq_E128,
    // [EVEX.128.F3.0F38.W0 20 /r] VPMOVSWB xmm1/m64 {k1}{z}, xmm2
    VPMOVSWB_WqVdq_E128,
    // [EVEX.128.F3.0F38.W0 10 /r] VPMOVUSWB xmm1/m64 {k1}{z}, xmm2
    VPMOVUSWB_WqVdq_E128,
    // [EVEX.256.F3.0F38.W0 30 /r] VPMOVWB xmm1/m128 {k1}{z}, ymm2
    VPMOVWB_WdqVqq_E256,
    // [EVEX.256.F3.0F38.W0 20 /r] VPMOVSWB xmm1/m128 {k1}{z}, ymm2
    VPMOVSWB_WdqVqq_E256,
    // [EVEX.256.F3.0F38.W0 10 /r] VPMOVUSWB xmm1/m128 {k1}{z}, ymm2
    VPMOVUSWB_WdqVqq_E256,
    // [EVEX.512.F3.0F38.W0 30 /r] VPMOVWB xmm1/m256 {k1}{z}, zmm2
    VPMOVWB_WqqVdqq_E512,
    // [EVEX.512.F3.0F38.W0 20 /r] VPMOVSWB xmm1/m256 {k1}{z}, zmm2
    VPMOVSWB_WqqVdqq_E512,
    // [EVEX.512.F3.0F38.W0 10 /r] VPMOVUSWB xmm1/m256 {k1}{z}, zmm2
    VPMOVUSWB_WqqVdqq_E512,

    // [EVEX.128.66.0F38.W1 83 /r] VPMULTISHIFTQB xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPMULTISHIFTQB_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 83 /r] VPMULTISHIFTQB ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPMULTISHIFTQB_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 83 /r] VPMULTISHIFTQB zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPMULTISHIFTQB_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 54 /r] VPOPCNTB xmm1 {k1}{z}, xmm2/m128
    VPOPCNTB_VdqWdq_E128,
    // [EVEX.256.66.0F38.W0 54 /r] VPOPCNTB ymm1 {k1}{z}, ymm2/m256
    VPOPCNTB_VqqWqq_E256,
    // [EVEX.512.66.0F38.W0 54 /r] VPOPCNTB zmm1 {k1}{z}, zmm2/m512
    VPOPCNTB_VdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 54 /r] VPOPCNTW xmm1 {k1}{z}, xmm2/m128
    VPOPCNTW_VdqWdq_E128,
    // [EVEX.256.66.0F38.W1 54 /r] VPOPCNTW ymm1 {k1}{z}, ymm2/m256
    VPOPCNTW_VqqWqq_E256,
    // [EVEX.512.66.0F38.W1 54 /r] VPOPCNTW zmm1 {k1}{z}, zmm2/m512
    VPOPCNTW_VdqqWdqq_E512,
    // [EVEX.128.66.0F38.W0 55 /r] VPOPCNTD xmm1 {k1}{z}, xmm2/m128/m32bcst
    VPOPCNTD_VdqWdq_E128,
    // [EVEX.256.66.0F38.W0 55 /r] VPOPCNTD ymm1 {k1}{z}, ymm2/m256/m32bcst
    VPOPCNTD_VqqWqq_E256,
    // [EVEX.512.66.0F38.W0 55 /r] VPOPCNTD zmm1 {k1}{z}, zmm2/m512/m32bcst
    VPOPCNTD_VdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 55 /r] VPOPCNTQ xmm1 {k1}{z}, xmm2/m128/m64bcst
    VPOPCNTQ_VdqWdq_E128,
    // [EVEX.256.66.0F38.W1 55 /r] VPOPCNTQ ymm1 {k1}{z}, ymm2/m256/m64bcst
    VPOPCNTQ_VqqWqq_E256,
    // [EVEX.512.66.0F38.W1 55 /r] VPOPCNTQ zmm1 {k1}{z}, zmm2/m512/m64bcst
    VPOPCNTQ_VdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 15 /r] VPROLVD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPROLVD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 72 /1 ib] VPROLD xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VPROLD_HdqWdqIb_E128,
    // [EVEX.128.66.0F38.W1 15 /r] VPROLVQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPROLVQ_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W1 72 /1 ib] VPROLQ xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VPROLQ_HdqWdqIb_E128,
    // [EVEX.256.66.0F38.W0 15 /r] VPROLVD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPROLVD_VqqHqqWqq_E128,
    // [EVEX.256.66.0F38.W0 72 /1 ib] VPROLD ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VPROLD_HqqWqqIb_E128,
    // [EVEX.256.66.0F38.W1 15 /r] VPROLVQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPROLVQ_VqqHqqWqq_E128,
    // [EVEX.256.66.0F38.W1 72 /1 ib] VPROLQ ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VPROLQ_HqqWqqIb_E128,
    // [EVEX.512.66.0F38.W0 15 /r] VPROLVD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPROLVD_VdqqHdqqWdqq_E128,
    // [EVEX.512.66.0F38.W0 72 /1 ib] VPROLD zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VPROLD_HdqqWdqqIb_E128,
    // [EVEX.512.66.0F38.W1 15 /r] VPROLVQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPROLVQ_VdqqHdqqWdqq_E128,
    // [EVEX.512.66.0F38.W1 72 /1 ib] VPROLQ zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VPROLQ_HdqqWdqqIb_E128,

    // [EVEX.128.66.0F38.W0 14 /r] VPRORVD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPRORVD_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W0 72 /0 ib] VPRORD xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VPRORD_HdqWdqIb_E128,
    // [EVEX.128.66.0F38.W1 14 /r] VPRORVQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPRORVQ_VdqHdqWdq_E128,
    // [EVEX.128.66.0F38.W1 72 /0 ib] VPRORQ xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VPRORQ_HdqWdqIb_E128,
    // [EVEX.256.66.0F38.W0 14 /r] VPRORVD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPRORVD_VqqHqqWqq_E128,
    // [EVEX.256.66.0F38.W0 72 /0 ib] VPRORD ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VPRORD_HqqWqqIb_E128,
    // [EVEX.256.66.0F38.W1 14 /r] VPRORVQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPRORVQ_VqqHqqWqq_E128,
    // [EVEX.256.66.0F38.W1 72 /0 ib] VPRORQ ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VPRORQ_HqqWqqIb_E128,
    // [EVEX.512.66.0F38.W0 14 /r] VPRORVD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPRORVD_VdqqHdqqWdqq_E128,
    // [EVEX.512.66.0F38.W0 72 /0 ib] VPRORD zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VPRORD_HdqqWdqqIb_E128,
    // [EVEX.512.66.0F38.W1 14 /r] VPRORVQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPRORVQ_VdqqHdqqWdqq_E128,
    // [EVEX.512.66.0F38.W1 72 /0 ib] VPRORQ zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VPRORQ_HdqqWdqqIb_E128,

    // [EVEX.128.66.0F38.W0 A0 /vsib] VPSCATTERDD vm32x {k1}, xmm1
    VPSCATTERDD_VMdVdq_E128,
    // [EVEX.256.66.0F38.W0 A0 /vsib] VPSCATTERDD vm32y {k1}, ymm1
    VPSCATTERDD_VMdVqq_E256,
    // [EVEX.512.66.0F38.W0 A0 /vsib] VPSCATTERDD vm32z {k1}, zmm1
    VPSCATTERDD_VMdVdqq_E512,
    // [EVEX.128.66.0F38.W1 A0 /vsib] VPSCATTERDQ vm32x {k1}, xmm1
    VPSCATTERDQ_VMdVdq_E128,
    // [EVEX.256.66.0F38.W1 A0 /vsib] VPSCATTERDQ vm32y {k1}, ymm1
    VPSCATTERDQ_VMdVqq_E256,
    // [EVEX.512.66.0F38.W1 A0 /vsib] VPSCATTERDQ vm32z {k1}, zmm1
    VPSCATTERDQ_VMdVdqq_E512,
    // [EVEX.128.66.0F38.W0 A1 /vsib] VPSCATTERQD vm64x {k1}, xmm1
    VPSCATTERQD_VMqVdq_E128,
    // [EVEX.256.66.0F38.W0 A1 /vsib] VPSCATTERQD vm64y {k1}, ymm1
    VPSCATTERQD_VMqVqq_E256,
    // [EVEX.512.66.0F38.W0 A1 /vsib] VPSCATTERQD vm64z {k1}, zmm1
    VPSCATTERQD_VMqVdqq_E512,
    // [EVEX.128.66.0F38.W1 A1 /vsib] VPSCATTERQQ vm64x {k1}, xmm1
    VPSCATTERQQ_VMqVdq_E128,
    // [EVEX.256.66.0F38.W1 A1 /vsib] VPSCATTERQQ vm64y {k1}, ymm1
    VPSCATTERQQ_VMqVqq_E256,
    // [EVEX.512.66.0F38.W1 A1 /vsib] VPSCATTERQQ vm64z {k1}, zmm1
    VPSCATTERQQ_VMqVdqq_E512,

    // [EVEX.128.66.0F3A.W1 70 /r ib] VPSHLDW xmm1 {k1}{z}, xmm2, xmm3/m128, imm8
    VPSHLDW_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 70 /r ib] VPSHLDW ymm1 {k1}{z}, ymm2, ymm3/m128, imm8
    VPSHLDW_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 70 /r ib] VPSHLDW zmm1 {k1}{z}, zmm2, zmm3/m128, imm8
    VPSHLDW_VdqqHdqqWdqqIb_E512,
    // [EVEX.128.66.0F3A.W0 71 /r ib] VPSHLDD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst, imm8
    VPSHLDD_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 71 /r ib] VPSHLDD ymm1 {k1}{z}, ymm2, ymm3/m128/m32bcst, imm8
    VPSHLDD_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 71 /r ib] VPSHLDD zmm1 {k1}{z}, zmm2, zmm3/m128/m32bcst, imm8
    VPSHLDD_VdqqHdqqWdqqIb_E512,
    // [EVEX.128.66.0F3A.W1 71 /r ib] VPSHLDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    VPSHLDQ_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 71 /r ib] VPSHLDQ ymm1 {k1}{z}, ymm2, ymm3/m128/m64bcst, imm8
    VPSHLDQ_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 71 /r ib] VPSHLDQ zmm1 {k1}{z}, zmm2, zmm3/m128/m64bcst, imm8
    VPSHLDQ_VdqqHdqqWdqqIb_E512,

    // [EVEX.128.66.0F38.W1 70 /r] VPSHLDVW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSHLDVW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 70 /r] VPSHLDVW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPSHLDVW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 70 /r] VPSHLDVW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPSHLDVW_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W0 71 /r] VPSHLDVW xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPSHLDVD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 71 /r] VPSHLDVW ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPSHLDVD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 71 /r] VPSHLDVW zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPSHLDVD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 71 /r] VPSHLDVW xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPSHLDVQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 71 /r] VPSHLDVW ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPSHLDVQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 71 /r] VPSHLDVW zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPSHLDVQ_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F3A.W1 72 /r ib] VPSHRDW xmm1 {k1}{z}, xmm2, xmm3/m128, imm8
    VPSHRDW_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 72 /r ib] VPSHRDW ymm1 {k1}{z}, ymm2, ymm3/m128, imm8
    VPSHRDW_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 72 /r ib] VPSHRDW zmm1 {k1}{z}, zmm2, zmm3/m128, imm8
    VPSHRDW_VdqqHdqqWdqqIb_E512,
    // [EVEX.128.66.0F3A.W0 73 /r ib] VPSHRDD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst, imm8
    VPSHRDD_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 73 /r ib] VPSHRDD ymm1 {k1}{z}, ymm2, ymm3/m128/m32bcst, imm8
    VPSHRDD_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 73 /r ib] VPSHRDD zmm1 {k1}{z}, zmm2, zmm3/m128/m32bcst, imm8
    VPSHRDD_VdqqHdqqWdqqIb_E512,
    // [EVEX.128.66.0F3A.W1 73 /r ib] VPSHRDQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    VPSHRDQ_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 73 /r ib] VPSHRDQ ymm1 {k1}{z}, ymm2, ymm3/m128/m64bcst, imm8
    VPSHRDQ_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 73 /r ib] VPSHRDQ zmm1 {k1}{z}, zmm2, zmm3/m128/m64bcst, imm8
    VPSHRDQ_VdqqHdqqWdqqIb_E512,

    // [EVEX.128.66.0F38.W1 72 /r] VPSHRDVW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSHRDVW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 72 /r] VPSHRDVW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPSHRDVW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 72 /r] VPSHRDVW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPSHRDVW_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W0 73 /r] VPSHRDVW xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPSHRDVD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 73 /r] VPSHRDVW ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPSHRDVD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 73 /r] VPSHRDVW zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPSHRDVD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 73 /r] VPSHRDVW xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPSHRDVQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 73 /r] VPSHRDVW ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPSHRDVQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 73 /r] VPSHRDVW zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPSHRDVQ_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F38.W0 8F /r] VPSHUFBITQMB k1 {k2}, xmm2, xmm3/m128
    VPSHUFBITQMB_KGqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 8F /r] VPSHUFBITQMB k1 {k2}, ymm2, ymm3/m256
    VPSHUFBITQMB_KGqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 8F /r] VPSHUFBITQMB k1 {k2}, zmm2, zmm3/m512
    VPSHUFBITQMB_KGqHdqqWdqq_E512,

    // [VEX.128.66.0F38.W0 47 /r] VPSLLVD xmm1, xmm2, xmm3/m128
    VPSLLVD_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W1 47 /r] VPSLLVQ xmm1, xmm2, xmm3/m128
    VPSLLVQ_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W0 47 /r] VPSLLVD ymm1, ymm2, ymm3/m256
    VPSLLVD_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W1 47 /r] VPSLLVQ ymm1, ymm2, ymm3/m256
    VPSLLVQ_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W1 12 /r] VPSLLVW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSLLVW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 12 /r] VPSLLVW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPSLLVW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 12 /r] VPSLLVW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPSLLVW_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W0 47 /r] VPSLLVD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPSLLVD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 47 /r] VPSLLVD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPSLLVD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 47 /r] VPSLLVD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPSLLVD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 47 /r] VPSLLVQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPSLLVQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 47 /r] VPSLLVQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPSLLVQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 47 /r] VPSLLVQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPSLLVQ_VdqqHdqqWdqq_E512,

    // [VEX.128.66.0F38.W0 46 /r] VPSRAVD xmm1, xmm2, xmm3/m128
    VPSRAVD_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W0 46 /r] VPSRAVD ymm1, ymm2, ymm3/m256
    VPSRAVD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W1 11 /r] VPSRAVW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSRAVW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 11 /r] VPSRAVW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPSRAVW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 11 /r] VPSRAVW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPSRAVW_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W0 46 /r] VPSRAVD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPSRAVD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 46 /r] VPSRAVD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPSRAVD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 46 /r] VPSRAVD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPSRAVD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 46 /r] VPSRAVQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPSRAVQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 46 /r] VPSRAVQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPSRAVQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 46 /r] VPSRAVQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPSRAVQ_VdqqHdqqWdqq_E512,

    // [VEX.128.66.0F38.W0 45 /r] VPSRLVD xmm1, xmm2, xmm3/m128
    VPSRLVD_VdqHdqWdq_V128,
    // [VEX.128.66.0F38.W1 45 /r] VPSRLVQ xmm1, xmm2, xmm3/m128
    VPSRLVQ_VdqHdqWdq_V128,
    // [VEX.256.66.0F38.W0 45 /r] VPSRLVD ymm1, ymm2, ymm3/m256
    VPSRLVD_VqqHqqWqq_V256,
    // [VEX.256.66.0F38.W1 45 /r] VPSRLV! ymm1, ymm2, ymm3/m256
    VPSRLVQ_VqqHqqWqq_V256,
    // [EVEX.128.66.0F38.W1 10 /r] VPSRLVW xmm1 {k1}{z}, xmm2, xmm3/m128
    VPSRLVW_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 10 /r] VPSRLVW ymm1 {k1}{z}, ymm2, ymm3/m256
    VPSRLVW_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 10 /r] VPSRLVW zmm1 {k1}{z}, zmm2, zmm3/m512
    VPSRLVW_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W0 45 /r] VPSRLVD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VPSRLVD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 45 /r] VPSRLVD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VPSRLVD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 45 /r] VPSRLVD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VPSRLVD_VdqqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 45 /r] VPSRLVQ xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VPSRLVQ_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 45 /r] VPSRLVQ ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VPSRLVQ_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 45 /r] VPSRLVQ zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VPSRLVQ_VdqqHdqqWdqq_E512,

    // [EVEX.128.66.0F3A.W0 25 /r ib] VPTERNLOGD xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst, imm8
    VPTERNLOGD_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 25 /r ib] VPTERNLOGD ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst, imm8
    VPTERNLOGD_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 25 /r ib] VPTERNLOGD zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst, imm8
    VPTERNLOGD_VdqqHdqqWdqqIb_E512,
    // [EVEX.128.66.0F3A.W1 25 /r ib] VPTERNLOGQ xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst, imm8
    VPTERNLOGQ_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 25 /r ib] VPTERNLOGQ ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst, imm8
    VPTERNLOGQ_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 25 /r ib] VPTERNLOGQ zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst, imm8
    VPTERNLOGQ_VdqqHdqqWdqqIb_E512,

    // [EVEX.128.66.0F38.W0 26 /r] VPTESTMB k2 {k1}, xmm2, xmm3/m128
    VPTESTMB_KGqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 26 /r] VPTESTMB k2 {k1}, ymm2, ymm3/m256
    VPTESTMB_KGqHqqWqq_E256,
    // [EVEX.521.66.0F38.W0 26 /r] VPTESTMB k2 {k1}, zmm2, zmm3/m512
    VPTESTMB_KGqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 26 /r] VPTESTMW k2 {k1}, xmm2, xmm3/m128
    VPTESTMW_KGqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 26 /r] VPTESTMW k2 {k1}, ymm2, ymm3/m256
    VPTESTMW_KGqHqqWqq_E256,
    // [EVEX.521.66.0F38.W1 26 /r] VPTESTMW k2 {k1}, zmm2, zmm3/m512
    VPTESTMW_KGqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W0 27 /r] VPTESTMD k2 {k1}, xmm2, xmm3/m128/m32bcst
    VPTESTMD_KGqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 27 /r] VPTESTMD k2 {k1}, ymm2, ymm3/m256/m32bcst
    VPTESTMD_KGqHqqWqq_E256,
    // [EVEX.521.66.0F38.W0 27 /r] VPTESTMD k2 {k1}, zmm2, zmm3/m512/m32bcst
    VPTESTMD_KGqHdqqWdqq_E512,
    // [EVEX.128.66.0F38.W1 27 /r] VPTESTMQ k2 {k1}, xmm2, xmm3/m128/m64bcst
    VPTESTMQ_KGqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 27 /r] VPTESTMQ k2 {k1}, ymm2, ymm3/m256/m64bcst
    VPTESTMQ_KGqHqqWqq_E256,
    // [EVEX.521.66.0F38.W1 27 /r] VPTESTMQ k2 {k1}, zmm2, zmm3/m512/m64bcst
    VPTESTMQ_KGqHdqqWdqq_E512,

    // [EVEX.128.F3.0F38.W0 26 /r] VPTESTNMB k2 {k1}, xmm2, xmm3/m128
    VPTESTNMB_KGqHdqWdq_E128,
    // [EVEX.256.F3.0F38.W0 26 /r] VPTESTNMB k2 {k1}, ymm2, ymm3/m256
    VPTESTNMB_KGqHqqWqq_E256,
    // [EVEX.521.F3.0F38.W0 26 /r] VPTESTNMB k2 {k1}, zmm2, zmm3/m512
    VPTESTNMB_KGqHdqqWdqq_E512,
    // [EVEX.128.F3.0F38.W1 26 /r] VPTESTNMW k2 {k1}, xmm2, xmm3/m128
    VPTESTNMW_KGqHdqWdq_E128,
    // [EVEX.256.F3.0F38.W1 26 /r] VPTESTNMW k2 {k1}, ymm2, ymm3/m256
    VPTESTNMW_KGqHqqWqq_E256,
    // [EVEX.521.F3.0F38.W1 26 /r] VPTESTNMW k2 {k1}, zmm2, zmm3/m512
    VPTESTNMW_KGqHdqqWdqq_E512,
    // [EVEX.128.F3.0F38.W0 27 /r] VPTESTNMD k2 {k1}, xmm2, xmm3/m128/m32bcst
    VPTESTNMD_KGqHdqWdq_E128,
    // [EVEX.256.F3.0F38.W0 27 /r] VPTESTNMD k2 {k1}, ymm2, ymm3/m256/m32bcst
    VPTESTNMD_KGqHqqWqq_E256,
    // [EVEX.521.F3.0F38.W0 27 /r] VPTESTNMD k2 {k1}, zmm2, zmm3/m512/m32bcst
    VPTESTNMD_KGqHdqqWdqq_E512,
    // [EVEX.128.F3.0F38.W1 27 /r] VPTESTNMQ k2 {k1}, xmm2, xmm3/m128/m64bcst
    VPTESTNMQ_KGqHdqWdq_E128,
    // [EVEX.256.F3.0F38.W1 27 /r] VPTESTNMQ k2 {k1}, ymm2, ymm3/m256/m64bcst
    VPTESTNMQ_KGqHqqWqq_E256,
    // [EVEX.521.F3.0F38.W1 27 /r] VPTESTNMQ k2 {k1}, zmm2, zmm3/m512/m64bcst
    VPTESTNMQ_KGqHdqqWdqq_E512,

    // [EVEX.128.66.0F3A.W1 50 /r ib] VRANGEPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst, imm8
    VRANGEPD_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 50 /r ib] VRANGEPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    VRANGEPD_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 50 /r ib] VRANGEPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    VRANGEPD_VdqqHdqqWdqqIb_E512,

    // [EVEX.128.66.0F3A.W0 50 /r ib] VRANGEPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst, imm8
    VRANGEPS_VdqHdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 50 /r ib] VRANGEPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst, imm8
    VRANGEPS_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 50 /r ib] VRANGEPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst, imm8
    VRANGEPS_VdqqHdqqWdqqIb_E512,

    // [EVEX.LIG.66.0F3A.W1 51 /r ib] VRANGESD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}, imm8
    VRANGESD_VdqHdqWdqIb_E,

    // [EVEX.LIG.66.0F3A.W0 51 /r ib] VRANGESS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}, imm8
    VRANGESS_VdqHdqWdqIb_E,

    // [EVEX.128.66.0F38.W1 4C /r] VRCP14PD xmm1 {k1}{z}, xmm2/m128/m64bcst
    VRCP14PD_VdqWdq_E128,
    // [EVEX.256.66.0F38.W1 4C /r] VRCP14PD ymm1 {k1}{z}, ymm2/m256/m64bcst
    VRCP14PD_VqqWqq_E256,
    // [EVEX.512.66.0F38.W1 4C /r] VRCP14PD zmm1 {k1}{z}, zmm2/m512/m64bcst
    VRCP14PD_VdqqWdqq_E512,

    // [EVEX.LIG.66.0F38.W1 4D /r] VRCP14SD xmm1 {k1}{z}, xmm2, xmm3/m64
    VRCP14SD_VdqHdqWq_E,

    // [EVEX.128.66.0F38.W0 4C /r] VRCP14PS xmm1 {k1}{z}, xmm2/m128/m32bcst
    VRCP14PS_VdqWdq_E128,
    // [EVEX.256.66.0F38.W0 4C /r] VRCP14PS ymm1 {k1}{z}, ymm2/m256/m32bcst
    VRCP14PS_VqqWqq_E256,
    // [EVEX.512.66.0F38.W0 4C /r] VRCP14PS zmm1 {k1}{z}, zmm2/m512/m32bcst
    VRCP14PS_VdqqWdqq_E512,

    // [EVEX.LIG.66.0F38.W0 4D /r] VRCP14SS xmm1 {k1}{z}, xmm2, xmm3/m32
    VRCP14SS_VdqHdqWd_E,

    // [EVEX.128.66.0F3A.W1 56 /r ib] VREDUCEPD xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VREDUCEPD_VdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 56 /r ib] VREDUCEPD ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VREDUCEPD_VqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 56 /r ib] VREDUCEPD zmm1 {k1}{z}, zmm2/m512/m64bcst, imm8
    VREDUCEPD_VdqqWdqqIb_E512,

    // [EVEX.LIG.66.0F3A.W1 57 /r ib] VREDUCESD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}, imm8
    VREDUCESD_VdqHdqWq_E,

    // [EVEX.128.66.0F3A.W0 56 /r ib] VREDUCEPS xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VREDUCEPS_VdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 56 /r ib] VREDUCEPS ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VREDUCEPS_VqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 56 /r ib] VREDUCEPS zmm1 {k1}{z}, zmm2/m512/m32bcst, imm8
    VREDUCEPS_VdqqWdqqIb_E512,

    // [EVEX.LIG.66.0F3A.W0 57 /r ib] VREDUCESS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}, imm8
    VREDUCESS_VdqHdqWd_E,

    // [EVEX.128.66.0F3A.W1 09 /r ib] VRNDSCALEPD xmm1 {k1}{z}, xmm2/m128/m64bcst, imm8
    VRNDSCALEPD_VdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W1 09 /r ib] VRNDSCALEPD ymm1 {k1}{z}, ymm2/m256/m64bcst, imm8
    VRNDSCALEPD_VqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 09 /r ib] VRNDSCALEPD zmm1 {k1}{z}, zmm2/m512/m64bcst{sae}, imm8
    VRNDSCALEPD_VdqqWdqqIb_E512,

    // [EVEX.LIG.66.0F3A.W1 0B /r ib] VRNDSCALESD xmm1 {k1}{z}, xmm2, xmm3/m64{sae}, imm8
    VRNDSCALESD_VdqHdqWqIb_E,

    // [EVEX.128.66.0F3A.W0 08 /r ib] VRNDSCALEPS xmm1 {k1}{z}, xmm2/m128/m32bcst, imm8
    VRNDSCALEPS_VdqWdqIb_E128,
    // [EVEX.256.66.0F3A.W0 08 /r ib] VRNDSCALEPS ymm1 {k1}{z}, ymm2/m256/m32bcst, imm8
    VRNDSCALEPS_VqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 08 /r ib] VRNDSCALEPS zmm1 {k1}{z}, zmm2/m512/m32bcst{sae}, imm8
    VRNDSCALEPS_VdqqWdqqIb_E512,

    // [EVEX.LIG.66.0F3A.W0 0A /r ib] VRNDSCALESS xmm1 {k1}{z}, xmm2, xmm3/m32{sae}, imm8
    VRNDSCALESS_VdqHdqWdIb_E,

    // [EVEX.128.66.0F38.W1 4E /r] VRSQRT14PD xmm1 {k1}{z}, xmm2/m128/m64bcst
    VRSQRT14PD_VdqWdq_E128,
    // [EVEX.256.66.0F38.W1 4E /r] VRSQRT14PD ymm1 {k1}{z}, ymm2/m256/m64bcst
    VRSQRT14PD_VqqWqq_E256,
    // [EVEX.512.66.0F38.W1 4E /r] VRSQRT14PD zmm1 {k1}{z}, zmm2/m512/m64bcst
    VRSQRT14PD_VdqqWdqq_E512,

    // [EVEX.LIG.66.0F38.W1 4F /r] VRSQRT14SD xmm1 {k1}{z}, xmm2, xmm3/m64
    VRSQRT14SD_VdqHdqWq_E,

    // [EVEX.128.66.0F38.W0 4E /r] VRSQRT14PS xmm1 {k1}{z}, xmm2/m128/m32bcst
    VRSQRT14PS_VdqWdq_E128,
    // [EVEX.256.66.0F38.W0 4E /r] VRSQRT14PS ymm1 {k1}{z}, ymm2/m256/m32bcst
    VRSQRT14PS_VqqWqq_E256,
    // [EVEX.512.66.0F38.W0 4E /r] VRSQRT14PS zmm1 {k1}{z}, zmm2/m512/m32bcst
    VRSQRT14PS_VdqqWdqq_E512,

    // [EVEX.LIG.66.0F38.W0 4F /r] VRSQRT14SS xmm1 {k1}{z}, xmm2, xmm3/m32
    VRSQRT14SS_VdqHdqWd_E,

    // [EVEX.128.66.0F38.W1 2C /r] VSCALEFPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VSCALEFPD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W1 2C /r] VSCALEFPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VSCALEFPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W1 2C /r] VSCALEFPD zmm1 {k1}{z}, zmm2, xmm3/m512/m64bcst
    VSCALEFPD_VdqqHdqqWdqq_E512,

    // [EVEX.LIG.66.0F38.W1 2D /r] VSCALEFSD xmm1 {k1}{z}, xmm2, xmm3/m64{er}
    VSCALEFSD_VdqHdqWq_E,

    // [EVEX.128.66.0F38.W0 2C /r] VSCALEFPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VSCALEFPS_VdqHdqWdq_E128,
    // [EVEX.256.66.0F38.W0 2C /r] VSCALEFPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VSCALEFPS_VqqHqqWqq_E256,
    // [EVEX.512.66.0F38.W0 2C /r] VSCALEFPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst{er}
    VSCALEFPS_VdqqHdqqWdqq_E512,

    // [EVEX.LIG.66.0F38.W0 2D /r] VSCALEFSS xmm1 {k1}{z}, xmm2, xmm3/m32{er}
    VSCALEFSS_VdqHdqWd_E,

    // [EVEX.128.66.0F38.W0 A2 /vsib] VSCATTERDPS vm32x {k1}, xmm1
    VSCATTERDPS_VMdVdq_E128,
    // [EVEX.256.66.0F38.W0 A2 /vsib] VSCATTERDPS vm32y {k1}, ymm1
    VSCATTERDPS_VMdVqq_E256,
    // [EVEX.512.66.0F38.W0 A2 /vsib] VSCATTERDPS vm32z {k1}, zmm1
    VSCATTERDPS_VMdVdqq_E512,
    // [EVEX.128.66.0F38.W1 A2 /vsib] VSCATTERDPD vm32x {k1}, xmm1
    VSCATTERDPD_VMdVdq_E128,
    // [EVEX.256.66.0F38.W1 A2 /vsib] VSCATTERDPD vm32y {k1}, ymm1
    VSCATTERDPD_VMdVqq_E256,
    // [EVEX.512.66.0F38.W1 A2 /vsib] VSCATTERDPD vm32z {k1}, zmm1
    VSCATTERDPD_VMdVdqq_E512,
    // [EVEX.128.66.0F38.W0 A3 /vsib] VSCATTERQPS vm64x {k1}, xmm1
    VSCATTERQPS_VMqVdq_E128,
    // [EVEX.256.66.0F38.W0 A3 /vsib] VSCATTERQPS vm64y {k1}, ymm1
    VSCATTERQPS_VMqVqq_E256,
    // [EVEX.512.66.0F38.W0 A3 /vsib] VSCATTERQPS vm64z {k1}, zmm1
    VSCATTERQPS_VMqVdqq_E512,
    // [EVEX.128.66.0F38.W1 A3 /vsib] VSCATTERQPD vm64x {k1}, xmm1
    VSCATTERQPD_VMqVdq_E128,
    // [EVEX.256.66.0F38.W1 A3 /vsib] VSCATTERQPD vm64y {k1}, ymm1
    VSCATTERQPD_VMqVqq_E256,
    // [EVEX.512.66.0F38.W1 A3 /vsib] VSCATTERQPD vm64z {k1}, zmm1
    VSCATTERQPD_VMqVdqq_E512,

    // [EVEX.256.66.0F3A.W0 23 /r ib] VSHUFF32X4 ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst, imm8
    VSHUFF32X4_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 23 /r ib] VSHUFF32X4 zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst, imm8
    VSHUFF32X4_VdqqHdqqWdqqIb_E512,
    // [EVEX.256.66.0F3A.W1 23 /r ib] VSHUFF64X2 ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    VSHUFF64X2_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 23 /r ib] VSHUFF64X2 zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    VSHUFF64X2_VdqqHdqqWdqqIb_E512,
    // [EVEX.256.66.0F3A.W0 43 /r ib] VSHUFI32X4 ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst, imm8
    VSHUFI32X4_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W0 43 /r ib] VSHUFI32X4 zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst, imm8
    VSHUFI32X4_VdqqHdqqWdqqIb_E512,
    // [EVEX.256.66.0F3A.W1 43 /r ib] VSHUFI64X2 ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst, imm8
    VSHUFI64X2_VqqHqqWqqIb_E256,
    // [EVEX.512.66.0F3A.W1 43 /r ib] VSHUFI64X2 zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst, imm8
    VSHUFI64X2_VdqqHdqqWdqqIb_E512,

    // [VEX.128.66.0F38.W0 0E /r] VTESTPS xmm1, xmm2/m128
    VTESTPS_VdqWdq_V128,
    // [VEX.256.66.0F38.W0 0E /r] VTESTPS ymm1, ymm2/m256
    VTESTPS_VqqWqq_V256,
    // [VEX.128.66.0F38.W0 0F /r] VTESTPD xmm1, xmm2/m128
    VTESTPD_VdqWdq_V128,
    // [VEX.256.66.0F38.W0 0F /r] VTESTPD ymm1, ymm2/m256
    VTESTPD_VqqWqq_V256,

    // [VEX.256.0F.WIG 77] VZEROALL
    VZEROALL,

    // [VEX.128.0F.WIG 77] VZEROUPPER
    VZEROUPPER,

    // [9B] WAIT
    WAIT,

    // [0F 09] WBINVD
    WBINVD,

    // [F3 0F 09] WBNOINVD
    WBNOINVD,

    // [F3 0F AE /2] WRFSBASE r32
    WRFSBASE_Gd,
    // [F3 REX.W 0F AE /2] WRFSBASE r64
    WRFSBASE_Gq,
    // [F3 0F AE /3] WRGSBASE r32
    WRGSBASE_Gd,
    // [F3 REX.W 0F AE /3] WRGSBASE r64
    WRGSBASE_Gq,

    // [0F 30] WRMSR
    WRMSR,

    // [NP 0F 01 EF] WRPKRU
    WRPKRU,

    // [0F 38 F6] WRSSD r/m32, r32
    WRSSD_EdGd,
    // [REX.W 0F 38 F6] WRSSQ r/m64, r64
    WRSSQ_EqGq,

    // [66 0F 38 F5] WRUSSD r/m32, r32
    WRUSSD_EdGd,
    // [66 REX.W 0F 38 F5] WRUSSQ r/m64, r64
    WRUSSQ_EqGq,

    // [F2] XACQUIRE
    XACQUIRE,
    // [F3] XRELEASE
    XRELEASE,

    // [C6 F8 ib] XABORT imm8
    XABORT_Ib,

    // [0F C0 /r] XADD r/m8, r8
    // [REX 0F C0 /r] XADD r/m8, r8
    XADD_EbGb,
    // [0F C1 /r] XADD r/m16, r16
    XADD_EwGw,
    // [0F C1 /r] XADD r/m32, r32
    XADD_EdGd,
    // [REX.W 0F C1 /r] XADD r/m64, r64
    XADD_EqGq,

    // [C7 F8] XBEGIN rel16
    XBEGIN_Iw,
    // [C7 F8] XBEGIN rel32
    XBEGIN_Id,

    // [90+rw] XCHG AX, r16
    // [90+rw] XCHG r16, AX
    XCHG_AXGw,
    // [90+rd] XCHG EAX, r32
    // [90+rd] XCHG r32, EAX
    XCHG_EAXGd,
    // [REX.W 90+rd] XCHG RAX, r64
    // [REX.W 90+rd] XCHG r64, RAX
    XCHG_RAXGd,
    // [86 /r] XCHG r/m8, r8
    // [REX 86 /r] XCHG r/m8, r8
    // [86 /r] XCHG r8, r/m8
    // [REX 86 /r] XCHG r8, r/m8
    XCHG_EbGb,
    // [87 /r] XCHG r/m16, r16
    // [87 /r] XCHG r16, r/m16
    XCHG_EwGw,
    // [87 /r] XCHG r/m32, r32
    // [87 /r] XCHG r32, r/m32
    XCHG_EdGd,
    // [REX.W 87 /r] XCHG r/m64, r64
    // [REX.W 87 /r] XCHG r64, r/m64
    XCHG_EqGq,

    // [NP 0F 01 D5] XEND
    XEND,

    // [NP 0F 01 D0] XGETBV
    XGETBV,

    // [D7] XLAT m8
    // [D7] XLATB
    // [REX.W D7] XLATB
    XLAT,

    // [34 ib] XOR AL, imm8
    XOR_ALIb,
    // [35 iw] XOR AX, imm16
    XOR_AXIw,
    // [35 id] XOR EAX, imm32
    XOR_EAXId,
    // [REX.W 35 id] XOR RAX, imm32
    XOR_RAXId,
    // [80 /6 ib] XOR r/m8, imm8
    // [REX 80 /6 ib] XOR r/m8, imm8
    XOR_EbIb,
    // [81 /6 iw] XOR r/m16, imm16
    XOR_EwIw,
    // [81 /6 id] XOR r/m32, imm32
    XOR_EdId,
    // [REX.W 81 /6 id] XOR r/m64, imm32
    XOR_EqId,
    // [83 /6 ib] XOR r/m16, imm8
    XOR_EwIb,
    // [83 /6 ib] XOR r/m32, imm8
    XOR_EdIb,
    // [REX.W 83 /6 ib] XOR r/m64, imm8
    XOR_EqIb,
    // [30 /r] XOR r/m8, r8
    // [REX 30 /r] XOR r/m8, r8
    XOR_EbGb,
    // [31 /r] XOR r/m16, r16
    XOR_EwGw,
    // [31 /r] XOR r/m32, r32
    XOR_EdGd,
    // [REX.W 31 /r] XOR r/m64, r64
    XOR_EqGq,
    // [32 /r] XOR r8, r/m8
    // [REX 32 /r] XOR r8, r/m8
    XOR_GbEb,
    // [33 /r] XOR r16, r/m16
    XOR_GwEw,
    // [33 /r] XOR r32, r/m32
    XOR_GdEd,
    // [REX.W 33 /r] XOR r64, r/m64
    XOR_GqEq,

    // [66 0F 57 /r] XORPD xmm1, xmm2/m128
    XORPD_VdqWdq,
    // [VEX.128.66.0F.WIG 57 /r] VXORPD xmm1, xmm2, xmm3/m128
    VXORPD_VdqHdqWdq_V128,
    // [VEX.256.66.0F.WIG 57 /r] VXORPD ymm1, ymm2, ymm3/m256
    VXORPD_VqqHqqWqq_V256,
    // [EVEX.128.66.0F.W1 57 /r] VXORPD xmm1 {k1}{z}, xmm2, xmm3/m128/m64bcst
    VXORPD_VdqHdqWdq_E128,
    // [EVEX.256.66.0F.W1 57 /r] VXORPD ymm1 {k1}{z}, ymm2, ymm3/m256/m64bcst
    VXORPD_VqqHqqWqq_E256,
    // [EVEX.512.66.0F.W1 57 /r] VXORPD zmm1 {k1}{z}, zmm2, zmm3/m512/m64bcst
    VXORPD_VdqqHdqqWdqq_E512,

    // [NP 0F 57 /r] XORPS xmm1, xmm2/m128
    XORPS_VdqWdq,
    // [VEX.128.0F.WIG 57 /r] VXORPS xmm1, xmm2, xmm3/m128
    VXORPS_VdqHdqWdq_V128,
    // [VEX.256.0F.WIG 57 /r] VXORPS ymm1, ymm2, ymm3/m256
    VXORPS_VqqHqqWqq_V256,
    // [EVEX.128.0F.W1 57 /r] VXORPS xmm1 {k1}{z}, xmm2, xmm3/m128/m32bcst
    VXORPS_VdqHdqWdq_E128,
    // [EVEX.256.0F.W1 57 /r] VXORPS ymm1 {k1}{z}, ymm2, ymm3/m256/m32bcst
    VXORPS_VqqHqqWqq_E256,
    // [EVEX.512.0F.W1 57 /r] VXORPS zmm1 {k1}{z}, zmm2, zmm3/m512/m32bcst
    VXORPS_VdqqHdqqWdqq_E512,

    // [NP 0F AE /5] XRSTOR mem
    XRSTOR,
    // [NP REX.W 0F AE /5] XRSTOR64 mem
    XRSTOR64,

    // [NP 0F C7 /3] XRSTORS mem
    XRSTORS,
    // [NP REX.W 0F C7 /3] XRSTORS64 mem
    XRSTORS64,

    // [NP 0F AE /4] XSAVE mem
    XSAVE,
    // [NP REX.W 0F AE /4] XSAVE64 mem
    XSAVE64,

    // [NP 0F C7 /4] XSAVEC mem
    XSAVEC,
    // [NP REX.W 0F C7 /4] XSAVEC64 mem
    XSAVEC64,

    // [NP 0F AE /6] XSAVEOPT mem
    XSAVEOPT,
    // [NP REX.W 0F AE /6] XSAVEOPT64 mem
    XSAVEOPT64,

    // [NP 0F C7 /5] XSAVES mem
    XSAVES,
    // [NP REX.W 0F C7 /5] XSAVES64 mem
    XSAVES64,

    // [NP 0F 01 D1] XSETBV
    XSETBV,

    // [NP 0F 01 D6] XTEST
    XTEST,
}
