/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2010  PCSX2 Dev Team
 *
 *  PCSX2 is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU Lesser General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 *
 *  PCSX2 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 *  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *  PURPOSE.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with PCSX2.
 *  If not, see <http://www.gnu.org/licenses/>.
 */


#include "PrecompiledHeader.h"


#include "Common.h"
#include "Arm64Rec.h"
#include "Arm64Emitter.h"

using namespace Arm64Gen;

constexpr int IMM12_MAX = ((1 << 12) - 1);

namespace R5900 {
namespace Dynarec {
namespace OpcodeImpl
{


void recADDI(opcode_t op)
{
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    u32 imm = op.uimm16();

    if (imm <= IMM12_MAX)
    {
        ADD(EncodeRegTo32(rt),EncodeRegTo32(rs),imm);
        SXTW(EncodeRegTo64(rt),EncodeRegTo64(rt));
    }
    else
    {
        ARM64Reg temp_reg = aarch64_get_free_reg();
        MOVZ(temp_reg,imm);
        ADD(EncodeRegTo32(rt),EncodeRegTo32(rs),temp_reg);
        SXTW(EncodeRegTo64(rt),EncodeRegTo64(rt));
        aarch64_free_reg(temp_reg);
    }
}

void recADDIU(opcode_t op)
{
    recADDI(op);
}

void recDADDI(opcode_t op)
{
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    u32 imm = op.uimm16();

    if (imm <= IMM12_MAX)
    {
        ADD(EncodeRegTo64(rt),EncodeRegTo64(rs),imm);
    }
    else
    {
        ARM64Reg temp_reg = aarch64_get_free_reg();
        MOVZ(temp_reg,imm);
        ADD(EncodeRegTo64(rt),EncodeRegTo64(rs),temp_reg);
        aarch64_free_reg(temp_reg);
    }
}

void recDADDIU(opcode_t op)
{
    recDADDI(op);
}

void recSLTIU(opcode_t op)
{
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    u32 imm = op.uimm16();

    ARM64Reg temp_reg = aarch64_get_free_reg();
    MOVZ(EncodeRegTo32(temp_reg),imm);
    SXTH(EncodeRegTo64(temp_reg),EncodeRegTo64(temp_reg));
    CMP(EncodeRegTo64(rs),EncodeRegTo64(temp_reg));
    CSET(EncodeRegTo64(rt),CC_LO);
}

void recSLTI(opcode_t op)
{
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    u32 imm = op.uimm16();

    ARM64Reg temp_reg = aarch64_get_free_reg();
    MOVZ(EncodeRegTo32(temp_reg),imm);
    SXTH(EncodeRegTo64(temp_reg),EncodeRegTo64(temp_reg));
    CMP(EncodeRegTo64(rs),EncodeRegTo64(temp_reg));
    CSET(EncodeRegTo64(rt),CC_LT);
}

void recANDI(opcode_t op)
{
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    u32 imm = op.uimm16();

    if (imm <= IMM12_MAX)
    {
        AND(EncodeRegTo64(rt),EncodeRegTo64(rs),imm);
    }
    else
    {
        ARM64Reg temp_reg = aarch64_get_free_reg();
        MOVZ(temp_reg,imm);
        AND(EncodeRegTo64(rt),EncodeRegTo64(rs),temp_reg);
        aarch64_free_reg(temp_reg);
    }
}

void recORI(opcode_t op)
{
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    u32 imm = op.uimm16();

    if (imm <= IMM12_MAX)
    {
        ORR(EncodeRegTo64(rt),EncodeRegTo64(rs),imm);
    }
    else
    {
        ARM64Reg temp_reg = aarch64_get_free_reg();
        MOVZ(temp_reg,imm);
        ORR(EncodeRegTo64(rt),EncodeRegTo64(rs),temp_reg);
        aarch64_free_reg(temp_reg);
    }
}


void recXORI(opcode_t op)
{
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    u32 imm = op.uimm16();

    if (imm <= IMM12_MAX)
    {
        EOR(EncodeRegTo64(rt),EncodeRegTo64(rs),imm);
    }
    else
    {
        ARM64Reg temp_reg = aarch64_get_free_reg();
        MOVZ(temp_reg,imm);
        EOR(EncodeRegTo64(rt),EncodeRegTo64(rs),temp_reg);
        aarch64_free_reg(temp_reg);
    }
}


} } }
