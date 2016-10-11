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

using namespace Arm64Gen

namespace R5900 {
namespace Dynarec {
namespace OpcodeImpl
{


void recLUI(opcode_t op)
{
    u32 imm = op.uimm16();
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    MOVZ(rt, imm, SHIFT_16);
    SXTW(rt, rt);
}

void recMFHI(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg hi = aarch64_get_mapped_reg(mips_reg_e::HI);

    MOV(EncodeRegTo64(rd),hi);
}

void recMFLO(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg lo = aarch64_get_mapped_reg(mips_reg_e::LO);

    MOV(EncodeRegTo64(rd),lo);
}

void recMTHI(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg hi = aarch64_get_mapped_reg(mips_reg_e::HI);

    MOV(EncodeRegTo64(hi), EncodeRegTo64(rs));
}

void recMTLO(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg hi = aarch64_get_mapped_reg(mips_reg_e::HI);

    MOV(EncodeRegTo64(hi), EncodeRegTo64(rs));
}


void recMFHI1(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    aarch64_load_upper64_from_mips_ctx(mips_reg_e::HI,rd);
}

void recMFLO1(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    aarch64_load_upper64_from_mips_ctx(mips_reg_e::LO,rd);
}
void recMTHI1(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    aarch64_flush_upper64_to_mips_ctx(mips_reg_e::HI,rs);
}

void recMTLO1(opcode_t op)
{
    ARM64Reg lo = aarch64_get_mapped_reg(op.rs());

    aarch64_flush_upper64_to_mips_ctx(mips_reg_e::LO,rs);
}


void recMOVZ(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    CMP(rs,0);
    CSEL(rd,rt,rd,CC_EQ);
}


void recMOVN(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    CMP(rs,0);
    CSEL(rd,rt,rd,CC_NEQ);
}


} } }
