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

namespace R5900 {
namespace Dynarec {
namespace OpcodeImpl
{


void recSLL(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    int sa = op.sa();

    LSL(EncodeRegTo32(rd),rt,sa);
    SXTW(rd,rd);
}

void recSRL(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    int sa = op.sa();

    LSR(EncodeRegTo32(rd),rt,sa);
    SXTW(rd,rd);
}


void recSRA(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    int sa = op.sa();

    ASR(EncodeRegTo32(rd),rt,sa);
    SXTW(rd,rd);
}


void recDSLL(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    int sa = op.sa();

    LSL(EncodeRegTo64(rd),rt,sa);
}

void recDSRL(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    int sa = op.sa();

    LSR(EncodeRegTo64(rd),rt,sa);
}


void recDSRA(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    int sa = op.sa();

    ASR(EncodeRegTo64(rd),rt,sa);
}


void recDSLL32(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    int sa = op.sa();

    LSL(EncodeRegTo64(rd),rt,sa+32);
}

void recDSRL32(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    int sa = op.sa();

    LSR(EncodeRegTo64(rd),rt,sa+32);
}


void recDSRA32(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    int sa = op.sa();

    ASR(EncodeRegTo64(rd),rt,sa+32);
}


void recSLLV(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    LSLV(EncodeRegTo32(rd),rs,rt);
    SXTW(rd,rd);
}


void recSRLV(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    LSRV(EncodeRegTo32(rd),rs,rt);
    SXTW(rd,rd);
}


void recSRAV(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    ASRV(EncodeRegTo32(rd),rs,rt);
    SXTW(rd,rd);
}


void recDSLLV(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    LSLV(EncodeRegTo64(rd),rs,rt);
}


void recDSRLV(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    LSRV(EncodeRegTo64(rd),rs,rt);
}


void recDSRAV(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    ASRV(EncodeRegTo64(rd),rs,rt);
}

} } }
