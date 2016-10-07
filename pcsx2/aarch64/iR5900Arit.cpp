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


void recADD(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    ADD(EncodeRegTo32(rd), EncodeRegTo32(rn), EncodeRegTo32(rm));
    SXTW(EncodeRegTo64(rd), EncodeRegTo32(rd));
}

void recADDU(opcode_t op)
{
    recADD(op);
}

void recDADD(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    ADD(EncodeRegTo64(rd), EncodeRegTo64(rn), EncodeRegTo64(rm));
}

void recSUB(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    SUB(EncodeRegTo32(rd),EncodeRegTo32(rn),EncodeRegTo32(rm));
    SXTW(EncodeRegTo64(rd),EncodeRegTo64(rd));
}

void recSUBU(opcode_t op)
{
    recSUB();
}


void recDSUB(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    SUB(EncodeRegTo64(rd),EncodeRegTo64(rn),EncodeRegTo64(rm));
}

void recDSUBU(opcode_t op)
{
    recDSUB();
}

void recAND(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    AND(EncodeRegTo64(rd), EncodeRegTo64(rn), EncodeRegTo64(rm));
}

void recOR(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    ORR(EncodeRegTo64(rd), EncodeRegTo64(rn), EncodeRegTo64(rm));
}

void recXOR(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    EOR(EncodeRegTo64(rd), EncodeRegTo64(rn), EncodeRegTo64(rm));
}


void recNOR(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    ORN(EncodeRegTo64(rd),EncodeRegTo64(rn),EncodeRegTo64(rm));
}



void recSLT(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    CMP(EncodeRegTo64(rn),EncodeRegTo64(rm));
    CSET(EncodeRegTo64(rd),CC_LT);
}


void recSLTU(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    CMP(EncodeRegTo64(rn),EncodeRegTo64(rm));
    CSET(EncodeRegTo64(rd),CC_LO);
}


} } }
