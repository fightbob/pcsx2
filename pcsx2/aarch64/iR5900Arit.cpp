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

void recDADD()
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    ADD(EncodeRegTo64(rd), EncodeRegTo64(rn), EncodeRegTo64(rm));
}

void recSUB()
{
}

void recSUBU(void)
{
}


void recDSUB()
{
}

void recDSUBU(void)
{
}

void recAND()
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    AND
}

void recOR()
{
}

void recXOR()
{
}


void recNOR()
{
}



void recSLT()
{
}


void recSLTU()
{
}


} } }
