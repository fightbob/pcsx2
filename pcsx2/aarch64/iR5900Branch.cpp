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


// recompiler reworked to add dynamic linking zerofrog(@gmail.com) Jan06

#include "PrecompiledHeader.h"

#include "Common.h"
#include "Arm64Rec.h"
#include "Arm64Emitter.h"

using namespace Arm64Gen;

namespace R5900 {
namespace Dynarec {
namespace OpcodeImpl
{


void recBEQ(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    s32 offset = op.offset();

    CMP(rs,rt);
    B
}

void recBNE(opcode_t op)
{

}

void recBEQL(opcode_t op)
{

}

void recBNEL(opcode_t op)
{

}

void recBLTZAL(opcode_t op)
{
}

void recBGEZAL(opcode_t op)
{
}

void recBLTZALL(opcode_t op)
{
}

void recBGEZALL(opcode_t op)
{
}

void recBLEZ(opcode_t op)
{
}

void recBGTZ(opcode_t op)
{
}

void recBLTZ(opcode_t op)
{
}

void recBGEZ(opcode_t op)
{
}

void recBLTZL(opcode_t op)
{
}

void recBGEZL(opcode_t op)
{
}

void recBLEZL(opcode_t op)
{
}

void recBGTZL(opcode_t op)
{
}

} } }
