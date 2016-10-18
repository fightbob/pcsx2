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
#include "R5900.h"

using namespace Arm64Gen;

namespace R5900 {
namespace Dynarec {
namespace OpcodeImpl
{



void recBEQ(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    FixupBranch b = rec_branch<ARM64Reg, false, false>(rs,rt,CC_EQ);
}

void recBNE(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    FixupBranch b = rec_branch<ARM64Reg, false, false>(rs,rt,CC_NEQ);
}

void recBEQL(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    FixupBranch b = rec_branch<ARM64Reg, false, true>(rs,rt,CC_EQ);
}

void recBNEL(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    FixupBranch b = rec_branch<ARM64Reg, false, true>(rs,rt,CC_NEQ);
}

void recBLTZAL(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, true, false>(rs,0,CC_LT);
}

void recBGEZAL(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, true, false>(rs,0,CC_GE);
}

void recBLTZALL(opcode_t op)
{
        ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

        FixupBranch b = rec_branch<u32, true, true>(rs,0,CC_LT);
}

void recBGEZALL(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, true, true>(rs,0,CC_GE);
}

void recBLEZ(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, false, false>(rs,0,CC_LE);
}

void recBGTZ(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, false, false>(rs,0,CC_GT);
}

void recBLTZ(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, false, false>(rs,0,CC_LT);
}

void recBGEZ(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, false, false>(rs,0,CC_GE);
}

void recBLTZL(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, false, true>(rs,0,CC_LT);
}

void recBGEZL(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, false, true>(rs,0,CC_GE);
}

void recBLEZL(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, false, true>(rs,0,CC_LE);
}

void recBGTZL(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, false, true>(rs,0,CC_GT);
}

} } }
