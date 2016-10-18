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
namespace OpcodeImpl {

}

void recLB(opcode_t op)
{
    ARM64Reg rt = rec_load<s8>(op);
    SXTB(rt,rt);
}

void recLBU(opcode_t op)
{
    ARM64Reg rt = rec_load<u8>(op);
    UXTB(rt,rt); //todo: necessary?
}

void recLH(opcode_t op)
{
    ARM64Reg rt = rec_load<s16>(op);
    SXTH(rt,rt);
}

void recLHU(opcode_t op)
{
    ARM64Reg rt = rec_load<u16>(op);
    UXTH(rt,rt); // todo: necessary?
}

void recLW(opcode_t op)
{
    ARM64Reg rt = rec_load<s32>(op);
    SXTW(rt,rt);
}

void recLWU(opcode_t op)
{
    ARM64Reg rt = rec_load<u32>(op);
}

void recLD(opcode_t op)
{
    ARM64Reg rt = rec_load<u64>(op);
}

void recLQ(opcode_t op)
{

}

void recSB(opcode_t op)
{
    rec_store<u8>(op);
}
void recSH(opcode_t op)
{
    rec_store<u16>(op);

}
void recSW(opcode_t op)
{
    rec_store<u32>(op);

}
void recSQ(opcode_t op)
{

}
void recSD(opcode_t op)
{
    rec_store<u64>(op);

}


void recLWL(opcode_t op)
{
}

void recLWR(opcode_t op)
{

}

void recSWL(opcode_t op)
{
}

void recSWR(opcode_t op)
{
}

void recLDL(opcode_t op)
{
}

void recLDR(opcode_t op)
{
}

void recSDL(opcode_t op)
{
}

void recSDR(opcode_t op)
{
}


void recLWC1(opcode_t op)
{
}

void recSWC1(opcode_t op)
{
}

void recLQC2(opcode_t op)
{
}

void recSQC2(opcode_t op)
{
}


} } }	// end namespace R5900::Dynarec::OpcodeImpl
