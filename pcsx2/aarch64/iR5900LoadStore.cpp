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

void recLB(opcode_t op)
{
    ARM64Reg base = aarch64_get_mapped_reg(op.base());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    u32 imm = op.uimm16();

    //TODO: some optimization can be done with immediates that are 12 bits or
    //fewer, but those can come later and will really only be useful when we
    //are doing reads and writes in the dynarec, rather than calling back into
    //the aarch64 thunk

    ARM64Reg temp_reg = aarch64_get_free_reg();
    MOVZ(temp_reg, imm);
    SXTH(EncodeRegTo32(temp_reg), EncodeRegTo32(temp_reg));
    ADD(temp_reg, base, temp_reg);
    //at this point, temp_reg has the ps2_addr_t we want to read from

}
void recLBU()
{

}
void recLH()
{

}
void recLHU()
{

}
void recLW()
{

}
void recLWU()
{

}
void recLD()
{

}
void recLQ()
{

}

void recSB()
{

}
void recSH()
{

}
void recSW()
{

}
void recSQ()
{

}
void recSD()
{

}


void recLWL()
{
}

void recLWR()
{

}

void recSWL()
{
}

void recSWR()
{
}

void recLDL()
{
}

void recLDR()
{
}

void recSDL()
{
}

void recSDR()
{
}


void recLWC1()
{
}

void recSWC1()
{
}

void recLQC2()
{
}

void recSQC2()
{
}


} } }	// end namespace R5900::Dynarec::OpcodeImpl
