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

using namespace x86Emitter;

namespace R5900 {
namespace Dynarec {


namespace OpcodeImpl {

void recCACHE() {

}


void recPREF()
{
}

void recSYNC()
{
}

void recMFSA()
{

}

void recMTSA()
{

}

void recMTSAB()
{

}

void recMTSAH()
{

}

////////////////////////////////////////////////////
void recNULL()
{
	Console.Error("EE: Unimplemented op %x", cpuRegs.code);
}

////////////////////////////////////////////////////
void recUnknown()
{
	// TODO : Unknown ops should throw an exception.
	Console.Error("EE: Unrecognized op %x", cpuRegs.code);
}

void recMMI_Unknown()
{
	// TODO : Unknown ops should throw an exception.
	Console.Error("EE: Unrecognized MMI op %x", cpuRegs.code);
}

void recCOP0_Unknown()
{
	// TODO : Unknown ops should throw an exception.
	Console.Error("EE: Unrecognized COP0 op %x", cpuRegs.code);
}

void recCOP1_Unknown()
{
	// TODO : Unknown ops should throw an exception.
	Console.Error("EE: Unrecognized FPU/COP1 op %x", cpuRegs.code);
}

void recTGE()
{
	recBranchCall( R5900::Interpreter::OpcodeImpl::TGE );
}

void recTGEU()
{
	recBranchCall( R5900::Interpreter::OpcodeImpl::TGEU );
}

void recTLT()
{
	recBranchCall( R5900::Interpreter::OpcodeImpl::TLT );
}

void recTLTU()
{
	recBranchCall( R5900::Interpreter::OpcodeImpl::TLTU );
}

void recTEQ()
{
	recBranchCall( R5900::Interpreter::OpcodeImpl::TEQ );
}

void recTNE()
{
	recBranchCall( R5900::Interpreter::OpcodeImpl::TNE );
}

void recTGEI()
{
	recBranchCall( R5900::Interpreter::OpcodeImpl::TGEI );
}

void recTGEIU()
{
	recBranchCall( R5900::Interpreter::OpcodeImpl::TGEIU );
}

void recTLTI()
{
	recBranchCall( R5900::Interpreter::OpcodeImpl::TLTI );
}

void recTLTIU()
{
	recBranchCall( R5900::Interpreter::OpcodeImpl::TLTIU );
}

void recTEQI()
{
	recBranchCall( R5900::Interpreter::OpcodeImpl::TEQI );
}

void recTNEI()
{
	recBranchCall( R5900::Interpreter::OpcodeImpl::TNEI );
}

} }}		// end Namespace R5900::Dynarec::OpcodeImpl
