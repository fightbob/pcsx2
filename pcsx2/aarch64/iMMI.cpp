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


/*********************************************************
*   cached MMI opcodes                                   *
*                                                        *
*********************************************************/

#include "PrecompiledHeader.h"

#include "Common.h"
#include "R5900OpcodeTables.h"
#include "iR5900.h"
#include "iMMI.h"
#include "Utilities/MathUtils.h"


namespace Interp = R5900::Interpreter::OpcodeImpl::MMI;

namespace R5900 {
namespace Dynarec {
namespace OpcodeImpl {
namespace MMI
{



void recPLZCW()
{
}

void recPMFHL()
{
}

void recPMTHL()
{
}

void recPSRLH()
{
}

void recPSRLW()
{
}

void recPSRAH()
{
}

void recPSRAW()
{
}

void recPSLLH()
{
}

void recPSLLW()
{
}

void recPMAXW()
{
}

void recPPACW()
{
}

void recPPACH()
{
}

void recPPACB()
{
}

void recPEXT5()
{
}

void recPPAC5()
{
}

void recPMAXH()
{
}

void recPCGTB()
{
}

void recPCGTH()
{
}

void recPCGTW()
{
}

void recPADDSB()
{
}

void recPADDSH()
{
}

//NOTE: check kh2 movies if changing this
void recPADDSW()
{
}

void recPSUBSB()
{
}

void recPSUBSH()
{
}

//NOTE: check kh2 movies if changing this
void recPSUBSW()
{
}

void recPADDB()
{

}

void recPADDH()
{
}

void recPADDW()
{
}

void recPSUBB()
{
}

void recPSUBH()
{
}

void recPSUBW()
{
}

void recPEXTLW()
{
}

void recPEXTLB()
{
}

void recPEXTLH()
{
}


void recPABSW() //needs clamping
{
}


void recPABSH()
{
}

void recPMINW()
{
}

void recPADSBH()
{
}

void recPADDUW()
{
}

void recPSUBUB()
{
}

void recPSUBUH()
{
}

void recPSUBUW()
{
}

void recPEXTUH()
{
}

void recQFSRV()
{
}


void recPEXTUB()
{
}

void recPEXTUW()
{
}

void recPMINH()
{
}

void recPCEQB()
{
}

void recPCEQH()
{
}

void recPCEQW()
{
}

void recPADDUB()
{
}

void recPADDUH()
{
}

void recPMADDW()
{
}

void recPSLLVW()
{
}

void recPSRLVW()
{
}

void recPMSUBW()
{
}

void recPMULTW()
{
}
void recPDIVW()
{
}

void recPDIVBW()
{
}


//upper word of each doubleword in LO and HI is undocumented/undefined
//contains the upper multiplication result (before the addition with the lower multiplication result)
void recPHMADH()
{
}

void recPMSUBH()
{
}

//upper word of each doubleword in LO and HI is undocumented/undefined
//it contains the NOT of the upper multiplication result (before the substraction of the lower multiplication result)
void recPHMSBH()
{
}

void recPEXEH()
{
}

void recPREVH()
{
}

void recPINTH()
{
}

void recPEXEW()
{
}

void recPROT3W()
{
}

void recPMULTH()
{
}

void recPMFHI()
{
}

void recPMFLO()
{
}

void recPAND()
{
}

void recPXOR()
{
}

void recPCPYLD()
{
}

void recPMADDH()
{
}

void recPSRAVW()
{
}



void recPINTEH()
{
}

void recPMULTUW()
{
}

void recPMADDUW()
{
}

void recPDIVUW()
{
}

void recPEXCW()
{
}

void recPEXCH()
{
}

void recPNOR()
{
}

void recPMTHI()
{
}

void recPMTLO()
{
}

void recPCPYUD()
{
}

void recPOR()
{
}

void recPCPYH()
{
}


} } } }
