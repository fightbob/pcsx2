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


// Important Note to Future Developers:
//   None of the COP0 instructions are really critical performance items,
//   so don't waste time converting any more them into recompiled code
//   unless it can make them nicely compact.  Calling the C versions will
//   suffice.

#include "PrecompiledHeader.h"

#include "Common.h"
#include "R5900OpcodeTables.h"
#include "iR5900.h"
#include "iCOP0.h"

namespace Interp = R5900::Interpreter::OpcodeImpl::COP0;

namespace R5900 {
namespace Dynarec {
namespace OpcodeImpl {
namespace COP0 {



void recBC0F()
{

}

void recBC0T()
{

}

void recBC0FL()
{

}

void recBC0TL()
{

}

void recTLBR() {  }
void recTLBP() {  }
void recTLBWI() {  }
void recTLBWR() {  }

void recERET()
{

}

void recEI()
{

}

void recDI()
{

}



void recMFC0()
{
	
}

void recMTC0()
{

}




}}}}
