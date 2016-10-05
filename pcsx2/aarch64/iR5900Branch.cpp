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
#include "R5900OpcodeTables.h"
#include "iR5900.h"

using namespace x86Emitter;

namespace R5900 {
namespace Dynarec {
namespace OpcodeImpl
{


void recBEQ_(int info) { recBEQ_process(info, 0); }

void recBNE_(int info) { recBNE_process(info, 0); }

void recBEQL_(int info) { recBEQL_process(info, 0); }

void recBNEL_(int info) { recBNEL_process(info, 0); }

void recBLTZAL()
{
}

void recBGEZAL()
{
}

void recBLTZALL()
{
}

void recBGEZALL()
{
}

void recBLEZ()
{
}

void recBGTZ()
{
}

void recBLTZ()
{
}

void recBGEZ()
{
}

void recBLTZL()
{
}

void recBGEZL()
{
}

void recBLEZL()
{
}

void recBGTZL()
{
}

} } }
