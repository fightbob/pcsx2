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
#include "R5900OpcodeTables.h"
#include "iR5900.h"
#include "iFPU.h"

namespace R5900 {
namespace Dynarec {
namespace OpcodeImpl {
namespace COP1 {

namespace DOUBLE {

void recABS_S();
void recADD_S();
void recADDA_S();
void recC_EQ();
void recC_LE();
void recC_LT();
void recCVT_S();
void recCVT_W();
void recDIV_S();
void recMADD_S();
void recMADDA_S();
void recMAX_S();
void recMIN_S();
void recMOV_S();
void recMSUB_S();
void recMSUBA_S();
void recMUL_S();
void recMULA_S();
void recNEG_S();
void recSUB_S();
void recSUBA_S();
void recSQRT_S();
void recRSQRT_S();

};

// FCR31 Flags
#define FPUflagC	0X00800000
#define FPUflagI	0X00020000
#define FPUflagD	0X00010000
#define FPUflagO	0X00008000
#define FPUflagU	0X00004000
#define FPUflagSI	0X00000040
#define FPUflagSD	0X00000020
#define FPUflagSO	0X00000010
#define FPUflagSU	0X00000008


void recCFC1()
{

}

void recCTC1()
{

}

void recMFC1()
{
	
}

void recMTC1()
{
	
}
void recABS_S()
{

}

void recADD_S()
{
}

void recADDA_S()
{
}

void recBC1F()
{
}

void recBC1T()
{
}

void recBC1FL()
{
}

void recBC1TL()
{
}

void recC_EQ()
{

}

void recC_F()
{
}

void recC_LE()
{
}

void recC_LT()
{
}
void recCVT_S()
{
}

void recCVT_W()
{
}

void recDIV_S()
{
}

void recMADD_S()
{
}

void recMADDA_S()
{
}

void recMAX_S()
{
}

void recMIN_S()
{
}

void recMOV_S()
{
}

void recMSUB_S()
{
}

void recMSUBA_S()
{
}

void recMUL_S()
{
}


void recMULA_S()
{
}

void recNEG_S() 
{
}

void recSUB_S()
{
}

void recSUBA_S()
{
}

void recSQRT_S()
{
}


void recRSQRT_S()
{
}

} } } }
