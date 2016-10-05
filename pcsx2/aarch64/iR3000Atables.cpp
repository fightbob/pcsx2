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
#include <time.h>

#include "IopCommon.h"
#include "iR3000A.h"
#include "IopMem.h"
#include "IopDma.h"



void rpsxADDIU()
{

}

void rpsxADDI() { }

void rpsxSLTI() {  }

void rpsxSLTIU() {  }

void rpsxANDI() {  }

void rpsxORI() {  }


void rpsxXORI() {  }

void rpsxLUI()
{

}

void rpsxADDU()
{

}

void rpsxADD() {  }


void rpsxSUBU()
{
}
void rpsxSUB() {  }
void rpsxAND() { }
void rpsxOR() {  }
void rpsxXOR() {  }
void rpsxNOR() { }
void rpsxSLT()
{
}
void rpsxSLTU()
{
}

void rpsxMULT() {  }
void rpsxMULTU() {  }

void rpsxDIV() {  }

void rpsxDIVU() {  }


static void rpsxLB()
{
}

static void rpsxLBU()
{
}

static void rpsxLH()
{
}

static void rpsxLHU()
{
}

static void rpsxLW()
{
}

static void rpsxSB()
{
}

static void rpsxSH()
{
}

static void rpsxSW()
{
}


void rpsxSLL() {  }

void rpsxSRL() {  }

void rpsxSRA() {  }

void rpsxSLLV()
{
}

void rpsxSRLV()
{
}

void rpsxSRAV()
{
}

void rpsxMFHI()
{
}

void rpsxMTHI()
{
}

void rpsxMFLO()
{
}

void rpsxMTLO()
{
}

void rpsxJ()
{
}

void rpsxJAL()
{
}

void rpsxJR()
{
}

void rpsxJALR()
{
}


void rpsxBEQ() {  }

void rpsxBNE() {  }

void rpsxBLTZ()
{
}

void rpsxBGEZ()
{
}

void rpsxBLTZAL()
{
}

void rpsxBGEZAL()
{
}

void rpsxBLEZ()
{
}

void rpsxBGTZ()
{
}

void rpsxMFC0()
{
}

void rpsxCFC0()
{
}

void rpsxMTC0()
{
}

void rpsxCTC0()
{
}

void rpsxRFE()
{
}



extern void (*rpsxBSC[64])();
extern void (*rpsxSPC[64])();
extern void (*rpsxREG[32])();
extern void (*rpsxCP0[32])();
extern void (*rpsxCP2[64])();
extern void (*rpsxCP2BSC[32])();

static void rpsxSPECIAL() { rpsxSPC[_Funct_](); }
static void rpsxREGIMM() { rpsxREG[_Rt_](); }
static void rpsxCOP0() { rpsxCP0[_Rs_](); }
static void rpsxCOP2() { rpsxCP2[_Funct_](); }
static void rpsxBASIC() { rpsxCP2BSC[_Rs_](); }

static void rpsxNULL() {
	Console.WriteLn("psxUNK: %8.8x", psxRegs.code);
}

void (*rpsxBSC[64])() = {
	rpsxSPECIAL, rpsxREGIMM, rpsxJ   , rpsxJAL  , rpsxBEQ , rpsxBNE , rpsxBLEZ, rpsxBGTZ,
	rpsxADDI   , rpsxADDIU , rpsxSLTI, rpsxSLTIU, rpsxANDI, rpsxORI , rpsxXORI, rpsxLUI ,
	rpsxCOP0   , rpsxNULL  , rpsxCOP2, rpsxNULL , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxNULL   , rpsxNULL  , rpsxNULL, rpsxNULL , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxLB     , rpsxLH    , rpsxLWL , rpsxLW   , rpsxLBU , rpsxLHU , rpsxLWR , rpsxNULL,
	rpsxSB     , rpsxSH    , rpsxSWL , rpsxSW   , rpsxNULL, rpsxNULL, rpsxSWR , rpsxNULL,
	rpsxNULL   , rpsxNULL  , rgteLWC2, rpsxNULL , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxNULL   , rpsxNULL  , rgteSWC2, rpsxNULL , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL
};

void (*rpsxSPC[64])() = {
	rpsxSLL , rpsxNULL, rpsxSRL , rpsxSRA , rpsxSLLV   , rpsxNULL , rpsxSRLV, rpsxSRAV,
	rpsxJR  , rpsxJALR, rpsxNULL, rpsxNULL, rpsxSYSCALL, rpsxBREAK, rpsxNULL, rpsxNULL,
	rpsxMFHI, rpsxMTHI, rpsxMFLO, rpsxMTLO, rpsxNULL   , rpsxNULL , rpsxNULL, rpsxNULL,
	rpsxMULT, rpsxMULTU, rpsxDIV, rpsxDIVU, rpsxNULL   , rpsxNULL , rpsxNULL, rpsxNULL,
	rpsxADD , rpsxADDU, rpsxSUB , rpsxSUBU, rpsxAND    , rpsxOR   , rpsxXOR , rpsxNOR ,
	rpsxNULL, rpsxNULL, rpsxSLT , rpsxSLTU, rpsxNULL   , rpsxNULL , rpsxNULL, rpsxNULL,
	rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL   , rpsxNULL , rpsxNULL, rpsxNULL,
	rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL   , rpsxNULL , rpsxNULL, rpsxNULL
};

void (*rpsxREG[32])() = {
	rpsxBLTZ  , rpsxBGEZ  , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxNULL  , rpsxNULL  , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxBLTZAL, rpsxBGEZAL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxNULL  , rpsxNULL  , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL
};

void (*rpsxCP0[32])() = {
	rpsxMFC0, rpsxNULL, rpsxCFC0, rpsxNULL, rpsxMTC0, rpsxNULL, rpsxCTC0, rpsxNULL,
	rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxRFE , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL
};

void (*rpsxCP2[64])() = {
	rpsxBASIC, rgteRTPS , rpsxNULL , rpsxNULL, rpsxNULL, rpsxNULL , rgteNCLIP, rpsxNULL, // 00
	rpsxNULL , rpsxNULL , rpsxNULL , rpsxNULL, rgteOP  , rpsxNULL , rpsxNULL , rpsxNULL, // 08
	rgteDPCS , rgteINTPL, rgteMVMVA, rgteNCDS, rgteCDP , rpsxNULL , rgteNCDT , rpsxNULL, // 10
	rpsxNULL , rpsxNULL , rpsxNULL , rgteNCCS, rgteCC  , rpsxNULL , rgteNCS  , rpsxNULL, // 18
	rgteNCT  , rpsxNULL , rpsxNULL , rpsxNULL, rpsxNULL, rpsxNULL , rpsxNULL , rpsxNULL, // 20
	rgteSQR  , rgteDCPL , rgteDPCT , rpsxNULL, rpsxNULL, rgteAVSZ3, rgteAVSZ4, rpsxNULL, // 28
	rgteRTPT , rpsxNULL , rpsxNULL , rpsxNULL, rpsxNULL, rpsxNULL , rpsxNULL , rpsxNULL, // 30
	rpsxNULL , rpsxNULL , rpsxNULL , rpsxNULL, rpsxNULL, rgteGPF  , rgteGPL  , rgteNCCT  // 38
};

void(*rpsxCP2BSC[32])() = {
	rgteMFC2, rpsxNULL, rgteCFC2, rpsxNULL, rgteMTC2, rpsxNULL, rgteCTC2, rpsxNULL,
	rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL
};

