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
#ifndef _R3000A_SUPERREC_
#define _R3000A_SUPERREC_

#include "x86emitter/x86emitter.h"
#include "R3000A.h"
#include "iCore.h"

// Cycle penalties for particularly slow instructions.
static const int psxInstCycles_Mult = 7;
static const int psxInstCycles_Div = 40;

// Currently unused (iop mod incomplete)
static const int psxInstCycles_Peephole_Store = 0;
static const int psxInstCycles_Store = 0;
static const int psxInstCycles_Load = 0;

// to be consistent with EE
#define PSX_HI XMMGPR_HI
#define PSX_LO XMMGPR_LO

extern uptr psxRecLUT[];

u8 _psxLoadWritesRs(u32 tempcode);
u8 _psxIsLoadStore(u32 tempcode);

void _psxFlushAllUnused();
int _psxFlushUnusedConstReg();
void _psxFlushCachedRegs();
void _psxFlushConstReg(int reg);
void _psxFlushConstRegs();

void _psxDeleteReg(int reg, int flush);
void _psxFlushCall(int flushtype);

void _psxOnWriteReg(int reg);

void _psxMoveGPRtoR(const x86Emitter::xRegisterLong& to, int fromgpr);

extern u32 psxpc;			// recompiler pc
extern int psxbranch;		// set for branch
extern u32 g_iopCyclePenalty;

void psxSaveBranchState();
void psxLoadBranchState();

extern void psxSetBranchReg(u32 reg);
extern void psxSetBranchImm( u32 imm );
extern void psxRecompileNextInstruction(int delayslot);


#endif
