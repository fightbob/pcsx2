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

#include "BaseblockEx.h"

class BasicBlock
{
    //basic block is separated into three parts: prologue, actual block, and epilogue
    // prologue : loads all registers from cpu ctx
    // actual block: the code that dos the actual emulation
    // epilogue: code that flushes all registers back to the cpu ctx
    // in general, we'll ju,p from emulator code to prologue when reenterin a block,
    // but should be able to link blocks fairly well. eventually we'll need a thunk to
    // link blocks with similar but not identical reg mappings
    uintptr_t entry, prologue, epilogue;
};

//map ps2addr ro basic block
std::map<u32, BasicBlock*> aarch64_basic_block_mapping;

bool is_branch_op(opcode_t op)
{
    //TODO
}

void aarch64_recompile_block(u32 start_pc)
{

}