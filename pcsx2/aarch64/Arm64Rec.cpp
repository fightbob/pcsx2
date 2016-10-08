/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2010  urisma
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
#include "Arm64Emitter.h"
#include "Arm64Reg.h"
#include "R5900.h"
#include "Memory.h"

using namespace Arm64Gen;



const std::array<ARM64Reg,26> aarch64_reg_alloc_order =
{
    W19,
    W20,
    W21,
    W22,
    W23,
    W24,
    W25,
    W18,
    W17,
    W16,
    W15,
    W14,
    W13,
    W12,
    W11,
    W10,
    W9,
    W8,
    W7,
    W6,
    W5,
    W4,
    W3,
    W2,
    W1,
    W0
};

const std::array<ARM64Reg,10> aarch64_callee_saved_regs =
{
  W19,
  W20,
  W21,
  W22,
  W23,
  W24,
  W25,
  W26,
  W27,
  W28
};

std::map<Arm64Reg, mips_reg_e> aarch64_current_reg_mapping;
std::map<Arm64Reg, reg_status_e> aarch64_current_reg_status;

constexpr bool aarch64_is_callee_saved_register(ARM64Reg reg)
{
  return std::find(std::begin(aarch64_callee_saved_regs), std::end(aarch64_callee_saved_regs), reg) != std::end(aarch64_callee_saved_regs);
}

// rec functions are expected to free all temp registers before returning
// so check if they actually did by checking the return value of this
int aarch64_get_num_temp_regs_in_use()
{
    int in_use = 0;
    for (const auto& status : aarch64_current_reg_status)
    {
        if (status.second == reg_status_e::USED)
        {
            in_use++;
        }
    }
    return in_use;
}

ARM64Reg aarch64_get_free_reg()
{
	for (ARM64Reg reg : aarch64_reg_alloc_order)
	{
		if (aarch64_current_reg_status[reg] == reg_status_e::UNUSED)
		{
            aarch64_current_reg_status[arm_reg] = reg_status_e::USED;
		    return reg;
		}
	}
	//TODO: this is a possibility but will take more code. Handle it when we run into it
	assert(false);
	return ARM64Reg::INVALID_REG;
}

void aarch64_free_reg(ARM64Reg reg)
{
    assert(aarch64_current_reg_status[arm_reg] == reg_status_e::USED);

    aarch64_current_reg_status[arm_reg] = reg_status_e::UNUSED;
}

void aarch64_map_reg(ARM64Reg arm_reg, mips_reg_e mips_reg)
{
    assert(aarch64_current_reg_mapping[arm_reg] == mips_reg_e::INVALID);
    assert(aarch64_current_reg_status[arm_reg] == reg_status_e::USED);

    //TODO: handle both 32 bit and 64 bit arm regs?
    aarch64_current_reg_mapping[arm_reg] = mips_reg;
    aarch64_current_reg_status[arm_reg] = reg_status_e::MAPPED;

    aarch64_load_from_mips_ctx(mips_reg, arm_reg);
}

//TODO: we only need to know arm_reg. drop the mips_reg?
void aarch64_unmap_reg(ARM64Reg arm_reg, mips_reg_e mips_reg)
{
    assert(aarch64_current_reg_mapping[arm_reg] == mips_reg);
    assert(aarch64_current_reg_status[arm_reg] == reg_status_e::MAPPED);

    aarch64_current_reg_map[arm_reg] = mips_reg_E::INVALID;
    aarch64_current_reg_status[arm_reg] = reg_status_e::USED;

    aarch64_flush_to_mips_ctx(mips_reg, arm_reg);
}

ARM64Reg aarch64_get_mapped_reg(mips_reg_e mips_reg)
{
    assert(mips_reg != mips_reg_e::INVALID);

    for (ARM64Reg aarch64_reg : aarch64_reg_alloc_order)
	{
		if (aarch64_current_reg_mapping[aarch64_reg] == mips_reg)
		{
		    return aarch64_reg;
		}
	}
	//mips_reg isn't mapped, so map it here
	arm_reg = aarch64_get_free_reg();
    aarch64_map_reg(arm_reg, mips_reg);

	return arm_reg;
}

void aarch64_load_from_mips_ctx(mips_reg_e mips_reg, ARM64Reg arm_reg)
{
    LDR(INDEX_UNSIGNED, arm_reg, MIPS_CPU_CTX_REG, MIPS_CPU_CTX_REG(GPR.r[mips_reg]));
}

void aarch64_flush_to_mips_ctx(mips_reg_e mips_reg, ARM64Reg arm_reg)
{
    STR(INDEX_UNSIGNED, arm_reg, MIPS_CPU_CTX_REG, MIPS_CPU_CTX_REG(GPR.r[mips_reg]));
}

void aarch64_flush_all_regs()
{
    for (const auto& mapping : aarch64_current_reg_mapping)
    {
        if (mapping.second != mips_reg_e::INVALID)
        {
            aarch64_flush_to_mips_ctx(mapping.second, mapping.first);
        }
    }
}

void aarch64_flush_and_unmap_all_regs()
{
    for (const auto& mapping : aarch64_current_reg_mapping)
    {
        if (mapping.second != mips_reg_e::INVALID)
        {
            aarch64_flush_to_mips_ctx(mapping.second, mapping.first);
            aarch64_unmap_reg(mapping.first, mapping.second);
        }
    }
}

void aarch64_load_all_mapped_regs()
{
    for (const auto& mapping : aarch64_current_reg_mapping)
    {
        if (mapping.second != mips_reg_e::INVALID)
        {
            aarch64_load_from_mips_ctx(mapping.second, mapping.first);
        }
    }
}

void aarch64_flush_callee_saved_regs()
{
    for (const auto& mapping : aarch64_current_reg_mapping)
    {
        if (mapping.second != mips_reg_e::INVALID && aarch64_is_callee_saved_register(mapping.first))
        {
            aarch64_flush_to_mips_ctx(mapping.second, mapping.first);
        }
    }
}



/*TODO: make all of this variadic templates and what not with this new fangled c++11 */

typedef void(*thunk_func_f)();

u8 thunk_mem_read_8(u32 addr)
{
    return memRead8(addr);
}
u16 thunk_mem_read_16(u32 addr)
{
    return memRead16(addr);
}
u32 thunk_mem_read_32(u32 addr)
{
    return memRead32(addr);
}
u64 thunk_mem_read_64(u32 addr)
{
    u64 temp;
    memRead64(addr, &temp);
    return temp;
}
u128 thunk_mem_read_128(u32 addr)
{
    u128 temp;
    memRead128(addr,&temp);
    return temp;
}

void thunk_mem_write_8(u32 addr, u8 data)
{
    memWrite8(addr, data);
}
void thunk_mem_write_16(u32 addr, u16 data)
{
    memWrite16(addr, data);
}
void thunk_mem_write_32(u32 addr, u32 data)
{
    memWrite32(addr,data);
}
void thunk_mem_write_64(u32 addr, u64 data)
{
    memWrite64(addr, data);
}
void thunk_mem_write_128(u32 addr, u128 data)
{
    memWrite128(addr,data);
}

void thunk_interpret(opcode_t op)
{
    //TODO: is PC updated here? update PC here. Or update before
    u32 pc = cpuRegs.pc;
    cpuRegs.pc += 4;

    cpuRegs.code = memRead32( pc );

    const OPCODE& opcode = GetInstruction(op.opcode);

    cpuBlockCycles += opcode.cycles;

    opcode.interpret();
}



const thunk_funcs_t aarch64_thunk_funcs =
{
    thunk_mem_read_8,
    thunk_mem_read_16,
    thunk_mem_read_32,
    thunk_mem_read_64,
    thunk_mem_read_128,
    thunk_mem_write_8,
    thunk_mem_write_16,
    thunk_mem_write_32,
    thunk_mem_write_64,
    thunk_mem_write_128,
    thunk_interpret
};