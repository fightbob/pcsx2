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
#include "Arm64Rec.h"
#include "R5900.h"
#include "Memory.h"
#include "../IopMem.h"

using namespace Arm64Gen;


const std::array<ARM64Reg,10> aarch64_callee_saved_regs =
{
	ARM64Reg::W19,
	ARM64Reg::W20,
	ARM64Reg::W21,
	ARM64Reg::W22,
	ARM64Reg::W23,
	ARM64Reg::W24,
	ARM64Reg::W25,
	ARM64Reg::W26,
	ARM64Reg::W27,
	ARM64Reg::W28
};

//LRU for keeping track of what register to allocate next
std::deque<ARM64Reg> aarch64_reg_alloc_lru =
{
	ARM64Reg::W19,
	ARM64Reg::W20,
	ARM64Reg::W21,
	ARM64Reg::W22,
	ARM64Reg::W23,
	ARM64Reg::W24,
	ARM64Reg::W25,
	ARM64Reg::W18,
	ARM64Reg::W17,
	ARM64Reg::W16,
	ARM64Reg::W15,
	ARM64Reg::W14,
	ARM64Reg::W13,
	ARM64Reg::W12,
	ARM64Reg::W11,
	ARM64Reg::W10,
	ARM64Reg::W9,
	ARM64Reg::W8,
	ARM64Reg::W7,
	ARM64Reg::W6,
	ARM64Reg::W5,
	ARM64Reg::W4,
	ARM64Reg::W3,
	ARM64Reg::W2,
	ARM64Reg::W1,
	ARM64Reg::W0
};

std::map<ARM64Reg, mips_reg_e> aarch64_current_reg_mapping;
std::map<ARM64Reg, reg_status_e> aarch64_current_reg_status;

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
    ARM64Reg arm_reg = aarch64_reg_alloc_lru.pop_front();

    switch(aarch64_current_reg_status[arm_reg])
    {
        case reg_status_e::UNUSED:
            aarch64_current_reg_status[arm_reg] = reg_status_e::USED;
            break;
        case reg_status_e::USED:
            // TODO: what to do? Case is very unlikley
            break;
        case reg_status_e::MAPPED:
            aarch64_unmap_reg(arm_reg, aarch64_current_reg_mapping[arm_reg]);
            aarch64_current_reg_status[arm_reg] = reg_status_e::USED;
    }
    aarch64_reg_alloc_lru.push_back(arm_reg);
    return arm_reg;
}

void aarch64_free_reg(ARM64Reg arm_reg)
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

// TODO: all of these need to keep track of 32/64 bit register getting passed in
// we'll be using this for both IOP and EE and need to flush / load 32 or 64 bits
// respectively
void aarch64_load_from_mips_ctx(ARM64Reg arm_reg, mips_reg_e mips_reg)
{
    LDR(INDEX_UNSIGNED, arm_reg, MIPS_CPU_CTX_REG, MIPS_CPU_CTX_REG(GPR.r[mips_reg]));
}

void aarch64_flush_to_mips_ctx(ARM64Reg arm_reg, mips_reg_e mips_reg)
{
    STR(INDEX_UNSIGNED, arm_reg, MIPS_CPU_CTX_REG, MIPS_CPU_CTX_REG(GPR.r[mips_reg]));
}

void aarch64_flush_to_mips_ctx(ARM64Reg arm_reg)
{
    aarch64_flush_to_mips_ctx(arm_reg, aarch64_current_reg_mapping[arm_reg]);
}

void aarch64_flush_to_mips_ctx(mips_reg_e mips_reg)
{
    for (const auto& mapping : aarch64_current_reg_mapping)
    {
        if (mapping.second == mips_reg)
        {
            aarch64_flush_to_mips_ctx(mapping.first, mips_reg);
        }
    }
    assert(false);
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

void aarch64_flush_caller_saved_regs()
{
    for (const auto& mapping : aarch64_current_reg_mapping)
    {
        if (mapping.second != mips_reg_e::INVALID && !aarch64_is_callee_saved_register(mapping.first))
        {
            aarch64_flush_to_mips_ctx(mapping.second, mapping.first);
        }
    }
}

void aarch64_recompile_instr(opcode_t op)
{

}

void aarch64_recompile_instr_at_addr(u32 ps2_addr)
{
    opcode_t op;
    op.opcode = memRead32(ps2_addr);
    aarch64_recompile_instr(op);
}

void aarch64_recompile_next_instr()
{
    aarch64_recompile_instr_at(cpuRegs.pc);
}


typedef void(*thunk_func_f)();

u8 thunk_ee_mem_read_8(u32 addr)
{
    return memRead8(addr);
}
u16 thunk_ee_mem_read_16(u32 addr)
{
    return memRead16(addr);
}
u32 thunk_ee_mem_read_32(u32 addr)
{
    return memRead32(addr);
}
u64 thunk_ee_mem_read_64(u32 addr)
{
    u64 temp;
    memRead64(addr, &temp);
    return temp;
}
u128 thunk_ee_mem_read_128(u32 addr)
{
    u128 temp;
    memRead128(addr,&temp);
    return temp;
}

void thunk_ee_mem_write_8(u32 addr, u8 data)
{
    memWrite8(addr, data);
}
void thunk_ee_mem_write_16(u32 addr, u16 data)
{
    memWrite16(addr, data);
}
void thunk_ee_mem_write_32(u32 addr, u32 data)
{
    memWrite32(addr,data);
}
void thunk_ee_mem_write_64(u32 addr, u64 data)
{
    memWrite64(addr, data);
}
void thunk_ee_mem_write_128(u32 addr, u128 data)
{
    memWrite128(addr,data);
}

void thunk_ee_interpret(opcode_t op)
{
    //TODO: is PC updated here? update PC here. Or update before
    u32 pc = cpuRegs.pc;
    cpuRegs.pc += 4;

    cpuRegs.code = memRead32( pc );

    const OPCODE& opcode = GetInstruction(op.opcode);

    cpuBlockCycles += opcode.cycles;

    opcode.interpret();
}

u8 thunk_iop_mem_read_8(u32 addr)
{
    return iopMemRead8(addr);
}
u16 thunk_iop_mem_read_16(u32 addr)
{
    return iopMemRead16(addr);
}
u32 thunk_iop_mem_read_32(u32 addr)
{
    return memRead32(addr);
}

void thunk_iop_mem_write_8(u32 addr, u8 data)
{
    iopMemWrite8(addr, data);
}
void thunk_iop_mem_write_16(u32 addr, u16 data)
{
    iopMemWrite16(addr, data);
}
void thunk_iop_mem_write_32(u32 addr, u32 data)
{
    iopMemWrite32(addr,data);
}

void thunk_iop_interpret(opcode_t op)
{

}

const thunk_funcs_t ee_thunk_funcs =
{
    thunk_ee_mem_read_8,
    thunk_ee_mem_read_16,
    thunk_ee_mem_read_32,
    thunk_ee_mem_read_64,
    thunk_ee_mem_read_128,
    thunk_ee_mem_write_8,
    thunk_ee_mem_write_16,
    thunk_ee_mem_write_32,
    thunk_ee_mem_write_64,
    thunk_ee_mem_write_128,
    thunk_ee_interpret
};

const thunk_funcs_t iop_thunk_funcs =
{
    thunk_iop_mem_read_8,
    thunk_iop_mem_read_16,
    thunk_iop_mem_read_32,
    nullptr,
    nullptr,
    thunk_iop_mem_write_8,
    thunk_iop_mem_write_16,
    thunk_iop_mem_write_32,
    nullptr,
    nullptr,
    thunk_iop_interpret
};

