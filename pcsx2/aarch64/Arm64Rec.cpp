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


constexpr std::array<ARM64Reg,29> aarch64_reg_alloc_order = 
{
  W0,
  W1,
  W2,
  W3,
  W4,
  W5,
  W6,
  W7,
  W8,
  W9,
  W10,
  W11,
  W12,
  W13,
  W14,
  W15,
  W16,
  W17,
  W18,
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

constexpr std::array<ARM64Reg,10> aarch64_callee_saved_regs = 
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
    for (reg_status_e status : aarch64_current_reg_status)
    {
        if (status == reg_status_e::USED)
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
}

//TODO: we only need to know arm_reg. drop the mips_reg?
void aarch64_unmap_reg(ARM64Reg arm_reg, mips_reg_e mips_reg)
{
    assert(aarch64_current_reg_mapping[arm_reg] == mips_reg);
    assert(aarch64_current_reg_status[arm_reg] == reg_status_e::MAPPED);
    
    aarch64_current_reg_map[arm_reg] = mips_reg_E::INVALID;
    aarch64_current_reg_status[arm_reg] = reg_status_e::USED;
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
	
	return ARM64Reg::REG_INVALID;
}

ARM64Reg aarch64_get_and_map_reg(mips_reg_e mips_reg)
{
    ARM64Reg arm_reg = aarch64_get_mapped_reg(mips_reg);
    if (arm_reg == ARM64Reg::REG_INVALID)
    {
        // mips_reg isn't mapped to an arm reg
        arm_reg = aarch64_get_free_reg();
        aarch64_map_reg(arm_reg, mips_reg);
    }   
    return arm_reg;
}
    

