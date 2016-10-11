//
// Created by forrest on 10/8/16.
//

#include "Arm64NeonRec.h"

#include "Arm64Emitter.h"



#include "PrecompiledHeader.h"
#include "Arm64Rec.h"
#include "R5900.h"
#include "Memory.h"

using namespace Arm64Gen;



const std::deque<ARM64Reg> neon_reg_alloc_lru =
{
S8,
S9,
S10,
S11,
S12,
S13,
S14,
S15,
S0, S1, S2, S3, S4, S5, S6,
S16, S17, S18, S19, S20, S21, S22, S23, S24, S25, S26, S27, S28, S29, S30, S31
};

const std::array<ARM64Reg,10> neon_callee_saved_regs =
{
S8,S9,S10,S11,S12,S13,S14,S15
};

std::map<Arm64Reg, mips_reg_e> neon_current_reg_mapping;
std::map<Arm64Reg, reg_status_e> neon_current_reg_status;

constexpr bool neon_is_callee_saved_register(ARM64Reg reg)
{
    return std::find(std::begin(neon_callee_saved_regs), std::end(neon_callee_saved_regs), reg) != std::end(neon_callee_saved_regs);
}

// rec functions are expected to free all temp registers before returning
// so check if they actually did by checking the return value of this
int neon_get_num_temp_regs_in_use()
{
    int in_use = 0;
    for (const auto& status : neon_current_reg_status)
    {
        if (status.second == reg_status_e::USED)
        {
            in_use++;
        }
    }
    return in_use;
}

ARM64Reg neon_get_free_reg()
{
    ARM64Reg arm_reg = neon_reg_alloc_lru.pop_front();

    switch(neon_current_reg_status[reg])
    {
        case reg_status_e::UNUSED:
            neon_current_reg_status[arm_reg] = reg_status_e::USED;
            break;
        case reg_status_e::USED:
            // TODO: what to do? Case is very unlikley
            break;
        case reg_status_e::MAPPED:
            neon_unmap_reg(arm_reg, neon_current_reg_mapping[arm_reg]);
            neon_current_reg_status[arm_reg] = reg_status_e::USED;
    }
    neon_reg_alloc_lru.push_back(arm_reg);
    return arm_reg;
}

void neon_free_reg(ARM64Reg reg)
{
    assert(neon_current_reg_status[arm_reg] == reg_status_e::USED);

    neon_current_reg_status[arm_reg] = reg_status_e::UNUSED;
}

void neon_map_reg(ARM64Reg arm_reg, mips_reg_e mips_reg)
{
    assert(neon_current_reg_mapping[arm_reg] == mips_reg_e::INVALID);
    assert(neon_current_reg_status[arm_reg] == reg_status_e::USED);

    //TODO: handle both 32 bit and 64 bit arm regs?
    neon_current_reg_mapping[arm_reg] = mips_reg;
    neon_current_reg_status[arm_reg] = reg_status_e::MAPPED;

    neon_load_from_mips_ctx(mips_reg, arm_reg);
}

//TODO: we only need to know arm_reg. drop the mips_reg?
void neon_unmap_reg(ARM64Reg arm_reg, mips_reg_e mips_reg)
{
    assert(neon_current_reg_mapping[arm_reg] == mips_reg);
    assert(neon_current_reg_status[arm_reg] == reg_status_e::MAPPED);

    neon_current_reg_map[arm_reg] = mips_reg_E::INVALID;
    neon_current_reg_status[arm_reg] = reg_status_e::USED;

    neon_flush_to_mips_ctx(mips_reg, arm_reg);
}

ARM64Reg neon_get_mapped_reg(mips_reg_e mips_reg)
{
    assert(mips_reg != mips_reg_e::INVALID);

    for (ARM64Reg neon_reg : neon_reg_alloc_order)
    {
        if (neon_current_reg_mapping[neon_reg] == mips_reg)
        {
            return neon_reg;
        }
    }
    //mips_reg isn't mapped, so map it here
    arm_reg = neon_get_free_reg();
    neon_map_reg(arm_reg, mips_reg);

    return arm_reg;
}

//TODO: full 128bit load
void neon_load_from_mips_ctx(mips_reg_e mips_reg, ARM64Reg arm_reg)
{
    LDR(INDEX_UNSIGNED, arm_reg, MIPS_FPU_CTX_REG, MIPS_FPU_CTX_OFFSET(GPR.r[mips_reg]));
}

//TODO: full 128bit store
void neon_flush_to_mips_ctx(mips_reg_e mips_reg, ARM64Reg arm_reg)
{
    STR(INDEX_UNSIGNED, arm_reg, MIPS_FPU_CTX_REG, MIPS_FPU_CTX_OFFSEt(GPR.r[mips_reg]));
}

void neon_flush_all_regs()
{
    for (const auto& mapping : neon_current_reg_mapping)
    {
        if (mapping.second != mips_reg_e::INVALID)
        {
            neon_flush_to_mips_ctx(mapping.second, mapping.first);
        }
    }
}

void neon_flush_and_unmap_all_regs()
{
    for (const auto& mapping : neon_current_reg_mapping)
    {
        if (mapping.second != mips_reg_e::INVALID)
        {
            neon_flush_to_mips_ctx(mapping.second, mapping.first);
            neon_unmap_reg(mapping.first, mapping.second);
        }
    }
}

void neon_load_all_mapped_regs()
{
    for (const auto& mapping : neon_current_reg_mapping)
    {
        if (mapping.second != mips_reg_e::INVALID)
        {
            neon_load_from_mips_ctx(mapping.second, mapping.first);
        }
    }
}

void neon_flush_caller_saved_regs()
{
    for (const auto& mapping : neon_current_reg_mapping)
    {
        if (mapping.second != mips_reg_e::INVALID && !neon_is_callee_saved_register(mapping.first))
        {
            neon_flush_to_mips_ctx(mapping.second, mapping.first);
        }
    }
}

