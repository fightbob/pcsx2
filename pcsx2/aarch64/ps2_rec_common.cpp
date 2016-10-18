//
// Created by forrest on 10/13/16.
//

#include "ps2_rec_common.h"

// this file contains all recompiler code that is common to > 1 dynarec and actually
// emulates some functionality

// ex: templated functions to emulate MIPS instructions

template <typename T>
void rec_load(opcode_t op)
{
    const int data_size = sizeof(T);

    //TODO: some optimization can be done with immediates that are 12 bits or
    //fewer, but those can come later and will really only be useful when we
    //are doing reads and writes in the dynarec, rather than calling back into
    //the aarch64 thunk
    ARM64Reg base = aarch64_get_mapped_reg(op.base());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    s32 imm = op.simm16();

    aarch64_flush_caller_saved_regs();

    // at this point, all of the contexts have been flushed back
    // so write and read to/from X0 - X7 with impunity in order
    // to pass parameters

    MOVZ(ARM64Reg::W0, imm);
    SXTH(ARM64Reg::W0, ARM64Reg::W0);
    ADD(ARM64Reg::W0, base, ARM64Reg::W0);

    ARM64Reg temp_reg = aarch64_get_free_reg();

    // This is a case we actually need to handle... not sure how though
    // so until then just assert false and handle it when we run into it
    assert(temp_reg != ARM64Reg::W0);

    switch(data_size)
    {
        case 8:
            ADD(temp_reg, EMU_THUNK_REG,THUNK_FUNCS_OFFSET(read_8_func));
            break;
        case 16:
            ADD(temp_reg, EMU_THUNK_REG,THUNK_FUNCS_OFFSET(read_16_func));
            break;
        case 32:
            ADD(temp_reg, EMU_THUNK_REG,THUNK_FUNCS_OFFSET(read_32_func));
            break;
        case 64:
            ADD(temp_reg, EMU_THUNK_REG,THUNK_FUNCS_OFFSET(read_64_func));
            break;
        case 128:
            ADD(temp_reg, EMU_THUNK_REG,THUNK_FUNCS_OFFSET(read_128_func));
            break;
    }

    BLR(temp_reg);

    //at this point, W0 has the value we want
    // I don't have a load all function that will skip a certain register
    // i.e. whatever register rt is... so just move it to the temp register
    // then move from that

    MOV(temp_reg, ARM64Reg::W0);

    aarch64_load_all_mapped_regs();

    MOV(rt, temp_reg);
    return rt;
}

template <typename T>
void rec_store(opcode_t op) {
    const int data_size = sizeof(T) * 8;

    //TODO: some optimization can be done with immediates that are 12 bits or
    //fewer, but those can come later and will really only be useful when we
    //are doing reads and writes in the dynarec, rather than calling back into
    //the aarch64 thunk
    ARM64Reg base = aarch64_get_mapped_reg(op.base());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    s32 imm = op.simm16();

    aarch64_flush_caller_saved_regs();

    // at this point, all of the contexts have been flushed back
    // so write and read to/from X0 - X7 with impunity in order
    // to pass parameters

    MOVZ(ARM64Reg::W0, imm);
    SXTH(ARM64Reg::W0, ARM64Reg::W0);
    ADD(ARM64Reg::W0, base, ARM64Reg::W0);
    //TODO: resize rt? we may not have to. depends on how thunk funcs choose to write memory
    MOV(ARM64Reg::W1, rt);

    ARM64Reg temp_reg = aarch64_get_free_reg();

    // This is a case we actually need to handle... not sure how though
    // so until then just assert false and handle it when we run into it
    assert(temp_reg != ARM64Reg::W0);
    assert(temp_reg != ARM64Reg::W1);


    switch (data_size) {
        case 8:
            ADD(temp_reg, EMU_THUNK_REG, THUNK_FUNCS_OFFSET(write_8_func));
            break;
        case 16:
            ADD(temp_reg, EMU_THUNK_REG, THUNK_FUNCS_OFFSET(write_16_func));
            break;
        case 32:
            ADD(temp_reg, EMU_THUNK_REG, THUNK_FUNCS_OFFSET(write_32_func));
            break;
        case 64:
            ADD(temp_reg, EMU_THUNK_REG, THUNK_FUNCS_OFFSET(write_64_func));
            break;
        case 128:
            ADD(temp_reg, EMU_THUNK_REG, THUNK_FUNCS_OFFSET(write_128_func));
            break;
    }

    BLR(temp_reg);

    aarch64_load_all_mapped_regs();
}

template <typename T, bool link, bool likely>
FixupBranch rec_branch(ARM64Reg rs, T other, CCFlags cond)
{
    FixupBranch not_taken;
    CMP(rs, other);
    if (link)
    {
        //TODO:
    }

    if (likely)
    {
        // if the condition is not taken we don't want to execute the delay slot
        // so we will ony take this branch (to skip the branch and delay slot code)
        // if !cond
        not_taken = B(!cond);
    }

    // TODO: we can optimize this case some probably

    // we have to recompile the instruction in the delay slot, but can't guarantee
    // that it won't modify both the CC flags AND the rs and rt registers that
    // are used by this function. Therefore, we can't trust the CCs, regardless of
    // CMPing then doing delay slot or vice versa.
    // INSTEAD, grab a temp register, set it to 1 if we should take the branch,
    // 0 otherwise, do delay slot, and then branch based on the temp reg
    ARM64Reg temp_reg = aarch64_get_free_reg();
    CSET(temp_reg, cond);

    //TODO(?): I'm kind of violating my rule about no temp registers across recompiler
    // function calls but since I'm not actually return to the dispatcher I don't
    // think it's an issue (we won't check in this function anyway)

    cpuRegs.IsDelaySlot = true;
    aarch64_recompile_next_instr();
    cpuRegs.IsDelaySlot = false;

    CMP(temp_reg,1);
    //TODO: wtf do I do with this fixup branch?
    FixupBranch b = B(CC_EQ);

    if(likely)
    {
        SetJumpTarget(not_taken);
    }

    aarch64_free_reg(temp_reg);
    return b;
}