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
#include "Arm64Rec.h"
#include "Arm64Emitter.h"

using namespace Arm64Gen;

namespace R5900 {
namespace Dynarec {
namespace OpcodeImpl {

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
    u32 imm = op.uimm16();

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
void rec_store()
{
    const int data_size = sizeof(T) * 8;

    //TODO: some optimization can be done with immediates that are 12 bits or
    //fewer, but those can come later and will really only be useful when we
    //are doing reads and writes in the dynarec, rather than calling back into
    //the aarch64 thunk
    ARM64Reg base = aarch64_get_mapped_reg(op.base());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    u32 imm = op.uimm16();

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


    switch(data_size)
    {
        case 8:
            ADD(temp_reg, EMU_THUNK_REG,THUNK_FUNCS_OFFSET(write_8_func));
            break;
        case 16:
            ADD(temp_reg, EMU_THUNK_REG,THUNK_FUNCS_OFFSET(write_16_func));
            break;
        case 32:
            ADD(temp_reg, EMU_THUNK_REG,THUNK_FUNCS_OFFSET(write_32_func));
            break;
        case 64:
            ADD(temp_reg, EMU_THUNK_REG,THUNK_FUNCS_OFFSET(write_64_func));
            break;
        case 128:
            ADD(temp_reg, EMU_THUNK_REG,THUNK_FUNCS_OFFSET(write_128_func));
            break;
    }

    BLR(temp_reg);

    aarch64_load_all_mapped_regs();
}

void recLB(opcode_t op)
{
    ARM64Reg rt = rec_load<s8>(op);
    SXTB(rt,rt);
}

void recLBU()
{
    ARM64Reg rt = rec_load<u8>(op);
    UXTB(rt,rt); //todo: necessary?
}

void recLH()
{
    ARM64Reg rt = rec_load<s16>(op);
    SXTH(rt,rt);
}

void recLHU()
{
    ARM64Reg rt = rec_load<u16>(op);
    UXTH(rt,rt); // todo: necessary?
}

void recLW()
{
    ARM64Reg rt = rec_load<s32>(op);
    SXTW(rt,rt);
}

void recLWU()
{
    ARM64Reg rt = rec_load<u32>(op);
}

void recLD()
{
    ARM64Reg rt = rec_load<u64>(op);
}

void recLQ()
{

}

void recSB()
{
    rec_store<u8>();
}
void recSH()
{
    rec_store<u16>();

}
void recSW()
{
    rec_store<u32>();

}
void recSQ()
{

}
void recSD()
{
    rec_store<u64>();

}


void recLWL()
{
}

void recLWR()
{

}

void recSWL()
{
}

void recSWR()
{
}

void recLDL()
{
}

void recLDR()
{
}

void recSDL()
{
}

void recSDR()
{
}


void recLWC1()
{
}

void recSWC1()
{
}

void recLQC2()
{
}

void recSQC2()
{
}


} } }	// end namespace R5900::Dynarec::OpcodeImpl
