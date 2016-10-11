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


/*********************************************************
*   cached MMI opcodes                                   *
*                                                        *
*********************************************************/

#include "PrecompiledHeader.h"

#include "Common.h"
#include "Arm64Rec.h"
#include "Arm64NeonRec.h"
#include "Arm64Emitter.h"

//TODO: we need some interop between the aarch64* and neon* to keep track of the liveness of the emulated mips reg
// i.e. we need to keep track if the most recent usage of a register is in a neon reg or GPR reg and flush (or dont)
// accordingly. Most of that will live in the neon* / aarch64* functions. These are pretty agnostic to that and
// should only care about the return value of *_get_mapped_reg

using namespace Arm64Gen;

namespace R5900 {
namespace Dynarec {
namespace OpcodeImpl {
namespace MMI
{


void recPLZCW(opcode_t op)
{

}

void recPMFHL(opcode_t op)
{
}

void recPMTHL(opcode_t op)
{
}

void recPSRLH(opcode_t op)
{
}

void recPSRLW(opcode_t op)
{
}

void recPSRAH(opcode_t op)
{
}

void recPSRAW(opcode_t op)
{
}

void recPSLLH(opcode_t op)
{
}

void recPSLLW(opcode_t op)
{
}

void recPMAXW(opcode_t op)
{
}

void recPPACW(opcode_t op)
{
}

void recPPACH(opcode_t op)
{
}

void recPPACB(opcode_t op)
{
}

void recPEXT5(opcode_t op)
{
}

void recPPAC5(opcode_t op)
{
}

void recPMAXH(opcode_t op)
{
}

void recPCGTB(opcode_t op)
{
}

void recPCGTH(opcode_t op)
{
}

void recPCGTW(opcode_t op)
{
}

void recPADDSB(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    SQADD(rd,rs,rt,simd_size_e::bytes_16);
}

void recPADDSH(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    SQADD(rd,rs,rt,simd_size_e::halves_8);
}

void recPADDSW(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    SQADD(rd,rs,rt,simd_size_e::words_4);
}

void recPSUBSB(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    SQSUB(rd,rs,rt,simd_size_e::bytes_16);
}

void recPSUBSH(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    SQSUB(rd,rs,rt,simd_size_e::halves_8);
}

void recPSUBSW(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    SQSUB(rd,rs,rt,simd_size_e::words_4);
}

void recPADDB(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    ADDP(rd,rs,rt,simd_size_e::bytes_16);
}

void recPADDH(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    ADDP(rd,rs,rt,simd_size_e::halves_8);
}

void recPADDW(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    ADDP(rd,rs,rt,simd_size_e::words_4);
}

void recPSUBB(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    SUBP(rd,rs,rt,simd_size_e::bytes_16);
}

void recPSUBH(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    SUBP(rd,rs,rt,simd_size_e::halves_8);
}

void recPSUBW(opcode_t op)
{

    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    SUBP(rd,rs,rt,simd_size_e::words_4);
}

void recPEXTLW(opcode_t op)
{
}

void recPEXTLB(opcode_t op)
{
}

void recPEXTLH(opcode_t op)
{
}


void recPABSW(opcode_t op) //needs clamping
{
}


void recPABSH(opcode_t op)
{
}

void recPMINW(opcode_t op)
{
}

void recPADSBH(opcode_t op)
{
}

void recPADDUW(opcode_t op)
{
}

void recPSUBUB(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    UQSUB(rd,rs,rt,simd_size_e::bytes_16);
}

void recPSUBUH(opcode_t op)
{

    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    UQSUB(rd,rs,rt,simd_size_e::words_8);
}

void recPSUBUW(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    UQSUB(rd,rs,rt,simd_size_e::words_4);
}

void recPEXTUH(opcode_t op)
{
}

void recQFSRV(opcode_t op)
{
}


void recPEXTUB(opcode_t op)
{
}

void recPEXTUW(opcode_t op)
{
}

void recPMINH(opcode_t op)
{
}

void recPCEQB(opcode_t op)
{
}

void recPCEQH(opcode_t op)
{
}

void recPCEQW(opcode_t op)
{
}

void recPADDUB(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    UQADD(rd,rs,rt,simd_size_e::bytes_16);
}

void recPADDUH(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    UQADD(rd,rs,rt,simd_size_e::halves_8);
}

void recPMADDW(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    UQADD(rd,rs,rt,simd_size_e::words_4);
}

void recPSLLVW(opcode_t op)
{
}

void recPSRLVW(opcode_t op)
{
}

void recPMSUBW(opcode_t op)
{
}

void recPMULTW(opcode_t op)
{
}
void recPDIVW(opcode_t op)
{
}

void recPDIVBW(opcode_t op)
{
}


//upper word of each doubleword in LO and HI is undocumented/undefined
//contains the upper multiplication result (before the addition with the lower multiplication result)
void recPHMADH(opcode_t op)
{
}

void recPMSUBH(opcode_t op)
{
}

//upper word of each doubleword in LO and HI is undocumented/undefined
//it contains the NOT of the upper multiplication result (before the substraction of the lower multiplication result)
void recPHMSBH(opcode_t op)
{
}

void recPEXEH(opcode_t op)
{
}

void recPREVH(opcode_t op)
{
}

void recPINTH(opcode_t op)
{
}

void recPEXEW(opcode_t op)
{
}

void recPROT3W(opcode_t op)
{
}

void recPMULTH(opcode_t op)
{
}

void recPMFHI(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(mips_reg_e::HI);

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg hi = neon_get_mapped_reg(mips_reg_e::HI);

    MOV(rd,hi,simd_size_e::bytes_16);
}

void recPMFLO(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(mips_reg_e::LO);

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg lo = neon_get_mapped_reg(mips_reg_e::LO);

    MOV(rd,lo,simd_size_e::bytes_16);
}

void recPAND(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    AND(rd,rs,rt,simd_size_e::bytes_16);
}

void recPXOR(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    EOR(rd,rs,rt,simd_size_e::bytes_16);
}

void recPCPYLD(opcode_t op)
{
}

void recPMADDH(opcode_t op)
{
}

void recPSRAVW(opcode_t op)
{
}



void recPINTEH(opcode_t op)
{
}

void recPMULTUW(opcode_t op)
{
}

void recPMADDUW(opcode_t op)
{
}

void recPDIVUW(opcode_t op)
{
}

void recPEXCW(opcode_t op)
{
}

void recPEXCH(opcode_t op)
{
}

void recPNOR(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    ORN(rd,rs,rt,simd_size_e::bytes_16);
}

void recPMTHI(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(mips_reg_e::HI);

    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg hi = neon_get_mapped_reg(mips_reg_e::HI);

    MOV(hi,rs,simd_size_e::bytes_16);
}

void recPMTLO(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(mips_reg_e::LO);

    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg lo = neon_get_mapped_reg(mips_reg_e::LO);

    MOV(lo,rs,simd_size_e::bytes_16);
}

void recPCPYUD(opcode_t op)
{
}

void recPOR(opcode_t op)
{
    aarch64_flush_to_mips_ctx(op.rd());
    aarch64_flush_to_mips_ctx(op.rs());
    aarch64_flush_to_mips_ctx(op.rt());

    ARM64Reg rd = neon_get_mapped_reg(op.rd());
    ARM64Reg rs = neon_get_mapped_reg(op.rs());
    ARM64Reg rt = neon_get_mapped_reg(op.rt());

    ORR(rd,rs,rt,simd_size_e::bytes_16);
}

void recPCPYH(opcode_t op)
{
}


} } } }
