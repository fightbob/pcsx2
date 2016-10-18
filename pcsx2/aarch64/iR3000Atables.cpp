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

void rpsxADDIU(opcode_t op)
{
    rpsxADDI(op);
}

void rpsxADDI(opcode_t op)
{
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    u32 imm = op.uimm16();

    if (imm <= IMM12_MAX)
    {
        ADD(EncodeRegTo32(rt),EncodeRegTo32(rs),imm);
    }
    else
    {
        ARM64Reg temp_reg = aarch64_get_free_reg();
        MOVZ(temp_reg,imm);
        ADD(EncodeRegTo32(rt),EncodeRegTo32(rs),temp_reg);
        aarch64_free_reg(temp_reg);
    }
}

void rpsxSLTI(opcode_t op)
{
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    u32 imm = op.uimm16();

    ARM64Reg temp_reg = aarch64_get_free_reg();
    MOVZ(EncodeRegTo32(temp_reg),imm);
    SXTH(EncodeRegTo32(temp_reg),EncodeRegTo32(temp_reg));
    CMP(EncodeRegTo32(rs),EncodeRegTo32(temp_reg));
    CSET(EncodeRegTo32(rt),CC_LT);
}

void rpsxSLTIU(opcode_t op)
{
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    u32 imm = op.uimm16();

    ARM64Reg temp_reg = aarch64_get_free_reg();
    MOVZ(EncodeRegTo32(temp_reg),imm);
    SXTH(EncodeRegTo32(temp_reg),EncodeRegTo32(temp_reg));
    CMP(EncodeRegTo32(rs),EncodeRegTo32(temp_reg));
    CSET(EncodeRegTo32(rt),CC_LO);
}

void rpsxANDI(opcode_t op)
{
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    s32 imm = op.simm16();

    if (imm <= IMM12_MAX)
    {
        AND(EncodeRegTo32(rt),EncodeRegTo32(rs),imm);
    }
    else
    {
        ARM64Reg temp_reg = aarch64_get_free_reg();
        MOVZ(temp_reg,imm);
        AND(EncodeRegTo32(rt),EncodeRegTo32(rs),temp_reg);
        aarch64_free_reg(temp_reg);
    }
}

void rpsxORI(opcode_t op)
{
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    u32 imm = op.uimm16();

    if (imm <= IMM12_MAX)
    {
        ORR(EncodeRegTo32(rt),EncodeRegTo32(rs),imm);
    }
    else
    {
        ARM64Reg temp_reg = aarch64_get_free_reg();
        MOVZ(temp_reg,imm);
        ORR(EncodeRegTo32(rt),EncodeRegTo32(rs),temp_reg);
        aarch64_free_reg(temp_reg);
    }
}

void rpsxXORI(opcode_t op)
{
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    u32 imm = op.uimm16();

    if (imm <= IMM12_MAX)
    {
        EOR(EncodeRegTo32(rt),EncodeRegTo32(rs),imm);
    }
    else
    {
        ARM64Reg temp_reg = aarch64_get_free_reg();
        MOVZ(temp_reg,imm);
        EOR(EncodeRegTo32(rt),EncodeRegTo32(rs),temp_reg);
        aarch64_free_reg(temp_reg);
    }
}

void rpsxLUI(opcode_t op)
{
    u32 imm = op.uimm16();
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    MOVZ(EncodeRegTo32(rt), imm, ShiftAmount::SHIFT_16);
}

void rpsxADDU(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    ADD(EncodeRegTo32(rd), EncodeRegTo32(rn), EncodeRegTo32(rm));
}

void rpsxADD(opcode_t op)
{
    rpsxADD(op);
}

void rpsxSUBU(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    SUB(EncodeRegTo32(rd),EncodeRegTo32(rn),EncodeRegTo32(rm));
}

void rpsxSUB(opcode_t op)
{
    rpsxSUB(op);
}

void rpsxAND(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    AND(EncodeRegTo32(rd), EncodeRegTo32(rn), EncodeRegTo32(rm));
}
void rpsxOR(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    ORR(EncodeRegTo32(rd), EncodeRegTo32(rn), EncodeRegTo32(rm));
}

void rpsxXOR(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    EOR(EncodeRegTo32(rd), EncodeRegTo32(rn), EncodeRegTo32(rm));
}

void rpsxNOR(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    ORN(EncodeRegTo32(rd), EncodeRegTo32(rn), EncodeRegTo32(rm));
}

void rpsxSLT(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    CMP(EncodeRegTo32(rn),EncodeRegTo32(rm));
    CSET(EncodeRegTo32(rd),CC_LT);
}

void rpsxSLTU(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rn = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rm = aarch64_get_mapped_reg(op.rt());

    CMP(EncodeRegTo32(rn),EncodeRegTo32(rm));
    CSET(EncodeRegTo32(rd),CC_LO);
}

void rpsxMULT(opcode_t op)
{

}
void rpsxMULTU(opcode_t op)
{

}

void rpsxDIV(opcode_t op)
{

}

void rpsxDIVU(opcode_t op)
{

}


void rpsxLB(opcode_t op)
{
    ARM64Reg rt = rec_load<s8>(op);
    SXTB(rt,rt);
}

void rpsxLBU(opcode_t op)
{
    ARM64Reg rt = rec_load<u8>(op);
    UXTB(rt,rt);
}

void rpsxLH(opcode_t op)
{
    ARM64Reg rt = rec_load<s16>(op);
    SXTH(rt,rt);
}

void rpsxLHU(opcode_t op)
{
    ARM64Reg rt = rec_load<u16>(op);
    UXTH(rt,rt);
}

void rpsxLW(opcode_t op)
{
    ARM64Reg rt = rec_load<u32>(op);
}

void rpsxSB(opcode_t op)
{
    rec_store<u8>(op);
}

void rpsxSH(opcode_t op)
{
    rec_store<u16>(op);
}

void rpsxSW(opcode_t op)
{
    rec_store<u32>(op);
}

void rpsxSLL(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    int sa = op.sa();

    LSL(EncodeRegTo32(rd),rt,sa);
}

void rpsxSRL(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    int sa = op.sa();

    LSR(EncodeRegTo32(rd),rt,sa);
}

void rpsxSRA(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());
    int sa = op.sa();

    ASR(EncodeRegTo32(rd),rt,sa);
}

void rpsxSLLV(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    LSLV(EncodeRegTo32(rd),rs,rt);
}

void rpsxSRLV(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    LSRV(EncodeRegTo32(rd),rs,rt);
}

void rpsxSRAV(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    ASRV(EncodeRegTo32(rd),rs,rt);
}

void rpsxMFHI(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg hi = aarch64_get_mapped_reg(mips_reg_e::HI);

    MOV(EncodeRegTo32(rd),hi);
}

void rpsxMTHI(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg hi = aarch64_get_mapped_reg(mips_reg_e::HI);

    MOV(EncodeRegTo32(rd),hi);
}

void rpsxMFLO(opcode_t op)
{
    ARM64Reg rd = aarch64_get_mapped_reg(op.rd());
    ARM64Reg lo = aarch64_get_mapped_reg(mips_reg_e::LO);

    MOV(EncodeRegTo32(rd),lo);
}

void rpsxMTLO(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg hi = aarch64_get_mapped_reg(mips_reg_e::HI);

    MOV(EncodeRegTo32(hi), EncodeRegTo32(rs));
}

void rpsxJ(opcode_t op)
{

}

void rpsxJAL(opcode_t op)
{

}

void rpsxJR(opcode_t op)
{

}

void rpsxJALR(opcode_t op)
{

}


void rpsxBEQ(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    FixupBranch b = rec_branch<ARM64Reg, false, false>(rs,rt,CC_EQ);
}

void rpsxBNE(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());
    ARM64Reg rt = aarch64_get_mapped_reg(op.rt());

    FixupBranch b = rec_branch<ARM64Reg, false, false>(rs,rt,CC_NEQ);
}

void rpsxBLTZ(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, false, false>(rs,0,CC_LT);
}

void rpsxBGEZ(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, false, false>(rs,0,CC_GE);
}

void rpsxBLTZAL(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, true, false>(rs,0,CC_LT);
}

void rpsxBGEZAL(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, true, false>(rs,0,CC_GE);
}

void rpsxBLEZ(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, false, false>(rs,0,CC_LE);
}

void rpsxBGTZ(opcode_t op)
{
    ARM64Reg rs = aarch64_get_mapped_reg(op.rs());

    FixupBranch b = rec_branch<u32, false, false>(rs,0,CC_GT);
}

void rpsxMFC0(opcode_t op)
{

}

void rpsxCFC0(opcode_t op)
{

}

void rpsxMTC0(opcode_t op)
{

}

void rpsxCTC0(opcode_t op)
{

}

void rpsxRFE(opcode_t op)
{

}



extern void (*rpsxBSC[64])(opcode_t op);
extern void (*rpsxSPC[64])(opcode_t op);
extern void (*rpsxREG[32])(opcode_t op);
extern void (*rpsxCP0[32])(opcode_t op);
extern void (*rpsxCP2[64])(opcode_t op);
extern void (*rpsxCP2BSC[32])(opcode_t op);

static void rpsxSPECIAL(opcode_t op) { rpsxSPC[_Funct_](); }
static void rpsxREGIMM(opcode_t op) { rpsxREG[_Rt_](); }
static void rpsxCOP0(opcode_t op) { rpsxCP0[_Rs_](); }
static void rpsxCOP2(opcode_t op) { rpsxCP2[_Funct_](); }
static void rpsxBASIC(opcode_t op) { rpsxCP2BSC[_Rs_](); }

static void rpsxNULL(opcode_t op) {
	Console.WriteLn("psxUNK: %8.8x", psxRegs.code);
}

void (*rpsxBSC[64])(opcode_t op) = {
	rpsxSPECIAL, rpsxREGIMM, rpsxJ   , rpsxJAL  , rpsxBEQ , rpsxBNE , rpsxBLEZ, rpsxBGTZ,
	rpsxADDI   , rpsxADDIU , rpsxSLTI, rpsxSLTIU, rpsxANDI, rpsxORI , rpsxXORI, rpsxLUI ,
	rpsxCOP0   , rpsxNULL  , rpsxCOP2, rpsxNULL , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxNULL   , rpsxNULL  , rpsxNULL, rpsxNULL , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxLB     , rpsxLH    , rpsxLWL , rpsxLW   , rpsxLBU , rpsxLHU , rpsxLWR , rpsxNULL,
	rpsxSB     , rpsxSH    , rpsxSWL , rpsxSW   , rpsxNULL, rpsxNULL, rpsxSWR , rpsxNULL,
	rpsxNULL   , rpsxNULL  , rgteLWC2, rpsxNULL , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxNULL   , rpsxNULL  , rgteSWC2, rpsxNULL , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL
};

void (*rpsxSPC[64])(opcode_t op) = {
	rpsxSLL , rpsxNULL, rpsxSRL , rpsxSRA , rpsxSLLV   , rpsxNULL , rpsxSRLV, rpsxSRAV,
	rpsxJR  , rpsxJALR, rpsxNULL, rpsxNULL, rpsxSYSCALL, rpsxBREAK, rpsxNULL, rpsxNULL,
	rpsxMFHI, rpsxMTHI, rpsxMFLO, rpsxMTLO, rpsxNULL   , rpsxNULL , rpsxNULL, rpsxNULL,
	rpsxMULT, rpsxMULTU, rpsxDIV, rpsxDIVU, rpsxNULL   , rpsxNULL , rpsxNULL, rpsxNULL,
	rpsxADD , rpsxADDU, rpsxSUB , rpsxSUBU, rpsxAND    , rpsxOR   , rpsxXOR , rpsxNOR ,
	rpsxNULL, rpsxNULL, rpsxSLT , rpsxSLTU, rpsxNULL   , rpsxNULL , rpsxNULL, rpsxNULL,
	rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL   , rpsxNULL , rpsxNULL, rpsxNULL,
	rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL   , rpsxNULL , rpsxNULL, rpsxNULL
};

void (*rpsxREG[32])(opcode_t op) = {
	rpsxBLTZ  , rpsxBGEZ  , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxNULL  , rpsxNULL  , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxBLTZAL, rpsxBGEZAL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxNULL  , rpsxNULL  , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL
};

void (*rpsxCP0[32])(opcode_t op) = {
	rpsxMFC0, rpsxNULL, rpsxCFC0, rpsxNULL, rpsxMTC0, rpsxNULL, rpsxCTC0, rpsxNULL,
	rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxRFE , rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL
};

void (*rpsxCP2[64])(opcode_t op) = {
	rpsxBASIC, rgteRTPS , rpsxNULL , rpsxNULL, rpsxNULL, rpsxNULL , rgteNCLIP, rpsxNULL, // 00
	rpsxNULL , rpsxNULL , rpsxNULL , rpsxNULL, rgteOP  , rpsxNULL , rpsxNULL , rpsxNULL, // 08
	rgteDPCS , rgteINTPL, rgteMVMVA, rgteNCDS, rgteCDP , rpsxNULL , rgteNCDT , rpsxNULL, // 10
	rpsxNULL , rpsxNULL , rpsxNULL , rgteNCCS, rgteCC  , rpsxNULL , rgteNCS  , rpsxNULL, // 18
	rgteNCT  , rpsxNULL , rpsxNULL , rpsxNULL, rpsxNULL, rpsxNULL , rpsxNULL , rpsxNULL, // 20
	rgteSQR  , rgteDCPL , rgteDPCT , rpsxNULL, rpsxNULL, rgteAVSZ3, rgteAVSZ4, rpsxNULL, // 28
	rgteRTPT , rpsxNULL , rpsxNULL , rpsxNULL, rpsxNULL, rpsxNULL , rpsxNULL , rpsxNULL, // 30
	rpsxNULL , rpsxNULL , rpsxNULL , rpsxNULL, rpsxNULL, rgteGPF  , rgteGPL  , rgteNCCT  // 38
};

void(*rpsxCP2BSC[32])(opcode_t op) = {
	rgteMFC2, rpsxNULL, rgteCFC2, rpsxNULL, rgteMTC2, rpsxNULL, rgteCTC2, rpsxNULL,
	rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL,
	rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL, rpsxNULL
};

