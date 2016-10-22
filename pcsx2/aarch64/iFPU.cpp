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
#include "Arm64NeonRec.h"

using namespace Arm64Gen;

namespace R5900 {
namespace Dynarec {
namespace OpcodeImpl {
namespace COP1 {


// FCR31 Flags
#define FPUflagC	0X00800000
#define FPUflagI	0X00020000
#define FPUflagD	0X00010000
#define FPUflagO	0X00008000
#define FPUflagU	0X00004000
#define FPUflagSI	0X00000040
#define FPUflagSD	0X00000020
#define FPUflagSO	0X00000010
#define FPUflagSU	0X00000008

//TODO: flags

void recCFC1(opcode_t op)
{

}

void recCTC1(opcode_t op)
{

}

void recMFC1(opcode_t op)
{

}

void recMTC1(opcode_t op)
{

}
void recABS_S(opcode_t op)
{
    ARM64Reg fs = neon_get_mapped_reg(op.fs());
    ARM64Reg fd = neon_get_mapped_reg(op.fd());

    FABS(EncodeRegToSingle(fd),fs);
}

void recADD_S(opcode_t op)
{
    ARM64Reg fs = neon_get_mapped_reg(op.fs());
    ARM64Reg fd = neon_get_mapped_reg(op.fd());
    ARM64Reg ft = neon_get_mapped_reg(op.ft());

    FADD(EncodeRegToSingle(fd),fs,ft);
}

void recADDA_S(opcode_t op)
{
    ARM64Reg fs = neon_get_mapped_reg(op.fs());
    ARM64Reg ft = neon_get_mapped_reg(op.ft());
    ARM64Reg acc = neon_get_mapped_reg(mips_reg_e::ACC);

    FADD(EncodeRegToSingle(acc),fs,ft);
}

void recBC1F(opcode_t op)
{

}

void recBC1T(opcode_t op)
{
}

void recBC1FL(opcode_t op)
{
}

void recBC1TL(opcode_t op)
{
}

void recC_EQ(opcode_t op)
{
    ARM64Reg fs = neon_get_mapped_reg(op.fs());
    ARM64Reg fd = neon_get_mapped_reg(op.fd());
    ARM64Reg fcr31 = aarch64_get_free_reg(mips_reg_e::FCR31);
    ARM64Reg temp_fcr31 = aarch64_get_free_reg();
    ARM64Reg temp_reg = aarch64_get_free_reg();

    MOV(temp_fcr31,fcr31);
    ORRI2R(temp_fcr31,temp_fcr31,1 << 23,temp_reg);
    ANDI2R(fcr31,fcr31,~(1 << 23),temp_reg);
    FCMP(fd,fs);
    CSEL(fcr31,temp_fcr31,fcr31,CC_EQ);

    aarch64_free_reg(temp_fcr31);
    aarch64_free_reg(temp_reg);
}

void recC_F(opcode_t op)
{
    ARM64Reg fcr31 = aarch64_get_free_reg(mips_reg_e::FCR31);
    ARM64Reg temp_reg = aarch64_get_free_reg();
    ANDI2R(fcr31,fcr31,~(1 << 23),temp_reg);
    aarch64_free_reg(temp_reg);
}

void recC_LE(opcode_t op)
{
    ARM64Reg fs = neon_get_mapped_reg(op.fs());
    ARM64Reg fd = neon_get_mapped_reg(op.fd());
    ARM64Reg fcr31 = aarch64_get_free_reg(mips_reg_e::FCR31);
    ARM64Reg temp_fcr31 = aarch64_get_free_reg();
    ARM64Reg temp_reg = aarch64_get_free_reg();

    MOV(temp_fcr31,fcr31);
    ORRI2R(temp_fcr31,temp_fcr31,1 << 23,temp_reg);
    ANDI2R(fcr31,fcr31,~(1 << 23),temp_reg);
    FCMP(fd,fs);
    CSEL(fcr31,temp_fcr31,fcr31,CC_LE);

    aarch64_free_reg(temp_fcr31);
    aarch64_free_reg(temp_reg);
}

void recC_LT(opcode_t op)
{
    ARM64Reg fs = neon_get_mapped_reg(op.fs());
    ARM64Reg fd = neon_get_mapped_reg(op.fd());
    ARM64Reg fcr31 = aarch64_get_free_reg(mips_reg_e::FCR31);
    ARM64Reg temp_fcr31 = aarch64_get_free_reg();
    ARM64Reg temp_reg = aarch64_get_free_reg();

    MOV(temp_fcr31,fcr31);
    ORRI2R(temp_fcr31,temp_fcr31,1 << 23,temp_reg);
    ANDI2R(fcr31,fcr31,~(1 << 23),temp_reg);
    FCMP(fd,fs);
    CSEL(fcr31,temp_fcr31,fcr31,CC_LT);

    aarch64_free_reg(temp_fcr31);
    aarch64_free_reg(temp_reg);
}
void recCVT_S(opcode_t op)
{
}

void recCVT_W(opcode_t op)
{
}

void recDIV_S(opcode_t op)
{
    ARM64Reg fs = neon_get_mapped_reg(op.fs());
    ARM64Reg fd = neon_get_mapped_reg(op.fd());
    ARM64Reg ft = neon_get_mapped_reg(op.ft());


}

void recMADD_S(opcode_t op)
{

}

void recMADDA_S(opcode_t op)
{
}

void recMAX_S(opcode_t op)
{
    ARM64Reg fs = neon_get_mapped_reg(op.fs());
    ARM64Reg fd = neon_get_mapped_reg(op.fd());
    ARM64Reg ft = neon_get_mapped_reg(op.ft());

    FMAX(fd,fs,ft);
}

void recMIN_S(opcode_t op)
{
    ARM64Reg fs = neon_get_mapped_reg(op.fs());
    ARM64Reg fd = neon_get_mapped_reg(op.fd());
    ARM64Reg ft = neon_get_mapped_reg(op.ft());

    FMIN(fd,fs,ft);
}

void recMOV_S(opcode_t op)
{
    ARM64Reg fd = neon_get_mapped_reg(op.fd());
    ARM64Reg fs = neon_get_mapped_reg(op.fs());

    FMOV(fd,fs);
}

void recMSUB_S(opcode_t op)
{
}

void recMSUBA_S(opcode_t op)
{
}

void recMUL_S(opcode_t op)
{
}


void recMULA_S(opcode_t op)
{
}

void recNEG_S(opcode_t op)
{
    ARM64Reg fs = neon_get_mapped_reg(op.fs());
    ARM64Reg fd = neon_get_mapped_reg(op.fd());

    FNEG(fd,fs);
}

void recSUB_S(opcode_t op)
{
    ARM64Reg fs = neon_get_mapped_reg(op.fs());
    ARM64Reg fd = neon_get_mapped_reg(op.fd());
    ARM64Reg ft = neon_get_mapped_reg(op.ft());

    FSUB(EncodeRegToSingle(fd),fs,ft);
}

void recSUBA_S(opcode_t op)
{
    ARM64Reg fs = neon_get_mapped_reg(op.fs());
    ARM64Reg ft = neon_get_mapped_reg(op.ft());
    ARM64Reg acc = neon_get_mapped_reg(mips_reg_e::ACC);

    FSUB(EncodeRegToSingle(acc),fs,ft);
}

void recSQRT_S(opcode_t op)
{
    ARM64Reg ft = neon_get_mapped_reg(op.ft());
    ARM64Reg fd = neon_get_mapped_reg(op.fd());

    FSQRT(EncodeRegToSingle(fd),fs);
}


void recRSQRT_S(opcode_t op)
{
    ARM64Reg ft = neon_get_mapped_reg(op.ft());
    ARM64Reg fd = neon_get_mapped_reg(op.fd());

    FRSQRTE(EncodeRegToSingle(fd),fs);
}

} } } }
