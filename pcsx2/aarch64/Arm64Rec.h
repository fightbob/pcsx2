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
#include "R5900.h"

#define MIPS_CPU_CTX_OFFSET(elem) (offsetof(cpuRegs,elem))

//TODO: move this somewhere better
struct opcode_t
{
	u32 opcode;
	u32 op() { return (opcode >> 26) & 0x3f; }
	u32 rd() { return (opcode >> 11) & 0x1f; }
	u32 rs() { return (opcode >> 21) & 0x1f; }
	u32 rt() { return (opcode >> 16) & 0x1f; }
	u32 func() { return opcode & 0x3f; }
	s16 simm16() { return (s16)opcode; }
	u16 uimm16() { return (u16)opcode; }
	s32 offset() { return ((s32)((s16)opcode)) << 2; }
	u32 sa() { return (opcode >> 6) & 0x1f; }
	u32 jmp(u32 pc) { return (pc & 0xf0000000) | ((opcode & 0x03ffffff) << 2); }
	u32 base() { return rs(); }
};

enum class mips_reg_e
{
    R0,
    R1,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7,
    R8,
    R9,
    R10,
    R11,
    R12,
    R13,
    R14,
    R15,
    R16,
    R17,
    R18,
    R19,
    R20,
    R21,
    R22,
    R23,
    R24,
    R25,
    R26,
    R27,
    R28,
    R29,
    R30,
    R31,
    LO,
    HI,
    INVALID
};

enum class reg_status_e
{
    UNUSED = 0x0, // the arm reg is not in use, available for allocation
    USED = 0x1,   // the arm reg is in use, but not mapped to a mips reg. usage should only last for the duration of the rec func
    MAPPED = 0x2, // the arm reg is mapped to a mips reg
};


constexpr bool aarch64_is_callee_saved_register(ARM64Reg reg);

int aarch64_get_num_temp_regs_in_use();

ARM64Reg aarch64_get_free_reg();

void aarch64_free_reg(ARM64Reg reg);

void aarch64_map_reg(ARM64Reg arm_reg, mips_reg_e mips_reg);

void aarch64_unmap_reg(ARM64Reg arm_reg, mips_reg_e mips_reg);

ARM64Reg aarch64_get_mapped_reg(mips_reg_e mips_reg);

ARM64Reg aarch64_get_and_map_reg(mips_reg_e mips_reg);

void aarch64_load_from_mips_ctx(mips_reg_e mips_reg, ARM64Reg arm_reg);

void aarch64_flush_to_mips_ctx(mips_reg_e mips_reg, ARM64Reg arm_reg);

void aarch64_flush_all_regs();