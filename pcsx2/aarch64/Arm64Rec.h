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

// aarch64 has a lot of registers, but no good way to load from some arbitrary address
// for flushing and loading registers, we need to work with the mips cpu context
// so just store a pointer to it in some given register, then LDR/STR to/from it whenever
// we need something. We have 29 registers to allocate... I think we can give one or two up
// for such usage
#define MIPS_CPU_CTX_REG W28
#define MIPS_FPU_CTX_REG W27
#define EMU_THUNK_REG W26

//we need everything in cpu and fpu regs to be within a offset of the beginning so that
// aarch64 instructions can reach them. That shouldn't be an issue
static_assert(sizeof(cpuRegs) < 16380, "cpuRegs is too big!");
static_assert(sizeof(fpuRegs) < 16380, "fpuRegs is too big!");

#define MIPS_CPU_CTX_OFFSET(elem) (offsetof(cpuRegs,elem))

constexpr int IMM12_MAX = ((1 << 12) - 1);

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

struct thunk_funcs_t
{
    u8(*read_8_func)(u32);
    u16(*read_16_func)(u32);
    u32(*read_32_func)(u32);
    u64(*read_64_func)(u32);
    u128(*read_128_func)(u32);
    void(*read_8_func)(u32,u8);
    void(*read_16_func)(u32,u16);
    void(*read_32_func)(u32,u32);
    void(*read_64_func)(u32,u64);
    void(*read_128_func)(u32,u128);
    void(*interpret_func)(opcode_t);
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

enum class thunk_op_e
{
    READ_8,
    READ_16,
    READ_32,
    READ_64,
    READ_128,
    WRITE_8,
    WRITE_16,
    WRITE_32,
    WRITE_64,
    WRITE_128,
    INTERPRET
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

void aarch64_flush_and_unmap_all_regs();

void aarch64_load_all_mapped_regs();

void aarch64_flush_callee_saved_regs();

void aarch64_thunk();

u8 thunk_mem_read_8(u32 addr);
u16 thunk_mem_read_16(u32 addr);
u32 thunk_mem_read_32(u32 addr);
u64 thunk_mem_read_64(u32 addr);
u128 thunk_mem_read_128(u32 addr);
void thunk_mem_write_8(u32 addr, u8 data);
void thunk_mem_write_16(u32 addr, u16 data);
void thunk_mem_write_32(u32 addr, u32 data);
void thunk_mem_write_64(u32 addr, u64 data);
void thunk_mem_write_128(u32 addr, u128 data);
void thunk_interpret(opcode_t op);