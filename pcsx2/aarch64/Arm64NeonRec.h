//
// Created by forrest on 10/8/16.
//

#ifndef PCSX2_ARM64NEONREC_H
#define PCSX2_ARM64NEONREC_H


constexpr bool neon_is_callee_saved_register(ARM64Reg reg);

int neon_get_num_temp_regs_in_use();

ARM64Reg neon_get_free_reg();

void neon_free_reg(ARM64Reg reg);

void neon_map_reg(ARM64Reg arm_reg, mips_reg_e mips_reg);

void neon_unmap_reg(ARM64Reg arm_reg, mips_reg_e mips_reg);

ARM64Reg neon_get_mapped_reg(mips_reg_e mips_reg);

ARM64Reg neon_get_and_map_reg(mips_reg_e mips_reg);

void neon_load_from_mips_ctx(mips_reg_e mips_reg, ARM64Reg arm_reg);

void neon_flush_to_mips_ctx(mips_reg_e mips_reg, ARM64Reg arm_reg);

void neon_load_upper64_from_mips_ctx(mips_reg_e mips_reg, ARM64Reg arm_reg);

void neon_flush_upper64_to_mips_ctx(mips_reg_e mips_reg, ARM64Reg arm_reg);

void neon_flush_all_regs();

void neon_flush_and_unmap_all_regs();

void neon_load_all_mapped_regs();

void neon_flush_caller_saved_regs();

void neon_recompile_instr(opcode_t op);

void neon_recompile_instr_at_addr(u32 ps2_addr);

void neon_recompile_next_instr();

#endif //PCSX2_ARM64NEONREC_H
