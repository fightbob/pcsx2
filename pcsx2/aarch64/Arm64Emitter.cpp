// Copyright 2015 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <algorithm>
#include <array>
#include <cstring>
#include <vector>

#include "PrecompiledHeader.h"
#include "Arm64Emitter.h"

namespace Arm64Gen
{

static void EncodeCompareBranchInst(u32 op, ARM64Reg Rt, const void* ptr);
static void EncodeTestBranchInst(u32 op, ARM64Reg Rt, u8 bits, const void* ptr);
static void EncodeUnconditionalBranchInst(u32 op, const void* ptr);
static void EncodeUnconditionalBranchInst(u32 opc, u32 op2, u32 op3, u32 op4, ARM64Reg Rn);
static void EncodeExceptionInst(u32 instenc, u32 imm);
static void EncodeSystemInst(u32 op0, u32 op1, u32 CRn, u32 CRm, u32 op2, ARM64Reg Rt);
static void EncodeArithmeticInst(u32 instenc, bool flags, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ArithOption Option);
static void EncodeArithmeticCarryInst(u32 op, bool flags, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm);
static void EncodeCondCompareImmInst(u32 op, ARM64Reg Rn, u32 imm, u32 nzcv, CCFlags cond);
static void EncodeCondCompareRegInst(u32 op, ARM64Reg Rn, ARM64Reg Rm, u32 nzcv, CCFlags cond);
static void EncodeCondSelectInst(u32 instenc, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, CCFlags cond);
static void EncodeData1SrcInst(u32 instenc, ARM64Reg Rd, ARM64Reg Rn);
static void EncodeData2SrcInst(u32 instenc, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm);
static void EncodeData3SrcInst(u32 instenc, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ARM64Reg Ra);
static void EncodeLogicalInst(u32 instenc, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ArithOption Shift);
static void EncodeLoadRegisterInst(u32 bitop, ARM64Reg Rt, u32 imm);
static void EncodeLoadStoreExcInst(u32 instenc, ARM64Reg Rs, ARM64Reg Rt2, ARM64Reg Rn, ARM64Reg Rt);
static void EncodeLoadStorePairedInst(u32 op, ARM64Reg Rt, ARM64Reg Rt2, ARM64Reg Rn, u32 imm);
static void EncodeLoadStoreIndexedInst(u32 op, u32 op2, ARM64Reg Rt, ARM64Reg Rn, s32 imm);
static void EncodeLoadStoreIndexedInst(u32 op, ARM64Reg Rt, ARM64Reg Rn, s32 imm, u8 size);
static void EncodeMOVWideInst(u32 op, ARM64Reg Rd, u32 imm, ShiftAmount pos);
static void EncodeBitfieldMOVInst(u32 op, ARM64Reg Rd, ARM64Reg Rn, u32 immr, u32 imms);
static void EncodeLoadStoreRegisterOffset(u32 size, u32 opc, ARM64Reg Rt, ARM64Reg Rn, ArithOption Rm);
static void EncodeAddSubImmInst(u32 op, bool flags, u32 shift, u32 imm, ARM64Reg Rn, ARM64Reg Rd);
static void EncodeLogicalImmInst(u32 op, ARM64Reg Rd, ARM64Reg Rn, u32 immr, u32 imms, int n);
static void EncodeLoadStorePair(u32 op, u32 load, IndexType type, ARM64Reg Rt, ARM64Reg Rt2, ARM64Reg Rn, s32 imm);
static void EncodeAddressInst(u32 op, ARM64Reg Rd, s32 imm);
static void EncodeLoadStoreUnscaled(u32 size, u32 op, ARM64Reg Rt, ARM64Reg Rn, s32 imm);


void EmitLoadStoreImmediate(u8 size, u32 opc, IndexType type, ARM64Reg Rt, ARM64Reg Rn, s32 imm);
void EmitScalar2Source(bool M, bool S, u32 type, u32 opcode, ARM64Reg Rd, ARM64Reg Rn,
                       ARM64Reg Rm);
void EmitThreeSame(bool U, u32 size, u32 opcode, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm);
void EmitCopy(bool Q, u32 op, u32 imm5, u32 imm4, ARM64Reg Rd, ARM64Reg Rn);
void Emit2RegMisc(bool Q, bool U, u32 size, u32 opcode, ARM64Reg Rd, ARM64Reg Rn);
void EmitLoadStoreSingleStructure(bool L, bool R, u32 opcode, bool S, u32 size, ARM64Reg Rt,
                                  ARM64Reg Rn);
void EmitLoadStoreSingleStructure(bool L, bool R, u32 opcode, bool S, u32 size, ARM64Reg Rt,
                                  ARM64Reg Rn, ARM64Reg Rm);
void Emit1Source(bool M, bool S, u32 type, u32 opcode, ARM64Reg Rd, ARM64Reg Rn);
void EmitConversion(bool sf, bool S, u32 type, u32 rmode, u32 opcode, ARM64Reg Rd, ARM64Reg Rn);
void EmitConversion2(bool sf, bool S, bool direction, u32 type, u32 rmode, u32 opcode, int scale,
                     ARM64Reg Rd, ARM64Reg Rn);
void EmitCompare(bool M, bool S, u32 op, u32 opcode2, ARM64Reg Rn, ARM64Reg Rm);
void EmitCondSelect(bool M, bool S, CCFlags cond, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm);
void EmitPermute(u32 size, u32 op, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm);
void EmitScalarImm(bool M, bool S, u32 type, u32 imm5, ARM64Reg Rd, u32 imm8);
void EmitShiftImm(bool Q, bool U, u32 immh, u32 immb, u32 opcode, ARM64Reg Rd, ARM64Reg Rn);
void EmitScalarShiftImm(bool U, u32 immh, u32 immb, u32 opcode, ARM64Reg Rd, ARM64Reg Rn);
void EmitLoadStoreMultipleStructure(u32 size, bool L, u32 opcode, ARM64Reg Rt, ARM64Reg Rn);
void EmitLoadStoreMultipleStructurePost(u32 size, bool L, u32 opcode, ARM64Reg Rt, ARM64Reg Rn,
                                        ARM64Reg Rm);
void EmitScalar1Source(bool M, bool S, u32 type, u32 opcode, ARM64Reg Rd, ARM64Reg Rn);
void EmitVectorxElement(bool U, u32 size, bool L, u32 opcode, bool H, ARM64Reg Rd, ARM64Reg Rn,
                        ARM64Reg Rm);
void EmitLoadStoreUnscaled(u32 size, u32 op, ARM64Reg Rt, ARM64Reg Rn, s32 imm);
void EmitConvertScalarToInt(ARM64Reg Rd, ARM64Reg Rn, RoundingMode round, bool sign);
void EmitScalar3Source(bool isDouble, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ARM64Reg Ra,
                       int opcode);
void EncodeLoadStorePair(u32 size, bool load, IndexType type, ARM64Reg Rt, ARM64Reg Rt2,
                         ARM64Reg Rn, s32 imm);
void EncodeLoadStoreRegisterOffset(u32 size, bool load, ARM64Reg Rt, ARM64Reg Rn, ArithOption Rm);
void EncodeModImm(bool Q, u8 op, u8 cmode, u8 o2, ARM64Reg Rd, u8 abcdefgh);

const int kWRegSizeInBits = 32;
const int kXRegSizeInBits = 64;

// The below few functions are taken from V8.
static int CountLeadingZeros(uint64_t value, int width)
{
  // TODO(jbramley): Optimize this for ARM64 hosts.
  int count = 0;
  uint64_t bit_test = 1ULL << (width - 1);
  while ((count < width) && ((bit_test & value) == 0))
  {
    count++;
    bit_test >>= 1;
  }
  return count;
}

static uint64_t LargestPowerOf2Divisor(uint64_t value)
{
  return value & -(int64_t)value;
}

static bool IsPowerOfTwo(uint64_t x)
{
  return (x != 0) && ((x & (x - 1)) == 0);
}

#define V8_UINT64_C(x) ((uint64_t)(x))

bool IsImmArithmetic(uint64_t input, u32* val, bool* shift)
{
  if (input < 4096)
  {
    *val = input;
    *shift = false;
    return true;
  }
  else if ((input & 0xFFF000) == input)
  {
    *val = input >> 12;
    *shift = true;
    return true;
  }
  return false;
}

bool IsImmLogical(uint64_t value, unsigned int width, unsigned int* n, unsigned int* imm_s,
                  unsigned int* imm_r)
{
  // DCHECK((n != NULL) && (imm_s != NULL) && (imm_r != NULL));
  // DCHECK((width == kWRegSizeInBits) || (width == kXRegSizeInBits));

  bool negate = false;

  // Logical immediates are encoded using parameters n, imm_s and imm_r using
  // the following table:
  //
  //    N   imms    immr    size        S             R
  //    1  ssssss  rrrrrr    64    UInt(ssssss)  UInt(rrrrrr)
  //    0  0sssss  xrrrrr    32    UInt(sssss)   UInt(rrrrr)
  //    0  10ssss  xxrrrr    16    UInt(ssss)    UInt(rrrr)
  //    0  110sss  xxxrrr     8    UInt(sss)     UInt(rrr)
  //    0  1110ss  xxxxrr     4    UInt(ss)      UInt(rr)
  //    0  11110s  xxxxxr     2    UInt(s)       UInt(r)
  // (s bits must not be all set)
  //
  // A pattern is constructed of size bits, where the least significant S+1 bits
  // are set. The pattern is rotated right by R, and repeated across a 32 or
  // 64-bit value, depending on destination register width.
  //
  // Put another way: the basic format of a logical immediate is a single
  // contiguous stretch of 1 bits, repeated across the whole word at intervals
  // given by a power of 2. To identify them quickly, we first locate the
  // lowest stretch of 1 bits, then the next 1 bit above that; that combination
  // is different for every logical immediate, so it gives us all the
  // information we need to identify the only logical immediate that our input
  // could be, and then we simply check if that's the value we actually have.
  //
  // (The rotation parameter does give the possibility of the stretch of 1 bits
  // going 'round the end' of the word. To deal with that, we observe that in
  // any situation where that happens the bitwise NOT of the value is also a
  // valid logical immediate. So we simply invert the input whenever its low bit
  // is set, and then we know that the rotated case can't arise.)

  if (value & 1)
  {
    // If the low bit is 1, negate the value, and set a flag to remember that we
    // did (so that we can adjust the return values appropriately).
    negate = true;
    value = ~value;
  }

  if (width == kWRegSizeInBits)
  {
    // To handle 32-bit logical immediates, the very easiest thing is to repeat
    // the input value twice to make a 64-bit word. The correct encoding of that
    // as a logical immediate will also be the correct encoding of the 32-bit
    // value.

    // The most-significant 32 bits may not be zero (ie. negate is true) so
    // shift the value left before duplicating it.
    value <<= kWRegSizeInBits;
    value |= value >> kWRegSizeInBits;
  }

  // The basic analysis idea: imagine our input word looks like this.
  //
  //    0011111000111110001111100011111000111110001111100011111000111110
  //                                                          c  b    a
  //                                                          |<--d-->|
  //
  // We find the lowest set bit (as an actual power-of-2 value, not its index)
  // and call it a. Then we add a to our original number, which wipes out the
  // bottommost stretch of set bits and replaces it with a 1 carried into the
  // next zero bit. Then we look for the new lowest set bit, which is in
  // position b, and subtract it, so now our number is just like the original
  // but with the lowest stretch of set bits completely gone. Now we find the
  // lowest set bit again, which is position c in the diagram above. Then we'll
  // measure the distance d between bit positions a and c (using CLZ), and that
  // tells us that the only valid logical immediate that could possibly be equal
  // to this number is the one in which a stretch of bits running from a to just
  // below b is replicated every d bits.
  uint64_t a = LargestPowerOf2Divisor(value);
  uint64_t value_plus_a = value + a;
  uint64_t b = LargestPowerOf2Divisor(value_plus_a);
  uint64_t value_plus_a_minus_b = value_plus_a - b;
  uint64_t c = LargestPowerOf2Divisor(value_plus_a_minus_b);

  int d, clz_a, out_n;
  uint64_t mask;

  if (c != 0)
  {
    // The general case, in which there is more than one stretch of set bits.
    // Compute the repeat distance d, and set up a bitmask covering the basic
    // unit of repetition (i.e. a word with the bottom d bits set). Also, in all
    // of these cases the N bit of the output will be zero.
    clz_a = CountLeadingZeros(a, kXRegSizeInBits);
    int clz_c = CountLeadingZeros(c, kXRegSizeInBits);
    d = clz_a - clz_c;
    mask = ((V8_UINT64_C(1) << d) - 1);
    out_n = 0;
  }
  else
  {
    // Handle degenerate cases.
    //
    // If any of those 'find lowest set bit' operations didn't find a set bit at
    // all, then the word will have been zero thereafter, so in particular the
    // last lowest_set_bit operation will have returned zero. So we can test for
    // all the special case conditions in one go by seeing if c is zero.
    if (a == 0)
    {
      // The input was zero (or all 1 bits, which will come to here too after we
      // inverted it at the start of the function), for which we just return
      // false.
      return false;
    }
    else
    {
      // Otherwise, if c was zero but a was not, then there's just one stretch
      // of set bits in our word, meaning that we have the trivial case of
      // d == 64 and only one 'repetition'. Set up all the same variables as in
      // the general case above, and set the N bit in the output.
      clz_a = CountLeadingZeros(a, kXRegSizeInBits);
      d = 64;
      mask = ~V8_UINT64_C(0);
      out_n = 1;
    }
  }

  // If the repeat period d is not a power of two, it can't be encoded.
  if (!IsPowerOfTwo(d))
    return false;

  // If the bit stretch (b - a) does not fit within the mask derived from the
  // repeat period, then fail.
  if (((b - a) & ~mask) != 0)
    return false;

  // The only possible option is b - a repeated every d bits. Now we're going to
  // actually construct the valid logical immediate derived from that
  // specification, and see if it equals our original input.
  //
  // To repeat a value every d bits, we multiply it by a number of the form
  // (1 + 2^d + 2^(2d) + ...), i.e. 0x0001000100010001 or similar. These can
  // be derived using a table lookup on CLZ(d).
  static const std::array<uint64_t, 6> multipliers = {{
      0x0000000000000001UL, 0x0000000100000001UL, 0x0001000100010001UL, 0x0101010101010101UL,
      0x1111111111111111UL, 0x5555555555555555UL,
  }};

  int multiplier_idx = CountLeadingZeros(d, kXRegSizeInBits) - 57;

  // Ensure that the index to the multipliers array is within bounds.
  _dbg_assert_(DYNA_REC,
               (multiplier_idx >= 0) && (static_cast<size_t>(multiplier_idx) < multipliers.size()));

  uint64_t multiplier = multipliers[multiplier_idx];
  uint64_t candidate = (b - a) * multiplier;

  // The candidate pattern doesn't match our input value, so fail.
  if (value != candidate)
    return false;

  // We have a match! This is a valid logical immediate, so now we have to
  // construct the bits and pieces of the instruction encoding that generates
  // it.

  // Count the set bits in our basic stretch. The special case of clz(0) == -1
  // makes the answer come out right for stretches that reach the very top of
  // the word (e.g. numbers like 0xffffc00000000000).
  int clz_b = (b == 0) ? -1 : CountLeadingZeros(b, kXRegSizeInBits);
  int s = clz_a - clz_b;

  // Decide how many bits to rotate right by, to put the low bit of that basic
  // stretch in position a.
  int r;
  if (negate)
  {
    // If we inverted the input right at the start of this function, here's
    // where we compensate: the number of set bits becomes the number of clear
    // bits, and the rotation count is based on position b rather than position
    // a (since b is the location of the 'lowest' 1 bit after inversion).
    s = d - s;
    r = (clz_b + 1) & (d - 1);
  }
  else
  {
    r = (clz_a + 1) & (d - 1);
  }

  // Now we're done, except for having to encode the S output in such a way that
  // it gives both the number of set bits and the length of the repeated
  // segment. The s field is encoded like this:
  //
  //     imms    size        S
  //    ssssss    64    UInt(ssssss)
  //    0sssss    32    UInt(sssss)
  //    10ssss    16    UInt(ssss)
  //    110sss     8    UInt(sss)
  //    1110ss     4    UInt(ss)
  //    11110s     2    UInt(s)
  //
  // So we 'or' (-d << 1) with our computed s to form imms.
  *n = out_n;
  *imm_s = ((-d << 1) | (s - 1)) & 0x3f;
  *imm_r = r;

  return true;
}

void SetCodePtrUnsafe(u8* ptr)
{
  m_code = ptr;
}

void SetCodePtr(u8* ptr)
{
  SetCodePtrUnsafe(ptr);
  m_lastCacheFlushEnd = ptr;
}

const u8* GetCodePtr() const
{
  return m_code;
}

u8* GetWritableCodePtr()
{
  return m_code;
}

void ReserveCodeSpace(u32 bytes)
{
  for (u32 i = 0; i < bytes / 4; i++)
    BRK(0);
}

const u8* AlignCode16()
{
  int c = int((u64)m_code & 15);
  if (c)
    ReserveCodeSpace(16 - c);
  return m_code;
}

const u8* AlignCodePage()
{
  int c = int((u64)m_code & 4095);
  if (c)
    ReserveCodeSpace(4096 - c);
  return m_code;
}

void Write32(u32 value)
{
  std::memcpy(m_code, &value, sizeof(u32));
  m_code += sizeof(u32);
}

void FlushIcache()
{
  FlushIcacheSection(m_lastCacheFlushEnd, m_code);
  m_lastCacheFlushEnd = m_code;
}

void FlushIcacheSection(u8* start, u8* end)
{
  if (start == end)
    return;

#if defined(IOS)
  // Header file says this is equivalent to: sys_icache_invalidate(start, end - start);
  sys_cache_control(kCacheFunctionPrepareForExecution, start, end - start);
#else
  // Don't rely on GCC's __clear_cache implementation, as it caches
  // icache/dcache cache line sizes, that can vary between cores on
  // big.LITTLE architectures.
  u64 addr, ctr_el0;
  static size_t icache_line_size = 0xffff, dcache_line_size = 0xffff;
  size_t isize, dsize;

  __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr_el0));
  isize = 4 << ((ctr_el0 >> 0) & 0xf);
  dsize = 4 << ((ctr_el0 >> 16) & 0xf);

  // use the global minimum cache line size
  icache_line_size = isize = icache_line_size < isize ? icache_line_size : isize;
  dcache_line_size = dsize = dcache_line_size < dsize ? dcache_line_size : dsize;

  addr = (u64)start & ~(u64)(dsize - 1);
  for (; addr < (u64)end; addr += dsize)
    // use "civac" instead of "cvau", as this is the suggested workaround for
    // Cortex-A53 errata 819472, 826319, 827319 and 824069.
    __asm__ volatile("dc civac, %0" : : "r"(addr) : "memory");
  __asm__ volatile("dsb ish" : : : "memory");

  addr = (u64)start & ~(u64)(isize - 1);
  for (; addr < (u64)end; addr += isize)
    __asm__ volatile("ic ivau, %0" : : "r"(addr) : "memory");

  __asm__ volatile("dsb ish" : : : "memory");
  __asm__ volatile("isb" : : : "memory");
#endif
}

// Exception generation
static const u32 ExcEnc[][3] = {
    {0, 0, 1},  // SVC
    {0, 0, 2},  // HVC
    {0, 0, 3},  // SMC
    {1, 0, 0},  // BRK
    {2, 0, 0},  // HLT
    {5, 0, 1},  // DCPS1
    {5, 0, 2},  // DCPS2
    {5, 0, 3},  // DCPS3
};

// Arithmetic generation
static const u32 ArithEnc[] = {
    0x058,  // ADD
    0x258,  // SUB
};

// Conditional Select
static const u32 CondSelectEnc[][2] = {
    {0, 0},  // CSEL
    {0, 1},  // CSINC
    {1, 0},  // CSINV
    {1, 1},  // CSNEG
};

// Data-Processing (1 source)
static const u32 Data1SrcEnc[][2] = {
    {0, 0},  // RBIT
    {0, 1},  // REV16
    {0, 2},  // REV32
    {0, 3},  // REV64
    {0, 4},  // CLZ
    {0, 5},  // CLS
};

// Data-Processing (2 source)
static const u32 Data2SrcEnc[] = {
    0x02,  // UDIV
    0x03,  // SDIV
    0x08,  // LSLV
    0x09,  // LSRV
    0x0A,  // ASRV
    0x0B,  // RORV
    0x10,  // CRC32B
    0x11,  // CRC32H
    0x12,  // CRC32W
    0x14,  // CRC32CB
    0x15,  // CRC32CH
    0x16,  // CRC32CW
    0x13,  // CRC32X (64bit Only)
    0x17,  // XRC32CX (64bit Only)
};

// Data-Processing (3 source)
static const u32 Data3SrcEnc[][2] = {
    {0, 0},  // MADD
    {0, 1},  // MSUB
    {1, 0},  // SMADDL (64Bit Only)
    {1, 1},  // SMSUBL (64Bit Only)
    {2, 0},  // SMULH (64Bit Only)
    {5, 0},  // UMADDL (64Bit Only)
    {5, 1},  // UMSUBL (64Bit Only)
    {6, 0},  // UMULH (64Bit Only)
};

// Logical (shifted register)
static const u32 LogicalEnc[][2] = {
    {0, 0},  // AND
    {0, 1},  // BIC
    {1, 0},  // OOR
    {1, 1},  // ORN
    {2, 0},  // EOR
    {2, 1},  // EON
    {3, 0},  // ANDS
    {3, 1},  // BICS
};

// Load/Store Exclusive
static const u32 LoadStoreExcEnc[][5] = {
    {0, 0, 0, 0, 0},  // STXRB
    {0, 0, 0, 0, 1},  // STLXRB
    {0, 0, 1, 0, 0},  // LDXRB
    {0, 0, 1, 0, 1},  // LDAXRB
    {0, 1, 0, 0, 1},  // STLRB
    {0, 1, 1, 0, 1},  // LDARB
    {1, 0, 0, 0, 0},  // STXRH
    {1, 0, 0, 0, 1},  // STLXRH
    {1, 0, 1, 0, 0},  // LDXRH
    {1, 0, 1, 0, 1},  // LDAXRH
    {1, 1, 0, 0, 1},  // STLRH
    {1, 1, 1, 0, 1},  // LDARH
    {2, 0, 0, 0, 0},  // STXR
    {3, 0, 0, 0, 0},  // (64bit) STXR
    {2, 0, 0, 0, 1},  // STLXR
    {3, 0, 0, 0, 1},  // (64bit) STLXR
    {2, 0, 0, 1, 0},  // STXP
    {3, 0, 0, 1, 0},  // (64bit) STXP
    {2, 0, 0, 1, 1},  // STLXP
    {3, 0, 0, 1, 1},  // (64bit) STLXP
    {2, 0, 1, 0, 0},  // LDXR
    {3, 0, 1, 0, 0},  // (64bit) LDXR
    {2, 0, 1, 0, 1},  // LDAXR
    {3, 0, 1, 0, 1},  // (64bit) LDAXR
    {2, 0, 1, 1, 0},  // LDXP
    {3, 0, 1, 1, 0},  // (64bit) LDXP
    {2, 0, 1, 1, 1},  // LDAXP
    {3, 0, 1, 1, 1},  // (64bit) LDAXP
    {2, 1, 0, 0, 1},  // STLR
    {3, 1, 0, 0, 1},  // (64bit) STLR
    {2, 1, 1, 0, 1},  // LDAR
    {3, 1, 1, 0, 1},  // (64bit) LDAR
};

static void EncodeCompareBranchInst(u32 op, ARM64Reg Rt, const void* ptr)
{
  bool b64Bit = Is64Bit(Rt);
  s64 distance = (s64)ptr - (s64)m_code;

  assert( !(distance & 0x3), "%s: distance must be a multiple of 4: %lx",
               __FUNCTION__, distance);

  distance >>= 2;

  assert(distance >= -0x40000 && distance <= 0x3FFFF,
               "%s: Received too large distance: %lx", __FUNCTION__, distance);

  Rt = DecodeReg(Rt);
  Write32((b64Bit << 31) | (0x34 << 24) | (op << 24) | (((u32)distance << 5) & 0xFFFFE0) | Rt);
}

static void EncodeTestBranchInst(u32 op, ARM64Reg Rt, u8 bits, const void* ptr)
{
  bool b64Bit = Is64Bit(Rt);
  s64 distance = (s64)ptr - (s64)m_code;

  assert(!(distance & 0x3), "%s: distance must be a multiple of 4: %lx",
               __FUNCTION__, distance);

  distance >>= 2;

  _assert_msg_(DYNA_REC, distance >= -0x3FFF && distance < 0x3FFF,
               "%s: Received too large distance: %lx", __FUNCTION__, distance);

  Rt = DecodeReg(Rt);
  Write32((b64Bit << 31) | (0x36 << 24) | (op << 24) | (bits << 19) |
          (((u32)distance << 5) & 0x7FFE0) | Rt);
}

static void EncodeUnconditionalBranchInst(u32 op, const void* ptr)
{
  s64 distance = (s64)ptr - s64(m_code);

  _assert_msg_(DYNA_REC, !(distance & 0x3), "%s: distance must be a multiple of 4: %lx",
               __FUNCTION__, distance);

  distance >>= 2;

  _assert_msg_(DYNA_REC, distance >= -0x2000000LL && distance <= 0x1FFFFFFLL,
               "%s: Received too large distance: %lx", __FUNCTION__, distance);

  Write32((op << 31) | (0x5 << 26) | (distance & 0x3FFFFFF));
}

static void EncodeUnconditionalBranchInst(u32 opc, u32 op2, u32 op3, u32 op4, ARM64Reg Rn)
{
  Rn = DecodeReg(Rn);
  Write32((0x6B << 25) | (opc << 21) | (op2 << 16) | (op3 << 10) | (Rn << 5) | op4);
}

static void EncodeExceptionInst(u32 instenc, u32 imm)
{
  _assert_msg_(DYNA_REC, !(imm & ~0xFFFF), "%s: Exception instruction too large immediate: %d",
               __FUNCTION__, imm);

  Write32((0xD4 << 24) | (ExcEnc[instenc][0] << 21) | (imm << 5) | (ExcEnc[instenc][1] << 2) |
          ExcEnc[instenc][2]);
}

static void EncodeSystemInst(u32 op0, u32 op1, u32 CRn, u32 CRm, u32 op2, ARM64Reg Rt)
{
  Write32((0x354 << 22) | (op0 << 19) | (op1 << 16) | (CRn << 12) | (CRm << 8) | (op2 << 5) | Rt);
}

static void EncodeArithmeticInst(u32 instenc, bool flags, ARM64Reg Rd, ARM64Reg Rn,
                                         ARM64Reg Rm, ArithOption Option)
{
  bool b64Bit = Is64Bit(Rd);

  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);
  Rm = DecodeReg(Rm);
  Write32((b64Bit << 31) | (flags << 29) | (ArithEnc[instenc] << 21) |
          (Option.GetType() == ArithOption::TYPE_EXTENDEDREG ? (1 << 21) : 0) | (Rm << 16) |
          Option.GetData() | (Rn << 5) | Rd);
}

static void EncodeArithmeticCarryInst(u32 op, bool flags, ARM64Reg Rd, ARM64Reg Rn,
                                              ARM64Reg Rm)
{
  bool b64Bit = Is64Bit(Rd);

  Rd = DecodeReg(Rd);
  Rm = DecodeReg(Rm);
  Rn = DecodeReg(Rn);
  Write32((b64Bit << 31) | (op << 30) | (flags << 29) | (0xD0 << 21) | (Rm << 16) | (Rn << 5) | Rd);
}

static void EncodeCondCompareImmInst(u32 op, ARM64Reg Rn, u32 imm, u32 nzcv, CCFlags cond)
{
  bool b64Bit = Is64Bit(Rn);

  _assert_msg_(DYNA_REC, !(imm & ~0x1F), "%s: too large immediate: %d", __FUNCTION__, imm)
      _assert_msg_(DYNA_REC, !(nzcv & ~0xF), "%s: Flags out of range: %d", __FUNCTION__, nzcv)

          Rn = DecodeReg(Rn);
  Write32((b64Bit << 31) | (op << 30) | (1 << 29) | (0xD2 << 21) | (imm << 16) | (cond << 12) |
          (1 << 11) | (Rn << 5) | nzcv);
}

static void EncodeCondCompareRegInst(u32 op, ARM64Reg Rn, ARM64Reg Rm, u32 nzcv,
                                             CCFlags cond)
{
  bool b64Bit = Is64Bit(Rm);

  _assert_msg_(DYNA_REC, !(nzcv & ~0xF), "%s: Flags out of range: %d", __FUNCTION__, nzcv)

      Rm = DecodeReg(Rm);
  Rn = DecodeReg(Rn);
  Write32((b64Bit << 31) | (op << 30) | (1 << 29) | (0xD2 << 21) | (Rm << 16) | (cond << 12) |
          (Rn << 5) | nzcv);
}

static void EncodeCondSelectInst(u32 instenc, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm,
                                         CCFlags cond)
{
  bool b64Bit = Is64Bit(Rd);

  Rd = DecodeReg(Rd);
  Rm = DecodeReg(Rm);
  Rn = DecodeReg(Rn);
  Write32((b64Bit << 31) | (CondSelectEnc[instenc][0] << 30) | (0xD4 << 21) | (Rm << 16) |
          (cond << 12) | (CondSelectEnc[instenc][1] << 10) | (Rn << 5) | Rd);
}

static void EncodeData1SrcInst(u32 instenc, ARM64Reg Rd, ARM64Reg Rn)
{
  bool b64Bit = Is64Bit(Rd);

  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);
  Write32((b64Bit << 31) | (0x2D6 << 21) | (Data1SrcEnc[instenc][0] << 16) |
          (Data1SrcEnc[instenc][1] << 10) | (Rn << 5) | Rd);
}

static void EncodeData2SrcInst(u32 instenc, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  bool b64Bit = Is64Bit(Rd);

  Rd = DecodeReg(Rd);
  Rm = DecodeReg(Rm);
  Rn = DecodeReg(Rn);
  Write32((b64Bit << 31) | (0x0D6 << 21) | (Rm << 16) | (Data2SrcEnc[instenc] << 10) | (Rn << 5) |
          Rd);
}

static void EncodeData3SrcInst(u32 instenc, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm,
                                       ARM64Reg Ra)
{
  bool b64Bit = Is64Bit(Rd);

  Rd = DecodeReg(Rd);
  Rm = DecodeReg(Rm);
  Rn = DecodeReg(Rn);
  Ra = DecodeReg(Ra);
  Write32((b64Bit << 31) | (0xD8 << 21) | (Data3SrcEnc[instenc][0] << 21) | (Rm << 16) |
          (Data3SrcEnc[instenc][1] << 15) | (Ra << 10) | (Rn << 5) | Rd);
}

static void EncodeLogicalInst(u32 instenc, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm,
                                      ArithOption Shift)
{
  bool b64Bit = Is64Bit(Rd);

  Rd = DecodeReg(Rd);
  Rm = DecodeReg(Rm);
  Rn = DecodeReg(Rn);
  Write32((b64Bit << 31) | (LogicalEnc[instenc][0] << 29) | (0x5 << 25) |
          (LogicalEnc[instenc][1] << 21) | Shift.GetData() | (Rm << 16) | (Rn << 5) | Rd);
}

static void EncodeLoadRegisterInst(u32 bitop, ARM64Reg Rt, u32 imm)
{
  bool b64Bit = Is64Bit(Rt);
  bool bVec = IsVector(Rt);

  _assert_msg_(DYNA_REC, !(imm & 0xFFFFF), "%s: offset too large %d", __FUNCTION__, imm);

  Rt = DecodeReg(Rt);
  if (b64Bit && bitop != 0x2)  // LDRSW(0x2) uses 64bit reg, doesn't have 64bit bit set
    bitop |= 0x1;
  Write32((bitop << 30) | (bVec << 26) | (0x18 << 24) | (imm << 5) | Rt);
}

static void EncodeLoadStoreExcInst(u32 instenc, ARM64Reg Rs, ARM64Reg Rt2, ARM64Reg Rn,
                                           ARM64Reg Rt)
{
  Rs = DecodeReg(Rs);
  Rt2 = DecodeReg(Rt2);
  Rn = DecodeReg(Rn);
  Rt = DecodeReg(Rt);
  Write32((LoadStoreExcEnc[instenc][0] << 30) | (0x8 << 24) | (LoadStoreExcEnc[instenc][1] << 23) |
          (LoadStoreExcEnc[instenc][2] << 22) | (LoadStoreExcEnc[instenc][3] << 21) | (Rs << 16) |
          (LoadStoreExcEnc[instenc][4] << 15) | (Rt2 << 10) | (Rn << 5) | Rt);
}

static void EncodeLoadStorePairedInst(u32 op, ARM64Reg Rt, ARM64Reg Rt2, ARM64Reg Rn,
                                              u32 imm)
{
  bool b64Bit = Is64Bit(Rt);
  bool b128Bit = IsQuad(Rt);
  bool bVec = IsVector(Rt);

  if (b128Bit)
    imm >>= 4;
  else if (b64Bit)
    imm >>= 3;
  else
    imm >>= 2;

  _assert_msg_(DYNA_REC, !(imm & ~0xF), "%s: offset too large %d", __FUNCTION__, imm);

  u32 opc = 0;
  if (b128Bit)
    opc = 2;
  else if (b64Bit && bVec)
    opc = 1;
  else if (b64Bit && !bVec)
    opc = 2;

  Rt = DecodeReg(Rt);
  Rt2 = DecodeReg(Rt2);
  Rn = DecodeReg(Rn);
  Write32((opc << 30) | (bVec << 26) | (op << 22) | (imm << 15) | (Rt2 << 10) | (Rn << 5) | Rt);
}

static void EncodeLoadStoreIndexedInst(u32 op, u32 op2, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  bool b64Bit = Is64Bit(Rt);
  bool bVec = IsVector(Rt);

  u32 offset = imm & 0x1FF;

  _assert_msg_(DYNA_REC, !(imm < -256 || imm > 255), "%s: offset too large %d", __FUNCTION__, imm);

  Rt = DecodeReg(Rt);
  Rn = DecodeReg(Rn);
  Write32((b64Bit << 30) | (op << 22) | (bVec << 26) | (offset << 12) | (op2 << 10) | (Rn << 5) |
          Rt);
}

static void EncodeLoadStoreIndexedInst(u32 op, ARM64Reg Rt, ARM64Reg Rn, s32 imm, u8 size)
{
  bool b64Bit = Is64Bit(Rt);
  bool bVec = IsVector(Rt);

  if (size == 64)
    imm >>= 3;
  else if (size == 32)
    imm >>= 2;
  else if (size == 16)
    imm >>= 1;

  _assert_msg_(DYNA_REC, imm >= 0, "%s(INDEX_UNSIGNED): offset must be positive %d", __FUNCTION__,
               imm);
  _assert_msg_(DYNA_REC, !(imm & ~0xFFF), "%s(INDEX_UNSIGNED): offset too large %d", __FUNCTION__,
               imm);

  Rt = DecodeReg(Rt);
  Rn = DecodeReg(Rn);
  Write32((b64Bit << 30) | (op << 22) | (bVec << 26) | (imm << 10) | (Rn << 5) | Rt);
}

static void EncodeMOVWideInst(u32 op, ARM64Reg Rd, u32 imm, ShiftAmount pos)
{
  bool b64Bit = Is64Bit(Rd);

  _assert_msg_(DYNA_REC, !(imm & ~0xFFFF), "%s: immediate out of range: %d", __FUNCTION__, imm);

  Rd = DecodeReg(Rd);
  Write32((b64Bit << 31) | (op << 29) | (0x25 << 23) | (pos << 21) | (imm << 5) | Rd);
}

static void EncodeBitfieldMOVInst(u32 op, ARM64Reg Rd, ARM64Reg Rn, u32 immr, u32 imms)
{
  bool b64Bit = Is64Bit(Rd);

  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);
  Write32((b64Bit << 31) | (op << 29) | (0x26 << 23) | (b64Bit << 22) | (immr << 16) |
          (imms << 10) | (Rn << 5) | Rd);
}

static void EncodeLoadStoreRegisterOffset(u32 size, u32 opc, ARM64Reg Rt, ARM64Reg Rn,
                                                  ArithOption Rm)
{
  Rt = DecodeReg(Rt);
  Rn = DecodeReg(Rn);
  ARM64Reg decoded_Rm = DecodeReg(Rm.GetReg());

  Write32((size << 30) | (opc << 22) | (0x1C1 << 21) | (decoded_Rm << 16) | Rm.GetData() |
          (1 << 11) | (Rn << 5) | Rt);
}

static void EncodeAddSubImmInst(u32 op, bool flags, u32 shift, u32 imm, ARM64Reg Rn,
                                        ARM64Reg Rd)
{
  bool b64Bit = Is64Bit(Rd);

  _assert_msg_(DYNA_REC, !(imm & ~0xFFF), "%s: immediate too large: %x", __FUNCTION__, imm);

  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);
  Write32((b64Bit << 31) | (op << 30) | (flags << 29) | (0x11 << 24) | (shift << 22) | (imm << 10) |
          (Rn << 5) | Rd);
}

static void EncodeLogicalImmInst(u32 op, ARM64Reg Rd, ARM64Reg Rn, u32 immr, u32 imms,
                                         int n)
{
  // Sometimes Rd is fixed to SP, but can still be 32bit or 64bit.
  // Use Rn to determine bitness here.
  bool b64Bit = Is64Bit(Rn);

  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);

  Write32((b64Bit << 31) | (op << 29) | (0x24 << 23) | (n << 22) | (immr << 16) | (imms << 10) |
          (Rn << 5) | Rd);
}

static void EncodeLoadStorePair(u32 op, u32 load, IndexType type, ARM64Reg Rt, ARM64Reg Rt2,
                                        ARM64Reg Rn, s32 imm)
{
  bool b64Bit = Is64Bit(Rt);
  u32 type_encode = 0;

  switch (type)
  {
  case INDEX_SIGNED:
    type_encode = 0b010;
    break;
  case INDEX_POST:
    type_encode = 0b001;
    break;
  case INDEX_PRE:
    type_encode = 0b011;
    break;
  case INDEX_UNSIGNED:
    _assert_msg_(DYNA_REC, false, "%s doesn't support INDEX_UNSIGNED!", __FUNCTION__);
    break;
  }

  if (b64Bit)
  {
    op |= 0b10;
    imm >>= 3;
  }
  else
  {
    imm >>= 2;
  }

  Rt = DecodeReg(Rt);
  Rt2 = DecodeReg(Rt2);
  Rn = DecodeReg(Rn);

  Write32((op << 30) | (0b101 << 27) | (type_encode << 23) | (load << 22) | ((imm & 0x7F) << 15) |
          (Rt2 << 10) | (Rn << 5) | Rt);
}
static void EncodeAddressInst(u32 op, ARM64Reg Rd, s32 imm)
{
  Rd = DecodeReg(Rd);

  Write32((op << 31) | ((imm & 0x3) << 29) | (0x10 << 24) | ((imm & 0x1FFFFC) << 3) | Rd);
}

static void EncodeLoadStoreUnscaled(u32 size, u32 op, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  _assert_msg_(DYNA_REC, !(imm < -256 || imm > 255), "%s received too large offset: %d",
               __FUNCTION__, imm);
  Rt = DecodeReg(Rt);
  Rn = DecodeReg(Rn);

  Write32((size << 30) | (0b111 << 27) | (op << 22) | ((imm & 0x1FF) << 12) | (Rn << 5) | Rt);
}

static constexpr bool IsInRangeImm19(s64 distance)
{
  return (distance >= -0x40000 && distance <= 0x3FFFF);
}

static constexpr bool IsInRangeImm14(s64 distance)
{
  return (distance >= -0x2000 && distance <= 0x1FFF);
}

static constexpr bool IsInRangeImm26(s64 distance)
{
  return (distance >= -0x2000000 && distance <= 0x1FFFFFF);
}

static constexpr u32 MaskImm19(s64 distance)
{
  return distance & 0x7FFFF;
}

static constexpr u32 MaskImm14(s64 distance)
{
  return distance & 0x3FFF;
}

static constexpr u32 MaskImm26(s64 distance)
{
  return distance & 0x3FFFFFF;
}

// FixupBranch branching
void SetJumpTarget(FixupBranch const& branch)
{
  bool Not = false;
  u32 inst = 0;
  s64 distance = (s64)(m_code - branch.ptr);
  distance >>= 2;

  switch (branch.type)
  {
  case 1:  // CBNZ
    Not = true;
  case 0:  // CBZ
  {
    _assert_msg_(DYNA_REC, IsInRangeImm19(distance), "%s(%d): Received too large distance: %lx",
                 __FUNCTION__, branch.type, distance);
    bool b64Bit = Is64Bit(branch.reg);
    ARM64Reg reg = DecodeReg(branch.reg);
    inst = (b64Bit << 31) | (0x1A << 25) | (Not << 24) | (MaskImm19(distance) << 5) | reg;
  }
  break;
  case 2:  // B (conditional)
    _assert_msg_(DYNA_REC, IsInRangeImm19(distance), "%s(%d): Received too large distance: %lx",
                 __FUNCTION__, branch.type, distance);
    inst = (0x2A << 25) | (MaskImm19(distance) << 5) | branch.cond;
    break;
  case 4:  // TBNZ
    Not = true;
  case 3:  // TBZ
  {
    _assert_msg_(DYNA_REC, IsInRangeImm14(distance), "%s(%d): Received too large distance: %lx",
                 __FUNCTION__, branch.type, distance);
    ARM64Reg reg = DecodeReg(branch.reg);
    inst = ((branch.bit & 0x20) << 26) | (0x1B << 25) | (Not << 24) | ((branch.bit & 0x1F) << 19) |
           (MaskImm14(distance) << 5) | reg;
  }
  break;
  case 5:  // B (uncoditional)
    _assert_msg_(DYNA_REC, IsInRangeImm26(distance), "%s(%d): Received too large distance: %lx",
                 __FUNCTION__, branch.type, distance);
    inst = (0x5 << 26) | MaskImm26(distance);
    break;
  case 6:  // BL (unconditional)
    _assert_msg_(DYNA_REC, IsInRangeImm26(distance), "%s(%d): Received too large distance: %lx",
                 __FUNCTION__, branch.type, distance);
    inst = (0x25 << 26) | MaskImm26(distance);
    break;
  }
  *(u32*)branch.ptr = inst;
}

FixupBranch CBZ(ARM64Reg Rt)
{
  FixupBranch branch;
  branch.ptr = m_code;
  branch.type = 0;
  branch.reg = Rt;
  HINT(HINT_NOP);
  return branch;
}
FixupBranch CBNZ(ARM64Reg Rt)
{
  FixupBranch branch;
  branch.ptr = m_code;
  branch.type = 1;
  branch.reg = Rt;
  HINT(HINT_NOP);
  return branch;
}
FixupBranch B(CCFlags cond)
{
  FixupBranch branch;
  branch.ptr = m_code;
  branch.type = 2;
  branch.cond = cond;
  HINT(HINT_NOP);
  return branch;
}
FixupBranch TBZ(ARM64Reg Rt, u8 bit)
{
  FixupBranch branch;
  branch.ptr = m_code;
  branch.type = 3;
  branch.reg = Rt;
  branch.bit = bit;
  HINT(HINT_NOP);
  return branch;
}
FixupBranch TBNZ(ARM64Reg Rt, u8 bit)
{
  FixupBranch branch;
  branch.ptr = m_code;
  branch.type = 4;
  branch.reg = Rt;
  branch.bit = bit;
  HINT(HINT_NOP);
  return branch;
}
FixupBranch B()
{
  FixupBranch branch;
  branch.ptr = m_code;
  branch.type = 5;
  HINT(HINT_NOP);
  return branch;
}
FixupBranch BL()
{
  FixupBranch branch;
  branch.ptr = m_code;
  branch.type = 6;
  HINT(HINT_NOP);
  return branch;
}

// Compare and Branch
void CBZ(ARM64Reg Rt, const void* ptr)
{
  EncodeCompareBranchInst(0, Rt, ptr);
}
void CBNZ(ARM64Reg Rt, const void* ptr)
{
  EncodeCompareBranchInst(1, Rt, ptr);
}

// Conditional Branch
void B(CCFlags cond, const void* ptr)
{
  s64 distance = (s64)ptr - (s64)m_code;

  distance >>= 2;

  _assert_msg_(DYNA_REC, IsInRangeImm19(distance),
               "%s: Received too large distance: %p->%p %ld %lx", __FUNCTION__, m_code, ptr,
               distance, distance);
  Write32((0x54 << 24) | (MaskImm19(distance) << 5) | cond);
}

// Test and Branch
void TBZ(ARM64Reg Rt, u8 bits, const void* ptr)
{
  EncodeTestBranchInst(0, Rt, bits, ptr);
}
void TBNZ(ARM64Reg Rt, u8 bits, const void* ptr)
{
  EncodeTestBranchInst(1, Rt, bits, ptr);
}

// Unconditional Branch
void B(const void* ptr)
{
  EncodeUnconditionalBranchInst(0, ptr);
}
void BL(const void* ptr)
{
  EncodeUnconditionalBranchInst(1, ptr);
}

void QuickCallFunction(ARM64Reg scratchreg, const void* func)
{
  s64 distance = (s64)func - (s64)m_code;
  distance >>= 2;  // Can only branch to opcode-aligned (4) addresses
  if (!IsInRangeImm26(distance))
  {
    // WARN_LOG(DYNA_REC, "Distance too far in function call (%p to %p)! Using scratch.", m_code,
    // func);
    MOVI2R(scratchreg, (uintptr_t)func);
    BLR(scratchreg);
  }
  else
  {
    BL(func);
  }
}

// Unconditional Branch (register)
void BR(ARM64Reg Rn)
{
  EncodeUnconditionalBranchInst(0, 0x1F, 0, 0, Rn);
}
void BLR(ARM64Reg Rn)
{
  EncodeUnconditionalBranchInst(1, 0x1F, 0, 0, Rn);
}
void RET(ARM64Reg Rn)
{
  EncodeUnconditionalBranchInst(2, 0x1F, 0, 0, Rn);
}
void ERET()
{
  EncodeUnconditionalBranchInst(4, 0x1F, 0, 0, SP);
}
void DRPS()
{
  EncodeUnconditionalBranchInst(5, 0x1F, 0, 0, SP);
}

// Exception generation
void SVC(u32 imm)
{
  EncodeExceptionInst(0, imm);
}

void HVC(u32 imm)
{
  EncodeExceptionInst(1, imm);
}

void SMC(u32 imm)
{
  EncodeExceptionInst(2, imm);
}

void BRK(u32 imm)
{
  EncodeExceptionInst(3, imm);
}

void HLT(u32 imm)
{
  EncodeExceptionInst(4, imm);
}

void DCPS1(u32 imm)
{
  EncodeExceptionInst(5, imm);
}

void DCPS2(u32 imm)
{
  EncodeExceptionInst(6, imm);
}

void DCPS3(u32 imm)
{
  EncodeExceptionInst(7, imm);
}

// System
void _MSR(PStateField field, u8 imm)
{
  u32 op1 = 0, op2 = 0;
  switch (field)
  {
  case FIELD_SPSel:
    op1 = 0;
    op2 = 5;
    break;
  case FIELD_DAIFSet:
    op1 = 3;
    op2 = 6;
    break;
  case FIELD_DAIFClr:
    op1 = 3;
    op2 = 7;
    break;
  default:
    _assert_msg_(DYNA_REC, false, "Invalid PStateField to do a imm move to");
    break;
  }
  EncodeSystemInst(0, op1, 4, imm, op2, WSP);
}

static void GetSystemReg(PStateField field, int& o0, int& op1, int& CRn, int& CRm, int& op2)
{
  switch (field)
  {
  case FIELD_NZCV:
    o0 = 3;
    op1 = 3;
    CRn = 4;
    CRm = 2;
    op2 = 0;
    break;
  case FIELD_FPCR:
    o0 = 3;
    op1 = 3;
    CRn = 4;
    CRm = 4;
    op2 = 0;
    break;
  case FIELD_FPSR:
    o0 = 3;
    op1 = 3;
    CRn = 4;
    CRm = 4;
    op2 = 1;
    break;
  case FIELD_PMCR_EL0:
    o0 = 3;
    op1 = 3;
    CRn = 9;
    CRm = 6;
    op2 = 0;
    break;
  case FIELD_PMCCNTR_EL0:
    o0 = 3;
    op1 = 3;
    CRn = 9;
    CRm = 7;
    op2 = 0;
    break;
  default:
    _assert_msg_(DYNA_REC, false, "Invalid PStateField to do a register move from/to");
    break;
  }
}

void _MSR(PStateField field, ARM64Reg Rt)
{
  int o0 = 0, op1 = 0, CRn = 0, CRm = 0, op2 = 0;
  _assert_msg_(DYNA_REC, Is64Bit(Rt), "MSR: Rt must be 64-bit");
  GetSystemReg(field, o0, op1, CRn, CRm, op2);
  EncodeSystemInst(o0, op1, CRn, CRm, op2, DecodeReg(Rt));
}

void MRS(ARM64Reg Rt, PStateField field)
{
  int o0 = 0, op1 = 0, CRn = 0, CRm = 0, op2 = 0;
  _assert_msg_(DYNA_REC, Is64Bit(Rt), "MRS: Rt must be 64-bit");
  GetSystemReg(field, o0, op1, CRn, CRm, op2);
  EncodeSystemInst(o0 | 4, op1, CRn, CRm, op2, DecodeReg(Rt));
}

void HINT(SystemHint op)
{
  EncodeSystemInst(0, 3, 2, 0, op, WSP);
}
void CLREX()
{
  EncodeSystemInst(0, 3, 3, 0, 2, WSP);
}
void DSB(BarrierType type)
{
  EncodeSystemInst(0, 3, 3, type, 4, WSP);
}
void DMB(BarrierType type)
{
  EncodeSystemInst(0, 3, 3, type, 5, WSP);
}
void ISB(BarrierType type)
{
  EncodeSystemInst(0, 3, 3, type, 6, WSP);
}

// Add/Subtract (extended register)
void ADD(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  ADD(Rd, Rn, Rm, ArithOption(Rd, ST_LSL, 0));
}

void ADD(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ArithOption Option)
{
  EncodeArithmeticInst(0, false, Rd, Rn, Rm, Option);
}

void ADDS(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeArithmeticInst(0, true, Rd, Rn, Rm, ArithOption(Rd, ST_LSL, 0));
}

void ADDS(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ArithOption Option)
{
  EncodeArithmeticInst(0, true, Rd, Rn, Rm, Option);
}

void SUB(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  SUB(Rd, Rn, Rm, ArithOption(Rd, ST_LSL, 0));
}

void SUB(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ArithOption Option)
{
  EncodeArithmeticInst(1, false, Rd, Rn, Rm, Option);
}

void SUBS(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeArithmeticInst(1, true, Rd, Rn, Rm, ArithOption(Rd, ST_LSL, 0));
}

void SUBS(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ArithOption Option)
{
  EncodeArithmeticInst(1, true, Rd, Rn, Rm, Option);
}

void CMN(ARM64Reg Rn, ARM64Reg Rm)
{
  CMN(Rn, Rm, ArithOption(Rn, ST_LSL, 0));
}

void CMN(ARM64Reg Rn, ARM64Reg Rm, ArithOption Option)
{
  EncodeArithmeticInst(0, true, Is64Bit(Rn) ? ZR : WZR, Rn, Rm, Option);
}

void CMP(ARM64Reg Rn, ARM64Reg Rm)
{
  CMP(Rn, Rm, ArithOption(Rn, ST_LSL, 0));
}

void CMP(ARM64Reg Rn, ARM64Reg Rm, ArithOption Option)
{
  EncodeArithmeticInst(1, true, Is64Bit(Rn) ? ZR : WZR, Rn, Rm, Option);
}

// Add/Subtract (with carry)
void ADC(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeArithmeticCarryInst(0, false, Rd, Rn, Rm);
}
void ADCS(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeArithmeticCarryInst(0, true, Rd, Rn, Rm);
}
void SBC(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeArithmeticCarryInst(1, false, Rd, Rn, Rm);
}
void SBCS(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeArithmeticCarryInst(1, true, Rd, Rn, Rm);
}

// Conditional Compare (immediate)
void CCMN(ARM64Reg Rn, u32 imm, u32 nzcv, CCFlags cond)
{
  EncodeCondCompareImmInst(0, Rn, imm, nzcv, cond);
}
void CCMP(ARM64Reg Rn, u32 imm, u32 nzcv, CCFlags cond)
{
  EncodeCondCompareImmInst(1, Rn, imm, nzcv, cond);
}

// Conditiona Compare (register)
void CCMN(ARM64Reg Rn, ARM64Reg Rm, u32 nzcv, CCFlags cond)
{
  EncodeCondCompareRegInst(0, Rn, Rm, nzcv, cond);
}
void CCMP(ARM64Reg Rn, ARM64Reg Rm, u32 nzcv, CCFlags cond)
{
  EncodeCondCompareRegInst(1, Rn, Rm, nzcv, cond);
}

// Conditional Select
void CSEL(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, CCFlags cond)
{
  EncodeCondSelectInst(0, Rd, Rn, Rm, cond);
}
void CSINC(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, CCFlags cond)
{
  EncodeCondSelectInst(1, Rd, Rn, Rm, cond);
}
void CSINV(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, CCFlags cond)
{
  EncodeCondSelectInst(2, Rd, Rn, Rm, cond);
}
void CSNEG(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, CCFlags cond)
{
  EncodeCondSelectInst(3, Rd, Rn, Rm, cond);
}

// Data-Processing 1 source
void RBIT(ARM64Reg Rd, ARM64Reg Rn)
{
  EncodeData1SrcInst(0, Rd, Rn);
}
void REV16(ARM64Reg Rd, ARM64Reg Rn)
{
  EncodeData1SrcInst(1, Rd, Rn);
}
void REV32(ARM64Reg Rd, ARM64Reg Rn)
{
  EncodeData1SrcInst(2, Rd, Rn);
}
void REV64(ARM64Reg Rd, ARM64Reg Rn)
{
  EncodeData1SrcInst(3, Rd, Rn);
}
void CLZ(ARM64Reg Rd, ARM64Reg Rn)
{
  EncodeData1SrcInst(4, Rd, Rn);
}
void CLS(ARM64Reg Rd, ARM64Reg Rn)
{
  EncodeData1SrcInst(5, Rd, Rn);
}

// Data-Processing 2 source
void UDIV(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData2SrcInst(0, Rd, Rn, Rm);
}
void SDIV(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData2SrcInst(1, Rd, Rn, Rm);
}
void LSLV(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData2SrcInst(2, Rd, Rn, Rm);
}
void LSRV(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData2SrcInst(3, Rd, Rn, Rm);
}
void ASRV(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData2SrcInst(4, Rd, Rn, Rm);
}
void RORV(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData2SrcInst(5, Rd, Rn, Rm);
}
void CRC32B(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData2SrcInst(6, Rd, Rn, Rm);
}
void CRC32H(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData2SrcInst(7, Rd, Rn, Rm);
}
void CRC32W(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData2SrcInst(8, Rd, Rn, Rm);
}
void CRC32CB(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData2SrcInst(9, Rd, Rn, Rm);
}
void CRC32CH(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData2SrcInst(10, Rd, Rn, Rm);
}
void CRC32CW(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData2SrcInst(11, Rd, Rn, Rm);
}
void CRC32X(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData2SrcInst(12, Rd, Rn, Rm);
}
void CRC32CX(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData2SrcInst(13, Rd, Rn, Rm);
}

// Data-Processing 3 source
void MADD(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ARM64Reg Ra)
{
  EncodeData3SrcInst(0, Rd, Rn, Rm, Ra);
}
void MSUB(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ARM64Reg Ra)
{
  EncodeData3SrcInst(1, Rd, Rn, Rm, Ra);
}
void SMADDL(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ARM64Reg Ra)
{
  EncodeData3SrcInst(2, Rd, Rn, Rm, Ra);
}
void SMULL(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  SMADDL(Rd, Rn, Rm, SP);
}
void SMSUBL(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ARM64Reg Ra)
{
  EncodeData3SrcInst(3, Rd, Rn, Rm, Ra);
}
void SMULH(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData3SrcInst(4, Rd, Rn, Rm, SP);
}
void UMADDL(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ARM64Reg Ra)
{
  EncodeData3SrcInst(5, Rd, Rn, Rm, Ra);
}
void UMULL(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  UMADDL(Rd, Rn, Rm, SP);
}
void UMSUBL(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ARM64Reg Ra)
{
  EncodeData3SrcInst(6, Rd, Rn, Rm, Ra);
}
void UMULH(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData3SrcInst(7, Rd, Rn, Rm, SP);
}
void MUL(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData3SrcInst(0, Rd, Rn, Rm, SP);
}
void MNEG(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EncodeData3SrcInst(1, Rd, Rn, Rm, SP);
}

// Logical (shifted register)
void AND(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ArithOption Shift)
{
  EncodeLogicalInst(0, Rd, Rn, Rm, Shift);
}
void BIC(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ArithOption Shift)
{
  EncodeLogicalInst(1, Rd, Rn, Rm, Shift);
}
void ORR(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ArithOption Shift)
{
  EncodeLogicalInst(2, Rd, Rn, Rm, Shift);
}
void ORN(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ArithOption Shift)
{
  EncodeLogicalInst(3, Rd, Rn, Rm, Shift);
}
void EOR(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ArithOption Shift)
{
  EncodeLogicalInst(4, Rd, Rn, Rm, Shift);
}
void EON(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ArithOption Shift)
{
  EncodeLogicalInst(5, Rd, Rn, Rm, Shift);
}
void ANDS(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ArithOption Shift)
{
  EncodeLogicalInst(6, Rd, Rn, Rm, Shift);
}
void BICS(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ArithOption Shift)
{
  EncodeLogicalInst(7, Rd, Rn, Rm, Shift);
}

void MOV(ARM64Reg Rd, ARM64Reg Rm, ArithOption Shift)
{
  ORR(Rd, Is64Bit(Rd) ? ZR : WZR, Rm, Shift);
}

void MOV(ARM64Reg Rd, ARM64Reg Rm)
{
  if (IsGPR(Rd) && IsGPR(Rm))
    ORR(Rd, Is64Bit(Rd) ? ZR : WZR, Rm, ArithOption(Rm, ST_LSL, 0));
  else
    _assert_msg_(DYNA_REC, false, "Non-GPRs not supported in MOV");
}
void MVN(ARM64Reg Rd, ARM64Reg Rm)
{
  ORN(Rd, Is64Bit(Rd) ? ZR : WZR, Rm, ArithOption(Rm, ST_LSL, 0));
}
void LSL(ARM64Reg Rd, ARM64Reg Rm, int shift)
{
  ORR(Rd, Is64Bit(Rd) ? ZR : WZR, Rm, ArithOption(Rm, ST_LSL, shift));
}
void LSR(ARM64Reg Rd, ARM64Reg Rm, int shift)
{
  ORR(Rd, Is64Bit(Rd) ? ZR : WZR, Rm, ArithOption(Rm, ST_LSR, shift));
}
void ASR(ARM64Reg Rd, ARM64Reg Rm, int shift)
{
  ORR(Rd, Is64Bit(Rd) ? ZR : WZR, Rm, ArithOption(Rm, ST_ASR, shift));
}
void ROR(ARM64Reg Rd, ARM64Reg Rm, int shift)
{
  ORR(Rd, Is64Bit(Rd) ? ZR : WZR, Rm, ArithOption(Rm, ST_ROR, shift));
}

// Logical (immediate)
void AND(ARM64Reg Rd, ARM64Reg Rn, u32 immr, u32 imms, bool invert)
{
  EncodeLogicalImmInst(0, Rd, Rn, immr, imms, invert);
}
void ANDS(ARM64Reg Rd, ARM64Reg Rn, u32 immr, u32 imms, bool invert)
{
  EncodeLogicalImmInst(3, Rd, Rn, immr, imms, invert);
}
void EOR(ARM64Reg Rd, ARM64Reg Rn, u32 immr, u32 imms, bool invert)
{
  EncodeLogicalImmInst(2, Rd, Rn, immr, imms, invert);
}
void ORR(ARM64Reg Rd, ARM64Reg Rn, u32 immr, u32 imms, bool invert)
{
  EncodeLogicalImmInst(1, Rd, Rn, immr, imms, invert);
}
void TST(ARM64Reg Rn, u32 immr, u32 imms, bool invert)
{
  EncodeLogicalImmInst(3, Is64Bit(Rn) ? ZR : WZR, Rn, immr, imms, invert);
}

// Add/subtract (immediate)
void ADD(ARM64Reg Rd, ARM64Reg Rn, u32 imm, bool shift)
{
  EncodeAddSubImmInst(0, false, shift, imm, Rn, Rd);
}
void ADDS(ARM64Reg Rd, ARM64Reg Rn, u32 imm, bool shift)
{
  EncodeAddSubImmInst(0, true, shift, imm, Rn, Rd);
}
void SUB(ARM64Reg Rd, ARM64Reg Rn, u32 imm, bool shift)
{
  EncodeAddSubImmInst(1, false, shift, imm, Rn, Rd);
}
void SUBS(ARM64Reg Rd, ARM64Reg Rn, u32 imm, bool shift)
{
  EncodeAddSubImmInst(1, true, shift, imm, Rn, Rd);
}
void CMP(ARM64Reg Rn, u32 imm, bool shift)
{
  EncodeAddSubImmInst(1, true, shift, imm, Rn, Is64Bit(Rn) ? SP : WSP);
}

// Data Processing (Immediate)
void MOVZ(ARM64Reg Rd, u32 imm, ShiftAmount pos)
{
  EncodeMOVWideInst(2, Rd, imm, pos);
}
void MOVN(ARM64Reg Rd, u32 imm, ShiftAmount pos)
{
  EncodeMOVWideInst(0, Rd, imm, pos);
}
void MOVK(ARM64Reg Rd, u32 imm, ShiftAmount pos)
{
  EncodeMOVWideInst(3, Rd, imm, pos);
}

// Bitfield move
void BFM(ARM64Reg Rd, ARM64Reg Rn, u32 immr, u32 imms)
{
  EncodeBitfieldMOVInst(1, Rd, Rn, immr, imms);
}
void SBFM(ARM64Reg Rd, ARM64Reg Rn, u32 immr, u32 imms)
{
  EncodeBitfieldMOVInst(0, Rd, Rn, immr, imms);
}
void UBFM(ARM64Reg Rd, ARM64Reg Rn, u32 immr, u32 imms)
{
  EncodeBitfieldMOVInst(2, Rd, Rn, immr, imms);
}

void BFI(ARM64Reg Rd, ARM64Reg Rn, u32 lsb, u32 width)
{
  u32 size = Is64Bit(Rn) ? 64 : 32;
  _assert_msg_(DYNA_REC, (lsb + width) <= size,
               "%s passed lsb %d and width %d which is greater than the register size!",
               __FUNCTION__, lsb, width);
  EncodeBitfieldMOVInst(1, Rd, Rn, (size - lsb) % size, width - 1);
}
void UBFIZ(ARM64Reg Rd, ARM64Reg Rn, u32 lsb, u32 width)
{
  u32 size = Is64Bit(Rn) ? 64 : 32;
  _assert_msg_(DYNA_REC, (lsb + width) <= size,
               "%s passed lsb %d and width %d which is greater than the register size!",
               __FUNCTION__, lsb, width);
  EncodeBitfieldMOVInst(2, Rd, Rn, (size - lsb) % size, width - 1);
}
void EXTR(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, u32 shift)
{
  bool sf = Is64Bit(Rd);
  bool N = sf;
  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);
  Rm = DecodeReg(Rm);
  Write32((sf << 31) | (0x27 << 23) | (N << 22) | (Rm << 16) | (shift << 10) | (Rm << 5) | Rd);
}
void SXTB(ARM64Reg Rd, ARM64Reg Rn)
{
  SBFM(Rd, Rn, 0, 7);
}
void SXTH(ARM64Reg Rd, ARM64Reg Rn)
{
  SBFM(Rd, Rn, 0, 15);
}
void SXTW(ARM64Reg Rd, ARM64Reg Rn)
{
  _assert_msg_(DYNA_REC, Is64Bit(Rd), "%s requires 64bit register as destination", __FUNCTION__);
  SBFM(Rd, Rn, 0, 31);
}
void UXTB(ARM64Reg Rd, ARM64Reg Rn)
{
  UBFM(Rd, Rn, 0, 7);
}
void UXTH(ARM64Reg Rd, ARM64Reg Rn)
{
  UBFM(Rd, Rn, 0, 15);
}

// Load Register (Literal)
void LDR(ARM64Reg Rt, u32 imm)
{
  EncodeLoadRegisterInst(0, Rt, imm);
}
void LDRSW(ARM64Reg Rt, u32 imm)
{
  EncodeLoadRegisterInst(2, Rt, imm);
}
void PRFM(ARM64Reg Rt, u32 imm)
{
  EncodeLoadRegisterInst(3, Rt, imm);
}

// Load/Store pair
void LDP(IndexType type, ARM64Reg Rt, ARM64Reg Rt2, ARM64Reg Rn, s32 imm)
{
  EncodeLoadStorePair(0, 1, type, Rt, Rt2, Rn, imm);
}
void LDPSW(IndexType type, ARM64Reg Rt, ARM64Reg Rt2, ARM64Reg Rn, s32 imm)
{
  EncodeLoadStorePair(1, 1, type, Rt, Rt2, Rn, imm);
}
void STP(IndexType type, ARM64Reg Rt, ARM64Reg Rt2, ARM64Reg Rn, s32 imm)
{
  EncodeLoadStorePair(0, 0, type, Rt, Rt2, Rn, imm);
}

// Load/Store Exclusive
void STXRB(ARM64Reg Rs, ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(0, Rs, SP, Rt, Rn);
}
void STLXRB(ARM64Reg Rs, ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(1, Rs, SP, Rt, Rn);
}
void LDXRB(ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(2, SP, SP, Rt, Rn);
}
void LDAXRB(ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(3, SP, SP, Rt, Rn);
}
void STLRB(ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(4, SP, SP, Rt, Rn);
}
void LDARB(ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(5, SP, SP, Rt, Rn);
}
void STXRH(ARM64Reg Rs, ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(6, Rs, SP, Rt, Rn);
}
void STLXRH(ARM64Reg Rs, ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(7, Rs, SP, Rt, Rn);
}
void LDXRH(ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(8, SP, SP, Rt, Rn);
}
void LDAXRH(ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(9, SP, SP, Rt, Rn);
}
void STLRH(ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(10, SP, SP, Rt, Rn);
}
void LDARH(ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(11, SP, SP, Rt, Rn);
}
void STXR(ARM64Reg Rs, ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(12 + Is64Bit(Rt), Rs, SP, Rt, Rn);
}
void STLXR(ARM64Reg Rs, ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(14 + Is64Bit(Rt), Rs, SP, Rt, Rn);
}
void STXP(ARM64Reg Rs, ARM64Reg Rt, ARM64Reg Rt2, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(16 + Is64Bit(Rt), Rs, Rt2, Rt, Rn);
}
void STLXP(ARM64Reg Rs, ARM64Reg Rt, ARM64Reg Rt2, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(18 + Is64Bit(Rt), Rs, Rt2, Rt, Rn);
}
void LDXR(ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(20 + Is64Bit(Rt), SP, SP, Rt, Rn);
}
void LDAXR(ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(22 + Is64Bit(Rt), SP, SP, Rt, Rn);
}
void LDXP(ARM64Reg Rt, ARM64Reg Rt2, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(24 + Is64Bit(Rt), SP, Rt2, Rt, Rn);
}
void LDAXP(ARM64Reg Rt, ARM64Reg Rt2, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(26 + Is64Bit(Rt), SP, Rt2, Rt, Rn);
}
void STLR(ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(28 + Is64Bit(Rt), SP, SP, Rt, Rn);
}
void LDAR(ARM64Reg Rt, ARM64Reg Rn)
{
  EncodeLoadStoreExcInst(30 + Is64Bit(Rt), SP, SP, Rt, Rn);
}

// Load/Store no-allocate pair (offset)
void STNP(ARM64Reg Rt, ARM64Reg Rt2, ARM64Reg Rn, u32 imm)
{
  EncodeLoadStorePairedInst(0xA0, Rt, Rt2, Rn, imm);
}
void LDNP(ARM64Reg Rt, ARM64Reg Rt2, ARM64Reg Rn, u32 imm)
{
  EncodeLoadStorePairedInst(0xA1, Rt, Rt2, Rn, imm);
}

// Load/Store register (immediate post-indexed)
// XXX: Most of these support vectors
void STRB(IndexType type, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  if (type == INDEX_UNSIGNED)
    EncodeLoadStoreIndexedInst(0x0E4, Rt, Rn, imm, 8);
  else
    EncodeLoadStoreIndexedInst(0x0E0, type == INDEX_POST ? 1 : 3, Rt, Rn, imm);
}
void LDRB(IndexType type, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  if (type == INDEX_UNSIGNED)
    EncodeLoadStoreIndexedInst(0x0E5, Rt, Rn, imm, 8);
  else
    EncodeLoadStoreIndexedInst(0x0E1, type == INDEX_POST ? 1 : 3, Rt, Rn, imm);
}
void LDRSB(IndexType type, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  if (type == INDEX_UNSIGNED)
    EncodeLoadStoreIndexedInst(Is64Bit(Rt) ? 0x0E6 : 0x0E7, Rt, Rn, imm, 8);
  else
    EncodeLoadStoreIndexedInst(Is64Bit(Rt) ? 0x0E2 : 0x0E3, type == INDEX_POST ? 1 : 3, Rt, Rn,
                               imm);
}
void STRH(IndexType type, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  if (type == INDEX_UNSIGNED)
    EncodeLoadStoreIndexedInst(0x1E4, Rt, Rn, imm, 16);
  else
    EncodeLoadStoreIndexedInst(0x1E0, type == INDEX_POST ? 1 : 3, Rt, Rn, imm);
}
void LDRH(IndexType type, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  if (type == INDEX_UNSIGNED)
    EncodeLoadStoreIndexedInst(0x1E5, Rt, Rn, imm, 16);
  else
    EncodeLoadStoreIndexedInst(0x1E1, type == INDEX_POST ? 1 : 3, Rt, Rn, imm);
}
void LDRSH(IndexType type, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  if (type == INDEX_UNSIGNED)
    EncodeLoadStoreIndexedInst(Is64Bit(Rt) ? 0x1E6 : 0x1E7, Rt, Rn, imm, 16);
  else
    EncodeLoadStoreIndexedInst(Is64Bit(Rt) ? 0x1E2 : 0x1E3, type == INDEX_POST ? 1 : 3, Rt, Rn,
                               imm);
}
void STR(IndexType type, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  if (type == INDEX_UNSIGNED)
    EncodeLoadStoreIndexedInst(Is64Bit(Rt) ? 0x3E4 : 0x2E4, Rt, Rn, imm, Is64Bit(Rt) ? 64 : 32);
  else
    EncodeLoadStoreIndexedInst(Is64Bit(Rt) ? 0x3E0 : 0x2E0, type == INDEX_POST ? 1 : 3, Rt, Rn,
                               imm);
}
void LDR(IndexType type, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  if (type == INDEX_UNSIGNED)
    EncodeLoadStoreIndexedInst(Is64Bit(Rt) ? 0x3E5 : 0x2E5, Rt, Rn, imm, Is64Bit(Rt) ? 64 : 32);
  else
    EncodeLoadStoreIndexedInst(Is64Bit(Rt) ? 0x3E1 : 0x2E1, type == INDEX_POST ? 1 : 3, Rt, Rn,
                               imm);
}
void LDRSW(IndexType type, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  if (type == INDEX_UNSIGNED)
    EncodeLoadStoreIndexedInst(0x2E6, Rt, Rn, imm, 32);
  else
    EncodeLoadStoreIndexedInst(0x2E2, type == INDEX_POST ? 1 : 3, Rt, Rn, imm);
}

// Load/Store register (register offset)
void STRB(ARM64Reg Rt, ARM64Reg Rn, ArithOption Rm)
{
  EncodeLoadStoreRegisterOffset(0, 0, Rt, Rn, Rm);
}
void LDRB(ARM64Reg Rt, ARM64Reg Rn, ArithOption Rm)
{
  EncodeLoadStoreRegisterOffset(0, 1, Rt, Rn, Rm);
}
void LDRSB(ARM64Reg Rt, ARM64Reg Rn, ArithOption Rm)
{
  bool b64Bit = Is64Bit(Rt);
  EncodeLoadStoreRegisterOffset(0, 3 - b64Bit, Rt, Rn, Rm);
}
void STRH(ARM64Reg Rt, ARM64Reg Rn, ArithOption Rm)
{
  EncodeLoadStoreRegisterOffset(1, 0, Rt, Rn, Rm);
}
void LDRH(ARM64Reg Rt, ARM64Reg Rn, ArithOption Rm)
{
  EncodeLoadStoreRegisterOffset(1, 1, Rt, Rn, Rm);
}
void LDRSH(ARM64Reg Rt, ARM64Reg Rn, ArithOption Rm)
{
  bool b64Bit = Is64Bit(Rt);
  EncodeLoadStoreRegisterOffset(1, 3 - b64Bit, Rt, Rn, Rm);
}
void STR(ARM64Reg Rt, ARM64Reg Rn, ArithOption Rm)
{
  bool b64Bit = Is64Bit(Rt);
  EncodeLoadStoreRegisterOffset(2 + b64Bit, 0, Rt, Rn, Rm);
}
void LDR(ARM64Reg Rt, ARM64Reg Rn, ArithOption Rm)
{
  bool b64Bit = Is64Bit(Rt);
  EncodeLoadStoreRegisterOffset(2 + b64Bit, 1, Rt, Rn, Rm);
}
void LDRSW(ARM64Reg Rt, ARM64Reg Rn, ArithOption Rm)
{
  EncodeLoadStoreRegisterOffset(2, 2, Rt, Rn, Rm);
}
void PRFM(ARM64Reg Rt, ARM64Reg Rn, ArithOption Rm)
{
  EncodeLoadStoreRegisterOffset(3, 2, Rt, Rn, Rm);
}

// Load/Store register (unscaled offset)
void STURB(ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  EncodeLoadStoreUnscaled(0, 0, Rt, Rn, imm);
}
void LDURB(ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  EncodeLoadStoreUnscaled(0, 1, Rt, Rn, imm);
}
void LDURSB(ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  EncodeLoadStoreUnscaled(0, Is64Bit(Rt) ? 2 : 3, Rt, Rn, imm);
}
void STURH(ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  EncodeLoadStoreUnscaled(1, 0, Rt, Rn, imm);
}
void LDURH(ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  EncodeLoadStoreUnscaled(1, 1, Rt, Rn, imm);
}
void LDURSH(ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  EncodeLoadStoreUnscaled(1, Is64Bit(Rt) ? 2 : 3, Rt, Rn, imm);
}
void STUR(ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  EncodeLoadStoreUnscaled(Is64Bit(Rt) ? 3 : 2, 0, Rt, Rn, imm);
}
void LDUR(ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  EncodeLoadStoreUnscaled(Is64Bit(Rt) ? 3 : 2, 1, Rt, Rn, imm);
}
void LDURSW(ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  _assert_msg_(DYNA_REC, !Is64Bit(Rt), "%s must have a 64bit destination register!", __FUNCTION__);
  EncodeLoadStoreUnscaled(2, 2, Rt, Rn, imm);
}

// Address of label/page PC-relative
void ADR(ARM64Reg Rd, s32 imm)
{
  EncodeAddressInst(0, Rd, imm);
}
void ADRP(ARM64Reg Rd, s32 imm)
{
  EncodeAddressInst(1, Rd, imm >> 12);
}

// Wrapper around MOVZ+MOVK (and later MOVN)
void MOVI2R(ARM64Reg Rd, u64 imm, bool optimize)
{
  unsigned int parts = Is64Bit(Rd) ? 4 : 2;
  BitSet32 upload_part(0);

  // Always start with a movz! Kills the dependency on the register.
  bool use_movz = true;

  if (!imm)
  {
    // Zero immediate, just clear the register. EOR is pointless when we have MOVZ, which looks
    // clearer in disasm too.
    MOVZ(Rd, 0, SHIFT_0);
    return;
  }

  if ((Is64Bit(Rd) && imm == std::numeric_limits<u64>::max()) ||
      (!Is64Bit(Rd) && imm == std::numeric_limits<u32>::max()))
  {
    // Max unsigned value (or if signed, -1)
    // Set to ~ZR
    ARM64Reg ZR = Is64Bit(Rd) ? SP : WSP;
    ORN(Rd, ZR, ZR, ArithOption(ZR, ST_LSL, 0));
    return;
  }

  // TODO: Make some more systemic use of MOVN, but this will take care of most cases.
  // Small negative integer. Use MOVN
  if (!Is64Bit(Rd) && (imm | 0xFFFF0000) == imm)
  {
    MOVN(Rd, ~imm, SHIFT_0);
    return;
  }

  // XXX: Use MOVN when possible.
  // XXX: Optimize more
  // XXX: Support rotating immediates to save instructions
  if (optimize)
  {
    for (unsigned int i = 0; i < parts; ++i)
    {
      if ((imm >> (i * 16)) & 0xFFFF)
        upload_part[i] = 1;
    }
  }

  u64 aligned_pc = (u64)GetCodePtr() & ~0xFFF;
  s64 aligned_offset = (s64)imm - (s64)aligned_pc;
  if (upload_part.Count() > 1 && std::abs(aligned_offset) < 0xFFFFFFFFLL)
  {
    // Immediate we are loading is within 4GB of our aligned range
    // Most likely a address that we can load in one or two instructions
    if (!(std::abs(aligned_offset) & 0xFFF))
    {
      // Aligned ADR
      ADRP(Rd, (s32)aligned_offset);
      return;
    }
    else
    {
      // If the address is within 1MB of PC we can load it in a single instruction still
      s64 offset = (s64)imm - (s64)GetCodePtr();
      if (offset >= -0xFFFFF && offset <= 0xFFFFF)
      {
        ADR(Rd, (s32)offset);
        return;
      }
      else
      {
        ADRP(Rd, (s32)(aligned_offset & ~0xFFF));
        ADD(Rd, Rd, imm & 0xFFF);
        return;
      }
    }
  }

  for (unsigned i = 0; i < parts; ++i)
  {
    if (use_movz && upload_part[i])
    {
      MOVZ(Rd, (imm >> (i * 16)) & 0xFFFF, (ShiftAmount)i);
      use_movz = false;
    }
    else
    {
      if (upload_part[i] || !optimize)
        MOVK(Rd, (imm >> (i * 16)) & 0xFFFF, (ShiftAmount)i);
    }
  }
}

void ABI_PushRegisters(BitSet32 registers)
{
  int num_regs = registers.Count();

  if (num_regs % 2)
  {
    bool first = true;

    // Stack is required to be quad-word aligned.
    u32 stack_size = ROUND_UP(num_regs * 8, 16);
    u32 current_offset = 0;
    std::vector<ARM64Reg> reg_pair;

    for (auto it : registers)
    {
      if (first)
      {
        STR(INDEX_PRE, (ARM64Reg)(X0 + it), SP, -(s32)stack_size);
        first = false;
        current_offset += 16;
      }
      else
      {
        reg_pair.push_back((ARM64Reg)(X0 + it));
        if (reg_pair.size() == 2)
        {
          STP(INDEX_SIGNED, reg_pair[0], reg_pair[1], SP, current_offset);
          reg_pair.clear();
          current_offset += 16;
        }
      }
    }
  }
  else
  {
    std::vector<ARM64Reg> reg_pair;

    for (auto it : registers)
    {
      reg_pair.push_back((ARM64Reg)(X0 + it));
      if (reg_pair.size() == 2)
      {
        STP(INDEX_PRE, reg_pair[0], reg_pair[1], SP, -16);
        reg_pair.clear();
      }
    }
  }
}

void ABI_PopRegisters(BitSet32 registers, BitSet32 ignore_mask)
{
  int num_regs = registers.Count();

  if (num_regs % 2)
  {
    bool first = true;

    std::vector<ARM64Reg> reg_pair;

    for (auto it : registers)
    {
      if (ignore_mask[it])
        it = WSP;

      if (first)
      {
        LDR(INDEX_POST, (ARM64Reg)(X0 + it), SP, 16);
        first = false;
      }
      else
      {
        reg_pair.push_back((ARM64Reg)(X0 + it));
        if (reg_pair.size() == 2)
        {
          LDP(INDEX_POST, reg_pair[0], reg_pair[1], SP, 16);
          reg_pair.clear();
        }
      }
    }
  }
  else
  {
    std::vector<ARM64Reg> reg_pair;

    for (int i = 31; i >= 0; --i)
    {
      if (!registers[i])
        continue;

      int reg = i;

      if (ignore_mask[reg])
        reg = WSP;

      reg_pair.push_back((ARM64Reg)(X0 + reg));
      if (reg_pair.size() == 2)
      {
        LDP(INDEX_POST, reg_pair[1], reg_pair[0], SP, 16);
        reg_pair.clear();
      }
    }
  }
}

// Float Emitter
void EmitLoadStoreImmediate(u8 size, u32 opc, IndexType type, ARM64Reg Rt,
                                               ARM64Reg Rn, s32 imm)
{
  Rt = DecodeReg(Rt);
  Rn = DecodeReg(Rn);
  u32 encoded_size = 0;
  u32 encoded_imm = 0;

  if (size == 8)
    encoded_size = 0;
  else if (size == 16)
    encoded_size = 1;
  else if (size == 32)
    encoded_size = 2;
  else if (size == 64)
    encoded_size = 3;
  else if (size == 128)
    encoded_size = 0;

  if (type == INDEX_UNSIGNED)
  {
    _assert_msg_(DYNA_REC, !(imm & ((size - 1) >> 3)),
                 "%s(INDEX_UNSIGNED) immediate offset must be aligned to size! (%d) (%p)",
                 __FUNCTION__, imm, m_emit->GetCodePtr());
    _assert_msg_(DYNA_REC, imm >= 0, "%s(INDEX_UNSIGNED) immediate offset must be positive!",
                 __FUNCTION__);
    if (size == 16)
      imm >>= 1;
    else if (size == 32)
      imm >>= 2;
    else if (size == 64)
      imm >>= 3;
    else if (size == 128)
      imm >>= 4;
    encoded_imm = (imm & 0xFFF);
  }
  else
  {
    _assert_msg_(DYNA_REC, !(imm < -256 || imm > 255),
                 "%s immediate offset must be within range of -256 to 256!", __FUNCTION__);
    encoded_imm = (imm & 0x1FF) << 2;
    if (type == INDEX_POST)
      encoded_imm |= 1;
    else
      encoded_imm |= 3;
  }

  Write32((encoded_size << 30) | (0xF << 26) | (type == INDEX_UNSIGNED ? (1 << 24) : 0) |
          (size == 128 ? (1 << 23) : 0) | (opc << 22) | (encoded_imm << 10) | (Rn << 5) | Rt);
}

void EmitScalar2Source(bool M, bool S, u32 type, u32 opcode, ARM64Reg Rd,
                                          ARM64Reg Rn, ARM64Reg Rm)
{
  _assert_msg_(DYNA_REC, !IsQuad(Rd), "%s only supports double and single registers!",
               __FUNCTION__);
  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);
  Rm = DecodeReg(Rm);

  Write32((M << 31) | (S << 29) | (0b11110001 << 21) | (type << 22) | (Rm << 16) | (opcode << 12) |
          (1 << 11) | (Rn << 5) | Rd);
}

void EmitThreeSame(bool U, u32 size, u32 opcode, ARM64Reg Rd, ARM64Reg Rn,
                                      ARM64Reg Rm)
{
  _assert_msg_(DYNA_REC, !IsSingle(Rd), "%s doesn't support singles!", __FUNCTION__);
  bool quad = IsQuad(Rd);
  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);
  Rm = DecodeReg(Rm);

  Write32((quad << 30) | (U << 29) | (0b1110001 << 21) | (size << 22) | (Rm << 16) |
          (opcode << 11) | (1 << 10) | (Rn << 5) | Rd);
}

void EmitCopy(bool Q, u32 op, u32 imm5, u32 imm4, ARM64Reg Rd, ARM64Reg Rn)
{
  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);

  Write32((Q << 30) | (op << 29) | (0b111 << 25) | (imm5 << 16) | (imm4 << 11) | (1 << 10) |
          (Rn << 5) | Rd);
}

void Emit2RegMisc(bool Q, bool U, u32 size, u32 opcode, ARM64Reg Rd, ARM64Reg Rn)
{
  _assert_msg_(DYNA_REC, !IsSingle(Rd), "%s doesn't support singles!", __FUNCTION__);
  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);

  Write32((Q << 30) | (U << 29) | (0b1110001 << 21) | (size << 22) | (opcode << 12) | (1 << 11) |
          (Rn << 5) | Rd);
}

void EmitLoadStoreSingleStructure(bool L, bool R, u32 opcode, bool S, u32 size,
                                                     ARM64Reg Rt, ARM64Reg Rn)
{
  _assert_msg_(DYNA_REC, !IsSingle(Rt), "%s doesn't support singles!", __FUNCTION__);
  bool quad = IsQuad(Rt);
  Rt = DecodeReg(Rt);
  Rn = DecodeReg(Rn);

  Write32((quad << 30) | (0b1101 << 24) | (L << 22) | (R << 21) | (opcode << 13) | (S << 12) |
          (size << 10) | (Rn << 5) | Rt);
}

void EmitLoadStoreSingleStructure(bool L, bool R, u32 opcode, bool S, u32 size,
                                                     ARM64Reg Rt, ARM64Reg Rn, ARM64Reg Rm)
{
  _assert_msg_(DYNA_REC, !IsSingle(Rt), "%s doesn't support singles!", __FUNCTION__);
  bool quad = IsQuad(Rt);
  Rt = DecodeReg(Rt);
  Rn = DecodeReg(Rn);
  Rm = DecodeReg(Rm);

  Write32((quad << 30) | (0x1B << 23) | (L << 22) | (R << 21) | (Rm << 16) | (opcode << 13) |
          (S << 12) | (size << 10) | (Rn << 5) | Rt);
}

void Emit1Source(bool M, bool S, u32 type, u32 opcode, ARM64Reg Rd, ARM64Reg Rn)
{
  _assert_msg_(DYNA_REC, !IsQuad(Rd), "%s doesn't support vector!", __FUNCTION__);
  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);

  Write32((M << 31) | (S << 29) | (0xF1 << 21) | (type << 22) | (opcode << 15) | (1 << 14) |
          (Rn << 5) | Rd);
}

void EmitConversion(bool sf, bool S, u32 type, u32 rmode, u32 opcode,
                                       ARM64Reg Rd, ARM64Reg Rn)
{
  _assert_msg_(DYNA_REC, Rn <= SP, "%s only supports GPR as source!", __FUNCTION__);
  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);

  Write32((sf << 31) | (S << 29) | (0xF1 << 21) | (type << 22) | (rmode << 19) | (opcode << 16) |
          (Rn << 5) | Rd);
}

void EmitConvertScalarToInt(ARM64Reg Rd, ARM64Reg Rn, RoundingMode round,
                                               bool sign)
{
  _dbg_assert_msg_(DYNA_REC, IsScalar(Rn), "fcvts: Rn must be floating point");
  if (IsGPR(Rd))
  {
    // Use the encoding that transfers the result to a GPR.
    bool sf = Is64Bit(Rd);
    int type = IsDouble(Rn) ? 1 : 0;
    Rd = DecodeReg(Rd);
    Rn = DecodeReg(Rn);
    int opcode = (sign ? 1 : 0);
    int rmode = 0;
    switch (round)
    {
    case ROUND_A:
      rmode = 0;
      opcode |= 4;
      break;
    case ROUND_P:
      rmode = 1;
      break;
    case ROUND_M:
      rmode = 2;
      break;
    case ROUND_Z:
      rmode = 3;
      break;
    case ROUND_N:
      rmode = 0;
      break;
    }
    EmitConversion2(sf, 0, true, type, rmode, opcode, 0, Rd, Rn);
  }
  else
  {
    // Use the encoding (vector, single) that keeps the result in the fp register.
    int sz = IsDouble(Rn);
    Rd = DecodeReg(Rd);
    Rn = DecodeReg(Rn);
    int opcode = 0;
    switch (round)
    {
    case ROUND_A:
      opcode = 0x1C;
      break;
    case ROUND_N:
      opcode = 0x1A;
      break;
    case ROUND_M:
      opcode = 0x1B;
      break;
    case ROUND_P:
      opcode = 0x1A;
      sz |= 2;
      break;
    case ROUND_Z:
      opcode = 0x1B;
      sz |= 2;
      break;
    }
    Write32((0x5E << 24) | (sign << 29) | (sz << 22) | (1 << 21) | (opcode << 12) | (2 << 10) |
            (Rn << 5) | Rd);
  }
}

void FCVTS(ARM64Reg Rd, ARM64Reg Rn, RoundingMode round)
{
  EmitConvertScalarToInt(Rd, Rn, round, false);
}

void FCVTU(ARM64Reg Rd, ARM64Reg Rn, RoundingMode round)
{
  EmitConvertScalarToInt(Rd, Rn, round, true);
}

void EmitConversion2(bool sf, bool S, bool direction, u32 type, u32 rmode,
                                        u32 opcode, int scale, ARM64Reg Rd, ARM64Reg Rn)
{
  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);

  Write32((sf << 31) | (S << 29) | (0xF0 << 21) | (direction << 21) | (type << 22) | (rmode << 19) |
          (opcode << 16) | (scale << 10) | (Rn << 5) | Rd);
}

void EmitCompare(bool M, bool S, u32 op, u32 opcode2, ARM64Reg Rn, ARM64Reg Rm)
{
  _assert_msg_(DYNA_REC, !IsQuad(Rn), "%s doesn't support vector!", __FUNCTION__);
  bool is_double = IsDouble(Rn);

  Rn = DecodeReg(Rn);
  Rm = DecodeReg(Rm);

  Write32((M << 31) | (S << 29) | (0xF1 << 21) | (is_double << 22) | (Rm << 16) | (op << 14) |
          (1 << 13) | (Rn << 5) | opcode2);
}

void EmitCondSelect(bool M, bool S, CCFlags cond, ARM64Reg Rd, ARM64Reg Rn,
                                       ARM64Reg Rm)
{
  _assert_msg_(DYNA_REC, !IsQuad(Rd), "%s doesn't support vector!", __FUNCTION__);
  bool is_double = IsDouble(Rd);

  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);
  Rm = DecodeReg(Rm);

  Write32((M << 31) | (S << 29) | (0xF1 << 21) | (is_double << 22) | (Rm << 16) | (cond << 12) |
          (3 << 10) | (Rn << 5) | Rd);
}

void EmitPermute(u32 size, u32 op, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  _assert_msg_(DYNA_REC, !IsSingle(Rd), "%s doesn't support singles!", __FUNCTION__);

  bool quad = IsQuad(Rd);

  u32 encoded_size = 0;
  if (size == 16)
    encoded_size = 1;
  else if (size == 32)
    encoded_size = 2;
  else if (size == 64)
    encoded_size = 3;

  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);
  Rm = DecodeReg(Rm);

  Write32((quad << 30) | (7 << 25) | (encoded_size << 22) | (Rm << 16) | (op << 12) | (1 << 11) |
          (Rn << 5) | Rd);
}

void EmitScalarImm(bool M, bool S, u32 type, u32 imm5, ARM64Reg Rd, u32 imm8)
{
  _assert_msg_(DYNA_REC, !IsQuad(Rd), "%s doesn't support vector!", __FUNCTION__);

  bool is_double = !IsSingle(Rd);

  Rd = DecodeReg(Rd);

  Write32((M << 31) | (S << 29) | (0xF1 << 21) | (is_double << 22) | (type << 22) | (imm8 << 13) |
          (1 << 12) | (imm5 << 5) | Rd);
}

void EmitShiftImm(bool Q, bool U, u32 immh, u32 immb, u32 opcode, ARM64Reg Rd,
                                     ARM64Reg Rn)
{
  _assert_msg_(DYNA_REC, immh, "%s bad encoding! Can't have zero immh", __FUNCTION__);

  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);

  Write32((Q << 30) | (U << 29) | (0xF << 24) | (immh << 19) | (immb << 16) | (opcode << 11) |
          (1 << 10) | (Rn << 5) | Rd);
}

void EmitScalarShiftImm(bool U, u32 immh, u32 immb, u32 opcode, ARM64Reg Rd,
                                           ARM64Reg Rn)
{
  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);

  Write32((2 << 30) | (U << 29) | (0x3E << 23) | (immh << 19) | (immb << 16) | (opcode << 11) |
          (1 << 10) | (Rn << 5) | Rd);
}

void EmitLoadStoreMultipleStructure(u32 size, bool L, u32 opcode, ARM64Reg Rt,
                                                       ARM64Reg Rn)
{
  bool quad = IsQuad(Rt);
  u32 encoded_size = 0;

  if (size == 16)
    encoded_size = 1;
  else if (size == 32)
    encoded_size = 2;
  else if (size == 64)
    encoded_size = 3;

  Rt = DecodeReg(Rt);
  Rn = DecodeReg(Rn);

  Write32((quad << 30) | (3 << 26) | (L << 22) | (opcode << 12) | (encoded_size << 10) | (Rn << 5) |
          Rt);
}

void EmitLoadStoreMultipleStructurePost(u32 size, bool L, u32 opcode,
                                                           ARM64Reg Rt, ARM64Reg Rn, ARM64Reg Rm)
{
  bool quad = IsQuad(Rt);
  u32 encoded_size = 0;

  if (size == 16)
    encoded_size = 1;
  else if (size == 32)
    encoded_size = 2;
  else if (size == 64)
    encoded_size = 3;

  Rt = DecodeReg(Rt);
  Rn = DecodeReg(Rn);
  Rm = DecodeReg(Rm);

  Write32((quad << 30) | (0b11001 << 23) | (L << 22) | (Rm << 16) | (opcode << 12) |
          (encoded_size << 10) | (Rn << 5) | Rt);
}

void EmitScalar1Source(bool M, bool S, u32 type, u32 opcode, ARM64Reg Rd,
                                          ARM64Reg Rn)
{
  _assert_msg_(DYNA_REC, !IsQuad(Rd), "%s doesn't support vector!", __FUNCTION__);

  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);

  Write32((M << 31) | (S << 29) | (0xF1 << 21) | (type << 22) | (opcode << 15) | (1 << 14) |
          (Rn << 5) | Rd);
}

void EmitVectorxElement(bool U, u32 size, bool L, u32 opcode, bool H,
                                           ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  bool quad = IsQuad(Rd);

  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);
  Rm = DecodeReg(Rm);

  Write32((quad << 30) | (U << 29) | (0xF << 24) | (size << 22) | (L << 21) | (Rm << 16) |
          (opcode << 12) | (H << 11) | (Rn << 5) | Rd);
}

void EmitLoadStoreUnscaled(u32 size, u32 op, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  _assert_msg_(DYNA_REC, !(imm < -256 || imm > 255), "%s received too large offset: %d",
               __FUNCTION__, imm);
  Rt = DecodeReg(Rt);
  Rn = DecodeReg(Rn);

  Write32((size << 30) | (0xF << 26) | (op << 22) | ((imm & 0x1FF) << 12) | (Rn << 5) | Rt);
}

void EncodeLoadStorePair(u32 size, bool load, IndexType type, ARM64Reg Rt,
                                            ARM64Reg Rt2, ARM64Reg Rn, s32 imm)
{
  u32 type_encode = 0;
  u32 opc = 0;

  switch (type)
  {
  case INDEX_SIGNED:
    type_encode = 0b010;
    break;
  case INDEX_POST:
    type_encode = 0b001;
    break;
  case INDEX_PRE:
    type_encode = 0b011;
    break;
  case INDEX_UNSIGNED:
    _assert_msg_(DYNA_REC, false, "%s doesn't support INDEX_UNSIGNED!", __FUNCTION__);
    break;
  }

  if (size == 128)
  {
    _assert_msg_(DYNA_REC, !(imm & 0xF), "%s received invalid offset 0x%x!", __FUNCTION__, imm);
    opc = 2;
    imm >>= 4;
  }
  else if (size == 64)
  {
    _assert_msg_(DYNA_REC, !(imm & 0x7), "%s received invalid offset 0x%x!", __FUNCTION__, imm);
    opc = 1;
    imm >>= 3;
  }
  else if (size == 32)
  {
    _assert_msg_(DYNA_REC, !(imm & 0x3), "%s received invalid offset 0x%x!", __FUNCTION__, imm);
    opc = 0;
    imm >>= 2;
  }

  Rt = DecodeReg(Rt);
  Rt2 = DecodeReg(Rt2);
  Rn = DecodeReg(Rn);

  Write32((opc << 30) | (0b1011 << 26) | (type_encode << 23) | (load << 22) | ((imm & 0x7F) << 15) |
          (Rt2 << 10) | (Rn << 5) | Rt);
}

void EncodeLoadStoreRegisterOffset(u32 size, bool load, ARM64Reg Rt, ARM64Reg Rn,
                                                      ArithOption Rm)
{
  _assert_msg_(DYNA_REC, Rm.GetType() == ArithOption::TYPE_EXTENDEDREG,
               "%s must contain an extended reg as Rm!", __FUNCTION__);

  u32 encoded_size = 0;
  u32 encoded_op = 0;

  if (size == 8)
  {
    encoded_size = 0;
    encoded_op = 0;
  }
  else if (size == 16)
  {
    encoded_size = 1;
    encoded_op = 0;
  }
  else if (size == 32)
  {
    encoded_size = 2;
    encoded_op = 0;
  }
  else if (size == 64)
  {
    encoded_size = 3;
    encoded_op = 0;
  }
  else if (size == 128)
  {
    encoded_size = 0;
    encoded_op = 2;
  }

  if (load)
    encoded_op |= 1;

  Rt = DecodeReg(Rt);
  Rn = DecodeReg(Rn);
  ARM64Reg decoded_Rm = DecodeReg(Rm.GetReg());

  Write32((encoded_size << 30) | (encoded_op << 22) | (0b111100001 << 21) | (decoded_Rm << 16) |
          Rm.GetData() | (1 << 11) | (Rn << 5) | Rt);
}

void EncodeModImm(bool Q, u8 op, u8 cmode, u8 o2, ARM64Reg Rd, u8 abcdefgh)
{
  union {
    u8 hex;
    struct
    {
      unsigned defgh : 5;
      unsigned abc : 3;
    };
  } v;
  v.hex = abcdefgh;
  Rd = DecodeReg(Rd);
  Write32((Q << 30) | (op << 29) | (0xF << 24) | (v.abc << 16) | (cmode << 12) | (o2 << 11) |
          (1 << 10) | (v.defgh << 5) | Rd);
}

void LDR(u8 size, IndexType type, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  EmitLoadStoreImmediate(size, 1, type, Rt, Rn, imm);
}
void STR(u8 size, IndexType type, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  EmitLoadStoreImmediate(size, 0, type, Rt, Rn, imm);
}

// Loadstore unscaled
void LDUR(u8 size, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  u32 encoded_size = 0;
  u32 encoded_op = 0;

  if (size == 8)
  {
    encoded_size = 0;
    encoded_op = 1;
  }
  else if (size == 16)
  {
    encoded_size = 1;
    encoded_op = 1;
  }
  else if (size == 32)
  {
    encoded_size = 2;
    encoded_op = 1;
  }
  else if (size == 64)
  {
    encoded_size = 3;
    encoded_op = 1;
  }
  else if (size == 128)
  {
    encoded_size = 0;
    encoded_op = 3;
  }

  EmitLoadStoreUnscaled(encoded_size, encoded_op, Rt, Rn, imm);
}
void STUR(u8 size, ARM64Reg Rt, ARM64Reg Rn, s32 imm)
{
  u32 encoded_size = 0;
  u32 encoded_op = 0;

  if (size == 8)
  {
    encoded_size = 0;
    encoded_op = 0;
  }
  else if (size == 16)
  {
    encoded_size = 1;
    encoded_op = 0;
  }
  else if (size == 32)
  {
    encoded_size = 2;
    encoded_op = 0;
  }
  else if (size == 64)
  {
    encoded_size = 3;
    encoded_op = 0;
  }
  else if (size == 128)
  {
    encoded_size = 0;
    encoded_op = 2;
  }

  EmitLoadStoreUnscaled(encoded_size, encoded_op, Rt, Rn, imm);
}

// Loadstore single structure
void LD1(u8 size, ARM64Reg Rt, u8 index, ARM64Reg Rn)
{
  bool S = 0;
  u32 opcode = 0;
  u32 encoded_size = 0;
  ARM64Reg encoded_reg = INVALID_REG;

  if (size == 8)
  {
    S = (index & 4) != 0;
    opcode = 0;
    encoded_size = index & 3;
    if (index & 8)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }
  else if (size == 16)
  {
    S = (index & 2) != 0;
    opcode = 2;
    encoded_size = (index & 1) << 1;
    if (index & 4)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }
  else if (size == 32)
  {
    S = (index & 1) != 0;
    opcode = 4;
    encoded_size = 0;
    if (index & 2)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }
  else if (size == 64)
  {
    S = 0;
    opcode = 4;
    encoded_size = 1;
    if (index == 1)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }

  EmitLoadStoreSingleStructure(1, 0, opcode, S, encoded_size, encoded_reg, Rn);
}

void LD1(u8 size, ARM64Reg Rt, u8 index, ARM64Reg Rn, ARM64Reg Rm)
{
  bool S = 0;
  u32 opcode = 0;
  u32 encoded_size = 0;
  ARM64Reg encoded_reg = INVALID_REG;

  if (size == 8)
  {
    S = (index & 4) != 0;
    opcode = 0;
    encoded_size = index & 3;
    if (index & 8)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }
  else if (size == 16)
  {
    S = (index & 2) != 0;
    opcode = 2;
    encoded_size = (index & 1) << 1;
    if (index & 4)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }
  else if (size == 32)
  {
    S = (index & 1) != 0;
    opcode = 4;
    encoded_size = 0;
    if (index & 2)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }
  else if (size == 64)
  {
    S = 0;
    opcode = 4;
    encoded_size = 1;
    if (index == 1)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }

  EmitLoadStoreSingleStructure(1, 0, opcode, S, encoded_size, encoded_reg, Rn, Rm);
}

void LD1R(u8 size, ARM64Reg Rt, ARM64Reg Rn)
{
  EmitLoadStoreSingleStructure(1, 0, 6, 0, size >> 4, Rt, Rn);
}
void LD2R(u8 size, ARM64Reg Rt, ARM64Reg Rn)
{
  EmitLoadStoreSingleStructure(1, 1, 6, 0, size >> 4, Rt, Rn);
}
void LD1R(u8 size, ARM64Reg Rt, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitLoadStoreSingleStructure(1, 0, 6, 0, size >> 4, Rt, Rn, Rm);
}
void LD2R(u8 size, ARM64Reg Rt, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitLoadStoreSingleStructure(1, 1, 6, 0, size >> 4, Rt, Rn, Rm);
}

void ST1(u8 size, ARM64Reg Rt, u8 index, ARM64Reg Rn)
{
  bool S = 0;
  u32 opcode = 0;
  u32 encoded_size = 0;
  ARM64Reg encoded_reg = INVALID_REG;

  if (size == 8)
  {
    S = (index & 4) != 0;
    opcode = 0;
    encoded_size = index & 3;
    if (index & 8)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }
  else if (size == 16)
  {
    S = (index & 2) != 0;
    opcode = 2;
    encoded_size = (index & 1) << 1;
    if (index & 4)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }
  else if (size == 32)
  {
    S = (index & 1) != 0;
    opcode = 4;
    encoded_size = 0;
    if (index & 2)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }
  else if (size == 64)
  {
    S = 0;
    opcode = 4;
    encoded_size = 1;
    if (index == 1)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }

  EmitLoadStoreSingleStructure(0, 0, opcode, S, encoded_size, encoded_reg, Rn);
}

void ST1(u8 size, ARM64Reg Rt, u8 index, ARM64Reg Rn, ARM64Reg Rm)
{
  bool S = 0;
  u32 opcode = 0;
  u32 encoded_size = 0;
  ARM64Reg encoded_reg = INVALID_REG;

  if (size == 8)
  {
    S = (index & 4) != 0;
    opcode = 0;
    encoded_size = index & 3;
    if (index & 8)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }
  else if (size == 16)
  {
    S = (index & 2) != 0;
    opcode = 2;
    encoded_size = (index & 1) << 1;
    if (index & 4)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }
  else if (size == 32)
  {
    S = (index & 1) != 0;
    opcode = 4;
    encoded_size = 0;
    if (index & 2)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }
  else if (size == 64)
  {
    S = 0;
    opcode = 4;
    encoded_size = 1;
    if (index == 1)
      encoded_reg = EncodeRegToQuad(Rt);
    else
      encoded_reg = EncodeRegToDouble(Rt);
  }

  EmitLoadStoreSingleStructure(0, 0, opcode, S, encoded_size, encoded_reg, Rn, Rm);
}

// Loadstore multiple structure
void LD1(u8 size, u8 count, ARM64Reg Rt, ARM64Reg Rn)
{
  _assert_msg_(DYNA_REC, !(count == 0 || count > 4), "%s must have a count of 1 to 4 registers!",
               __FUNCTION__);
  u32 opcode = 0;
  if (count == 1)
    opcode = 0b111;
  else if (count == 2)
    opcode = 0b1010;
  else if (count == 3)
    opcode = 0b0110;
  else if (count == 4)
    opcode = 0b0010;
  EmitLoadStoreMultipleStructure(size, 1, opcode, Rt, Rn);
}
void LD1(u8 size, u8 count, IndexType type, ARM64Reg Rt, ARM64Reg Rn,
                            ARM64Reg Rm)
{
  _assert_msg_(DYNA_REC, !(count == 0 || count > 4), "%s must have a count of 1 to 4 registers!",
               __FUNCTION__);
  _assert_msg_(DYNA_REC, type == INDEX_POST, "%s only supports post indexing!", __FUNCTION__);

  u32 opcode = 0;
  if (count == 1)
    opcode = 0b111;
  else if (count == 2)
    opcode = 0b1010;
  else if (count == 3)
    opcode = 0b0110;
  else if (count == 4)
    opcode = 0b0010;
  EmitLoadStoreMultipleStructurePost(size, 1, opcode, Rt, Rn, Rm);
}
void ST1(u8 size, u8 count, ARM64Reg Rt, ARM64Reg Rn)
{
  _assert_msg_(DYNA_REC, !(count == 0 || count > 4), "%s must have a count of 1 to 4 registers!",
               __FUNCTION__);
  u32 opcode = 0;
  if (count == 1)
    opcode = 0b111;
  else if (count == 2)
    opcode = 0b1010;
  else if (count == 3)
    opcode = 0b0110;
  else if (count == 4)
    opcode = 0b0010;
  EmitLoadStoreMultipleStructure(size, 0, opcode, Rt, Rn);
}
void ST1(u8 size, u8 count, IndexType type, ARM64Reg Rt, ARM64Reg Rn,
                            ARM64Reg Rm)
{
  _assert_msg_(DYNA_REC, !(count == 0 || count > 4), "%s must have a count of 1 to 4 registers!",
               __FUNCTION__);
  _assert_msg_(DYNA_REC, type == INDEX_POST, "%s only supports post indexing!", __FUNCTION__);

  u32 opcode = 0;
  if (count == 1)
    opcode = 0b111;
  else if (count == 2)
    opcode = 0b1010;
  else if (count == 3)
    opcode = 0b0110;
  else if (count == 4)
    opcode = 0b0010;
  EmitLoadStoreMultipleStructurePost(size, 0, opcode, Rt, Rn, Rm);
}

// Scalar - 1 Source
void FMOV(ARM64Reg Rd, ARM64Reg Rn, bool top)
{
  if (IsScalar(Rd) && IsScalar(Rn))
  {
    EmitScalar1Source(0, 0, IsDouble(Rd), 0, Rd, Rn);
  }
  else
  {
    _assert_msg_(DYNA_REC, !IsQuad(Rd) && !IsQuad(Rn), "FMOV can't move to/from quads");
    int rmode = 0;
    int opcode = 6;
    int sf = 0;
    if (IsSingle(Rd) && !Is64Bit(Rn) && !top)
    {
      // GPR to scalar single
      opcode |= 1;
    }
    else if (!Is64Bit(Rd) && IsSingle(Rn) && !top)
    {
      // Scalar single to GPR - defaults are correct
    }
    else
    {
      // TODO
      _assert_msg_(DYNA_REC, 0, "FMOV: Unhandled case");
    }
    Rd = DecodeReg(Rd);
    Rn = DecodeReg(Rn);
    Write32((sf << 31) | (0x1e2 << 20) | (rmode << 19) | (opcode << 16) | (Rn << 5) | Rd);
  }
}

// Loadstore paired
void LDP(u8 size, IndexType type, ARM64Reg Rt, ARM64Reg Rt2, ARM64Reg Rn,
                            s32 imm)
{
  EncodeLoadStorePair(size, true, type, Rt, Rt2, Rn, imm);
}
void STP(u8 size, IndexType type, ARM64Reg Rt, ARM64Reg Rt2, ARM64Reg Rn,
                            s32 imm)
{
  EncodeLoadStorePair(size, false, type, Rt, Rt2, Rn, imm);
}

// Loadstore register offset
void STR(u8 size, ARM64Reg Rt, ARM64Reg Rn, ArithOption Rm)
{
  EncodeLoadStoreRegisterOffset(size, false, Rt, Rn, Rm);
}
void LDR(u8 size, ARM64Reg Rt, ARM64Reg Rn, ArithOption Rm)
{
  EncodeLoadStoreRegisterOffset(size, true, Rt, Rn, Rm);
}

void FABS(ARM64Reg Rd, ARM64Reg Rn)
{
  EmitScalar1Source(0, 0, IsDouble(Rd), 1, Rd, Rn);
}
void FNEG(ARM64Reg Rd, ARM64Reg Rn)
{
  EmitScalar1Source(0, 0, IsDouble(Rd), 2, Rd, Rn);
}
void FSQRT(ARM64Reg Rd, ARM64Reg Rn)
{
  EmitScalar1Source(0, 0, IsDouble(Rd), 3, Rd, Rn);
}

// Scalar - 2 Source
void FADD(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitScalar2Source(0, 0, IsDouble(Rd), 2, Rd, Rn, Rm);
}
void FMUL(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitScalar2Source(0, 0, IsDouble(Rd), 0, Rd, Rn, Rm);
}
void FSUB(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitScalar2Source(0, 0, IsDouble(Rd), 3, Rd, Rn, Rm);
}
void FDIV(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitScalar2Source(0, 0, IsDouble(Rd), 1, Rd, Rn, Rm);
}
void FMAX(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitScalar2Source(0, 0, IsDouble(Rd), 4, Rd, Rn, Rm);
}
void FMIN(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitScalar2Source(0, 0, IsDouble(Rd), 5, Rd, Rn, Rm);
}
void FMAXNM(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitScalar2Source(0, 0, IsDouble(Rd), 6, Rd, Rn, Rm);
}
void FMINNM(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitScalar2Source(0, 0, IsDouble(Rd), 7, Rd, Rn, Rm);
}
void FNMUL(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitScalar2Source(0, 0, IsDouble(Rd), 8, Rd, Rn, Rm);
}

void FMADD(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ARM64Reg Ra)
{
  EmitScalar3Source(IsDouble(Rd), Rd, Rn, Rm, Ra, 0);
}
void FMSUB(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ARM64Reg Ra)
{
  EmitScalar3Source(IsDouble(Rd), Rd, Rn, Rm, Ra, 1);
}
void FNMADD(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ARM64Reg Ra)
{
  EmitScalar3Source(IsDouble(Rd), Rd, Rn, Rm, Ra, 2);
}
void FNMSUB(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, ARM64Reg Ra)
{
  EmitScalar3Source(IsDouble(Rd), Rd, Rn, Rm, Ra, 3);
}

void EmitScalar3Source(bool isDouble, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm,
                                          ARM64Reg Ra, int opcode)
{
  int type = isDouble ? 1 : 0;
  Rd = DecodeReg(Rd);
  Rn = DecodeReg(Rn);
  Rm = DecodeReg(Rm);
  Ra = DecodeReg(Ra);
  int o1 = opcode >> 1;
  int o0 = opcode & 1;
  m_emit->Write32((0x1F << 24) | (type << 22) | (o1 << 21) | (Rm << 16) | (o0 << 15) | (Ra << 10) |
                  (Rn << 5) | Rd);
}

// Scalar floating point immediate
void FMOV(ARM64Reg Rd, uint8_t imm8)
{
  EmitScalarImm(0, 0, 0, 0, Rd, imm8);
}

// Vector
void AND(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitThreeSame(0, 0, 3, Rd, Rn, Rm);
}
void BSL(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitThreeSame(1, 1, 3, Rd, Rn, Rm);
}
void DUP(u8 size, ARM64Reg Rd, ARM64Reg Rn, u8 index)
{
  u32 imm5 = 0;

  if (size == 8)
  {
    imm5 = 1;
    imm5 |= index << 1;
  }
  else if (size == 16)
  {
    imm5 = 2;
    imm5 |= index << 2;
  }
  else if (size == 32)
  {
    imm5 = 4;
    imm5 |= index << 3;
  }
  else if (size == 64)
  {
    imm5 = 8;
    imm5 |= index << 4;
  }

  EmitCopy(IsQuad(Rd), 0, imm5, 0, Rd, Rn);
}
void FABS(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 0, 2 | (size >> 6), 0xF, Rd, Rn);
}
void FADD(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitThreeSame(0, size >> 6, 0x1A, Rd, Rn, Rm);
}
void FMAX(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitThreeSame(0, size >> 6, 0b11110, Rd, Rn, Rm);
}
void FMLA(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitThreeSame(0, size >> 6, 0x19, Rd, Rn, Rm);
}
void FMIN(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitThreeSame(0, 2 | size >> 6, 0b11110, Rd, Rn, Rm);
}
void FCVTL(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(false, 0, size >> 6, 0x17, Rd, Rn);
}
void FCVTL2(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(true, 0, size >> 6, 0x17, Rd, Rn);
}
void FCVTN(u8 dest_size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 0, dest_size >> 5, 0x16, Rd, Rn);
}
void FCVTZS(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 0, 2 | (size >> 6), 0x1B, Rd, Rn);
}
void FCVTZU(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 1, 2 | (size >> 6), 0x1B, Rd, Rn);
}
void FDIV(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitThreeSame(1, size >> 6, 0x1F, Rd, Rn, Rm);
}
void FMUL(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitThreeSame(1, size >> 6, 0x1B, Rd, Rn, Rm);
}
void FNEG(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 1, 2 | (size >> 6), 0xF, Rd, Rn);
}
void FRSQRTE(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 1, 2 | (size >> 6), 0x1D, Rd, Rn);
}
void FSUB(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitThreeSame(0, 2 | (size >> 6), 0x1A, Rd, Rn, Rm);
}
void FMLS(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitThreeSame(0, 2 | (size >> 6), 0x19, Rd, Rn, Rm);
}
void NOT(ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 1, 0, 5, Rd, Rn);
}
void ORR(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitThreeSame(0, 2, 3, Rd, Rn, Rm);
}
void REV16(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 0, size >> 4, 1, Rd, Rn);
}
void REV32(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 1, size >> 4, 0, Rd, Rn);
}
void REV64(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 0, size >> 4, 0, Rd, Rn);
}
void SCVTF(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 0, size >> 6, 0x1D, Rd, Rn);
}
void UCVTF(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 1, size >> 6, 0x1D, Rd, Rn);
}
void SCVTF(u8 size, ARM64Reg Rd, ARM64Reg Rn, int scale)
{
  int imm = size * 2 - scale;
  EmitShiftImm(IsQuad(Rd), 0, imm >> 3, imm & 7, 0x1C, Rd, Rn);
}
void UCVTF(u8 size, ARM64Reg Rd, ARM64Reg Rn, int scale)
{
  int imm = size * 2 - scale;
  EmitShiftImm(IsQuad(Rd), 1, imm >> 3, imm & 7, 0x1C, Rd, Rn);
}
void SQXTN(u8 dest_size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(false, 0, dest_size >> 4, 0b10100, Rd, Rn);
}
void SQXTN2(u8 dest_size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(true, 0, dest_size >> 4, 0b10100, Rd, Rn);
}
void UQXTN(u8 dest_size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(false, 1, dest_size >> 4, 0b10100, Rd, Rn);
}
void UQXTN2(u8 dest_size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(true, 1, dest_size >> 4, 0b10100, Rd, Rn);
}
void XTN(u8 dest_size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(false, 0, dest_size >> 4, 0b10010, Rd, Rn);
}
void XTN2(u8 dest_size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(true, 0, dest_size >> 4, 0b10010, Rd, Rn);
}

// Move
void DUP(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  u32 imm5 = 0;

  if (size == 8)
    imm5 = 1;
  else if (size == 16)
    imm5 = 2;
  else if (size == 32)
    imm5 = 4;
  else if (size == 64)
    imm5 = 8;

  EmitCopy(IsQuad(Rd), 0, imm5, 1, Rd, Rn);
}
void INS(u8 size, ARM64Reg Rd, u8 index, ARM64Reg Rn)
{
  u32 imm5 = 0;

  if (size == 8)
  {
    imm5 = 1;
    imm5 |= index << 1;
  }
  else if (size == 16)
  {
    imm5 = 2;
    imm5 |= index << 2;
  }
  else if (size == 32)
  {
    imm5 = 4;
    imm5 |= index << 3;
  }
  else if (size == 64)
  {
    imm5 = 8;
    imm5 |= index << 4;
  }

  EmitCopy(1, 0, imm5, 3, Rd, Rn);
}
void INS(u8 size, ARM64Reg Rd, u8 index1, ARM64Reg Rn, u8 index2)
{
  u32 imm5 = 0, imm4 = 0;

  if (size == 8)
  {
    imm5 = 1;
    imm5 |= index1 << 1;
    imm4 = index2;
  }
  else if (size == 16)
  {
    imm5 = 2;
    imm5 |= index1 << 2;
    imm4 = index2 << 1;
  }
  else if (size == 32)
  {
    imm5 = 4;
    imm5 |= index1 << 3;
    imm4 = index2 << 2;
  }
  else if (size == 64)
  {
    imm5 = 8;
    imm5 |= index1 << 4;
    imm4 = index2 << 3;
  }

  EmitCopy(1, 1, imm5, imm4, Rd, Rn);
}

void UMOV(u8 size, ARM64Reg Rd, ARM64Reg Rn, u8 index)
{
  bool b64Bit = Is64Bit(Rd);
  _assert_msg_(DYNA_REC, Rd < SP, "%s destination must be a GPR!", __FUNCTION__);
  _assert_msg_(DYNA_REC, !(b64Bit && size != 64),
               "%s must have a size of 64 when destination is 64bit!", __FUNCTION__);
  u32 imm5 = 0;

  if (size == 8)
  {
    imm5 = 1;
    imm5 |= index << 1;
  }
  else if (size == 16)
  {
    imm5 = 2;
    imm5 |= index << 2;
  }
  else if (size == 32)
  {
    imm5 = 4;
    imm5 |= index << 3;
  }
  else if (size == 64)
  {
    imm5 = 8;
    imm5 |= index << 4;
  }

  EmitCopy(b64Bit, 0, imm5, 7, Rd, Rn);
}
void SMOV(u8 size, ARM64Reg Rd, ARM64Reg Rn, u8 index)
{
  bool b64Bit = Is64Bit(Rd);
  _assert_msg_(DYNA_REC, Rd < SP, "%s destination must be a GPR!", __FUNCTION__);
  _assert_msg_(DYNA_REC, size != 64, "%s doesn't support 64bit destination. Use UMOV!",
               __FUNCTION__);
  u32 imm5 = 0;

  if (size == 8)
  {
    imm5 = 1;
    imm5 |= index << 1;
  }
  else if (size == 16)
  {
    imm5 = 2;
    imm5 |= index << 2;
  }
  else if (size == 32)
  {
    imm5 = 4;
    imm5 |= index << 3;
  }

  EmitCopy(b64Bit, 0, imm5, 5, Rd, Rn);
}

// One source
void FCVT(u8 size_to, u8 size_from, ARM64Reg Rd, ARM64Reg Rn)
{
  u32 dst_encoding = 0;
  u32 src_encoding = 0;

  if (size_to == 16)
    dst_encoding = 3;
  else if (size_to == 32)
    dst_encoding = 0;
  else if (size_to == 64)
    dst_encoding = 1;

  if (size_from == 16)
    src_encoding = 3;
  else if (size_from == 32)
    src_encoding = 0;
  else if (size_from == 64)
    src_encoding = 1;

  Emit1Source(0, 0, src_encoding, 4 | dst_encoding, Rd, Rn);
}

void SCVTF(ARM64Reg Rd, ARM64Reg Rn)
{
  if (IsScalar(Rn))
  {
    // Source is in FP register (like destination!). We must use a vector encoding.
    bool sign = false;
    Rd = DecodeReg(Rd);
    Rn = DecodeReg(Rn);
    int sz = IsDouble(Rn);
    Write32((0x5e << 24) | (sign << 29) | (sz << 22) | (0x876 << 10) | (Rn << 5) | Rd);
  }
  else
  {
    bool sf = Is64Bit(Rn);
    u32 type = 0;
    if (IsDouble(Rd))
      type = 1;
    EmitConversion(sf, 0, type, 0, 2, Rd, Rn);
  }
}

void UCVTF(ARM64Reg Rd, ARM64Reg Rn)
{
  if (IsScalar(Rn))
  {
    // Source is in FP register (like destination!). We must use a vector encoding.
    bool sign = true;
    Rd = DecodeReg(Rd);
    Rn = DecodeReg(Rn);
    int sz = IsDouble(Rn);
    Write32((0x5e << 24) | (sign << 29) | (sz << 22) | (0x876 << 10) | (Rn << 5) | Rd);
  }
  else
  {
    bool sf = Is64Bit(Rn);
    u32 type = 0;
    if (IsDouble(Rd))
      type = 1;

    EmitConversion(sf, 0, type, 0, 3, Rd, Rn);
  }
}

void SCVTF(ARM64Reg Rd, ARM64Reg Rn, int scale)
{
  bool sf = Is64Bit(Rn);
  u32 type = 0;
  if (IsDouble(Rd))
    type = 1;

  EmitConversion2(sf, 0, false, type, 0, 2, 64 - scale, Rd, Rn);
}

void UCVTF(ARM64Reg Rd, ARM64Reg Rn, int scale)
{
  bool sf = Is64Bit(Rn);
  u32 type = 0;
  if (IsDouble(Rd))
    type = 1;

  EmitConversion2(sf, 0, false, type, 0, 3, 64 - scale, Rd, Rn);
}

void FCMP(ARM64Reg Rn, ARM64Reg Rm)
{
  EmitCompare(0, 0, 0, 0, Rn, Rm);
}
void FCMP(ARM64Reg Rn)
{
  EmitCompare(0, 0, 0, 8, Rn, (ARM64Reg)0);
}
void FCMPE(ARM64Reg Rn, ARM64Reg Rm)
{
  EmitCompare(0, 0, 0, 0x10, Rn, Rm);
}
void FCMPE(ARM64Reg Rn)
{
  EmitCompare(0, 0, 0, 0x18, Rn, (ARM64Reg)0);
}
void FCMEQ(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitThreeSame(0, size >> 6, 0x1C, Rd, Rn, Rm);
}
void FCMEQ(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 0, 2 | (size >> 6), 0xD, Rd, Rn);
}
void FCMGE(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitThreeSame(1, size >> 6, 0x1C, Rd, Rn, Rm);
}
void FCMGE(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 1, 2 | (size >> 6), 0x0C, Rd, Rn);
}
void FCMGT(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitThreeSame(1, 2 | (size >> 6), 0x1C, Rd, Rn, Rm);
}
void FCMGT(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 0, 2 | (size >> 6), 0x0C, Rd, Rn);
}
void FCMLE(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 1, 2 | (size >> 6), 0xD, Rd, Rn);
}
void FCMLT(u8 size, ARM64Reg Rd, ARM64Reg Rn)
{
  Emit2RegMisc(IsQuad(Rd), 0, 2 | (size >> 6), 0xE, Rd, Rn);
}

void FCSEL(ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, CCFlags cond)
{
  EmitCondSelect(0, 0, cond, Rd, Rn, Rm);
}

// Permute
void UZP1(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitPermute(size, 0b001, Rd, Rn, Rm);
}
void TRN1(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitPermute(size, 0b010, Rd, Rn, Rm);
}
void ZIP1(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitPermute(size, 0b011, Rd, Rn, Rm);
}
void UZP2(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitPermute(size, 0b101, Rd, Rn, Rm);
}
void TRN2(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitPermute(size, 0b110, Rd, Rn, Rm);
}
void ZIP2(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm)
{
  EmitPermute(size, 0b111, Rd, Rn, Rm);
}

// Shift by immediate
void SSHLL(u8 src_size, ARM64Reg Rd, ARM64Reg Rn, u32 shift)
{
  SSHLL(src_size, Rd, Rn, shift, false);
}
void SSHLL2(u8 src_size, ARM64Reg Rd, ARM64Reg Rn, u32 shift)
{
  SSHLL(src_size, Rd, Rn, shift, true);
}
void SHRN(u8 dest_size, ARM64Reg Rd, ARM64Reg Rn, u32 shift)
{
  SHRN(dest_size, Rd, Rn, shift, false);
}
void SHRN2(u8 dest_size, ARM64Reg Rd, ARM64Reg Rn, u32 shift)
{
  SHRN(dest_size, Rd, Rn, shift, true);
}
void USHLL(u8 src_size, ARM64Reg Rd, ARM64Reg Rn, u32 shift)
{
  USHLL(src_size, Rd, Rn, shift, false);
}
void USHLL2(u8 src_size, ARM64Reg Rd, ARM64Reg Rn, u32 shift)
{
  USHLL(src_size, Rd, Rn, shift, true);
}
void SXTL(u8 src_size, ARM64Reg Rd, ARM64Reg Rn)
{
  SXTL(src_size, Rd, Rn, false);
}
void SXTL2(u8 src_size, ARM64Reg Rd, ARM64Reg Rn)
{
  SXTL(src_size, Rd, Rn, true);
}
void UXTL(u8 src_size, ARM64Reg Rd, ARM64Reg Rn)
{
  UXTL(src_size, Rd, Rn, false);
}
void UXTL2(u8 src_size, ARM64Reg Rd, ARM64Reg Rn)
{
  UXTL(src_size, Rd, Rn, true);
}

void SSHLL(u8 src_size, ARM64Reg Rd, ARM64Reg Rn, u32 shift, bool upper)
{
  _assert_msg_(DYNA_REC, shift < src_size, "%s shift amount must less than the element size!",
               __FUNCTION__);
  u32 immh = 0;
  u32 immb = shift & 0xFFF;

  if (src_size == 8)
  {
    immh = 1;
  }
  else if (src_size == 16)
  {
    immh = 2 | ((shift >> 3) & 1);
  }
  else if (src_size == 32)
  {
    immh = 4 | ((shift >> 3) & 3);
    ;
  }
  EmitShiftImm(upper, 0, immh, immb, 0b10100, Rd, Rn);
}

void USHLL(u8 src_size, ARM64Reg Rd, ARM64Reg Rn, u32 shift, bool upper)
{
  _assert_msg_(DYNA_REC, shift < src_size, "%s shift amount must less than the element size!",
               __FUNCTION__);
  u32 immh = 0;
  u32 immb = shift & 0xFFF;

  if (src_size == 8)
  {
    immh = 1;
  }
  else if (src_size == 16)
  {
    immh = 2 | ((shift >> 3) & 1);
  }
  else if (src_size == 32)
  {
    immh = 4 | ((shift >> 3) & 3);
    ;
  }
  EmitShiftImm(upper, 1, immh, immb, 0b10100, Rd, Rn);
}

void SHRN(u8 dest_size, ARM64Reg Rd, ARM64Reg Rn, u32 shift, bool upper)
{
  _assert_msg_(DYNA_REC, shift < dest_size, "%s shift amount must less than the element size!",
               __FUNCTION__);
  u32 immh = 0;
  u32 immb = shift & 0xFFF;

  if (dest_size == 8)
  {
    immh = 1;
  }
  else if (dest_size == 16)
  {
    immh = 2 | ((shift >> 3) & 1);
  }
  else if (dest_size == 32)
  {
    immh = 4 | ((shift >> 3) & 3);
    ;
  }
  EmitShiftImm(upper, 1, immh, immb, 0b10000, Rd, Rn);
}

void SXTL(u8 src_size, ARM64Reg Rd, ARM64Reg Rn, bool upper)
{
  SSHLL(src_size, Rd, Rn, 0, upper);
}

void UXTL(u8 src_size, ARM64Reg Rd, ARM64Reg Rn, bool upper)
{
  USHLL(src_size, Rd, Rn, 0, upper);
}

// vector x indexed element
void FMUL(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, u8 index)
{
  _assert_msg_(DYNA_REC, size == 32 || size == 64, "%s only supports 32bit or 64bit size!",
               __FUNCTION__);

  bool L = false;
  bool H = false;
  if (size == 32)
  {
    L = index & 1;
    H = (index >> 1) & 1;
  }
  else if (size == 64)
  {
    H = index == 1;
  }

  EmitVectorxElement(0, 2 | (size >> 6), L, 0x9, H, Rd, Rn, Rm);
}

void FMLA(u8 size, ARM64Reg Rd, ARM64Reg Rn, ARM64Reg Rm, u8 index)
{
  _assert_msg_(DYNA_REC, size == 32 || size == 64, "%s only supports 32bit or 64bit size!",
               __FUNCTION__);

  bool L = false;
  bool H = false;
  if (size == 32)
  {
    L = index & 1;
    H = (index >> 1) & 1;
  }
  else if (size == 64)
  {
    H = index == 1;
  }

  EmitVectorxElement(0, 2 | (size >> 6), L, 1, H, Rd, Rn, Rm);
}

// Modified Immediate
void MOVI(u8 size, ARM64Reg Rd, u64 imm, u8 shift)
{
  bool Q = IsQuad(Rd);
  u8 cmode = 0;
  u8 op = 0;
  u8 abcdefgh = imm & 0xFF;
  if (size == 8)
  {
    _assert_msg_(DYNA_REC, shift == 0, "%s(size8) doesn't support shift!", __FUNCTION__);
    _assert_msg_(DYNA_REC, !(imm & ~0xFFULL), "%s(size8) only supports 8bit values!", __FUNCTION__);
  }
  else if (size == 16)
  {
    _assert_msg_(DYNA_REC, shift == 0 || shift == 8, "%s(size16) only supports shift of {0, 8}!",
                 __FUNCTION__);
    _assert_msg_(DYNA_REC, !(imm & ~0xFFULL), "%s(size16) only supports 8bit values!",
                 __FUNCTION__);

    if (shift == 8)
      cmode |= 2;
  }
  else if (size == 32)
  {
    _assert_msg_(DYNA_REC, shift == 0 || shift == 8 || shift == 16 || shift == 24,
                 "%s(size32) only supports shift of {0, 8, 16, 24}!", __FUNCTION__);
    // XXX: Implement support for MOVI - shifting ones variant
    _assert_msg_(DYNA_REC, !(imm & ~0xFFULL), "%s(size32) only supports 8bit values!",
                 __FUNCTION__);
    switch (shift)
    {
    case 8:
      cmode |= 2;
      break;
    case 16:
      cmode |= 4;
      break;
    case 24:
      cmode |= 6;
      break;
    default:
      break;
    }
  }
  else  // 64
  {
    _assert_msg_(DYNA_REC, shift == 0, "%s(size64) doesn't support shift!", __FUNCTION__);

    op = 1;
    cmode = 0xE;
    abcdefgh = 0;
    for (int i = 0; i < 8; ++i)
    {
      u8 tmp = (imm >> (i << 3)) & 0xFF;
      _assert_msg_(DYNA_REC, tmp == 0xFF || tmp == 0, "%s(size64) Invalid immediate!",
                   __FUNCTION__);
      if (tmp == 0xFF)
        abcdefgh |= (1 << i);
    }
  }
  EncodeModImm(Q, op, cmode, 0, Rd, abcdefgh);
}

void BIC(u8 size, ARM64Reg Rd, u8 imm, u8 shift)
{
  bool Q = IsQuad(Rd);
  u8 cmode = 1;
  u8 op = 1;
  if (size == 16)
  {
    _assert_msg_(DYNA_REC, shift == 0 || shift == 8, "%s(size16) only supports shift of {0, 8}!",
                 __FUNCTION__);

    if (shift == 8)
      cmode |= 2;
  }
  else if (size == 32)
  {
    _assert_msg_(DYNA_REC, shift == 0 || shift == 8 || shift == 16 || shift == 24,
                 "%s(size32) only supports shift of {0, 8, 16, 24}!", __FUNCTION__);
    // XXX: Implement support for MOVI - shifting ones variant
    switch (shift)
    {
    case 8:
      cmode |= 2;
      break;
    case 16:
      cmode |= 4;
      break;
    case 24:
      cmode |= 6;
      break;
    default:
      break;
    }
  }
  else
    _assert_msg_(DYNA_REC, false, "%s only supports size of {16, 32}!", __FUNCTION__);
  EncodeModImm(Q, op, cmode, 0, Rd, imm);
}

//TODO: see if there's a better way to mix these in with the rest of the emitter
void ADDP(Arm64Reg rd, Arm64Reg rn, Arm64Reg rm, simd_size_e simd_size)
{
    assert(size != simd_size_e::invalid);
    rd = DecodeReg(rd);
    rn = DecodeReg(rn);
    rm = DecodeReg(rm);
    u32 Q = (static_cast<u32>(simd_size) & 1) << 30;
    u32 size = (static_cast<u32>(simd_size) & 3) << 22;
    Write32(Q << 30 | size << 22 | rm << 16 | rn << 5 | rd << 0 | 0x0e20bc00);
}

void SUBP(Arm64Reg rd, Arm64Reg rn, Arm64Reg rm, simd_size_e simd_size)
{
    assert(size != simd_size_e::invalid);
    rd = DecodeReg(rd);
    rn = DecodeReg(rn);
    rm = DecodeReg(rm);
    u32 Q = (static_cast<u32>(simd_size) & 1) << 30;
    u32 size = (static_cast<u32>(simd_size) & 3) << 22;
    Write32(Q << 30 | size << 22 | rm << 16 | rn << 5 | rd << 0 | 0x2e208400);
}

void SQADD(Arm64Reg rd, Arm64Reg rn, Arm64Reg rm, simd_size_e simd_size)
{
    assert(size != simd_size_e::invalid);
    rd = DecodeReg(rd);
    rn = DecodeReg(rn);
    rm = DecodeReg(rm);
    u32 Q = (static_cast<u32>(simd_size) & 1) << 30;
    u32 size = (static_cast<u32>(simd_size) & 3) << 22;
    Write32(Q << 30 | size << 22 | rm << 16 | rn << 5 | rd << 0 | 0x5e200c00);
}

void SQSUB(Arm64Reg rd, Arm64Reg rn, Arm64Reg rm, simd_size_e simd_size)
{
    assert(size != simd_size_e::invalid);
    rd = DecodeReg(rd);
    rn = DecodeReg(rn);
    rm = DecodeReg(rm);
    u32 Q = (static_cast<u32>(simd_size) & 1) << 30;
    u32 size = (static_cast<u32>(simd_size) & 3) << 22;
    Write32(Q << 30 | size << 22 | rm << 16 | rn << 5 | rd << 0 | 0x0e202c00);
}

void UQADD(Arm64Reg rd, Arm64Reg rn, Arm64Reg rm, simd_size_e simd_size)
{
    assert(size != simd_size_e::invalid);
    rd = DecodeReg(rd);
    rn = DecodeReg(rn);
    rm = DecodeReg(rm);
    u32 Q = (static_cast<u32>(simd_size) & 1) << 30;
    u32 size = (static_cast<u32>(simd_size) & 3) << 22;
    Write32(Q << 30 | size << 22 | rm << 16 | rn << 5 | rd << 0 | 0x2e200c00);
}

void UQSUB(Arm64Reg rd, Arm64Reg rn, Arm64Reg rm, simd_size_e simd_size)
{
    assert(size != simd_size_e::invalid);
    rd = DecodeReg(rd);
    rn = DecodeReg(rn);
    rm = DecodeReg(rm);
    u32 Q = (static_cast<u32>(simd_size) & 1) << 30;
    u32 size = (static_cast<u32>(simd_size) & 3) << 22;
    Write32(Q << 30 | size << 22 | rm << 16 | rn << 5 | rd << 0 | 0x2e202c00);
}

void ORR(ARM64Reg rd, ARM64Reg rn, ARM64Reg rm, simd_size_e simd_size)
{
    assert(size == simd_size_e::bytes_8 || size == simd_size_e::bytes_16);
    rd = DecodeReg(rd);
    rn = DecodeReg(rn);
    rm = DecodeReg(rm);
    u32 Q = (static_cast<u32>(simd_size) & 1) << 30;
    Write32(Q << 30  | rm << 16 | rn << 5 | rd << 0 | 0x0ea01c00);
}

void ORN(ARM64Reg rd, ARM64Reg rn, ARM64Reg rm, simd_size_e simd_size)
{
    assert(size == simd_size_e::bytes_8 || size == simd_size_e::bytes_16);
    rd = DecodeReg(rd);
    rn = DecodeReg(rn);
    rm = DecodeReg(rm);
    u32 Q = (static_cast<u32>(simd_size) & 1) << 30;
    Write32(Q << 30  | rm << 16 | rn << 5 | rd << 0 | 0x0ee01c00);
}

void AND(ARM64Reg rd, ARM64Reg rn, ARM64Reg rm, simd_size_e simd_size)
{
    assert(size == simd_size_e::bytes_8 || size == simd_size_e::bytes_16);
    rd = DecodeReg(rd);
    rn = DecodeReg(rn);
    rm = DecodeReg(rm);
    u32 Q = (static_cast<u32>(simd_size) & 1) << 30;
    Write32(Q << 30  | rm << 16 | rn << 5 | rd << 0 | 0x0e201c00);
}


void EOR(ARM64Reg rd, ARM64Reg rn, ARM64Reg rm, simd_size_e simd_size)
{
    assert(size == simd_size_e::bytes_8 || size == simd_size_e::bytes_16);
    rd = DecodeReg(rd);
    rn = DecodeReg(rn);
    rm = DecodeReg(rm);
    u32 Q = (static_cast<u32>(simd_size) & 1) << 30;
    Write32(Q << 30  | rm << 16 | rn << 5 | rd << 0 | 0x2e201c00);
}



void MOV(ARM64Reg rd, ARM64Reg rn, simd_size_e simd_size)
{
    ORR(rd,rn,rn,simd_size);
}

void ABI_PushRegisters(BitSet32 registers, ARM64Reg tmp)
{
  bool bundled_loadstore = false;

  for (int i = 0; i < 32; ++i)
  {
    if (!registers[i])
      continue;

    int count = 0;
    while (++count < 4 && (i + count) < 32 && registers[i + count])
    {
    }
    if (count > 1)
    {
      bundled_loadstore = true;
      break;
    }
  }

  if (bundled_loadstore && tmp != INVALID_REG)
  {
    int num_regs = registers.Count();
    m_emit->SUB(SP, SP, num_regs * 16);
    m_emit->ADD(tmp, SP, 0);
    std::vector<ARM64Reg> island_regs;
    for (int i = 0; i < 32; ++i)
    {
      if (!registers[i])
        continue;

      int count = 0;

      // 0 = true
      // 1 < 4 && registers[i + 1] true!
      // 2 < 4 && registers[i + 2] true!
      // 3 < 4 && registers[i + 3] true!
      // 4 < 4 && registers[i + 4] false!
      while (++count < 4 && (i + count) < 32 && registers[i + count])
      {
      }

      if (count == 1)
        island_regs.push_back((ARM64Reg)(Q0 + i));
      else
        ST1(64, count, INDEX_POST, (ARM64Reg)(Q0 + i), tmp);

      i += count - 1;
    }

    // Handle island registers
    std::vector<ARM64Reg> pair_regs;
    for (auto& it : island_regs)
    {
      pair_regs.push_back(it);
      if (pair_regs.size() == 2)
      {
        STP(128, INDEX_POST, pair_regs[0], pair_regs[1], tmp, 32);
        pair_regs.clear();
      }
    }
    if (pair_regs.size())
      STR(128, INDEX_POST, pair_regs[0], tmp, 16);
  }
  else
  {
    std::vector<ARM64Reg> pair_regs;
    for (auto it : registers)
    {
      pair_regs.push_back((ARM64Reg)(Q0 + it));
      if (pair_regs.size() == 2)
      {
        STP(128, INDEX_PRE, pair_regs[0], pair_regs[1], SP, -32);
        pair_regs.clear();
      }
    }
    if (pair_regs.size())
      STR(128, INDEX_PRE, pair_regs[0], SP, -16);
  }
}
void ABI_PopRegisters(BitSet32 registers, ARM64Reg tmp)
{
  bool bundled_loadstore = false;
  int num_regs = registers.Count();

  for (int i = 0; i < 32; ++i)
  {
    if (!registers[i])
      continue;

    int count = 0;
    while (++count < 4 && (i + count) < 32 && registers[i + count])
    {
    }
    if (count > 1)
    {
      bundled_loadstore = true;
      break;
    }
  }

  if (bundled_loadstore && tmp != INVALID_REG)
  {
    // The temporary register is only used to indicate that we can use this code path
    std::vector<ARM64Reg> island_regs;
    for (int i = 0; i < 32; ++i)
    {
      if (!registers[i])
        continue;

      int count = 0;
      while (++count < 4 && (i + count) < 32 && registers[i + count])
      {
      }

      if (count == 1)
        island_regs.push_back((ARM64Reg)(Q0 + i));
      else
        LD1(64, count, INDEX_POST, (ARM64Reg)(Q0 + i), SP);

      i += count - 1;
    }

    // Handle island registers
    std::vector<ARM64Reg> pair_regs;
    for (auto& it : island_regs)
    {
      pair_regs.push_back(it);
      if (pair_regs.size() == 2)
      {
        LDP(128, INDEX_POST, pair_regs[0], pair_regs[1], SP, 32);
        pair_regs.clear();
      }
    }
    if (pair_regs.size())
      LDR(128, INDEX_POST, pair_regs[0], SP, 16);
  }
  else
  {
    bool odd = num_regs % 2;
    std::vector<ARM64Reg> pair_regs;
    for (int i = 31; i >= 0; --i)
    {
      if (!registers[i])
        continue;

      if (odd)
      {
        // First load must be a regular LDR if odd
        odd = false;
        LDR(128, INDEX_POST, (ARM64Reg)(Q0 + i), SP, 16);
      }
      else
      {
        pair_regs.push_back((ARM64Reg)(Q0 + i));
        if (pair_regs.size() == 2)
        {
          LDP(128, INDEX_POST, pair_regs[1], pair_regs[0], SP, 32);
          pair_regs.clear();
        }
      }
    }
  }
}

void ANDI2R(ARM64Reg Rd, ARM64Reg Rn, u64 imm, ARM64Reg scratch)
{
  unsigned int n, imm_s, imm_r;
  if (!Is64Bit(Rn))
    imm &= 0xFFFFFFFF;
  if (IsImmLogical(imm, Is64Bit(Rn) ? 64 : 32, &n, &imm_s, &imm_r))
  {
    AND(Rd, Rn, imm_r, imm_s, n != 0);
  }
  else
  {
    _assert_msg_(DYNA_REC, scratch != INVALID_REG,
                 "ANDSI2R - failed to construct logical immediate value from %08x, need scratch",
                 (u32)imm);
    MOVI2R(scratch, imm);
    AND(Rd, Rn, scratch);
  }
}

void ORRI2R(ARM64Reg Rd, ARM64Reg Rn, u64 imm, ARM64Reg scratch)
{
  unsigned int n, imm_s, imm_r;
  if (IsImmLogical(imm, Is64Bit(Rn) ? 64 : 32, &n, &imm_s, &imm_r))
  {
    ORR(Rd, Rn, imm_r, imm_s, n != 0);
  }
  else
  {
    _assert_msg_(DYNA_REC, scratch != INVALID_REG,
                 "ORRI2R - failed to construct logical immediate value from %08x, need scratch",
                 (u32)imm);
    MOVI2R(scratch, imm);
    ORR(Rd, Rn, scratch);
  }
}

void EORI2R(ARM64Reg Rd, ARM64Reg Rn, u64 imm, ARM64Reg scratch)
{
  unsigned int n, imm_s, imm_r;
  if (IsImmLogical(imm, Is64Bit(Rn) ? 64 : 32, &n, &imm_s, &imm_r))
  {
    EOR(Rd, Rn, imm_r, imm_s, n != 0);
  }
  else
  {
    _assert_msg_(DYNA_REC, scratch != INVALID_REG,
                 "EORI2R - failed to construct logical immediate value from %08x, need scratch",
                 (u32)imm);
    MOVI2R(scratch, imm);
    EOR(Rd, Rn, scratch);
  }
}

void ANDSI2R(ARM64Reg Rd, ARM64Reg Rn, u64 imm, ARM64Reg scratch)
{
  unsigned int n, imm_s, imm_r;
  if (IsImmLogical(imm, Is64Bit(Rn) ? 64 : 32, &n, &imm_s, &imm_r))
  {
    ANDS(Rd, Rn, imm_r, imm_s, n != 0);
  }
  else
  {
    _assert_msg_(DYNA_REC, scratch != INVALID_REG,
                 "ANDSI2R - failed to construct logical immediate value from %08x, need scratch",
                 (u32)imm);
    MOVI2R(scratch, imm);
    ANDS(Rd, Rn, scratch);
  }
}

void ADDI2R(ARM64Reg Rd, ARM64Reg Rn, u64 imm, ARM64Reg scratch)
{
  u32 val;
  bool shift;
  if (IsImmArithmetic(imm, &val, &shift))
  {
    ADD(Rd, Rn, val, shift);
  }
  else
  {
    _assert_msg_(DYNA_REC, scratch != INVALID_REG,
                 "ADDI2R - failed to construct arithmetic immediate value from %08x, need scratch",
                 (u32)imm);
    MOVI2R(scratch, imm);
    ADD(Rd, Rn, scratch);
  }
}

void SUBI2R(ARM64Reg Rd, ARM64Reg Rn, u64 imm, ARM64Reg scratch)
{
  u32 val;
  bool shift;
  if (IsImmArithmetic(imm, &val, &shift))
  {
    SUB(Rd, Rn, val, shift);
  }
  else
  {
    _assert_msg_(DYNA_REC, scratch != INVALID_REG,
                 "SUBI2R - failed to construct arithmetic immediate value from %08x, need scratch",
                 (u32)imm);
    MOVI2R(scratch, imm);
    SUB(Rd, Rn, scratch);
  }
}

void CMPI2R(ARM64Reg Rn, u64 imm, ARM64Reg scratch)
{
  u32 val;
  bool shift;
  if (IsImmArithmetic(imm, &val, &shift))
  {
    CMP(Rn, val, shift);
  }
  else
  {
    _assert_msg_(DYNA_REC, scratch != INVALID_REG,
                 "CMPI2R - failed to construct arithmetic immediate value from %08x, need scratch",
                 (u32)imm);
    MOVI2R(scratch, imm);
    CMP(Rn, scratch);
  }
}

bool TryADDI2R(ARM64Reg Rd, ARM64Reg Rn, u32 imm)
{
  u32 val;
  bool shift;
  if (IsImmArithmetic(imm, &val, &shift))
    ADD(Rd, Rn, val, shift);
  else
    return false;

  return true;
}

bool TrySUBI2R(ARM64Reg Rd, ARM64Reg Rn, u32 imm)
{
  u32 val;
  bool shift;
  if (IsImmArithmetic(imm, &val, &shift))
    SUB(Rd, Rn, val, shift);
  else
    return false;

  return true;
}

bool TryCMPI2R(ARM64Reg Rn, u32 imm)
{
  u32 val;
  bool shift;
  if (IsImmArithmetic(imm, &val, &shift))
    CMP(Rn, val, shift);
  else
    return false;

  return true;
}

bool TryANDI2R(ARM64Reg Rd, ARM64Reg Rn, u32 imm)
{
  u32 n, imm_r, imm_s;
  if (IsImmLogical(imm, 32, &n, &imm_s, &imm_r))
    AND(Rd, Rn, imm_r, imm_s, n != 0);
  else
    return false;

  return true;
}
bool TryORRI2R(ARM64Reg Rd, ARM64Reg Rn, u32 imm)
{
  u32 n, imm_r, imm_s;
  if (IsImmLogical(imm, 32, &n, &imm_s, &imm_r))
    ORR(Rd, Rn, imm_r, imm_s, n != 0);
  else
    return false;

  return true;
}
bool TryEORI2R(ARM64Reg Rd, ARM64Reg Rn, u32 imm)
{
  u32 n, imm_r, imm_s;
  if (IsImmLogical(imm, 32, &n, &imm_s, &imm_r))
    EOR(Rd, Rn, imm_r, imm_s, n != 0);
  else
    return false;

  return true;
}

float FPImm8ToFloat(uint8_t bits)
{
  int sign = bits >> 7;
  uint32_t f = (sign << 31);
  int bit6 = (bits >> 6) & 1;
  uint32_t exp = ((!bit6) << 7) | (0x7C * bit6) | ((bits >> 4) & 3);
  uint32_t mantissa = (bits & 0xF) << 19;
  f |= exp << 23;
  f |= mantissa;
  float fl;
  memcpy(&fl, &f, sizeof(float));
  return fl;
}

bool FPImm8FromFloat(float value, uint8_t* immOut)
{
  uint32_t f;
  memcpy(&f, &value, sizeof(float));
  uint32_t mantissa4 = (f & 0x7FFFFF) >> 19;
  uint32_t exponent = (f >> 23) & 0xFF;
  uint32_t sign = f >> 31;
  if ((exponent >> 7) == ((exponent >> 6) & 1))
    return false;
  uint8_t imm8 = (sign << 7) | ((!(exponent >> 7)) << 6) | ((exponent & 3) << 4) | mantissa4;
  float newFloat = FPImm8ToFloat(imm8);
  if (newFloat == value)
    *immOut = imm8;
  else
    return false;
  return true;
}

void MOVI2F(ARM64Reg Rd, float value, ARM64Reg scratch, bool negate)
{
  _assert_msg_(DYNA_REC, !IsDouble(Rd), "MOVI2F does not yet support double precision");
  uint8_t imm8;
  if (value == 0.0)
  {
    FMOV(Rd, IsDouble(Rd) ? ZR : WZR);
    if (negate)
      FNEG(Rd, Rd);
    // TODO: There are some other values we could generate with the float-imm instruction, like
    // 1.0...
  }
  else if (FPImm8FromFloat(value, &imm8))
  {
    FMOV(Rd, imm8);
  }
  else
  {
    _assert_msg_(DYNA_REC, scratch != INVALID_REG,
                 "Failed to find a way to generate FP immediate %f without scratch", value);
    u32 ival;
    if (negate)
      value = -value;

    memcpy(&ival, &value, sizeof(ival));
    m_emit->MOVI2R(scratch, ival);
    FMOV(Rd, scratch);
  }
}

// TODO: Quite a few values could be generated easily using the MOVI instruction and friends.
void MOVI2FDUP(ARM64Reg Rd, float value, ARM64Reg scratch)
{
  // TODO: Make it work with more element sizes
  // TODO: Optimize - there are shorter solution for many values
  ARM64Reg s = (ARM64Reg)(S0 + DecodeReg(Rd));
  MOVI2F(s, value, scratch);
  DUP(32, Rd, Rd, 0);
}

void SUBSI2R(ARM64Reg Rd, ARM64Reg Rn, u64 imm, ARM64Reg scratch)
{
  u32 val;
  bool shift;
  if (IsImmArithmetic(imm, &val, &shift))
  {
    SUBS(Rd, Rn, val, shift);
  }
  else
  {
    _assert_msg_(DYNA_REC, scratch != INVALID_REG,
                 "ANDSI2R - failed to construct immediate value from %08x, need scratch", (u32)imm);
    MOVI2R(scratch, imm);
    SUBS(Rd, Rn, scratch);
  }
}

}  // namespace
