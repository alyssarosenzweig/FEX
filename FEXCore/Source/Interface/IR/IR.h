// SPDX-License-Identifier: MIT
#pragma once

#include <FEXCore/Utils/ThreadPoolAllocator.h>
#include <FEXCore/IR/IR.h>

#include <FEXCore/fextl/memory.h>
#include <FEXCore/fextl/sstream.h>

namespace FEXCore::IR {

class OrderedNode;
class RegisterAllocationPass;
class RegisterAllocationData;

/**
 * @brief The IROp_Header is an dynamically sized array
 * At the end it contains a uint8_t for the number of arguments that Op has
 * Then there is an unsized array of NodeWrapper arguments for the number of arguments this op has
 * The op structures that are including the header must ensure that they pad themselves correctly to the number of arguments used
 */
struct IROp_Header;

struct RegisterClassType final {
  using value_type = uint32_t;

  value_type Val;
  [[nodiscard]] constexpr operator value_type() const {
    return Val;
  }
  [[nodiscard]]
  friend constexpr bool
  operator==(const RegisterClassType&, const RegisterClassType&) = default;
};

union PhysicalRegister {
  uint8_t Raw;
  struct {
    // 32 maximum physical registers
    uint8_t Reg : 5;
    // 8 Maximum classes
    uint8_t Class : 3;
  };

  bool operator==(const PhysicalRegister& Other) const {
    return Raw == Other.Raw;
  }

  PhysicalRegister(RegisterClassType Class, uint8_t Reg)
    : Reg(Reg)
    , Class((uint8_t)Class.Val) {}


  /* Reserve the all-1's bit pattern */
  static const uint8_t _Invalid = 0xFF;

  static const PhysicalRegister Invalid() {
    return PhysicalRegister((RegisterClassType)_Invalid, _Invalid);
  }

  bool IsInvalid() const {
    return *this == Invalid();
  }
};

static_assert(sizeof(PhysicalRegister) == 1);

struct Ref {
  uint32_t ID      : 23;
  uint8_t Reg      : 5;
  uint8_t _IsValid : 1;

  // Size in bytes, log2. 0 is 8-bit, 1 is 16-bit...
  // TODO: I'd like to remove this some day.
  uint8_t Size_log2 : 3;

  bool Valid() const {
    return _IsValid;
  }

  unsigned Size() const {
    return 1u << ((unsigned)Size_log2);
  }
};

static Ref InvalidRef = {
  ._IsValid = false,
};

struct CondClassType final {
  uint8_t Val;
  [[nodiscard]] constexpr operator uint8_t() const {
    return Val;
  }
  [[nodiscard]]
  friend constexpr bool
  operator==(const CondClassType&, const CondClassType&) = default;
};

struct MemOffsetType final {
  uint8_t Val;
  [[nodiscard]] constexpr operator uint8_t() const {
    return Val;
  }
  [[nodiscard]]
  friend constexpr bool
  operator==(const MemOffsetType&, const MemOffsetType&) = default;
};

struct TypeDefinition final {
  uint16_t Val;

  [[nodiscard]] constexpr operator uint16_t() const {
    return Val;
  }

  [[nodiscard]]
  static constexpr TypeDefinition Create(uint8_t Bytes) {
    TypeDefinition Type {};
    Type.Val = Bytes << 8;
    return Type;
  }

  [[nodiscard]]
  static constexpr TypeDefinition Create(uint8_t Bytes, uint8_t Elements) {
    TypeDefinition Type {};
    Type.Val = (Bytes << 8) | (Elements & 255);
    return Type;
  }

  [[nodiscard]]
  constexpr uint8_t Bytes() const {
    return Val >> 8;
  }

  [[nodiscard]]
  constexpr uint8_t Elements() const {
    return Val & 255;
  }

  [[nodiscard]]
  friend constexpr bool
  operator==(const TypeDefinition&, const TypeDefinition&) = default;
};

static_assert(std::is_trivial_v<TypeDefinition>);

struct FenceType final {
  using value_type = uint8_t;

  value_type Val;
  [[nodiscard]] constexpr operator value_type() const {
    return Val;
  }
  [[nodiscard]]
  friend constexpr bool
  operator==(const FenceType&, const FenceType&) = default;
};

struct RoundType final {
  uint8_t Val;
  [[nodiscard]] constexpr operator uint8_t() const {
    return Val;
  }
  [[nodiscard]]
  friend constexpr bool
  operator==(const RoundType&, const RoundType&) = default;
};

// This must directly match bytes to the named opsize.
// Implicit sized IR operations does math to get between sizes.
enum OpSize : uint8_t {
  i8Bit = 1,
  i16Bit = 2,
  i32Bit = 4,
  i64Bit = 8,
  i128Bit = 16,
  i256Bit = 32,
};

enum class FloatCompareOp : uint8_t {
  EQ = 0,
  LT,
  LE,
  UNO,
  NEQ,
  ORD,
};

enum class ShiftType : uint8_t {
  LSL = 0,
  LSR,
  ASR,
  ROR,
};


// Converts a size stored as an integer in to an OpSize enum.
// This is a nop operation and will be eliminated by the compiler.
static inline OpSize SizeToOpSize(uint8_t Size) {
  switch (Size) {
  case 1: return OpSize::i8Bit;
  case 2: return OpSize::i16Bit;
  case 4: return OpSize::i32Bit;
  case 8: return OpSize::i64Bit;
  case 16: return OpSize::i128Bit;
  case 32: return OpSize::i256Bit;
  default: FEX_UNREACHABLE;
  }
}

#define IROP_ENUM
#define IROP_STRUCTS
#define IROP_SIZES
#define IROP_REG_CLASSES
#include <FEXCore/IR/IRDefines.inc>

bool IsFragmentExit(FEXCore::IR::IROps Op);
bool IsBlockExit(FEXCore::IR::IROps Op);
} // namespace FEXCore::IR

template<>
struct fmt::formatter<FEXCore::IR::RegisterClassType> : fmt::formatter<FEXCore::IR::RegisterClassType::value_type> {
  using Base = fmt::formatter<FEXCore::IR::RegisterClassType::value_type>;

  template<typename FormatContext>
  auto format(const FEXCore::IR::RegisterClassType& Class, FormatContext& ctx) const {
    return Base::format(Class.Val, ctx);
  }
};

template<>
struct fmt::formatter<FEXCore::IR::FenceType> : fmt::formatter<FEXCore::IR::FenceType::value_type> {
  using Base = fmt::formatter<FEXCore::IR::FenceType::value_type>;

  template<typename FormatContext>
  auto format(const FEXCore::IR::FenceType& Fence, FormatContext& ctx) const {
    return Base::format(Fence.Val, ctx);
  }
};

template<>
struct fmt::formatter<FEXCore::IR::OpSize> : fmt::formatter<std::underlying_type_t<FEXCore::IR::OpSize>> {
  using Base = fmt::formatter<std::underlying_type_t<FEXCore::IR::OpSize>>;

  template<typename FormatContext>
  auto format(const FEXCore::IR::OpSize& OpSize, FormatContext& ctx) const {
    return Base::format(FEXCore::ToUnderlying(OpSize), ctx);
  }
};
