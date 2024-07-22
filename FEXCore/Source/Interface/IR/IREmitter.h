// SPDX-License-Identifier: MIT
#pragma once

#include "Interface/IR/IR.h"

#include <FEXCore/Core/CoreState.h>
#include <FEXCore/IR/IR.h>

#include <FEXCore/Utils/LogManager.h>
#include <FEXCore/fextl/vector.h>

#include <algorithm>
#include <new>
#include <stdint.h>
#include <string.h>

namespace FEXCore::IR {
class Pass;
class PassManager;

class Block {
public:
  /* Instructions within the block are encoded as a stream of 32-bit tokens. */
  std::vector<uint32_t> Tokens;
};

class IREmitter {
  friend class FEXCore::IR::Pass;
  friend class FEXCore::IR::PassManager;

public:
  IREmitter(FEXCore::Utils::IntrusivePooledAllocator& ThreadAllocator) {}

  virtual ~IREmitter() = default;

  void ResetWorkingList();

  /**
   * @name IR allocation routines
   *
   * @{ */

  FEXCore::IR::RegisterClassType WalkFindRegClass(Ref Node);

// These handlers add cost to the constructor and destructor
// If it becomes an issue then blow them away
// GCC also generates some pretty atrocious code around these
// Use Clang!
#define IROP_ALLOCATE_HELPERS
#define IROP_DISPATCH_HELPERS
#include <FEXCore/IR/IRDefines.inc>
  Ref _Constant(uint8_t Size, uint64_t Constant) {
    IROp_Constant* Op = AllocateOp<IROp_Constant, IROps::OP_CONSTANT>();
    uint64_t Mask = ~0ULL >> (64 - Size);
    Op->Dest = AllocateTemp();
    Op->Constant = (Constant & Mask);
    Op->Header.Size = Size / 8;
    Op->Header.ElementSize = Size / 8;
    return Op->Dest;
  }
  IROp_Jump* _Jump() {
    return _Jump(InvalidRef);
  }
  IROp_CondJump* _CondJump(Ref ssa0, CondClassType cond = {COND_NEQ}) {
    return _CondJump(ssa0, _Constant(0), InvalidRef, InvalidRef, cond, ssa0.Size());
  }
  IROp_CondJump* _CondJump(Ref ssa0, Ref ssa1, Ref ssa2, CondClassType cond = {COND_NEQ}) {
    return _CondJump(ssa0, _Constant(0), ssa1, ssa2, cond, ssa0.Size());
  }
  // TODO: Work to remove this implicit sized Select implementation.
  Ref _Select(uint8_t Cond, Ref ssa0, Ref ssa1, Ref ssa2, Ref ssa3, uint8_t CompareSize = 0) {
    if (CompareSize == 0) {
      CompareSize = std::max<uint8_t>(4, std::max<uint8_t>(ssa0.Size(), ssa1.Size()));
    }

    return _Select(IR::SizeToOpSize(std::max<uint8_t>(4, std::max<uint8_t>(ssa2.Size(), ssa3.Size()))), IR::SizeToOpSize(CompareSize),
                   CondClassType {Cond}, ssa0, ssa1, ssa2, ssa3);
  }
  Ref _LoadMem(FEXCore::IR::RegisterClassType Class, uint8_t Size, Ref ssa0, uint8_t Align = 1) {
    return _LoadMem(Class, Size, ssa0, Invalid(), Align, MEM_OFFSET_SXTX, 1);
  }
  Ref _LoadMemTSO(FEXCore::IR::RegisterClassType Class, uint8_t Size, Ref ssa0, uint8_t Align = 1) {
    return _LoadMemTSO(Class, Size, ssa0, Invalid(), Align, MEM_OFFSET_SXTX, 1);
  }
  IROp_StoreMem* _StoreMem(FEXCore::IR::RegisterClassType Class, uint8_t Size, Ref Addr, Ref Value, uint8_t Align = 1) {
    return _StoreMem(Class, Size, Value, Addr, Invalid(), Align, MEM_OFFSET_SXTX, 1);
  }
  IROp_StoreMemTSO* _StoreMemTSO(FEXCore::IR::RegisterClassType Class, uint8_t Size, Ref Addr, Ref Value, uint8_t Align = 1) {
    return _StoreMemTSO(Class, Size, Value, Addr, Invalid(), Align, MEM_OFFSET_SXTX, 1);
  }

  // TODO: Remove me.
  Ref Invalid() {
    return InvalidRef;
  }

  void SetJumpTarget(IR::IROp_Jump* Op, Ref Target) {
    Op->TargetBlock = Target;
  }
  void SetTrueJumpTarget(IR::IROp_CondJump* Op, Ref Target) {
    Op->TrueBlock = Target;
  }
  void SetFalseJumpTarget(IR::IROp_CondJump* Op, Ref Target) {
    Op->FalseBlock = Target;
  }

  void ReplaceNodeArgument(Ref Node, uint8_t Arg, Ref NewArg);

  void SetPackedRFLAG(bool Lower8, Ref Src);
  Ref GetPackedRFLAG(bool Lower8);

  Block* CreateNewCodeBlockAtEnd() {
    CodeBlocks.push_back({});
    return &CodeBlocks[CodeBlocks.size() - 1];
  }
  void SetCurrentCodeBlock(Block* Block) {
    LOGMAN_THROW_A_FMT((Block - CodeBlocks.data()) < CodeBlocks.size(), "must be valid block");
    CurrentCodeBlock = Block;
  }

protected:
  // Overriden by dispatcher, stubbed for IR tests
  virtual void RecordX87Use() {}
  virtual void SaveNZCV(IROps Op) {}

  Block* CurrentCodeBlock;
  fextl::vector<Block> CodeBlocks;
  uint32_t TempAlloc;
  uint64_t Entry;

  template<class T, IROps T2>
  T* AllocateOp() {
    size_t Size = FEXCore::IR::GetSize(T2);
    uint32_t N = FEXCore::IR::GetSize(T2) / sizeof(uint32_t);
    uint32_t OldSize = CurrentCodeBlock->Tokens.size();
    CurrentCodeBlock->Tokens.resize(OldSize + N);
    uint32_t* Data = CurrentCodeBlock->Tokens.data();
    auto Op = reinterpret_cast<T*>(&Data[OldSize]);
    memset(Op, 0, Size);
    Op->Header.Op = T2;
    return Op;
  }

  Ref AllocateTemp() {
    // TODO: Missing size
    return {
      .ID = TempAlloc++,
      ._IsValid = 1,
    };
  }
};

} // namespace FEXCore::IR
