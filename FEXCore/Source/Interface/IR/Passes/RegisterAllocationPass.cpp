// SPDX-License-Identifier: MIT
/*
$info$
tags: ir|opts
$end_info$
*/

#include "Interface/IR/Passes/RegisterAllocationPass.h"
#include "Interface/IR/IR.h"
#include "Interface/IR/IREmitter.h"
#include "Interface/IR/RegisterAllocationData.h"
#include "Interface/IR/Passes.h"
#include <FEXCore/IR/IR.h>
#include <FEXCore/Utils/LogManager.h>
#include <FEXCore/Utils/Profiler.h>
#include <FEXCore/fextl/vector.h>
#include <bit>
#include <cstdint>

namespace FEXCore::IR {
namespace {
  constexpr uint32_t INVALID_REG = FEXCore::IR::InvalidReg;
  constexpr uint32_t INVALID_CLASS = FEXCore::IR::InvalidClass.Val;
  constexpr uint32_t EVEN_BITS = 0x55555555;

  struct RegisterClass {
    uint32_t Available;
    uint32_t Count;

    // If bit R of Available is 0, then RegToSSA[R] is the node
    // currently allocated to R.
    //
    // Else, RegToSSA[R] is UNDEFINED. This means we don't need to clear this
    // when clearing bits from Available.
    OrderedNode *RegToSSA[32];
  };

  struct RegisterSet {
    fextl::vector<RegisterClass> Classes;
    uint32_t ClassCount;
  };

  struct RegisterGraph : public FEXCore::Allocator::FEXAllocOperators {
    IR::RegisterAllocationData::UniquePtr AllocData;
    RegisterSet Set;
  };

  RegisterGraph* AllocateRegisterGraph(uint32_t ClassCount) {
    RegisterGraph* Graph = new RegisterGraph {};

    // Allocate the register set
    Graph->Set.ClassCount = ClassCount;
    Graph->Set.Classes.resize(ClassCount);

    return Graph;
  }

  FEXCore::IR::RegisterClassType GetRegClassFromNode(FEXCore::IR::IRListView* IR, FEXCore::IR::IROp_Header* IROp) {
    using namespace FEXCore;

    FEXCore::IR::RegisterClassType Class = IR::GetRegClass(IROp->Op);
    if (Class != FEXCore::IR::ComplexClass) {
      return Class;
    }

    // Complex register class handling
    switch (IROp->Op) {
    case IR::OP_LOADCONTEXT:
      return IROp->C<IR::IROp_LoadContext>()->Class;

    case IR::OP_LOADREGISTER:
      return IROp->C<IR::IROp_LoadRegister>()->Class;

    case IR::OP_LOADCONTEXTINDEXED:
      return IROp->C<IR::IROp_LoadContextIndexed>()->Class;

    case IR::OP_LOADMEM:
    case IR::OP_LOADMEMTSO:
      return IROp->C<IR::IROp_LoadMem>()->Class;

    case IR::OP_FILLREGISTER:
      return IROp->C<IR::IROp_FillRegister>()->Class;

    default:
      // Unreachable
      return FEXCore::IR::InvalidClass;
    }
  };
} // Anonymous namespace

class ConstrainedRAPass final : public RegisterAllocationPass {
public:
  ConstrainedRAPass(bool SupportsAVX);
  ~ConstrainedRAPass();
  bool Run(IREmitter* IREmit) override;

  void AllocateRegisterSet(uint32_t ClassCount) override;
  void AddRegisters(FEXCore::IR::RegisterClassType Class, uint32_t RegisterCount) override;

  RegisterAllocationData* GetAllocationData() override;
  RegisterAllocationData::UniquePtr PullAllocationData() override;

private:
  RegisterGraph* Graph;
  bool SupportsAVX;
};

ConstrainedRAPass::ConstrainedRAPass(bool _SupportsAVX)
  : SupportsAVX {_SupportsAVX} {}

ConstrainedRAPass::~ConstrainedRAPass() {
  delete Graph;
}

void ConstrainedRAPass::AllocateRegisterSet(uint32_t ClassCount) {
  LOGMAN_THROW_AA_FMT(ClassCount <= INVALID_CLASS, "Up to {} classes supported", INVALID_CLASS);

  Graph = AllocateRegisterGraph(ClassCount);
}

void ConstrainedRAPass::AddRegisters(FEXCore::IR::RegisterClassType Class, uint32_t RegisterCount) {
  LOGMAN_THROW_AA_FMT(RegisterCount <= INVALID_REG, "Up to {} regs supported", INVALID_REG);

  Graph->Set.Classes[Class].Count = RegisterCount;
}

RegisterAllocationData* ConstrainedRAPass::GetAllocationData() {
  return Graph->AllocData.get();
}

RegisterAllocationData::UniquePtr ConstrainedRAPass::PullAllocationData() {
  return std::move(Graph->AllocData);
}

#define foreach_valid_arg(IROp, index, arg) \
  for (auto [index, Arg] = std::tuple(0, IROp->Args[0]); \
       index < IR::GetRAArgs(IROp->Op); \
       Arg = IROp->Args[++index]) \
    if (IsValidArg(Arg))

bool ConstrainedRAPass::Run(IREmitter* IREmit) {
  FEXCORE_PROFILE_SCOPED("PassManager::RA");

  using namespace FEXCore;

  auto IR = IREmit->ViewIR();

  // Map of assigned registers. We need this to be a resizable vector (differing
  // from RegisterAllocationData) in order to implement live range splits, which
  // will materialize new SSA indices.
  PhysicalRegister InvalidPhysReg = PhysicalRegister(InvalidClass, InvalidReg);
  fextl::vector<PhysicalRegister> SSAToReg(IR.GetSSACount(), InvalidPhysReg);

  // SSA def remapping. FEX's original RA could only assign a single register to
  // a given def for its entire live range, and this limitation is baked deep
  // into the IR. However, we need to be able to split live ranges to
  // implement register pairs and spilling properly.
  //
  // To reconcile these two worlds, we'll generate new SSA nodes when we split
  // live ranges, and remap SSA sources accordingly. This is a bit annoying,
  // because it means SSAToReg can grow, but it avoids disturbing the rest of
  // FEX (for now).
  //
  // Down the road, we might want to optimize this but I'm not prepared to
  // bulldoze the IR for this.
  //
  // This data structure tracks the current remapping. nullptr indicates no
  // remapping.
  //
  // Unlike SSAToReg, this data structure does not grow, since it only tracks
  // SSA defs that were there before we started splitting live ranges.
  fextl::vector<OrderedNode *> SSAToSSA(IR.GetSSACount(), nullptr);

  // Mapping of spilled SSA defs to their assigned slot + 1, or 0 for defs that
  // have not been spilled. Persisting this mapping avoids repeated spills of
  // the same long-lived SSA value.
  //
  // Does not grow. XXX: yes it does
  fextl::vector<unsigned> SpillSlots(IR.GetSSACount(), 0);

  // IP of next-use of each SSA source. IPs are measured from the end of the
  // block, so we don't need to size the block up-front.
  //
  // TODO: Do we want to merge with SourceSeen?
  //
  // XXX: grows
  fextl::vector<uint32_t> NextUses(IR.GetSSACount(), 0);

  unsigned SpillSlotCount = 0;

  auto IsValidArg = [&IR](auto Arg){
    if (Arg.IsInvalid()) return false;

    switch (IR.GetOp<IROp_Header>(Arg)->Op) {
    case OP_INLINECONSTANT:
    case OP_INLINEENTRYPOINTOFFSET:
    case OP_IRHEADER:
      return false;

    case OP_SPILLREGISTER:
      LOGMAN_MSG_A_FMT("should not be seen");
      return false;

    default:
      return true;
    }
  };

  auto FreeReg = [this](auto Reg){
    bool Pair = Reg.Class == GPRPairClass;
    auto ClassType = Pair ? GPRClass : Reg.Class;
    auto RegBits = Pair ? (0x3 << (2 * Reg.Reg)) : (1 << Reg.Reg);

    auto Class = &Graph->Set.Classes[ClassType];
    printf("freeing %u\n", Reg.Reg);
    LOGMAN_THROW_AA_FMT(!(Class->Available & RegBits), "Register double-free");
    Class->Available |= RegBits;
  };

  auto ClassSize = [](RegisterClassType T){
    return (T == GPRPairClass) ? 2 : 1;
  };

  auto SpillReg = [this, &IR, &IREmit, &SpillSlotCount, &SpillSlots, &SSAToReg, &FreeReg, &NextUses](auto Class, uint32_t IP) {
    // First, find the best node to spill. We use the well-known
    // "furthest-first" heuristic, spilling the node whose next-use is the
    // farthest in the future.
    //
    // Since we defined IPs relative to the end of the block, the furthest
    // next-use has the /smallest/ unsigned IP.
    //
    // TODO: Remat.
    OrderedNode *Candidate = nullptr;
    uint32_t BestDistance = UINT32_MAX;
    uint32_t BestReg = 0;

    printf("\n!!\n");
    for (int i = 0;i<Class->Count;++i){
      if (!(Class->Available & (1 << i))) {
        OrderedNode *SSA = Class->RegToSSA[i];

        // TODO: avoid this edge case
        if (SSA == nullptr)
          continue;

        auto Index = IR.GetID(SSA).Value;

        // XXX: fills etc
        uint32_t NextUse = NextUses.at(Index);
        printf("%u: %u, %u / %u\n", i, Index, NextUse, IP);
        if (NextUse < BestDistance) {
          BestDistance = NextUse;
          Candidate = SSA;
          BestReg = i;
        }
      }
    }
    printf("!!\n\n");

    LOGMAN_THROW_AA_FMT(BestDistance < IP, "use must be in a future instruction");
    LOGMAN_THROW_AA_FMT(Candidate != nullptr, "must've found something..");

    // If we already spilled the Candidate, we don't need to spill again, we can
    // just free the register and insert another fill later. Otherwise, we
    // actually insert the spill here.
    //
    // TODO: also handle copies inserted for live range splitting.
    auto Header = IR.GetOp<IROp_Header>(Candidate);
    auto Value = IR.GetID(Candidate).Value;
    printf("spilling %u : index %u\n", BestReg, Value);

    if (Header->Op == OP_FILLREGISTER) {
      auto Value = IR.GetID(Candidate).Value;
      if (Value >= SpillSlots.size()) {
        SpillSlots.resize(Value + 1, 0);
      }

      auto Fill = IR.GetOp<IROp_FillRegister>(Candidate);
      SpillSlots.at(Value) = Fill->Slot;
    } else {
      auto CT = GetRegClassFromNode(&IR, Header);

      // TODO: we should colour spill slots
      auto Slot = SpillSlotCount++;

      printf("at %u\n", Value);
      auto SpillOp = IREmit->_SpillRegister(Candidate, Slot, CT);
      SpillOp.first->Header.Size = Header->Size;
      SpillOp.first->Header.ElementSize = Header->ElementSize;
      SpillSlots.at(Value) = Slot + 1;
    }

    // Now that we've spilled the value, take it out of the register file
    printf("freeing after spill\n");
    FreeReg(SSAToReg.at(Value));
  };

  auto AssignReg = [this, &IR, IREmit, &SSAToReg, &SpillReg, &ClassSize, InvalidPhysReg](OrderedNode *CodeNode, uint32_t IP) {
    const auto Node = IR.GetID(CodeNode);
    const auto IROp = IR.GetOp<IROp_Header>(CodeNode);

    LOGMAN_THROW_AA_FMT(Node.IsValid(), "Node must be valid");

    auto OrigClassType = GetRegClassFromNode(&IR, IROp);
    bool Pair = OrigClassType == GPRPairClass;
    auto ClassType = Pair ? GPRClass : OrigClassType;
    auto Size = ClassSize(ClassType);

    auto Class = &Graph->Set.Classes[ClassType];

    // First, we need to limit the register file to ensure space, spilling if
    // necessary. This is based only on the number of bits set in Available, not
    // their order. At this point, free registers need not be contiguous, even
    // if we're allocating a pair. We'll worry about shuffle code later.
    //
    // TODO: Maybe specialize this function for pairs vs not-pairs?
    while (std::popcount(Class->Available) < Size) {
      IREmit->SetWriteCursorBefore(CodeNode);
      SpillReg(Class, IP);
    }

    // Now that we've spilled, there are enough registers. Try to assign one.
    uint32_t Available = Class->Available;
    uint32_t SizeMask = Pair ? 0b11 : 0b1;

    // Limit Available to only valid base registers for pairs
    if (Pair) {
      // Only choose register R if R and R + 1 are both free.
      Available &= (Available >> 1);

      // Only consider aligned registers
      Available &= EVEN_BITS;
    }

    if (!Available) {
      LOGMAN_THROW_AA_FMT(OrigClassType == GPRPairClass, "Already spilled");
      /* TODO: Live range split */
      printf("live range split!\n");
      abort();
    }

    // Assign a free register in the appropriate class
    // Now that we have split live ranges, this must succeed.
    unsigned Reg = std::countr_zero(Available);
    uint32_t RegBits = SizeMask << Reg;

    LOGMAN_THROW_AA_FMT((Class->Available & RegBits) == RegBits, "Ensured available");
    Class->Available &= ~RegBits;
    Class->RegToSSA[Reg] = CodeNode;

    if (Node.Value >= SSAToReg.size()) {
      SSAToReg.resize(Node.Value + 1, InvalidPhysReg);
    }

      printf("allocating %u -> %u\n", Node.Value, Reg);
    SSAToReg.at(Node.Value) =
      PhysicalRegister(OrigClassType, Pair ? (Reg >> 1) : Reg);
  };

  for (auto [BlockNode, BlockHeader] : IR.GetBlocks()) {
    for (auto &Class : Graph->Set.Classes) {
      // At the start of each block, all registers are available. Initialize the
      // available bit set. This is a bit set.
      Class.Available = (1 << 3 /*Class.Count*/) - 1;
    }

    // Stream of sources in the block, backwards. (First element is the last
    // source in the block.)
    //
    // Contains the next-use distance (relative to the end of the block) of the
    // source following this instruction.
    fextl::vector<uint32_t> SourcesNextUses;

    // IP relative to the end of the block.
    uint32_t IP = 1;

    // Backwards pass:
    //  - analyze kill bits, next-use distances, and affinities (TODO).
    //  - insert moves for tied operands (TODO)
    {
      // Reverse iteration is not yet working with the iterators
      auto BlockIROp = BlockHeader->CW<FEXCore::IR::IROp_CodeBlock>();

      // We grab these nodes this way so we can iterate easily
      auto CodeBegin = IR.at(BlockIROp->Begin);
      auto CodeLast = IR.at(BlockIROp->Last);

      while (1) {
        auto [CodeNode, IROp] = CodeLast();
        // End of iteration gunk

        // We iterate sources backwards, since we're walking backwards and this
        // will ensure the order of SourcesKilled is consistent. This means the
        // forward pass can iterate forwards and just flip the order.
        const uint8_t NumArgs = IR::GetRAArgs(IROp->Op);
        for (int8_t i = NumArgs - 1; i >= 0; --i) {
          const auto& Arg = IROp->Args[i];
          if (!IsValidArg(Arg))
            continue;

          const auto Index = Arg.ID().Value;

          SourcesNextUses.push_back(NextUses[Index]);
          NextUses[Index] = IP;
        }

        // IP is relative to end of the block and we're iterating backwards, so
        // increment here.
        ++IP;

        // Rest is iteration gunk
        if (CodeLast == CodeBegin) {
          break;
        }
        --CodeLast;
      }
    }

    // After the backwards pass, NextUses contains with the distances of
    // the first use in each block, which is exactly what we need to initialize
    // at the start of the forward pass. So there's no need for explicit
    // initialization here.

    // SourcesKilled is read backwards, this tracks the index
    //unsigned SourceIndex = SourcesKilled.size();
    unsigned SourceIndex = SourcesNextUses.size();

    // Forward pass: Assign registers, spilling as we go.
    for (auto [CodeNode, IROp] : IR.GetCode(BlockNode)) {
      LOGMAN_THROW_AA_FMT(IROp->Op != OP_SPILLREGISTER &&
                          IROp->Op != OP_FILLREGISTER,
                          "Spills/fills inserted before the node,"
                          "so we don't see them iterating forward");

      // Fill all sources read.
      //
      // This happens before processing kill bits, since we need all sources in
      // the register file at the same time.
      foreach_valid_arg(IROp, _, Arg) {
        auto SlotPlusOne = SpillSlots[Arg.ID().Value];
        if (!SlotPlusOne)
          continue;

        // We found a source that needs to be filled.
        IREmit->SetWriteCursorBefore(CodeNode);

        auto Header = IR.GetOp<IROp_Header>(Arg);
        auto RegClass = GetRegClassFromNode(&IR, Header);

        auto Fill = IREmit->_FillRegister(IR.GetNode(Arg), SlotPlusOne - 1, RegClass);
        Fill.first->Header.Size = Header->Size;
        Fill.first->Header.ElementSize = Header->ElementSize;

        // Remap the source to access the fill destination.
        auto Index = Arg.ID().Value;
        SSAToSSA[Index] = Fill;

        auto FillIndex = IR.GetID(Fill).Value;

        // Transfer the next-use info
        if (FillIndex >= NextUses.size()) {
          NextUses.resize(FillIndex + 1, 0);
        }

        NextUses[FillIndex] = Index;

        // Assign a register for the Fill destination. This may cause something
        // else to be spilled, but that's ok.
        AssignReg(Fill, IP);
      }

      // Remap sources, in case we split any live ranges.
      // Then process killed/next-use info. This must happen _before_ remapping.
      foreach_valid_arg(IROp, i, Arg) {
        const auto Remapped = SSAToSSA[Arg.ID().Value];

        if (Remapped != nullptr) {
          IREmit->ReplaceNodeArgument(CodeNode, i, Remapped);
        }

        SourceIndex--;
        LOGMAN_THROW_AA_FMT(SourceIndex >= 0, "Consistent source count");

        unsigned Index = IROp->Args[i].ID().Value;

        //if (SourcesKilled[SourceIndex]) {
        if (!SourcesNextUses[SourceIndex]) {
          printf("killing reg %u, index %u\n", SSAToReg.at(Index).Reg, Index);
          FreeReg(SSAToReg.at(Index));
        }

        if (Index >= NextUses.size()) {
          NextUses.resize(Index + 1, 0);
        }

      printf("next %u\n", Index);
        NextUses.at(Index) = SourcesNextUses[SourceIndex];
      }

      // Assign destinations
      if (GetHasDest(IROp->Op)) {
        AssignReg(CodeNode, IP);
      }

      LOGMAN_THROW_AA_FMT(IP >= 1, "IP relative to end of block, iterating forward");
      --IP;
    }

    LOGMAN_THROW_AA_FMT(SourceIndex == 0, "Consistent source count in block");
  }

  /* Now that we're done growing things, we can finalize our results.
   *
   * TODO: Rework RegisterAllocationData to remove this memcpy, it's pointless.
   */
  Graph->AllocData = RegisterAllocationData::Create(SSAToReg.size());
  Graph->AllocData->SpillSlotCount = SpillSlotCount;
  memcpy(Graph->AllocData->Map, SSAToReg.data(),
         sizeof(PhysicalRegister) * SSAToReg.size());

  /* No point tracking this finely, RA is always one-shot */
  return true;
}

fextl::unique_ptr<FEXCore::IR::RegisterAllocationPass> CreateRegisterAllocationPass(FEXCore::IR::Pass* CompactionPass, bool SupportsAVX) {
  return fextl::make_unique<ConstrainedRAPass>(SupportsAVX);
}
} // namespace FEXCore::IR
