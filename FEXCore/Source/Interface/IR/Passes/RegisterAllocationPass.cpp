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

namespace FEXCore::IR {
namespace {
  constexpr uint32_t INVALID_REG = FEXCore::IR::InvalidReg;
  constexpr uint32_t INVALID_CLASS = FEXCore::IR::InvalidClass.Val;

  struct RegisterClass {
    uint32_t Available;
    uint32_t Count;
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

bool ConstrainedRAPass::Run(IREmitter* IREmit) {
  FEXCORE_PROFILE_SCOPED("PassManager::RA");

  using namespace FEXCore;

  auto IR = IREmit->ViewIR();

  Graph->AllocData = RegisterAllocationData::Create(IR.GetSSACount());

  auto IsValidArg = [&IR](auto Arg){
    if (Arg.IsInvalid()) return false;

    switch (IR.GetOp<IROp_Header>(Arg)->Op) {
    case OP_INLINECONSTANT:
    case OP_INLINEENTRYPOINTOFFSET:
    case OP_IRHEADER:
      return false;

    default:
      return true;
    }
  };

  // Set of sources that have been seen in a backwards walk. Since sources are
  // all block-local, this doesn't need to be reinitialized in each block.
  //
  // TODO: Optimize with a bitset.
  fextl::vector<bool> SourceSeen(IR.GetSSACount(), false);

  for (auto [BlockNode, BlockHeader] : IR.GetBlocks()) {
    for (auto &Class : Graph->Set.Classes) {
      // At the start of each block, all registers are available. Initialize the
      // available bit set. This is a bit set.
      Class.Available = (1 << Class.Count) - 1;
    }

    // Stream of sources in the block, backwards. (First element is the last
    // source in the block.)
    //
    // If set to true, that indicates the corresponding source kills its def.
    //
    // TODO: Optimize with a bitset.
    fextl::vector<bool> SourcesKilled;

    // Pass 1: Iterate the block backwards.
    //  - analyze kill bits.
    //  - analyze SRA affinities (TODO)
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

          // Sources are killed the first time we see them backwards.
          const bool Killed = !SourceSeen[Index];
          SourceSeen[Index] = true;

          SourcesKilled.push_back(Killed);
        }

        // Rest is iteration gunk
        if (CodeLast == CodeBegin) {
          break;
        }
        --CodeLast;
      }
    }

    // SourcesKilled is read backwards, this tracks the index
    unsigned SourceIndex = SourcesKilled.size();

    // Pass 2: Iterate the block forward.
    //  - split live ranges if necessary (shuffle, spills) (TODO)
    //  - update available set per kill bits
    //  - assign destination registers, guaranteed to succeed.
    for (auto [CodeNode, IROp] : IR.GetCode(BlockNode)) {
      const auto Node = IR.GetID(CodeNode);

      // Update available bit sets.
      const uint8_t NumArgs = IR::GetRAArgs(IROp->Op);
      for (uint8_t i = 0; i < NumArgs; ++i) {
        const auto& Arg = IROp->Args[i];
        if (!IsValidArg(Arg))
          continue;

        const auto Index = Arg.ID().Value;

        SourceIndex--;
        LOGMAN_THROW_AA_FMT(SourceIndex >= 0, "Consistent source count");
        const bool Killed = SourcesKilled[SourceIndex];

        if (Killed) {
          auto Reg = Graph->AllocData->Map[Index];
          auto Class = &Graph->Set.Classes[Reg.Class];
          auto RegBit = 1 << Reg.Reg;

          LOGMAN_THROW_AA_FMT(!(Class->Available & RegBit),
                              "In-use sources are not available");
          Class->Available |= RegBit;
        }
      }

      /* Assign destinations */
      if (GetHasDest(IROp->Op)) {
        LOGMAN_THROW_AA_FMT(Node.IsValid(), "Dest must be valid");

        auto ClassType = GetRegClassFromNode(&IR, IROp);
        auto Class = &Graph->Set.Classes[ClassType];

        // TODO: reg pairs
        LOGMAN_THROW_AA_FMT(ClassType != GPRPairClass, "TODO: pair");
        LOGMAN_THROW_AA_FMT(Class->Available, "TODO: spill");

        if (!Class->Available) {
          /* TODO: Spill */
          printf("spill!\n");
          abort();
        }

        // Assign a free register in the appropriate class
        // Now that we have limited the RA size, this must succeed.
        unsigned Reg = std::countr_zero(Class->Available);
        LOGMAN_THROW_AA_FMT(Reg < Class->Count, "Ensured available");
        Class->Available &= ~(1 << Reg);

        Graph->AllocData->Map[Node.Value] = PhysicalRegister(ClassType, Reg);
      }
    }

    LOGMAN_THROW_AA_FMT(SourceIndex == 0, "Consistent source count in block");
  }

  /* No point tracking this finely, RA is always one-shot */
  return true;
}

fextl::unique_ptr<FEXCore::IR::RegisterAllocationPass> CreateRegisterAllocationPass(FEXCore::IR::Pass* CompactionPass, bool SupportsAVX) {
  return fextl::make_unique<ConstrainedRAPass>(SupportsAVX);
}
} // namespace FEXCore::IR
