// SPDX-License-Identifier: MIT
/*
$info$
tags: ir|opts
$end_info$
*/

#include "Utils/BucketList.h"

#include "Interface/IR/Passes/RegisterAllocationPass.h"
#include "FEXCore/Core/X86Enums.h"
#include "Interface/IR/IR.h"
#include "Interface/IR/IREmitter.h"
#include "Interface/IR/RegisterAllocationData.h"
#include "Interface/IR/Passes.h"
#include <FEXCore/Core/CoreState.h>
#include <FEXCore/IR/IR.h>
#include <FEXCore/Utils/LogManager.h>
#include <FEXCore/Utils/MathUtils.h>
#include <FEXCore/Utils/Profiler.h>
#include <FEXCore/Utils/TypeDefines.h>
#include <FEXCore/fextl/fmt.h>
#include <FEXCore/fextl/set.h>
#include <FEXCore/fextl/unordered_map.h>
#include <FEXCore/fextl/unordered_set.h>
#include <FEXCore/fextl/vector.h>

#include <FEXHeaderUtils/BitUtils.h>

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <strings.h>
#include <utility>

namespace FEXCore::IR {
namespace {
  constexpr uint32_t INVALID_REG = FEXCore::IR::InvalidReg;
  constexpr uint32_t INVALID_CLASS = FEXCore::IR::InvalidClass.Val;

  constexpr uint32_t DEFAULT_NODE_COUNT = 8192;

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
    uint32_t NodeCount {};
  };

  RegisterGraph* AllocateRegisterGraph(uint32_t ClassCount) {
    RegisterGraph* Graph = new RegisterGraph {};

    // Allocate the register set
    Graph->Set.ClassCount = ClassCount;
    Graph->Set.Classes.resize(ClassCount);

    // Allocate default nodes
    Graph->AllocData = RegisterAllocationData::Create(DEFAULT_NODE_COUNT);
    Graph->NodeCount = DEFAULT_NODE_COUNT;

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

  /**
   * @brief Returns the register and class encoded together
   * Top 32bits is the class, lower 32bits is the register
   */
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

  Graph->AllocData->SpillSlotCount = 0;

  using namespace FEXCore;

  auto IR = IREmit->ViewIR();

  for (auto [BlockNode, BlockHeader] : IR.GetBlocks()) {
    for (auto &Class : Graph->Set.Classes) {
      // At the start of each block, all registers are available. Initialize the
      // available bit set. This is a bit set.
      Class.Available = (1 << Class.Count) - 1;
    }

    // Pass 1: Iterate the block backwards.
    //  - analyze kill bits. (TODO)
    //  - analyze SRA affinities (TODO)
    //  - insert moves for tied operands (TODO)

    // Pass 2: Iterate the block forward.
    //  - split live ranges if necessary (shuffle, spills) (TODO)
    //  - update available set per kill bits (TODO)
    //  - assign destination registers, guaranteed to succeed.
    //const auto BlockNodeID = IR.GetID(BlockNode);
    for (auto [CodeNode, IROp] : IR.GetCode(BlockNode)) {
      const auto Node = IR.GetID(CodeNode);

      /* Assign destinations */
      if (GetHasDest(IROp->Op)) {
        LOGMAN_THROW_AA_FMT(Node.IsValid(), "Dest must be valid");

        auto ClassType = GetRegClassFromNode(&IR, IROp);
        auto Class = &Graph->Set.Classes[ClassType];

        if (!Class->Available) {
          /* TODO: Spill */
          abort();
        }

        // Assign a free register in the appropriate class
        // Now that we have limited the RA size, this must succeed.
        // TODO: reg pairs
        unsigned Reg = std::countr_zero(Class->Available);
        LOGMAN_THROW_AA_FMT(Reg < Class->Count, "Ensured available");
        Class->Available &= ~(1 << Reg);

        Graph->AllocData->Map[Node.Value] = PhysicalRegister(ClassType, Reg);
      }
    }
  }

  /* No point tracking this finely, RA is always one-shot */
  return true;
}

fextl::unique_ptr<FEXCore::IR::RegisterAllocationPass> CreateRegisterAllocationPass(FEXCore::IR::Pass* CompactionPass, bool SupportsAVX) {
  return fextl::make_unique<ConstrainedRAPass>(SupportsAVX);
}
} // namespace FEXCore::IR
