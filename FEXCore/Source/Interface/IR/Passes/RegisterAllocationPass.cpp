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

  struct Register {
    bool Virtual;
    uint64_t Index;
  };

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

  void AllocatePhysicalRegisters(RegisterGraph* Graph, FEXCore::IR::RegisterClassType Class, uint32_t Count) {
    Graph->Set.Classes[Class].Count = Count;
  }

  FEXCore::IR::RegisterClassType GetRegClassFromNode(FEXCore::IR::IRListView* IR, FEXCore::IR::IROp_Header* IROp) {
    using namespace FEXCore;

    FEXCore::IR::RegisterClassType Class = IR::GetRegClass(IROp->Op);
    if (Class != FEXCore::IR::ComplexClass) {
      return Class;
    }

    // Complex register class handling
    switch (IROp->Op) {
    case IR::OP_LOADCONTEXT: {
      auto Op = IROp->C<IR::IROp_LoadContext>();
      return Op->Class;
      break;
    }
    case IR::OP_LOADREGISTER: {
      auto Op = IROp->C<IR::IROp_LoadRegister>();
      return Op->Class;
      break;
    }
    case IR::OP_LOADCONTEXTINDEXED: {
      auto Op = IROp->C<IR::IROp_LoadContextIndexed>();
      return Op->Class;
      break;
    }
    case IR::OP_LOADMEM:
    case IR::OP_LOADMEMTSO: {
      auto Op = IROp->C<IR::IROp_LoadMem>();
      return Op->Class;
      break;
    }
    case IR::OP_FILLREGISTER: {
      auto Op = IROp->C<IR::IROp_FillRegister>();
      return Op->Class;
      break;
    }
    default: break;
    }

    // Unreachable
    return FEXCore::IR::InvalidClass;
  };
} // Anonymous namespace

class ConstrainedRAPass final : public RegisterAllocationPass {
public:
  ConstrainedRAPass(FEXCore::IR::Pass* _CompactionPass, bool SupportsAVX);
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
  FEXCore::IR::Pass* CompactionPass;
  bool SupportsAVX;
};

ConstrainedRAPass::ConstrainedRAPass(FEXCore::IR::Pass* _CompactionPass, bool _SupportsAVX)
  : CompactionPass {_CompactionPass}
  , SupportsAVX {_SupportsAVX} {}

ConstrainedRAPass::~ConstrainedRAPass() {
  delete Graph;
}

void ConstrainedRAPass::AllocateRegisterSet(uint32_t ClassCount) {
  LOGMAN_THROW_AA_FMT(ClassCount <= INVALID_CLASS, "Up to {} classes supported", INVALID_CLASS);

  Graph = AllocateRegisterGraph(ClassCount);
}

void ConstrainedRAPass::AddRegisters(FEXCore::IR::RegisterClassType Class, uint32_t RegisterCount) {
  LOGMAN_THROW_AA_FMT(RegisterCount <= INVALID_REG, "Up to {} regs supported", INVALID_REG);

  AllocatePhysicalRegisters(Graph, Class, RegisterCount);
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
        unsigned Reg = FindFirstSetBit(Class->Available) - 1;
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
  return fextl::make_unique<ConstrainedRAPass>(CompactionPass, SupportsAVX);
}
} // namespace FEXCore::IR
