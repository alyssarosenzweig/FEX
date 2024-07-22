// SPDX-License-Identifier: MIT
/*
$info$
meta: ir|emitter ~ C++ Functions to generate IR. See IR.json for spec.
tags: ir|emitter
$end_info$
*/

#include "Interface/IR/IREmitter.h"

#include <FEXCore/IR/IR.h>
#include <FEXCore/Utils/EnumUtils.h>
#include <FEXCore/Utils/LogManager.h>

#include <array>
#include <stdint.h>
#include <string.h>

namespace FEXCore::IR {

bool IsFragmentExit(FEXCore::IR::IROps Op) {
  switch (Op) {
  case OP_EXITFUNCTION:
  case OP_BREAK: return true;
  default: return false;
  }
}

bool IsBlockExit(FEXCore::IR::IROps Op) {
  switch (Op) {
  case OP_JUMP:
  case OP_CONDJUMP: return true;
  default: return IsFragmentExit(Op);
  }
}

FEXCore::IR::RegisterClassType IREmitter::WalkFindRegClass(Ref Node) {
  auto Class = GetOpRegClass(Node);
  switch (Class) {
  case GPRClass:
  case GPRPairClass:
  case FPRClass:
  case GPRFixedClass:
  case FPRFixedClass:
  case InvalidClass: return Class;
  default: break;
  }

  // Complex case, needs to be handled on an op by op basis
  uintptr_t DataBegin = DualListData.DataBegin();

  FEXCore::IR::IROp_Header* IROp = Node->Op(DataBegin);

  switch (IROp->Op) {
  case IROps::OP_LOADREGISTER: {
    auto Op = IROp->C<IROp_LoadRegister>();
    return Op->Class;
    break;
  }
  case IROps::OP_LOADCONTEXT: {
    auto Op = IROp->C<IROp_LoadContext>();
    return Op->Class;
    break;
  }
  case IROps::OP_LOADCONTEXTINDEXED: {
    auto Op = IROp->C<IROp_LoadContextIndexed>();
    return Op->Class;
    break;
  }
  case IROps::OP_FILLREGISTER: {
    auto Op = IROp->C<IROp_FillRegister>();
    return Op->Class;
    break;
  }
  case IROps::OP_LOADMEM: {
    auto Op = IROp->C<IROp_LoadMem>();
    return Op->Class;
    break;
  }
  case IROps::OP_LOADMEMTSO: {
    auto Op = IROp->C<IROp_LoadMemTSO>();
    return Op->Class;
    break;
  }
  default: LOGMAN_MSG_A_FMT("Unhandled op type: {} {} in argument class validation", ToUnderlying(IROp->Op), GetOpName(Node)); break;
  }
  return InvalidClass;
}

void IREmitter::ResetWorkingList() {
  CodeBlocks.clear();
  CurrentWriteCursor = nullptr;
  CurrentCodeBlock = nullptr;
}

IREmitter::IRPair<IROp_CodeBlock> IREmitter::CreateNewCodeBlockAfter(Ref insertAfter) {
  auto OldCursor = GetWriteCursor();

  auto CodeNode = CreateCodeNode();

  if (insertAfter) {
    LinkCodeBlocks(insertAfter, CodeNode);
  } else {
    LOGMAN_THROW_AA_FMT(CurrentCodeBlock != nullptr, "CurrentCodeBlock must not be null here");

    // Find last block
    auto LastBlock = CurrentCodeBlock;

    while (LastBlock->Header.Next.GetNode(DualListData.ListBegin()) != InvalidNode) {
      LastBlock = LastBlock->Header.Next.GetNode(DualListData.ListBegin());
    }

    // Append it after the last block
    LinkCodeBlocks(LastBlock, CodeNode);
  }

  SetWriteCursor(OldCursor);

  return CodeNode;
}

void IREmitter::SetCurrentCodeBlock(Ref Node) {
  CurrentCodeBlock = Node;
  SetWriteCursor(Node->Op(DualListData.DataBegin())->CW<IROp_CodeBlock>()->Begin.GetNode(DualListData.ListBegin()));
}

} // namespace FEXCore::IR
