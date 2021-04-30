#include "Common/Paths.h"
#include "Interface/Context/Context.h"
#include "Interface/Core/Core.h"
#include "Interface/Core/OpcodeDispatcher.h"

#include <FEXCore/Config/Config.h>
#include <FEXCore/Core/CoreState.h>
#include <FEXCore/Debug/X86Tables.h>

namespace FEXCore::Context {
  void InitializeStaticTables(OperatingMode Mode) {
    FEXCore::Paths::InitializePaths();
    X86Tables::InitializeInfoTables(Mode);
    IR::InstallOpcodeHandlers(Mode);
  }

  FEXCore::Context::Context *CreateNewContext() {
    return new FEXCore::Context::Context{};
  }

  bool InitializeContext(FEXCore::Context::Context *CTX) {
    return FEXCore::CPU::CreateCPUCore(CTX);
  }

  void DestroyContext(FEXCore::Context::Context *CTX) {
    if (CTX->ParentThread) {
      CTX->DestroyThread(CTX->ParentThread);
    }
    delete CTX;
  }

  bool InitCore(FEXCore::Context::Context *CTX, FEXCore::CodeLoader *Loader) {
    return CTX->InitCore(Loader);
  }

  void SetExitHandler(FEXCore::Context::Context *CTX,
      std::function<void(uint64_t ThreadId, FEXCore::Context::ExitReason)> handler) {
    CTX->CustomExitHandler = handler;
  }

  std::function<void(uint64_t ThreadId, FEXCore::Context::ExitReason)> GetExitHandler(FEXCore::Context::Context *CTX) {
    return CTX->CustomExitHandler;
  }

  void Run(FEXCore::Context::Context *CTX) {
    CTX->Run();
  }

  void Step(FEXCore::Context::Context *CTX) {
    CTX->Step();
  }

  void CompileRIP(FEXCore::Core::InternalThreadState *Thread, uint64_t GuestRIP) {
    Thread->CTX->CompileBlock(Thread->CurrentFrame, GuestRIP);
  }

  FEXCore::Context::ExitReason RunUntilExit(FEXCore::Context::Context *CTX) {
    return CTX->RunUntilExit();
  }

  int GetProgramStatus(FEXCore::Context::Context *CTX) {
    return CTX->GetProgramStatus();
  }

  FEXCore::Context::ExitReason GetExitReason(FEXCore::Context::Context *CTX) {
    return CTX->ParentThread->ExitReason;
  }

  bool IsDone(FEXCore::Context::Context *CTX) {
    return CTX->IsPaused();
  }

  void GetCPUState(FEXCore::Context::Context *CTX, FEXCore::Core::CPUState *State) {
    memcpy(State, CTX->ParentThread->CurrentFrame, sizeof(FEXCore::Core::CPUState));
  }

  void SetCPUState(FEXCore::Context::Context *CTX, FEXCore::Core::CPUState *State) {
    memcpy(CTX->ParentThread->CurrentFrame, State, sizeof(FEXCore::Core::CPUState));
  }

  void Pause(FEXCore::Context::Context *CTX) {
    CTX->Pause();
  }

  void Stop(FEXCore::Context::Context *CTX) {
    CTX->Stop(false);
  }

  void SetCustomCPUBackendFactory(FEXCore::Context::Context *CTX, CustomCPUFactoryType Factory) {
    CTX->CustomCPUFactory = std::move(Factory);
  }

  bool AddVirtualMemoryMapping([[maybe_unused]] FEXCore::Context::Context *CTX, [[maybe_unused]] uint64_t VirtualAddress, [[maybe_unused]] uint64_t PhysicalAddress, [[maybe_unused]] uint64_t Size) {
    return false;
  }

  void RegisterExternalSyscallVisitor(FEXCore::Context::Context *CTX, [[maybe_unused]] uint64_t Syscall, [[maybe_unused]] FEXCore::HLE::SyscallVisitor *Visitor) {
  }

  void HandleCallback(FEXCore::Context::Context *CTX, uint64_t RIP) {
    CTX->HandleCallback(RIP);
  }

  void RegisterHostSignalHandler(FEXCore::Context::Context *CTX, int Signal, HostSignalDelegatorFunction Func) {
      CTX->RegisterHostSignalHandler(Signal, Func);
  }

  void RegisterFrontendHostSignalHandler(FEXCore::Context::Context *CTX, int Signal, HostSignalDelegatorFunction Func) {
    CTX->RegisterFrontendHostSignalHandler(Signal, Func);
  }

  FEXCore::Core::InternalThreadState* CreateThread(FEXCore::Context::Context *CTX, FEXCore::Core::CPUState *NewThreadState, uint64_t ParentTID) {
    return CTX->CreateThread(NewThreadState, ParentTID);
  }

  void InitializeThread(FEXCore::Context::Context *CTX, FEXCore::Core::InternalThreadState *Thread) {
    return CTX->InitializeThread(Thread);
  }

  void RunThread(FEXCore::Context::Context *CTX, FEXCore::Core::InternalThreadState *Thread) {
    CTX->RunThread(Thread);
  }

  void StopThread(FEXCore::Context::Context *CTX, FEXCore::Core::InternalThreadState *Thread) {
    CTX->StopThread(Thread);
  }

  void DestroyThread(FEXCore::Context::Context *CTX, FEXCore::Core::InternalThreadState *Thread) {
    CTX->DestroyThread(Thread);
  }

  void CleanupAfterFork(FEXCore::Context::Context *CTX, FEXCore::Core::InternalThreadState *Thread) {
    CTX->CleanupAfterFork(Thread);
  }

  void SetSignalDelegator(FEXCore::Context::Context *CTX, FEXCore::SignalDelegator *SignalDelegation) {
    CTX->SignalDelegation = SignalDelegation;
  }

  void SetSyscallHandler(FEXCore::Context::Context *CTX, FEXCore::HLE::SyscallHandler *Handler) {
    CTX->SyscallHandler = Handler;
  }

  FEXCore::CPUID::FunctionResults RunCPUIDFunction(FEXCore::Context::Context *CTX, uint32_t Function, uint32_t Leaf) {
    return CTX->CPUID.RunFunction(Function, Leaf);
  }

  void SetAOTIRLoader(FEXCore::Context::Context *CTX, std::function<int(const std::string&)> CacheReader) {
    CTX->AOTIRLoader = CacheReader;
  }

  void SetAOTIRWriter(FEXCore::Context::Context *CTX, std::function<std::unique_ptr<std::ostream>(const std::string&)> CacheWriter) {
    CTX->AOTIRWriter = CacheWriter;
  }

  void FinalizeAOTIRCache(FEXCore::Context::Context *CTX) {
    CTX->FinalizeAOTIRCache();
  }

  void WriteFilesWithCode(FEXCore::Context::Context *CTX, std::function<void(const std::string& fileid, const std::string& filename)> Writer) {
    CTX->WriteFilesWithCode(Writer);
  }

  void AddNamedRegion(FEXCore::Context::Context *CTX, uintptr_t Base, uintptr_t Length, uintptr_t Offset, const std::string& Name) {
    return CTX->AddNamedRegion(Base, Length, Offset, Name);
  }
  void RemoveNamedRegion(FEXCore::Context::Context *CTX, uintptr_t Base, uintptr_t Length) {
    return CTX->RemoveNamedRegion(Base, Length);
  }

namespace Debug {
  void CompileRIP(FEXCore::Context::Context *CTX, uint64_t RIP) {
    CTX->CompileRIP(CTX->ParentThread, RIP);
  }
  uint64_t GetThreadCount(FEXCore::Context::Context *CTX) {
    return CTX->GetThreadCount();
  }

  FEXCore::Core::RuntimeStats *GetRuntimeStatsForThread(FEXCore::Context::Context *CTX, uint64_t Thread) {
    return CTX->GetRuntimeStatsForThread(Thread);
  }

  bool GetDebugDataForRIP(FEXCore::Context::Context *CTX, uint64_t RIP, FEXCore::Core::DebugData *Data) {
    return CTX->GetDebugDataForRIP(RIP, Data);
  }

  bool FindHostCodeForRIP(FEXCore::Context::Context *CTX, uint64_t RIP, uint8_t **Code) {
    return CTX->FindHostCodeForRIP(RIP, Code);
  }

  // XXX:
  // bool FindIRForRIP(FEXCore::Context::Context *CTX, uint64_t RIP, FEXCore::IR::IntrusiveIRList **ir) {
  //   return CTX->FindIRForRIP(RIP, ir);
  // }

  // void SetIRForRIP(FEXCore::Context::Context *CTX, uint64_t RIP, FEXCore::IR::IntrusiveIRList *const ir) {
  //   CTX->SetIRForRIP(RIP, ir);
  // }
}

}
