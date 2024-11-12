/*
 *  Popeye lifts protocol source code in C to its specification in BNF
 *  Copyright (C) 2021 Qingkai Shi <qingkaishi@gmail.com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <llvm/IR/Constants.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/Format.h>

#include "BNF/BNF.h"
#include "Core/DistinctMetadataAnalysis.h"
#include "Core/DomInformationAnalysis.h"
#include "Core/Executor.h"
#include "Core/FSM.h"
#include "Core/LoopInformationAnalysis.h"
#include "Core/PLang.h"
#include "Core/DDLLang.h"
#include "Core/SliceGraph.h"
#include "Core/SymbolicExecution.h"
#include "Core/SymbolicExecutionTree.h"
#include "LiftingPass.h"
#include "Support/Debug.h"
#include "Support/DL.h"
#include "Support/TimeRecorder.h"
#include "Support/Z3.h"

#define DEBUG_TYPE "LiftingPass"

using namespace llvm;

static cl::opt<std::string> EntryFunctionName("popeye-entry",
                                              cl::desc("specify from which function we start our analysis"),
                                              cl::init("popeye_main"));

static cl::list<std::string> EnableOutputs("popeye-output",
                                           cl::desc("bnf[:file] | fsm:file | p:file | dot:file | ddl:file"),
                                           cl::ZeroOrMore);

char LiftingPass::ID = 0;
static RegisterPass<LiftingPass> X(DEBUG_TYPE, "The core engine of Popeye.");

void LiftingPass::getAnalysisUsage(AnalysisUsage &AU) const {
    AU.setPreservesAll();
    AU.addRequired<LoopInformationAnalysis>();
    AU.addRequired<DomInformationAnalysis>();
    AU.addRequired<DistinctMetadataAnalysis>();
}

bool LiftingPass::runOnModule(Module &M) {
    DL::initialize(M.getDataLayout());
    Z3::initialize();

    checkBuiltInFunctions(M);
    auto *Entry = M.getFunction(EntryFunctionName.getValue());
    if (!Entry) {
        std::string ErrorMsg("The entry function --- ");
        ErrorMsg.append(EntryFunctionName.getValue());
        ErrorMsg.append(" --- is not found!");
        POPEYE_WARN(ErrorMsg.c_str());

        bool MultipleEntries = false;
        for (auto &F: M) {
            if (!F.isDeclaration() && F.getName().startswith("popeye_main")) {
                if (Entry) MultipleEntries = true;
                if (!Entry) Entry = &F;
                POPEYE_WARN("\tPossible entry --- " + F.getName().str());
            }
        }
        if (MultipleEntries || !Entry) exit(1);
    }

    // prepare output options
    std::string OutputPFile = "";
    std::string OutputDotFile = "";
    std::string OutputBNFFile = "";
    std::string OutputFSMFile = "";
    std::string OutputDDLFile = ""; //Daedalus dsl
    for (auto &Op: EnableOutputs) {
        StringRef OpStr(Op);
        if (OpStr.startswith("p:")) {
            OutputPFile = OpStr.substr(strlen("p:")).str();
        } else if (OpStr.startswith("dot:")) {
            OutputDotFile = OpStr.substr(strlen("dot:")).str();
        } else if (OpStr.startswith("bnf")) { // not "bnf:" for use simplicity ... we often do not print file
            OutputBNFFile = OpStr.substr(strlen("bnf:")).str();
            if (OutputBNFFile.empty()) OutputBNFFile = "-";
        } else if (OpStr.startswith("fsm:")) {
            OutputFSMFile = OpStr.substr(strlen("fsm:")).str();
        } else if (OpStr.startswith("ddl:")) {
            OutputDDLFile = OpStr.substr(strlen("ddl:")).str();
        }
    }

    // start the analysis
    TimeRecorder Timer("Analyzing the input llvm bitcode");
    auto PC = Z3::bool_val(true);
    {
        TimeRecorder AITimer("Step 1: Abstract interpreting the code");
        Executor Exe(this);
        Exe.visit(Entry);
        PC = Exe.getPC();
    }

    {
        TimeRecorder SETimer("Step 2: Executing on the slice");
        SliceGraph *Slice = SliceGraph::get(PC);
        assert(Slice);
        if (!OutputDotFile.empty()) Slice->dot(OutputDotFile, "graph");
        Slice->simplifyBeforeSymbolicExecution();
        if (!OutputDotFile.empty()) Slice->dot(OutputDotFile, "graph.simplify");
        SymbolicExecutionTree *Tree = SymbolicExecution().run(PC, *Slice);
        delete Slice;
        Tree->simplify();
        if (!OutputDotFile.empty()) Tree->dot(OutputDotFile, "tree");
        PC = Tree->pc();
        delete Tree;
    }

    {
        TimeRecorder FinalTimer("Step 3: Generating final results");
        auto PhiVec = Z3::find_all(PC, true, [](const z3::expr &E) { return Z3::is_phi(E); });
        assert(PhiVec.empty());
        auto *NewSlice = SliceGraph::get(PC, true);
        NewSlice->simplifyAfterSymbolicExecution();
        if (!OutputDotFile.empty()) NewSlice->dot(OutputDotFile, "final");
        if (!OutputBNFFile.empty()) BNF(NewSlice->pc()).dump(OutputBNFFile);
        if (!OutputFSMFile.empty()) FSM(NewSlice).dump(OutputFSMFile);
        if (!OutputPFile.empty()) PLang(NewSlice, guessEntryName(Entry)).dump(OutputPFile);
        if (!OutputDDLFile.empty()) 
        {
            BNF b(NewSlice->pc());
            DDLLang d(&b);
            // for(auto P : d.Bnf->Productions)
            // {
            //     d.Production2DDL(*P);
            //     errs()<<"\n";
            // }
            d.dump(OutputDDLFile);
        }
        delete NewSlice;
    }

    DL::finalize();
    Z3::finalize();
    return false;
}

void LiftingPass::checkBuiltInFunctions(Module &M) {
    /*
     * void *popeye_make_object(uint64_t size);
     * void *popeye_make_named_object(uint64_t size, const char *);
     * void popeye_make_global(void *);
     * void *popeye_make_message(void);
     * uint16_t popeye_make_message_length(void);
     */

    auto DeleteBody = [](Function *F) {
        if (!F->empty()) {
            F->deleteBody();
            POPEYE_WARN("The body of " << F->getName() << " not empty, deleted!");
        }
    };

    auto *PopeyeMakeObject = M.getFunction("popeye_make_object");
    if (PopeyeMakeObject) {
        assert(PopeyeMakeObject->getReturnType()->isPointerTy());
        assert(PopeyeMakeObject->arg_size() == 1);
        assert(PopeyeMakeObject->getArg(0)->getType()->isIntegerTy());
        DeleteBody(PopeyeMakeObject);
    }

    auto *PopeyeMakeNamedObject = M.getFunction("popeye_make_named_object");
    if (PopeyeMakeNamedObject) {
        assert(PopeyeMakeNamedObject->getReturnType()->isPointerTy());
        assert(PopeyeMakeNamedObject->arg_size() == 2);
        assert(PopeyeMakeNamedObject->getArg(0)->getType()->isIntegerTy());
        assert(PopeyeMakeNamedObject->getArg(1)->getType()->isPointerTy());
        DeleteBody(PopeyeMakeNamedObject);
    }

    auto *PopeyeMakeMessage = M.getFunction("popeye_make_message");
    if (PopeyeMakeMessage) {
        assert(PopeyeMakeMessage->getReturnType()->isPointerTy());
        assert(PopeyeMakeMessage->arg_size() == 0);
        DeleteBody(PopeyeMakeMessage);
    }

    auto *PopeyeMakeMessageLength = M.getFunction("popeye_make_message_length");
    if (PopeyeMakeMessageLength) {
        assert(PopeyeMakeMessageLength->getReturnType()->isIntegerTy());
        assert(PopeyeMakeMessageLength->arg_size() == 0);
        DeleteBody(PopeyeMakeMessageLength);
    }

    auto *PopeyeMakeGlobal = M.getFunction("popeye_make_global");
    if (PopeyeMakeGlobal) {
        assert(PopeyeMakeGlobal->getReturnType()->isVoidTy());
        assert(PopeyeMakeGlobal->arg_size() == 1);
        assert(PopeyeMakeGlobal->getArg(0)->getType()->isPointerTy());
        DeleteBody(PopeyeMakeGlobal);
    }

    auto *PopeyeName = M.getFunction("popeye_name");
    if (PopeyeName) {
        assert(PopeyeName->getReturnType()->isVoidTy());
        assert(PopeyeName->arg_size() == 2);
        assert(PopeyeName->getArg(0)->getType()->isIntegerTy());
        assert(PopeyeName->getArg(1)->getType()->isPointerTy());
        DeleteBody(PopeyeName);
    }
}

std::string LiftingPass::guessEntryName(Function *F) {
    if (!F->getName().startswith("popeye_")) return "";

    std::vector<StringRef> CalleeVec;
    for (auto &B: *F) {
        for (auto &I: B) {
            if (auto *CI = dyn_cast<CallInst>(&I)) {
                if (auto *Callee = CI->getCalledFunction()) {
                    if (!Callee->empty()) CalleeVec.push_back(Callee->getName());
                }
            }
        }
    }
    for (unsigned K = CalleeVec.size(); K > 0; --K) {
        StringRef Callee = CalleeVec[K - 1];
        if (Callee.contains("printf")
            || Callee.startswith("popeye_")
            || Callee.contains("malloc")
            || Callee.startswith("llvm."))
            continue;
        return Callee.str();
    }
    return "";
}
