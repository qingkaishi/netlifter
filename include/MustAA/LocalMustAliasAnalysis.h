/*
 *  Popeye lifts protocol source code in C to its specification in BNF
 *  Copyright (C) 2023 Qingkai Shi <qingkaishi@gmail.com>
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


#ifndef MUSTAA_LOCALMUSTALIASANALYSIS_H
#define MUSTAA_LOCALMUSTALIASANALYSIS_H

#include <llvm/IR/Module.h>
#include <llvm/Pass.h>

#include "MustAA/XGraph.h"

using namespace llvm;

class LocalMustAliasAnalysis : public ModulePass {
private:
    std::map<Function *, std::shared_ptr<XGraph>> XGraphMap;

public:
    static char ID;

    LocalMustAliasAnalysis() : ModulePass(ID) {}

    ~LocalMustAliasAnalysis() override = default;

    void getAnalysisUsage(AnalysisUsage &) const override;

    bool runOnModule(Module &) override;

    /// test if two values are must-alias in a given function
    bool mustAlias(Function *, Value *, Value *);

private:
    void runOnFunction(Function &Root, std::set<Function *> &FuncSet);

    Function *findRoot(std::set<Function *> &FuncSet);
};

#endif //MUSTAA_LOCALMUSTALIASANALYSIS_H
