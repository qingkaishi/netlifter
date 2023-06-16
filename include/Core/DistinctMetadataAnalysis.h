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


#ifndef CORE_DISTINCTMETADATAANALYSIS_H
#define CORE_DISTINCTMETADATAANALYSIS_H

#include <llvm/IR/Module.h>
#include <llvm/Pass.h>
#include <set>

using namespace llvm;

class DistinctMetadataAnalysis : public ModulePass {
private:
    std::set<MDNode *> MDSet;
    std::map<MDNode *, MDNode *> MDRepMap;

public:
    static char ID;

    DistinctMetadataAnalysis() : ModulePass(ID) {}

    ~DistinctMetadataAnalysis() override = default;

    void getAnalysisUsage(AnalysisUsage &) const override;

    bool runOnModule(Module &) override;

    MDNode *getRep(MDNode *);

private:
    void runOnFunction(Function &);

    void collectMetadata(MDNode *N);

    void mergeDupDICompositeType();
};

#endif //CORE_DISTINCTMETADATAANALYSIS_H
