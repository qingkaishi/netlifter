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

#include <llvm/Support/CommandLine.h>
#include <llvm/Support/ManagedStatic.h>
#include <vector>
#include "Support/Debug.h"

using namespace llvm;

bool PopeyeDebugFlag = false;

static ManagedStatic<std::vector<std::string>> PopeyeCurrentDebugType;

bool isPopeyeCurrentDebugType(const char *DebugType) {
    if (PopeyeCurrentDebugType->empty())
        return true; // debug everything

    for (auto &D: *PopeyeCurrentDebugType)
        if (D == DebugType)
            return true;
    return false;
}

struct PopeyeDebugOpt {
    void operator=(const std::string &Val) const {
        if (Val.empty())
            return;
        PopeyeDebugFlag = true;
        SmallVector<StringRef, 8> DbgTypes;
        StringRef(Val).split(DbgTypes, ',', -1, false);
        for (auto DbgType: DbgTypes)
            PopeyeCurrentDebugType->push_back(std::string(DbgType));
    }
};

static PopeyeDebugOpt DebugOptLoc;

static cl::opt<PopeyeDebugOpt, true, cl::parser<std::string>>
        DebugOnly("popeye-debug", cl::desc("Enable a specific type of debug output (comma separated list of types)"),
                  cl::Hidden, cl::ZeroOrMore, cl::value_desc("debug string"),
                  cl::location(DebugOptLoc), cl::ValueRequired);