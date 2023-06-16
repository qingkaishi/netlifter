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

#include "Core/FunctionMap.h"

std::map<Function *, size_t> FunctionMap::Func2ID;
std::vector<Function *> FunctionMap::ID2Func;

Function *FunctionMap::getFunction(size_t ID) {
    if (ID < ID2Func.size())
        return ID2Func[ID];
    return nullptr;
}

size_t FunctionMap::getFunctionID(Function *Func) {
    auto It = Func2ID.find(Func);
    if (It != Func2ID.end()) {
        return It->second;
    } else {
        auto ID = ID2Func.size();
        Func2ID[Func] = ID2Func.size();
        ID2Func.push_back(Func);
        return ID;
    }
}