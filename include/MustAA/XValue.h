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


#ifndef MUSTAA_XVALUE_H
#define MUSTAA_XVALUE_H

#include <llvm/IR/Value.h>
#include <map>

#include "MustAA/XNode.h"

using namespace llvm;

/// wrapper of an llvm value that allows us to name *val， &val
class XValue {
public:
    enum ModifierKind {
        MK_None,
        MK_Dereference,
        MK_AddressTaken
    };

private:
    Value *Val;
    ModifierKind Modifier;

public:
    XValue(Value *V, ModifierKind M) : Val(V), Modifier(M) {}

private:
    static std::map<std::pair<Value *, ModifierKind>, XValue *> XValueMap;

public:
    static XValue *getXValue(Value *, ModifierKind);
};

#endif //MUSTAA_XVALUE_H
