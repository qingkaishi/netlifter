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


#ifndef CORE_FSM_H
#define CORE_FSM_H

#include <llvm/Support/raw_os_ostream.h>
#include "Support/Z3.h"

using namespace llvm;

class SliceGraph;

class FSMState;

typedef std::shared_ptr<FSMState> FSMStateRef;

class FSMState {
private:
    int ID;
    std::vector<std::pair<z3::expr, FSMStateRef>> Transitions;

public:
    FSMState(int64_t ID) : ID(ID) {}

    void addTransition(const z3::expr &Condition, FSMStateRef);

    int id() const { return ID; }

    std::vector<std::pair<z3::expr, FSMStateRef>>::const_iterator
    begin() const { return Transitions.begin(); }

    std::vector<std::pair<z3::expr, FSMStateRef>>::const_iterator
    end() const { return Transitions.end(); }
};

class FSM {
private:
    std::map<int, FSMStateRef> ID2StateMap;

    void replaceWildcardState();

public:
    explicit FSM(const SliceGraph *);

    void dump(StringRef FileName);

public:
    friend raw_ostream &operator<<(llvm::raw_ostream &, const FSM &);
};

raw_ostream &operator<<(llvm::raw_ostream &, const FSM &);

#endif //CORE_FSM_H
