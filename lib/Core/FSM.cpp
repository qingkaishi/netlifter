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

#include "Core/FSM.h"
#include "Core/SliceGraph.h"
#include "Support/Debug.h"

FSM::FSM(const SliceGraph *G) {
    //FSMRef RetFSM = std::make_shared<FSM>();
    std::set<SliceGraphNode *> AllNodes;
    G->dfs([&AllNodes](SliceGraphNode *Node) {
        AllNodes.insert(Node);
    });

    // create states
    std::vector<SliceGraphNode *> StateNodes;
    for (auto *Node: AllNodes) {
        auto Expr = Node->getCondition();
        int64_t StateId;
        if (Expr.decl().decl_kind() == Z3_OP_EQ
            && Z3::is_state(Expr.arg(0))
            && Z3::is_numeral_i64(Expr.arg(1), StateId)) {
            if (!this->ID2StateMap.count(StateId))
                this->ID2StateMap[StateId] = std::make_shared<FSMState>(StateId);
            StateNodes.push_back(Node);
        } else if (Z3::is_transit_to(Expr) && Z3::is_numeral_i64(Expr.arg(0), StateId)) {
            if (!this->ID2StateMap.count(StateId))
                this->ID2StateMap[StateId] = std::make_shared<FSMState>(StateId);
        }
    }

    // create state transitions
    // 1. forward slicing, and collect reachable nodes and transit-to nodes
    // 2. use the forward slice to collect constraints to each transit-to node
    for (auto *StateNode: StateNodes) {
        int64_t SrcStateId;
        FSMStateRef SrcState;
        if (Z3::is_numeral_i64(StateNode->getCondition().arg(1), SrcStateId)) {
            SrcState = this->ID2StateMap.at(SrcStateId);
        } else {
            llvm_unreachable("what... not possible...");
        }

        std::vector<SliceGraphNode *> Targets;
        G->dfs([&Targets, SrcStateId](SliceGraphNode *Node) {
            auto Expr = Node->getCondition();
            int64_t StateId;
            if (Z3::is_transit_to(Expr) && Z3::is_numeral_i64(Expr.arg(0), StateId) && StateId != SrcStateId) {
                // StateId != SrcStateId (all self-cycles should be implicit)
                Targets.emplace_back(Node);
            }
        }, StateNode);

        z3::expr_vector CondVec = G->collectConstraints(StateNode, Targets, true);
        for (unsigned K = 0; K < Targets.size(); ++K) {
            auto *Target = Targets[K];
            int64_t TargetId;
            auto Expr = Target->getCondition();
            if (Z3::is_transit_to(Expr) && Z3::is_numeral_i64(Expr.arg(0), TargetId)) {
                SrcState->addTransition(CondVec[K], this->ID2StateMap.at(TargetId));
            } else {
                llvm_unreachable("what... not possible...");
            }
        }
    }

    this->replaceWildcardState();
}

void FSM::replaceWildcardState() {
    // state with id -1, should be replaced by meaningful states
    auto It = ID2StateMap.find(-1);
    if (It == ID2StateMap.end()) return;
    auto WildcardState = It->second;
    ID2StateMap.erase(It); // remove it

    for (auto &StatePair : ID2StateMap) {
        auto State = StatePair.second;
        for (auto &Ch: *WildcardState) {
            if (Ch.second->id() == -1) continue;
            State->addTransition(Ch.first, Ch.second);
        }
    }
}

void FSM::dump(llvm::StringRef FileName) {
    std::error_code EC;
    raw_fd_ostream PStream(FileName.str(), EC, sys::fs::F_None);
    if (PStream.has_error()) {
        errs() << "[Error] Cannot open the file <" << FileName << "> for writing.\n";
        return;
    }
    PStream << *this << "\n";
    POPEYE_INFO(FileName << " dumped!");
}

raw_ostream &operator<<(raw_ostream &DotStream, const FSM &Machine) {
    auto DotState = [](FSMStateRef State, raw_ostream &OS) {
        OS << "\tstate_" << State->id() << "[label=\"" << State->id() << "\"];\n";
        for (auto &Ch: *State) {
            OS << "\tstate_" << State->id() << "->state_" << Ch.second->id()
               << "[label=\"" << Z3::to_string(Ch.first) << "\"];\n";
        }
        OS << "\n";
    };

    DotStream << "\n";
    DotStream << "digraph machine {\n";
    for (auto &It: Machine.ID2StateMap) {
        DotState(It.second, DotStream);
    }
    DotStream << "}\n";
    return DotStream;
}

void FSMState::addTransition(const z3::expr &Condition, FSMStateRef State) {
    if (this->id() == State->id()) return; // omit self-cycle
    Transitions.emplace_back(Condition, State);
}
