/*
 *  Popeye lifts protocol source code in C to its specification in BNF
 *  Copyright (C) 2022 Qingkai Shi <qingkaishi@gmail.com>
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


#ifndef BNF_SLICEGRAPH_H
#define BNF_SLICEGRAPH_H

#include "Support/Z3.h"

class SliceGraph;

class SliceGraphNode {
    friend class SliceGraph;

private:
    /// @{
    z3::expr Condition;
    unsigned ConditionID;
    /// @}

    /// @{
    const char *Color;
    std::string Hint;
    /// @}

    std::set<SliceGraphNode *> Children;
    std::set<SliceGraphNode *> Parents;

public:
    SliceGraphNode(z3::expr Cond) : Condition(Cond), ConditionID(Z3::id(Cond)), Color(0) {}

    z3::expr getCondition() const { return Condition; }

    void setCondition(const z3::expr &C) { Condition = C; }

    unsigned getConditionID() const { return ConditionID; };

    void setConditionID(unsigned ID) { ConditionID = ID; }

    const char *getColor() const { return Color; }

    void setColor(const char *C) { Color = C; }

    const std::string &getHint() const { return Hint; }

    void setHint(std::string H) { Hint = H; }

    void addChild(SliceGraphNode *N) { Children.insert(N); }

    void addParent(SliceGraphNode *N) { Parents.insert(N); }

    unsigned getNumChildren() const { return Children.size(); }

    unsigned getNumParents() const { return Parents.size(); }

    std::set<SliceGraphNode *>::const_iterator child_begin() const { return Children.begin(); }

    std::set<SliceGraphNode *>::const_iterator child_end() const { return Children.end(); }

    std::set<SliceGraphNode *>::const_iterator parent_begin() const { return Parents.begin(); }

    std::set<SliceGraphNode *>::const_iterator parent_end() const { return Parents.end(); }
};

class SliceGraph {
private:
    std::set<SliceGraphNode *> Entries;
    std::set<SliceGraphNode *> Exits;

public:
    ~SliceGraph();

    void dot(std::string &File, const char * = "") const;

    unsigned size() const;

    void simplifyBeforeSymbolicExecution();

    void simplifyAfterSymbolicExecution();

    void simplifyByRewriting();

    void simplifyByRemoving();

    void simplifyByRemovingNoSelect();

    void simplifyByMerging();

    void simplifyByHashConsing();

    z3::expr pc();

    std::set<SliceGraphNode *>::iterator entry_begin() { return Entries.begin(); }

    std::set<SliceGraphNode *>::iterator entry_end() { return Entries.end(); }

    template<class ActionAtDFS>
    void dfs(ActionAtDFS Act, SliceGraphNode *From = nullptr) const {
        std::vector<SliceGraphNode *> Stack;
        std::set<SliceGraphNode *> Visited;
        if (From)
            Stack.push_back(From);
        else
            for (auto *En: Entries) Stack.push_back(En);
        while (!Stack.empty()) {
            auto *Top = Stack.back();
            Stack.pop_back();
            if (Visited.count(Top)) continue;
            Visited.insert(Top);

            Act(Top);

            for (auto *Ch: Top->Children) {
                Stack.push_back(Ch);
            }
        }
    }

    z3::expr_vector collectConstraints(SliceGraphNode *From, std::vector<SliceGraphNode *> &Tos, bool ExFrom) const;

private:
    z3::expr simplify(const z3::expr &) const;

    void remove(SliceGraphNode *, bool PreserveReachability);

    void remove(SliceGraphNode *, bool PreserveReachability, std::set<SliceGraphNode *> *Deleted);

    void postOrder(std::vector<SliceGraphNode *> &) const;

    void postOrder(SliceGraphNode *, std::set<SliceGraphNode *> &, std::vector<SliceGraphNode *> &) const;

    void topoOrder(std::vector<SliceGraphNode *> &);

    std::vector<z3::expr> merge(std::vector<std::vector<z3::expr>> &) const;

    void validate(SliceGraphNode *) const;

    void removeFrom(SliceGraphNode *, std::set<SliceGraphNode *> &Removed);

public:
    static SliceGraph *get(const z3::expr PC, bool AllExpanded = false);

private:
    static SliceGraph *getAnd(const z3::expr &And);

    static SliceGraph *getOr(const z3::expr &Or);

    static SliceGraph *getUnknown(const z3::expr &Unk);

    static SliceGraph *getGraph(const z3::expr &Any);
};

#endif //BNF_SLICEGRAPH_H
