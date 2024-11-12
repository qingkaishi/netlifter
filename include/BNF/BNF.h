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

#ifndef BNF_BNF_H
#define BNF_BNF_H

#include <llvm/Support/Casting.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <list>
#include "BNF/Bound.h"
#include "Support/Z3.h"

using namespace llvm;

class BNF;

class RHSItem {
public:
    enum RHSKind {
        RK_Interval,
        RK_Production
    };

private:
    RHSKind Kind;

public:
    explicit RHSItem(RHSKind K) : Kind(K) {}

    virtual ~RHSItem() = 0;

    RHSKind getKind() const { return Kind; }
};

class Interval : public RHSItem {
private:
    BoundRef From;
    BoundRef To;

public:
    Interval(const BoundRef &F, const BoundRef &T) : RHSItem(RK_Interval), From(F), To(T) {}

    ~Interval() override = default;

    BoundRef getFrom() const { return From; }

    BoundRef getTo() const { return To; }

    void setFrom(BoundRef &F) { From = F; }

    void setTo(BoundRef &T) { To = T; }

public:
    static bool classof(const RHSItem *M) {
        return M->getKind() == RK_Interval;
    }
};

class Production : public RHSItem {
    /// the id of this production, standing for a non-terminal symbol on the left-hand side of the production
    /// may be printed as L_LHS
    unsigned LHS;

    /// the id for printing a production L_LHS := xxx. we print it as L_LHS4Print := xxx, if L_LHS4Print := L_LHS | ...
    /// this is just to simplify the output BNF by eliminating unnecessary productions like L1 := L2 | L3 | L4 | ...
    unsigned LHS4Print;

    /// the right-hand side is a disjunction of multiple cases (std::list)
    /// each case is a sequence of RHSItem (std::vector)
    std::list<std::vector<RHSItem *>> RHS;

    /// the assertions associated with this production
    z3::expr_vector Assertions;

    /// this is an 'L -> epsilon' production
    bool Epsilon = false;

public:
    Production(const z3::expr &, BNF *);

    ~Production() override = default;

    void releaseMemory() {
        for (auto &Vec: RHS) for (auto *Item: Vec) if (!isa<Production>(Item)) delete Item;
    }

    void setLHS(unsigned L) { LHS = L; }

    unsigned getLHS() const { return LHS; }

    std::list<std::vector<RHSItem *>>::const_iterator rhs_begin() const { return RHS.begin(); }

    std::list<std::vector<RHSItem *>>::const_iterator rhs_end() const { return RHS.end(); }

    friend raw_ostream &operator<<(llvm::raw_ostream &, const Production &);

    bool nonTerminalDisjunction() const;

    bool containsNonTerminal() const;

    unsigned getNumAssertions() const { return Assertions.size(); }

    void setEpsilon(bool E = true) { Epsilon = E; }

    bool isEpsilon() const { return Epsilon; }

private:
    friend class BNF;
    friend class DDLLang;

public:
    static bool classof(const RHSItem *M) {
        return M->getKind() == RK_Production;
    }
};

struct ProductionLessThan {
    bool operator()(const Production *P1, const Production *P2) const {
        return P1->getLHS() > P2->getLHS();
    }
};

class BNF {
    std::set<Production *, ProductionLessThan> Productions;
    std::set<Production *> FalseProductions;

public:
    explicit BNF(const z3::expr &);

    ~BNF();

    void add(Production *P) { Productions.insert(P); }

    void dump(StringRef FileName);

    auto getProductions(){return Productions;}

private:
    void pad();

    void padHead(Production *, BoundRef);

    void padHead(RHSItem *, BoundRef);

    void padTail(RHSItem *, BoundRef);

    void simplify();

    void simplifyByCheckingConflict();

    void simplifyByMergingProductions();

    void simplifyByRemovingUnusedProductions();

    void simplifyByMappingProductionNames();

    void simplifyByMergingFieldName();

    void simplifyByMergingFieldConstraint();

    void simplifyByMergingField();

    int getEndianness(Production *P);

    friend raw_ostream &operator<<(llvm::raw_ostream &, const BNF &);
};

raw_ostream &operator<<(llvm::raw_ostream &, const BNF &);

raw_ostream &operator<<(llvm::raw_ostream &, const Production &);

#endif //BNF_BNF_H
