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

#ifndef BNF_BOUND_H
#define BNF_BOUND_H

#include <llvm/Support/Casting.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include "Support/Z3.h"

using namespace llvm;

class Bound;

typedef std::shared_ptr<Bound> BoundRef;

class Bound {
public:
    enum BoundKind {
        BK_Constant,
        BK_Symbolic,
        BK_Upper
    };

private:
    BoundKind Kind;

public:
    explicit Bound(BoundKind BK) : Kind(BK) {}

    BoundKind getKind() const { return Kind; }

    virtual z3::expr expr() const { llvm_unreachable("Error: Not supported!"); }

    virtual int64_t constant() const { llvm_unreachable("Error: Not supported!"); }

    static BoundRef createBound(const z3::expr &);

    static BoundRef createBound(int64_t);
};

class ConstantBound : public Bound {
private:
    int64_t Constant;

public:
    explicit ConstantBound(int64_t B) : Bound(BK_Constant), Constant(B) {}

    int64_t constant() const override { return Constant; }

public:
    static bool classof(const Bound *M) {
        return M->getKind() == BK_Constant;
    }
};

class SymbolicBound : public Bound {
private:
    z3::expr Symbolic;

public:
    explicit SymbolicBound(const z3::expr &E) : Bound(BK_Symbolic), Symbolic(E) {}

    z3::expr expr() const override { return Symbolic; }

public:
    static bool classof(const Bound *M) {
        return M->getKind() == BK_Symbolic;
    }
};

class UpperBound : public Bound {
public:
    UpperBound() : Bound(BK_Upper) {}

public:
    static bool classof(const Bound *M) {
        return M->getKind() == BK_Upper;
    }
};

raw_ostream &operator<<(llvm::raw_ostream &, const Bound &);

bool operator==(const Bound &, const Bound &);

bool operator!=(const Bound &, const Bound &);

bool operator<(const Bound &, const Bound &);

bool operator>=(const Bound &, const Bound &);

bool operator>(const Bound &, const Bound &);

bool operator<=(const Bound &, const Bound &);

BoundRef operator+(const Bound &, int);

BoundRef operator-(const Bound &, int);

raw_ostream &operator<<(llvm::raw_ostream &, const BoundRef &);

bool operator==(const BoundRef &, const BoundRef &);

bool operator!=(const BoundRef &, const BoundRef &);

bool operator<(const BoundRef &, const BoundRef &);

bool operator>=(const BoundRef &, const BoundRef &);

bool operator>(const BoundRef &, const BoundRef &);

bool operator<=(const BoundRef &, const BoundRef &);

BoundRef operator+(const BoundRef &, int);

BoundRef operator-(const BoundRef &, int);

class BoundVec;

typedef std::shared_ptr<BoundVec> BoundVecRef;

class BoundVec {
public:
    z3::expr Op;
    std::vector<BoundRef> Vec;
    std::vector<BoundRef> MinVec;
    std::vector<BoundRef> MaxVec;

private:
    /// for initialization of a vector of bound
    /// @{
    BoundVec(const z3::expr &Op) : Op(Op) {}

    void push_back(const BoundRef &BR);
    /// @}

public:
    BoundRef &back() { return Vec.back(); }

    std::vector<BoundRef>::iterator begin() { return Vec.begin(); }

    std::vector<BoundRef>::iterator end() { return Vec.end(); }

    BoundRef &operator[](unsigned I) { return Vec[I]; }

    const BoundRef &operator[](unsigned I) const { return Vec[I]; }

    size_t size() const { return Vec.size(); }

    void merge(BoundVecRef From);

public:
    /// create a vector of bound from a first-order-logic formula (usually a path constraint)
    static BoundVecRef createBoundVecRef(const z3::expr &);
};

#endif //BNF_BOUND_H
