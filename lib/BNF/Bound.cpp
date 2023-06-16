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

#include <llvm/Support/CommandLine.h>
#include <llvm/Support/Debug.h>
#include "BNF/Bound.h"
#include "Support/Z3.h"

#define DEBUG_TYPE "Bound"

static bool lessThan(const z3::expr &A, const z3::expr &B) {
    return Z3::byte_array_element_index_less_than(A, B);
}

raw_ostream &operator<<(llvm::raw_ostream &O, const Bound &B) {
    if (auto *CB = dyn_cast<ConstantBound>(&B)) {
        if (CB->constant() >= 0) O << CB->constant();
    } else if (auto *SB = dyn_cast<SymbolicBound>(&B)) {
        O << SB->expr();
    } else {
        O << "";
    }
    return O;
}

bool operator<(const Bound &B1, const Bound &B2) {
    if (isa<UpperBound>(&B1)) return false;
    if (isa<UpperBound>(&B2)) return true;

    if (isa<ConstantBound>(&B1) && isa<ConstantBound>(&B2)) {
        return B1.constant() < B2.constant();
    } else if (isa<ConstantBound>(&B1) && isa<SymbolicBound>(&B2)) {
        return lessThan(Z3::bv_val(B1.constant(), 64), B2.expr());
    } else if (isa<SymbolicBound>(&B1) && isa<ConstantBound>(&B2)) {
        return lessThan(B1.expr(), Z3::bv_val(B2.constant(), 64));
    } else {
        return lessThan(B1.expr(), B2.expr());
    }
}

bool operator==(const Bound &B1, const Bound &B2) {
    return !(B1 < B2) && !(B2 < B1);
}

bool operator!=(const Bound &B1, const Bound &B2) {
    return B1 < B2 || B2 < B1;
}

bool operator>=(const Bound &B1, const Bound &B2) {
    return !(B1 < B2);
}

bool operator>(const Bound &B1, const Bound &B2) {
    return B2 < B1;
}

bool operator<=(const Bound &B1, const Bound &B2) {
    return !(B2 < B1);
}

BoundRef operator+(const Bound &B, int X) {
    if (auto *CB = dyn_cast<ConstantBound>(&B)) {
        return Bound::createBound(CB->constant() + X);
    } else if (auto *SB = dyn_cast<SymbolicBound>(&B)) {
        return Bound::createBound((SB->expr() + X).simplify());
    } else {
        llvm_unreachable("Error : unsupported!");
    }
}

BoundRef operator-(const Bound &B, int X) {
    if (auto *CB = dyn_cast<ConstantBound>(&B)) {
        return Bound::createBound(CB->constant() - X);
    } else if (auto *SB = dyn_cast<SymbolicBound>(&B)) {
        return Bound::createBound((SB->expr() - X).simplify());
    } else {
        llvm_unreachable("Error : unsupported!");
    }
}

raw_ostream &operator<<(llvm::raw_ostream &O, const BoundRef &B) {
    O << *B;
    return O;
}

bool operator==(const BoundRef &B1, const BoundRef &B2) {
    return *B1 == *B2;
}

bool operator!=(const BoundRef &B1, const BoundRef &B2) {
    return *B1 != *B2;
}

bool operator<(const BoundRef &B1, const BoundRef &B2) {
    return *B1 < *B2;
}

bool operator>=(const BoundRef &B1, const BoundRef &B2) {
    return *B1 >= *B2;
}

bool operator>(const BoundRef &B1, const BoundRef &B2) {
    return *B1 > *B2;
}

bool operator<=(const BoundRef &B1, const BoundRef &B2) {
    return *B1 <= *B2;
}

BoundRef operator+(const BoundRef &B, int X) {
    return *B + X;
}

BoundRef operator-(const BoundRef &B, int X) {
    return *B - X;
}

BoundRef Bound::createBound(const z3::expr &Expr) {
    int64_t Num;
    if (Expr.is_numeral_i64(Num))
        return std::make_shared<ConstantBound>(Num);
    else
        return std::make_shared<SymbolicBound>(Expr);
}

BoundRef Bound::createBound(int64_t N) {
    if (N == -1)
        return std::make_shared<UpperBound>();
    return std::make_shared<ConstantBound>(N);
}
