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


#ifndef P_DDLLANG_H
#define P_DDLLANG_H

#include "Core/SliceGraph.h"
#include "BNF/BNF.h"
#include <list>
#include <vector>

class DDLLang {
public:
    BNF *Bnf;

public:
    DDLLang(BNF *B) : Bnf(B){};

    void dump(StringRef FileName);

    std::string Production2DDL(Production& P);
    std::string toStringTemplate(const z3::expr &Expr, const char *Op);
    std::string toStringDefault(const z3::expr &Expr);
    std::string toStringExtract(const z3::expr &Expr);
    std::string toString(const z3::expr &Expr);
};



#endif //P_DDLLang_H
