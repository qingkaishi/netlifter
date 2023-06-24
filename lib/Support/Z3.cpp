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

#include "Support/Debug.h"
#include "Support/Z3.h"
#include "Z3Macro.h"

static z3::context *Ctx = nullptr;
static z3::expr *Len = nullptr;

static z3::solver *Solver = nullptr;
static z3::expr_vector *SolverAssumptions = nullptr;

static z3::context &ctx() {
    if (!Ctx)
        Ctx = new z3::context;
    return *Ctx;
}

static z3::solver &solver() {
    if (!Solver)
        Solver = new z3::solver(ctx());
    return *Solver;
}

static z3::expr_vector &solver_assumptions() {
    if (!SolverAssumptions)
        SolverAssumptions = new z3::expr_vector(ctx());
    return *SolverAssumptions;
}

void Z3::initialize() {
    POPEYE_INFO("Z3 version: " << Z3_get_full_version());
}

void Z3::finalize() {
#define DEINIT(X) do { if (X) { delete X; X = nullptr; } } while(0)

    // fixme:
    // do not delete them now, because some structures, e.g., AbstractValue,
    // that rely on z3 may be released after this function...
    //
    // DEINIT(Len);
    // DEINIT(SolverAssumptions);
    // DEINIT(Solver);
    // DEINIT(Ctx);
}

z3::expr Z3::bv_val(unsigned V, unsigned Size) {
    return ctx().bv_val(V, Size);
}

z3::expr Z3::bv_val(int V, unsigned Size) {
    return ctx().bv_val(V, Size);
}

z3::expr Z3::bv_val(uint64_t V, unsigned Size) {
    return ctx().bv_val(V, Size);
}

z3::expr Z3::bv_val(int64_t V, unsigned Size) {
    return ctx().bv_val(V, Size);
}

z3::expr Z3::bv_const(const char *Name, unsigned int Size) {
    return ctx().bv_const(Name, Size);
}

z3::expr Z3::bool_val(bool B) {
    return ctx().bool_val(B);
}

z3::expr Z3::free_bool() {
    static unsigned I = 0;
    std::string Name(FREE_VAR);
    Name.append(std::to_string(I++));
    return ctx().bool_const(Name.c_str());
}

z3::expr Z3::free_bv(unsigned Bitwidth) {
    static unsigned K = 0;
    std::string Name(FREE_VAR);
    Name.append(std::to_string(K++));
    return ctx().bv_const(Name.c_str(), Bitwidth);
}

bool Z3::is_free(const z3::expr &E) {
    return E.is_const() && E.to_string().find(FREE_VAR) != std::string::npos;
}

z3::expr Z3::k() {
    return ctx().bv_const("k", 64);
}

z3::expr Z3::substitute(const z3::expr &E, const z3::expr &F, const z3::expr &T) {
    auto Expr = E;
    auto FV = Z3::vec();
    auto TV = Z3::vec();
    FV.push_back(F);
    TV.push_back(T);
    return Expr.substitute(FV, TV);
}

z3::expr Z3::index_var() {
    static unsigned J = 0;
    std::string Name(INDEX_VAR);
    Name.append(std::to_string(J++));
    return ctx().bv_const(Name.c_str(), 64);
}

bool Z3::is_index_var(const z3::expr &E) {
    return E.is_bv() && E.is_const() && E.to_string().find(INDEX_VAR) != std::string::npos;
}

z3::expr Z3::length(unsigned Bitwidth) {
    if (!Len) {
        assert(Bitwidth != UINT32_MAX);
        Len = new z3::expr(ctx());
        *Len = Z3::bv_const(LENGTH, Bitwidth);
        return *Len;
    } else {
        assert((Bitwidth == UINT32_MAX || Len->get_sort().bv_size() == Bitwidth) &&
               "Bitwidth of length cannot be changed!");
        return *Len;
    }
}

bool Z3::is_length(const z3::expr &E) {
    return E.is_bv() && E.is_const() && E.to_string() == LENGTH;
}

z3::expr Z3::base(unsigned K) {
    std::string Name(BASE);
    Name.append(std::to_string(K));
    return Z3::bv_const(Name.c_str(), 64);
}

bool Z3::is_base(const z3::expr &E) {
    return E.is_bv() && E.is_const() && E.to_string().find(BASE) == 0;
}

z3::expr Z3::transit_to(const z3::expr &E) {
    auto Transfer = z3::function(STATE_TRANSITION, E.get_sort(), Z3::bool_val(true).get_sort());
    return Transfer(E);
}

bool Z3::is_transit_to(const z3::expr &E) {
    if (E.num_args() == 1)
        return E.decl().name().str() == STATE_TRANSITION;
    return false;
}

bool Z3::is_state(const z3::expr &E) {
    return E.is_bv() && E.is_const() && E.to_string() == STATE;
}

z3::expr Z3::trip_count(unsigned K) {
    std::string Name(TRIP_COUNT);
    Name.append(std::to_string(K));
    return Z3::bv_const(Name.c_str(), 64);
}

bool Z3::is_trip_count(const z3::expr &E) {
    return E.is_bv() && E.is_const() && E.to_string().find(TRIP_COUNT) == 0;
}

z3::expr_vector Z3::vec() {
    return {ctx()};
}

z3::expr Z3::packing(const char *Name, const z3::expr_vector &Args, unsigned Ret) {
    z3::sort_vector SortVec(ctx());
    for (auto E: Args)
        SortVec.push_back(E.get_sort());
    auto RetSort = Ret == 0 ? ctx().bool_sort() : ctx().bv_sort(Ret);
    auto Decl = z3::function(Name, SortVec, RetSort);
    return Decl(Args);
}

z3::expr Z3::naming(const z3::expr &E, const char *Name) {
    auto Decl = z3::function(NAME, E.get_sort(), E.get_sort());
    auto Naming = Decl(E);
    auto NameExpr = Z3::bv_const(Name, E.get_sort().bv_size());
    return Naming == NameExpr;
}

bool Z3::is_naming(const z3::expr &E) {
    return E.num_args() == 1 && E.decl().name().str() == NAME;
}

bool Z3::is_naming_eq(const z3::expr &E) {
    if (E.is_eq())
        return is_naming(E.arg(0));
    return false;
}

unsigned Z3::id(const z3::expr &E) {
    return Z3_get_ast_id(ctx(), E);
}

bool Z3::same(const z3::expr &E1, const z3::expr &E2) {
    return Z3_get_ast_id(ctx(), E1) == Z3_get_ast_id(ctx(), E2);
}

bool Z3::is_numeral_i64(const z3::expr &E, int64_t &R) {
    uint64_t Const;
    if (Z3::is_numeral_u64(E, Const)) {
        unsigned OrigBitwidth = E.get_sort().bv_size();
        Const = Const << (64 - OrigBitwidth);
        R = ((int64_t) Const) >> (64 - OrigBitwidth);
        return true;
    }
    return false;
}

bool Z3::is_numeral_u64(const z3::expr &E, uint64_t &R) {
    return E.is_numeral_u64(R);
}

bool Z3::is_zero(const z3::expr &E) {
    uint64_t Const;
    return Z3::is_numeral_u64(E, Const) && Const == 0;
}

bool Z3::is_minus_one(const z3::expr &E) {
    int64_t Const;
    return Z3::is_numeral_i64(E, Const) && Const == -1;
}

bool Z3::find(const z3::expr &Expr, const z3::expr &SubExpr) {
    std::set<unsigned> Visited;
    z3::expr_vector Stack = vec();
    Stack.push_back(Expr);
    while (!Stack.empty()) {
        auto Top = Stack.back();
        Stack.pop_back();
        auto TopID = id(Top);
        if (Visited.count(TopID)) {
            continue;
        }
        Visited.insert(TopID);
        if (Z3::same(Top, SubExpr)) {
            return true;
        }

        auto NumArgs = Top.num_args();
        for (unsigned I = 0; I < NumArgs; ++I) {
            Stack.push_back(Top.arg(I));
        }
    }
    return false;
}

bool Z3::find(const z3::expr &Expr, bool (*P)(const z3::expr &)) {
    std::set<unsigned> Visited;
    z3::expr_vector Stack = vec();
    Stack.push_back(Expr);
    while (!Stack.empty()) {
        auto Top = Stack.back();
        Stack.pop_back();
        auto TopID = id(Top);
        if (Visited.count(TopID)) {
            continue;
        }
        Visited.insert(TopID);
        if (P(Top)) {
            return true;
        }

        auto NumArgs = Top.num_args();
        for (unsigned I = 0; I < NumArgs; ++I) {
            Stack.push_back(Top.arg(I));
        }
    }
    return false;
}

z3::expr_vector Z3::find_all(const z3::expr &Expr, bool Recursive, bool (*P)(const z3::expr &)) {
    z3::expr_vector Result = vec();

    std::set<unsigned> Visited;
    z3::expr_vector Stack = vec();
    Stack.push_back(Expr);
    while (!Stack.empty()) {
        auto Top = Stack.back();
        Stack.pop_back();
        auto TopID = id(Top);
        if (Visited.count(TopID)) {
            continue;
        }
        Visited.insert(TopID);
        if (P(Top)) {
            Result.push_back(Top);
            if (!Recursive) continue;
        }

        auto NumArgs = Top.num_args();
        for (unsigned I = 0; I < NumArgs; ++I) {
            Stack.push_back(Top.arg(I));
        }
    }
    return Result;
}

z3::expr_vector Z3::find_all(const z3::expr &Expr, bool (*P)(const z3::expr &), bool (*T)(const z3::expr &)) {
    z3::expr_vector Result = vec();

    std::set<unsigned> Visited;
    z3::expr_vector Stack = vec();
    Stack.push_back(Expr);
    while (!Stack.empty()) {
        auto Top = Stack.back();
        Stack.pop_back();
        auto TopID = id(Top);
        if (Visited.count(TopID)) {
            continue;
        }
        Visited.insert(TopID);
        if (P(Top)) {
            Result.push_back(Top);
        }
        if (T(Top)) {
            continue;
        }

        auto NumArgs = Top.num_args();
        for (unsigned I = 0; I < NumArgs; ++I) {
            Stack.push_back(Top.arg(I));
        }
    }
    return Result;
}

template<typename OpKind>
static void postVisit(const z3::expr &Expr, OpKind P, z3::expr_vector Ret, std::set<unsigned> *Visited) {
    if (Visited) {
        if (Visited->count(Z3::id(Expr)))
            return;
        Visited->insert(Z3::id(Expr));
    }

    if (!P(Expr)) {
        Ret.push_back(Expr);
        return;
    }
    auto NumArgs = Expr.num_args();
    for (unsigned I = 0; I < NumArgs; ++I) {
        postVisit(Expr.arg(I), P, Ret, Visited);
    }
}

z3::expr_vector Z3::find_consecutive_ops(const z3::expr &Expr, bool (*P)(const z3::expr &), bool AllowRep) {
    z3::expr_vector Result = vec();
    if (!AllowRep) {
        std::set<unsigned> Visited;
        postVisit(Expr, P, Result, &Visited);
    } else {
        postVisit(Expr, P, Result, nullptr);
    }
    return Result;
}

z3::expr_vector Z3::find_consecutive_ops(const z3::expr &Expr, Z3_decl_kind OpKind, bool AllowRep) {
    auto P = [OpKind](const z3::expr &E) { return E.decl().decl_kind() == OpKind; };
    z3::expr_vector Result = vec();
    if (!AllowRep) {
        std::set<unsigned> Visited;
        postVisit(Expr, P, Result, &Visited);
    } else {
        postVisit(Expr, P, Result, nullptr);
    }
    return Result;
}

z3::expr_vector Z3::find_consecutive_ops(const z3::expr &Expr, const char *OpName, bool AllowRep) {
    auto P = [OpName](const z3::expr &E) { return E.decl().name().str() == OpName; };
    z3::expr_vector Result = vec();
    if (!AllowRep) {
        std::set<unsigned> Visited;
        postVisit(Expr, P, Result, &Visited);
    } else {
        postVisit(Expr, P, Result, nullptr);
    }
    return Result;
}


bool Z3::is_distinct_or_not_eq(const z3::expr &E) {
    if (E.is_distinct()) return true;
    if (E.is_not() && E.arg(0).is_eq()) return true;
    return false;
}

// https://en.cppreference.com/w/c/language/operator_precedence
static std::map<Z3_decl_kind, int> OpPriorityMap = {
        {Z3_OP_BMUL, 3},
        {Z3_OP_BADD, 4},
        {Z3_OP_BSUB, 4},
        // to extend ...
};

static int get_op_priority(Z3_decl_kind Op) {
    int ExprOpPriority = -1;
    auto It = OpPriorityMap.find(Op);
    if (It != OpPriorityMap.end()) return It->second;
    return ExprOpPriority;
}

static std::string to_string_template(const z3::expr &Expr, const char *Op) {
    assert(Expr.num_args() > 1);
    int ExprOpPriority = get_op_priority(Expr.decl().decl_kind());

    std::string Ret;
    for (unsigned I = 0; I < Expr.num_args(); ++I) {
        int SubOpPriority = get_op_priority(Expr.arg(I).decl().decl_kind());
        if (ExprOpPriority == -1 || SubOpPriority == -1)
            Ret.append(Z3::to_string(Expr.arg(I)));
        else if (ExprOpPriority < SubOpPriority)
            Ret.append("(").append(Z3::to_string(Expr.arg(I))).append(")");
        else
            Ret.append(Z3::to_string(Expr.arg(I)));
        if (I != Expr.num_args() - 1) {
            Ret.append(" ").append(Op).append(" ");
        }
    }
    return Ret;
}

static std::string to_string_default(const z3::expr &Expr) {
    auto Decl = Expr.decl();
    std::string Ret = Decl.name().str();

    if (Ret == BYTE_ARRAY_RANGE) {
        Ret = Z3::to_string(Expr.arg(0));
        Ret += "[";
        Ret += Z3::to_string(Expr.arg(1));
        Ret += "..";
        Ret += Z3::to_string(Expr.arg(2));
        Ret += "]";
        return Ret;
    }

    if (Expr.decl().decl_kind() == Z3_OP_EXTRACT && Z3::is_trip_count(Expr.arg(0))) {
        return Z3::to_string(Expr.arg(0));
    }

    bool Phi = Z3::is_phi(Expr);
    unsigned NumParams = Z3_get_decl_num_parameters(ctx(), Decl);
    unsigned NumArgs = Expr.num_args();
    if (NumParams + NumArgs > 0) {
        Ret.append("(");
        for (unsigned I = 0; I < NumArgs + NumParams; ++I) {
            if (I < NumArgs) {
                if (!Phi) {
                    Ret.append(Z3::to_string(Expr.arg(I)));
                } else {
                    Ret.append(Z3::to_string(Expr.arg(I))).append(", ");
                    auto CondID = Z3::phi_cond_id(Z3::phi_id(Expr), I);
                    Ret.append("$").append(std::to_string(CondID));
                }
            } else {
                auto ParamKind = Z3_get_decl_parameter_kind(ctx(), Decl, I - NumArgs);
                switch (ParamKind) {
                    case Z3_PARAMETER_INT:
                        Ret.append(std::to_string(Z3_get_decl_int_parameter(ctx(), Decl, I - NumArgs)));
                        break;
                    case Z3_PARAMETER_DOUBLE:
                    case Z3_PARAMETER_RATIONAL:
                    case Z3_PARAMETER_SYMBOL:
                    case Z3_PARAMETER_SORT:
                    case Z3_PARAMETER_AST:
                    case Z3_PARAMETER_FUNC_DECL:
                        Ret.append("?");
                        break;
                }
            }
            if (I != NumArgs + NumParams - 1) {
                Ret.append(", ");
            }
        }
        Ret.append(")");
    }

    return Ret;
}

std::string Z3::to_string(const z3::expr &Expr, bool Easy) {
    if (!Easy)
        return Expr.to_string();
    int64_t Num64;
    if (Z3::is_numeral_i64(Expr, Num64)) {
        if (Expr.get_sort().bv_size() > 4)
            return std::to_string(Num64);
        else
            return Expr.to_string();
    } else {
        auto Kind = Expr.decl().decl_kind();
        switch (Kind) {
            case Z3_OP_TRUE:
                return "true";
            case Z3_OP_FALSE:
                return "false";
            case Z3_OP_SELECT: {
                std::string Ret(to_string(Expr.arg(0)));
                Ret.append("[").append(to_string(Expr.arg(1))).append("]");
                return Ret;
            }
            case Z3_OP_EQ: {
                std::string Ret(to_string(Expr.arg(0)));
                Ret.append(" = ").append(to_string(Expr.arg(1)));
                return Ret;
            }
            case Z3_OP_DISTINCT: {
                std::string Ret(to_string(Expr.arg(0)));
                Ret.append(" ≠ ").append(to_string(Expr.arg(1)));
                return Ret;
            }
            case Z3_OP_CONCAT: {
                std::string Ret;
                unsigned K = 0;
                auto Head = Expr.arg(K);
                int64_t Zero;
                if (Z3::is_numeral_i64(Head, Zero) && Zero == 0) {
                    K++;
                    assert(K < Expr.num_args());
                } else if (Head.get_sort().bv_size() == 1 && Head.decl().decl_kind() == Z3_OP_EXTRACT) {
                    unsigned J = 1;
                    for (; J < Expr.num_args(); ++J) {
                        if (!Z3::same(Expr.arg(J), Head)) break;
                    }
                    if (J < Expr.num_args() && Z3::same(Head.arg(0), Expr.arg(J))) {
                        K = J;
                    }
                }

                for (unsigned I = K; I < Expr.num_args(); ++I)
                    Ret.append(to_string(Expr.arg(I)));
                return Ret;
            }
            case Z3_OP_ADD:
            case Z3_OP_BADD: {
                return to_string_template(Expr, "+");
            }
            case Z3_OP_SUB:
            case Z3_OP_BSUB: {
                return to_string_template(Expr, "-");
            }
            case Z3_OP_MUL:
            case Z3_OP_BMUL: {
                return to_string_template(Expr, "x");
            }
            case Z3_OP_DIV:
            case Z3_OP_BSDIV_I:
            case Z3_OP_BSDIV:
            case Z3_OP_BUDIV_I:
            case Z3_OP_BUDIV: {
                return to_string_template(Expr, "/");
            }
            case Z3_OP_MOD:
            case Z3_OP_REM:
            case Z3_OP_BSMOD:
            case Z3_OP_BSMOD_I:
            case Z3_OP_BSREM:
            case Z3_OP_BSREM_I:
            case Z3_OP_BUREM:
            case Z3_OP_BUREM_I: {
                return to_string_template(Expr, "%");
            }
            case Z3_OP_AND: {
                return to_string_template(Expr, "&&");
            }
            case Z3_OP_BAND: {
                return to_string_template(Expr, "&");
            }
            case Z3_OP_OR: {
                return to_string_template(Expr, "||");
            }
            case Z3_OP_BOR: {
                return to_string_template(Expr, "|");
            }
            case Z3_OP_XOR3:
            case Z3_OP_BXOR:
            case Z3_OP_XOR: {
                return to_string_template(Expr, "^");
            }
            case Z3_OP_GE:
            case Z3_OP_SGEQ:
            case Z3_OP_UGEQ: {
                return to_string_template(Expr, "≥");
            }
            case Z3_OP_LE:
            case Z3_OP_SLEQ:
            case Z3_OP_ULEQ: {
                return to_string_template(Expr, "≤");
            }
            case Z3_OP_GT:
            case Z3_OP_SGT:
            case Z3_OP_UGT: {
                return to_string_template(Expr, ">");
            }
            case Z3_OP_LT:
            case Z3_OP_SLT:
            case Z3_OP_ULT: {
                return to_string_template(Expr, "<");
            }
            case Z3_OP_BLSHR: {
                return to_string_template(Expr, ">>>");
            }
            case Z3_OP_BASHR: {
                return to_string_template(Expr, ">>");
            }
            case Z3_OP_BSHL: {
                return to_string_template(Expr, "<<");
            }
            case Z3_OP_BNOT: {
                return "~" + to_string(Expr.arg(0));
            }
            case Z3_OP_NOT: {
                return "NOT(" + to_string(Expr.arg(0)) + ")";
            }
            case Z3_OP_UMINUS: {
                return "-" + to_string(Expr.arg(0));
            }
            case Z3_OP_SIGN_EXT:
            case Z3_OP_ZERO_EXT:
            case Z3_OP_BV2INT: {
                return to_string(Expr.arg(0));
            }
            case Z3_OP_ITE: {
                return to_string(Expr.arg(0)) + " ? " + to_string(Expr.arg(1)) + " : " + to_string(Expr.arg(2));
            }
            default: {
                return to_string_default(Expr);
            }
        }
    }
}

static Z3::Z3Format PrintFormat = Z3::ZF_Easy;

raw_ostream &operator<<(llvm::raw_ostream &O, const Z3::Z3Format &F) {
    PrintFormat = F;
    return O;
}

raw_ostream &operator<<(llvm::raw_ostream &O, const z3::expr &E) {
    switch (PrintFormat) {
        case Z3::ZF_Orig:
            O << E.to_string();
            break;
        case Z3::ZF_Easy:
            O << Z3::to_string(E);
            break;
        case Z3::ZF_SMTLib:
            O << Z3_benchmark_to_smtlib_string(*Ctx, 0, 0, 0, 0, 0, 0, E);
            break;
    }
    PrintFormat = Z3::ZF_Easy; // reset
    return O;
}

void Z3Solver::addAssumption(const z3::expr &A) {
    solver_assumptions().push_back(A);
}

void Z3Solver::clearAssumptions() {
    delete SolverAssumptions;
    SolverAssumptions = nullptr;
}

bool Z3Solver::check(const z3::expr &A, std::vector<uint8_t> &Ret) {
    solver().reset();
    solver().add(A);
    auto Result = solver().check();
    if (Result != z3::sat) {
        Ret.clear();
        return false;
    }
    auto Model = solver().get_model();
    std::vector<bool> Set(Ret.size(), false);
    for (unsigned K = 0; K < Model.num_consts(); ++K) {
        auto Decl = Model.get_const_decl(K);
        auto Interp = Model.get_const_interp(Decl);
        auto DeclName = Decl.name().str();

        if (DeclName == LENGTH) {
            uint64_t LenConst;
            bool Const = Z3::is_numeral_u64(Interp, LenConst);
            assert(Const);
            Ret.resize(LenConst);
            Set.resize(LenConst, false);
        } else if (DeclName == BYTE_ARRAY) {
            while (Interp.decl().decl_kind() == Z3_OP_STORE) {
                uint64_t Index;
                bool Const = Z3::is_numeral_u64(Interp.arg(1), Index);
                assert(Const);
                uint64_t Val;
                Const = Z3::is_numeral_u64(Interp.arg(2), Val);
                assert(Const);
                if (Index >= Ret.size()) {
                    Ret.resize(Index + 1);
                    Set.resize(Index + 1, false);
                }
                if (!Set[Index]) {
                    Ret[Index] = (uint8_t) Val;
                    Set[Index] = true;
                }

                Interp = Interp.arg(0);
            }
            if (Interp.decl().decl_kind() == Z3_OP_CONST_ARRAY) {
                uint64_t Val;
                bool Const = Z3::is_numeral_u64(Interp.arg(0), Val);
                assert(Const);
                for (unsigned J = 0; J < Set.size(); ++J) {
                    if (!Set[J]) {
                        Ret[J] = Val;
                    }
                }
            }
        }
    }
    return true;
}

bool Z3Solver::check(const z3::expr &A) {
    solver().reset();
    solver().add(A);
    return solver().check() == z3::sat;
}

bool Z3Solver::check(const std::vector<z3::expr> &V) {
    solver().reset();
    for (auto &E: V) solver().add(E);
    auto Result = solver().check();
    assert(Result != z3::unknown);
    return Result == z3::sat;
}

bool Z3Solver::check(const z3::expr_vector &V) {
    solver().reset();
    for (auto E: V) solver().add(E);
    auto Result = solver().check();
    assert(Result != z3::unknown);
    return Result == z3::sat;
}
