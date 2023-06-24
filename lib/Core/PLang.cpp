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

#include "Core/PLang.h"
#include "Support/Debug.h"
#include "Support/VSpell.h"

static std::string space(unsigned N) {
    std::string Ret;
    while (N-- > 0) {
        Ret.append(" ");
    }
    return Ret;
}

raw_ostream &operator<<(llvm::raw_ostream &O, const FuncCode &FC) {
    for (auto &S: FC.CodeVec) {
        O << S;
    }
    return O;
}

raw_ostream &operator<<(raw_ostream &O, const PLang &L) {
    for (auto &C: L.FuncVec) {
        O << C << "\n\n";
    }
    return O;
}

PLang::PLang(SliceGraph *G, const std::string &EntryName) {
    auto RealEntryName = VSpell::enabled() ? VSpell::function() : EntryName;

    std::set<z3::expr, Z3::less_than> Vars;
    G->dfs([&Vars](const SliceGraphNode *N) {
        auto E = N->getCondition();
        auto CV = Z3::find_all(E,
                               [](const z3::expr &E) { return E.is_const() && !E.is_numeral() && !Z3::is_length(E); },
                               [](const z3::expr &E) { return Z3::is_naming_eq(E); });
        for (auto C: CV) Vars.insert(C);
    });
    bool HasSeq = false;
    for (auto V: Vars) {
        std::string VName = Z3::to_string(V);
        std::string VType = "int";
        auto VS = V.get_sort();
        if (VS.is_array()) {
            VType = "seq[byte]";
            HasSeq = true;
        }
        Formals.append(VName).append(" : ").append(VType).append(", ");
        Actuals.append(VName).append(", ");
    }
    if (!Formals.empty()) {
        Formals.pop_back();
        Formals.pop_back(); // remove tail ", "
        Actuals.pop_back();
        Actuals.pop_back(); // remove tail ", "
    }

    FuncVec.emplace_front();
    auto &EntryFuncCode = FuncVec.front();
    EntryFuncCode.append("fun ").append(RealEntryName).append("(").append(Formals).append(") : int {\n");
    if (HasSeq)
        EntryFuncCode.append("    var len : int;\n").append("    len = sizeof(B);\n");
    for (auto EIt = G->entry_begin(), EEnd = G->entry_end(); EIt != EEnd; ++EIt) {
        auto *Entry = *EIt;
        gen(Entry, EntryFuncCode, 4);
    }
    // return 1 means the function fails to parse the input packet,
    // return 0 means the function succeeds parsing the input packet
    EntryFuncCode.append(space(4)).append("return 1;\n}\n");

    FuncVec.emplace_front();
    auto &Header = FuncVec.front();
    if (HasSeq || UseStrLen) {
        Header.append("type byte = int;\n\n");
        Header.append("fun select(B : seq[byte], i : int) : byte {\n");
        Header.append("    assert i < sizeof(B);\n");
        Header.append("    return B[i];\n");
        Header.append("}\n\n");
        Header.append("fun handle_field(B : seq[byte], from_idx : int, to_idx : int, name : string) : int {\n");
        Header.append("    if (to_idx >= sizeof(B)) return 1;\n");
        Header.append("    // add code here to handle a field from B[from_idx] to B[to_idx] according to its name\n");
        Header.append("    return 0;\n");
        Header.append("}");
    }
    if (UseExtract) {
        Header.append("\n\n");
        Header.append("fun extract(val : byte, from_idx : int, to_idx : int) : int {\n");
        Header.append("    assert from_idx >= 0;\n");
        Header.append("    assert to_idx <= 7;\n");
        Header.append("    // should call an external function to return a few bits in val.\n");
        Header.append("    // this is because P does not support bit-wise operation by itself.\n");
        Header.append("    return 0;\n");
        Header.append("}");
    }
    if (UseStrLen) {
        Header.append("\n\n");
        Header.append("fun strlen(B : seq[byte], i : int) : int {\n");
        Header.append("    // treat B as a C-string and calculate the length of string from B[i]\n");
        Header.append("    var ret : int;\n");
        Header.append("    ret = 0;\n");
        Header.append("    while (i < sizeof(B) && B[i] != 0) {\n");
        Header.append("        ret = ret + 1;\n");
        Header.append("        i = i + 1;\n");
        Header.append("    }\n");
        Header.append("    return ret;\n");
        Header.append("}");
    }
}

void PLang::gen(SliceGraphNode *Node, FuncCode &Code, unsigned Ident, bool Enforce) {
    if (Node->getNumParents() > 1 && !Enforce) {
        auto It = ReentryCodeMap.find(Node);
        if (It != ReentryCodeMap.end()) {
            Code.append(space(Ident)).append(It->second).append("\n");
        } else {
            // gen func code for node, add to FuncCodeVec
            FuncVec.emplace_front();
            auto &FuncCode = FuncVec.front();
            static unsigned FuncID = 0;

            // add a call to the map; add the call to Code
            std::string Call = "if (f_" + std::to_string(++FuncID) + "(" + Actuals + ") == 0) { return 0; }";
            ReentryCodeMap[Node] = Call;
            Code.append(space(Ident)).append(Call).append("\n");

            FuncCode.append("fun f_").append(std::to_string(FuncID)).append("(" + Formals + ") : int {\n");
            gen(Node, FuncCode, 4, true);
            FuncCode.append(space(4)).append("return 1;\n}");
        }
    } else {
        // gen code node expr
        // add the code to Code
        Code.append(space(Ident)).append("if (").append(toString(Node->getCondition())).append(") {\n");
        for (auto ChIt = Node->child_begin(), ChE = Node->child_end(); ChIt != ChE; ++ChIt) {
            gen(*ChIt, Code, Ident + 4);
        }
        if (Node->getNumChildren() == 0) {
            Code.append(space(Ident + 4)).append("return 0;\n");
        } else {
            // Code.append(space(Ident + 4)).append("return 1;\n");
        }
        Code.append(space(Ident)).append("}\n");
    }
}

void PLang::dump(StringRef FileName) {
    std::error_code EC;
    raw_fd_ostream PStream(FileName.str(), EC, sys::fs::F_None);
    if (PStream.has_error()) {
        errs() << "[Error] Cannot open the file <" << FileName << "> for writing.\n";
        return;
    }
    PStream << *this << "\n";
    POPEYE_INFO(FileName << " dumped!");
}

std::string PLang::toStringTemplate(const z3::expr &Expr, const char *Op) {
    std::string Ret;
    for (unsigned I = 0; I < Expr.num_args(); ++I) {
        Ret.append(toString(Expr.arg(I)));
        if (I != Expr.num_args() - 1) {
            Ret.append(" ").append(Op).append(" ");
        }
    }
    return Ret;
}

std::string PLang::toStringDefault(const z3::expr &Expr) {
    auto Decl = Expr.decl();
    std::string Ret = Decl.name().str();
    if (!UseStrLen && Ret == "strlen")
        UseStrLen = true;

    unsigned NumParams = Z3_get_decl_num_parameters(Expr.ctx(), Decl);
    unsigned NumArgs = Expr.num_args();
    if (NumParams + NumArgs > 0) {
        Ret.append("(");
        for (unsigned I = 0; I < NumArgs + NumParams; ++I) {
            if (I < NumArgs) {
                Ret.append(toString(Expr.arg(I)));
            } else {
                auto ParamKind = Z3_get_decl_parameter_kind(Expr.ctx(), Decl, I - NumArgs);
                switch (ParamKind) {
                    case Z3_PARAMETER_INT:
                        Ret.append(std::to_string(Z3_get_decl_int_parameter(Expr.ctx(), Decl, I - NumArgs)));
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

static std::string power(std::string S, int N) {
    if (N == 0) return std::string();
    assert(N > 0);

    std::string Ret;
    for (int K = 0; K < N; ++K) {
        Ret += S;
    }
    return Ret;
}

std::string PLang::toString(const z3::expr &Expr) {
    uint64_t Num64;
    int64_t Int64;
    if (Z3::is_numeral_i64(Expr, Int64)) {
        return std::to_string(Int64);
    } else if (Z3::is_numeral_u64(Expr, Num64)) {
        return std::to_string(Num64);
    } else {
        auto Kind = Expr.decl().decl_kind();
        switch (Kind) {
            case Z3_OP_TRUE:
                return "true";
            case Z3_OP_FALSE:
                return "false";
//            case Z3_OP_SELECT: {
//                std::string Ret(toString(Expr.arg(0)));
//                Ret.append("[").append(toString(Expr.arg(1))).append("]");
//                return Ret;
//            }
            case Z3_OP_EQ: {
                if (Z3::is_naming_eq(Expr)) {
                    z3::expr_vector SelectOps = Z3::find_all(Expr, false, [](const z3::expr &A) {
                        return A.decl().decl_kind() == Z3_OP_SELECT;
                    });
                    std::vector<z3::expr> Indices;
                    for (z3::expr E: SelectOps)
                        Indices.push_back(E.arg(1));
                    std::sort(Indices.begin(), Indices.end(), [](const z3::expr &A, const z3::expr &B) {
                        return Z3::byte_array_element_index_less_than(A, B);
                    });

                    std::string FieldName = toString(Expr.arg(1));
                    std::string Ret("handle_field(B, ");
                    Ret.append(toString(Indices.front())).append(", ").append(toString(Indices.back()));
                    Ret.append(", \"").append(FieldName).append("\"");
                    Ret.append(") == 0");
                    return Ret;
                } else {
                    std::string Ret(toString(Expr.arg(0)));
                    Ret.append(" == ").append(toString(Expr.arg(1)));
                    return Ret;
                }
            }
            case Z3_OP_DISTINCT: {
                std::string Ret(toString(Expr.arg(0)));
                Ret.append(" != ").append(toString(Expr.arg(1)));
                return Ret;
            }
            case Z3_OP_CONCAT: {
                std::vector<std::string> ConcatVec;
                bool PrefixZero = true;
                for (unsigned I = 0; I < Expr.num_args(); ++I) {
                    auto Concat = Expr.arg(I);
                    int Zero;
                    if (Concat.is_numeral_i(Zero) && Zero == 0) if (PrefixZero) continue;
                    if (PrefixZero) PrefixZero = false;
                    ConcatVec.push_back(toString(Concat));
                }
                std::string Ret = ConcatVec.empty() ? "0" : "";
                int i = ConcatVec.size();
                for (auto &C: ConcatVec) {
                    std::string Power = power("256 * ", --i);
                    Ret += Power;
                    Ret += C;
                    if (i != 0) Ret += " + ";
                }
                return Ret;
            }
            case Z3_OP_ADD:
            case Z3_OP_BADD: {
                return toStringTemplate(Expr, "+");
            }
            case Z3_OP_SUB:
            case Z3_OP_BSUB: {
                return toStringTemplate(Expr, "-");
            }
            case Z3_OP_MUL:
            case Z3_OP_BMUL: {
                return toStringTemplate(Expr, "*");
            }
            case Z3_OP_DIV:
            case Z3_OP_BSDIV_I:
            case Z3_OP_BSDIV:
            case Z3_OP_BUDIV_I:
            case Z3_OP_BUDIV: {
                return toStringTemplate(Expr, "/");
            }
            case Z3_OP_MOD:
            case Z3_OP_REM:
            case Z3_OP_BSMOD:
            case Z3_OP_BSMOD_I:
            case Z3_OP_BSREM:
            case Z3_OP_BSREM_I:
            case Z3_OP_BUREM:
            case Z3_OP_BUREM_I: {
                return toStringTemplate(Expr, "%");
            }
            case Z3_OP_AND: {
                return toStringTemplate(Expr, "&&");
            }
//            case Z3_OP_BAND: {
//                return toStringTemplate(Expr, "&");
//            }
            case Z3_OP_OR: {
                return toStringTemplate(Expr, "||");
            }
//            case Z3_OP_BOR: {
//                return toStringTemplate(Expr, "|");
//            }
//            case Z3_OP_XOR3:
//            case Z3_OP_BXOR:
//            case Z3_OP_XOR: {
//                return toStringTemplate(Expr, "^");
//            }
            case Z3_OP_GE:
            case Z3_OP_SGEQ:
            case Z3_OP_UGEQ: {
                return toStringTemplate(Expr, ">=");
            }
            case Z3_OP_LE:
            case Z3_OP_SLEQ:
            case Z3_OP_ULEQ: {
                return toStringTemplate(Expr, "<=");
            }
            case Z3_OP_GT:
            case Z3_OP_SGT:
            case Z3_OP_UGT: {
                return toStringTemplate(Expr, ">");
            }
            case Z3_OP_LT:
            case Z3_OP_SLT:
            case Z3_OP_ULT: {
                return toStringTemplate(Expr, "<");
            }
//            case Z3_OP_BLSHR: {
//                return toStringTemplate(Expr, ">>>");
//            }
//            case Z3_OP_BASHR: {
//                return toStringTemplate(Expr, ">>");
//            }
//            case Z3_OP_BSHL: {
//                return toStringTemplate(Expr, "<<");
//            }
//            case Z3_OP_BNOT: {
//                return "~" + toString(Expr.arg(0));
//            }
            case Z3_OP_NOT: {
                return "!(" + toString(Expr.arg(0)) + ")";
            }
            case Z3_OP_UMINUS: {
                return "-" + toString(Expr.arg(0));
            }
//            case Z3_OP_SIGN_EXT:
//            case Z3_OP_ZERO_EXT:
//            case Z3_OP_BV2INT: {
//                return toString(Expr.arg(0));
//            }
//            case Z3_OP_ITE: {
//                return toString(Expr.arg(0)) + " ? " + toString(Expr.arg(1)) + " : " + toString(Expr.arg(2));
//            }
            case Z3_OP_EXTRACT:
                UseExtract = true;
            default: {
                return toStringDefault(Expr);
            }
        }
    }
}
