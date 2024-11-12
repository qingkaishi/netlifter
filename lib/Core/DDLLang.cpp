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

#include "Core/DDLLang.h"
#include "Support/Debug.h"
#include "Support/VSpell.h"
#include <iomanip>

static std::string space(unsigned N) {
    std::string Ret;
    while (N-- > 0) {
        Ret.append(" ");
    }
    return Ret;
}

std::string uint64ToHex(uint64_t value) {
    std::ostringstream oss;
    oss << "0x" << std::hex << std::uppercase << std::setw(16) << std::setfill('0') << (value & 0xFFFFFFFFFFFFFFFF);
    return oss.str();
}

std::string DDLLang::toStringTemplate(const z3::expr &Expr, const char *Op) {
    std::string Ret("(");
    for (unsigned I = 0; I < Expr.num_args(); ++I) {
        Ret.append(toString(Expr.arg(I)));
        if (I != Expr.num_args() - 1) {
            Ret.append(" ").append(Op).append(" ");
        }
    }
    Ret.append(")");
    return Ret;
}

std::string DDLLang::toStringDefault(const z3::expr &Expr) {
    auto Decl = Expr.decl();
    std::string Ret = Decl.name().str();
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

std::string DDLLang::toStringExtract(const z3::expr &Expr)
{
    auto Decl = Expr.decl();
    unsigned NumParams = Z3_get_decl_num_parameters(Expr.ctx(), Decl);
    unsigned NumArgs = Expr.num_args();
    if (NumParams + NumArgs > 0) {
        // std::string Ret = Decl.name().str();
        std::string Ret("(Extract ");
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
                Ret.append(" ");
            }
        }
        Ret.append(")");
        return Ret;
    }
    else
    {
        llvm_unreachable("Unexpected behavior of z3::extract!");
    }
}

std::string DDLLang::toString(const z3::expr &Expr) {
    uint64_t Num64;
    int64_t Int64;
    
    if (Z3::is_numeral_u64(Expr, Num64)) {
        return uint64ToHex(Num64);
    }
    else if (Z3::is_numeral_i64(Expr, Int64)) {
        if(Int64 >= 0)
        {
            return uint64ToHex(Int64);
        }
        else
        {
            Num64 = static_cast<uint64_t>(Int64);
            return uint64ToHex(Num64); 
        }
    } 
    else {
        auto Kind = Expr.decl().decl_kind();
        switch (Kind) {
            case Z3_OP_TRUE:
                return "true";
            case Z3_OP_FALSE:
                return "false";
            case Z3_OP_SELECT: {
                std::string Ret("(Select ");
                if(Z3::is_numeral_i64(Expr.arg(1), Int64))
                {
                    Ret.append(std::to_string(Int64)).append(")");
                }
                else if(Z3::is_numeral_u64(Expr.arg(1), Num64))
                {
                    Ret.append(std::to_string(Num64)).append(")");
                }
                else
                {
                    Ret.append(toString(Expr.arg(1))).append(")");
                }
                return Ret;
            }
            case Z3_OP_EQ: {
                if (Z3::is_naming_eq(Expr)) {
                    // z3::expr_vector SelectOps = Z3::find_all(Expr, false, [](const z3::expr &A) {
                    //     return A.decl().decl_kind() == Z3_OP_SELECT;
                    // });
                    // std::vector<z3::expr> Indices;
                    // for (z3::expr E: SelectOps)
                    //     Indices.push_back(E.arg(1));
                    // std::sort(Indices.begin(), Indices.end(), [](const z3::expr &A, const z3::expr &B) {
                    //     return Z3::byte_array_element_index_less_than(A, B);
                    // });

                    // std::string FieldName = toString(Expr.arg(1));
                    // std::string Ret("handle_field(B, ");
                    // Ret.append(toString(Indices.front())).append(", ").append(toString(Indices.back()));
                    // Ret.append(", \"").append(FieldName).append("\"");
                    // Ret.append(") == 0");
                    // return Ret;
                    return "";
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
                return toStringExtract(Expr);
                // UseExtract = true;
            default: {
                return toStringDefault(Expr);
            }
        }
    }
}

/*
def Select (N : uint 64) =
  block
    let cur = GetStream
    let a = bytesOfStream cur
    (Index a N) as uint 64

def Len =
  block
    let cur = GetStream
    let a = bytesOfStream cur
    length a

def Extract (High : uint 64) (Low : uint 64) (N : uint 64) = 
  block
    let mask = (1 << (High - Low + 1)) - 1
    (N >> Low) .&. mask
*/

std::string GenHelperFunctionsInDDL()
{
    std::string s;
    s += "def Select (N : uint 64) =\n";
    s += space(2) + "block\n";
    s += space(4) + "let cur = GetStream\n";
    s += space(4) + "let a = bytesOfStream cur\n";
    s += space(4) + "(Index a N) as uint 64\n\n";

    s += "def Len =\n";
    s += space(2) + "block\n";
    s += space(4) + "let cur = GetStream\n";
    s += space(4) + "let a = bytesOfStream cur\n";
    s += space(4) + "length a\n\n";

    s += "def Extract (High : uint 64) (Low : uint 64) (N : uint 64) =\n";
    s += space(2) + "block\n";
    s += space(4) + "let mask = (1 << (High - Low + 1)) - 1\n";
    s += space(4) + "(N >> Low) .&. mask\n\n";

    return s;
}


std::string DDLLang::Production2DDL(Production& P)
{
    z3::expr_vector ExtraCond = Z3::vec();
    z3::expr_vector BeforeSubstitution = Z3::vec();
    z3::expr_vector AfterSubstitution = Z3::vec();
    std::string code;
    if(P.getLHS() == 0)
    {
        std::string lhs = "def Main = \n" + space(2) + "block\n" + space(4) + "let len = Len\n";
        code += lhs;
        errs()<<lhs;
    }
    else
    {
        std::string lhs = "def L" + std::to_string(P.getLHS()) + " = \n" + space(2) + "block\n" + space(4) + "let len = Len\n";
        code += lhs;
        errs()<<lhs;
    }
    std::vector<std::vector<unsigned>> Productions;
    for(auto rhs_it = P.rhs_begin(); rhs_it != P.rhs_end(); rhs_it = std::next(rhs_it))
    {
        std::vector<unsigned> RHS_conjuction;
        auto Items = *rhs_it;
        for (unsigned i = 0; i < Items.size(); ++i) 
        {
            RHSItem *Item = Items[i];
            if (auto *PItem = dyn_cast<Production>(Item)) 
            {
                RHS_conjuction.push_back(PItem->getLHS());
                // std::string rhs_l = space(4) + "L" + std::to_string(PItem->getLHS()) + "\n";
                // code += rhs_l;
                // errs()<<rhs_l;
            }
            else if (auto *IItem = dyn_cast<Interval>(Item)) 
            {
                auto From = IItem->getFrom();
                auto To = IItem->getTo();
                if(From == nullptr || To == nullptr)
                {
                    llvm_unreachable("Null ptr!");
                }
                // We do not need to define var for constant index
                if(isa<ConstantBound>(From.get()) && isa<ConstantBound>(To.get()))
                {
                    // if(From == To)
                    // {
                    //     // errs()<< space(4) << "let b" <<From<<" = Select "<<From<<"\n";
                    //     continue;
                    // }
                    // ConstantBound* From_ = dynamic_cast<ConstantBound*>(From.get());
                    // ConstantBound* To_ = dynamic_cast<ConstantBound*>(To.get());
                    // int64_t From_constant = From_->constant();
                    // int64_t To_constant = To_->constant();
                    // if(From_constant >= 0 && To_constant >= 0)
                    // {
                    //     for(int64_t j = From_constant; j <= To_constant; j++)
                    //     {
                    //         // errs()<< space(4) << "let b" << j << " = Select "<<j<<"\n";
                    //     }
                    // }
                    // //this should be followed by a symbolic index
                    // else if(To_constant < 0)
                    // {
                    //     // The next item should be a symbolic one so we take next.from - 1 as this.to
                    //     // No next item? Abort
                    //     if(i + 1 == Items.size())
                    //     {
                    //         llvm_unreachable("No next item?");
                    //     }
                    //     if(auto *IItem1 = dyn_cast<Interval>(Items[i+1]))
                    //     {
                    //         auto From1 = IItem1->getFrom();
                    //         auto To1 = IItem1->getTo();
                    //         if(isa<ConstantBound>(From1.get()) && isa<ConstantBound>(To1.get()))
                    //         {
                    //             llvm_unreachable("Next item constant?");
                    //         }
                    //         // b[7...] B[ii1...ii2]
                    //         else if(isa<SymbolicBound>(From1.get()) && isa<SymbolicBound>(To1.get()))
                    //         {
                    //             SymbolicBound *From1_ = dynamic_cast<SymbolicBound*>(From1.get());
                    //             SymbolicBound *To1_ = dynamic_cast<SymbolicBound*>(To1.get());

                    //             BoundRef FromSub1, ToSub1;
                    //             auto NewIndexVar = Z3::index_var();
                    //             ExtraCond.push_back(NewIndexVar == From1->expr());
                    //             std::string let_defs;
                    //             let_defs = space(4) + "let " + toString(NewIndexVar) + " = " + toString(From1->expr()) + "\n";
                    //             errs() << let_defs;
                    //             code += let_defs;
                    //             // errs() << space(4) << "let b" << NewIndexVar << " = Select "<<NewIndexVar<<"\n";

                    //             FromSub1 = Bound::createBound(NewIndexVar);
                    //             BeforeSubstitution.push_back(From1->expr());
                    //             AfterSubstitution.push_back(FromSub1->expr());
                    //             NewIndexVar = Z3::index_var();
                    //             ExtraCond.push_back(NewIndexVar == To1->expr());
                    //             let_defs = space(4) + "let " + toString(NewIndexVar) + " = " + toString(To1->expr()) + "\n";
                    //             errs() << let_defs;
                    //             code += let_defs;
                    //             // errs() << space(4) << "let b" << NewIndexVar << " = Select "<<NewIndexVar<<"\n";

                    //             ToSub1 = Bound::createBound(NewIndexVar);
                    //             BeforeSubstitution.push_back(To1->expr());
                    //             AfterSubstitution.push_back(ToSub1->expr());

                    //             // errs() << space(4) << "let b" << From_constant << " = Select "<<From_constant<<"\n";
                    //             //skip the next item because it is used here
                    //             i++;
                    //             continue;
                    //         }
                    //         //b[7] b[ii1...constant]?
                    //         else if(isa<SymbolicBound>(From.get()))
                    //         {   
                    //             llvm_unreachable("Next item from symbolic. Implement it!");
                    //         }
                    //         else
                    //         {
                    //             llvm_unreachable("Next item unimplemented!");
                    //         }
                    //     }
                    //     else 
                    //     {
                    //         llvm_unreachable("Next item is not interval?");
                    //     }

                    // }
                    // else
                    // {
                    //     llvm_unreachable("from < 0?");
                    // }
                }
                //has symbolic index
                else if(isa<SymbolicBound>(From.get()) && isa<SymbolicBound>(To.get()))
                {
                    BoundRef FromSub, ToSub;
                    auto NewIndexVar = Z3::index_var();
                    ExtraCond.push_back(NewIndexVar == From->expr());
                    std::string let_defs;
                    let_defs = space(4) + "let " + toString(NewIndexVar) + " = " + toString(From->expr()) + "\n";
                    errs() << let_defs;
                    code += let_defs;
                    // errs() <<space(4) << ExtraCond.back()<<"\n";
                    FromSub = Bound::createBound(NewIndexVar);
                    BeforeSubstitution.push_back(From->expr());
                    AfterSubstitution.push_back(FromSub->expr());
                    NewIndexVar = Z3::index_var();
                    ExtraCond.push_back(NewIndexVar == To->expr());
                    let_defs = space(4) + "let " + toString(NewIndexVar) + " = " + toString(To->expr()) + "\n";
                    errs() << let_defs;
                    code += let_defs;
                    // errs() <<space(4) << ExtraCond.back()<<"\n";
                    ToSub = Bound::createBound(NewIndexVar);
                    BeforeSubstitution.push_back(To->expr());
                    AfterSubstitution.push_back(ToSub->expr());
                    // errs() <<space(4) << "b" << (FromSub ? FromSub : From) << "_" <<(ToSub ? ToSub : To);
                }
                else if(isa<SymbolicBound>(From.get()))
                {
                    BoundRef FromSub;
                    auto NewIndexVar = Z3::index_var();
                    ExtraCond.push_back(NewIndexVar == From->expr());
                    std::string let_defs;
                    let_defs = space(4) + "let " + toString(NewIndexVar) + " = " + toString(From->expr()) + "\n";
                    errs() << let_defs;
                    code += let_defs;
                    // errs() <<space(4) << ExtraCond.back()<<"\n";
                    FromSub = Bound::createBound(NewIndexVar);
                    BeforeSubstitution.push_back(From->expr());
                    AfterSubstitution.push_back(FromSub->expr());
                }
                else if(isa<SymbolicBound>(To.get()))
                {
                    BoundRef ToSub;
                    auto NewIndexVar = Z3::index_var();
                    ExtraCond.push_back(NewIndexVar == To->expr());
                    std::string let_defs;
                    let_defs = space(4) + "let " + toString(NewIndexVar) + " = " + toString(To->expr()) + "\n";
                    errs() << let_defs;
                    code += let_defs;
                    // errs() <<space(4) << ExtraCond.back()<<"\n";
                    ToSub = Bound::createBound(NewIndexVar);
                    BeforeSubstitution.push_back(To->expr());
                    AfterSubstitution.push_back(ToSub->expr());
                }
                else
                {
                    llvm_unreachable("Unknown pattern. Not implemented!\n");
                }
            }
            else
            {
                llvm_unreachable("Error : unknown rhs type!");
            }

            if (i != Items.size() - 1) 
            {
                // errs() << " ";
            }
        }
        // add a conjunction
        if(!RHS_conjuction.empty())
        {
            Productions.push_back(RHS_conjuction);
        }
        
        if (std::next(rhs_it)== P.rhs_end()) 
        {
            // errs() << " ";
        } 
        else 
        {
            // code += space(4) + "<|\n";
            // errs() << space(4)<<"<|\n";
        }
    }

    std::string disj_parsers = space(4);
    for(int i = 0; i < Productions.size(); i++)
    {
        std::string conj_parsers = "{ ";
        for(int j = 0; j < Productions[i].size(); j++)
        {
            conj_parsers += "L" + std::to_string(Productions[i][j]) + "; ";
        }
        conj_parsers += "}";
        disj_parsers += conj_parsers;
        if(i != Productions.size() - 1)
        {
            disj_parsers += " <| ";
        }
    }
    disj_parsers += "\n";
    errs()<<disj_parsers;
    code += disj_parsers;

    if (!P.Assertions.empty()) 
    {
        for (auto Assert: P.Assertions) {
            z3::expr_vector ConjOps = Z3::find_consecutive_ops(Assert, Z3_OP_AND);
            for (auto ConjOp: ConjOps) {
                if (!ConjOp.is_true())
                {
                    std::string s = toString(ConjOp.substitute(BeforeSubstitution, AfterSubstitution));
                    if(s == "")
                    {
                        continue;
                    }
                    else
                    {
                        errs()<<"\n    ("<<s<<") is true";
                        code += "\n    (" + s + ") is true";
                    }
                }
            }
        }
        // for (auto Assert: ExtraCond) {
        //     errs() << "\n    assert(" << Assert << ")";
        // }
    } 
    
    errs() << "\n\n";
    code += "\n\n";
    return code;
}


void DDLLang::dump(StringRef FileName) {
    std::vector<std::string> CodeVec;
    CodeVec.push_back(GenHelperFunctionsInDDL());
    // for(auto P : this->Bnf->Productions)
    for(auto &P : this->Bnf->getProductions())
    {
        std::string code = Production2DDL(*P);
        CodeVec.push_back(code);
    }
    if (FileName != "-") {
        std::error_code EC;
        raw_fd_ostream PStream(FileName.str(), EC, sys::fs::F_None);
        if (PStream.has_error()) {
            errs() << "[Error] Cannot open the file <" << FileName << "> for writing.\n";
            return;
        }
        for(auto code : CodeVec)
        {
            PStream << code << "\n";
        }
        POPEYE_INFO(FileName << " dumped!");
    }
}