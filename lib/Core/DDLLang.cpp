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
 
 std::string DDLLang::toStringArgs(const z3::expr &Expr, const char* Op) {
     auto recurse = [&](auto&& self, unsigned I) -> std::string {
         if (I == Expr.num_args() - 1) {
             return toString(Expr.arg(I));
         } else {
             return std::string("(") + Op + " " + toString(Expr.arg(I)) + " " + self(self, I + 1) + ")";
         }
     };
 
     return std::string("(") + recurse(recurse, 0) + ")";
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
         std::string Ret("(ExtractBV ");
         for (unsigned I = 0; I < NumArgs + NumParams; ++I) {
             if (I < NumArgs) {
                 Ret.append(toString(Expr.arg(I)));
             } else {
                 auto ParamKind = Z3_get_decl_parameter_kind(Expr.ctx(), Decl, I - NumArgs);
                 switch (ParamKind) {
                     case Z3_PARAMETER_INT:
                         Ret.append(std::string("(BV ") + std::to_string(Z3_get_decl_int_parameter(Expr.ctx(), Decl, I - NumArgs)) + ")");
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
         return std::string("(BV ") + uint64ToHex(Num64) + ")";
     }
     else if (Z3::is_numeral_i64(Expr, Int64)) {
         if(Int64 >= 0)
         {
             return std::string("(BV ") + uint64ToHex(Int64) + ")";
         }
         else
         {
             Num64 = static_cast<uint64_t>(Int64);
             return std::string("(BV ") + uint64ToHex(Num64) + ")"; 
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
                 std::string Ret("(SelectBV ");
                 if(Z3::is_numeral_i64(Expr.arg(1), Int64))
                 {
                     Ret.append(std::string("(BV ") + std::to_string(Int64)).append("))");
                 }
                 else if(Z3::is_numeral_u64(Expr.arg(1), Num64))
                 {
                     Ret.append(std::string("(BV ") + std::to_string(Num64)).append("))");
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
                     // std::string Ret(toString(Expr.arg(0)));
                     // Ret.append(" == ").append(toString(Expr.arg(1)));
                     std::string Ret("(EqBV ");
                     Ret.append(toString(Expr.arg(0))).append(" ");
                     Ret.append(toString(Expr.arg(1)));
                     Ret.append(")");
                     return Ret;
                 }
             }
             case Z3_OP_DISTINCT: {
                 // std::string Ret(toString(Expr.arg(0)));
                 // Ret.append(" != ").append(toString(Expr.arg(1)));
                 // return Ret;
                 std::string Ret("(NeqBV ");
                 Ret.append(toString(Expr.arg(0))).append(" ");
                 Ret.append(toString(Expr.arg(1)));
                 Ret.append(")");
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
                 if(ConcatVec.empty())
                     return "(BV 0) ";
                 // int i = ConcatVec.size();
                 // for (auto &C: ConcatVec) {
                 //     std::string Power = power("256 * ", --i);
                 //     Ret += Power;
                 //     Ret += C;
                 //     if (i != 0) Ret += " + ";
                 // }
                 std::function<std::string(int)> recurse = [&](int I) -> std::string {
                     if (I == (int)ConcatVec.size() - 1) {
                         return ConcatVec[I];
                     } else {
                         return "(ConcatBV " + ConcatVec[I] + " " + recurse(I + 1) + ") ";
                     }
                 };
                 return recurse(0);
             }
             case Z3_OP_ADD:
             case Z3_OP_BADD: {
                 return toStringArgs(Expr, "AddBV");
             }
             case Z3_OP_SUB:
             case Z3_OP_BSUB: {
                 return toStringArgs(Expr, "SubBV");
             }
             case Z3_OP_MUL:
             case Z3_OP_BMUL: {
                 return toStringArgs(Expr, "MulBV");
             }
             case Z3_OP_DIV:
             case Z3_OP_BSDIV_I:
             case Z3_OP_BSDIV:
             case Z3_OP_BUDIV_I:
             case Z3_OP_BUDIV: {
                 return toStringArgs(Expr, "DivBV");
             }
             case Z3_OP_MOD:
             case Z3_OP_REM:
             case Z3_OP_BSMOD:
             case Z3_OP_BSMOD_I:
             case Z3_OP_BSREM:
             case Z3_OP_BSREM_I:
             case Z3_OP_BUREM:
             case Z3_OP_BUREM_I: {
                 return toStringArgs(Expr, "ModBV");
             }
             case Z3_OP_AND: {
                 return toStringTemplate(Expr, "&&");
             }
             case Z3_OP_BAND: {
                 return toStringArgs(Expr, "AndBV");
             }
             case Z3_OP_OR: {
                 return toStringTemplate(Expr, "||");
             }
            case Z3_OP_BOR: {
                return toStringArgs(Expr, "OrBV");
            }
 //            case Z3_OP_XOR3:
 //            case Z3_OP_BXOR:
 //            case Z3_OP_XOR: {
 //                return toStringTemplate(Expr, "^");
 //            }
             case Z3_OP_GE:
             case Z3_OP_SGEQ:
             case Z3_OP_UGEQ: {
                 return toStringArgs(Expr, "GeBV");
             }
             case Z3_OP_LE:
             case Z3_OP_SLEQ:
             case Z3_OP_ULEQ: {
                 return toStringArgs(Expr, "GeBV");
             }
             case Z3_OP_GT:
             case Z3_OP_SGT:
             case Z3_OP_UGT: {
                 return toStringArgs(Expr, "GtBV");
             }
             case Z3_OP_LT:
             case Z3_OP_SLT:
             case Z3_OP_ULT: {
                 return toStringArgs(Expr, "LtBV");
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
             case Z3_OP_BNOT: {
                 return "(NotBV " + toString(Expr.arg(0)) + ")";
             }
             case Z3_OP_NOT: {
                 return "(!" + toString(Expr.arg(0)) + ")";
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
 def RequireBits (N : uint 64) : uint 64 =
   block
   if   N < (1 <<  1) then  1
     else if N < (1 <<  2) then  2
     else if N < (1 <<  3) then  3
     else if N < (1 <<  4) then  4
     else if N < (1 <<  5) then  5
     else if N < (1 <<  6) then  6
     else if N < (1 <<  7) then  7
     else if N < (1 <<  8) then  8
     else if N < (1 <<  9) then  9
     else if N < (1 << 10) then 10
     else if N < (1 << 11) then 11
     else if N < (1 << 12) then 12
     else if N < (1 << 13) then 13
     else if N < (1 << 14) then 14
     else if N < (1 << 15) then 15
     else if N < (1 << 16) then 16
     else if N < (1 << 17) then 17
     else if N < (1 << 18) then 18
     else if N < (1 << 19) then 19
     else if N < (1 << 20) then 20
     else if N < (1 << 21) then 21
     else if N < (1 << 22) then 22
     else if N < (1 << 23) then 23
     else if N < (1 << 24) then 24
     else if N < (1 << 25) then 25
     else if N < (1 << 26) then 26
     else if N < (1 << 27) then 27
     else if N < (1 << 28) then 28
     else if N < (1 << 29) then 29
     else if N < (1 << 30) then 30
     else if N < (1 << 31) then 31
     else if N < (1 << 32) then 32
     else if N < (1 << 33) then 33
     else if N < (1 << 34) then 34
     else if N < (1 << 35) then 35
     else if N < (1 << 36) then 36
     else if N < (1 << 37) then 37
     else if N < (1 << 38) then 38
     else if N < (1 << 39) then 39
     else if N < (1 << 40) then 40
     else if N < (1 << 41) then 41
     else if N < (1 << 42) then 42
     else if N < (1 << 43) then 43
     else if N < (1 << 44) then 44
     else if N < (1 << 45) then 45
     else if N < (1 << 46) then 46
     else if N < (1 << 47) then 47
     else if N < (1 << 48) then 48
     else if N < (1 << 49) then 49
     else if N < (1 << 50) then 50
     else if N < (1 << 51) then 51
     else if N < (1 << 52) then 52
     else if N < (1 << 53) then 53
     else if N < (1 << 54) then 54
     else if N < (1 << 55) then 55
     else if N < (1 << 56) then 56
     else if N < (1 << 57) then 57
     else if N < (1 << 58) then 58
     else if N < (1 << 59) then 59
     else if N < (1 << 60) then 60
     else if N < (1 << 61) then 61
     else if N < (1 << 62) then 62
     else if N < (1 << 63) then 63
     else 64
 
 def BV (N : uint 64) = 
   block
     n    = N
     bits = RequireBits N
 
 def EqBV bv1 bv2 = 
   block
     bv1.n == bv2.n
 
 def NeqBV bv1 bv2 = 
   block
     bv1.n != bv2.n
 
 def LtBV bv1 bv2 =
   block
     bv1.n < bv2.n
 
 def GtBV bv1 bv2 =
   block
     bv1.n > bv2.n
 
 def LeBV bv1 bv2 =
   block
     bv1.n <= bv2.n
 
 def GeBV bv1 bv2 =
   block
     bv1.n >= bv2.n
 
 def MulBV bv1 bv2 = 
   block
     n = bv1.n * bv2.n
     bits = RequireBits n
 
 def AddBV bv1 bv2 =
   block
     n = bv1.n + bv2.n
     bits = RequireBits n
 
 def SubBV bv1 bv2 =
   block
     n = bv1.n - bv2.n
     bits = RequireBits n
 
 def ModBV bv1 bv2 =
   block
     n = bv1.n % bv2.n
     bits = RequireBits n
 
 def DivBV bv1 bv2 =
   block
     n = bv1.n / bv2.n
     bits = RequireBits n
 
 def AndBV bv1 bv2 =
   block
     bits = if bv1.bits > bv2.bits then bv1.bits else bv2.bits
     let mask = (1 << bits) - 1
     n = (bv1.n .&. bv2.n) .&. mask
 
 def OrBV bv1 bv2 =
   block
     bits = if bv1.bits > bv2.bits then bv1.bits else bv2.bits
     let mask = (1 << bits) - 1
     n = (bv1.n .|. bv2.n) .&. mask
 
 def XorBV bv1 bv2 =
   block
     bits = if bv1.bits > bv2.bits then bv1.bits else bv2.bits
     let mask = (1 << bits) - 1
     n = (bv1.n .^. bv2.n) .&. mask
 
 def NotBV bv = 
   block
     let mask = (1 << bv.bits) - 1
     n = (~bv.n) .&. mask
     bits = bv.bits
 
 def GetN bv =
   bv.n
 
 def Select (N : uint 64) =
   block
     let cur = GetStream
     let a = bytesOfStream cur
     (Index a N) as uint 64
 
 def SelectBV bv =
   block
     n = Select bv.n
     bits = 8 : uint 64
 
 def Len =
   block
     let cur = GetStream
     let a = bytesOfStream cur
     length a
 
 def LenBV =
   block
     BV Len
 
 def Extract (N : uint 64) (High : uint 64) (Low : uint 64) =
   block
     let mask = (1 << (High - Low + 1)) - 1
     (N >> Low) .&. mask
 
 def ExtractBV N High Low = 
   block
     n = Extract N.n High.n Low.n
     bits = High.n - Low.n + 1
 
 def ConcatBV bv1 bv2 = 
   block
     let mask2 = if bv2.bits == 64 then (-1 as uint 64)
                 else (1 << bv2.bits) - 1
     n    = (bv1.n << bv2.bits) .|. (bv2.n .&. mask2)
     bits = bv1.bits + bv2.bits
 */
 
 std::string GenHelperFunctionsInDDL()
 {
     std::string s;
 
     auto space = [](int n) { return std::string(n, ' '); };
 
     /* ---------- require bits ---------- */
     s += "def RequireBits (N : uint 64) : uint 64=\n";
     s += space(2) + "block\n";
     s += space(4) + "if N < (1 << 1) then 1\n";
     for (int i = 2; i <= 63; ++i)
     {
         s += space(6) + "else if N < (1 << " + std::to_string(i) + ") then " + std::to_string(i) + "\n";
     }
     s += space(6) + "else 64" + "\n\n";
 
     /* ---------- bit‑vector wrapper ---------- */
     s += "def BV (N : uint 64) =\n";
     s += space(2) + "block\n";
     s += space(4) + "n    = N\n";
     s += space(4) + "bits = RequireBits N\n\n";
 
     /* ---------- equality ---------- */
     s += "def EqBV bv1 bv2 =\n";
     s += space(2) + "block\n";
     s += space(4) + "bv1.n == bv2.n\n\n";
 
     s += "def NeqBV bv1 bv2 =\n";
     s += space(2) + "block\n";
     s += space(4) + "bv1.n != bv2.n\n\n";
 
     /* ---------- comparison ---------- */
     s += "def LtBV bv1 bv2 =\n";
     s += space(2) + "block\n";
     s += space(4) + "bv1.n < bv2.n\n\n";
 
     s += "def GtBV bv1 bv2 =\n";
     s += space(2) + "block\n";
     s += space(4) + "bv1.n > bv2.n\n\n";
 
     s += "def LeBV bv1 bv2 =\n";
     s += space(2) + "block\n";
     s += space(4) + "bv1.n <= bv2.n\n\n";
 
     s += "def GeBV bv1 bv2 =\n";
     s += space(2) + "block\n";
     s += space(4) + "bv1.n >= bv2.n\n\n";
 
     /* ---------- arithmetic ---------- */
     s += "def MulBV bv1 bv2 =\n";
     s += space(2) + "block\n";
     s += space(4) + "n = bv1.n * bv2.n\n";
     s += space(4) + "bits = RequireBits n\n\n";
 
     s += "def AddBV bv1 bv2 =\n";
     s += space(2) + "block\n";
     s += space(4) + "n = bv1.n + bv2.n\n";
     s += space(4) + "bits = RequireBits n\n\n";
 
     s += "def SubBV bv1 bv2 =\n";
     s += space(2) + "block\n";
     s += space(4) + "n = bv1.n - bv2.n\n";
     s += space(4) + "bits = RequireBits n\n\n";
 
     s += "def ModBV bv1 bv2 =\n";
     s += space(2) + "block\n";
     s += space(4) + "n = bv1.n % bv2.n\n";
     s += space(4) + "bits = RequireBits n\n\n";
 
     s += "def DivBV bv1 bv2 =\n";
     s += space(2) + "block\n";
     s += space(4) + "n = bv1.n / bv2.n\n";
     s += space(4) + "bits = RequireBits n\n\n";
 
     /* ---------- logical ops ---------- */
     s += "def AndBV bv1 bv2 =\n";
     s += space(2) + "block\n";
     s += space(4) + "bits = if bv1.bits > bv2.bits then bv1.bits else bv2.bits\n";
     s += space(4) + "let mask = (1 << bits) - 1\n";
     s += space(4) + "n = (bv1.n .&. bv2.n) .&. mask\n\n";
 
     s += "def OrBV bv1 bv2 =\n";
     s += space(2) + "block\n";
     s += space(4) + "bits = if bv1.bits > bv2.bits then bv1.bits else bv2.bits\n";
     s += space(4) + "let mask = (1 << bits) - 1\n";
     s += space(4) + "n = (bv1.n .|. bv2.n) .&. mask\n\n";
 
     s += "def XorBV bv1 bv2 =\n";
     s += space(2) + "block\n";
     s += space(4) + "bits = if bv1.bits > bv2.bits then bv1.bits else bv2.bits\n";
     s += space(4) + "let mask = (1 << bits) - 1\n";
     s += space(4) + "n = (bv1.n .^. bv2.n) .&. mask\n\n";
 
     /* ---------- logical not ---------- */
     s += "def NotBV bv =\n";
     s += space(2) + "block\n";
     s += space(4) + "let mask = (1 << bv.bits) - 1\n";
     s += space(4) + "n = (~bv.n) .&. mask\n";
     s += space(4) + "bits = bv.bits\n\n";
 
     /* ---------- accessor ---------- */
     s += "def GetN bv =\n";
     s += space(2) + "bv.n\n\n";
 
     /* ---------- byte‑oriented primitives ---------- */
     s += "def Select (N : uint 64) =\n";
     s += space(2) + "block\n";
     s += space(4) + "let cur = GetStream\n";
     s += space(4) + "let a = bytesOfStream cur\n";
     s += space(4) + "(Index a N) as uint 64\n\n";
 
     s += "def SelectBV bv =\n";
     s += space(2) + "block\n";
     s += space(4) + "n = Select bv.n\n";
     s += space(4) + "bits = 8 : uint 64\n\n";
 
     s += "def Len =\n";
     s += space(2) + "block\n";
     s += space(4) + "let cur = GetStream\n";
     s += space(4) + "let a = bytesOfStream cur\n";
     s += space(4) + "length a\n\n";
 
     s += "def LenBV =\n";
     s += space(2) + "block\n";
     s += space(4) + "BV Len\n\n";
 
     /* ---------- slicing ---------- */
     s += "def Extract (N : uint 64) (High : uint 64) (Low : uint 64) =\n";
     s += space(2) + "block\n";
     s += space(4) + "let mask = (1 << (High - Low + 1)) - 1\n";
     s += space(4) + "(N >> Low) .&. mask\n\n";
 
     s += "def ExtractBV N High Low =\n";
     s += space(2) + "block\n";
     s += space(4) + "n = Extract N.n High.n Low.n\n";
     s += space(4) + "bits = High.n - Low.n + 1\n\n";
 
     /* ---------- concatenation ---------- */
     s += "def ConcatBV bv1 bv2 =\n";
     s += space(2) + "block\n";
     s += space(4) + "let mask2 = if bv2.bits == 64 then (-1 as uint 64)\n";
     s += space(8) + "else (1 << bv2.bits) - 1\n";
     s += space(4) + "n    = (bv1.n << bv2.bits) .|. (bv2.n .&. mask2)\n";
     s += space(4) + "bits = bv1.bits + bv2.bits\n\n";
 
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
         std::string lhs = "def Main = \n" + space(2) + "block\n" + space(4) + "let len = LenBV\n";
         code += lhs;
         errs()<<lhs;
     }
     else
     {
         std::string lhs = "def L" + std::to_string(P.getLHS()) + " = \n" + space(2) + "block\n" + space(4) + "let len = LenBV\n";
         code += lhs;
         errs()<<lhs;
     }
 
     for(auto rhs_it = P.rhs_begin(); rhs_it != P.rhs_end(); rhs_it = std::next(rhs_it))
     {
         auto Items = *rhs_it;
         for (unsigned i = 0; i < Items.size(); ++i) 
         {
             RHSItem *Item = Items[i];
             if (auto *PItem = dyn_cast<Production>(Item)) 
             {
                 std::string rhs_l = space(4) + "L" + std::to_string(PItem->getLHS());
                 code += rhs_l;
                 errs()<<rhs_l;
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
                 code += "\n";
                 errs()<<"\n";
             }
         }
         if (std::next(rhs_it)== P.rhs_end()) 
         {
             // errs() << " ";
         } 
         else 
         {
             code += " <| ";
             errs() << " <| ";
         }
         
     }
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