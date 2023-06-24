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

#include <llvm/Support/CommandLine.h>

#include "Z3Macro.h"

static cl::opt<bool> CompleteStrLen(
        "popeye-enable-z3-strlen",
        cl::desc("enable z3 strlen"),
        cl::init(false));

z3::expr Z3::byte_array() {
    auto &Ctx = Z3::bool_val(true).ctx();
    return {Ctx,
            Z3_mk_const(
                    Ctx,
                    Z3_mk_string_symbol(Ctx, BYTE_ARRAY),
                    Ctx.array_sort(Ctx.bv_sort(64), Ctx.bv_sort(8))
            )
    };
}

z3::expr Z3::byte_array_element(const z3::expr &Array, int Index) {
    assert(Array.is_array());
    return z3::select(Array, Index);
}

z3::expr Z3::byte_array_element(const z3::expr &Array, const z3::expr &Index) {
    assert(Array.is_array());
    assert(Index.is_bv());

    auto IndexBVSz = Index.get_sort().bv_size();
    assert(IndexBVSz <= 64);
    auto RealIndex = IndexBVSz < 64 ? Z3::zext(Index, 64 - IndexBVSz) : Index;
    return z3::select(Array, RealIndex);
}

z3::expr Z3::byte_array_range(const z3::expr &Array, const z3::expr &F, const z3::expr &T) {
    auto IV = Z3::vec();
    IV.push_back(Array);
    IV.push_back(F);
    IV.push_back(T);

    std::vector<z3::sort> SortVec;
    SortVec.push_back(Array.get_sort());
    SortVec.push_back(F.get_sort());
    SortVec.push_back(T.get_sort());

    std::string Range(BYTE_ARRAY_RANGE);
    auto RangeFunc = z3::function(Range.c_str(), SortVec.size(), &SortVec[0], SortVec[1]);
    return RangeFunc(IV);
}

bool Z3::is_byte_array_range(const z3::expr &E) {
    if (E.num_args() != 3) return false;
    if (!E.arg(0).is_array()) return false;
    return E.decl().name().str() == BYTE_ARRAY_RANGE;
}

bool Z3::is_byte_eq_zero(const z3::expr &E) {
    if (!E.is_eq()) return false;
    if (E.arg(0).decl().decl_kind() != Z3_OP_SELECT) return false;
    if (!Z3::is_zero(E.arg(1))) return false;
    return true;
}

bool Z3::is_byte_eq_minus_one(const z3::expr &E) {
    if (!E.is_eq()) return false;
    if (E.arg(0).decl().decl_kind() != Z3_OP_SELECT) return false;
    if (!Z3::is_minus_one(E.arg(1))) return false;
    return true;
}

static int truncLessThan(const z3::expr_vector &V1, const z3::expr_vector &V2) {
    // 1 - true
    // 0 - false
    // -1 - unknown
    unsigned Bits2Remove = 0;
    for (auto E: V1) {
        if (E.decl().decl_kind() == Z3_OP_CONCAT) {
            auto Head = E.arg(0);
            int64_t Const;
            if (Z3::is_numeral_i64(Head, Const) && Const == 0) {
                Bits2Remove = Head.get_sort().bv_size();
                break;
            } else if (Head.decl().decl_kind() == Z3_OP_EXTRACT && Head.get_sort().bv_size() == 1) {
                // this may be a signed ext
                unsigned K = 1;
                for (; K < E.num_args() - 1; K++) {
                    if (!Z3::same(Head, E.arg(K))) {
                        break;
                    }
                }
                if (Z3::same(Head.arg(0), E.arg(K))) {
                    Bits2Remove = K;
                    break;
                }
            }
        }
    }
    if (Bits2Remove == 0)
        return -1;
    unsigned Bits2Preserve = V1[0].get_sort().bv_size() - Bits2Remove;

    auto TruncHeadZero = [Bits2Remove, Bits2Preserve](const z3::expr_vector &V) {
        auto Ret = Z3::vec();
        int64_t Const;
        for (auto E: V) {
            if (Z3::is_numeral_i64(E, Const)) {
                Ret.push_back(Z3::bv_val(Const, Bits2Preserve));
            } else {
                if (E.decl().decl_kind() == Z3_OP_CONCAT) {
                    auto Head = E.arg(0);
                    if (Z3::is_numeral_i64(Head, Const) && Const == 0) {
                        if (Head.get_sort().bv_size() == Bits2Remove) {
                            auto ConcatVec = Z3::vec();
                            for (unsigned K = 1; K < E.num_args(); ++K)
                                ConcatVec.push_back(E.arg(K));
                            Ret.push_back(z3::concat(ConcatVec));
                        } else if (Head.get_sort().bv_size() > Bits2Remove) {
                            auto ConcatVec = Z3::vec();
                            ConcatVec.push_back(Z3::bv_val(0, Head.get_sort().bv_size() - Bits2Remove));
                            for (unsigned K = 1; K < E.num_args(); ++K)
                                ConcatVec.push_back(E.arg(K));
                            Ret.push_back(z3::concat(ConcatVec));
                        } else {
                            break;
                        }
                    } else if (Head.decl().decl_kind() == Z3_OP_EXTRACT && Head.get_sort().bv_size() == 1) {
                        if (Bits2Remove < E.num_args() &&
                            (Z3::same(E.arg(Bits2Remove), Head.arg(0)) || Z3::same(E.arg(Bits2Remove), Head))) {
                            auto ConcatVec = Z3::vec();
                            for (unsigned K = Bits2Remove; K < E.num_args(); ++K)
                                ConcatVec.push_back(E.arg(K));
                            Ret.push_back(z3::concat(ConcatVec));
                        } else {
                            break;
                        }
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            }
        }
        return Ret;
    };
    auto NV1 = TruncHeadZero(V1);
    if (NV1.size() != V1.size()) return -1;
    auto NV2 = TruncHeadZero(V2);
    if (NV2.size() != V2.size()) return -1;

    auto BvSum = [](const z3::expr_vector &V) {
        // note: z3::sum() does not support bit vector
        assert(V.size() > 0);
        auto Ret = V[0];
        for (unsigned K = 1; K < V.size(); ++K) {
            Ret = Ret + V[K];
        }
        return Ret.simplify();
    };
    auto N1 = BvSum(NV1);
    auto N2 = BvSum(NV2);
    return Z3::byte_array_element_index_less_than(N1, N2) ? 1 : 0;
}

bool Z3::byte_array_element_index_less_than(const z3::expr &A, const z3::expr &B) {
//    if (Z3::to_string(B) == "3 + K1 x 9 + strlem" && Z3::to_string(A) == "6 + 9 x K1 + strlem + K1 x strlem") {
//        outs() << "";
//    }

    // first try
    auto Res = Z3::ult(A, B);
    if (Res.is_true())
        return true;
    if (Res.is_false())
        return false;

    auto O1Ops = Z3::find_consecutive_ops(A, Z3_OP_BADD, true);
    auto O2Ops = Z3::find_consecutive_ops(B, Z3_OP_BADD, true);

    // move negative items to the other side
    auto MoveNegativeItem = [](z3::expr_vector &O1Ops, z3::expr_vector &O2Ops) {
        for (unsigned I = 0; I < O1Ops.size(); ++I) {
            auto E = O1Ops[I];
            int64_t Const;
            if (E.decl().decl_kind() == Z3_OP_BMUL) {
                assert(E.num_args() == 2);
                if (Z3::is_numeral_i64(E.arg(0), Const) && Const < 0) {
                    auto Back = O1Ops.back();
                    O1Ops.set(I--, Back);
                    O1Ops.pop_back();
                    O2Ops.push_back(Const == -1 ? E.arg(1) : ((int) -Const) * E.arg(1));
                } else if (Z3::is_numeral_i64(E.arg(1), Const) && Const < 0) {
                    auto Back = O1Ops.back();
                    O1Ops.set(I--, Back);
                    O1Ops.pop_back();
                    O2Ops.push_back(Const == -1 ? E.arg(0) : ((int) -Const) * E.arg(0));
                }
            } else if (Z3::is_numeral_i64(E, Const)) {
                if (Const == 0) {
                    auto Back = O1Ops.back();
                    O1Ops.set(I--, Back);
                    O1Ops.pop_back();
                } else if (Const < 0) {
                    auto Back = O1Ops.back();
                    O1Ops.set(I--, Back);
                    O1Ops.pop_back();
                    O2Ops.push_back(Z3::bv_val(-Const, E.get_sort().bv_size()));
                }
            }
        }
    };
    MoveNegativeItem(O1Ops, O2Ops);
    MoveNegativeItem(O2Ops, O1Ops);

    // a x (b + c) => a x b + a x c
    auto DistributiveLaw = [](z3::expr_vector &Ops) {
        auto Size = Ops.size();
        for (unsigned I = 0; I < Size; ++I) {
            auto E = Ops[I];
            if (E.decl().decl_kind() != Z3_OP_BMUL) continue;
            auto A = E.arg(0);
            auto B = E.arg(1);
            if (A.decl().decl_kind() == Z3_OP_BADD && B.decl().decl_kind() != Z3_OP_BADD) {
                auto X = A;
                A = B;
                B = X;
            }
            if (A.decl().decl_kind() == Z3_OP_BADD || B.decl().decl_kind() != Z3_OP_BADD) continue;
            // B must be an add op
            auto Bs = Z3::find_consecutive_ops(B, Z3_OP_BADD, true);
            assert(!Bs.empty());
            auto FirstSubB = Bs[0];
            auto First = Z3::mul(FirstSubB, A);
            Ops.set(I, First);
            for (unsigned K = 1; K < Bs.size(); ++K) {
                Ops.push_back(Z3::mul(Bs[K], A));
            }
        }
    };
    DistributiveLaw(O1Ops);
    DistributiveLaw(O2Ops);

    // expand mul op: 3 * a => a + a + a
    auto ExpandMul = [](z3::expr_vector &Ops) {
        auto Size = Ops.size();
        for (unsigned I = 0; I < Size; ++I) {
            auto E = Ops[I];
            if (E.decl().decl_kind() == Z3_OP_BMUL) {
                assert(E.num_args() == 2);
                int64_t Const = -1;
                auto Op = Z3::bv_val(0, E.get_sort().bv_size()); // just init the expr
                if (Z3::is_numeral_i64(E.arg(0), Const)) {
                    assert(Const > 0);
                    Op = E.arg(1);
                } else if (Z3::is_numeral_i64(E.arg(1), Const)) {
                    assert(Const > 0);
                    Op = E.arg(0);
                }
                if (Const > 0) {
                    Ops.set(I, Op);
                    Const--;
                    while (Const-- > 0) {
                        Ops.push_back(Op);
                    }
                }
            }
        }
    };
    ExpandMul(O1Ops);
    ExpandMul(O2Ops);

    // remove the same items in two sides
    for (unsigned I = 0; I < O1Ops.size(); ++I) {
        auto E1 = O1Ops[I];
        int64_t Const1;
        bool E1Const = Z3::is_numeral_i64(E1, Const1);
        int64_t OldConst1 = Const1;

        bool FoundSame = false;
        for (unsigned J = 0; J < O2Ops.size(); ++J) {
            auto E2 = O2Ops[J];
            int64_t Const2;
            bool E2Const = Z3::is_numeral_i64(E2, Const2);

            if (Z3::same(E1, E2)) {
                // remove E2 from O2Ops
                auto Back2 = O2Ops.back();
                O2Ops.set(J, Back2);
                O2Ops.pop_back();
                FoundSame = true;
                break;
            } else if (E1Const && E2Const) {
                assert(Const1 > 0 && Const2 > 0);
                if (Const1 > Const2) {
                    // remove E2 from O2Ops
                    auto Back2 = O2Ops.back();
                    O2Ops.set(J, Back2);
                    O2Ops.pop_back();

                    Const1 -= Const2;
                } else {
                    assert(Const1 < Const2);
                    FoundSame = true;
                    auto New = Z3::bv_val(Const2 - Const1, E2.get_sort().bv_size());
                    O2Ops.set(J, New);
                    break;
                }
            }
        }
        if (FoundSame) {
            // remove E1 from O1Ops
            auto Back1 = O1Ops.back();
            O1Ops.set(I, Back1);
            O1Ops.pop_back();
            I--;
        } else if (OldConst1 > Const1) {
            auto New = Z3::bv_val(Const1, E1.get_sort().bv_size());
            O1Ops.set(I, New);
        }
    }

    if (O1Ops.empty() && !O2Ops.empty()) {
        LLVM_DEBUG(dbgs() << A << " < " << B << " yields true!\n");
        return true;
    } else if (!O1Ops.empty() && O2Ops.empty()) {
        LLVM_DEBUG(dbgs() << A << " < " << B << " yields false (>)!\n");
        return false;
    } else if (O1Ops.empty() && O2Ops.empty()) {
        LLVM_DEBUG(dbgs() << A << " < " << B << " yields false (=)!\n");
        return false;
    } else {
        int64_t Const;
        if (O1Ops.size() == 1 && Z3::is_numeral_i64(O1Ops[0], Const)) {
            LLVM_DEBUG(dbgs() << A << " < " << B << " yields true (*)!\n");
            return true;
        } else if (O2Ops.size() == 1 && Z3::is_numeral_i64(O2Ops[0], Const)) {
            LLVM_DEBUG(dbgs() << A << " < " << B << " yields false (*)!\n");
            return false;
        } else {
            auto Ret = truncLessThan(O1Ops, O2Ops);
            if (Ret != -1) {
                LLVM_DEBUG(dbgs() << A << " < " << B << " yields " << Ret << " (*)!\n");
                return Ret;
            } else {
                Ret = truncLessThan(O2Ops, O1Ops);
                if (Ret != -1) {
                    LLVM_DEBUG(dbgs() << A << " < " << B << " yields " << !Ret << " (*)!\n");
                    return Ret == 0 ? 1 : 0;
                }
            }
        }

        LLVM_DEBUG(
                dbgs() << A << " < " << B << "\n-----\nO1Ops: \n";
                for (auto X: O1Ops) dbgs() << X << ", ";
                dbgs() << "\n-----\nO2Ops: \n";
                for (auto X: O2Ops) dbgs() << X << ", ";
                dbgs() << "\n-----\n"
        );

        std::string ErrMsg;
        raw_string_ostream Str(ErrMsg);
        Str << "Error: not comparable...\n\n";
        Str << A << " < " << B << "\n-----\nO1Ops: \n";
        for (auto X: O1Ops) Str << X << ", ";
        Str << "\n-----\nO2Ops: \n";
        for (auto X: O2Ops) Str << X << ", ";
        Str << "\n-----\n";
        Str.flush();
        throw std::runtime_error(ErrMsg);
    }
}

z3::expr Z3::strlem(const z3::expr &FirstByte, int BitWidth) {
    assert(FirstByte.decl().decl_kind() == Z3_OP_SELECT);

    if (CompleteStrLen) {
        // printing too much info may make it noisy..
        z3::sort DomainSorts[2] = {FirstByte.arg(0).get_sort(), FirstByte.arg(1).get_sort()};
        auto RangeSort = FirstByte.ctx().bv_sort(BitWidth);
        auto StrLenX = z3::function(STR_LEN, 2, DomainSorts, RangeSort);
        return StrLenX(FirstByte.arg(0), FirstByte.arg(1)) + 1;
    }
    return Z3::bv_const(STR_LEM, BitWidth);
}