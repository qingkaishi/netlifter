/*
 *  Popeye lifts protocol source code in C to its specification in BNF
 *  Copyright (C) 2023 Qingkai Shi <qingkaishi@gmail.com>
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

#include "Core/Executor.h"


GlobalMemoryBlock *Executor::isConstantString(Value *PtrVal) {
    auto *PtrAbsVal = dyn_cast_or_null<AddressValue>(ES->boundValue(PtrVal));
    if (!PtrAbsVal || PtrAbsVal->poison() || PtrAbsVal->size() != 1) return nullptr;
    auto *GM = dyn_cast_or_null<GlobalMemoryBlock>(PtrAbsVal->base(0));
    // constant string should be defined as a global var in llvm bitcode
    if (!GM) return nullptr;
    // we never find a case with a positive offset for constant string
    auto OffsetExpr = PtrAbsVal->offset(0);
    uint64_t Offset;
    if (!Z3::is_numeral_u64(OffsetExpr, Offset) || Offset != 0) return nullptr;
    // string in llvm bitcode is a constant data array
    if (!GM->isConstantDataArray()) return nullptr;
    auto *FirstByte = GM->at(Offset);
    if (!isa_and_nonnull<ScalarValue>(FirstByte)) return nullptr;
    // the constant data array must be a byte array
    if (FirstByte->bytewidth() != 1) return nullptr;
    assert(!FirstByte->poison());
    return GM;
}

void Executor::visitStrLen(CallInst &I) {
    visitCallDefault(I);
    if ((I.getNumArgOperands() != 1 && I.getNumArgOperands() != 2) || !I.getType()->isIntegerTy()) return;
    auto *StrPtr = I.getArgOperand(0); // the str
    auto *MaxLen = I.getNumArgOperands() == 2 ? I.getArgOperand(1) : nullptr; // strnlen instead of strlen
    auto *MaxLenAbsVal = MaxLen ? ES->boundValue(MaxLen) : nullptr;
    if (auto *ConstString = isConstantString(StrPtr)) {
        size_t MaxLenConst;
        if (!MaxLenAbsVal || !MaxLenAbsVal->uint64(MaxLenConst)) // if not a const, treat it as strlen
            MaxLenConst = UINT64_MAX;
        size_t Len = ConstString->size() - 1; // remove the tail '\0'
        if (Len > MaxLenConst) Len = MaxLenConst;
        auto RetAbsVal = ES->boundValue(&I);
        RetAbsVal->set(Z3::bv_val(Len, RetAbsVal->bytewidth() * 8));
    } else {
        auto *StrPtrAbsVal = dyn_cast<AddressValue>(ES->boundValue(StrPtr));
        if (StrPtrAbsVal->size() == 1 && isa<MessageBuffer>(StrPtrAbsVal->base(0))) {
            // treat it as strlen
            auto Offset = StrPtrAbsVal->offset(0);
            auto InitByte = StrPtrAbsVal->base(0)->at(Offset)->value();
            auto RetAbsVal = ES->boundValue(&I);
            RetAbsVal->set(Z3::strlem(InitByte, RetAbsVal->bytewidth() * 8) - 1);
            return;
        }

        // default path
        return;
    }
}

void Executor::visitStrNCmp(CallInst &I) {
    visitCallDefault(I);
    if (I.getNumArgOperands() != 3 || !I.getType()->isIntegerTy()) return;

    auto N = I.getArgOperand(2);
    auto NVal = ES->boundValue(N);
    if (!isa_and_nonnull<ScalarValue>(NVal) || NVal->poison()) return;

    uint64_t Len;
    if (!NVal->uint64(Len) || Len == 0 || Len > 20) return;

    auto *Op1 = I.getArgOperand(0); // the const str
    auto *Op2 = I.getArgOperand(1); // the buffer

    GlobalMemoryBlock *ConstString = nullptr;
    if (auto *Ret1 = isConstantString(Op1)) {
        ConstString = Ret1;
    } else if (auto *Ret2 = isConstantString(Op2)) {
        ConstString = Ret2;
        auto *Tmp = Op1;
        Op1 = Op2;
        Op2 = Tmp;
    } else {
        return;
    }
    if (ConstString->size() < Len) return;

    auto *Op2AbsVal = dyn_cast_or_null<AddressValue>(ES->boundValue(Op2));
    if (!Op2AbsVal || Op2AbsVal->poison() || Op2AbsVal->size() != 1) return;
    auto *M = dyn_cast_or_null<MessageBuffer>(Op2AbsVal->base(0));
    if (!M) return;
    auto Off = Op2AbsVal->offset(0);
    auto ResExpr = M->at(Off)->value() == ConstString->at(0)->value();
    for (unsigned K = 1; K < Len; ++K) {
        auto NewOff = Z3::add(Off, Z3::bv_val(K, Off.get_sort().bv_size()));
        ResExpr = ResExpr && M->at(NewOff)->value() == ConstString->at(K)->value();
    }
    auto *ResAbsVal = dyn_cast_or_null<ScalarValue>(ES->boundValue(&I));
    ResAbsVal->set(Z3::bool_to_bv(Z3::negation(ResExpr), ResAbsVal->bytewidth() * 8));
}

void Executor::visitStrCmp(CallInst &I) {
    visitCallDefault(I);
    if (I.getNumArgOperands() != 2 || !I.getType()->isIntegerTy()) return;

    auto *Op1 = I.getArgOperand(0); // the const str
    auto *Op2 = I.getArgOperand(1); // the buffer

    GlobalMemoryBlock *ConstString = nullptr;
    if (auto *Ret1 = isConstantString(Op1)) {
        ConstString = Ret1;
    } else if (auto *Ret2 = isConstantString(Op2)) {
        ConstString = Ret2;
        auto *Tmp = Op1;
        Op1 = Op2;
        Op2 = Tmp;
    } else {
        return;
    }
    unsigned Len = ConstString->size();
    if (Len > 20) return;

    auto *Op2AbsVal = dyn_cast_or_null<AddressValue>(ES->boundValue(Op2));
    if (!Op2AbsVal) return;
    if (Op2AbsVal->poison() || Op2AbsVal->size() != 1) return;
    auto *M = dyn_cast_or_null<MessageBuffer>(Op2AbsVal->base(0));
    if (!M) return;
    auto Off = Op2AbsVal->offset(0);
    auto ResExpr = M->at(Off)->value() == ConstString->at(0)->value();
    for (unsigned K = 1; K < Len; ++K) {
        auto NewOff = Z3::add(Off, Z3::bv_val(K, Off.get_sort().bv_size()));
        ResExpr = ResExpr && M->at(NewOff)->value() == ConstString->at(K)->value();
    }
    auto *ResAbsVal = dyn_cast_or_null<ScalarValue>(ES->boundValue(&I));
    ResAbsVal->set(Z3::bool_to_bv(Z3::negation(ResExpr), ResAbsVal->bytewidth() * 8));
}