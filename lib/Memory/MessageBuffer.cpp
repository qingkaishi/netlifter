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

#include "Memory/MessageBuffer.h"
#include "Support/Debug.h"

using namespace llvm;

#define DEBUG_TYPE "MessageBuffer"

MessageBuffer::MessageBuffer(Type *Ty, unsigned Num) : MemoryBlock(Ty, Num, MK_Message), Data(Z3::byte_array()) {
    assert(Ty->isIntOrIntVectorTy(8));
}

MessageBuffer::~MessageBuffer() {
    for (auto &It: VariableOffsetStore)
        delete It.second;
    VariableOffsetStore.clear();
}

AbstractValue *MessageBuffer::at(size_t ID) {
    if (ID >= Mem.size()) {
        while (!Mem.back())
            Mem.pop_back();

        size_t Delta = ID - Mem.size() + 1;
        while (Delta-- > 0) {
            Mem.push_back(new ScalarValue(1, Z3::byte_array_element(Data, (int) Mem.size())));
        }
    }
    auto *AV = Mem[ID];
    if (AV->poison()) {
        AV->set(Z3::byte_array_element(Data, (int) ID));
    }
    return AV;
}

z3::expr MessageBuffer::at(const z3::expr &Start, size_t Size, bool Swap) {
    auto Vec = Z3::vec();
    if (Swap) {
        for (int I = Size; I > 0; --I) {
            auto Offset = Z3::add(Start, I - 1);
            Vec.push_back(at(Offset)->value());
        }
    } else {
        for (int I = 0; I < Size; ++I) {
            auto Offset = Z3::add(Start, I);
            Vec.push_back(at(Offset)->value());
        }
    }
    return Z3::concat(Vec);
}

AbstractValue *MessageBuffer::at(const z3::expr &ID) {
    auto NormalizedID = ID.simplify();
    uint64_t Const;
    if (Z3::is_numeral_u64(NormalizedID, Const)) {
        return at(Const);
    }

    auto It = VariableOffsetStore.find(NormalizedID);
    if (It != VariableOffsetStore.end()) {
        POPEYE_DEBUG(dbgs() << "load " << It->second->value() << " from " << NormalizedID << "\n");
        return It->second;
    } else {
        auto *Ret = new ScalarValue(1, Z3::byte_array_element(Data, NormalizedID));
        VariableOffsetStore.insert(std::make_pair(NormalizedID, Ret));
        return Ret;
    }
}

void MessageBuffer::store(const z3::expr &Val, const z3::expr Offset) {
    auto ValBitWidth = Val.get_sort().bv_size();
    assert(ValBitWidth % 8 == 0);
    auto ByteWidth = ValBitWidth / 8;
    auto OffsetBitWidth = Offset.get_sort().bv_size();
    for (unsigned K = 0; K < ByteWidth; ++K) {
        auto RealOffset = Z3::add(Offset, Z3::bv_val(K, OffsetBitWidth));
        auto RealValue = Z3::extract_byte(Val, K);

        uint64_t Const;
        if (Z3::is_numeral_u64(RealOffset, Const)) {
            at(Const)->set(RealValue);
        } else {
            VariableOffsetStore.insert(std::make_pair(RealOffset, new ScalarValue(1, RealValue)));
        }
        POPEYE_DEBUG(dbgs() << "store " << RealValue << " to " << RealOffset << "\n");
    }
}
