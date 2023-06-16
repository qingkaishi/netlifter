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

#include "Memory/HeapMemoryBlock.h"


HeapMemoryBlock::HeapMemoryBlock(Type *Ty, unsigned Num) : MemoryBlock(Ty, Num, MK_Heap),
                                                           HMS(nullptr) {
}

HeapMemoryBlock::~HeapMemoryBlock() {
    delete HMS;
}

HeapMemorySummary::HeapMemorySummary(uint64_t TypeBitWidth) : SummarizedLen(Z3::free_bool()),
                                                              SummarizedVal(Z3::free_bv(TypeBitWidth)) {

}

void HeapMemoryBlock::setSummarizedHeap(bool S) {
    if (S) {
        assert(Mem.size());
        HMS = new HeapMemorySummary(Mem.at(0)->bytewidth() * 8);
    } else {
        delete HMS;
        HMS = nullptr;
    }
}

bool HeapMemoryBlock::isSummarizedHeap() const {
    return HMS;
}

void HeapMemoryBlock::setSummarizedLength(const z3::expr &E) {
    assert(HMS);
    HMS->SummarizedLen = E;
}

z3::expr HeapMemoryBlock::getSummarizedLength() const {
    assert(HMS);
    return HMS->SummarizedLen;
}

void HeapMemoryBlock::setSummarizedValue(const z3::expr &E) {
    assert(HMS);
    HMS->SummarizedVal = E;
}

z3::expr HeapMemoryBlock::getSummarizedValue() const {
    assert(HMS);
    return HMS->SummarizedVal;
}


AbstractValue *HeapMemoryBlock::at(size_t Offset) {
    return MemoryBlock::at(Offset);
}

AbstractValue *HeapMemoryBlock::at(const z3::expr &Offset) {
    return MemoryBlock::at(Offset);
}

AbstractValue *HeapMemoryBlock::before(size_t Offset) {
    return MemoryBlock::before(Offset);
}

AbstractValue *HeapMemoryBlock::before(const z3::expr &Offset) {
    return MemoryBlock::before(Offset);
}
