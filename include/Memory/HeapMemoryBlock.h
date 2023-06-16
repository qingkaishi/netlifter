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

#ifndef MEMORY_HEAPMEMORYBLOCK_H
#define MEMORY_HEAPMEMORYBLOCK_H

#include "Memory/MemoryBlock.h"

struct HeapMemorySummary {
    z3::expr SummarizedLen;
    z3::expr SummarizedVal;

    HeapMemorySummary(uint64_t);
};

class HeapMemoryBlock : public MemoryBlock {
private:
    /// only a heap memory may be of var length and summarized
    /// every element in this block should have the same bitwidth
    /// and the value is parameterized by an index variable Z3::k()
    HeapMemorySummary *HMS;

public:
    HeapMemoryBlock(Type *Ty, unsigned Num = 1);

    ~HeapMemoryBlock();

    void setSummarizedHeap(bool S);

    bool isSummarizedHeap() const override;

    void setSummarizedLength(const z3::expr &E);

    z3::expr getSummarizedLength() const;

    void setSummarizedValue(const z3::expr &E);

    z3::expr getSummarizedValue() const;

public:
    AbstractValue *at(size_t Offset) override;

    AbstractValue *at(const z3::expr &Offset) override;

    AbstractValue *before(size_t Offset) override;

    AbstractValue *before(const z3::expr &Offset) override;

public:
    static bool classof(const MemoryBlock *M) {
        return M->getKind() == MK_Heap;
    }
};

#endif //MEMORY_HEAPMEMORYBLOCK_H
