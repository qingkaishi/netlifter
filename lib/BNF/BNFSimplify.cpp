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

#include "BNF/BNF.h"
#include "Support/Debug.h"

#define DEBUG_TYPE "BNFSimplify"

void BNF::simplify() {
    // simplify productions
    simplifyByCheckingConflict(); // remove false productions
    simplifyByMergingProductions(); // merge some productions
    simplifyByRemovingUnusedProductions(); // merge may lead to some unused productions
    simplifyByMappingProductionNames(); // L1 := L2 | L3 => when printing, do not show L1, and replace all L2/L3 with L1

    // infer fields
    simplifyByMergingFieldName(); // name(B[1]) = a, name(B[2]) = a => name(B[1..2]) = a
    simplifyByMergingFieldConstraint(); // B[0] = 0 & B[1] = 0 => B[0..1] = 0 and B[0] = -1 & B[1] = -1 => B[0..1] = -1
    simplifyByMergingField(); // L1 := B[1] B[2] => L1 := B[1..2] if the two bytes belong to the same field.
}

static std::pair<bool, z3::expr> desimplifyNE(const z3::expr &Assert, int C, bool BigEndian) {
    assert(C == 0 || C == -1);
    if (Assert.is_not() && Assert.arg(0).is_and() && Assert.arg(0).num_args() > 1) {
        auto And = Assert.arg(0);

        bool AllGood = true; // all formulas are = 0 or = -1
        z3::expr Array = Z3::bool_val(true);
        z3::expr MinByteIndex = Z3::bool_val(true);
        z3::expr MaxByteIndex = Z3::bool_val(true);
        for (unsigned K = 0; K < And.num_args(); ++K) {
            auto ArgK = And.arg(K);
            bool Good = C ? Z3::is_byte_eq_minus_one(ArgK) : Z3::is_byte_eq_zero(ArgK);
            if (Good) {
                if (K == 0) {
                    Array = ArgK.arg(0).arg(0);
                    MinByteIndex = ArgK.arg(0).arg(1);
                    MaxByteIndex = MinByteIndex;
                } else {
                    auto ArgKIndex = ArgK.arg(0).arg(1);
                    auto PrevArgIndex = And.arg(K - 1).arg(0).arg(1);
                    if (Z3::same(ArgKIndex, Z3::add(PrevArgIndex, 1))) {
                        MaxByteIndex = ArgKIndex;
                    } else if (Z3::same(Z3::add(ArgKIndex, 1), PrevArgIndex)) {
                        MinByteIndex = ArgKIndex;
                    } else {
                        AllGood = false;
                        break;
                    }
                }
            } else {
                AllGood = false;
                break;
            }
        }
        if (AllGood) {
            std::vector<z3::expr> Vec;
            auto Index = MinByteIndex;
            int K = 0;
            while (true) {
                Vec.push_back(Z3::byte_array_element(Array, Index));
                if (Z3::same(Index, MaxByteIndex)) break;
                Index = Z3::add(Index, 1);
                K++;
                // possible infinite loop
                if (K > 100) throw std::runtime_error("fail to desimplifyNE");
            }
            if (!BigEndian)
                std::reverse(Vec.begin(), Vec.end());
            auto Ret = Z3::concat(Vec) != C;
            return std::make_pair(true, Ret); // success
        }
    }
    return std::make_pair(false, Z3::bool_val(false)); // fail
}

static bool desimplifyEQ(z3::expr_vector &Asserts, int C, bool BigEndian) {
    std::vector<z3::expr> And;
    for (auto E: Asserts) {
        bool Good = C ? Z3::is_byte_eq_minus_one(E) : Z3::is_byte_eq_zero(E);
        if (Good) And.push_back(E);
    }
    std::sort(And.begin(), And.end(), [](const z3::expr &A, const z3::expr &B) {
        return Z3::byte_array_element_index_less_than(A.arg(0).arg(1), B.arg(0).arg(1));
    });

    std::set<z3::expr, Z3::less_than> Merged;
    z3::expr Array = Z3::bool_val(true);
    z3::expr MinByteIndex = Z3::bool_val(true);
    z3::expr MaxByteIndex = Z3::bool_val(true);
    for (unsigned K = 0; K < And.size(); ++K) {
        auto ArgK = And.at(K);
        if (K == 0) {
            Array = ArgK.arg(0).arg(0);
            MinByteIndex = ArgK.arg(0).arg(1);
            MaxByteIndex = MinByteIndex;
            Merged.insert(ArgK);
        } else {
            auto ArgKIndex = ArgK.arg(0).arg(1);
            auto PrevArgIndex = And.at(K - 1).arg(0).arg(1);
            if (Z3::same(ArgKIndex, Z3::add(PrevArgIndex, 1))) {
                MaxByteIndex = ArgKIndex;
                Merged.insert(ArgK);
            } else if (Z3::same(Z3::add(ArgKIndex, 1), PrevArgIndex)) {
                MinByteIndex = ArgKIndex;
                Merged.insert(ArgK);
            } else {
                break;
            }
        }
    }
    if (!Z3::same(MinByteIndex, MaxByteIndex)) {
        std::vector<z3::expr> Vec;
        auto Index = MinByteIndex;
        int K = 0;
        while (true) {
            Vec.push_back(Z3::byte_array_element(Array, Index));
            if (Z3::same(Index, MaxByteIndex)) break;
            Index = Z3::add(Index, 1);
            K++;
            // possible infinite loop
            if (K > 100) throw std::runtime_error("fail to desimplifyEQ");
        }
        if (!BigEndian)
            std::reverse(Vec.begin(), Vec.end());
        auto Ret = Z3::concat(Vec) == C;

        // replaces those with Ret in Asserts
        for (K = 0; K < Asserts.size(); ++K) {
            if (Merged.count(Asserts[K])) {
                auto Back = Asserts[Asserts.size() - 1];
                Asserts.set(K, Back);
                Asserts.pop_back();
                --K;
            }
        }
        Asserts.push_back(Ret);
        return true; // success
    }

    return false; // fail
}

int BNF::getEndianness(Production *P) {
    // 0 - little, 1 - big, -1 unknown
    for (auto Assert: P->Assertions) {
        auto Concats = Z3::find_all(
                Assert,
                [](const z3::expr &E) { return E.decl().decl_kind() == Z3_OP_CONCAT; },
                [](const z3::expr &E) {
                    return E.decl().decl_kind() == Z3_OP_CONCAT || E.decl().decl_kind() == Z3_OP_SELECT;
                }
        );
        for (auto Concat: Concats) {
            auto FirstIndex = Z3::bool_val(true);
            auto SecondIndex = Z3::bool_val(true);
            for (unsigned K = 0; K < Concat.num_args(); ++K) {
                auto ArgK = Concat.arg(K);
                if (ArgK.decl().decl_kind() == Z3_OP_SELECT) {
                    if (FirstIndex.is_bool()) {
                        FirstIndex = ArgK.arg(1);
                    } else {
                        SecondIndex = ArgK.arg(1);
                        break;
                    }
                }
            }
            if (!FirstIndex.is_bool() && !SecondIndex.is_bool()) {
                if (Z3::same(Z3::add(FirstIndex, 1), SecondIndex)) {
                    return 1; // big
                } else if (Z3::same(Z3::add(FirstIndex, -1), SecondIndex)) {
                    return 0; // little
                }
            }
        }
    }
    return -1; // unknown
}

void BNF::simplifyByMergingFieldConstraint() {
    bool BigEndian = true;
    for (auto *P: Productions) {
        int Ret = getEndianness(P);
        if (Ret != -1) {
            BigEndian = (Ret == 1);
            break;
        }
    }
    POPEYE_DEBUG(dbgs() << "use " << (BigEndian ? "big" : "little") << " endianness\n");

    for (auto *P: Productions) {
        // NOT(B[0] = 0 & B[1] = 0 & ...) => NOT(B[0]B[1]... = 0)
        // NOT(B[0] = -1 & B[1] = -1 & ...) => NOT(B[0]B[1]... = -1)
        for (unsigned J = 0; J < P->Assertions.size(); ++J) {
            auto Assert = P->Assertions[J];
            auto DesimplifyResult = desimplifyNE(Assert, 0, BigEndian);
            if (DesimplifyResult.first) {
                P->Assertions.set(J, DesimplifyResult.second);
                continue;
            }
            DesimplifyResult = desimplifyNE(Assert, -1, BigEndian);
            if (DesimplifyResult.first) {
                P->Assertions.set(J, DesimplifyResult.second);
                continue;
            }
        }

        // B[0] = 0 & B[1] = 0 & ... => B[0]B[1].. = 0
        // B[0] = -1 & B[1] = -1 & ... => B[0]B[1].. = -1
        desimplifyEQ(P->Assertions, 0, BigEndian);
        desimplifyEQ(P->Assertions, -1, BigEndian);
    }
}

static void buildNewFields(BoundRef F, BoundRef T, std::list<Interval *> &Fields, std::vector<RHSItem *> &Ret) {
    POPEYE_DEBUG(dbgs() << "--------------------------------\n");
    POPEYE_DEBUG(dbgs() << "debugging buildNewFields: " << F << ".." << T << "\n");
    POPEYE_DEBUG(dbgs() << "--------------------------------\n");

    unsigned OrigSize = Ret.size();
    while (!Fields.empty()) {
        auto Field = Fields.front();
        Fields.pop_front();
        auto FieldF = Field->getFrom();
        auto FieldT = Field->getTo();
        delete Field; // safe to delete, because we only use FieldF/FieldT

        POPEYE_DEBUG(dbgs() << "visit field: " << FieldF << ".." << FieldT << " @ range " << F << " .. " << T << "\n");

        if (FieldF < F) {
            POPEYE_DEBUG(dbgs() << "\tadjust " << FieldF << " -> " << F << "\n");
            FieldF = F;
        }
        if (FieldT > T) {
            POPEYE_DEBUG(dbgs() << "\tadjust " << FieldT << " -> " << T << "\n");
            FieldT = T;
        }

        if (FieldF > F) {
            Ret.push_back(new Interval(F, Bound::createBound(-2)));
            Ret.push_back(new Interval(FieldF, FieldT));
            POPEYE_DEBUG(dbgs() << "\tadd field: " << F << ".." << -2 << "\n");
            POPEYE_DEBUG(dbgs() << "\tadd field: " << FieldF << ".." << FieldT << "\n");

            F = FieldT + 1;
            if (F > T) break;
        } else if (FieldF == F) {
            Ret.push_back(new Interval(FieldF, FieldT));
            POPEYE_DEBUG(dbgs() << "\tadd field: " << FieldF << ".." << FieldT << "\n");

            F = FieldT + 1;
            if (F > T) break;
        }
    }
    if (F < T) {
        Ret.push_back(new Interval(F, T));
        POPEYE_DEBUG(dbgs() << "\tadd a tail field: " << F << ".." << T << "\n");
    }
    POPEYE_DEBUG(dbgs() << "--------------------------------\n");
}

void BNF::simplifyByMergingField() {
    POPEYE_DEBUG(dbgs() << "--------------------------------\n");
    POPEYE_DEBUG(dbgs() << "debugging simplifyByMergingField\n");
    POPEYE_DEBUG(dbgs() << "--------------------------------\n");

    std::vector<Interval *> Fields; // a vector to collect fields in a production
    std::list<Interval *> UniqueFields; // a copy of Fields without duplicates
    for (auto *P: Productions) {
        // infer fields in constraints, and rearrange byte seq in the production
        Fields.clear(); // init
        UniqueFields.clear(); // init
        for (auto Assert: P->Assertions) {
            POPEYE_DEBUG(dbgs() << "checking assert " << Assert << "\n");
            if (Z3::is_naming_eq(Assert)) {
                auto FieldExpr = Assert.arg(0).arg(0);
                if (FieldExpr.decl().decl_kind() == Z3_OP_SELECT) {
                    // single-byte field
                    auto From = Bound::createBound(FieldExpr.arg(1));
                    auto To = From;
                    auto *Field = new Interval(From, To);
                    Fields.push_back(Field);
                    POPEYE_DEBUG(dbgs() << "\tfind field " << Field->getFrom() << ".." << Field->getTo() << "\n");
                } else if (Z3::is_byte_array_range(FieldExpr)) {
                    auto From = Bound::createBound(FieldExpr.arg(1));
                    auto To = Bound::createBound(FieldExpr.arg(2));
                    auto *Field = new Interval(From, To);
                    Fields.push_back(Field);
                    POPEYE_DEBUG(dbgs() << "\tfind field " << Field->getFrom() << ".." << Field->getTo() << "\n");
                } else {
                    // for strange protocol like quic...
                }
            } else {
                // find concat (multi-byte field)
                auto Concats = Z3::find_all(
                        Assert,
                        [](const z3::expr &E) { return E.decl().decl_kind() == Z3_OP_CONCAT; },
                        [](const z3::expr &E) {
                            return E.decl().decl_kind() == Z3_OP_CONCAT || E.decl().decl_kind() == Z3_OP_SELECT;
                        }
                );
                std::set<z3::expr, Z3::less_than> SelectInConcats;
                for (auto Concat: Concats) {
                    auto Selects = Z3::find_all(Concat, false,
                                                [](const z3::expr &E) { return E.decl().decl_kind() == Z3_OP_SELECT; });
                    if (Selects.size() <= 1) continue;
                    for (auto Select: Selects) SelectInConcats.insert(Select);
                    BoundVecRef BV = BoundVec::createBoundVecRef(Concat);
                    assert(BV->MinVec.size() == 1);
                    assert(BV->MaxVec.size() == 1);
                    auto *Field = new Interval(BV->MinVec[0], BV->MaxVec[0]);
                    Fields.push_back(Field);
                    POPEYE_DEBUG(dbgs() << "\tfind field " << Field->getFrom() << ".." << Field->getTo() << "\n");
                }

                // find select not in concat (single-byte field)
                auto Selects = Z3::find_all(Assert, false,
                                            [](const z3::expr &E) { return E.decl().decl_kind() == Z3_OP_SELECT; });
                for (auto Select: Selects) {
                    if (SelectInConcats.count(Select)) continue;
                    auto From = Bound::createBound(Select.arg(1));
                    auto To = From;
                    auto *Field = new Interval(From, To);
                    Fields.push_back(Field);
                    POPEYE_DEBUG(dbgs() << "\tfind field " << Field->getFrom() << ".." << Field->getTo() << "\n");
                }
            }
        }

        // check if fields are mutually exclusive, if not, skip the following procedure. we may improve this later.
        bool ConflictFields = false;
        try {
            std::sort(Fields.begin(), Fields.end(), [](const Interval *IA, const Interval *IB) {
                if (IA->getFrom() < IB->getFrom()) return true;
                if (IA->getFrom() == IB->getFrom()) return IA->getTo() < IB->getTo();
                return false;
            });
        } catch (const std::runtime_error &Err) {
            ConflictFields = true; // for strange protocol like quic ...
        }
        for (unsigned K = 0; K < Fields.size(); ++K) {
            auto *F = Fields[K];
            if (ConflictFields || UniqueFields.empty()) {
                UniqueFields.push_back(F);
            } else {
                auto Recent = UniqueFields.back();
                if (F->getFrom() != Recent->getFrom() || F->getTo() != Recent->getTo()) {
                    UniqueFields.push_back(F);
                }
            }
            if (UniqueFields.back() != F) delete F; // gc duplicates
        }
        Fields.clear();
        auto UFIt = UniqueFields.begin();
        auto PrevUFIt = UFIt;
        if (UFIt != UniqueFields.end()) UFIt++;
        while (UFIt != UniqueFields.end() && !ConflictFields) {
            auto Previous = *PrevUFIt;
            auto Current = *UFIt;
            if (Current->getFrom() <= Previous->getTo()) {
                ConflictFields = true;
                break;
            }
            PrevUFIt++;
            UFIt++;
        }
        POPEYE_DEBUG(
                dbgs() << "fields in a production: \n";
                for (auto *F: UniqueFields) {
                    dbgs() << "\t[field] " << F->getFrom() << ".." << F->getTo() << "\n";
                }
                dbgs() << "\t[conflict?] " << ConflictFields << "\n";
                dbgs() << "------------------------------\n";
        );

        // rearrange the production
        std::vector<RHSItem *> NewConjunction;
        for (auto &Conjunction: P->RHS) {
            if (Conjunction.empty()) continue;
            NewConjunction.clear();
            unsigned K = 0;
            while (true) {
                for (; K < Conjunction.size(); ++K) {
                    if (isa<Interval>(Conjunction[K])) break;
                    else NewConjunction.push_back(Conjunction[K]);
                }
                if (K == Conjunction.size()) break;
                unsigned J = K + 1;
                for (; J < Conjunction.size(); ++J)
                    if (isa<Production>(Conjunction[J])) break;
                // split intervals [K, J) by UniqueFields
                auto From = dyn_cast<Interval>(Conjunction[K])->getFrom();
                auto To = dyn_cast<Interval>(Conjunction[J - 1])->getTo();
                if (ConflictFields) NewConjunction.push_back(new Interval(From, To));
                else buildNewFields(From, To, UniqueFields, NewConjunction);
                if (J == Conjunction.size()) break;
                K = J;
            }

            // swap and gc
            Conjunction.swap(NewConjunction);
            for (auto *Item: NewConjunction)
                if (!isa<Production>(Item)) delete Item;
        }

        // gc UniqueFields
        for (auto *Field: UniqueFields) delete Field;
    }
}

void BNF::simplifyByMappingProductionNames() {
    for (auto *P: Productions) {
        if (P->nonTerminalDisjunction()) {
            for (auto &OneRHS: P->RHS) {
                assert(OneRHS.size() == 1);
                auto Prod = dyn_cast<Production>(OneRHS[0]);
                assert(Prod);
                Prod->LHS4Print = P->LHS4Print;
            }
        }
    }
}

void BNF::simplifyByMergingFieldName() {
    for (auto *P: Productions) {
        std::map<std::string, std::vector<z3::expr>> NameByteMap;
        for (auto E: P->Assertions) {
            if (Z3::is_naming_eq(E)) {
                auto NameStrExpr = E.arg(1);
                auto BytesExpr = E.arg(0).arg(0);
                auto ByteV = Z3::find_consecutive_ops(BytesExpr, Z3_OP_CONCAT, false);
                for (auto B: ByteV) {
                    if (B.decl().decl_kind() != Z3_OP_SELECT) continue;
                    NameByteMap[NameStrExpr.to_string()].push_back(B);
                }
            }
        }

        auto It = NameByteMap.begin();
        while (It != NameByteMap.end()) {
            if (It->second.size() <= 1) {
                It = NameByteMap.erase(It);
            } else {
                ++It;
            }
        }

        for (unsigned K = 0; K < P->Assertions.size(); ++K) {
            auto E = P->Assertions[K];
            if (Z3::is_naming_eq(E)) {
                auto NameStrExpr = E.arg(1);
                if (NameByteMap.count(NameStrExpr.to_string())) {
                    auto Back = P->Assertions.back();
                    P->Assertions.set(K, Back);
                    P->Assertions.pop_back();
                    --K;
                }
            }
        }

        for (auto &NIt: NameByteMap) {
            auto &Name = NIt.first;
            auto &Vec = NIt.second;
            std::sort(Vec.begin(), Vec.end(), [](const z3::expr &A, const z3::expr &B) {
                assert(A.decl().decl_kind() == Z3_OP_SELECT);
                assert(B.decl().decl_kind() == Z3_OP_SELECT);
                return Z3::byte_array_element_index_less_than(A.arg(1), B.arg(1));
            });
            auto Naming = Z3::naming(Z3::byte_array_range(Vec.front().arg(0), Vec.front().arg(1), Vec.back().arg(1)),
                                     Name.c_str());
            P->Assertions.push_back(Naming);
        }
    }
}

void BNF::simplifyByMergingProductions() {
    // L1 := L2 | L3;
    // L2 := L4 | L5
    // =>
    // L1 := L4 | L5 | L3
    for (auto *L1: Productions) {
        for (auto It = L1->RHS.begin(); It != L1->RHS.end();) {
            auto &OneRHS = *It;
            if (OneRHS.size() == 1 && isa<Production>(OneRHS[0])) {
                auto *L2 = (Production *) OneRHS[0];
                if (L2->nonTerminalDisjunction()) {
                    for (auto &X: L2->RHS) {
                        L1->RHS.emplace_back(std::move(X));
                    }

                    It = L1->RHS.erase(It); // remove RHSProd from Prod's RHS
                    assert(L1 != L2);
                    continue;
                }
            }
            ++It;
        }
    }

    // L2 := *****
    // L1 := L2
    // =>
    // L1 := *****
    for (auto *L1: Productions) {
        if (L1->RHS.size() == 1 && L1->RHS.back().size() == 1) {
            if (auto *L2 = dyn_cast<Production>(L1->RHS.back().back())) {
                if (L2->isEpsilon()) continue;
                L1->RHS = std::move(L2->RHS);
                L1->Assertions = L2->Assertions;
                assert(L2->RHS.empty());
            }
        }
    }
}

void BNF::simplifyByCheckingConflict() {
    for (auto It = Productions.begin(); It != Productions.end();) {
        auto *P = *It;
        if (P->Assertions.empty()) {
            ++It;
            continue;
        }
        // pre-processing assertions
        auto AndExprOfAssert = Z3::make_and(P->Assertions);
        P->Assertions.resize(0);
        if (AndExprOfAssert.is_and()) {
            for (unsigned I = 0; I < AndExprOfAssert.num_args(); ++I) {
                P->Assertions.push_back(AndExprOfAssert.arg(I));
            }
        } else {
            P->Assertions.push_back(AndExprOfAssert);
        }

        // remove useless products; a product with only terminals but with only distinct in its assertions
        // do this before call z3::simplify, which will over-simplify the formulae and make the heuristics not work
        if (!P->containsNonTerminal()) {
            bool AllNe = true;
            if (AndExprOfAssert.is_and())
                for (unsigned I = 0; I < AndExprOfAssert.num_args(); ++I) {
                    if (!Z3::is_distinct_or_not_eq(AndExprOfAssert.arg(I))) {
                        AllNe = false;
                        break;
                    }
                }
            else AllNe = Z3::is_distinct_or_not_eq(AndExprOfAssert);
            if (AllNe) {
                P->Assertions.resize(0);
                P->Assertions.push_back(Z3::bool_val(false));

                It = Productions.erase(It);
                FalseProductions.insert(P);
                continue;
            }
        }

        // compute simplified form, remove obviously infeasible product
        // and remove obviously unnecessary formulae
        auto SimplifiedAssert = Z3::simplify(P->Assertions);
        if (SimplifiedAssert.is_false()) {
            P->Assertions.resize(0);
            P->Assertions.push_back(SimplifiedAssert);

            It = Productions.erase(It);
            FalseProductions.insert(P);
            continue;
        } else {
            P->Assertions = Z3::find_consecutive_ops(SimplifiedAssert, Z3_OP_AND);
            for (int I = 0; I < P->Assertions.size(); ++I) {
                auto Assert = P->Assertions[I];
                if (Assert.decl().decl_kind() == Z3_OP_NOT) {
                    auto SimplifiedForm = Z3::negation(Assert.arg(0));
                    P->Assertions.set(I, SimplifiedForm);
                }
            }
        }
        ++It;
    }

    auto Revisit = [this](Production *P) {
        auto &RHS = P->RHS;
        bool InitiallyEmpty = RHS.empty();
        auto It = RHS.begin();
        while (It != RHS.end()) {
            auto &Conj = *It;
            bool False = false;
            for (auto *Item: Conj) {
                if (auto *PItem = dyn_cast<Production>(Item)) {
                    if (FalseProductions.count(PItem)) {
                        False = true;
                        break;
                    }
                }
            }
            if (False) {
                It = RHS.erase(It);
            } else {
                ++It;
            }
        }
        if (!InitiallyEmpty && RHS.empty())
            return true;
        return false;
    };

    // remove redundant products, because many have become false
    unsigned FalseProductOrgSize = FalseProductions.size();
    unsigned FalseProductNewSize = 0;
    while (FalseProductNewSize != FalseProductOrgSize) {
        FalseProductOrgSize = FalseProductions.size();
        for (auto It = Productions.begin(); It != Productions.end();) {
            auto *P = *It;
            if (Revisit(P)) {
                P->Assertions.resize(0);
                P->Assertions.push_back(Z3::bool_val(false));
                It = Productions.erase(It);
                FalseProductions.insert(P);
                continue;
            }
            ++It;
        }
        FalseProductNewSize = FalseProductions.size();
    }
}

void BNF::simplifyByRemovingUnusedProductions() {
    // find unused product and remove
    std::map<Production *, unsigned> RefCountMap;
    for (auto *P: Productions) {
        for (auto &RHS: P->RHS) {
            for (auto *Item: RHS) {
                if (auto *ProdItem = dyn_cast<Production>(Item)) {
                    auto It = RefCountMap.find(ProdItem);
                    if (It == RefCountMap.end()) {
                        RefCountMap[ProdItem] = 1;
                    } else {
                        It->second++;
                    }
                }
            }
        }
        if (P->getLHS() == 0) RefCountMap[P] = 1; // the start symbol
    }
    auto It = Productions.begin();
    while (It != Productions.end()) {
        auto *P = *It;
        auto RCIt = RefCountMap.find(P);
        if (RCIt != RefCountMap.end()) {
            assert(RCIt->second);
            It++;
        } else {
            for (auto &RHS: P->RHS) {
                for (auto *Item: RHS) {
                    if (auto *ProdItem = dyn_cast<Production>(Item)) {
                        auto ItemRCIt = RefCountMap.find(ProdItem);
                        assert(ItemRCIt != RefCountMap.end());
                        ItemRCIt->second--;
                        if (ItemRCIt->second == 0 && ProdItem != P) {
                            FalseProductions.insert(ProdItem);
                            Productions.erase(ProdItem);
                        }
                    }
                }
            }
            FalseProductions.insert(P);
            It = Productions.erase(It);
        }
    }
}
