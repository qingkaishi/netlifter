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

#include <llvm/Support/CommandLine.h>
#include <map>
#include <set>
#include <vector>
#include "BNF/BNF.h"
#include "Support/ADT.h"
#include "Support/Debug.h"
#include "Support/VSpell.h"

#define DEBUG_TYPE "BNF"

/// for regression testing
/// @{
static cl::opt<unsigned> OraclePNum("popeye-oracle-production-num",
                                    cl::desc("test if the resulting #productions equals the given number"),
                                    cl::init(0));

static cl::opt<unsigned> OracleANum("popeye-oracle-assertion-num",
                                    cl::desc("test if the resulting #assertions equals the given number"),
                                    cl::init(0));
/// @}

RHSItem::~RHSItem() = default;

Production::Production(const z3::expr &Expr, BNF *NF) : RHSItem(RK_Production), Assertions(Z3::vec()) {
    static unsigned LHSCounter = 0;
    LHS = LHSCounter++;
    LHS4Print = LHS;

    if (Expr.is_or()) {
        z3::expr_vector OrOps = Z3::find_consecutive_ops(Expr, Z3_OP_OR);
        unsigned NA = OrOps.size();
        for (int I = 0; I < NA; ++I) {
            auto *Prod = new Production(OrOps[I], NF);
            NF->add(Prod);
            RHS.emplace_back();
            RHS.back().emplace_back(Prod);
        }
    } else {
        std::vector<BoundVecRef> Indices;
        {
            z3::expr_vector AndOps = Z3::find_consecutive_ops(Expr, Z3_OP_AND);
            for (auto Op: AndOps) {
                if (!Z3::find(Op, [](const z3::expr &E) { return E.decl().decl_kind() == Z3_OP_SELECT; })) {
                    continue;
                }
                Indices.emplace_back(BoundVec::createBoundVecRef(Op));
            }
            std::sort(Indices.begin(), Indices.end(), [](const BoundVecRef &A, const BoundVecRef &B) {
                // should be always comparable, and if there are multiple min,
                // the comparative result should be always the same
                return *A->MinVec.begin() < *B->MinVec.begin();
            });

            POPEYE_DEBUG(
                    for (auto &Vec: Indices) {
                        for (auto Ex: *Vec) {
                            dbgs() << *Ex << ", ";
                        }
                        dbgs() << "\n--------\n";
                    }
            );
        }
        if (Indices.empty()) return; // a constraint not containing any B[*]

        RHS.emplace_back();
        auto &ConjunctiveVec = RHS.back();
        int PrevIndicesIt = -1;
        int IndicesIt = 0;
        while (IndicesIt != Indices.size()) {
            if (PrevIndicesIt != -1) {
                // we need to check if comparable,
                // if nothing comparable,
                // it means that there is some violation of path conditions,
                // thus, just drop CurrVec.
                auto &CurrVec = Indices[IndicesIt];
                auto &PrevVec = Indices[PrevIndicesIt];
                if (!ADT::exists(CurrVec->MinVec, PrevVec->MaxVec,
                                 [](const BoundRef &A, const BoundRef B) {
                                     try {
                                         (void) (A < B); // check if comparable
                                         return true;
                                     } catch (const std::runtime_error &Err) {
                                         return false;
                                     }
                                 })) {
                    IndicesIt++;
                    continue;
                }
            }
            PrevIndicesIt = IndicesIt;

            auto CurrVecFrom = IndicesIt;
            auto CurrVecTo = CurrVecFrom;
            auto &CurrVec = Indices[IndicesIt++];
            if (IndicesIt != Indices.size()) {
                auto *NextVec = &Indices[IndicesIt];
                while (ADT::exists((*NextVec)->MinVec, CurrVec->MaxVec,
                                   [](const BoundRef &A, const BoundRef B) {
                                       try {
                                           return A <= B;
                                       } catch (const std::runtime_error &Err) {
                                           return false;
                                       }
                                   })) {
                    CurrVec->merge(*NextVec);
                    IndicesIt++;
                    CurrVecTo++;
                    if (IndicesIt != Indices.size()) {
                        NextVec = &Indices[IndicesIt];
                    } else {
                        break;
                    }
                }
            }

            auto WholeExpr = CurrVec->Op;
            if (WholeExpr.is_or()) {
                auto *Prod = new Production(WholeExpr, NF);
                NF->add(Prod);
                ConjunctiveVec.push_back(Prod);
            } else {
                std::sort(CurrVec->begin(), CurrVec->end(), [](const BoundRef &A, const BoundRef &B) {
                    // should always comparable
                    return A < B;
                });
                for (int K = 0; K < CurrVec->size(); ++K) {
                    auto Index = (*CurrVec)[K];
                    if (K > 0) {
                        auto Prev = (*CurrVec)[K - 1];
                        if (Prev == Index) {
                            continue;
                        } else if (Prev + 1 != Index) {
                            // padding middle
                            ConjunctiveVec.push_back(new Interval(Prev + 1, Index - 1));
                        }
                    }
                    ConjunctiveVec.push_back(new Interval(Index, Index));
                }
                Assertions.push_back(WholeExpr);
            }
        }
    }
}

BNF::BNF(const z3::expr &Expr) {
    if (Expr.is_false() || Expr.is_true() || Z3::is_free(Expr))
        return;

    add(new Production(Expr, this));
    pad();

    for (auto *P: this->Productions)
        if (P->RHS.empty()) P->setEpsilon();

    simplify();
}

BNF::~BNF() {
    for (auto *P: Productions) P->releaseMemory();
    for (auto *P: FalseProductions) P->releaseMemory();
    for (auto *P: Productions) delete P;
    for (auto *P: FalseProductions) delete P;
}

raw_ostream &operator<<(llvm::raw_ostream &O, const Production &P) {
    z3::expr_vector ExtraCond = Z3::vec();
    z3::expr_vector BeforeSubstitution = Z3::vec();
    z3::expr_vector AfterSubstitution = Z3::vec();

    POPEYE_DEBUG(O << "(" << P.LHS << ") ");
    if (P.LHS4Print) O << "L" << P.LHS4Print << " := ";
    else O << "S := ";
    if (P.isEpsilon()) {
        assert(P.RHS.empty());
        O << "epsilon;";
    }
    for (auto &Conjunction: P.RHS) {
        BoundRef Last;
        for (unsigned J = 0; J < Conjunction.size(); ++J) {
            auto *Item = Conjunction[J];
            if (auto *PItem = dyn_cast<Production>(Item)) {
                O << "L" << PItem->getLHS();
                Last.reset();
            } else if (auto *IItem = dyn_cast<Interval>(Item)) {
                auto From = IItem->getFrom();
                auto To = IItem->getTo();
                if (Last) {
                    auto NewIndexVar = Z3::index_var();
                    ExtraCond.push_back(NewIndexVar == Z3::add(Last->expr(), 1));
                    Last = Bound::createBound(NewIndexVar);
                    BeforeSubstitution.push_back(From->expr());
                    AfterSubstitution.push_back(Last->expr());
                } else if (isa<SymbolicBound>(From.get())) {
                    auto NewIndexVar = Z3::index_var();
                    ExtraCond.push_back(NewIndexVar == From->expr());
                    Last = Bound::createBound(NewIndexVar);
                    BeforeSubstitution.push_back(From->expr());
                    AfterSubstitution.push_back(Last->expr());
                } else {
                    Last.reset();
                }
                O << "B[" << (Last ? Last : From);
                if (From == To) {
                    O << "]";
                } else {
                    if (isa<SymbolicBound>(To.get())) {
                        auto NewIndexVar = Z3::index_var();
                        ExtraCond.push_back(NewIndexVar == To->expr());
                        Last = Bound::createBound(NewIndexVar);
                        BeforeSubstitution.push_back(To->expr());
                        AfterSubstitution.push_back(Last->expr());
                    } else {
                        Last.reset();
                    }
                    O << ".." << (Last ? Last : To) << "]";
                }
            } else {
                llvm_unreachable("Error : unknown rhs type!");
            }

            if (J != Conjunction.size() - 1) {
                O << " ";
            }
        }
        if (&Conjunction == &P.RHS.back()) {
            O << ";";
        } else {
            O << " | ";
        }
    }
    if (!P.Assertions.empty() || !ExtraCond.empty()) {
        for (auto Assert: P.Assertions) {
            z3::expr_vector ConjOps = Z3::find_consecutive_ops(Assert, Z3_OP_AND);
            for (auto ConjOp: ConjOps) {
                if (!ConjOp.is_true())
                    O << "\n     assert(" << ConjOp.substitute(BeforeSubstitution, AfterSubstitution) << ")";
            }
        }
        for (auto Assert: ExtraCond) {
            O << "\n     assert(" << Assert << ")";
        }
    } else {
        O << "\n     assert(true)";
    }
    return O;
}

bool Production::nonTerminalDisjunction() const {
    bool Ret = true;
    for (auto &OneRHS: RHS) {
        assert(!OneRHS.empty());
        if (OneRHS.size() != 1 || !isa<Production>(OneRHS[0])) {
            Ret = false;
            break;
        }
    }
    return !isEpsilon() && Ret;
}

bool Production::containsNonTerminal() const {
    for (auto &OneRHS: RHS) {
        for (auto *Item: OneRHS) {
            if (isa<Production>(Item))
                return true;
        }
    }
    return false;
}

raw_ostream &operator<<(llvm::raw_ostream &O, const BNF &B) {
    O << "\n<BNF>\n";
    bool First = true;
    unsigned NumAssertions = 0;
    unsigned NumProductions = 0;
    for (auto *P: B.Productions) {
        if (P->nonTerminalDisjunction()) continue;

        if (First) {
            O << "    " << *P << "\n";
            First = false;
        } else {
            O << "\n    " << *P << "\n";
        }
        NumAssertions += P->getNumAssertions();
        NumProductions += 1;
    }
    if (!NumProductions) {
        O << "    " << "S := epsilon;\n";
        NumProductions++;
    }
    if (VSpell::enabled()) {
        O << "\n    ";
        O << "INIT := "
          << "C[" << VSpell::file() << ":1:" << VSpell::startLine() - 1 << "] "
          << "C[S] "
          << "C[" << VSpell::file() << ":" << VSpell::endLine() + 1 << ":eof]";
        O << "\n";
        NumProductions++;
    }
    O << "</BNF>\n";
    O << "\n# Productions: " << NumProductions << ".";
    O << "\n# Assertions: " << NumAssertions << ".";
    O << "\nAssertions/Production: " << NumAssertions / NumProductions << ".\n";

    if (OraclePNum.getNumOccurrences()) {
        if (OraclePNum.getValue() != NumProductions)
            O << "Test failed (p): " << OraclePNum.getValue() << " ≠ " << NumProductions << ". ";
        else
            O << "Test passed (p): " << OraclePNum.getValue() << " = " << NumProductions << ". ";
    }
    if (OracleANum.getNumOccurrences()) {
        if (OracleANum.getValue() != NumAssertions)
            O << "Test failed (a): " << OracleANum.getValue() << " ≠ " << NumAssertions << ".";
        else
            O << "Test passed (a): " << OracleANum.getValue() << " = " << NumAssertions << ".";
    }
    return O;
}

static void findMax(RHSItem *RHS, std::vector<BoundRef> &MaxVec) {
    if (auto *Prod = dyn_cast_or_null<Production>(RHS)) {
        for (auto It = Prod->rhs_begin(), E = Prod->rhs_end(); It != E; ++It) {
            findMax(It->back(), MaxVec);
        }
    } else if (auto *Int = dyn_cast_or_null<Interval>(RHS)) {
        auto CurrBoundRef = Int->getTo();
        if (MaxVec.empty()) {
            MaxVec.push_back(CurrBoundRef);
        } else {
            bool NeedAdd = false;
            for (unsigned I = 0; I < MaxVec.size(); ++I) {
                auto PossibleMax = MaxVec[I];
                if (CurrBoundRef > PossibleMax) { // must always comparable
                    NeedAdd = true;
                    MaxVec[I] = MaxVec.back();
                    MaxVec.pop_back();
                    --I;
                }
            }
            if (NeedAdd) MaxVec.push_back(CurrBoundRef);
        }
    } else {
        llvm_unreachable("Error: unknown RHS type!");
    }
}

void BNF::pad() {
    auto FirstProd = *Productions.rbegin();
    padHead(FirstProd, Bound::createBound(0));
}

void BNF::padHead(Production *P, BoundRef Start) {
    for (auto &RHS: P->RHS) {
        if (RHS.empty()) continue;
        padHead(RHS[0], Start);

        for (unsigned K = 1; K < RHS.size(); ++K) {
            auto *PrevRHS = RHS[K - 1];
            auto *NextRHS = RHS[K];
            // if we have 2nd rhs, we need to pad the tail of the previous rhs
            // and determine the start boundref of the 2nd rhs
            std::vector<BoundRef> PrevMax;
            findMax(PrevRHS, PrevMax);
            assert(PrevMax.size() == 1);
            padTail(PrevRHS, PrevMax[0]);

            // pad the next
            padHead(NextRHS, PrevMax[0] + 1);
        }
    }
}

void BNF::padHead(RHSItem *RHS, BoundRef Start) {
    if (auto *Prod = dyn_cast_or_null<Production>(RHS)) {
        padHead(Prod, Start);
    } else if (auto *Int = dyn_cast_or_null<Interval>(RHS)) {
        Int->setFrom(Start);
    } else {
        llvm_unreachable("Error: unknown RHS type!");
    }
}

void BNF::padTail(RHSItem *RHS, BoundRef End) {
    if (auto *Prod = dyn_cast_or_null<Production>(RHS)) {
        for (auto It = Prod->rhs_begin(), E = Prod->rhs_end(); It != E; ++It) {
            padTail(It->back(), End);
        }
    } else if (auto *Int = dyn_cast_or_null<Interval>(RHS)) {
        Int->setTo(End);
    } else {
        llvm_unreachable("Error: unknown RHS type!");
    }
}


void BNF::dump(StringRef FileName) {
    if (FileName != "-") {
        std::error_code EC;
        raw_fd_ostream PStream(FileName.str(), EC, sys::fs::F_None);
        if (PStream.has_error()) {
            errs() << "[Error] Cannot open the file <" << FileName << "> for writing.\n";
            return;
        }
        PStream << *this << "\n";
        POPEYE_INFO(FileName << " dumped!");
    }
    outs() << *this << "\n";
}