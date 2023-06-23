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

#include <llvm/Support/CommandLine.h>
#include <llvm/Support/FileSystem.h>
#include "Support/VSpell.h"

using namespace llvm;

static cl::opt<std::string> VSpellFileName("vspell-file-name",
                                           cl::desc("the file where the hooking function locates"),
                                           cl::init(""), cl::ReallyHidden, cl::ValueRequired);

static cl::opt<std::string> VSpellFuncName("vspell-function-name",
                                           cl::desc("the name of the hooking function"),
                                           cl::init(""), cl::ReallyHidden, cl::ValueRequired);

static cl::opt<unsigned> VSpellStartLine("vspell-start-line",
                                         cl::desc("the start line number of the hooking function"),
                                         cl::init(0), cl::ReallyHidden, cl::ValueRequired);

static cl::opt<unsigned> VSpellEndLine("vspell-end-line",
                                       cl::desc("the end line number of the hooking function"),
                                       cl::init(0), cl::ReallyHidden, cl::ValueRequired);

struct VSpellConfigReader {
    void operator=(const std::string &ConfFile) const {
        ErrorOr<std::unique_ptr<MemoryBuffer>> FileBuffer = MemoryBuffer::getFile(ConfFile);
        if (std::error_code EC = FileBuffer.getError()) {
            errs() << "Error opening file: " << EC.message() << "\n";
            llvm_unreachable("Invalid config file!");
        }

        StringRef FileContents = FileBuffer.get()->getBuffer();
        SmallVector<StringRef, 16> Lines;
        FileContents.split(Lines, '\n');
        if (Lines.size() < 4) {
            llvm_unreachable("Incomplete config file!");
        }

        auto FileName = Lines[0];
        VSpellFileName.setValue(FileName.str());

        auto FunctionName = Lines[1];
        VSpellFuncName.setValue(FunctionName.str());

        auto StartLine = Lines[2];
        VSpellStartLine.setValue(std::atoi(StartLine.str().c_str()));

        auto EndLine = Lines[3];
        VSpellEndLine.setValue(std::atoi(EndLine.str().c_str()));
    }
};

static VSpellConfigReader VCR;

static cl::opt<VSpellConfigReader, true, cl::parser<std::string>>
        VSpellConf("vspell-configure", cl::desc("vspell configure file"),
                   cl::ReallyHidden, cl::location(VCR), cl::ValueRequired);

bool VSpell::enabled() {
    bool Enabled = false;
    if (VSpellConf.getNumOccurrences()) Enabled = true;
    else if (VSpellFileName.getNumOccurrences() && VSpellFuncName.getNumOccurrences()
             && VSpellStartLine.getNumOccurrences() && VSpellEndLine.getNumOccurrences())
        Enabled = true;

    if (Enabled) {
        assert(!VSpellFileName.empty() && "file name cannot be empty");
        assert(!VSpellFuncName.empty() && "func name cannot be empty");
        assert(VSpellStartLine != 0 && "line number cannot be zero");
        assert(VSpellEndLine != 0 && "line number cannot be zero");
        assert(VSpellEndLine >= VSpellStartLine && "end line must >= start line");
    }

    return Enabled;
}

std::string VSpell::file() {
    std::string FileName = VSpellFileName.getValue();
    auto Pos = FileName.find_last_of('/');
    if (Pos != std::string::npos) {
        FileName = FileName.substr(Pos + 1);
    }
    Pos = FileName.find_last_of('\\');
    if (Pos != std::string::npos) {
        FileName = FileName.substr(Pos + 1);
    }
    return FileName;
}

std::string VSpell::function() {
    return VSpellFuncName.getValue();
}

unsigned VSpell::startLine() {
    return VSpellStartLine.getValue();
}

unsigned VSpell::endLine() {
    return VSpellEndLine.getValue();
}
