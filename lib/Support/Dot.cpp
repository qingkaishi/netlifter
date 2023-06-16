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

#include <llvm/Support/raw_os_ostream.h>
#include "Support/Dot.h"

#ifdef HAVE_GRAPHVIZ

#include "Support/AutoClose.h"
#include "Support/Debug.h"
#include "graphviz/gvc.h"
#include "graphviz/cgraph.h"

bool Dot::toImage(StringRef FileName) {
    FILE *FP = fopen(FileName.str().c_str(), "r+");
    AUTO_CLOSE(FILE, FP, fclose);
    if (!FP) {
        errs() << "[Error] fopen cannot open the file <" << FileName << "> for reading.\n";
        return false;
    }
    Agraph_t *G = agread(FP, 0);
    AUTO_CLOSE(Agraph_t, G,  agclose);
    if (!G) {
        errs() << "[Error] agread cannot open the file <" << FileName << "> for reading.\n";
        return false;
    }

    // image name
    if (FileName.endswith(".dot")) {
        FileName = FileName.substr(0, FileName.size() - 4);
    }
    auto ImgFileNameStr = FileName.str();
    ImgFileNameStr = ImgFileNameStr + ".png";

    // render image
    GVC_t *GVC = gvContext();
    AUTO_CLOSE(GVC_t, GVC, gvFreeContext);
    if (gvLayout(GVC, G, "dot") != 0) {
        return false;
    }
    if (gvRenderFilename(GVC, G, "png", ImgFileNameStr.c_str()) != 0) {
        gvFreeLayout(GVC, G);
        return false;
    }

    POPEYE_INFO(ImgFileNameStr << " is created!");
    // release resources
    gvFreeLayout(GVC, G);
    return true;
}

#else
bool Dot::toImage(StringRef) {
    return false;
}
#endif
