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


#ifndef MUSTAA_XGRAPH_H
#define MUSTAA_XGRAPH_H

#include <map>
#include <memory>
#include <set>

class XNode;

class XValue;

/// a graph containing one or more xnodes to represent the must alias relations
class XGraph {
private:
    std::set<XNode *> Nodes;

    std::map<XValue *, XNode *> ValNodeMap;

public:
    ~XGraph() {
        // todo delete xnode/xvalue
    }

    XNode *getXNode(XValue *V);
};

typedef std::shared_ptr<XGraph> XGraphRef;

#endif //MUSTAA_XGRAPH_H
