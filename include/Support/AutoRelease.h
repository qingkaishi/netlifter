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


#ifndef SUPPORT_AUTORELEASE_H
#define SUPPORT_AUTORELEASE_H

#include <functional>

template<class T>
class AutoRelease {
private:
    T *Resource;
    std::function<void(T *)> ReleaseFunction;

public:
    template<class RF>
    AutoRelease(T *R, RF F) : Resource(R) {
        ReleaseFunction = F;
    }

    ~AutoRelease() {
        ReleaseFunction(Resource);
    }
};

// type, pointer, destructor
#define AUTO_RELEASE(T, P, F) AutoRelease<T> Release##P(P, [](T *P) { if (P) F(P); })

#endif //SUPPORT_AUTORELEASE_H
