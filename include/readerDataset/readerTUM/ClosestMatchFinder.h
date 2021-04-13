//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_CLOSESTMATCHFINDER_H
#define GDR_CLOSESTMATCHFINDER_H

#include <map>

namespace gdr {
    class ClosestMatchFinder {
    public:
        template<class K, class V>
        static
        typename std::map<K, V>::iterator findClosestKeyMatch(std::map<K, V> &collection,
                                                              K keyToFind) {
            if (collection.empty()) {
                return collection.end();
            }
            auto lowerBoundIterator = collection.lower_bound(keyToFind);

            if (lowerBoundIterator == collection.begin()) {
                return lowerBoundIterator;
            }

            if (lowerBoundIterator == collection.end()) {
                return std::prev(lowerBoundIterator);
            }

            auto prevLowerBoundIterator = std::prev(lowerBoundIterator);

            K foundValue = lowerBoundIterator->first;
            K prevToFoundValue = prevLowerBoundIterator->first;

            if (std::abs(foundValue - keyToFind) < std::abs(prevToFoundValue - keyToFind)) {
                return lowerBoundIterator;
            } else {
                return prevLowerBoundIterator;
            }
        }
    };
}
#endif
