//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_IMAGERETRIEVER_H
#define GDR_IMAGERETRIEVER_H

#include "boost/dynamic_bitset.hpp"
#include <mutex>

namespace gdr {
    class ImageRetriever {

        int indexFromLessPrev = 0;
        int indexToBiggerPrev = 0;

        std::mutex mutexBitset;
        std::vector<boost::dynamic_bitset<>> imagePairsToCompare;

        int findFirst(const boost::dynamic_bitset<> &bs);

        int findNext(const boost::dynamic_bitset<> &bs,
                      int index);
    public:

        void setPairToBeCompared(int indexFromLess, int indexToBigger);

        void markPairComparedAndSetAsLastReturned(int indexFromLess, int indexToBigger);

        int getNumberOfImages() const;

        void markAllPairsToBeCompared();

        ImageRetriever(int numberOfImages);

        bool tryGetSimilarImagesPair(std::pair<int, int> &nextIndicesFromLessAndBigger);
    };
}


#endif
