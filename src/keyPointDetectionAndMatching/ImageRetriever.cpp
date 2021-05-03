//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#include <iostream>

#include "keyPointDetectionAndMatching/ImageRetriever.h"

namespace gdr {

    void ImageRetriever::markAllPairsToBeCompared() {

        {
            std::unique_lock<std::mutex> lockBitset(mutexBitset);

            int numberOfImages = imagePairsToCompare.size();
            assert(numberOfImages > 0);

            for (int indexFromLess = 0; indexFromLess < numberOfImages; ++indexFromLess) {
                for (int indexToBigger = indexFromLess + 1; indexToBigger < numberOfImages; ++indexToBigger) {
                    assert(indexToBigger > indexFromLess);
                    assert(indexToBigger >= 0 && indexToBigger < imagePairsToCompare.size());

                    imagePairsToCompare[indexFromLess][indexToBigger] = true;
                }
            }
        }
    }

    ImageRetriever::ImageRetriever(int numberOfImages) :
            imagePairsToCompare(numberOfImages, boost::dynamic_bitset<>(numberOfImages)) {
        assert(imagePairsToCompare.size() == numberOfImages);

        for (int indexFromLess = 0; indexFromLess < numberOfImages; ++indexFromLess) {
            assert(imagePairsToCompare[indexFromLess].size() == numberOfImages);
        }
    }

    bool ImageRetriever::tryGetSimilarImagesPair(std::pair<int, int> &nextIndicesFromLessAndBigger) {
        std::unique_lock<std::mutex> lockBitset(mutexBitset);

        size_t notFoundIndex = boost::dynamic_bitset<>::npos;

        assert(indexFromLessPrev >= 0 && indexToBiggerPrev < getNumberOfImages());

        {
            if (indexToBiggerPrev == 0 && indexFromLessPrev == 0) {
                //find first [true] pair in all bitsets

                for (int indexToSearchFromLess = indexFromLessPrev;
                     indexToSearchFromLess < getNumberOfImages(); ++indexToSearchFromLess) {

                    int firstIndexToBigger = findFirst(imagePairsToCompare[indexToSearchFromLess]);

                    if (firstIndexToBigger <= indexToSearchFromLess) {
                        assert(false && "indexTo is always bigger than indexFrom");
                    }

                    if (firstIndexToBigger < getNumberOfImages()) {

                        //there is a [true] pair in this bitset
                        nextIndicesFromLessAndBigger = {indexToSearchFromLess, firstIndexToBigger};

                        markPairComparedAndSetAsLastReturned(nextIndicesFromLessAndBigger.first,
                                                             nextIndicesFromLessAndBigger.second);

                        assert(indexFromLessPrev == nextIndicesFromLessAndBigger.first);
                        assert(indexToBiggerPrev == nextIndicesFromLessAndBigger.second);

                        return true;
                    }
                }

                nextIndicesFromLessAndBigger = {0, 0};

                return false;
            } else {

                {
                    int firstIndexToBigger = findNext(imagePairsToCompare[indexFromLessPrev], indexToBiggerPrev);

                    assert(firstIndexToBigger > indexToBiggerPrev);
                    assert(firstIndexToBigger > indexFromLessPrev);

                    if (firstIndexToBigger < getNumberOfImages()) {

                        nextIndicesFromLessAndBigger = {indexFromLessPrev, firstIndexToBigger};

                        markPairComparedAndSetAsLastReturned(nextIndicesFromLessAndBigger.first,
                                                             nextIndicesFromLessAndBigger.second);
                        assert(indexFromLessPrev == nextIndicesFromLessAndBigger.first);
                        assert(indexToBiggerPrev == nextIndicesFromLessAndBigger.second);

                        return true;
                    }
                }

                //not found pair in last bitset -- search in next ones
                for (int indexToSearchFromLess = indexFromLessPrev + 1;
                     indexToSearchFromLess < getNumberOfImages(); ++indexToSearchFromLess) {

                    int firstIndexToBigger = findFirst(imagePairsToCompare[indexToSearchFromLess]);

                    assert(firstIndexToBigger > indexFromLessPrev + 1);

                    if (firstIndexToBigger < getNumberOfImages()) {
                        nextIndicesFromLessAndBigger = {indexToSearchFromLess, firstIndexToBigger};

                        markPairComparedAndSetAsLastReturned(nextIndicesFromLessAndBigger.first,
                                                             nextIndicesFromLessAndBigger.second);
                        assert(indexFromLessPrev == nextIndicesFromLessAndBigger.first);
                        assert(indexToBiggerPrev == nextIndicesFromLessAndBigger.second);

                        return true;
                    }
                }

                //not found any pairs -- return false
                nextIndicesFromLessAndBigger = {0, 0};

                return false;
            }
        }

        assert(false && "unreachable code");
    }

    int ImageRetriever::getNumberOfImages() const {
        return imagePairsToCompare.size();
    }

    void ImageRetriever::markPairComparedAndSetAsLastReturned(int indexFromLess, int indexToBigger) {
        assert(indexFromLess < indexToBigger);
        assert(indexFromLess >= 0 && indexToBigger < getNumberOfImages());

        indexFromLessPrev = indexFromLess;
        indexToBiggerPrev = indexToBigger;

        imagePairsToCompare[indexFromLess][indexToBigger] = false;
    }

    void ImageRetriever::setPairToBeCompared(int indexFromLess, int indexToBigger) {
        assert(indexFromLess < indexToBigger);
        assert(indexFromLess >= 0 && indexToBigger < getNumberOfImages());

        imagePairsToCompare[indexFromLess][indexToBigger] = true;
    }

    int ImageRetriever::findFirst(const boost::dynamic_bitset<> &bs) {
        size_t notFoundIndex = boost::dynamic_bitset<>::npos;

        size_t foundIndex = bs.find_first();

        if (foundIndex == notFoundIndex) {
            return bs.size();
        }

        return static_cast<int>(foundIndex);
    }

    int ImageRetriever::findNext(const boost::dynamic_bitset<> &bs,
                                 int index) {
        size_t notFoundIndex = boost::dynamic_bitset<>::npos;

        assert(index >= 0 && index < bs.size());

        size_t foundIndex = bs.find_next(index);

        if (foundIndex == notFoundIndex) {
            return bs.size();
        }

        return static_cast<int>(foundIndex);
    }
}