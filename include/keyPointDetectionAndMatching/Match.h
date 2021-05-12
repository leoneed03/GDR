//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#ifndef GDR_MATCH_H
#define GDR_MATCH_H

namespace gdr {

    /** Contains indices of corresponding keypoints on images */
    struct Match {
    private:

        //kept for debug purposes
        int frameNumber;

        /** each pair contains keypoint index on "destination" image
         *      and index of the corresponding keypoint on the "to be transformed image
         */
        std::vector<std::pair<int, int>> matchNumbers;

    public:

        int getFrameNumber() const;

        int getSize() const;

        const std::pair<int, int> &getKeyPointIndexDestinationAndToBeTransformed(int matchPairIndex) const;

        Match(int newFrameNumber,
              std::vector<std::pair<int, int>> &&newMatchNumbers);
    };
}
#endif
