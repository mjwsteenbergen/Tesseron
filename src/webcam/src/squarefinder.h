//
// Created by newnottakenname on 18.11.15.
//

#ifndef WEBCAM_SQUAREFINDER_H
#define WEBCAM_SQUAREFINDER_H

#include <opencv2/core/core.hpp>
#include "Tile.h"

class squarefinder {

public:

    void findSquares(const cv::Mat &image, std::vector<std::vector<cv::Point> > &squares, const TileColor color);

    void drawSquares(cv::Mat &image, const std::vector<std::vector<cv::Point> > &squares);
};


#endif //WEBCAM_SQUAREFINDER_H
