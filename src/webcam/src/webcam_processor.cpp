#include "webcam_processor.hpp"


void ProcessImage(cv::Mat webcam_image, std::vector<Tile> &tiles)
{
    if (webcam_image.empty()) {
        return;
    }
    resize(webcam_image, webcam_image, Size(0, 0), 1, 1);

    vector<vector<Point> > squares;

    //Convert BGR to HSV
    Mat HSVmat;
    cvtColor(webcam_image, HSVmat, COLOR_BGR2HSV);

    //Threshold the HSV image to get only blue colors
    Mat greyMask;
    inRange(HSVmat, Scalar(12, 25, 30), Scalar(29, 120, 170), greyMask);
    Mat whiteMask;
    inRange(HSVmat, Scalar(0, 0, 220), Scalar(63, 50, 360), whiteMask);
    Mat blueMask;
    inRange(HSVmat, Scalar(5, 110, 111), Scalar(27, 256, 255), blueMask);
    Mat blackMask;
    inRange(HSVmat, Scalar(0, 0, 0), Scalar(256, 256, 100), blackMask);

    squarefinder finder;
    finder.findSquares(blueMask, squares, Blue);
    finder.findSquares(blackMask, squares, Black);
    finder.findSquares(whiteMask, squares, White);
    finder.findSquares(greyMask, squares, Grey);

    finder.drawSquares(webcam_image, squares);
    waitKey(30);

    getPositionOfSquares(tiles, squares);

    ROS_INFO("Processed Image");

    finder.drawSquares(webcam_image, squares);
    waitKey(30);

}

void getPositionOfSquares(std::vector<Tile> &tiles, std::vector<std::vector<Point> > &vector) {

    std::vector<std::vector<Point> > singleSquare;

    for (int i = 0; i < vector.size(); i++) {
        std::vector<Point> vic = vector[i];
        bool breaked = false;
        for (int j = 0; j < singleSquare.size(); j++) {
            std::vector<Point> vic2 = singleSquare[j];
            for (int k = 0; k < 4; k++) {
                if (abs(vic[k].x - vic2[k].x) < 30 && abs(vic[k].y - vic2[k].y) < 30) {
                    int x = abs(vic[k].x - vic2[k].x);
                    int y = abs(vic[k].y - vic2[k].y);
                    breaked = true;
                }
            }
            if (breaked) {
                break;
            }
        }
        if (breaked) {
            continue;
        }
        Point ld = vic[0];
        Point lu = vic[1];
        Point ru = vic[2];
        Point rd = vic[3];

        double west = sqrt(pow(lu.y - ld.y, 2) + pow(lu.x - ld.x, 2));
        double east = sqrt(pow(ru.y - rd.y, 2) + pow(ru.x - rd.x, 2));
        double north = sqrt(pow(lu.y - ru.y, 2) + pow(lu.x - ru.x, 2));
        double south = sqrt(pow(ld.y - rd.y, 2) + pow(ld.x - rd.x, 2));

        double average = (west + east + north + south) / 4;

        double temp = abs(east - west + north - south);
        if (temp > 20) {
            continue;
        }

        Tile tile;

        float yl = (ld.y + lu.y)/2;
        float yr = (rd.y + ru.y)/2;
        float y = (yl + yr)/2;

        float xr = (rd.x + ru.x)/2;
        float xl = (ld.x + lu.x)/2;
        float x = (xl + xr)/2;


        tile.init(average, x, y, Grey);
        tiles.push_back(tile);

        singleSquare.push_back(vic);

    }

    vector = singleSquare;
}

