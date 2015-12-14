//
// Created by newnottakenname on 30.11.15.
//

#ifndef WEBCAM_SQUARE_H
#define WEBCAM_SQUARE_H

enum TileColor{ Blue, Grey, White, Black};

class Tile {
protected:
    double x;
    double y;
    double size;
    std::vector <cv::Point> edges;
    TileColor color;
public:
    double GetX();
    double GetY();
    double GetSize();

    void init(double size, double x, double y, TileColor col);

    TileColor getTileColor();

    void init(const TileColor color, std::vector <cv::Point>);
};



#endif //WEBCAM_SQUARE_H
