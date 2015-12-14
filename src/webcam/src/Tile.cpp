//
// Created by newnottakenname on 30.11.15.
//

#include <opencv2/core/types.hpp>
#include "Tile.h"


TileColor Tile::getTileColor() {
    return color;
}

void Tile::init(double size, double x, double y, TileColor col) {
    this->size = size;
    this->x = x;
    this->y = y;
    this->color = col;
}

double Tile::GetX() {
    return x;
}

double Tile::GetY() {
    return y;
}

double Tile::GetSize() {
    return size;
}

void Tile::init(const TileColor color, std::vector <cv::Point> point) {
    this->color = color;
    this->edges = point;
}
