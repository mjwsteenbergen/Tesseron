//
// Created by newnottakenname on 07.01.16.
//

#ifndef SUPERVISOR_IMAGEREADER_H
#define SUPERVISOR_IMAGEREADER_H

#include <stdlib.h>
#include <iostream>
#include "Tile.h"

using namespace std;

const int MOSAIC_SIZE = 4;

class ImageReader {
public:

    void ConvertToColor(int imageData[MOSAIC_SIZE][MOSAIC_SIZE], TileColor color[MOSAIC_SIZE][MOSAIC_SIZE]);
    void GetImage(std::string path, TileColor imageColors[MOSAIC_SIZE][MOSAIC_SIZE]);
    void ReadImage(std::string path, int imageData[MOSAIC_SIZE][MOSAIC_SIZE]);

};


#endif //SUPERVISOR_IMAGEREADER_H
