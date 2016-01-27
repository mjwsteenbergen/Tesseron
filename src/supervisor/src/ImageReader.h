//
// Created by newnottakenname on 07.01.16.
//

#ifndef SUPERVISOR_IMAGEREADER_H
#define SUPERVISOR_IMAGEREADER_H

#include <stdlib.h>
#include <iostream>

using namespace std;

const int MOSAIC_SIZE = 16;

class ImageReader {
public:

    void ConvertToColor(char imageData[MOSAIC_SIZE][MOSAIC_SIZE], char color[MOSAIC_SIZE][MOSAIC_SIZE]);
    void GetImage(std::string path, char imageColors[MOSAIC_SIZE][MOSAIC_SIZE]);
    void ReadImage(std::string path, char imageData[MOSAIC_SIZE][MOSAIC_SIZE]);

};


#endif //SUPERVISOR_IMAGEREADER_H
