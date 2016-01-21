//
// Created by newnottakenname on 07.01.16.
//

#include "ImageReader.h"
#include <string>
#include <iostream>
#include <fstream>
#include <stdlib.h>

using namespace std;



void ImageReader::GetImage(std::string path, TileColor imageColors[MOSAIC_SIZE][MOSAIC_SIZE])
{
    int imageData[MOSAIC_SIZE][MOSAIC_SIZE];
    ImageReader::ReadImage(path, imageData);


    ConvertToColor(imageData, imageColors);
}

void ImageReader::ReadImage(std::string path, int imageData[MOSAIC_SIZE][MOSAIC_SIZE])
{

    std::string line;
    ifstream myfile (path.c_str());


    if (myfile.is_open())
    {
        for(int i=0; i < MOSAIC_SIZE; i++)
        {
            getline (myfile,line);
            cout << line << "\n";
            for(int j=0; j < MOSAIC_SIZE; j++){
                char c = line.c_str()[j];
                imageData[i][j] = (int)c - (int)'0';
            }
        }
        myfile.close();
    }

    else cout << "Unable to open file";
    {

    }
}

void ImageReader::ConvertToColor(int imageData[MOSAIC_SIZE][MOSAIC_SIZE], TileColor imageColors[MOSAIC_SIZE][MOSAIC_SIZE]){
    for(int i = 0; i < MOSAIC_SIZE; i++)
    {
        for (int j = 0; j < MOSAIC_SIZE; ++j) {
            imageColors[i][j] = Tile::ConvertToColor(imageData[i][j]);
        }
    }
}


