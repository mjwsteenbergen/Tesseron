//
// Created by newnottakenname on 07.01.16.
//

#include "ImageReader.h"
#include <string>
#include <iostream>
#include <fstream>
#include <stdlib.h>

using namespace std;



void ImageReader::GetImage(std::string path, char imageColors[MOSAIC_SIZE][MOSAIC_SIZE])
{
    ImageReader::ReadImage(path, imageColors);
}

void ImageReader::ReadImage(std::string path, char imageData[MOSAIC_SIZE][MOSAIC_SIZE])
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
                if(c == 'A' || c == 'B' || c == 'C' ||c == 'D') {
                    imageData[i][j] = c;
                }
            }
        }
        myfile.close();
    }

    else cout << "Unable to open file";
    {

    }
}


