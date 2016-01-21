//
// Created by newnottakenname on 07.01.16.
//

#include <stdlib.h>
#include "string"
#include "Tile.h"

TileColor Tile::ConvertToColor(int i){
    switch(i){
        case 0:
            return Black;
        case 1:
            return Blue;
        case 2:
            return Grey;
        case 3:
            return White;
        default:
            std::string exc= "Yeah. No. That is not a color we have for you";
            throw std::bad_exception();
    }
}