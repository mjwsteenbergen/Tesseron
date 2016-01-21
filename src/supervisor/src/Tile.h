//
// Created by newnottakenname on 07.01.16.
//

#ifndef SUPERVISOR_TILE_H
#define SUPERVISOR_TILE_H

enum TileColor{ Black = 0, Blue = 1, Grey = 2, White = 3 };

class Tile {
public:
    static TileColor ConvertToColor(int i);
};


#endif //SUPERVISOR_TILE_H
