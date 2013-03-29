/********************************
** VoiceCommandTypes.h
*********************************/
#ifndef VOICE_COMMAND_TYPE_H
#define VOICE_COMMAND_TYPE_H

struct Point {
	float x;
	float y;
};

enum CommandCode {
	RETURN,
	MOVE,
	TURN,
	STOP,
	GOTO
};

#define CURRENT_MAP 0 // this is the bwi test world map

#if CURRENT_MAP == 0
#define NUM_MAP_POINTS 3 // three map points for this map
enum LocationCode {
	BOOKCASE,
	WALL,
	HALL
};

char* LocationStrings[NUM_MAP_POINTS] = {
        "bookcase",
        "wall",
        "hallway"
};

Point LocationPoints[NUM_MAP_POINTS] = {
       {-1.5, -1.0},
       {2.0, -1.0},
       {0.0, 5.0}
};


#endif


#endif
