/********************************
** VoiceCommandTypes.h
*********************************/
#ifndef VOICE_COMMAND_TYPE_H
#define VOICE_COMMAND_TYPE_H

enum CommandCode {
	RETURN,
	MOVE,
	TURN,
	STOP,
	GOTO
};

#define CURRENT_MAP "bwi_test_world"

enum LocationCode {
	BOOKCASE,
	DOOR,
	HALL
};

struct Point {
	float x;
	float y;
};



#endif
