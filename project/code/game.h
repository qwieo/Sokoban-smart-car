

#ifndef __GAME_H__
#define __GAME_H__
extern float auto_target_y;
extern float auto_target_x;
extern float target_angle;

enum {
	Game1=1,
	Game2,
	Game3
}Game_level;

enum {
	Idle=0,
	Explore,
	Feedback,
	Upload,
	Start_push,
	Completed
}Game_phase;

#endif