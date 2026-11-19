/* ======================================================================
 * >> 状态枚举（game.h）
 * 【面试问答】
 * Q: 关卡/阶段枚举这样设计的好处？
 * A: 用具名枚举代替魔法数字，关卡与阶段解耦，switch 分支语义清晰；新增关卡或阶段
 * A: 只需追加枚举值。
 * ----------------------------------------------------------------------
 * ====================================================================== */
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