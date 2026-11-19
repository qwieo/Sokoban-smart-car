#include "sokoban_workspace.h"
#include <string.h>

/* 共享工作区采用静态分配，裸机运行期间不申请动态内存。 */
__attribute__((section(".bss.OCRAM_CACHE")))
SokobanSharedWorkspace g_sokoban_workspace;

/**
 * @brief 清零所有关卡复用的共享静态工作区。
 *
 * @return 无。
 *
 * @note 第一关在 clear_sokoban_memory1() 中调用本函数释放其临时状态。
 */
void sokoban_workspace_clear(void)
{
    memset(&g_sokoban_workspace, 0, sizeof(g_sokoban_workspace));
}
