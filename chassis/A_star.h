
#ifndef A_START_H
#define A_START_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define MAP_X  3
#define MAP_Y  3
#define NODES  (MAP_X * MAP_Y)  // 9个节点

/* ===== 变量定义在 A_star.c,这里只做声明 =====
 * (原来定义在头文件里: 无 static 的会重复定义,
 *  带 static 的会让每个包含它的 .c 各有一份独立副本) */

extern uint8_t  edge_cost[NODES][NODES];
extern uint16_t obstacle;        // 障碍物位图,bit n 对应节点 n

typedef struct{
      
    uint16_t g;                 //  起点->当前节点的实际代价
    uint16_t f;                 //  f = g + h (总代价)
    int8_t   parent;            //  父节点索引,-1表示没有
    uint8_t  in_open;           //  1:在待探索的列表
    uint8_t  in_close;             //  1:已处理过的标志

} Node;



/* ===== 函数声明 ===== */
void    init_edges(void);
uint8_t astar(uint8_t start, uint8_t goal, uint8_t *path, uint8_t max_len);

#endif
