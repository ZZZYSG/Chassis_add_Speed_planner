#include "A_star.h"

/* ==================== 全局变量定义(声明在 A_star.h) ==================== */
uint8_t  edge_cost[NODES][NODES]; // 边权,0 表示不通
uint16_t obstacle = 0;            // 障碍物位图,bit n 对应节点 n

/* ==================== A* 内部状态(仅本文件使用) ==================== */
static Node    nodes[NODES];      // 保存9个节点的信息
static uint8_t open_list[NODES];  // 用于存待探索节点的编号
static uint8_t open_cnt = 0;      // 待探索名单的数量

/*================ 边权操作 ================*/
void init_edges(void) {
    memset(edge_cost, 0, sizeof(edge_cost));
    // 横向边
    edge_cost[0][1] = edge_cost[1][0] = 10;
    edge_cost[1][2] = edge_cost[2][1] = 10;
    edge_cost[3][4] = edge_cost[4][3] = 10;
    edge_cost[4][5] = edge_cost[5][4] = 10;
    edge_cost[6][7] = edge_cost[7][6] = 10;
    edge_cost[7][8] = edge_cost[8][7] = 10;
    // 纵向边
    edge_cost[0][3] = edge_cost[3][0] = 10;
    edge_cost[1][4] = edge_cost[4][1] = 10;
    edge_cost[2][5] = edge_cost[5][2] = 10;
    edge_cost[3][6] = edge_cost[6][3] = 10;
    edge_cost[4][7] = edge_cost[7][4] = 10;
    edge_cost[5][8] = edge_cost[8][5] = 10;
}


/*================ 工具函数 ================*/
//计算曼哈顿距离 * 10 (用整数，避免浮点) 
static uint16_t h_cost(uint8_t a, uint8_t b ){

    uint8_t ax = a % MAP_X ,ay = a / MAP_Y;
    uint8_t bx = b % MAP_X ,by = b / MAP_Y;
    return (uint16_t)(abs(ax - bx) + abs(ay - by))*10;

}

//在待探索的节点里，找 f 最小的节点编号
static int8_t get_min_f(void){

    uint8_t best_i = 0;
    for(uint8_t i = 1; i < open_cnt; i++){

         if(nodes[open_list[i]].f < nodes[open_list[best_i]].f)
         best_i = i;
    
    }
    return (int8_t)open_list[best_i];

}

//从待探索列表中删除某个节点编号
static void open_remove(uint8_t idx){

    for(uint8_t i = 0 ; i < open_cnt; i++){

        if(open_list[i] == idx){
            open_list[i] = open_list[--open_cnt]; 
         }

    }

}

/*================ A* 核心 ================*/
// 返回路径长度(0表示没找到)，path[] 被填入从起点到终点的节点序列
uint8_t astar(uint8_t start, uint8_t goal, uint8_t*path, uint8_t max_len){

    //1. 清零，并初始化
    memset(nodes, 0, sizeof(nodes));
    for (uint8_t i = 0; i < NODES; i++)
    nodes[i].parent = -1;
    open_cnt = 0;

    //2. 起点入队
    nodes[start].g = 0;
    nodes[start].f = h_cost(start, goal);
    open_list[open_cnt++] = start;
    nodes[start].in_open = 1;

    //3. 主循环；上、下、左、右
    const int8_t dx[4] = {0, 0, -1, 1};
    const int8_t dy[4] = {-1, 1, 0, 0};

    //存在待探索的节点
    while(open_cnt){

        //取f最小节点作为当前节点
        int8_t cur = get_min_f();

        //取完 f 值最小的节点，从待探索列表中删除
        open_remove((uint8_t)cur);

        //标记节点状态
        nodes[(uint8_t)cur].in_open = 0;
        nodes[(uint8_t)cur].in_close = 1;

        //是否到终点(是的话回溯路径)
        if((uint8_t)cur == goal){
        
            uint8_t len = 0;
            int8_t  n = cur;
            //从终点->起点，算路径大小
            while(n >= 0){
               len++;
               n = nodes[(uint8_t)n].parent;
               
            }
            
            //路径比缓冲区长,防止写越界
            if (len > max_len) return 0;

            //从终点->起点，记录走过的路
            n = cur;
            for (uint8_t i = len; i > 0; i--){

                path[i - 1] = (uint8_t)n;
                n = nodes[(uint8_t)n].parent;

            }
        
        return len;}

        //没到终点，拓展邻居(受边界条件、障碍物条件、已探明条件限制)
        uint8_t cx = (uint8_t)cur % MAP_X;
        uint8_t cy = (uint8_t)cur / MAP_Y;
        
        for(uint8_t i = 0 ; i < 4; i++){
            int8_t nx = (int8_t)cx + dx[i];
            int8_t ny = (int8_t)cy + dy[i];
        
        if (nx < 0 || nx >= MAP_X || ny < 0 || ny >= MAP_Y) 
                    continue; //不存在于地图上的点
        uint8_t nb = (uint8_t)(ny * MAP_X + nx); // 节点下标 = 行 * 列数 + 列
        if (obstacle & (1u << nb))   // 位图: 1 左移 nb 位,生成只有第 nb 位为 1 的掩码
                    continue; // 是障碍物
        if (nodes[nb].in_close)            
                    continue; // 已处理过

        uint16_t step = edge_cost[cur][nb];
        if (step == 0) continue;              // 本来就不通，跳过
        uint16_t g_new = nodes[cur].g + step; // 动态代价
        if (open_cnt >= NODES) continue;  // 待探索列表已满,防御
            
        if (!nodes[nb].in_open) {
            // 新发现的节点
            nodes[nb].g = g_new;
            nodes[nb].f = g_new + h_cost(nb, goal);
            nodes[nb].parent = cur;
            open_list[open_cnt++] = nb;
            nodes[nb].in_open = 1;
        }
        else if (g_new < nodes[nb].g) {
            // 发现更短的路，更新
            nodes[nb].g = g_new;
            nodes[nb].f = g_new + h_cost(nb, goal);
            nodes[nb].parent = cur;
        }

    }
    }
    //没有找到路
    return 0;  
}

