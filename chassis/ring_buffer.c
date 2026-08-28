#include "ring_buffer.h"


/*空和满的判断
空：head == tail;  满 (head + 1) % size == tail ; 可入队 (head + 1) % size != tail
注：满的时候却一个空，用于区分状态
*/

void RingBuffer_Init(RingBuffer *rb)
{
    rb->head = 0;
    rb->tail = 0;
   
}

//往缓冲区写数据
void Write_rb (uint8_t byte, RingBuffer *rb){

    uint16_t next = (rb->head + 1) % RB_SIZE;  // 先算下一个位置
		if (next == rb->tail) {
       return;  															// 满了，丢弃（或者覆盖旧数据，看策略）
     }
		 rb->buffer[rb->head] = byte;               //写入当前位置
		 rb->head = next;                          //更新写到的位置
		 
}

//向缓冲区里读数据
uint8_t Read_rb(uint8_t *data, RingBuffer *rb) {
    if (rb->head == rb->tail) {
        return 0;  // 空了，没数据
    }
    
    *data = rb->buffer[rb->tail];           // 取出当前数据
    rb->tail = (rb->tail + 1) % RB_SIZE;       // 推进读指针
    return 1;  // 成功
}
