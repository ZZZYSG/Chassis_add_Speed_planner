#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#define RB_SIZE 64            //缓存区固定为64字节大小

typedef struct {
    uint8_t buffer[RB_SIZE];
    volatile uint16_t head;  //生产者写的位置
    volatile uint16_t tail;  //消费者读的位置
} RingBuffer;


//重置缓存区
void RingBuffer_Init(RingBuffer *rb);

//往缓存区写数据
void Write_rb (uint8_t byte, RingBuffer *rb);

//读缓存区的数据
uint8_t Read_rb(uint8_t *data, RingBuffer *rb);


#endif
