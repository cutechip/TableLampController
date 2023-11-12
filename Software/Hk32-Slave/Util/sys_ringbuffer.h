#ifndef _SYS_RING_BUF_H_
#define _SYS_RING_BUF_H_


#ifdef __cplusplus
extern "C" {
#endif

/*环形缓冲区管理器*/
typedef struct {
     unsigned char *buf;    /*环形缓冲区        */
     unsigned int size;     /*环形缓冲区        */
     unsigned int front;    /*头指针            */
     unsigned int rear;     /*尾指针            */
}ring_buf_t;

unsigned char ring_buf_init(ring_buf_t *r, unsigned char *buf,unsigned int size);

void ring_buf_clr(ring_buf_t *r);

unsigned int ring_buf_len(ring_buf_t *r);

unsigned int ring_buf_put(ring_buf_t *r, unsigned char *buf, unsigned int len);

unsigned int ring_buf_get(ring_buf_t *r,unsigned char *buf,unsigned int len);


#ifdef __cplusplus
}
#endif

#endif
