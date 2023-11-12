#ifndef _SYS_RING_BUF_H_
#define _SYS_RING_BUF_H_


#ifdef __cplusplus
extern "C" {
#endif

/*���λ�����������*/
typedef struct {
     unsigned char *buf;    /*���λ�����        */
     unsigned int size;     /*���λ�����        */
     unsigned int front;    /*ͷָ��            */
     unsigned int rear;     /*βָ��            */
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
