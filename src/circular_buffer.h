#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#define N_circular_buf 4096

typedef struct {
    short *buf; // actuall capacity = N-1
    int write_idx; // idx to write into circular buf
    int read_idx; // idx to read out from circular buf
} circular_buf;

bool is_empty(circular_buf *buf) {
    if (buf->read_idx == buf->write_idx) {
        return true;
    }
    else {
        return false;
    }
}

bool is_full(circular_buf *buf) {
    if ((buf->write_idx + 1) % N_circular_buf == buf->read_idx) {
        return true;
    }
    else {
        return false;
    }
}

int buf_length(circular_buf *buf) {
    // NRF_GPIOTE->INTENCLR |= 0x00000080;
    int temp = (buf->write_idx - buf->read_idx + N_circular_buf) % N_circular_buf;
    // NRF_GPIOTE->INTENSET |= 0x00000080;
    return temp;
}

int write_in_buf(circular_buf *buf, short *value, int L) {
    // L should be sizeof(value)/sizeof(short)
    if (L + buf_length(buf) > N_circular_buf-1) {
        printk("circular_buf out of range\n");
        return -1;
    }
    
    for (int i = 0;i < L;i++) {
        buf->buf[buf->write_idx] = value[i];
        buf->write_idx = (buf->write_idx + 1) % N_circular_buf;
    }
    return 0;
}

int read_out_buf(circular_buf *buf, short *value, uint16_t *buf_l) {
    uint16_t L = (uint16_t)buf_length(buf);
    buf_l[0] = L;
    for (int i=0;i<L;i++) {
        value[i] = buf->buf[buf->read_idx];
        buf->read_idx = (buf->read_idx + 1) % N_circular_buf;
    }
    // if (L > 0) {
    //     printk("buf_length=%d\n, write_idx=%d, read_idx=%d\n",L, buf->write_idx, buf->read_idx);
    // }
    return 0;
}