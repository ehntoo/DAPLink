/**
 * @file    circular_buffer.c
 * @brief   Implementation of a circular buffer
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2016-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "circ_buf.h"

#include "cortex_m.h"
#include "util.h"
#include "string.h"

void circ_buf_init(circ_buf_t *circ_buf, uint8_t *buffer, uint32_t size)
{
    cortex_int_state_t state;
    state = cortex_int_get_and_disable();

    circ_buf->buf = buffer;
    circ_buf->size = size;
    circ_buf->read = 0;
    circ_buf->write = 0;

    cortex_int_restore(state);
}

bool circ_buf_empty(circ_buf_t *circ_buf)
{
    cortex_int_state_t state;
    bool is_empty;
    state = cortex_int_get_and_disable();
    is_empty = circ_buf->read == circ_buf->write;
    cortex_int_restore(state);
    return is_empty;
}

bool circ_buf_full(circ_buf_t *circ_buf)
{
    return circ_buf->size == circ_buf_count_used(circ_buf);
}

static uint32_t circ_buf_idx_mask(circ_buf_t *circ_buf, uint32_t idx)
{
    return idx & (circ_buf->size - 1);
}

void circ_buf_push(circ_buf_t *circ_buf, uint8_t data)
{
    cortex_int_state_t state;
    state = cortex_int_get_and_disable();

    util_assert(!circ_buf_full(circ_buf));

    circ_buf->buf[circ_buf_idx_mask(circ_buf, circ_buf->write)] = data;
    circ_buf->write += 1;

    cortex_int_restore(state);
}

uint8_t circ_buf_pop(circ_buf_t *circ_buf)
{
    uint8_t data;
    cortex_int_state_t state;

    state = cortex_int_get_and_disable();

    // Assert buffer isn't empty
    util_assert(!circ_buf_empty(circ_buf));

    data = circ_buf->buf[circ_buf_idx_mask(circ_buf, circ_buf->read)];
    circ_buf->read += 1;

    cortex_int_restore(state);

    return data;
}

uint32_t circ_buf_count_used(circ_buf_t *circ_buf)
{
    uint32_t cnt;
    cortex_int_state_t state;

    state = cortex_int_get_and_disable();

    cnt = circ_buf->write - circ_buf->read;

    cortex_int_restore(state);
    return cnt;
}

uint32_t circ_buf_count_free(circ_buf_t *circ_buf)
{
    return circ_buf->size - circ_buf_count_used(circ_buf);
}

// size-1 contiguous free
// r
// w.........
// 0123456789

// size-2 contiguous free
//  r
// .w........
// 0123456789

// size-2 contiguous free
// rw........
// 0123456789

// full
// wr........
// 0123456789

// full
// .wr.......
// 0123456789

// 1 free
// w.r.......
// 0123456789

// uint32_t circ_buf_count_contiguous_free(circ_buf_t *circ_buf)
// {
//     uint32_t cnt;
//     cortex_int_state_t state;

//     state = cortex_int_get_and_disable();

//     if (circ_buf->tail < circ_buf->head) {
//         cnt = circ_buf->head - circ_buf->tail;
//     } else {
//         cnt = circ_buf->size - circ_buf->tail;
//     }
//     cortex_int_restore(state);
//     return cnt;
// }

uint8_t* circ_buf_write_peek(circ_buf_t *circ_buf, uint32_t* size)
{
    uint32_t cnt;
    cortex_int_state_t state;
    uint8_t* buf;

    state = cortex_int_get_and_disable();
    uint32_t masked_read = circ_buf_idx_mask(circ_buf, circ_buf->read);
    uint32_t masked_write = circ_buf_idx_mask(circ_buf, circ_buf->write);
    if (circ_buf_full(circ_buf)) {
        *size = 0;
    } else if (circ_buf_empty(circ_buf)) {
        *size = circ_buf->size - masked_write;
    } else if (masked_write < masked_read) {
        *size = masked_read - masked_write;
    } else {
        *size = circ_buf->size - masked_write;
    }
    buf = &circ_buf->buf[masked_write];
    cortex_int_restore(state);
    return buf;
}

void circ_buf_push_n(circ_buf_t *circ_buf, uint32_t size)
{
    cortex_int_state_t state;

    state = cortex_int_get_and_disable();
    util_assert(circ_buf_count_free(circ_buf) >= size);
    circ_buf->write += size;
    cortex_int_restore(state);
}

uint32_t circ_buf_read(circ_buf_t *circ_buf, uint8_t *data, uint32_t size)
{
    uint32_t cnt;
    cortex_int_state_t state;
    uint32_t i;

    state = cortex_int_get_and_disable();

    cnt = circ_buf_count_used(circ_buf);
    cnt = MIN(size, cnt);
    uint32_t masked_read = circ_buf_idx_mask(circ_buf, circ_buf->read);
    uint32_t masked_write = circ_buf_idx_mask(circ_buf, circ_buf->write);

    uint32_t tail_cnt = MIN(cnt, circ_buf->size - masked_read);
    uint32_t head_cnt = MIN(cnt - tail_cnt, masked_write);
    memcpy(data, &circ_buf->buf[masked_read], tail_cnt);
    memcpy(&data[tail_cnt], circ_buf->buf, head_cnt);
    circ_buf->read += cnt;

    cortex_int_restore(state);
    return cnt;
}

uint32_t circ_buf_write(circ_buf_t *circ_buf, const uint8_t *data, uint32_t size)
{
    uint32_t cnt;
    uint32_t i;
    cortex_int_state_t state;

    state = cortex_int_get_and_disable();

    cnt = circ_buf_count_free(circ_buf);
    cnt = MIN(size, cnt);
    uint32_t masked_read = circ_buf_idx_mask(circ_buf, circ_buf->read);
    uint32_t masked_write = circ_buf_idx_mask(circ_buf, circ_buf->write);

    uint32_t tail_cnt = MIN(cnt, circ_buf->size - masked_write);
    uint32_t head_cnt = MIN(cnt - tail_cnt, masked_read);
    memcpy(&circ_buf->buf[masked_write], data, tail_cnt);
    memcpy(circ_buf->buf, &data[tail_cnt], head_cnt);
    circ_buf->write += cnt;

    cortex_int_restore(state);

    return cnt;
}

const uint8_t* circ_buf_peek(circ_buf_t *circ_buf, uint32_t* size)
{
    uint32_t cnt;
    uint8_t* ret;
    cortex_int_state_t state;

    state = cortex_int_get_and_disable();

    uint32_t masked_read = circ_buf_idx_mask(circ_buf, circ_buf->read);
    uint32_t masked_write = circ_buf_idx_mask(circ_buf, circ_buf->write);

    if (circ_buf_full(circ_buf)) {
        *size = circ_buf->size - masked_read;
    } else if (circ_buf_empty(circ_buf)) {
        *size = 0;
    } else if (masked_write > masked_read) {
        cnt = masked_write - masked_read;
    } else {
        cnt = circ_buf->size - masked_read;
    }
    ret = &circ_buf->buf[masked_read];

    cortex_int_restore(state);

    if (size) {
        *size = cnt;
    }
    return ret;
}

void circ_buf_pop_n(circ_buf_t *circ_buf, uint32_t n)
{
    cortex_int_state_t state;

    state = cortex_int_get_and_disable();

    util_assert(circ_buf_count_used(circ_buf) >= n);
    circ_buf->read += n;

    cortex_int_restore(state);
}
