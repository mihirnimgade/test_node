/*
 * xorshift.c
 *
 *  Created on: Oct 20, 2022
 *      Author: Mihir Nimgade
 */

#include "xorshift.h"

uint32_t xorshift32(struct xorshift32_state *state) {
    uint32_t x = state->a;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return state->a = x;
}

uint32_t rand(uint32_t limit) {
    // initialize xorshift state
    static struct xorshift32_state state = {
        .a = 3
    };

    return (xorshift32(&state) % limit);
}

void rand_array(uint8_t *array, uint8_t length) {
    for (uint8_t i=0; i<length; i++) {
        array[i] = rand(256);
    }
}


