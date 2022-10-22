/*
 * xorshift.h
 *
 *  Created on: Oct 20, 2022
 *      Author: Mihir Nimgade
 */

#ifndef INC_XORSHIFT_H_
#define INC_XORSHIFT_H_

#include "stdint.h"


// struct declarations

struct xorshift32_state {
    uint32_t a;
};


// function declarations

uint32_t xorshift32(struct xorshift32_state *state);
uint32_t rand(uint32_t limit);
void rand_array(uint8_t *array, uint8_t length);


#endif /* INC_XORSHIFT_H_ */
