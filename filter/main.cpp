/*
	Filename: fir.cpp
		FIR lab wirtten for WES/CSE237C class at UCSD.

	INPUT:
		x: signal (chirp)

	OUTPUT:
		y: filtered output

*/

#include "fir.h"

void fir (
  data_t *y,
  data_t x
  )
{
	coef_t c[N/2 + 1] = {53, 0, -91, 0, 313, 500};
	// Write your code here
	static
		data_t shift_reg[N];
		acc_t acc;
		int i;

	acc = 0;
	Shift_Accum_Loop:
	for (i = N-1; i >= 0; i--) {
        if (i == 0) {
            shift_reg[0] = x;
        } else {
            shift_reg[i] = shift_reg[i-1];
        }

        if (i == N/2) {
            acc += shift_reg[i] * c[i];
        } else if (i < N/2) {
            acc += (shift_reg[i] + shift_reg[N-1-i]) * c[i];
        }
    }
	*y = acc;
}
