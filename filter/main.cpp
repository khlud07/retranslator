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
	for (int i = N-1; i > 0; i--)
	    shift_reg[i] = shift_reg[i-1];
	    shift_reg[0] = x;

	for (int i = 0; i < N/2; i++)
	    acc += (shift_reg[i] + shift_reg[N-1-i]) * c[i];

	acc += shift_reg[N/2] * c[N/2]; // middle tap

	*y = acc;
}
