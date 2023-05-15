//
// Created by gabri on 2023-05-12.
//
#include "../../Inc/waves.h"
#include "math.h"

unsigned int sine_val[N_SAMPLES];
unsigned int saw_val[N_SAMPLES];
unsigned int tri_val[N_SAMPLES];
unsigned int square_val[N_SAMPLES];


void calcsin (void)
{
    for (int i=0; i<N_SAMPLES; i++)
    {
        sine_val[i] = ((sin(i*2*PI/N_SAMPLES) + 1)*(4096/2));
    }
}

void calcsaw (void)
{
    for (int j=0; j<N_SAMPLES; j++)
    {
        saw_val[j] = ((float)j/N_SAMPLES)*(4096);
    }
}

void calctri (void)
{
    for (int k=0; k<N_SAMPLES/2; k++)
    {
        tri_val[k] = ((float)k/(N_SAMPLES/2))*(4096);
    }
    for (int k=N_SAMPLES/2; k<N_SAMPLES; k++)
    {
        tri_val[k] = (1-((float)(k-(N_SAMPLES/2)))/(N_SAMPLES/2))*(4096);
    }
}

void calcsquare (void)
{
    for (int h=0; h<N_SAMPLES/2; h++)
    {
        square_val[h] = 4095;
    }
    for (int h=N_SAMPLES/2; h<N_SAMPLES; h++)
    {
        square_val[h] = 0;
    }
}