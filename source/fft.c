#include "fft.h"
#include <assert.h>
#include <math.h>

#ifdef OFDM_OWN_FFT

struct ofdm_fft_s {
    // e^ix
    // x = 0, step pi / N
    float twiddle[OFDM_TXRX_NFFT][4];
};

// based on https://zestedesavoir.com/tutoriels/3939/jouons-a-implementer-une-transformee-de-fourier-rapide/implementons-la-fft/

ofdm_fft_t ofdm_fft_new(int inverse)
{
    ofdm_fft_t out = calloc(1, sizeof(struct ofdm_fft_s));
    const float ang = -2.0f * 3.14159265358979323846f / OFDM_TXRX_NFFT;
    const float complex wn = cexpf(I * ang);
    float complex w = 1.0f;
    for(int i = 0; i < OFDM_TXRX_NFFT; ++i)
    {
        const float re = crealf(w);
        const float im = cimagf(w);
        out->twiddle[i][0] = re;
        out->twiddle[i][1] = im;
        out->twiddle[i][2] = -re;
        out->twiddle[i][3] = -im;
        w *= wn;
    }
    return out;
}
void ofdm_fft_delete(ofdm_fft_t fft)
{
    free(fft);
}

static uint32_t reverse_bits(uint32_t x)
{
    x = (((x & 0xaaaaaaaa) >> 1) | ((x & 0x55555555) << 1));
    x = (((x & 0xcccccccc) >> 2) | ((x & 0x33333333) << 2));
    x = (((x & 0xf0f0f0f0) >> 4) | ((x & 0x0f0f0f0f) << 4));
    x = (((x & 0xff00ff00) >> 8) | ((x & 0x00ff00ff) << 8));
    return((x >> 16) | (x << 16));
}
static uint32_t reverse_bits_n(const uint32_t x, unsigned nbits)
{
    assert(x == (x & ((1u << nbits) - 1)));
    return (reverse_bits(x) >> (32 - nbits)) & ((1u << nbits) - 1);
}

#define OFDM_USE_SIMD

#ifdef OFDM_USE_SIMD
#include <stdint.h>
extern uint32_t ofdm_fft_simd_start(void*);
extern void ofdm_fft_simd_end(uint32_t, const void*);
extern void ofdm_fft_simd_compute(float complex* a0_ptr, float complex* a1_ptr, const float twiddle[4]);
#else
// not actually simd
static void ofdm_fft_simd_compute(float complex* a0_ptr, float complex* a1_ptr, const float twiddle[4])
{
    const float complex a0 = *a0_ptr;
    // OK: the first 2 components of the array pointed to by twiddle are a real and imaginary component
    const float complex a1 = *(const float complex*)twiddle * *a1_ptr;
    *a0_ptr = a0 + a1;
    *a1_ptr = a0 - a1;
}
#endif

static void ofdm_fft(ofdm_fft_t fft, float complex* data)
{
#ifdef OFDM_USE_SIMD
    float saved_banks[32];
    const uint32_t fpcsr_saved = ofdm_fft_simd_start(saved_banks);
#endif

    int n1 = 0;
    int n2 = 1;
    for(int i = 1; i <= OFDM_TXRX_POW2; ++i)
    {
        n1 = n2;
        n2 *= 2;
        int angle_index = 0;
        for(int j = 0; j < n1; ++j)
        {
            for(int k = j; k < OFDM_TXRX_NFFT; k += n2)
            {
                ofdm_fft_simd_compute(&data[k], &data[k + n1], &fft->twiddle[angle_index][0]);
            }
            angle_index += OFDM_TXRX_NFFT >> i;
        }
    }

#ifdef OFDM_USE_SIMD
    ofdm_fft_simd_end(fpcsr_saved, saved_banks);
#endif
    return;
}

void ofdm_fft_forward_r2c(ofdm_fft_t fft, const float* input, float complex* output)
{
    for(int i = 0; i < OFDM_TXRX_NFFT; ++i)
    {
        const int j = reverse_bits_n(i, OFDM_TXRX_POW2);
        output[j] = input[i];
        output[i] = input[j];
    }
    ofdm_fft(fft, output);
}
void ofdm_fft_backward_c2r(ofdm_fft_t fft, const float complex* input, float* output)
{
    float complex data[OFDM_TXRX_NFFT];
    for(int i = 0; i < OFDM_TXRX_NFFT; ++i)
    {
        const int j = reverse_bits_n(i, OFDM_TXRX_POW2);
        data[j] = conjf(input[i]);
        data[i] = conjf(input[j]);
    }
    ofdm_fft(fft, data);
    for(int i = 0; i < OFDM_TXRX_NFFT; ++i)
    {
        output[i] = crealf(data[i]);
    }
}

#else

ofdm_fft_t ofdm_fft_new(int inverse)
{
    return kiss_fftr_alloc(OFDM_TXRX_NFFT, inverse, NULL, NULL);
}
void ofdm_fft_delete(ofdm_fft_t fft)
{
    kiss_fftr_free(fft);
}
// All these work OFDM_TXRX_NFFT length buffers
void ofdm_fft_forward_r2c(ofdm_fft_t fft, const float* input, float complex* output)
{
    kiss_fftr(fft, input, (kiss_fft_cpx*)output);
}
void ofdm_fft_backward_c2r(ofdm_fft_t fft, const float complex* input, float* output)
{
    kiss_fftri(fft, (const kiss_fft_cpx*)input, output);
}

#endif
