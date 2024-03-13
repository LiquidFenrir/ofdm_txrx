#include "fft.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>

#ifdef OFDM_OWN_FFT

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

struct ofdm_fft_s {
    // e^ix
    // x = 0, step pi / N
    float complex twiddle[OFDM_TXRX_NFFT];
    unsigned mapping[OFDM_TXRX_NFFT];
};

// based on https://zestedesavoir.com/tutoriels/3939/jouons-a-implementer-une-transformee-de-fourier-rapide/implementons-la-fft/

ofdm_fft_t ofdm_fft_new(int inverse)
{
    ofdm_fft_t out = calloc(1, sizeof(struct ofdm_fft_s));
    const float ang = -2.0f * 3.14159265358979323846f / OFDM_TXRX_NFFT;
    const float complex wn = cexpf(I * ang);
    float complex w = 1.0f;
    for(unsigned i = 0; i < OFDM_TXRX_NFFT; ++i)
    {
        out->twiddle[i] = w;
        out->mapping[i] = reverse_bits_n(i, OFDM_TXRX_POW2);
        w *= wn;
    }
    return out;
}
void ofdm_fft_delete(ofdm_fft_t fft)
{
    free(fft);
}


#ifdef __3DS__
#define OFDM_USE_SIMD
#endif

#ifdef OFDM_USE_SIMD
extern uint32_t ofdm_fft_simd_start(void*);
extern void ofdm_fft_simd_end(uint32_t, const void*);
// s0-s1, r0, r1, r2, r3
extern void ofdm_fft_simd_compute(const float complex twiddle, float complex* src_ata_a0, float complex* src_data_a1, const void* const src_data_end, const unsigned step_bytes);
#endif

static void ofdm_fft(ofdm_fft_ct fft, float complex* data)
{
#ifdef OFDM_USE_SIMD
    float saved_banks[16];
    const uint32_t fpcsr_saved = ofdm_fft_simd_start(saved_banks);
    const void* const data_end = data + OFDM_TXRX_NFFT;
#endif

#pragma GCC unroll 7
    for(int i = 1; i <= OFDM_TXRX_POW2; ++i)
    {
        const int n1 = 1 << (i- 1);
#ifdef OFDM_USE_SIMD
        const unsigned step_bytes = sizeof(float complex) << i;
#else
        const int n2 = 1 << i;
#endif

        int angle_index = 0;
#pragma GCC unroll 128
        for(int j = 0; j < n1; ++j)
        {
#ifdef OFDM_USE_SIMD
            ofdm_fft_simd_compute(fft->twiddle[angle_index], data + j, data + j + n1, data_end, step_bytes);
#else
            const float complex twiddle = fft->twiddle[angle_index];
            for(int k = j; k < OFDM_TXRX_NFFT; k += n2)
            {
                const float complex a0 = data[k];
                const float complex a1 = twiddle * data[k + n1];
                data[k] = a0 + a1;
                data[k + n1] = a0 - a1;
            }
#endif
            angle_index += OFDM_TXRX_NFFT >> i;
        }
    }

#ifdef OFDM_USE_SIMD
    ofdm_fft_simd_end(fpcsr_saved, saved_banks);
#endif
    return;
}

void ofdm_fft_forward_r2c(ofdm_fft_ct fft, const float* input, float complex* output)
{
    for(int i = 0; i < OFDM_TXRX_NFFT; ++i)
    {
        const int j = fft->mapping[i];
        output[j] = input[i];
        output[i] = input[j];
    }
    ofdm_fft(fft, output);
}
void ofdm_fft_backward_c2r(ofdm_fft_ct fft, const float complex* input, float* output)
{
    float complex data[OFDM_TXRX_NFFT];
    for(int i = 0; i < OFDM_TXRX_NFFT; ++i)
    {
        const int j = fft->mapping[i];
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
void ofdm_fft_forward_r2c(ofdm_fft_ct fft, const float* input, float complex* output)
{
    kiss_fftr(fft, input, (kiss_fft_cpx*)output);
}
void ofdm_fft_backward_c2r(ofdm_fft_ct fft, const float complex* input, float* output)
{
    kiss_fftri(fft, (const kiss_fft_cpx*)input, output);
}

#endif
