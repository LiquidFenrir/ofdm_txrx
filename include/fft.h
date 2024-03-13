#ifndef ofdm_fft_h
#define ofdm_fft_h

#include "ofdm.h"
#include <complex.h>

#define OFDM_OWN_FFT

#ifdef OFDM_OWN_FFT
struct ofdm_fft_s;
typedef struct ofdm_fft_s* ofdm_fft_t;
typedef const struct ofdm_fft_s* ofdm_fft_ct;
#else
#include <kiss_fft.h>
#include <kiss_fftr.h>
typedef kiss_fftr_cfg ofdm_fft_t;
typedef kiss_fftr_cfg ofdm_fft_ct;
#endif

ofdm_fft_t ofdm_fft_new(int inverse);
void ofdm_fft_delete(ofdm_fft_t);

// OFDM_TXRX_NFFT length buffers
void ofdm_fft_forward_r2c(ofdm_fft_ct fft, const float* input, float complex* output);
// OFDM_TXRX_NFFT length buffers
void ofdm_fft_backward_c2r(ofdm_fft_ct fft, const float complex* input, float* output);

#endif
