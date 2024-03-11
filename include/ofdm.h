#ifndef ofdm_txrx_h
#define ofdm_txrx_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// #define OFDM_TXRX_MAX_PACKET_BYTES (1u << 8)
#define OFDM_TXRX_MAX_PACKET_BYTES (1u << 16)
#define OFDM_TXRX_NFFT 128
#define OFDM_ALLOW_RX 1
#define OFDM_ALLOW_TX 2
#define OFDM_ALLOW_SCRAMBLE 4
#define OFDM_ALLOW_DIRECTION (OFDM_ALLOW_TX | OFDM_ALLOW_RX)
#define OFDM_ALLOW_ALL ((OFDM_ALLOW_DIRECTION) | OFDM_ALLOW_SCRAMBLE)

enum ofdm_txrx_state_e {
    OFDM_TXRX_INVALID = -1,
    OFDM_TXRX_NOTHING = 0,
    OFDM_TXRX_NEW_PACKET, // call ofdm_get_packet_size after
    OFDM_TXRX_PACKET_COMPLETE_SUCCESS,
    OFDM_TXRX_PACKET_COMPLETE_FAILURE,
};
typedef enum ofdm_txrx_state_e ofdm_txrx_state_t;

struct ofdm_txrx_s;
typedef struct ofdm_txrx_s* ofdm_t;

ofdm_t ofdm_new(int bits_per_qam, int flags);
void ofdm_delete(ofdm_t ofdm);

// pass in a float buffer as user_data
static inline void ofdm_callback_read_f32_samples(float* output_samples, const size_t sample_count, void* user_data)
{
    memcpy(output_samples, user_data, sample_count * sizeof(float));
}
// pass in a pcm16 buffer as user_data
static inline void ofdm_callback_read_pcm16_samples(float* output_samples, const size_t sample_count, void* user_data)
{
    for(size_t i = 0; i < sample_count; ++i)
    {
        output_samples[i] = ((int16_t*)user_data)[i] * 0.000030517578125f;
    }
}

// copies in the samples from the user buffer, with conversion to float if needed
// zero pad if needed, but always write sample_count samples.
typedef void (*ofdm_read_samples_cb_t)(float* output_samples, const size_t sample_count, void* user_data);
ofdm_txrx_state_t ofdm_read_samples(ofdm_t ofdm, ofdm_read_samples_cb_t callback, void* user_data);
// in bytes <= OFDM_TXRX_MAX_PACKET_BYTES
size_t ofdm_get_packet_recv_size(ofdm_t ofdm);
size_t ofdm_get_packet_recv_size_remaining(ofdm_t ofdm);
int ofdm_get_packet_recv_checksums(ofdm_t ofdm, uint32_t* computed, uint32_t* expected);

// expects a buffer of size >= ofdm_get_packet_recv_size
// use size OFDM_TXRX_MAX_PACKET_BYTES for safety/simplicity
// or NULL to reset to ignore the packet data
int ofdm_set_packet_recv_buffer(ofdm_t ofdm, unsigned char* buffer);

// in samples, 1 <= data_length <= OFDM_TXRX_MAX_PACKET_BYTES
size_t ofdm_get_packet_send_length(ofdm_t ofdm, size_t data_length);

typedef struct ofdm_samples_s {
    size_t index;
    void* data;
} ofdm_samples_t;

static inline void ofdm_callback_write_f32_samples(const float* input_samples, const size_t sample_count, void* user_data)
{
    ofdm_samples_t* samples = user_data;
    float* output_samples = &(((float*)samples->data)[samples->index]);
    memcpy(output_samples, input_samples, sample_count * sizeof(float));
    samples->index += sample_count;
}
static inline void ofdm_callback_write_pcm16_samples(const float* input_samples, const size_t sample_count, void* user_data)
{
    ofdm_samples_t* samples = user_data;
    int16_t* output_samples = &(((int16_t*)samples->data)[samples->index]);
    for(size_t i = 0; i < sample_count; ++i)
    {
        const float x = input_samples[i];
        const float c = 1.0f + ((x < -1) ? -1 : ((x > 1) ? 1 : x));
        int r = (int)(c * 32767.5f) - 32768;
        output_samples[i] = (int16_t)r;
    }
    samples->index += sample_count;
}

// copies out the samples to the user buffer, with conversion from float if needed
typedef void (*ofdm_write_samples_cb_t)(const float* input_samples, const size_t sample_count, void* user_data);
int ofdm_write_samples(ofdm_t ofdm, const unsigned char* data, size_t data_length, ofdm_write_samples_cb_t callback, void* user_data);

#ifdef __cplusplus
}
#endif

#endif
