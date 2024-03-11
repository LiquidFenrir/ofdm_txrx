#include "ofdm.h"

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>
#include <complex.h>
#include "bitstream.h"
#include "crc32.h"
#include "kiss_fft.h"
#include "kiss_fftr.h"

#define OFDM_SCRAMBLE_INIT_STATE 0x5D
static uint8_t ofdm_scrambling_alg(uint8_t* output, const uint8_t* input, const size_t length, uint8_t state)
{
    struct bitstream_reader_t reader;
    bitstream_reader_init(&reader, input);
    struct bitstream_writer_t writer;
    bitstream_writer_init(&writer, output);
    const size_t length_in_bits = length * 8;
    for(size_t i = 0; i < length_in_bits; ++i)
    {
        const uint8_t newbit = ((state & (1u << 0)) >> 0) ^ ((state & (1u << 3)) >> 3);
        bitstream_writer_write_bit(&writer, newbit ^ bitstream_reader_read_bit(&reader));
        state >>= 1;
        state |= newbit << 6;
    }
    return state;
}

static void cross_correlation(const float* check_against, const float* check, const int n, int* maximum_index_out, float* maximum_value_out)
{
    int lastArrayIndex = n - 1;
    float maxCorrelationValue = 0;
    int maximum_index = -1;
    float currentCorrelationValue = 0;

    for(int i = 0; i < lastArrayIndex; i++)
    {
        currentCorrelationValue = 0;

        for(int b = lastArrayIndex - i; b >= 0; b--)
        {
            currentCorrelationValue += check_against[b] * check[b + i];
        }

        // polarity may be reversed, don't care, pilots will fix
        if (fabsf(currentCorrelationValue) > maxCorrelationValue)
        {
            maxCorrelationValue = fabsf(currentCorrelationValue);
            maximum_index = i;
        }

        /*
        if(i != 0)
        {
            currentCorrelationValue = 0;
            for(int b = lastArrayIndex - i; b >= 0; b--)
            {
                currentCorrelationValue += check_against[b + i] * check[b];
            }
            
            if (currentCorrelationValue > maxCorrelationValue)
            {
                maxCorrelationValue = currentCorrelationValue;
            }
        }
        */
    }

    if(maximum_index_out)
    {
        *maximum_index_out = maximum_index;
    }
    if(maximum_value_out)
    {
        *maximum_value_out = maxCorrelationValue;
    }
}

static const int ofdm_pilots_indices[4] = {
    13,
    26,
    39,
    52,
};
static const float complex ofdm_pilots_expect[4] = {
    1.0f + I * 0.0f,
    1.0f + I * 0.0f,
    1.0f + I * 0.0f,
    -1.0f + I * 0.0f,
};

#define OFDM_CARRIER_START 2
#define OFDM_CARRIER_END ((OFDM_TXRX_NFFT / 2) - 2)
#define OFDM_CARRIER_AMOUNT ((OFDM_CARRIER_END) - (OFDM_CARRIER_START) - 4)

struct ofdm_rx_s {
    kiss_fftr_cfg cfg_recv;
    size_t samples_seen;
    unsigned long long max_idx;
    unsigned long long max_idx_max;
    unsigned long long prev_max_idx_max;
    float complex initial;
    int work_buffer_section_idx;
    int max_idx_count;
    int max_idx_count_max;
    int max_idx_count_max_max;
    float max_val;
    uint8_t* symbol_output_buffer;
    uint8_t* descrambled_data;
    unsigned char* data_output_buffer; // user provided
    float header_max_correl;
    int disabled;
    int output_buffer_real[OFDM_CARRIER_AMOUNT];
    int output_buffer_imag[OFDM_CARRIER_AMOUNT];
    float complex work_buffer[OFDM_TXRX_NFFT];
    float samples_buffer[OFDM_TXRX_NFFT * 3];
    bool locking, locked, got_new_idx;
    unsigned data_length, data_length_done;
    uint32_t checksum;
    uint32_t expect_checksum;
    uint8_t scrambling_state;
};
struct ofdm_tx_s {
    kiss_fftr_cfg cfg_send;
    unsigned char* meta_buffer;
    uint8_t* scrambled_data;
    float complex work_buffer[OFDM_TXRX_NFFT];
    float samples_buffer[OFDM_TXRX_NFFT];
    float scaler;
};
struct ofdm_txrx_s {
    int flags;
    int bits_per_qam;
    int bytes_per_symbol;
    int max_range;
    int mapping_size;
    float* mapping;
    float expect_header[OFDM_TXRX_NFFT];
    int carrier_indices[OFDM_CARRIER_AMOUNT];
    struct ofdm_rx_s* rx;
    struct ofdm_tx_s* tx;
};

static void ofdm_tx_delete(struct ofdm_tx_s* tx)
{
    kiss_fftr_free(tx->cfg_send);
    free(tx->meta_buffer);
    free(tx->scrambled_data); // may be NULL
    free(tx);
}
static void ofdm_rx_delete(struct ofdm_rx_s* rx)
{
    kiss_fftr_free(rx->cfg_recv);
    free(rx->symbol_output_buffer);
    if(rx->symbol_output_buffer != rx->descrambled_data)
        free(rx->descrambled_data);
    free(rx);
}
ofdm_t ofdm_new(int bits_per_qam, int flags)
{
    if(!(flags & OFDM_ALLOW_DIRECTION))
        return NULL;

    ofdm_t out = calloc(1, sizeof(struct ofdm_txrx_s));
    out->flags = flags;
    out->tx = calloc(1, sizeof(struct ofdm_tx_s));
    out->rx = calloc(1, sizeof(struct ofdm_rx_s));

    out->bits_per_qam = bits_per_qam;
    const int BITS_PER_SYMBOL = OFDM_CARRIER_AMOUNT * bits_per_qam;
    out->bytes_per_symbol = BITS_PER_SYMBOL / 8;
    assert(out->bytes_per_symbol * 8 == BITS_PER_SYMBOL);

    int k = 0;
    for(int i = 0, j = OFDM_CARRIER_START; j < OFDM_CARRIER_END; ++i, ++j)
    {
        if(k < 4 && j == ofdm_pilots_indices[k])
        {
            ++k;
            ++j;
        }
        out->carrier_indices[i] = j;
    }

    // const int QAM_SIZE = 1 << bits_per_qam;
    out->max_range = (1 << (bits_per_qam / 2)) - 1;
    out->mapping_size = 1 << (bits_per_qam / 2);
    out->mapping = (float*)calloc(out->mapping_size, sizeof(float));
    k = 0;
    for(int i = -out->max_range; i <= -1; i += 2)
    {
        out->mapping[k++] = (float)i;
    }
    for(int i = 1; i <= out->max_range; i += 2)
    {
        out->mapping[k++] = (float)i;
    }

    out->tx->cfg_send = kiss_fftr_alloc(OFDM_TXRX_NFFT, 1, NULL, NULL);

    for(int i = 0; i < 4; ++i)
    {
        // printf("scaled_out_buffer[%d]\n", ofdm_pilots_indices[i]);
        out->tx->work_buffer[ofdm_pilots_indices[i]] = ofdm_pilots_expect[i] * out->max_range;
        // printf("Re Im %+.2f %+.2f\n", crealf(scaled_out_buffer[ofdm_pilots_indices[i]]), cimagf(scaled_out_buffer[ofdm_pilots_indices[i]]));
    }
    kiss_fftri(out->tx->cfg_send, (const kiss_fft_cpx*)out->tx->work_buffer, out->expect_header);
    // out->tx->scaler = 1.0f / (OFDM_TXRX_NFFT * sqrtf(out->max_range * 2.0f));
    out->tx->scaler = 1.0f / (OFDM_TXRX_NFFT * out->max_range);
    for(int i = 0; i < OFDM_TXRX_NFFT; ++i)
    {
        out->expect_header[i] *= out->tx->scaler;
    }

    // printf("out->expect_header\n");
    // for(int i = 0; i < OFDM_TXRX_NFFT; ++i)
    // {
    //     printf("%03d %+.2f\n", out->expect_header[i]);
    // }

    if(!(flags & OFDM_ALLOW_RX))
    {
        ofdm_rx_delete(out->rx);
        out->rx = NULL;
    }
    else
    {
        out->rx->cfg_recv = kiss_fftr_alloc(OFDM_TXRX_NFFT, 0, NULL, NULL);
        // printf("max correl: %.3f\n", header_max_correl);
        cross_correlation(out->expect_header, out->expect_header, OFDM_TXRX_NFFT, NULL, &out->rx->header_max_correl);
        out->rx->header_max_correl = sqrtf(out->rx->header_max_correl);
        // printf("sqrt max correl: %.3f\n", header_max_correl);
        out->rx->symbol_output_buffer = malloc(out->bytes_per_symbol);
        if(flags & OFDM_ALLOW_SCRAMBLE)
            out->rx->descrambled_data = malloc(out->bytes_per_symbol);
        else
            out->rx->descrambled_data = out->rx->symbol_output_buffer;
        out->rx->initial = 1.0f + I * 0.0f;
        out->rx->samples_seen = -OFDM_TXRX_NFFT;
    }

    if(!(flags & OFDM_ALLOW_TX))
    {
        ofdm_tx_delete(out->tx);
        out->tx = NULL;
    }
    else
    {
        out->tx->meta_buffer = malloc(out->bytes_per_symbol);
        if(flags & OFDM_ALLOW_SCRAMBLE)
            out->tx->scrambled_data = malloc(out->bytes_per_symbol);
    }

    // printf("ofdm %p rx %p tx %p\n", out, out->rx, out->tx);
    return out;
}

void ofdm_delete(ofdm_t ofdm)
{
    if(!ofdm)
        return;

    if(ofdm->rx)
    {
        ofdm_rx_delete(ofdm->rx);
    }
    if(ofdm->tx)
    {
        ofdm_tx_delete(ofdm->tx);
    }
    free(ofdm->mapping);
    free(ofdm);
}

#define OFDM_BUFFER_NEW_LEN (OFDM_TXRX_NFFT)
#define OFDM_BUFFER_NEW_POS ((OFDM_TXRX_NFFT * 3) - OFDM_BUFFER_NEW_LEN)
#define OFDM_BUFFER_OLD_LEN (OFDM_BUFFER_NEW_POS)
#define OFDM_BUFFER_OLD_POS (OFDM_BUFFER_NEW_LEN)
ofdm_txrx_state_t ofdm_read_samples(ofdm_t ofdm, ofdm_read_samples_cb_t callback, void* user_data)
{
    if(!ofdm || !ofdm->rx)
        return OFDM_TXRX_INVALID;

    // printf("ofdm_read_samples ofdm %p rx %p tx %p\n", ofdm, ofdm->rx, ofdm->tx);
    ofdm_txrx_state_t out = OFDM_TXRX_NOTHING;
    ofdm->rx->samples_seen += OFDM_TXRX_NFFT;

    memmove(&ofdm->rx->samples_buffer[0], &ofdm->rx->samples_buffer[OFDM_BUFFER_OLD_POS], OFDM_BUFFER_OLD_LEN * sizeof(float));
    callback(&ofdm->rx->samples_buffer[OFDM_BUFFER_NEW_POS], OFDM_BUFFER_NEW_LEN, user_data);

    if(ofdm->rx->disabled >= OFDM_TXRX_NFFT)
        ofdm->rx->disabled -= OFDM_TXRX_NFFT;
    if(ofdm->rx->disabled >= OFDM_TXRX_NFFT)
        return out;

    if(!ofdm->rx->locking && !ofdm->rx->locked)
    {
        const bool had_new_idx = ofdm->rx->got_new_idx;
        ofdm->rx->got_new_idx = false;

        for(int offset = OFDM_BUFFER_OLD_POS; offset < OFDM_BUFFER_NEW_POS; ++offset)
        {
            float new_max_val = 0;
            int new_max_idx = -1;
            cross_correlation(ofdm->expect_header, &ofdm->rx->samples_buffer[offset], OFDM_TXRX_NFFT, &new_max_idx, &new_max_val);
            new_max_idx += offset;

            // printf("xcorr: %f %d\n", new_max_val, new_max_idx);
            if(new_max_val > ofdm->rx->max_val)
            {
                const unsigned long long new_max_idx_absolute = (unsigned long long)(new_max_idx + ofdm->rx->samples_seen - OFDM_BUFFER_NEW_POS);
                if(ofdm->rx->max_idx == new_max_idx_absolute)
                {
                    ofdm->rx->max_idx_count += 1;
                }
                else
                {
                    ofdm->rx->max_idx_count = 1;
                    ofdm->rx->max_idx = new_max_idx_absolute;
                    ofdm->rx->got_new_idx = true; 
                }

                if(ofdm->rx->max_idx_count > ofdm->rx->max_idx_count_max)
                {
                    ofdm->rx->max_idx_count_max = ofdm->rx->max_idx_count;
                    ofdm->rx->max_idx_max = ofdm->rx->max_idx;
                }

                ofdm->rx->max_val = new_max_val;
            }
        }

        if(ofdm->rx->max_idx_max == ofdm->rx->prev_max_idx_max)
            ofdm->rx->max_idx_count_max_max += 1;
        else
        {
            ofdm->rx->prev_max_idx_max = ofdm->rx->max_idx_max;
            ofdm->rx->max_idx_count_max_max = 1;
        }

        ofdm->rx->work_buffer_section_idx = ofdm->rx->max_idx_max - (ofdm->rx->samples_seen - OFDM_BUFFER_NEW_POS);
        // printf("maxes: %d %d %lld %d %.2f\n", ofdm->rx->max_idx_count_max, ofdm->rx->max_idx_count_max_max, ofdm->rx->max_idx_max, ofdm->rx->work_buffer_section_idx, ofdm->rx->max_val);
        if(ofdm->rx->max_idx_count_max < (OFDM_TXRX_NFFT / 4) && (ofdm->rx->max_idx_count_max_max < 2 || ofdm->rx->max_idx_count_max < 4))
            return out;

        if(!had_new_idx)
            return out;

        ofdm->rx->locking = true;
        // printf("ofdm->rx->locking at %lld @ val %.2f\n", ofdm->rx->max_idx_max, ofdm->rx->max_val);
        // printf("means length at %lld\n", ofdm->rx->max_idx_max + (OFDM_TXRX_NFFT * 2) + (OFDM_TXRX_NFFT / 4));

        if(ofdm->rx->work_buffer_section_idx >= OFDM_BUFFER_NEW_POS)
        {
            ofdm->rx->work_buffer_section_idx -= OFDM_TXRX_NFFT;
            return out;
        }
    }

    if(ofdm->rx->locking)
    {
        // printf("locking work_buffer_section_idx: %d\n", ofdm->rx->work_buffer_section_idx);
        const float* const section = &ofdm->rx->samples_buffer[ofdm->rx->work_buffer_section_idx];
        kiss_fftr(ofdm->rx->cfg_recv, section, (kiss_fft_cpx*)(ofdm->rx->work_buffer));
        float complex pilots_found[4];
        for(int i = 0; i < 4; ++i)
        {
            pilots_found[i] = ofdm->rx->work_buffer[ofdm_pilots_indices[i]];
        }

        float complex temp_initial = 0;
        for(int i = 0; i < 3; ++i)
            temp_initial += pilots_found[i];
        temp_initial = (ofdm->max_range * 3.0f / temp_initial);

        for(int i = 0; i < 4; ++i)
        {
            pilots_found[i] *= temp_initial;
            // printf("Pilot %d @ %d: r %.2f i %.2f\n", i, ofdm_pilots_indices[i], crealf(pilots_found[i]), cimagf(pilots_found[i]));
        }

        bool condition = true;
        for(int i = 0; i < 4; ++i)
        {
            const float complex delta = pilots_found[i] - ofdm_pilots_expect[i] * (ofdm->max_range);
            condition &= cabsf(delta) <= 0.05f;
            // printf("cond %d : r %.2f i %.2f ; abs %.2f\n", i, crealf(delta), cimagf(delta), cabsf(delta));
        }

        float complex delta = pilots_found[0] - pilots_found[1];
        condition &= cabsf(delta) <= 0.05f;
        // printf("cond a : r %.2f i %.2f ; abs %.2f\n", crealf(delta), cimagf(delta), cabsf(delta));
        delta = pilots_found[1] - pilots_found[2];
        condition &= cabsf(delta) <= 0.05f;
        // printf("cond b : r %.2f i %.2f ; abs %.2f\n", crealf(delta), cimagf(delta), cabsf(delta));
        delta = pilots_found[2] + pilots_found[3];
        condition &= cabsf(delta) <= 0.05f;
        // printf("cond c : r %.2f i %.2f ; abs %.2f\n", crealf(delta), cimagf(delta), cabsf(delta));

        assert(condition);

        ofdm->rx->disabled = ofdm->rx->max_idx_max - (ofdm->rx->samples_seen - OFDM_BUFFER_NEW_POS) + OFDM_TXRX_NFFT * 9 / 4;
        // printf("disabled set: %d\n", ofdm->rx->disabled);
        ofdm->rx->max_idx = -1;
        ofdm->rx->max_val = 0;
        ofdm->rx->max_idx_count = 0;
        ofdm->rx->work_buffer_section_idx = 0;
        ofdm->rx->prev_max_idx_max = 0;
        ofdm->rx->max_idx_count_max = 0;
        ofdm->rx->max_idx_count_max_max = 0;
        ofdm->rx->locked = true;
        ofdm->rx->initial = temp_initial;
        ofdm->rx->scrambling_state = OFDM_SCRAMBLE_INIT_STATE;
        // printf("initial: r %.2f i %.2f\n", crealf(ofdm->rx->initial), cimagf(ofdm->rx->initial));
    }

    if(ofdm->rx->locking && ofdm->rx->locked)
    {
        ofdm->rx->locking = false;
        if(ofdm->rx->disabled >= OFDM_TXRX_NFFT)
            ofdm->rx->disabled -= OFDM_TXRX_NFFT;
        if(ofdm->rx->disabled >= OFDM_TXRX_NFFT)
            return out;
    }

    const int offset = ofdm->rx->disabled + OFDM_TXRX_NFFT;
    const float* const section = &ofdm->rx->samples_buffer[offset];
    kiss_fftr(ofdm->rx->cfg_recv, section, (kiss_fft_cpx*)(ofdm->rx->work_buffer));

    // printf("disabled inner %d\n", ofdm->rx->disabled);
    if(ofdm->rx->disabled < 0)
        ofdm->rx->disabled += (OFDM_TXRX_NFFT / 4);
    else
        ofdm->rx->disabled += (OFDM_TXRX_NFFT + (OFDM_TXRX_NFFT / 4));
    // printf("disabled next %d\n", ofdm->rx->disabled);

    // if(ofdm->rx->data_length == 0)
    // {
    //     printf("data length @ %lld\n", ofdm->rx->samples_seen + offset - OFDM_TXRX_NFFT - OFDM_TXRX_NFFT);
    // }
    // else
    // {
    //     printf("data @ %lld\n", recv_idx + offset - nfft - meta_offset - nfft);
    // }

    struct bitstream_writer_t writer;
    bitstream_writer_init(&writer, ofdm->rx->symbol_output_buffer);
    // for(int i = 0; i < 4; ++i)
    // {
    //     const kiss_fft_cpx* const cpx = &out_work_buffer[pilots_indices[i]];
    //     const float complex pil = (cpx->r + I * cpx->i) * initial;
    //     printf("Pilot %d @ %d: r %.2f i %.2f -> r %.2f i %.2f\n", i, pilots_indices[i], cpx->r, cpx->i, crealf(pil), cimagf(pil));
    // }
    for(int i = 0; i < OFDM_CARRIER_AMOUNT; ++i)
    {
        const float complex original = ofdm->rx->work_buffer[ofdm->carrier_indices[i]];
        const float complex scaled = original * ofdm->rx->initial;
        ofdm->rx->output_buffer_real[i] = -1;
        ofdm->rx->output_buffer_imag[i] = -1;
        for(int j = 0; j < ofdm->mapping_size; ++j)
        {
            if(fabsf(crealf(scaled) - ofdm->mapping[j]) <= 0.375f)
            {
                ofdm->rx->output_buffer_real[i] = j;
            }
            if(fabsf(cimagf(scaled) - ofdm->mapping[j]) <= 0.375f)
            {
                ofdm->rx->output_buffer_imag[i] = j;
            }
        }
        assert(ofdm->rx->output_buffer_real[i] != -1);
        assert(ofdm->rx->output_buffer_imag[i] != -1);
        // printf("Carrier %d @ %d: r %.2f i %.2f -> r %.2f i %.2f\n", i, ofdm->carrier_indices[i], crealf(original), cimagf(original), crealf(scaled), cimagf(scaled));
    }
    for(int i = 0; i < OFDM_CARRIER_AMOUNT; ++i)
    {
        for(int j = ((ofdm->bits_per_qam / 2) - 1); j >= 0; --j)
        {
            bitstream_writer_write_bit(&writer, (ofdm->rx->output_buffer_real[i] & (1 << j)) == 0 ? 0 : 1);
        }
        for(int j = ((ofdm->bits_per_qam / 2) - 1); j >= 0; --j)
        {
            bitstream_writer_write_bit(&writer, (ofdm->rx->output_buffer_imag[i] & (1 << j)) == 0 ? 0 : 1);
        }
    }

    if(ofdm->flags & OFDM_ALLOW_SCRAMBLE)
        ofdm->rx->scrambling_state = ofdm_scrambling_alg(ofdm->rx->descrambled_data, ofdm->rx->symbol_output_buffer, ofdm->bytes_per_symbol, ofdm->rx->scrambling_state);

    if(ofdm->rx->data_length == 0)
    {
#if OFDM_TXRX_MAX_PACKET_BYTES >= (1u << 16)
        ofdm->rx->data_length |= ((unsigned)ofdm->rx->descrambled_data[ofdm->bytes_per_symbol - 2] << 8);
#endif
        ofdm->rx->data_length |= (unsigned)ofdm->rx->descrambled_data[ofdm->bytes_per_symbol - 1];
        ofdm->rx->data_length_done = 0;

        ofdm->rx->expect_checksum = \
            ((unsigned)ofdm->rx->descrambled_data[0] << 24)
            | ((unsigned)ofdm->rx->descrambled_data[1] << 16)
            | ((unsigned)ofdm->rx->descrambled_data[2] << 8)
            | ((unsigned)ofdm->rx->descrambled_data[3]);
        ofdm->rx->checksum = 0;

        out = OFDM_TXRX_NEW_PACKET;
    }
    else
    {
        const int remaining = ofdm->rx->data_length - ofdm->rx->data_length_done + 1;
        const int remaining_block = remaining >= ofdm->bytes_per_symbol ? ofdm->bytes_per_symbol : remaining;
        if(ofdm->rx->data_output_buffer)
        {
            memcpy(
                &ofdm->rx->data_output_buffer[ofdm->rx->data_length_done],
                ofdm->rx->descrambled_data,
                remaining_block
            );
        }
        ofdm->rx->checksum = crc32(ofdm->rx->descrambled_data, remaining_block, ofdm->rx->checksum);
        ofdm->rx->data_length_done += remaining_block;

        if(ofdm->rx->data_length_done > ofdm->rx->data_length)
        {
            out = ofdm->rx->checksum == ofdm->rx->expect_checksum ? OFDM_TXRX_PACKET_COMPLETE_SUCCESS : OFDM_TXRX_PACKET_COMPLETE_FAILURE;
            // for(unsigned i = 0; i < data_length; ++i)
            // {
            //     printf("%02x", data_buffer[i]);
            // }
            // printf("\n");
            ofdm->rx->data_length = 0;
            ofdm->rx->data_length_done = 0;
            ofdm->rx->disabled = OFDM_TXRX_NFFT * 2; // minimum time between packets
            ofdm->rx->initial = 1.0f + I * 0.0f;
            ofdm->rx->locked = false;
        }
    }

    return out;
}

size_t ofdm_get_packet_recv_size(ofdm_t ofdm)
{
    if(!ofdm || !ofdm->rx)
        return 0;

    return ofdm->rx->data_length + 1;
}
size_t ofdm_get_packet_recv_size_remaining(ofdm_t ofdm)
{
    if(!ofdm || !ofdm->rx)
        return 0;

    return ofdm->rx->data_length - ofdm->rx->data_length_done;
}
int ofdm_get_packet_recv_checksums(ofdm_t ofdm, uint32_t* computed, uint32_t* expected)
{
    if(!ofdm || !ofdm->rx)
        return -1;
    
    if(computed)
        *computed = ofdm->rx->checksum;
    if(expected)
        *expected = ofdm->rx->expect_checksum;

    return 0;
}

int ofdm_set_packet_recv_buffer(ofdm_t ofdm, unsigned char* buffer)
{
    if(!ofdm || !ofdm->rx)
        return -1;

    ofdm->rx->data_output_buffer = buffer;
    return 0;
}

size_t ofdm_get_packet_send_length(ofdm_t ofdm, size_t data_length)
{
    if(!ofdm || !ofdm->tx)
        return 0;

    const ldiv_t div_res = ldiv(data_length, ofdm->bytes_per_symbol);
    const size_t num_chunks = div_res.quot + (div_res.rem ? 1 : 0);

    // 2 header/pilots, 1 checksum and length, num_chunks data, 2 footer/guard
    return OFDM_TXRX_NFFT * 2 + (1 + num_chunks) * OFDM_TXRX_NFFT * 5 / 4 + OFDM_TXRX_NFFT * 2;
}

// data of length size ofdm->bytes_per_symbol
static uint8_t ofdm_write_samples_chunk(ofdm_t ofdm, uint8_t state, const unsigned char* data, ofdm_write_samples_cb_t callback, void* user_data)
{
    struct bitstream_reader_t reader;
    if(ofdm->flags & OFDM_ALLOW_SCRAMBLE)
    {
        state = ofdm_scrambling_alg(ofdm->tx->scrambled_data, data, ofdm->bytes_per_symbol, state);
        data = ofdm->tx->scrambled_data;
    }
    
    bitstream_reader_init(&reader, data);
    for(int i = 0; i < ofdm->bytes_per_symbol; ++i)
    {
        int index_real = bitstream_reader_read_u64_bits(&reader, ofdm->bits_per_qam / 2);
        int index_imag = bitstream_reader_read_u64_bits(&reader, ofdm->bits_per_qam / 2);
        // printf("index real, imag: %d %d\n", index_real, index_imag);
        ofdm->tx->work_buffer[ofdm->carrier_indices[i]] = ofdm->mapping[index_real] + I * ofdm->mapping[index_imag];
    }
    kiss_fftri(ofdm->tx->cfg_send, (const kiss_fft_cpx*)ofdm->tx->work_buffer, ofdm->tx->samples_buffer);
    for(int i = 0; i < OFDM_TXRX_NFFT; ++i)
    {
        ofdm->tx->samples_buffer[i] *= ofdm->tx->scaler;
    }

    // cyclic prefix
    callback(&ofdm->tx->samples_buffer[OFDM_TXRX_NFFT * 3 / 4], OFDM_TXRX_NFFT / 4, user_data);
    callback(ofdm->tx->samples_buffer, OFDM_TXRX_NFFT, user_data);
    return state;
}

int ofdm_write_samples(ofdm_t ofdm, const unsigned char* data, size_t data_length, ofdm_write_samples_cb_t callback, void* user_data)
{
    if(!ofdm || !ofdm->tx)
        return -1;

    const ldiv_t div_res = ldiv(data_length, ofdm->bytes_per_symbol);
    callback(ofdm->expect_header, OFDM_TXRX_NFFT, user_data);
    callback(ofdm->expect_header, OFDM_TXRX_NFFT, user_data);
    const uint32_t checksum = crc32(data, data_length, 0);
    memset(ofdm->tx->meta_buffer, 0, ofdm->bytes_per_symbol);
    ofdm->tx->meta_buffer[0] = (checksum >> 24) & 0xff;
    ofdm->tx->meta_buffer[1] = (checksum >> 16) & 0xff;
    ofdm->tx->meta_buffer[2] = (checksum >> 8) & 0xff;
    ofdm->tx->meta_buffer[3] = (checksum) & 0xff;
    data_length -= 1;
#if OFDM_TXRX_MAX_PACKET_BYTES >= (1u << 16)
    ofdm->tx->meta_buffer[ofdm->bytes_per_symbol - 2] = (data_length >> 8) & 0xff;
#endif
    ofdm->tx->meta_buffer[ofdm->bytes_per_symbol - 1] = (data_length) & 0xff;
    uint8_t state = ofdm_write_samples_chunk(ofdm, OFDM_SCRAMBLE_INIT_STATE, ofdm->tx->meta_buffer, callback, user_data);
    
    long j = 0;
    for(long i = 0; i < div_res.quot; ++i, j += ofdm->bytes_per_symbol)
    {
        state = ofdm_write_samples_chunk(ofdm, state, &data[j], callback, user_data);
    }

    if(div_res.rem)
    {
        memset(ofdm->tx->meta_buffer, 0, ofdm->bytes_per_symbol);
        memcpy(ofdm->tx->meta_buffer, &data[j], div_res.rem);
        state = ofdm_write_samples_chunk(ofdm, state, ofdm->tx->meta_buffer, callback, user_data);
    }

    memset(ofdm->tx->samples_buffer, 0, sizeof(ofdm->tx->samples_buffer));
    callback(ofdm->tx->samples_buffer, OFDM_TXRX_NFFT, user_data);
    callback(ofdm->tx->samples_buffer, OFDM_TXRX_NFFT, user_data);

    return 0;
}