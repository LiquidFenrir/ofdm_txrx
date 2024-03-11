#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "ofdm.h"
#define DR_WAV_IMPLEMENTATION
#define DR_WAV_NO_WCHAR
#include "dr_wav.h"

int main(int argc,char ** argv)
{
    if(argc < 2)
        return EXIT_FAILURE;

    const char* str_end = argv[1] + strlen(argv[1]);
    str_end -= 7; /* "fft.wav" */
    const char* const fft_n_end = str_end;
    while(str_end > argv[1] && *str_end != '_')
        --str_end;

    if(str_end == argv[1])
        return EXIT_FAILURE;

    /*
    const char* fft_n_end_got = NULL;
    const int nfft = strtol(str_end + 1, &fft_n_end_got, 10);
    if(fft_n_end != fft_n_end_got)
        return EXIT_FAILURE;
    */
   const int nfft = 128;

    str_end -= 3;
    const char* const qam_n_end = str_end;
    while(str_end > argv[1] && *str_end != '_')
        --str_end;

    if(str_end == argv[1])
        return EXIT_FAILURE;

    const char* qam_n_end_got = NULL;
    const int BITS_PER_QAM = strtol(str_end + 1, &qam_n_end_got, 10);
    if(qam_n_end != qam_n_end_got)
        return EXIT_FAILURE;

    drwav wav;
    if (!drwav_init_file(&wav, argv[1], NULL))
        return EXIT_FAILURE;

    float* samples_buffer = (float*)calloc(wav.totalPCMFrameCount + OFDM_TXRX_NFFT * 2, sizeof(float));
    unsigned long long numSamples = drwav_read_pcm_frames_f32(&wav, wav.totalPCMFrameCount, &samples_buffer[OFDM_TXRX_NFFT]);
    drwav_uninit(&wav);

    printf("NFFT: %d, BITS_PER_QAM: %d, samps: %llu\n", OFDM_TXRX_NFFT, BITS_PER_QAM, numSamples);
    unsigned char* data_buffer = malloc(OFDM_TXRX_MAX_PACKET_BYTES);
    for(int i = 0; i < OFDM_TXRX_NFFT; ++i)
    // for(int i = 0; i < 1; ++i)
    // int i = 0;
    {
        printf("Meta index: %d\n", i);
        ofdm_t ofdm = ofdm_new(BITS_PER_QAM, OFDM_ALLOW_ALL);
        ofdm_set_packet_recv_buffer(ofdm, data_buffer);

        size_t packet_len = 0;
        for(unsigned long long j = 0; j < numSamples; j += OFDM_TXRX_NFFT)
        {
            float* samples = &samples_buffer[i + j];
            ofdm_txrx_state_t info = ofdm_read_samples(ofdm, ofdm_callback_read_f32_samples, samples);
            uint32_t checksum = 0, expected = 0;
            switch(info)
            {
            case OFDM_TXRX_NEW_PACKET:
                packet_len = ofdm_get_packet_recv_size(ofdm);
                printf("Have new packet: %5zd\n", packet_len);
                break;
            case OFDM_TXRX_PACKET_COMPLETE_FAILURE:
                ofdm_get_packet_recv_checksums(ofdm, &checksum, &expected);
                printf("Fail packet, %5zd checksum: 0x%08lx (wanted 0x%08lx)\n", packet_len, (unsigned long)checksum, (unsigned long)expected);
                packet_len = 0;
                break;
            case OFDM_TXRX_PACKET_COMPLETE_SUCCESS:
                ofdm_get_packet_recv_checksums(ofdm, &checksum, &expected);
                printf("Have packet, %5zd checksum: 0x%08lx (wanted 0x%08lx)\n", packet_len, (unsigned long)checksum, (unsigned long)expected);
                // for(size_t k = 0; k < packet_len; ++k)
                //     printf("%02x", data_buffer[k]);
                // printf("\n");
                packet_len = 0;
                break;
            default:
                break;
            }
        }

        if(argv[2])
        {
            const unsigned char send_data[] = "Hello, world!\nazertyuiopazertyuiop\nazertyuiopazertyuiop\nazertyuiopazertyuiop\nazertyuiopazertyuiop\nazertyuiopazertyuiop\n";
            const size_t samples_output_length = ofdm_get_packet_send_length(ofdm, strlen(send_data));
            printf("Outputting %zd samples\n", samples_output_length);
            int16_t* samples_output = malloc(samples_output_length * sizeof(int16_t));
            ofdm_write_samples(ofdm, send_data, strlen(send_data), ofdm_callback_write_pcm16_samples, &(ofdm_samples_t){0, samples_output});

            drwav_data_format format = {
                .container = drwav_container_riff,
                .format = DR_WAVE_FORMAT_PCM,
                .channels = 1,
                .sampleRate = 44100,
                .bitsPerSample = 16,
            };
            if (!drwav_init_file_write(&wav, argv[2], &format, NULL))
                return EXIT_FAILURE;

            drwav_write_pcm_frames(&wav, samples_output_length, samples_output);
            drwav_uninit(&wav);

        }

        ofdm_delete(ofdm);
    }
    free(data_buffer);

    return 0;
}
