#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "ofdm.h"
#define DR_WAV_IMPLEMENTATION
#define DR_WAV_NO_WCHAR
#include "dr_wav.h"

int main(int argc,char ** argv)
{
    const char* in_filename = NULL;
    const char* out_filename = NULL;
    const char* out_data_filename = NULL;
    bool want_scramble = false;
    bool want_print_text = false;
    bool want_print_hex = false;
    bool want_meta_check = false;
    int BITS_PER_QAM = 0;
    bool print_help = argc == 1;

    for(int i = 1; i < argc; ++i)
    {
        if(strcmp(argv[i], "-h") == 0)
            print_help = true;
        else
        if(strcmp(argv[i], "-s") == 0)
            want_scramble = true;
        else
        if(strcmp(argv[i], "-m") == 0)
            want_meta_check = true;
        else
        if(strcmp(argv[i], "-p") == 0)
            want_print_text = true;
        else
        if(strcmp(argv[i], "-x") == 0)
            want_print_hex = true;
        else
        if(strncmp(argv[i], "-q=", 3) == 0)
            BITS_PER_QAM = strtol(argv[i] + 3, NULL, 10);
        else
        if(strcmp(argv[i], "-f") == 0)
        {
            ++i;
            if(i >= argc)
                return EXIT_FAILURE;
            in_filename = argv[i];
        }
        else
        if(strcmp(argv[i], "-o") == 0)
        {
            ++i;
            if(i >= argc)
                return EXIT_FAILURE;
            out_filename = argv[i];
        }
        else
        if(strcmp(argv[i], "-d") == 0)
        {
            ++i;
            if(i >= argc)
                return EXIT_FAILURE;
            out_data_filename = argv[i];
        }
    }

    if(print_help)
    {
        printf( "Usage: %s [-s] [-m] [-p] [-x] -q=<bits per QAM>"
                " [-f <input wav path>]"
                " [-o <output wav path>]"
                " [-d <output data source path>]"
                "\n\n"
                "-s: enable scrambling\n"
                "-m: enable edge case check\n"
                "-p: print data as text\n"
                "-x: print data as hex\n"
                "-q: >= 2, even number\n",
                argv[0]);
        return EXIT_SUCCESS;
    }

    if(BITS_PER_QAM == 0 || (BITS_PER_QAM & 1) == 1)
        return EXIT_FAILURE;

    drwav wav;
    if(in_filename)
    {
        printf("Reading %s\n", in_filename);
        if (!drwav_init_file(&wav, in_filename, NULL))
            return EXIT_FAILURE;

        float* samples_buffer = (float*)calloc(wav.totalPCMFrameCount + OFDM_TXRX_NFFT * 2, sizeof(float));
        const unsigned long long numSamples = drwav_read_pcm_frames_f32(&wav, wav.totalPCMFrameCount, &samples_buffer[OFDM_TXRX_NFFT]);
        drwav_uninit(&wav);

        printf("NFFT: %d, BITS_PER_QAM: %d, samps: %llu\n", OFDM_TXRX_NFFT, BITS_PER_QAM, numSamples);
        unsigned char* data_buffer = malloc(OFDM_TXRX_MAX_PACKET_BYTES);
        const int max_loops = want_meta_check ? OFDM_TXRX_NFFT : 1;
        for(int i = 0; i < max_loops; ++i) // tests at every offset to ensure no edge cases (when enabled)
        {
            if(want_meta_check)
                printf("Meta index: %d\n", i);
            ofdm_t ofdm = ofdm_new(BITS_PER_QAM, OFDM_ALLOW_DIRECTION | (want_scramble ? OFDM_ALLOW_SCRAMBLE : 0));
            if(!ofdm)
                return EXIT_FAILURE;

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
                    if(want_print_hex)
                    {
                        for(size_t k = 0; k < packet_len; ++k)
                            printf("%02x", data_buffer[k]);
                        printf("\n");
                    }
                    if(want_print_text)
                    {
                        fwrite(data_buffer, 1, packet_len, stdout);
                        printf("\n");
                    }
                    packet_len = 0;
                    break;
                default:
                    break;
                }
            }

            ofdm_delete(ofdm);
        }
        free(data_buffer);
    }

    if(out_filename && out_data_filename)
    {
        drwav_data_format format = {
            .container = drwav_container_riff,
            .format = DR_WAVE_FORMAT_IEEE_FLOAT,
            .channels = 1,
            .sampleRate = 44100,
            .bitsPerSample = 32,
        };
        if (!drwav_init_file_write(&wav, out_filename, &format, NULL))
            return EXIT_FAILURE;

        ofdm_t ofdm = ofdm_new(BITS_PER_QAM, OFDM_ALLOW_DIRECTION | (want_scramble ? OFDM_ALLOW_SCRAMBLE : 0));
        if(!ofdm)
            return EXIT_FAILURE;

        unsigned char* send_data = calloc(OFDM_TXRX_MAX_PACKET_BYTES, 1);
        size_t samples_output_length = ofdm_get_packet_send_length(ofdm, OFDM_TXRX_MAX_PACKET_BYTES);
        printf("Outputting %zd samples\n", samples_output_length);
        float* samples_output = malloc(samples_output_length * sizeof(float));

        FILE* fh = fopen(out_data_filename,"rb");
        fseek(fh, 0, SEEK_END);
        long sz = ftell(fh);
        fseek(fh, 0, SEEK_SET);
        ldiv_t divided = ldiv(sz, OFDM_TXRX_MAX_PACKET_BYTES);
        for(long i = 0; i < divided.quot; ++i)
        {
            fread(send_data, 1, OFDM_TXRX_MAX_PACKET_BYTES, fh);
            ofdm_write_samples(ofdm, send_data, OFDM_TXRX_MAX_PACKET_BYTES, ofdm_callback_write_f32_samples, &(ofdm_samples_t){0, samples_output});
            drwav_write_pcm_frames(&wav, samples_output_length, samples_output);
        }
        if(divided.rem)
        {
            fread(send_data, 1, divided.rem, fh);
            samples_output_length = ofdm_get_packet_send_length(ofdm, divided.rem);
            ofdm_write_samples(ofdm, send_data, divided.rem, ofdm_callback_write_f32_samples, &(ofdm_samples_t){0, samples_output});
            drwav_write_pcm_frames(&wav, samples_output_length, samples_output);
        }

        fclose(fh);
        free(send_data);
        free(samples_output);
        send_data = NULL;
        samples_output = NULL;
        samples_output_length = 0;

        drwav_uninit(&wav);
        ofdm_delete(ofdm);
    }


    return 0;
}
