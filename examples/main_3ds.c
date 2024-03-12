#include <3ds.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "ofdm.h"
#define DR_WAV_IMPLEMENTATION
#define DR_WAV_NO_WCHAR
#include "dr_wav.h"

static int read_and_handle_file(const char* in_filename, bool want_scramble)
{
    bool want_print_text = false;
    bool want_print_hex = false;
    bool want_meta_check = false;
    int BITS_PER_QAM = 8;

    drwav wav;
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

        TickCounter ctr;
        osTickCounterStart(&ctr);
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
        osTickCounterUpdate(&ctr);
        printf("Took: %f\n", osTickCounterRead(&ctr));

        ofdm_delete(ofdm);
    }
    free(data_buffer);
    return 0;
}

int main(int argc,char ** argv)
{
    const char* in_filenames[2] = {
        "/license_unscramble.wav",
        "/license_scramble.wav",
    };

    gfxInitDefault();
    consoleInit(GFX_TOP, NULL);
    // consoleDebugInit(debugDevice_SVC);

    printf("Hello, world!\n");
    printf("A for unscrambled\n");
    printf("X for scrambled\n");
    printf("lower keys for speedup\n");

    while(aptMainLoop())
    {
        gspWaitForVBlank();
        gfxSwapBuffers();
        hidScanInput();
        const u32 kDown = hidKeysDown();
        if(kDown & KEY_START)
            break;
        
        if(kDown & KEY_A)
            read_and_handle_file(in_filenames[0], false);
        if(kDown & KEY_X)
            read_and_handle_file(in_filenames[1], true);
        if(kDown & KEY_Y)
        {
            osSetSpeedupEnable(true);
            read_and_handle_file(in_filenames[1], true);
            osSetSpeedupEnable(false);
        }
        if(kDown & KEY_B)
        {
            osSetSpeedupEnable(true);
            read_and_handle_file(in_filenames[0], false);
            osSetSpeedupEnable(false);
        }
    }
    gfxExit();
    return 0;
}
