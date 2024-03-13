#include <3ds/asminc.h>

.arm
.align 4
.fpu vfpv2

BEGIN_ASM_FUNC ofdm_fft_simd_start
    vstm r0, {s16-s31}
    vmrs r0, fpscr

    orr r1, r0, #0x00010000
    vmsr fpscr, r1

    bx lr
END_ASM_FUNC

BEGIN_ASM_FUNC ofdm_fft_simd_end
    vmsr fpscr, r0
    vldm r1, {s16-s31}

    bx lr
END_ASM_FUNC

@ void ofdm_fft_simd_compute(const float complex twiddle, float complex* src_ata_a0, float complex* src_data_a1, float complex* src_data_end, int step_bytes)
BEGIN_ASM_FUNC ofdm_fft_simd_compute

_continue_looping:

@ load iteration complex numbers
    @ s16 = a1_re
    @ s17 = a1_im
    vldm.f32 r1, {s16-s17}

    @ s8 = a0_re
    @ s9 = a0_im
    vldm.f32 r0, {s8-s9}

    @ s18 = -a1_re
    @ s19 = -a1_im
    vneg.f32 s18, s16

@ perform complex multiply
    @ (A + Bi)(C + Di) = (AC - BD) + (AD + BC)i
    @ aka (AC - BD) + (BC + AD)i

    @ tmpA_re <- A * C = a1_re * re_factor
    @ tmpA_im <- B * C = a1_im * re_factor
    vmul.f32 s24, s16, s0

    @ tmpA_re <- tmpA_re -  B * D = tmpA_re - a1_im * im_factor
    @ tmpA_im <- tmpA_im - -A * D = tmpA_im + a1_re * im_factor
    vmls.f32 s24, s17, s1

@ final fft calculation
    @ a0_out <- a0 + factor * a1
    @ a1_out <- a0 - factor * a1
    vadd.f32 s10, s8, s24
    vsub.f32 s12, s8, s24

@ writeback & update iterators & loop
    vstm.f32 r0, {s10-s11}
    add r0, r3

    vstm.f32 r1, {s12-s13}

    cmp r0, r2
    add r1, r3

    bxhs lr

    b _continue_looping

END_ASM_FUNC
