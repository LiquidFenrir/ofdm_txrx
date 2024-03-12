#include <3ds/asminc.h>

.arm
.align 4
.fpu vfpv2

BEGIN_ASM_FUNC ofdm_fft_simd_start
    vstm r0, {s0-s31}
    vmrs r0, fpscr

    orr r1, r0, #0x00030000
    vmsr fpscr, r1

    bx lr
END_ASM_FUNC

BEGIN_ASM_FUNC ofdm_fft_simd_end
    vmsr fpscr, r0
    vldm r1, {s0-s31}

    bx lr
END_ASM_FUNC

@ void ofdm_fft_simd_compute(float complex* a0_ptr, float complex* a1_ptr, const float twiddle[4])
BEGIN_ASM_FUNC ofdm_fft_simd_compute
    @ a1_re a1_im
    vldm.f32 r1, {s0, s1}

    @ a0_re a0_im a0_re a0_im
    vldm.f32 r0, {s8, s9}
    vldm.f32 r0, {s10, s11}

    @ re_factor im_factor -re_factor -im_factor, dontcare, dontcare, dontcare, -im_factor
    vldm.f32 r2, {s24-s27}
    vmov.f32 s28, s24

    @ make s16 full of a1_re
    vmov.f32 s16, s0
    @ make s20 full of a1_im
    vmov.f32 s20, s1

    @ (A + Bi)(C + Di) = (AC - BD) + (AD + BC)i
    @ a0_out <- a0 + factor * a1
    @ a1_out <- a0 - factor * a1

    @ a0_re + factor_re * a1_re - factor_im * a1_im
    @ a0_im + factor_im * a1_re + factor_re * a1_im
    @ a0_re - factor_re * a1_re + factor_im * a1_im
    @ a0_im - factor_im * a1_re - factor_re * a1_im
    vmla.f32 s8, s16, s24
    vmla.f32 s8, s20, s31

    vstm.f32 r0, {s8, s9}
    vstm.f32 r1, {s10, s11}

    bx lr
END_ASM_FUNC
