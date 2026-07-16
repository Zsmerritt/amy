// amy_blockops.h -- length-parameterized SAMPLE block ops with ESP32-S3
// PIE (128-bit vector) fast paths (OPT-4). The idiom follows algorithms.c's
// zero()/copy() (see the provenance note there): vector path only for
// 16-byte-aligned pointers and sizes, scalar/libc fallback otherwise, so
// callers never need their own alignment guarantees. fbl/per_osc_fb are
// allocated with malloc_caps_block so the guards pass in practice.
#ifndef __AMY_BLOCKOPS_H
#define __AMY_BLOCKOPS_H

#include <string.h>
#include <strings.h>

static inline void amy_block_zero(SAMPLE *a, size_t n_samples) {
    const size_t nbytes = n_samples * sizeof(SAMPLE);
#if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32S3)
    if ((((uintptr_t)a | nbytes) & 15u) == 0) {
        void *pp = a;
        __asm__ volatile(
            "ee.xorq q0, q0, q0\n\t"
            "loopnez %1, .Lamy_bzero%=\n\t"
            "ee.vst.128.ip q0, %0, 16\n"
            ".Lamy_bzero%=:"
            : "+&r"(pp)
            : "r"(nbytes / 16)
            : "memory");
        return;
    }
#endif
    bzero((void *)a, nbytes);
}

static inline void amy_block_copy(const SAMPLE *src, SAMPLE *dst, size_t n_samples) {
    const size_t nbytes = n_samples * sizeof(SAMPLE);
#if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32S3)
    if ((((uintptr_t)src | (uintptr_t)dst | nbytes) & 15u) == 0) {
        const void *s = src;
        void *d = dst;
        __asm__ volatile(
            "loopnez %2, .Lamy_bcopy%=\n\t"
            "ee.vld.128.ip q0, %1, 16\n\t"
            "ee.vst.128.ip q0, %0, 16\n"
            ".Lamy_bcopy%=:"
            : "+&r"(d), "+&r"(s)
            : "r"(nbytes / 16)
            : "memory");
        return;
    }
#endif
    memcpy((void *)dst, (const void *)src, nbytes);
}

// dst += src. The PIE add (ee.vadds.s32) SATURATES where the scalar loop
// would wrap -- indistinguishable except at >0dBFS-by-256x signal levels,
// where saturation is the better answer anyway (s8.23 samples sit ~8 bits
// below int32 full scale).
static inline void amy_block_add(SAMPLE *dst, const SAMPLE *src, size_t n_samples) {
    const size_t nbytes = n_samples * sizeof(SAMPLE);
#if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32S3)
    if ((((uintptr_t)src | (uintptr_t)dst | nbytes) & 15u) == 0) {
        const void *s = src;
        const void *dr = dst;   // read cursor
        void *dw = dst;         // write cursor
        __asm__ volatile(
            "loopnez %3, .Lamy_badd%=\n\t"
            "ee.vld.128.ip q0, %1, 16\n\t"
            "ee.vld.128.ip q1, %2, 16\n\t"
            "ee.vadds.s32 q2, q0, q1\n\t"
            "ee.vst.128.ip q2, %0, 16\n"
            ".Lamy_badd%=:"
            : "+&r"(dw), "+&r"(s), "+&r"(dr)
            : "r"(nbytes / 16)
            : "memory");
        return;
    }
#endif
    for (size_t i = 0; i < n_samples; i++) dst[i] += src[i];
}

#endif
