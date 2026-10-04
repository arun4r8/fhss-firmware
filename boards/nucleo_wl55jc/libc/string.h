/* Minimal string.h for the freestanding build (Homebrew's arm-none-eabi-gcc
 * ships without a C library). Implementations in ../libc_min.c. */
#ifndef LIBC_MIN_STRING_H
#define LIBC_MIN_STRING_H

#include <stddef.h>

void *memset(void *dst, int c, size_t n);
void *memcpy(void *dst, const void *src, size_t n);
void *memmove(void *dst, const void *src, size_t n);
int memcmp(const void *a, const void *b, size_t n);
size_t strlen(const char *s);

#endif
