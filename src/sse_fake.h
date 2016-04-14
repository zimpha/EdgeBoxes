#ifndef _SSE_FAKE_H
#define _SSE_FAKE_H

#include <cmath>

float min(float x, float y) {return x < y ? x : y;}

struct __mm128 {
  float a, b, c, d;
};

struct __mm128i {
  int a, b, c, d;
};

// set, load and store values
__mm128 __mm_set1_ps(float w) {
  return (__mm128){w, w, w, w};
}

__mm128 __mm_set_ps(float z, float y, float x, float w) {
  return (__mm128){w, x, y, z};
}

__mm128i __mm_set1_epi32(int i) {
  return (__mm128i){i, i, i, i};
}

__mm128 __mm_load_ps(float *p) {
  return (__mm128){p[0], p[1], p[2], p[3]};
}

__mm128 __mm_loadu_ps(float *p) {
  return (__mm128){p[0], p[1], p[2], p[3]};
}

void __mm_store_ps(float *p, __mm128 a) {
  p[0] = a.a;
  p[1] = a.b;
  p[2] = a.c;
  p[3] = a.d;
}

void __mm_storeu_ps(float *p, __mm128 a) {
  p[0] = a.a;
  p[1] = a.b;
  p[2] = a.c;
  p[3] = a.d;
}

// arithmetic operators
__mm128i __mm_add_epi32(const __mm128i &a, const __mm128i &b) {
  return (__mm128i){a.a + b.a, a.b + b.b, a.c + b.c, a.d + b.d};
}

__mm128 __mm_add_ps(const __mm128 &a, const __mm128 &b) {
  return (__mm128){a.a + b.a, a.b + b.b, a.c + b.c, a.d + b.d};
}

__mm128 __mm_sub_ps(const __mm128 &a, const __mm128 &b) {
  return (__mm128){a.a - b.a, a.b - b.b, a.c - b.c, a.d - b.d};
}

__mm128 __mm_mul_ps(const __mm128 &a, const __mm128 &b) {
  return (__mm128){a.a * b.a, a.b * b.b, a.c * b.c, a.d * b.d};
}

__mm128 __mm_rcp_ps(const __mm128 &a) {
  return (__mm128){1.f / a.a, 1.f / a.b, 1.f / a.c, 1.f / a.d};
}

__mm128 __mm_rsqrt_ps(const __mm128 &a) {
  return (__mm128){1.f / sqrt(a.a), 1.f / sqrt(a.b), 1.f / sqrt(a.c), 1.f / sqrt(a.d)};
}

__mm128 __mm_min_ps(const __mm128 &a, const __mm128 &b) {
  return (__mm128){min(a.a, b.a), min(a.b, b.b), min(a.c, b.c), min(a.d, b.d)};
}

// logical operators
__mm128 __mm_and_ps(const __mm128 &a, const __mm128 &b) {
  return (__mm128){a.a & b.a, a.b & b.b, a.c & b.c, a.d & b.d};
}

__mm128i __mm_and_si128(const __mm128i &a, const __mm128i &b) {
  return (__mm128i){a.a & b.a, a.b & b.b, a.c & b.c, a.d & b.d};
}

__mm128 __mm_andnot_ps(const __mm128 &a, const __mm128 &b) {
  return (__mm128){~a.a & b.a, ~a.b & b.b, ~a.c & b.c, ~a.d & b.d};
}

__mm128 __mm_or_ps(const __mm128 &a, const __mm128 &b) {
  return (__mm128){a.a | b.a, a.b | b.b, a.c | b.c, a.d | b.d};
}

__mm128 __mm_xor_ps(const __mm128 &a, const __mm128 &b) {
  return (__mm128){a.a ^ b.a, a.b ^ b.b, a.c ^ b.c, a.d ^ b.d};
}

// comparison operators
__mm128 __mm_cmpgt_ps(const __mm128 &a, const __mm128 &b) {
  return (__mm128){
    a.a > b.a ? 0xffffffff : 0x0,
    a.b > b.b ? 0xffffffff : 0x0,
    a.c > b.c ? 0xffffffff : 0x0,
    a.d > b.d ? 0xffffffff : 0x0
  };
}

__mm128 __mm_cmplt_ps(const __mm128 &a, const __mm128 &b) {
  return (__mm128){
    a.a < b.a ? 0xffffffff : 0x0,
    a.b < b.b ? 0xffffffff : 0x0,
    a.c < b.c ? 0xffffffff : 0x0,
    a.d < b.d ? 0xffffffff : 0x0
  };
}

__mm128i __mm_cmpgt_epi32(const __mm128i &a, const __mm128i &b) {
  return (__mm128i){
    a.a > b.a ? 0xffffffff : 0x0,
    a.b > b.b ? 0xffffffff : 0x0,
    a.c > b.c ? 0xffffffff : 0x0,
    a.d > b.d ? 0xffffffff : 0x0
  };
}

__mm128i __mm_cmplt_epi32(const __mm128i &a, const __mm128i &b) {
  return (__mm128i){
    a.a < b.a ? 0xffffffff : 0x0,
    a.b < b.b ? 0xffffffff : 0x0,
    a.c < b.c ? 0xffffffff : 0x0,
    a.d < b.d ? 0xffffffff : 0x0
  };
}

// conversion operators
__mm128 __mm_cvtepi32_ps(const __mm128i &a) {
  return (__mm128){float(a.a), float(a.b), float(a.c), float(a.d)};
}

__mm128i __mm_cvttps_epi32(const __mm128 &a) {
  return (__mm128i){int(a.a), int(a.b), int(a.c), int(a.d)};
}

#endif
