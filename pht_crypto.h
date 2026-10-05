/*
 * pht_crypto.h —— 自带的 SHA-256 + HMAC-SHA256（设备一机一密用）
 * ============================================================================
 *
 * 为什么自己写而不是用 mbedtls：
 *   ESP-IDF 确实带 mbedtls（`mbedtls/md.h` 可用），但这个项目一直刻意
 *   少引组件 —— 多一个头文件就多一处"换 core 版本时编译不过"的风险。
 *   SHA-256 是**完全确定**的算法（FIPS 180-4），实现一次就再也不用动，
 *   代码量也小。宁可控在手里。
 *
 * ⚠️ 与服务器 PHT_SERVER/devicekey.py 必须逐字节一致：
 *     待签串 = "v1|" + device_id + "|" + ts + "|" + body
 *     签名   = hex( HMAC-SHA256(secret, 待签串) )
 *     头     = X-PHT-Auth: v1|<device_id>|<ts>|<hex 签名>
 *   任何一边改了拼接方式，另一边必须同步改，否则全线 401。
 *
 * 为什么必须是 HMAC 而不是 SHA256(secret + ts)：
 *   后者可以**离线枚举反推密钥**（前缀已知的哈希是可暴力搜索的）。
 *   HMAC 的结构（两次带密钥的哈希 + ipad/opad）没有这个弱点。
 * ============================================================================
 */
#ifndef PHT_CRYPTO_H
#define PHT_CRYPTO_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// SHA-256（FIPS 180-4）
// ---------------------------------------------------------------------------
typedef struct {
  uint32_t h[8];
  uint64_t len;          // 已处理字节数
  uint8_t  buf[64];
  size_t   buflen;
} pht_sha256_ctx;

static const uint32_t PHT_SHA256_K[64] = {
  0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
  0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
  0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
  0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
  0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
  0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
  0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
  0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2
};

static inline uint32_t pht_rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

static void pht_sha256_block(pht_sha256_ctx* c, const uint8_t* p) {
  uint32_t w[64];
  for (int i = 0; i < 16; i++) {
    w[i] = ((uint32_t)p[i*4] << 24) | ((uint32_t)p[i*4+1] << 16) |
           ((uint32_t)p[i*4+2] << 8) | (uint32_t)p[i*4+3];
  }
  for (int i = 16; i < 64; i++) {
    uint32_t s0 = pht_rotr(w[i-15], 7) ^ pht_rotr(w[i-15], 18) ^ (w[i-15] >> 3);
    uint32_t s1 = pht_rotr(w[i-2], 17) ^ pht_rotr(w[i-2], 19) ^ (w[i-2] >> 10);
    w[i] = w[i-16] + s0 + w[i-7] + s1;
  }
  uint32_t a=c->h[0], b=c->h[1], cc=c->h[2], d=c->h[3];
  uint32_t e=c->h[4], f=c->h[5], g=c->h[6], h=c->h[7];
  for (int i = 0; i < 64; i++) {
    uint32_t S1 = pht_rotr(e,6) ^ pht_rotr(e,11) ^ pht_rotr(e,25);
    uint32_t ch = (e & f) ^ (~e & g);
    uint32_t t1 = h + S1 + ch + PHT_SHA256_K[i] + w[i];
    uint32_t S0 = pht_rotr(a,2) ^ pht_rotr(a,13) ^ pht_rotr(a,22);
    uint32_t mj = (a & b) ^ (a & cc) ^ (b & cc);
    uint32_t t2 = S0 + mj;
    h=g; g=f; f=e; e=d+t1; d=cc; cc=b; b=a; a=t1+t2;
  }
  c->h[0]+=a; c->h[1]+=b; c->h[2]+=cc; c->h[3]+=d;
  c->h[4]+=e; c->h[5]+=f; c->h[6]+=g; c->h[7]+=h;
}

static void pht_sha256_init(pht_sha256_ctx* c) {
  c->h[0]=0x6a09e667; c->h[1]=0xbb67ae85; c->h[2]=0x3c6ef372; c->h[3]=0xa54ff53a;
  c->h[4]=0x510e527f; c->h[5]=0x9b05688c; c->h[6]=0x1f83d9ab; c->h[7]=0x5be0cd19;
  c->len = 0; c->buflen = 0;
}

static void pht_sha256_update(pht_sha256_ctx* c, const void* data, size_t n) {
  const uint8_t* p = (const uint8_t*)data;
  c->len += n;
  while (n > 0) {
    size_t take = 64 - c->buflen;
    if (take > n) take = n;
    memcpy(c->buf + c->buflen, p, take);
    c->buflen += take; p += take; n -= take;
    if (c->buflen == 64) { pht_sha256_block(c, c->buf); c->buflen = 0; }
  }
}

static void pht_sha256_final(pht_sha256_ctx* c, uint8_t out[32]) {
  uint64_t bits = c->len * 8;
  uint8_t pad = 0x80;
  pht_sha256_update(c, &pad, 1);
  uint8_t zero = 0x00;
  while (c->buflen != 56) pht_sha256_update(c, &zero, 1);
  uint8_t lenb[8];
  for (int i = 0; i < 8; i++) lenb[i] = (uint8_t)(bits >> (56 - i*8));
  // ⚠️ 上面 update 会改 c->len，但长度字节用的是**之前存下的 bits**，所以没问题
  pht_sha256_update(c, lenb, 8);
  for (int i = 0; i < 8; i++) {
    out[i*4]   = (uint8_t)(c->h[i] >> 24);
    out[i*4+1] = (uint8_t)(c->h[i] >> 16);
    out[i*4+2] = (uint8_t)(c->h[i] >> 8);
    out[i*4+3] = (uint8_t)(c->h[i]);
  }
}

// ---------------------------------------------------------------------------
// HMAC-SHA256（RFC 2104）
// ---------------------------------------------------------------------------
static void pht_hmac_sha256(const uint8_t* key, size_t keylen,
                            const void* msg, size_t msglen,
                            uint8_t out[32]) {
  uint8_t k[64];
  memset(k, 0, sizeof(k));
  if (keylen > 64) {
    pht_sha256_ctx t; pht_sha256_init(&t);
    pht_sha256_update(&t, key, keylen);
    pht_sha256_final(&t, k);          // 超长密钥先哈希
  } else {
    memcpy(k, key, keylen);
  }

  uint8_t ipad[64], opad[64];
  for (int i = 0; i < 64; i++) { ipad[i] = k[i] ^ 0x36; opad[i] = k[i] ^ 0x5c; }

  uint8_t inner[32];
  pht_sha256_ctx c;
  pht_sha256_init(&c);
  pht_sha256_update(&c, ipad, 64);
  pht_sha256_update(&c, msg, msglen);
  pht_sha256_final(&c, inner);

  pht_sha256_init(&c);
  pht_sha256_update(&c, opad, 64);
  pht_sha256_update(&c, inner, 32);
  pht_sha256_final(&c, out);

  // 清掉中间态（里面有密钥派生材料）
  memset(k, 0, sizeof(k));
  memset(ipad, 0, sizeof(ipad));
  memset(opad, 0, sizeof(opad));
  memset(inner, 0, sizeof(inner));
}

static void pht_hex(const uint8_t* in, size_t n, char* out) {
  static const char* H = "0123456789abcdef";
  for (size_t i = 0; i < n; i++) { out[i*2] = H[in[i] >> 4]; out[i*2+1] = H[in[i] & 15]; }
  out[n*2] = '\0';
}

// ---------------------------------------------------------------------------
// 十六进制工具
// ---------------------------------------------------------------------------
static int pht_hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// 把 hex 字符串解成字节；长度必须是 2 的倍数且全为 hex，否则返回 false
static bool pht_unhex(const char* s, uint8_t* out, size_t outlen) {
  if (!s || strlen(s) != outlen * 2) return false;
  for (size_t i = 0; i < outlen; i++) {
    int hi = pht_hexval(s[i*2]), lo = pht_hexval(s[i*2+1]);
    if (hi < 0 || lo < 0) return false;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  return true;
}

// 校验：是否恰好 len 个 hex 字符
static bool pht_is_hex(const char* s, size_t len) {
  if (!s || strlen(s) != len) return false;
  for (size_t i = 0; i < len; i++) if (pht_hexval(s[i]) < 0) return false;
  return true;
}

// ---------------------------------------------------------------------------
// 自检：SHA-256 / HMAC 的已知测试向量
//   开机跑一次（几十微秒），防止"算法写错但没发现"——
//   这类错误的表现是全线 401，排查起来很费劲。
// ---------------------------------------------------------------------------
static bool pht_crypto_selftest() {
  // SHA-256("abc")
  static const uint8_t want_abc[32] = {
    0xba,0x78,0x16,0xbf,0x8f,0x01,0xcf,0xea,0x41,0x41,0x40,0xde,0x5d,0xae,0x22,0x23,
    0xb0,0x03,0x61,0xa3,0x96,0x17,0x7a,0x9c,0xb4,0x10,0xff,0x61,0xf2,0x00,0x15,0xad
  };
  uint8_t got[32];
  pht_sha256_ctx c; pht_sha256_init(&c);
  pht_sha256_update(&c, "abc", 3);
  pht_sha256_final(&c, got);
  if (memcmp(got, want_abc, 32) != 0) return false;

  // SHA-256("") —— 空输入也要对（final 的填充逻辑最容易在这写错）
  static const uint8_t want_empty[32] = {
    0xe3,0xb0,0xc4,0x42,0x98,0xfc,0x1c,0x14,0x9a,0xfb,0xf4,0xc8,0x99,0x6f,0xb9,0x24,
    0x27,0xae,0x41,0xe4,0x64,0x9b,0x93,0x4c,0xa4,0x95,0x99,0x1b,0x78,0x52,0xb8,0x55
  };
  pht_sha256_init(&c);
  pht_sha256_final(&c, got);
  if (memcmp(got, want_empty, 32) != 0) return false;

  // HMAC-SHA256(key="key", msg="The quick brown fox jumps over the lazy dog")
  static const uint8_t want_hmac[32] = {
    0xf7,0xbc,0x83,0xf4,0x30,0x53,0x84,0x24,0xb1,0x32,0x98,0xe6,0xaa,0x6f,0xb1,0x43,
    0xef,0x4d,0x59,0xa1,0x49,0x46,0x17,0x59,0x97,0x47,0x9d,0xbc,0x2d,0x1a,0x3c,0xd8
  };
  pht_hmac_sha256((const uint8_t*)"key", 3,
                  "The quick brown fox jumps over the lazy dog", 43, got);
  if (memcmp(got, want_hmac, 32) != 0) return false;

  return true;
}

#endif  // PHT_CRYPTO_H
