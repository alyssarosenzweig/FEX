%ifdef CONFIG
{
  "RegData": {
      "XMM1": ["0x97D4574EE323773D", "0xA934C32F562D8E88"],
      "XMM2": ["0x6868C3F3AAED56E0", "0xF0FCE9E294E6E6DE"]
  },
  "HostFeatures": ["SHA"]
}
%endif

lea rdx, [rel .data]

movaps xmm0, [rdx + 16 * 2]
movaps xmm1, [rdx + 16 * 0]
movaps xmm2, [rdx + 16 * 1]

sha256rnds2 xmm1, xmm2

hlt

align 16
.data:
db 0xe0, 0xfc, 0x2b, 0xa1, 0x06, 0x4f, 0x6c, 0xa7, 0x0f, 0x06, 0x6a, 0x1e, 0x7f, 0x76, 0x80, 0x9b
db 0xe0, 0x56, 0xed, 0xaa, 0xf3, 0xc3, 0x68, 0x68, 0xde, 0xe6, 0xe6, 0x94, 0xe2, 0xe9, 0xfc, 0xf0
db 0xc7, 0xcd, 0x73, 0xec, 0x95, 0xd6, 0x6f, 0x6a, 0xbb, 0xae, 0xf2, 0xbb, 0x27, 0xb9, 0xa1, 0xdd
