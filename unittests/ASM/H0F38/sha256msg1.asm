%ifdef CONFIG
{
  "RegData": {
      "XMM1": ["0x43DEA25DAB8EF585", "0x1D30D1491042EED2"],
      "XMM2": ["0x6868C3F3AAED56E0", "0xF0FCE9E294E6E6DE"]
  },
  "HostFeatures": ["SHA"]
}
%endif

lea rdx, [rel .data]

movaps xmm1, [rdx + 16 * 0]
movaps xmm2, [rdx + 16 * 1]

sha256msg1 xmm1, xmm2

hlt

align 16
.data:
db 0xe0, 0xfc, 0x2b, 0xa1, 0x06, 0x4f, 0x6c, 0xa7, 0x0f, 0x06, 0x6a, 0x1e, 0x7f, 0x76, 0x80, 0x9b
db 0xe0, 0x56, 0xed, 0xaa, 0xf3, 0xc3, 0x68, 0x68, 0xde, 0xe6, 0xe6, 0x94, 0xe2, 0xe9, 0xfc, 0xf0
