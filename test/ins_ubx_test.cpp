#include "ins_ubx.h"
#include <assert.h>
#include <stdio.h>

namespace {
bool same(const uint8_t *a,const uint8_t *b,unsigned n) { return memcmp(a,b,n)==0; }
}

int main() {
  uint8_t out[Ubx::MaxFrame];
  // Reference frames published for u-blox 6 (u-center generated).
  const uint8_t gllOff[]={0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11};
  const uint8_t rate5[]={0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A};
  assert(Ubx::sequence(true,2,out)==sizeof(gllOff) && same(out,gllOff,sizeof(gllOff)));
  assert(Ubx::sequence(true,8,out)==sizeof(rate5) && same(out,rate5,sizeof(rate5)));
  unsigned total=0;
  for (unsigned i=0;;++i) {
    const unsigned n=Ubx::sequence(true,i,out);
    if (!n) { assert(i==10); break; }
    uint8_t a=0,b=0;
    for (unsigned k=2;k<n-2;++k) { a=static_cast<uint8_t>(a+out[k]); b=static_cast<uint8_t>(b+a); }
    assert(out[0]==0xB5 && out[1]==0x62 && out[n-2]==a && out[n-1]==b);
    assert(out[4]+8u==n && out[5]==0);
    total+=n;
  }
  assert(total==157);
  assert(Ubx::sequence(true,0,out)==28 && out[2]==0x06 && out[3]==0x00);
  assert(out[6]==1 && out[14]==0x00 && out[15]==0x96 && out[16]==0 && out[17]==0); // 38400.
  assert(out[18]==3 && out[20]==2); // In UBX+NMEA, out NMEA only.
  assert(Ubx::switchesBaudAfter(0) && !Ubx::switchesBaudAfter(1));
  assert(Ubx::sequence(false,0,out)==21 && out[3]==0x09 && out[6]==0xFF && out[14]==0xFF);
  assert(Ubx::sequence(false,1,out)==21 && Ubx::sequence(false,2,out)==0);
  puts("ins_ubx_test: PASS");
}
