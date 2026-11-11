#pragma once

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

namespace Ins {

struct Fix {
  double latitude=0,longitude=0;
  float height=0,speed=0,course=0,hdop=99;
  uint32_t received=0,sequence=0;
  uint8_t satellites=0,dimension=0;
  bool valid=false,heightValid=false,velocityValid=false;
};

class GPS {
public:
  Fix fix;
  uint32_t bytes=0,validSentences=0,errors=0,lastByte=0;
  bool feed(uint8_t c,uint32_t now) {
    ++bytes; lastByte=now;
    if (c=='$') { collecting_=true; length_=0; return false; }
    if (!collecting_) return false;
    if (c=='\r') return false;
    if (c!='\n') {
      if (c<32 || c>126 || length_>=sizeof(line_)-1) {
        collecting_=false; ++errors; return false;
      }
      line_[length_++]=static_cast<char>(c);
      return false;
    }
    collecting_=false; line_[length_]=0;
    char *star=strchr(line_,'*');
    if (!star || star!=line_+length_-3 || hex(star[1])<0 || hex(star[2])<0) {
      ++errors; return false;
    }
    uint8_t checksum=0;
    for (char *p=line_;p<star;++p) checksum^=static_cast<uint8_t>(*p);
    if (checksum!=hex(star[1])*16+hex(star[2])) { ++errors; return false; }
    ++validSentences; *star=0;
    char *field[22]={line_};
    unsigned n=1;
    for (char *p=line_;*p;++p) {
      if (*p==',') {
        *p=0;
        if (n>=22) { ++errors; return false; }
        field[n++]=p+1;
      }
    }
    if (strlen(field[0])!=5) return false;
    const char *type=field[0]+2;
    double v=0;
    if (strcmp(type,"GSV")==0 && n>=4) {
      if (number(field[3],v) && v>=0 && v<=72 && floor(v)==v) {
        visible_=static_cast<uint8_t>(v); visibleTime_=now; haveVisible_=true;
      }
      return false;
    }
    if (strcmp(type,"GSA")==0 && n>=3) {
      if (number(field[2],v) && (v==1 || v==2 || v==3)) {
        dimension_=static_cast<uint8_t>(v); dimensionTime_=now;
        if (dimension_==1) fix.valid=false;
      }
      return false;
    }
    if (strcmp(type,"RMC")==0 && n>=10) {
      rmc_.valid=false;
      if (strcmp(field[2],"A")!=0) { fix.valid=false; return false; }
      if (!utc(field[1],rmc_.time) || !coordinate(field[3],field[4],true,rmc_.lat) ||
          !coordinate(field[5],field[6],false,rmc_.lon)) {
        ++errors; fix.valid=false; return false;
      }
      rmc_.velocity=number(field[7],v) && v>=0 && v<650;
      rmc_.speed=rmc_.velocity?static_cast<float>(v*0.5144444444):0;
      rmc_.velocity=rmc_.velocity && number(field[8],v) && v>=0 && v<360;
      rmc_.course=rmc_.velocity?static_cast<float>(v):0;
      rmc_.received=now; rmc_.valid=true;
    } else if (strcmp(type,"GGA")==0 && n>=11) {
      gga_.valid=false;
      if (!number(field[6],v) || (v!=1 && v!=2)) { fix.valid=false; return false; }
      if (!utc(field[1],gga_.time) || !coordinate(field[2],field[3],true,gga_.lat) ||
          !coordinate(field[4],field[5],false,gga_.lon) ||
          !number(field[7],v) || v<0 || v>72 || floor(v)!=v) {
        ++errors; fix.valid=false; return false;
      }
      gga_.satellites=static_cast<uint8_t>(v);
      if (!number(field[8],v) || v<=0 || v>50) { ++errors; fix.valid=false; return false; }
      gga_.hdop=static_cast<float>(v);
      gga_.heightValid=number(field[9],v) && v>=-1000 && v<=30000 && strcmp(field[10],"M")==0;
      gga_.height=gga_.heightValid?static_cast<float>(v):0;
      gga_.received=now; gga_.valid=true;
    } else return false;
    if (!rmc_.valid || !gga_.valid || now-rmc_.received>1500 || now-gga_.received>1500 ||
        fabs(rmc_.time-gga_.time)>0.02 || fabs(rmc_.lat-gga_.lat)>0.00005 ||
        fabs(rmc_.lon-gga_.lon)>0.00005 || rmc_.time==lastEpoch_) return false;
    lastEpoch_=rmc_.time;
    fix.latitude=rmc_.lat; fix.longitude=rmc_.lon;
    fix.height=gga_.height; fix.heightValid=gga_.heightValid;
    fix.speed=rmc_.speed; fix.course=rmc_.course; fix.velocityValid=rmc_.velocity;
    fix.satellites=gga_.satellites; fix.hdop=gga_.hdop;
    const bool dimensionFresh=now-dimensionTime_<3000 && dimension_!=0;
    fix.dimension=dimensionFresh?dimension_:2;
    fix.valid=fix.satellites>=4 && fix.hdop<=10 && fix.dimension>=2;
    fix.heightValid=fix.heightValid && fix.dimension==3;
    fix.received=now; ++fix.sequence;
    return true;
  }
  bool fresh(uint32_t now) const { return fix.valid && now-fix.received<2000; }
  uint8_t satellites(uint32_t now) const {
    return haveVisible_ && now-visibleTime_<5000?visible_:fresh(now)?fix.satellites:0;
  }
private:
  struct Partial {
    double time=-1,lat=0,lon=0;
    float speed=0,course=0,height=0,hdop=99;
    uint32_t received=0;
    uint8_t satellites=0;
    bool valid=false,velocity=false,heightValid=false;
  } rmc_,gga_;
  char line_[128]={};
  unsigned length_=0;
  bool collecting_=false;
  double lastEpoch_=-1;
  uint8_t dimension_=0;
  uint32_t dimensionTime_=0;
  uint8_t visible_=0;
  uint32_t visibleTime_=0;
  bool haveVisible_=false;
  static int hex(char c) {
    if (c>='0' && c<='9') return c-'0';
    if (c>='A' && c<='F') return c-'A'+10;
    if (c>='a' && c<='f') return c-'a'+10;
    return -1;
  }
  static bool number(const char *s,double &v) {
    if (!s || !*s || *s==' ' || *s=='+' || *s=='\t') return false;
    bool digit=false,point=false;
    for (const char *p=s;*p;++p) {
      if (*p>='0' && *p<='9') { digit=true; continue; }
      if (*p=='.' && !point) { point=true; continue; }
      if (*p=='-' && p==s) continue;
      return false;
    }
    if (!digit) return false;
    char *end=nullptr;
    v=strtod(s,&end);
    return end!=s && *end==0 && isfinite(v);
  }
  static bool utc(const char *s,double &seconds) {
    double v;
    if (!number(s,v) || v<0 || v>=240000) return false;
    const unsigned hour=static_cast<unsigned>(v/10000);
    const unsigned minute=static_cast<unsigned>(v/100)%100;
    const double sec=fmod(v,100);
    if (hour>23 || minute>59 || sec>=60) return false;
    seconds=hour*3600+minute*60+sec;
    return true;
  }
  static bool coordinate(const char *s,const char *direction,bool latitude,double &out) {
    double v;
    if (!number(s,v) || v<0 || strlen(direction)!=1) return false;
    const double degrees=floor(v/100),minutes=v-degrees*100;
    if (minutes>=60 || degrees>(latitude?90:180)) return false;
    out=degrees+minutes/60;
    if (out>(latitude?90:180)) return false;
    if (*direction==(latitude?'S':'W')) out=-out;
    else if (*direction!=(latitude?'N':'E')) return false;
    return true;
  }
};

}
