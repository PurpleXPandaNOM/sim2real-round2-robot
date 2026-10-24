#pragma once
#include <cstddef>
#include <cstdint>
struct WireMock {
 bool ok=true;uint8_t reg=0,index=0;int writes=0;int16_t raw=0;
 bool begin(int,int){return true;}void setClock(int){}void setTimeOut(int){}
 void beginTransmission(uint8_t){writes=0;}
 void write(uint8_t v){if(!writes++)reg=v;}
 int endTransmission(bool=true){return ok?0:4;}
 size_t requestFrom(uint8_t,uint8_t n){index=0;return ok?n:0;}
 int read(){if(reg==0x75)return 0x68;return index++==0?((uint16_t)raw>>8):((uint16_t)raw&255);}
};
inline WireMock Wire;
