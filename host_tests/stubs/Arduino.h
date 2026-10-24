#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <cmath>
#include <map>
#include <string>
#define PI 3.1415926535897932384626433832795
#define ARDUINO_ISR_ATTR
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 3
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define portENTER_CRITICAL_ISR(x) ((void)(x))
#define portEXIT_CRITICAL_ISR(x) ((void)(x))
inline unsigned long clockUs=1000000;
inline std::map<int,int> gpio;
inline unsigned long micros(){return clockUs;}
inline unsigned long millis(){return clockUs/1000;}
inline void delay(unsigned ms){clockUs+=ms*1000;}
inline void pinMode(int,int){}
inline int digitalRead(int p){return gpio.count(p)?gpio[p]:1;}
inline void digitalWrite(int p,int v){gpio[p]=v;}
inline int digitalPinToInterrupt(int p){return p;}
inline void attachInterrupt(int,void(*)(),int){}
inline bool ledcAttach(int,int,int){return true;}
inline std::map<int,int> pwm;
inline bool ledcWrite(int p,int v){pwm[p]=v;return true;}
struct SerialMock {
 std::string output,input;
 void begin(int){}
 int available(){return (int)input.size();}int availableForWrite(){return 127;}char read(){char c=input[0];input.erase(0,1);return c;}
 void print(const char*s){output+=s;}void println(const char*s){output+=s;output+='\n';}
 void printf(const char*f,...){char b[1024];va_list a;va_start(a,f);vsnprintf(b,sizeof(b),f,a);va_end(a);output+=b;}
};
inline SerialMock Serial;
