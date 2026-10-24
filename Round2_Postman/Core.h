#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>
namespace nav {
constexpr float kPi=3.14159265358979323846f;
inline float clip(float x,float lo,float hi){return std::max(lo,std::min(x,hi));}
inline float wrap(float a){return std::atan2(std::sin(a),std::cos(a));}
struct Point {float x=0,y=0;};
struct Pose {float x=0,y=0,yaw=0;};
inline float distance(Point a,Point b){return std::hypot(a.x-b.x,a.y-b.y);}
inline float ramp(float old,float target,float delta){return old+clip(target-old,-delta,delta);}
struct PID {
 float kp,ki,kd,integral=0,previousError=0;
 PID(float p,float i,float d):kp(p),ki(i),kd(d){}
 void reset(){integral=0;previousError=0;}
 // Derivative supplied as negative measured rate to avoid setpoint kick.
 float step(float error,float derivative,float dt,float limit,float ff=0){
  if(error*previousError<0)integral=0;
  previousError=error;
  float candidate=integral+error*dt;
  if(ki>0) candidate=clip(candidate,-limit/ki,limit/ki);
  float u=kp*error+ki*candidate+kd*derivative+ff;
  if(std::fabs(u)<=limit || (u>limit && error<0) || (u<-limit && error>0)) integral=candidate;
  return clip(kp*error+ki*integral+kd*derivative+ff,-limit,limit);
 }
};
inline void integrate(Pose &p,float dl,float dr,float gyroRate,float dt,float track,float weight){
 float da=weight*gyroRate*dt+(1-weight)*(dr-dl)/track;
 float ds=(dl+dr)*0.5f;
 float sinc=std::fabs(da)<1e-5f?1.0f:std::sin(da*0.5f)/(da*0.5f);
 p.x+=ds*sinc*std::cos(p.yaw+da*0.5f);
 p.y+=ds*sinc*std::sin(p.yaw+da*0.5f);
 p.yaw=wrap(p.yaw+da);
}
// Exhaustively test 5! = 120 orders. Exact optimum for Euclidean distances.
inline float plan(const Point points[5],int result[5],bool includeReturn,float turnPenalty=0){
 int order[5]={0,1,2,3,4};
 float best=std::numeric_limits<float>::infinity();
 do {
  Point last{0,0}; float cost=0, yaw=0;
  auto edge = [&](Point next) {
   float length=distance(last,next);
   if(length>1e-6f){
    float h=std::atan2(next.y-last.y,next.x-last.x);
    cost+=length+turnPenalty*std::fabs(wrap(h-yaw));yaw=h;
   }
   last=next;
  };
  for(int i:order)edge(points[i]);
  if(includeReturn)edge({0,0});
  if(cost<best){best=cost;std::copy(order,order+5,result);}
 } while(std::next_permutation(order,order+5));
 return best;
}
}

namespace nav {
inline float friction(float output,float error,float minimum){
 // Do not amplify a derivative braking command that opposes position error.
 if(output*error<=0 || std::fabs(output)<1e-6f)return output;
 return std::copysign(std::max(std::fabs(output),minimum),output);
}
inline void limitPair(float &l,float &r,float limit){
 float scale=std::max(1.0f,std::max(std::fabs(l),std::fabs(r))/limit);
 l/=scale;r/=scale;
}
}

namespace nav {
// Stillness evidence is accumulated only from completed short windows.
// Missing gyro intervals are excluded, not silently counted as stationary.
struct RestGate {
 float windowMs=0, freshMs=0, quietMs=0;
 float windowL=0,windowR=0,totalL=0,totalR=0,totalYaw=0;
 bool quiet=false,closed=false;
 void reset(){*this=RestGate{};}
 void prime(float ms){reset();quiet=true;quietMs=ms;}
 void sample(float dt,float dl,float dr,float gyro,bool fresh,bool gapAllowed,
             float windowDuration,float wheelWindowLimit,float totalTravelLimit,
             float gyroLimit,float totalYawLimit){
  closed=false;
  if(dt<=0 || dt>0.1f || (!fresh&&!gapAllowed) || (fresh&&std::fabs(gyro)>gyroLimit)){
   reset();return;
  }
  float al=std::fabs(dl), ar=std::fabs(dr);
  windowMs+=dt*1000;windowL+=al;windowR+=ar;totalL+=al;totalR+=ar;
  if(fresh){freshMs+=dt*1000;totalYaw+=std::fabs(gyro)*dt;}
  if(totalL>totalTravelLimit||totalR>totalTravelLimit||totalYaw>totalYawLimit){reset();return;}
  if(windowMs+0.001f>=windowDuration){
   closed=true;
   quiet=windowL<=wheelWindowLimit*(windowMs/windowDuration)&&windowR<=wheelWindowLimit*(windowMs/windowDuration);
   if(quiet)quietMs=std::min(quietMs+freshMs,1000000.0f);
   else{quietMs=0;totalL=totalR=totalYaw=0;}
   windowMs=freshMs=windowL=windowR=0;
  }
 }
 bool ready(unsigned duration) const{return closed&&quiet&&quietMs+0.001f>=duration;}
};
}
