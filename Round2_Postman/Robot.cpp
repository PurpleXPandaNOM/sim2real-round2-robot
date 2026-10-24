#include <Arduino.h>
#include <Wire.h>
#include <esp_arduino_version.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include "Config.h"
#include "Core.h"
#if ESP_ARDUINO_VERSION_MAJOR < 3
#error "Select esp32 by Espressif Systems version 3.x (pin-based LEDC API)."
#endif
using nav::clip; using nav::wrap;
static constexpr float DEG=nav::kPi/180.0f;
static_assert(cfg::POS_TOL_M<=cfg::HOME_ACCEPT_M && cfg::HOME_ACCEPT_M<=cfg::ROUTE_ACCEPT_M && cfg::DWELL_MARGIN_M>0,"Invalid arrival/dwell radii");
static_assert(cfg::ROUTE_ACCEPT_M+cfg::DWELL_MARGIN_M<0.10f,"Keep margin inside the 10 cm circle; estimate is not truth");
static_assert(cfg::TRACK_M>0 && cfg::TICKS_L_PER_WHEEL_REV>0 && cfg::TICKS_R_PER_WHEEL_REV>0,"Invalid geometry");
static_assert(cfg::GYRO_WEIGHT>=0 && cfg::GYRO_WEIGHT<=1,"Invalid fusion weight");
static_assert(cfg::PWM_LIMIT>0 && cfg::PWM_LIMIT<=255,"8-bit PWM limit");
static portMUX_TYPE encoderMux=portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t ticksL=0,ticksR=0;
static void ARDUINO_ISR_ATTR encoderL(){
 portENTER_CRITICAL_ISR(&encoderMux);
 ticksL+=(digitalRead(cfg::ENC_L_A)==digitalRead(cfg::ENC_L_B)?1:-1)*cfg::ENC_L_SIGN;
 portEXIT_CRITICAL_ISR(&encoderMux);
}
static void ARDUINO_ISR_ATTR encoderR(){
 portENTER_CRITICAL_ISR(&encoderMux);
 ticksR+=(digitalRead(cfg::ENC_R_A)==digitalRead(cfg::ENC_R_B)?1:-1)*cfg::ENC_R_SIGN;
 portEXIT_CRITICAL_ISR(&encoderMux);
}
static void snapshot(int64_t &l,int64_t &r){
 portENTER_CRITICAL(&encoderMux);l=ticksL;r=ticksR;portEXIT_CRITICAL(&encoderMux);
}
enum class State {IDLE,CALIBRATING,COUNTDOWN,RESUMING,TURN,DRIVE,REALIGN,DWELL,DONE,FAULT};
static State state=State::IDLE,resumeState=State::IDLE,faultState=State::IDLE;
static nav::Pose pose;
static nav::Point points[5],goal,driveOrigin;
static bool pointsReady[5]={};
static int order[5]={0,1,2,3,4},leg=0,lastStopLeg=-1,recoverySelection=0;
static bool pwmOK=false,imuOK=false,imuFresh=false,calibrated=false;
static bool armed=false,route=false,paused=false,poseUncertain=false;
static bool handling=false,noiseLog=false;
static float drivePWM=cfg::DRIVE_PWM_LIMIT,turnPWM=cfg::TURN_PWM_LIMIT;
static float minDrivePWM=cfg::MIN_DRIVE_PWM,minTurnPWM=cfg::MIN_TURN_PWM;
static float noiseL=0,noiseR=0,noiseGyro=0;
static unsigned noiseSamples=0,noiseBad=0;
static void enablePin(int pin,bool enabled){if(pin>=0)digitalWrite(pin,enabled?HIGH:LOW);}
static bool stopPressed(){return cfg::STOP_PIN>=0 && digitalRead(cfg::STOP_PIN)==LOW;}
static int64_t previousL=0,previousR=0;
static float speedL=0,speedR=0,gyro=0,bias=0,angularRate=0,travel=0;
static float requestDrive=0,requestTurn=0,outputL=0,outputR=0;
static float goalHeading=0,driveDistance=0,driveError=0,turnError=0;
static nav::RestGate rest;
static unsigned realigns=0;
static uint32_t taskAt=0,recoveryBlinkAt=0;
struct Calibration {
 uint32_t started=0,quietSince=0,sampleSince=0;
 unsigned samples=0;double sum=0,sum2=0;
 float movementL=0,movementR=0;
};
static Calibration calibration;
static void invoke(const char *text);
static nav::PID heading(cfg::HEADING_KP,cfg::HEADING_KI,cfg::HEADING_KD);
static nav::PID distPID(cfg::DIST_KP,cfg::DIST_KI,cfg::DIST_KD);
static uint32_t stateAt=0,runAt=0,stillAt=0,lastMicros=0,lastReport=0,pauseAt=0;
static uint32_t stallAtL=0,stallAtR=0,lastGoodIMU=0,imuWindowAt=0;
static unsigned healthyTicks=0,imuBadInWindow=0;
static uint32_t resumeTaskElapsed=0;
static const char *stateName(){
 switch(state){
 case State::IDLE:return "IDLE";case State::CALIBRATING:return "CALIBRATING";case State::COUNTDOWN:return "COUNTDOWN";
 case State::RESUMING:return "RESUMING";case State::TURN:return "TURN";
 case State::DRIVE:return "DRIVE";case State::REALIGN:return "REALIGN";case State::DWELL:return "DWELL";
 case State::DONE:return "DONE";default:return "FAULT";
 }
}
static void led(bool on){if(cfg::LED_PIN>=0)digitalWrite(cfg::LED_PIN,on?HIGH:LOW);}
static void zeroPWM(bool brake=false){
 // Disable first. Brake only for normal settling after explicit driver validation.
 enablePin(cfg::L_EN,false);enablePin(cfg::R_EN,false);
 if(pwmOK){
  ledcWrite(cfg::L_RPWM,0);ledcWrite(cfg::L_LPWM,0);
  ledcWrite(cfg::R_RPWM,0);ledcWrite(cfg::R_LPWM,0);
  if(brake && cfg::BRAKE_WHEN_SETTLING){enablePin(cfg::L_EN,true);enablePin(cfg::R_EN,true);}
 }
}
static void stopMotion(){
 requestDrive=requestTurn=outputL=outputR=0;
 heading.reset();distPID.reset();stallAtL=stallAtR=0;zeroPWM();
}
static bool busy(){return state==State::CALIBRATING||state==State::COUNTDOWN||state==State::RESUMING||state==State::TURN||state==State::DRIVE||state==State::REALIGN||state==State::DWELL;}
static bool stationary(){
 return imuFresh&&imuOK&&rest.quiet&&fabsf(gyro)<cfg::STILL_GYRO_RADPS;
}
static bool inside(float x,float y){return std::isfinite(x)&&std::isfinite(y)&&x>=cfg::X_MIN&&x<=cfg::X_MAX&&y>=cfg::Y_MIN&&y<=cfg::Y_MAX;}
static void transition(State next){
 stopMotion();state=next;stateAt=millis();stillAt=0;rest.reset();
 led(next==State::DWELL||next==State::DONE);
 Serial.printf("# STATE %s\n",stateName());
}
static void fault(const char *reason,bool uncertain=false){
 poseUncertain=poseUncertain||uncertain;
 if(state!=State::FAULT){
  faultState=(state==State::RESUMING?resumeState:state);
  recoverySelection=leg;recoveryBlinkAt=millis();pauseAt=millis();
 }
 stopMotion();armed=false;paused=route;stillAt=0;rest.reset();led(false);
 if(state!=State::FAULT)Serial.printf("# FAULT %s; route retained=%d; RETRY/RECOVER or STOP\n",reason,route);
 state=State::FAULT;
}
static void pauseMotion(){
 if(paused || !busy() || state==State::CALIBRATING)return;
 if(state==State::RESUMING){taskAt+=uint32_t(millis()-stateAt);state=resumeState;stateAt=millis()-resumeTaskElapsed;}
 paused=true;pauseAt=millis();stillAt=0;rest.reset();stopMotion();recoverySelection=leg;recoveryBlinkAt=millis();
 Serial.println("# PAUSED; no movement until explicit RESUME.");
}
static bool regWrite(uint8_t reg,uint8_t v){
 Wire.beginTransmission(cfg::IMU_ADDRESS);Wire.write(reg);Wire.write(v);return Wire.endTransmission()==0;
}
static bool readBytes(uint8_t reg,uint8_t *b,uint8_t n){
 Wire.beginTransmission(cfg::IMU_ADDRESS);Wire.write(reg);
 if(Wire.endTransmission(false)!=0)return false;
 if(Wire.requestFrom(cfg::IMU_ADDRESS,n)!=n)return false;
 for(unsigned i=0;i<n;i++){int v=Wire.read();if(v<0)return false;b[i]=(uint8_t)v;}
 return true;
}
static bool readGyro(float &rate){
 uint8_t b[2];
 // One bounded I2C transaction per tick; subsequent ticks provide retries.
 // Avoid 3 blocking retries inside a 10 ms control period.
 if(!readBytes(0x47,b,2))return false;
 int16_t raw=(int16_t)(((uint16_t)b[0]<<8)|b[1]);
 if(raw>=32700||raw<=-32700)return false; // saturated gyro is not valid yaw data
 rate=cfg::GYRO_SIGN*(float)raw/131.0f*DEG;return true;
}
static bool initIMU(){
 uint8_t id;if(!readBytes(0x75,&id,1)||(id&0x7e)!=0x68)return false;
 if(!regWrite(0x6b,0x01))return false;
 delay(100);
 return regWrite(0x1b,0x00)&&regWrite(0x1a,0x03)&&regWrite(0x19,0x09);
}
static void syncEncoders(){snapshot(previousL,previousR);speedL=speedR=0;lastMicros=micros();}
static void resetPose(float x,float y,float angle){
 pose={x,y,wrap(angle)};travel=0;poseUncertain=false;syncEncoders();
}
// Calibration is a state, so STOP and input remain serviced throughout.
static bool calibrate(){
 if(route)return false;
 stopMotion();armed=false;calibrated=false;
 calibration={};calibration.started=millis();calibration.quietSince=millis();
 transition(State::CALIBRATING);
 Serial.println("# CAL: hands off. Stable 1.5s, then 4s samples; automatic retry up to 20s.");
 return true;
}
static void calibrationTick(float raw,float dl,float dr){
 uint32_t now=millis();
 if(uint32_t(now-calibration.started)>=cfg::CAL_TIMEOUT_MS){fault("calibration did not obtain a stable window");return;}
 calibration.movementL+=fabsf(dl);calibration.movementR+=fabsf(dr);
 bool disturbed=!imuFresh||calibration.movementL>cfg::CAL_MAX_TRAVEL_M||calibration.movementR>cfg::CAL_MAX_TRAVEL_M||fabsf(raw)>10*DEG;
 if(disturbed){
  uint32_t begun=calibration.started;calibration={};calibration.started=begun;calibration.quietSince=now;return;
 }
 if(uint32_t(now-calibration.quietSince)<cfg::CAL_SETTLE_MS)return;
 if(!calibration.samples)calibration.sampleSince=now;
 calibration.sum+=raw;calibration.sum2+=raw*raw;++calibration.samples;
 if(uint32_t(now-calibration.sampleSince)<cfg::CAL_SAMPLE_MS || calibration.samples<cfg::CAL_MIN_SAMPLES)return;
 float mean=calibration.sum/calibration.samples;
 float sd=sqrtf(fmaxf(0,calibration.sum2/calibration.samples-mean*mean));
 if(sd>cfg::CAL_MAX_SD_RADPS){
  uint32_t begun=calibration.started;calibration={};calibration.started=begun;calibration.quietSince=now;
  Serial.println("# CAL: noisy sample window; waiting for stillness again.");return;
 }
 bias=mean;calibrated=imuOK=imuFresh=true;gyro=0;healthyTicks=cfg::HEALTHY_TICKS_TO_RETRY;
 lastGoodIMU=now;imuWindowAt=now;imuBadInWindow=0;
 transition(State::IDLE);rest.prime(cfg::SETTLE_MS);
 Serial.printf("# CAL OK bias=%.5f deg/s sd=%.5f deg/s\n",bias/DEG,sd/DEG);
}
static bool allPoints(){for(bool v:pointsReady)if(!v)return false;return true;}
static void printPlan(){
 float cost=nav::plan(points,order,cfg::OPTIMIZE_RETURN,cfg::TURN_PENALTY_M_PER_RAD);
 float length=0;nav::Point p{0,0};
 Serial.print("# ORDER origin");
 for(int i:order){Serial.printf(" -> P%d",i+1);length+=nav::distance(p,points[i]);p=points[i];}
 Serial.printf(" -> home; objective=%s cost=%.3f; total_distance=%.2f cm\n",cfg::OPTIMIZE_RETURN?"closed":"open",cost,100*(length+nav::distance(p,{0,0})));
}
static void beginLeg(){
 taskAt=millis();realigns=0;recoverySelection=leg;
 goal=leg<5?points[order[leg]]:nav::Point{0,0};
 goalHeading=atan2f(goal.y-pose.y,goal.x-pose.x);
 Serial.printf("# TARGET %d %.2f %.2f cm\n",leg<5?order[leg]+1:0,goal.x*100,goal.y*100);
 if(nav::distance({pose.x,pose.y},goal)<=cfg::POS_TOL_M){lastStopLeg=leg;transition(State::DWELL);}
 else transition(State::TURN);
}
static void finishTask(){bool wasRoute=route;transition(State::DONE);if(wasRoute)armed=false;else rest.prime(cfg::SETTLE_MS);route=false;Serial.println("# COMPLETE: estimated pose only; verify physical position.");}
static bool heldStill(unsigned duration){return rest.ready(duration)&&imuOK&&imuFresh;}
static float arrivalRadius(){return leg==5?cfg::HOME_ACCEPT_M:cfg::ROUTE_ACCEPT_M;}
static void enterDwell(){lastStopLeg=leg;transition(State::DWELL);}
static void requestRealign(){
 if(++realigns>cfg::MAX_REALIGNS_PER_LEG){fault("repeated alignment failure; inspect/recover");return;}
 transition(State::REALIGN); // stop and settle before changing the line heading
}
static bool armChecks(){
 return cfg::HARDWARE_CONFIRMED&&pwmOK&&imuOK&&imuFresh&&calibrated&&!poseUncertain&&
 healthyTicks>=cfg::HEALTHY_TICKS_TO_RETRY&&!stopPressed()&&stationary();
}
static bool resumeMotion(){
 if(handling||!paused||!armChecks())return false; // Handling needs a known pose reset first.
 if(paused&&route&&state==State::FAULT&&armChecks()&&inside(pose.x,pose.y)){
  State next=faultState;
  if(next==State::FAULT||next==State::IDLE||next==State::DONE)next=State::TURN;
  transition(next);taskAt=millis();paused=true;pauseAt=millis();armed=true;
 }
 if(!armed||state==State::FAULT)return false;
 resumeState=state;taskAt+=uint32_t(millis()-pauseAt);
 resumeTaskElapsed=uint32_t(pauseAt-stateAt);
 transition(State::RESUMING);paused=false;
 Serial.println("# RESUME in 3 seconds. Hands clear; no pose reset.");return true;
}
static void beginHandling(){
 if(!route || state==State::CALIBRATING){Serial.println("# HANDLE needs an active/paused retained route.");return;}
 if(handling)return;
 if(!paused && state!=State::FAULT)pauseMotion();
 handling=true;paused=true;armed=false;poseUncertain=true;stopMotion();rest.reset();
 Serial.println("# HANDLING: motors stopped; pose invalid. Place at selected target with known heading, then RECOVER id heading, then RESUME.");
}
static bool recoverAt(int id,float angle){
 if(!route||!paused||id<0||id>5||!std::isfinite(angle)||fabsf(angle)>360)return false;
 int k=5;
 if(id){for(k=0;k<5&&order[k]!=id-1;k++){} }
 if(k>leg)return false; // cannot skip forward to an unvisited target
 stopMotion();
 bool wasHandling=handling;handling=false;
 nav::Point p=id?points[id-1]:nav::Point{0,0};
 resetPose(p.x,p.y,angle*DEG);leg=k;goal=p;lastStopLeg=k;recoverySelection=k;recoveryBlinkAt=millis();taskAt=millis();realigns=0;
 goalHeading=angle*DEG;faultState=State::DWELL;
 if(state!=State::FAULT)transition(State::DWELL);
 else stateAt=millis();
 paused=true;pauseAt=millis();stillAt=0;rest.reset();
 if(wasHandling){healthyTicks=0;imuOK=false;lastGoodIMU=millis();imuBadInWindow=0;imuWindowAt=millis();}
 if(state!=State::FAULT)armed=true; // RESUME still checks configuration, pose and sensor health.
 Serial.printf("# RECOVER P%d heading=%.1f; still PAUSED. Hands clear, wait for healthy stillness, then RESUME.\n",id,angle);
 return true;
}
static void control(float dt){
 requestDrive=requestTurn=0;
 // Run clock continues during pauses and faults; no recovery extends the limit.
 if(route&&uint32_t(millis()-runAt)>cfg::RUN_LIMIT_MS){route=false;fault("run time limit");return;}
 if(handling||!armed||paused||state==State::FAULT)return;
 if(state!=State::COUNTDOWN&&state!=State::RESUMING&&busy()&&uint32_t(millis()-taskAt)>cfg::TASK_TIMEOUT_MS){fault("task timeout");return;}
 if(state==State::COUNTDOWN){
  if(uint32_t(millis()-stateAt)>=cfg::START_MS){
   if(!stationary()||!imuOK){fault("not stationary/healthy at start");return;}
   resetPose(0,0,0);beginLeg();
  }
  return;
 }
 if(state==State::RESUMING){
  if(uint32_t(millis()-stateAt)>=cfg::RESUME_MS){
   if(!stationary()||!imuOK){fault("not stationary/healthy at resume");return;}
   taskAt+=uint32_t(millis()-stateAt);
   State next=resumeState;transition(next);stateAt=millis()-resumeTaskElapsed;
  }
  return;
 }
 if(state==State::TURN){
  float e=wrap(goalHeading-pose.yaw);
  if(fabsf(e)>cfg::ANG_TOL_RAD){rest.reset();turnError=e;requestTurn=heading.step(e,-angularRate,dt,turnPWM);}
  else if(heldStill(cfg::SETTLE_MS)){
   if(route){
    // Recalculate the line after turning, allowing for small physical translation.
    goalHeading=atan2f(goal.y-pose.y,goal.x-pose.x);transition(State::DRIVE);
   }else finishTask();
  }
 }else if(state==State::REALIGN){
  if(heldStill(cfg::SETTLE_MS)){
   if(nav::distance({pose.x,pose.y},goal)<=arrivalRadius())enterDwell();
   else{goalHeading=atan2f(goal.y-pose.y,goal.x-pose.x);transition(State::TURN);}
  }
 }else if(state==State::DRIVE){
  float e=0,he=0;
  if(route){
   float dx=goal.x-pose.x,dy=goal.y-pose.y;
   float radius=hypotf(dx,dy);
   if(radius<=cfg::POS_TOL_M){enterDwell();return;}
   // A fixed segment direction makes the remaining distance signed after overshoot.
   // No atan2(target-current) reversal or 180-degree turn near the waypoint.
   float along=dx*cosf(goalHeading)+dy*sinf(goalHeading);
   float cross=-dx*sinf(goalHeading)+dy*cosf(goalHeading);
   if(fabsf(along)<=cfg::POS_TOL_M){
    if(radius<=arrivalRadius())enterDwell();else requestRealign();
    return;
   }
   float direction=along>=0?1.0f:-1.0f;
   float correction=clip(atan2f(direction*cross,fmaxf(fabsf(along),cfg::LOOKAHEAD_M)),-0.45f,0.45f);
   he=wrap(goalHeading+correction-pose.yaw);
   if(fabsf(he)>0.8f){requestRealign();return;}
   e=direction*radius;
  }else{
   float projected=(pose.x-driveOrigin.x)*cosf(goalHeading)+(pose.y-driveOrigin.y)*sinf(goalHeading);
   e=driveDistance-projected;he=wrap(goalHeading-pose.yaw);
   if(fabsf(e)<=cfg::POS_TOL_M){if(heldStill(cfg::SETTLE_MS))finishTask();return;}
  }
  stillAt=0;rest.reset();driveError=e;turnError=he;
  requestDrive=distPID.step(e,-(speedL+speedR)*0.5f,dt,drivePWM);
  requestTurn=heading.step(he,-angularRate,dt,cfg::STEER_PWM_LIMIT);
 }else if(state==State::DWELL){
  if(nav::distance({pose.x,pose.y},goal)>arrivalRadius()+cfg::DWELL_MARGIN_M){
   // Keep segment heading: reverse a small overshoot without a U-turn.
   transition(State::DRIVE);return;
  }
  if(heldStill(cfg::DWELL_MS)){
   Serial.printf("# STOP RECORDED target=%d\n",leg<5?order[leg]+1:0);
   if(leg==5)finishTask();else{++leg;beginLeg();}
  }
 }
}
static void motorPWM(int rp,int lp,int en,int value){
 // Inactive direction first: never briefly drive both inputs during a reversal.
 if(value>=0){ledcWrite(lp,0);ledcWrite(rp,value);}
 else{ledcWrite(rp,0);ledcWrite(lp,-value);}
 enablePin(en,true);
}
static bool stalled(float command,float measured,uint32_t &since){
 if(fabsf(command)>=cfg::STALL_PWM&&fabsf(measured)<0.003f){
  if(!since)since=millis();
  return uint32_t(millis()-since)>=cfg::STALL_MS;
 }
 since=0;return false;
}
static void actuate(float dt){
 if(!armed||paused||state==State::FAULT||(requestDrive==0&&requestTurn==0)){
  outputL=outputR=0;stallAtL=stallAtR=0;
  // Clear integrators during deadband/settling to avoid residual restart kicks.
  heading.reset();distPID.reset();zeroPWM(armed&&!paused&&(state==State::DWELL||state==State::TURN||state==State::DRIVE));return;
 }
 if(!std::isfinite(requestDrive)||!std::isfinite(requestTurn)){fault("invalid controller output",true);return;}
 float base=nav::friction(requestDrive,driveError,minDrivePWM);
 // Adapted from Yashas v5: slow translation while correcting direction.
 // Apply AFTER friction compensation so the minimum-PWM boost cannot undo it.
 // Preserve opposite-error braking torque; forward AND reverse approaches scale.
 if(state==State::DRIVE && base*driveError>0)base*=fmaxf(0,cosf(turnError));
 float steer=state==State::TURN?nav::friction(requestTurn,turnError,minTurnPWM):requestTurn;
 float l=(base-steer)*cfg::MOTOR_L_TRIM,r=(base+steer)*cfg::MOTOR_R_TRIM;
 nav::limitPair(l,r,cfg::PWM_LIMIT);
 outputL=nav::ramp(outputL,l,cfg::PWM_SLEW_PER_SEC*dt);
 outputR=nav::ramp(outputR,r,cfg::PWM_SLEW_PER_SEC*dt);
 if(stalled(outputL,speedL,stallAtL)||stalled(outputR,speedR,stallAtR)){
  // A route-only practical settle window; demonstration tolerances stay strict.
  if(route&&state==State::DRIVE&&nav::distance({pose.x,pose.y},goal)<=arrivalRadius())enterDwell();
  else fault("wheel stalled/encoder absent",true);
  return;
 }
 motorPWM(cfg::L_RPWM,cfg::L_LPWM,cfg::L_EN,lroundf(outputL)*cfg::MOTOR_L_SIGN);
 motorPWM(cfg::R_RPWM,cfg::R_LPWM,cfg::R_EN,lroundf(outputR)*cfg::MOTOR_R_SIGN);
}
static void tick(float dt){
 float raw=0;imuFresh=readGyro(raw);uint32_t now=millis();
 if(uint32_t(now-imuWindowAt)>=1000){imuWindowAt=now;imuBadInWindow=0;}
 if(imuFresh){
  lastGoodIMU=now;gyro=raw-(calibrated?bias:0);
  if(healthyTicks<cfg::HEALTHY_TICKS_TO_RETRY)++healthyTicks;
  if(healthyTicks>=cfg::HEALTHY_TICKS_TO_RETRY)imuOK=true;
 }else{healthyTicks=0;++imuBadInWindow;gyro=0;}
 int64_t l,r;snapshot(l,r);
 float dl=(l-previousL)*nav::kPi*cfg::WHEEL_L_DIAMETER_M/cfg::TICKS_L_PER_WHEEL_REV;
 float dr=(r-previousR)*nav::kPi*cfg::WHEEL_R_DIAMETER_M/cfg::TICKS_R_PER_WHEEL_REV;
 previousL=l;previousR=r;
 if(handling){
  // Observe health but intentionally discard carried motion; recovery supplies a known pose.
  speedL=speedR=angularRate=0;rest.reset();control(dt);zeroPWM();return;
 }
 if(noiseLog){noiseL+=fabsf(dl);noiseR+=fabsf(dr);noiseGyro=fmaxf(noiseGyro,fabsf(gyro));++noiseSamples;if(!imuFresh)++noiseBad;}
 if(fabsf(dl/dt)>2||fabsf(dr/dt)>2){fault("implausible encoder increment",true);return;}
 speedL+=0.25f*(dl/dt-speedL);speedR+=0.25f*(dr/dt-speedR);
 float weight=(calibrated&&imuFresh)?cfg::GYRO_WEIGHT:0;
 angularRate=weight*gyro+(1-weight)*(dr-dl)/(cfg::TRACK_M*dt);
 nav::integrate(pose,dl,dr,gyro,dt,cfg::TRACK_M,weight);travel+=(dl+dr)*0.5f;
 if(uint32_t(now-lastGoodIMU)>cfg::IMU_GRACE_MS || (imuBadInWindow>=10&&!imuFresh)){
  imuOK=false;if(state!=State::FAULT&&state!=State::CALIBRATING)fault("persistent/repeated IMU loss");
 }
 if(armed&&!inside(pose.x,pose.y)){fault("estimated boundary exceeded");return;}
 rest.sample(dt,dl,dr,gyro,imuFresh,uint32_t(now-lastGoodIMU)<=cfg::STILL_GAP_MS,
             cfg::STILL_WINDOW_MS,cfg::STILL_WINDOW_TRAVEL_M,cfg::STILL_TOTAL_TRAVEL_M,
             cfg::STILL_GYRO_RADPS,cfg::STILL_TOTAL_YAW_RAD);
 if(state==State::CALIBRATING)calibrationTick(raw,dl,dr);
 control(dt);actuate(dt);
}
static void report(){
 Serial.printf("# Firmware Combined V4; route_leg=%d target=P%d recovery=P%d\n",
 route?leg+1:0,route?(leg<5?order[leg]+1:0):-1,
 route?(recoverySelection<5?order[recoverySelection]+1:0):-1);
 Serial.printf("# DIST gains %.3f %.3f %.3f; HEADING gains %.3f %.3f %.3f; lock=%d PWM_cap=%d\n",
 distPID.kp,distPID.ki,distPID.kd,heading.kp,heading.ki,heading.kd,!cfg::HARDWARE_CONFIRMED,cfg::PWM_LIMIT);
 Serial.printf("# handling=%d PWM drive=%.1f turn=%.1f minimums=%.1f,%.1f profile=%d\n",handling,drivePWM,turnPWM,minDrivePWM,minTurnPWM,cfg::WIRING_PROFILE);
 int64_t l,r;snapshot(l,r);
 Serial.printf("# %s paused=%d armed=%d cal=%d imu=%d fresh=%d uncertain=%d ticks=%lld,%lld bias=%.5f deg/s\n",stateName(),paused,armed,calibrated,imuOK,imuFresh,poseUncertain,(long long)l,(long long)r,bias/DEG);
}
static void telemetry(){
 if(noiseLog){
  char sample[160];
  int size=snprintf(sample,sizeof sample,"NOISE,%lu,%u,%.4f,%.4f,%.4f,%u,%.0f\n",(unsigned long)millis(),noiseSamples,noiseL*1000,noiseR*1000,noiseGyro/DEG,noiseBad,rest.quietMs);
  if(size>0&&size<(int)sizeof sample&&Serial.availableForWrite()>=size)Serial.print(sample);
  noiseL=noiseR=noiseGyro=0;noiseSamples=noiseBad=0;return;
 }
 // Data lines are CSV; diagnostic lines begin '#'. Drop telemetry if TX is full.
 char line[160];
 int n=snprintf(line,sizeof line,"%lu,%s,%d,%.2f,%.2f,%.2f,%.3f,%.3f,%d,%d,%d\n",(unsigned long)millis(),stateName(),paused,pose.x*100,pose.y*100,pose.yaw/DEG,speedL,speedR,route?(leg<5?order[leg]+1:0):-1,imuFresh,armed);
 if(n>0 && n<(int)sizeof line && Serial.availableForWrite()>=n)Serial.print(line);
}
static void help(){
 Serial.println("# Combined V4 commands: newline, 115200 baud, case insensitive\n"
 "# PREP (or CAL): calibrate only. ZERO: physically origin/+X, disarmed.\n"
 "# START: arm and RUN after preparation; 5s countdown. BOOT idle=START.\n"
 "# ARM | TURN absolute_deg | ROTATE relative_deg | DRIVE signed_cm\n"
 "# Demo completion stays armed. STOP disarms/cancels. STATUS | HELP | CLEAR\n"
 "# WP id x_cm y_cm | PLAN | RUN | PAUSE | RESUME (3s countdown)\n"
 "# HANDLE -> physically place -> RECOVER id heading_deg -> RESUME\n"
 "# Recovery id 1..5 or 0=home; current/previous only; no skipped dwell.\n"
 "# BOOT active=pause; paused short=resume; medium 0.6..2s=select target.\n"
 "# Paused hold >=2s=enter handling; next long hold=reset selected pose at +X.\n"
 "# GAIND kp ki kd | GAINH kp ki kd | PWM drive turn | MINPWM drive turn\n"
 "# Tuning: idle/disarmed, RAM-only; PWM bounded by Config PWM_LIMIT.\n"
 "# NOISE 1/0: idle/disarmed stillness logging on/off. RETRY remains available.\n"
 "# CSV: ms,state,paused,x_cm,y_cm,yaw_deg,left_mps,right_mps,target,imu_fresh,armed");
}
static bool parseNumber(const char *s,float &out){char *end;out=strtof(s,&end);return end!=s&&*end=='\0'&&std::isfinite(out);}
static void command(char *line){
 char *tokens[5];int count=0;char *save=nullptr;
 for(char *p=strtok_r(line," \t\r",&save);p;p=strtok_r(nullptr," \t\r",&save)){
  if(count==5){Serial.println("# Too many arguments");return;}tokens[count++]=p;
 }
 if(!count)return;
 for(char *p=tokens[0];*p;p++)*p=(char)toupper((unsigned char)*p);
 const char *c=tokens[0];float a=0,b=0,d=0;
 for(int i=1;i<count;i++){float v;if(!parseNumber(tokens[i],v)){Serial.println("# Invalid number");return;}if(i==1)a=v;if(i==2)b=v;if(i==3)d=v;}
 if(!strcmp(c,"STOP")&&count==1){
  stopMotion();armed=route=paused=handling=noiseLog=false;if(state!=State::FAULT)transition(State::IDLE);return;
 }
 if(!strcmp(c,"HELP")&&count==1){help();return;}
 if(!strcmp(c,"STATUS")&&count==1){report();telemetry();return;}
 if(!strcmp(c,"HANDLE")&&count==1){beginHandling();return;}
 if(!strcmp(c,"PAUSE")&&count==1){pauseMotion();return;}
 if(!strcmp(c,"SELECT")&&count==2&&paused&&route&&a>=0&&a<=5&&a==floorf(a)){
  int k=5;if(a>0){for(k=0;k<5&&order[k]!=(int)a-1;k++){} }
  if(k<=leg){recoverySelection=k;recoveryBlinkAt=millis();Serial.printf("# Recovery selection P%d; no pose change.\n",(int)a);}
  else Serial.println("# Cannot select a future target.");
  return;
 }
 if(!strcmp(c,"RECOVER")&&count==3&&a>=0&&a<=5&&a==floorf(a)){
  if(!recoverAt((int)a,b))Serial.println("# RECOVER rejected: pause active route; use current/previous target ID and valid heading.");
  return;
 }
 if(!strcmp(c,"RETRY")&&count==1&&state==State::FAULT){
  if(handling||!route||!armChecks()||!inside(pose.x,pose.y)){Serial.println("# RETRY rejected: need retained route, healthy stationary hardware, valid pose. RECOVER if pose uncertain.");return;}
  State next=faultState;
  if(next==State::FAULT||next==State::IDLE||next==State::DONE)next=State::TURN;
  transition(next);taskAt=millis();armed=true;paused=true;pauseAt=millis();
  Serial.println("# RETRY accepted; still PAUSED. RESUME separately.");return;
 }
 if(!strcmp(c,"RESUME")&&count==1){if(!resumeMotion())Serial.println("# RESUME rejected: pause/arm/health/pose required.");return;}
 if(!strcmp(c,"CLEAR")&&count==1&&(!busy()||state==State::FAULT)){
  if(stopPressed()){Serial.println("# Release STOP first");return;}
  stopMotion();armed=route=paused=calibrated=handling=noiseLog=false;imuOK=initIMU();healthyTicks=0;
  lastGoodIMU=millis();imuBadInWindow=0;imuWindowAt=millis();syncEncoders();
  transition(imuOK?State::IDLE:State::FAULT);Serial.println("# Route cleared. CAL and known-pose ZERO required if uncertain.");return;
 }
 if(busy()||route){Serial.println("# Active task: PAUSE/RECOVER/RETRY/RESUME or STOP.");return;}
 if(state==State::FAULT){Serial.println("# Fault latched: fix cause then CLEAR.");return;}
 if((!strcmp(c,"CAL")||!strcmp(c,"PREP"))&&count==1){noiseLog=false;calibrate();return;}
 if(!strcmp(c,"NOISE")&&count==2&&!armed&&(a==0||a==1)){
  noiseLog=a==1;noiseL=noiseR=noiseGyro=0;noiseSamples=noiseBad=0;
  Serial.println("# NOISE,ms,samples,abs_left_mm,abs_right_mm,peak_gyro_deg_s,bad_imu,quiet_ms");return;
 }
 if(!strcmp(c,"PWM")&&count==3&&!armed&&a>=minDrivePWM&&b>=minTurnPWM&&a<=cfg::PWM_LIMIT&&b<=cfg::PWM_LIMIT){
  drivePWM=a;turnPWM=b;Serial.println("# PWM limits updated in RAM.");return;
 }
 if(!strcmp(c,"MINPWM")&&count==3&&!armed&&a>=0&&b>=0&&a<=drivePWM&&b<=turnPWM){
  minDrivePWM=a;minTurnPWM=b;Serial.println("# Friction minimums updated in RAM.");return;
 }
 if(!strcmp(c,"START")&&count==1){
  if(!allPoints()||!armChecks()){Serial.println("# START needs five points, PREP/CAL completed, confirmed healthy hardware and known origin/+X pose.");return;}
  noiseLog=false;armed=true;invoke("RUN");return;
 }
 if(!strcmp(c,"ZERO")&&count==1&&!armed){resetPose(0,0,0);Serial.println("# ZERO: must physically be at origin facing +X.");return;}
 if((!strcmp(c,"GAIND")||!strcmp(c,"GAINH"))&&count==4&&!armed&&a>=0&&b>=0&&d>=0&&a<=5000&&b<=5000&&d<=5000){
  nav::PID &p=!strcmp(c,"GAIND")?distPID:heading;p.kp=a;p.ki=b;p.kd=d;p.reset();Serial.println("# Gains changed in RAM; copy to Config.h for reset persistence.");return;
 }
 if(!strcmp(c,"WP")&&count==4&&a>=1&&a<=5&&a==floorf(a)&&inside(b/100,d/100)){
  points[(int)a-1]={b/100,d/100};pointsReady[(int)a-1]=true;Serial.printf("# P%d %.2f %.2f cm\n",(int)a,b,d);return;
 }
 if(!strcmp(c,"PLAN")&&count==1){if(allPoints())printPlan();else Serial.println("# Enter five points");return;}
 if(!strcmp(c,"ARM")&&count==1){
  if(!armChecks()){Serial.println("# ARM rejected: hardware lock, calibration, IMU/PWM, pose, stillness or STOP.");return;}
  noiseLog=false;armed=true;Serial.println("# ARMED awaiting command.");return;
 }
 if(!armed){Serial.println("# Not armed. CAL/ARM after hardware setup.");return;}
 if((!strcmp(c,"TURN")||!strcmp(c,"ROTATE")||!strcmp(c,"DRIVE")||!strcmp(c,"RUN"))&&!armChecks()){Serial.println("# Motion rejected: health/pose/stillness check failed.");return;}
 if((!strcmp(c,"TURN")||!strcmp(c,"ROTATE"))&&count==2&&fabsf(a)<=360){
  route=false;taskAt=millis();goalHeading=wrap((!strcmp(c,"ROTATE")?pose.yaw:0)+a*DEG);transition(State::TURN);return;
 }
 if(!strcmp(c,"DRIVE")&&count==2&&fabsf(a)<=240&&fabsf(a)>0){
  float m=a/100;
  if(!inside(pose.x+m*cosf(pose.yaw),pose.y+m*sinf(pose.yaw))){Serial.println("# Endpoint outside bounds");return;}
  route=false;taskAt=millis();driveOrigin={pose.x,pose.y};driveDistance=m;goalHeading=pose.yaw;transition(State::DRIVE);return;
 }
 if(!strcmp(c,"RUN")&&count==1&&allPoints()){
  printPlan();route=true;leg=0;lastStopLeg=-1;recoverySelection=0;runAt=millis();transition(State::COUNTDOWN);
  Serial.println("# START in 5 seconds. Already at origin facing +X; hands clear.");return;
 }
 Serial.println("# Invalid command/range or missing points. HELP lists commands.");
}
static void invoke(const char *text){char b[96];snprintf(b,sizeof b,"%s",text);command(b);}
static void serialInput(){
 static char buffer[96];static unsigned used=0;static bool overflow=false;
 for(int n=0;n<64&&Serial.available();n++){
  char c=Serial.read();
  if(c=='\n'){
   if(overflow)Serial.println("# Line too long; discarded");else{buffer[used]=0;command(buffer);}
   used=0;overflow=false;
  }else if(used<sizeof(buffer)-1&&!overflow)buffer[used++]=c;else overflow=true;
 }
}
static void controlButton(){
 if(cfg::CONTROL_PIN<0)return;
 static bool previous=false,stable=false,pausedOnPress=false;
 static uint32_t changedAt=0,pressedAt=0;
 bool down=digitalRead(cfg::CONTROL_PIN)==LOW;uint32_t now=millis();
 if(down!=previous){previous=down;changedAt=now;}
 if(down==stable||uint32_t(now-changedAt)<30)return;
 stable=down;
 if(down){
  pressedAt=now;pausedOnPress=busy()&&!paused;
  if(pausedOnPress){if(state==State::CALIBRATING)invoke("STOP");else pauseMotion();}
  return;
 }
 if(pausedOnPress)return; // same press cannot both pause and recover/resume
 uint32_t duration=now-pressedAt;
 if(paused&&route){
  if(duration>=2000){
   if(!handling){beginHandling();return;}
   int k=recoverySelection;
   recoverAt(k<5?order[k]+1:0,0); // physically align +X for button recovery
  }else if(duration>=600){
   recoverySelection=recoverySelection>0?recoverySelection-1:leg;recoveryBlinkAt=millis();
   Serial.printf("# Recovery selection P%d; LED pulses ID (home=6). No pose change.\n",recoverySelection<5?order[recoverySelection]+1:0);
  }else invoke("RESUME");
 }else if(!busy()&&state!=State::FAULT){
  if(!allPoints()){Serial.println("# Button start needs all five source/Serial points.");return;}
  invoke("START");
 }
}
void robotSetup(){
 if(cfg::L_EN>=0)pinMode(cfg::L_EN,OUTPUT);
 if(cfg::R_EN>=0)pinMode(cfg::R_EN,OUTPUT);
 zeroPWM();
 if(cfg::STOP_PIN>=0)pinMode(cfg::STOP_PIN,INPUT_PULLUP);
 if(cfg::CONTROL_PIN>=0)pinMode(cfg::CONTROL_PIN,INPUT_PULLUP);
 if(cfg::LED_PIN>=0)pinMode(cfg::LED_PIN,OUTPUT);
 Serial.begin(115200);
 bool p1=ledcAttach(cfg::L_RPWM,cfg::PWM_HZ,cfg::PWM_BITS);
 bool p2=ledcAttach(cfg::L_LPWM,cfg::PWM_HZ,cfg::PWM_BITS);
 bool p3=ledcAttach(cfg::R_RPWM,cfg::PWM_HZ,cfg::PWM_BITS);
 bool p4=ledcAttach(cfg::R_LPWM,cfg::PWM_HZ,cfg::PWM_BITS);
 pwmOK=p1&&p2&&p3&&p4;zeroPWM();
 for(int p:{cfg::ENC_L_A,cfg::ENC_L_B,cfg::ENC_R_A,cfg::ENC_R_B})pinMode(p,INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(cfg::ENC_L_A),encoderL,CHANGE);
 attachInterrupt(digitalPinToInterrupt(cfg::ENC_R_A),encoderR,CHANGE);
 Wire.begin(cfg::SDA_PIN,cfg::SCL_PIN);Wire.setClock(400000);Wire.setTimeOut(3);
 imuOK=initIMU();lastGoodIMU=millis();imuWindowAt=millis();
 if(cfg::SOURCE_POINTS_READY)for(int i=0;i<5;i++){
  points[i]={cfg::POINTS_CM[i][0]/100,cfg::POINTS_CM[i][1]/100};pointsReady[i]=inside(points[i].x,points[i].y);
 }
 syncEncoders();help();
 if(!imuOK||!pwmOK)fault("initial IMU/PWM setup failed");
 else Serial.println("# IDLE motors disabled; CAL/ZERO, verify hardware before ARM.");
}
void robotLoop(){
 if(stopPressed()&&state!=State::FAULT)fault("STOP input");
 controlButton();serialInput();
 uint32_t now=micros(),elapsed=now-lastMicros;
 if(elapsed>=cfg::CONTROL_US){
  lastMicros=now;
  if(elapsed>100000){
   // Missing integration time makes pose uncertain even while disarmed.
   if(!handling)fault("control loop overrun",true);
   syncEncoders();
  }else tick(elapsed*1e-6f); // keep observing sensors while paused/faulted
 }
 if(uint32_t(millis()-lastReport)>=100){lastReport=millis();telemetry();}
 if(paused&&route){
  unsigned pulses=recoverySelection<5?order[recoverySelection]+1:6;
  uint32_t phase=uint32_t(millis()-recoveryBlinkAt)%(pulses*400+1600);
  led(phase<pulses*400 && phase%400<180);
 }else if(state==State::FAULT)led((millis()/200)%2);
 else if(state==State::CALIBRATING||state==State::COUNTDOWN||state==State::RESUMING)led((millis()/500)%2);
}
