#include <cassert>
#include <iostream>
#include <random>
#include "../Round2_Postman/Robot.cpp"
static void cmd(const char*s){char b[160];snprintf(b,sizeof b,"%s",s);command(b);}
static void baseline(){
 state=State::IDLE;resumeState=faultState=State::IDLE;
 armed=route=paused=poseUncertain=handling=noiseLog=false;
 drivePWM=cfg::DRIVE_PWM_LIMIT;turnPWM=cfg::TURN_PWM_LIMIT;minDrivePWM=cfg::MIN_DRIVE_PWM;minTurnPWM=cfg::MIN_TURN_PWM;pose={};goal={};leg=0;lastStopLeg=-1;
 speedL=speedR=gyro=angularRate=travel=0;requestDrive=requestTurn=outputL=outputR=0;
 pwmOK=imuOK=imuFresh=calibrated=true;bias=0;healthyTicks=cfg::HEALTHY_TICKS_TO_RETRY;
 gpio[cfg::STOP_PIN]=gpio[cfg::CONTROL_PIN]=HIGH;clockUs=1000000;
 stateAt=runAt=pauseAt=lastGoodIMU=imuWindowAt=millis();stillAt=stallAtL=stallAtR=0;imuBadInWindow=0;
 ticksL=ticksR=previousL=previousR=0;Wire.ok=true;Wire.raw=0;Serial.output.clear();Serial.input.clear();
 for(int i=0;i<5;i++){pointsReady[i]=false;points[i]={};order[i]=i;}
 heading={cfg::HEADING_KP,cfg::HEADING_KI,cfg::HEADING_KD};distPID={cfg::DIST_KP,cfg::DIST_KI,cfg::DIST_KD};
 syncEncoders();rest.prime(cfg::SETTLE_MS);taskAt=millis();realigns=0;recoverySelection=0;
}
static void step(){
 clockUs+=10000;
 rest.sample(.01f,speedL*.01f,speedR*.01f,gyro,true,true,cfg::STILL_WINDOW_MS,cfg::STILL_WINDOW_TRAVEL_M,cfg::STILL_TOTAL_TRAVEL_M,cfg::STILL_GYRO_RADPS,cfg::STILL_TOTAL_YAW_RAD);
 control(.01f);actuate(.01f);
 // Simplified first-order wheel plant with friction. It is NOT a model of the actual kit.
 auto target=[](float p){return std::copysign(std::max(0.f,std::fabs(p)-22.f)*.003f,p);};
 speedL+=.12f*(target(outputL)-speedL);speedR+=.12f*(target(outputR)-speedR);
 angularRate=gyro=(speedR-speedL)/cfg::TRACK_M;
 nav::integrate(pose,speedL*.01f,speedR*.01f,gyro,.01f,cfg::TRACK_M,cfg::GYRO_WEIGHT);
 travel+=(speedL+speedR)*.005f;
}
static void simulate(int maximum=60000){
 for(int n=0;n<maximum&&busy();n++)step();
 if(state!=State::DONE){std::cerr<<Serial.output.substr(Serial.output.size()>2000?Serial.output.size()-2000:0)<<"pose "<<pose.x<<","<<pose.y<<" yaw "<<pose.yaw<<"\n";}
 assert(state==State::DONE);
}
static void button(bool down,unsigned ms){gpio[cfg::CONTROL_PIN]=down?LOW:HIGH;controlButton();clockUs+=ms*1000;controlButton();}
int main(){
 baseline();cmd("ARM");assert(armed==cfg::HARDWARE_CONFIRMED);
 if(!cfg::HARDWARE_CONFIRMED){
  for(bool &v:pointsReady)v=true;
  cmd("START");assert(!armed&&state==State::IDLE);
  armed=true;cmd("DRIVE 100");assert(state==State::IDLE);
  std::cout<<"PASS shipping hardware lock blocks ARM/START/movement\n";return 0;
 }
 baseline();cmd("WP 1 nan 0");assert(!pointsReady[0]);cmd("WP 1 50 60");assert(pointsReady[0]);
 cmd("WP 2 1000 0");assert(!pointsReady[1]);cmd("WP 2 20 30 trailing");assert(!pointsReady[1]);
 cmd("WP 3 20 30 40 50");assert(!pointsReady[2]);
 Serial.input=std::string(100,'a')+"\n";serialInput();serialInput();assert(Serial.output.find("Line too long")!=std::string::npos);
 cmd("gaind 100 0 10");assert(distPID.kp==100);cmd("GAIND -1 0 0");assert(distPID.kp==100);
 // Planner optimum verified independently via recursive enumeration.
 std::mt19937 rng(123);std::uniform_real_distribution<float> u(-1.3f,1.3f);
 for(int trial=0;trial<30;trial++){
  nav::Point p[5];for(auto &v:p)v={u(rng),u(rng)};
  for(bool closed:{false,true}){
   float best=1e9f;
   auto dfs=[&](auto &&self,nav::Point last,int used,float cost)->void{
    if(used==31){best=std::min(best,cost+(closed?nav::distance(last,{0,0}):0));return;}
    for(int i=0;i<5;i++)if(!(used&(1<<i)))self(self,p[i],used|(1<<i),cost+nav::distance(last,p[i]));
   };
   dfs(dfs,{0,0},0,0);int result[5];assert(fabs(nav::plan(p,result,closed)-best)<1e-5);
  }
 }
 nav::Point p[5]={{1,0},{2,0},{3,0},{4,0},{5,0}};int o[5];
 assert(fabs(nav::plan(p,o,false)-5)<1e-5);assert(fabs(nav::plan(p,o,true)-10)<1e-5);
 nav::Point zeros[5]={};assert(nav::plan(zeros,o,true,1)==0); // no phantom turns at duplicate points
 nav::Pose q;nav::integrate(q,1,1,0,1,.15,.9);assert(fabs(q.x-1)<1e-5&&q.y==0);
 q={};nav::integrate(q,0,0,1,1,.15,1);assert(fabs(q.yaw-1)<1e-5);
 q={};nav::integrate(q,-.1,.1,0,1,.2,0);assert(fabs(q.yaw-1)<1e-5);
 // Atomic x2 decoding: forward A edges count same sign, reverse flips sign.
 baseline();gpio[cfg::ENC_L_A]=gpio[cfg::ENC_L_B]=1;encoderL();
 gpio[cfg::ENC_L_A]=gpio[cfg::ENC_L_B]=0;encoderL();assert(ticksL==2*cfg::ENC_L_SIGN);
 gpio[cfg::ENC_L_A]=1;encoderL();assert(ticksL==cfg::ENC_L_SIGN);
 // Synthetic control with friction, inertia and wrap boundary.
 for(const char *s:{"TURN 90","TURN -179","ROTATE 180","DRIVE 100","DRIVE -100"}){
  baseline();armed=true;cmd(s);simulate(20000);
 }
 baseline();pose.yaw=179*DEG;armed=true;cmd("TURN -179");control(.01f);assert(requestTurn>0);simulate();
 // Dwell evidence tolerates one dropped frame without counting missing time.
 baseline();armed=route=true;goal={};leg=0;transition(State::DWELL);
 for(int i=0;i<240;i++){clockUs+=10000;tick(.01f);}assert(leg==0&&rest.quietMs>=2300);
 Wire.ok=false;clockUs+=10000;tick(.01f);assert(leg==0&&rest.quietMs>=2300);
 Wire.ok=true;for(int i=0;i<9;i++){clockUs+=10000;tick(.01f);}assert(leg==0);
 for(int i=0;i<11;i++){clockUs+=10000;tick(.01f);}assert(leg==1);
 // One encoder tick does not destroy the full dwell; creeping does.
 baseline();armed=route=true;goal={};leg=0;transition(State::DWELL);
 for(int i=0;i<260;i++){if(i==200)++ticksL;clockUs+=10000;tick(.01f);}assert(leg==1);
 baseline();armed=route=true;goal={};leg=0;transition(State::DWELL);
 for(int i=0;i<300;i++){if(i%5==0){++ticksL;++ticksR;}clockUs+=10000;tick(.01f);}assert(leg==0);
 // Practical arrival is radial and its dwell exit is strictly larger.
 baseline();armed=route=true;state=State::DRIVE;goal={1,0};pose={1,.042f,0};goalHeading=0;
 control(.01f);assert(state==State::DWELL);control(.01f);assert(state==State::DWELL);
 pose.x=1.08f;pose.y=0;control(.01f);assert(state==State::DRIVE);control(.01f);assert(requestDrive<0&&fabs(requestTurn)<1e-5);
 pose={1,.08f,0};control(.01f);assert(state==State::REALIGN&&route);
 simulate(); // completes current target plus the remaining zero-valued targets/home
 // Repeated realignments do not reset the same leg's deadline.
 baseline();armed=route=true;state=State::DRIVE;goal={1,0};pose={1,.08f,0};
 uint32_t before=taskAt;control(.01f);assert(taskAt==before);
 clockUs+=90001000;control(.01f);assert(state==State::FAULT&&route);
 assert(nav::friction(-2,1,30)==-2);assert(nav::friction(2,1,30)==30);
 // Sensor loss uses encoder increments, does not reuse stale gyro or cancel route.
 baseline();armed=route=true;state=State::DRIVE;goal={1,0};goalHeading=0;
 Wire.ok=false;ticksR+=10;clockUs+=10000;tick(.01f);
 assert(route&&state!=State::FAULT&&pose.yaw>0&&!imuFresh);
 for(int i=0;i<20;i++){clockUs+=10000;tick(.01f);}assert(state==State::FAULT&&route&&!armed);
 Wire.ok=true;for(int i=0;i<25;i++){clockUs+=10000;tick(.01f);}assert(imuOK&&healthyTicks>=20&&state==State::FAULT);
 // STOP is immediate; recovery cannot skip an unvisited point; home recovery supported.
 baseline();armed=route=true;state=State::DRIVE;leg=2;points[1]={.2f,.3f};
 cmd("PAUSE");assert(paused);cmd("RECOVER 5 0");assert(leg==2);
 cmd("RECOVER 2 90");assert(paused&&state==State::DWELL&&leg==1&&fabs(pose.yaw-nav::kPi/2)<1e-5);
 if(cfg::HARDWARE_CONFIRMED){
  rest.prime(cfg::SETTLE_MS);cmd("RESUME");assert(state==State::RESUMING&&!paused);control(.01f);assert(requestDrive==0&&requestTurn==0);
  clockUs+=2900000;control(.01f);assert(state==State::RESUMING);
  clockUs+=110000;rest.prime(cfg::SETTLE_MS);control(.01f);assert(state==State::DWELL); // recovery heading retained
 }
 cmd("STOP");assert(!armed&&!route&&state==State::IDLE);
 baseline();armed=route=true;state=State::DRIVE;leg=5;cmd("PAUSE");cmd("RECOVER 0 -90");assert(leg==5&&fabs(pose.yaw+nav::kPi/2)<1e-5);
 // Fault recovery, no automatic motor restart, uncertain pose must be reset knowingly.
 fault("encoder glitch",true);cmd("RETRY");assert(state==State::FAULT&&poseUncertain);
 cmd("RECOVER 0 0");assert(!poseUncertain&&state==State::FAULT);
 if(cfg::HARDWARE_CONFIRMED){rest.prime(cfg::SETTLE_MS);cmd("RETRY");assert(state==State::DWELL&&paused&&armed);rest.prime(cfg::SETTLE_MS);cmd("RESUME");assert(state==State::RESUMING);}
 if(cfg::STOP_PIN>=0){baseline();armed=route=true;state=State::TURN;gpio[cfg::STOP_PIN]=LOW;robotLoop();assert(state==State::FAULT&&!armed&&route);}
 // Stall detection, hardware motor outputs and direction changes bounded.
 baseline();armed=true;state=State::DRIVE;requestDrive=70;
 for(int i=0;i<300&&state!=State::FAULT;i++){clockUs+=10000;actuate(.01f);}
 assert(state==State::FAULT);if(cfg::L_EN>=0)assert(gpio[cfg::L_EN]==LOW&&gpio[cfg::R_EN]==LOW);
 baseline();armed=true;state=State::DRIVE;requestDrive=70;speedL=speedR=.1f;
 actuate(.01f);assert(fabs(outputL)<=3.01f);assert(pwm[cfg::L_LPWM]==0);
 motorPWM(cfg::L_RPWM,cfg::L_LPWM,cfg::L_EN,-40);assert(pwm[cfg::L_RPWM]==0&&pwm[cfg::L_LPWM]==40);
 // Overrun while disarmed still flags pose uncertainty; 32-bit microsecond wrap works.
 baseline();lastMicros=(uint32_t)micros()-200000;robotLoop();assert(state==State::FAULT&&poseUncertain);
 baseline();clockUs=(1ULL<<32)+5000;lastMicros=uint32_t(clockUs-10000);lastGoodIMU=millis();imuWindowAt=millis();robotLoop();assert(state!=State::FAULT);
 // Button first press pauses; release cannot instantly resume, long NEXT press recovers.
 baseline();armed=route=true;state=State::DRIVE;leg=1;lastStopLeg=0;points[0]={.4f,.2f};
 button(true,40);assert(paused);button(false,40);assert(paused&&state==State::DRIVE);
 button(true,40);clockUs+=2100000;button(false,40);assert(paused&&handling&&poseUncertain&&!armed);
 button(true,40);clockUs+=2100000;button(false,40);assert(paused&&!handling&&leg==1&&state==State::DWELL&&pose.yaw==0);
 for(int i=0;i<25;i++){clockUs+=10000;tick(.01f);}
 cmd("SELECT 1");assert(recoverySelection==0);cmd("RECOVER 1 0");assert(leg==0);
 if(cfg::HARDWARE_CONFIRMED){rest.prime(cfg::SETTLE_MS);button(true,40);button(false,40);assert(state==State::RESUMING&&!paused);}
 baseline();route=true;paused=true;state=State::FAULT;runAt=millis();clockUs+=600001000;control(.01f);assert(!route&&state==State::FAULT);
 // Calibration responds to STOP and retries movement without relaxing variance checks.
 baseline();cmd("CAL");assert(state==State::CALIBRATING&&!armed);
 for(int i=0;i<100;i++){clockUs+=10000;tick(.01f);}
 cmd("STOP");assert(state==State::IDLE&&!calibrated);
 cmd("CAL");
 for(int i=0;i<750;i++){
  if(i==100)ticksL+=3;
  clockUs+=10000;tick(.01f);
 }
 assert(state==State::IDLE&&calibrated);
 baseline();cmd("CAL");
 for(int i=0;i<2100&&state!=State::FAULT;i++){
  if(i%50==0)ticksL+=3;
  clockUs+=10000;tick(.01f);
 }
 assert(state==State::FAULT&&!calibrated&&!armed);
 // An encoder spike is not turned into invented distance by clamping.
 baseline();armed=route=true;state=State::DRIVE;ticksL+=10000;
 clockUs+=10000;tick(.01f);assert(state==State::FAULT&&poseUncertain&&pose.x==0);
 // Full routes including duplicate/origin/behind targets. No external pose corrections.
 for(int trial=0;trial<20;trial++){
  baseline();
  for(int i=0;i<5;i++){points[i]={u(rng),u(rng)};pointsReady[i]=true;}
  if(trial==0){points[1]=points[0];points[2]={0,0};points[3]={-1,0};}
  armed=true;cmd("RUN");simulate();assert(hypot(pose.x,pose.y)<cfg::HOME_ACCEPT_M+cfg::DWELL_MARGIN_M);
 }
 // Heading-aware drive slowdown, reverse symmetry and unchanged braking torque.
 baseline();armed=true;state=State::DRIVE;requestDrive=60;driveError=1;turnError=.6f;
 speedL=speedR=.1f;
 for(int i=0;i<100;i++)actuate(.01f);
 assert(fabs(outputL-60*cosf(.6f))<.001f&&fabs(outputL-outputR)<.001f);
 requestDrive=-60;driveError=-1;
 for(int i=0;i<100;i++)actuate(.01f);
 assert(fabs(outputL+60*cosf(.6f))<.001f);
 requestDrive=-10;driveError=1;
 for(int i=0;i<100;i++)actuate(.01f);
 assert(fabs(outputL+10)<.001f); // do not reduce or boost damping torque
 // Reaching the target's projected distance is insufficient if lateral error is large.
 baseline();armed=route=true;state=State::DRIVE;goal={1,0};pose={.995f,.20f,0};goalHeading=0;
 control(.01f);assert(state==State::REALIGN);
 // PLAN/WP cannot change active or paused route semantics; malformed recovery is rejected.
 baseline();armed=route=true;state=State::DRIVE;leg=1;points[1]={.4f,.5f};
 cmd("PAUSE");int beforeOrder[5];std::copy(order,order+5,beforeOrder);
 cmd("PLAN");cmd("WP 2 100 100");assert(points[1].x==.4f&&std::equal(order,order+5,beforeOrder));
 cmd("RECOVER 2 nan");assert(pose.x==0&&pose.y==0&&pose.yaw==0);
 cmd("WP 2.4 20 30");assert(points[1].x==.4f);
 cmd("STATUS");assert(Serial.output.find("Combined V4")!=std::string::npos);
 // End-to-end sensor pipeline: quantised ticks, gyro noise and isolated I2C gaps.
 for(int trial=0;trial<10;trial++){
  baseline();
  for(int i=0;i<5;i++){points[i]={u(rng),u(rng)};pointsReady[i]=true;}
  if(trial==0){points[1]=points[0];points[2]={0,0};}
  armed=true;cmd("RUN");nav::Pose truth;float vl=0,vr=0;double pathL=0,pathR=0;
  for(int n=0;n<60000&&busy();n++){
   auto drive=[](float p){return std::copysign(std::max(0.f,std::fabs(p)-22.f)*.003f,p);};
   vl+=.12f*(drive(outputL)-vl);vr+=.12f*(1.01f*drive(outputR)-vr);
   float dl=vl*.01f,dr=vr*.01f,w=(vr-vl)/cfg::TRACK_M;
   nav::integrate(truth,dl,dr,w,.01f,cfg::TRACK_M,1);
   pathL+=dl;pathR+=dr;
   ticksL=llround(pathL/(nav::kPi*cfg::WHEEL_L_DIAMETER_M/cfg::TICKS_L_PER_WHEEL_REV));
   ticksR=llround(pathR/(nav::kPi*cfg::WHEEL_R_DIAMETER_M/cfg::TICKS_R_PER_WHEEL_REV));
   Wire.raw=(int16_t)lround(w/DEG*131+(int(rng()%5)-2));
   Wire.ok=(n%997!=500);clockUs+=10000;tick(.01f);
  }
  if(state!=State::DONE)std::cerr<<Serial.output<<"sensor plant trial "<<trial<<"\n";
  assert(state==State::DONE);assert(hypot(pose.x,pose.y)<.04f);assert(hypot(truth.x,truth.y)<.06f);
 }
 // V4: calibration/preparation never starts motion; start reuses calibration.
 baseline();cmd("PREP");for(int i=0;i<650;i++){clockUs+=10000;tick(.01f);}
 assert(calibrated&&state==State::IDLE&&!armed&&!route);
 for(bool &v:pointsReady)v=true;
 cmd("START");assert(state==State::COUNTDOWN&&armed&&route&&calibrated);
 baseline();calibrated=false;for(bool &v:pointsReady)v=true;cmd("START");assert(state==State::IDLE&&!armed);
 baseline();for(bool &v:pointsReady)v=true;button(true,40);button(false,40);assert(state==State::COUNTDOWN);
 // Consecutive judge demonstrations keep arming and pose; STOP disarms.
 baseline();cmd("ARM");cmd("TURN 90");simulate();assert(armed);
 cmd("DRIVE 100");assert(state==State::DRIVE);simulate();assert(armed&&pose.y>.9f);
 cmd("STOP");assert(!armed);
 // Runtime tuning is bounded, rejects NaN, and cannot change an active task.
 baseline();cmd("PWM 90 85");assert(drivePWM==90&&turnPWM==85);
 cmd("PWM 255 85");assert(drivePWM==90);
 cmd("MINPWM 30 35");assert(minDrivePWM==30&&minTurnPWM==35);
 cmd("PWM 20 20");assert(drivePWM==90);
 cmd("MINPWM nan 35");assert(minDrivePWM==30);
 cmd("ARM");cmd("PWM 80 80");assert(drivePWM==90);
 // Handling suppresses expected motion faults but invalidates pose and requires reset.
 baseline();armed=route=true;state=State::DRIVE;leg=1;points[1]={.4f,.5f};
 cmd("HANDLE");assert(handling&&paused&&poseUncertain&&!armed);
 Wire.raw=32767;
 for(int i=0;i<50;i++){ticksL+=10000;clockUs+=10000;tick(.01f);}
 assert(state==State::DRIVE&&pose.x==0&&outputL==0);
 cmd("RESUME");assert(handling&&paused);
 cmd("RECOVER 5 0");assert(handling);
 cmd("RECOVER 2 90");assert(!handling&&!poseUncertain&&paused&&state==State::DWELL);
 cmd("RESUME");assert(paused); // saturated sensor still blocks
 Wire.raw=0;for(int i=0;i<30;i++){clockUs+=10000;tick(.01f);}
 cmd("RESUME");assert(state==State::RESUMING&&!paused);
 // The reset does not hide a persistent hardware fault after handling.
 baseline();armed=route=true;state=State::DRIVE;cmd("HANDLE");cmd("RECOVER 1 0");Wire.ok=false;
 for(int i=0;i<30;i++){clockUs+=10000;tick(.01f);}cmd("RESUME");assert(state==State::FAULT&&paused);
 Wire.ok=true;for(int i=0;i<30;i++){clockUs+=10000;tick(.01f);}cmd("RESUME");assert(state==State::RESUMING&&!paused);
 baseline();cmd("NOISE 1");ticksL+=3;clockUs+=10000;tick(.01f);telemetry();
 assert(Serial.output.find("NOISE,")!=std::string::npos);cmd("ARM");assert(!noiseLog);
 // Disabled pins must never be driven, and GPIO4 belongs only to PWM in profile 1.
 baseline();gpio.erase(-1);zeroPWM();motorPWM(cfg::L_RPWM,cfg::L_LPWM,cfg::L_EN,20);
 assert(gpio.count(-1)==0);robotSetup();assert(gpio.count(-1)==0);
 if(cfg::WIRING_PROFILE==1){assert(cfg::STOP_PIN==-1&&cfg::R_LPWM==4);gpio[4]=LOW;assert(!stopPressed());}
 std::cout<<"PASS hardware_lock="<<!cfg::HARDWARE_CONFIRMED<<": planner, wrap, signed control, dwell, overshoot, faults, recovery, button, PWM, calibration, alignment slowdown, 20 ideal-feedback routes and 10 quantised-sensor routes.\n";
}
