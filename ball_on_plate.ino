#include <Ball_On_Plate.h>
/******************** CONSTANTS ********************/
/* DEBUG OPTIONS */
#define PERFORMANCE_CHECK 0
#define RECORD_PID_E OFF
#define RECORD_PID_OUTPUT OFF
#define RECORD_BALL_POSITION ON
#define RECORD_X OFF
/* TOUCHPAD OPTIONS */
#define FILTER_POSITION_BUT1_5HZ ON // OFF
#define FILTER_POSITION_BUT1_2HZ OFF // OFF
#define FILTER_POSITION_BUT2 OFF // OFF
#define X_OFFSET 518
#define Y_OFFSET 500
#define X_SCALING 11.5/425 // 40 bins per cm approximately.
#define Y_SCALING 8.75/400
#define D_SAMPLE_TH 40 // sample again if sample variation is bigger than ~1cm 
#define MAX_SMP 10 // max sampling attempts
/* CONTROL OPTIONS */
#define T_MSEC 20 // sample time [msec] for loop period calculus.
#define T_SEC 0.02           // [sec]. sample time
#define KP -0.6
#define KI -0.01
#define KD -0.2
#define SOFTKP -0.5
#define SOFTKI -0.05
#define SOFTKD 0
#define LEAD_A 0.2
#define LEAD_B 1
#define LEAD_C 0.1
#define PID_ADAPTIVE_EN 0 // enable select soft pid coefficients when the error is under threshold.
#define PID_DEAD_ZONE_EN 0 // disable the derivative component around the target position
#define PID_LEAD_EN 0 // apply lead compensator
#define PID_FILTER_EN 0 // filter the derivative component ud, through 1st order iir.
#define PID_RADIUS_TH 1 // [cm]
#define PID_VELOCITY_TH 1 // [cm]
#define PID_TAU 0.2 // iir coef of pid.ud filter
#define FILTER_CONTROL OFF
/* STEWART PLATFORM OPTIONS */
#define SEARCH_ERR_TH 0.2 // [mm]
#define SEARCH_MAX_ITER 10 // for bad resolution presentation use 3
/* BUTTERWORTH FILTERS */
const float fc2a1 = -0.7757;
const float fc2b[2] = {0.1122,0.1122};
const float fc5a1 = -0.5095;
const float fc5b[2] = {0.2452,0.2452};
const float but2a[3] = {1, -1.1430, 0.4128};
const float but2b[3] = {0.0675,0.1349,0.0675};
const float screen_offset[2] = {X_OFFSET,Y_OFFSET};
const float screen_scaling[2] = {X_SCALING,Y_SCALING};
const int touchpad_pins[4] = {A0,A1,A2,A3};
const int motor_pins[6] = {3,5,6,9,10,11};
const float motor_ang_offset[6] = {88,89,85,92,97,90}; //initial motor's angles [deg]
// Platform constants:
// Motor's spindles coordinates (base system):
const float b[6][2] = { {-83.2079,-18.4797},{-57.6079,-62.8203},{57.6079,-62.8203},{83.2079,-18.4797},{25.6,81.3},{-25.6,81.3}};
// Plate joints to the rods (plate system):
// old1: const float p[6][2] = { {-89,0},{-44.5,-77.0763},{44.5,-77.0763},{89,0},{44.5,77.0763},{-44.5,77.0763}};
// old2: const float p[6][2] = { {-88,39},{-11,-95},{11,-95},{88,39},{77,57},{-77,57}};
const float p[6][2] = {{-58,26},{-6,-64},{6,-64},{58,26},{52,36},{-52,36}};
const float T[3] = {0,0,154};
// Motor's spindle angle relative to positive x axis:
const float beta[6] = {2*PI/3, 2*PI/3, -2*PI/3, -2*PI/3, 0, 0};
// cos\sin _beta, take care of odd motors(idx = 0,2,4), and add PI to beta in trig calculus. *(-1) in brief.

/******************** COMPONENTS ********************/
Touchpad touchpad = Touchpad(touchpad_pins,screen_offset,screen_scaling,D_SAMPLE_TH,MAX_SMP);
StewartPlatform platform = StewartPlatform(motor_pins,motor_ang_offset, b, p, T, beta);
Pid pid = Pid(T_SEC);
// position filters:
Butterworth1 but1_5hz= Butterworth1(fc5a1,fc5b);
Butterworth1 but1_2hz= Butterworth1(fc2a1,fc2b);
Butterworth2 but2_5hz = Butterworth2(but2a,but2b);

Butterworth1 control_filter = Butterworth1(fc2a1,fc2b);
/********************  GLOBALS  ********************/
float posSample[2] = {0};
float plate_angle[2]; //[rad] TODO:check if not [deg]
float ball_position[2]; // [0] - x, [1] - y. [m] 
float ref_position[2] = {0,-3}; //reference to the final position of the ball. input of the system [m]
unsigned long time, last_time, print_time, last_print_time;
float motor_angle_cmd_vec[6]={0}; //[deg]
    

void setup(){
  print_time = 0; last_print_time = 0;
  int i; 
  last_time =0;
  platform.init_motors();
  platform.set_search_err_th(SEARCH_ERR_TH);
  platform.set_search_max_iter(SEARCH_MAX_ITER);
  pid.set_pid_coef(KP,KI,KD);
  pid.set_pid_soft_coef(SOFTKP,SOFTKI,SOFTKD);
  pid.set_lead_coef(LEAD_A,LEAD_B,LEAD_C);
  pid.set_ud_filter_coef(PID_TAU);
  pid.set_th(PID_RADIUS_TH,PID_VELOCITY_TH);
  pid.enable_adaptive(PID_ADAPTIVE_EN);
  pid.enable_dead_zone(PID_DEAD_ZONE_EN);
  pid.enable_lead(PID_LEAD_EN);
  pid.enable_filter(PID_FILTER_EN);
  pid.record_e(RECORD_PID_E);
  pid.record_output(RECORD_PID_OUTPUT);
  
  Serial.begin(9600);

  if(PERFORMANCE_CHECK){
    Serial.print("Analizing cpu performance for ERR_TH = ");Serial.print(SEARCH_ERR_TH);Serial.print(" in [usec]:");
    Serial.println("---------------------------------------------------------------------------------");
    Serial.println("Max_iter|Read position|Filter position| Pid |Angles transform|Set angles|Total time");
    Serial.println("--------|-------------|---------------|-----|----------------|----------|----------");
    
    for(int iter = 0;iter<SEARCH_MAX_ITER;iter++){
      platform.set_search_max_iter(iter+1);
      int t[4][10] = {{0},{0},{0},{0}};
      for(int n = 0; n<10;n++){
        last_time = micros();
        touchpad.read_ball_position(posSample);
        time = micros();t[0][n]= time - last_time;
        
        last_time = micros();
        pid.get_ctrl(ref_position,ball_position,plate_angle);
        time = micros();t[2][n]= time - last_time;
      
        last_time = micros();
        platform.get_motors_angles(plate_angle[0], -plate_angle[1], motor_angle_cmd_vec);
        time = micros();t[3][n]= time - last_time;
        
        last_time = micros();
        platform.set_motors_angles(motor_angle_cmd_vec);
        time = micros();t[4][n]= time - last_time;
      }
      int avg[4]={0};
      for(int j = 0; j<4;j++){
        for(int n = 0; n<10;n++){
          avg[j]+=t[j][n]/10;
        }
      }
      
      Serial.print(iter+1);Serial.print("          ");
      Serial.print(avg[0]); Serial.print("          ");
      Serial.print(avg[1]); Serial.print("             ");
      Serial.print(avg[2]); Serial.print("       ");
      Serial.print(avg[3]); Serial.print("          ");
      Serial.println(avg[0]+avg[1]+avg[2]+avg[3]);
      
    
    }
  }
    
  delay(1000);
  touchpad.init();

  
}

void loop() {
  touchpad.read_ball_position(ball_position);
  if(FILTER_POSITION_BUT1_5HZ) but1_5hz.filter2d(ball_position,ball_position);
  if(FILTER_POSITION_BUT1_2HZ) but1_2hz.filter2d(ball_position,ball_position);
  if(FILTER_POSITION_BUT2)     but2_5hz.filter2d(ball_position,ball_position);
  if(RECORD_X)               Serial.print(ball_position[0]); Serial.print(" "); // record ball x coordinate before iir
  if(RECORD_BALL_POSITION)  {Serial.println(ball_position[0]);Serial.println(ball_position[1]);}
  pid.get_ctrl(ref_position,ball_position,plate_angle);
  if(FILTER_CONTROL) control_filter.filter2d(plate_angle,plate_angle);
  platform.get_motors_angles(plate_angle[0], -plate_angle[1], motor_angle_cmd_vec);
  platform.set_motors_angles(motor_angle_cmd_vec);
  // Set the cycle period T:
  do {time = millis();} while(time - last_time < T_MSEC);
  last_time = time;
}
