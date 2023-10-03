#include <ros.h>
#include <Arduino.h>
#include <std_msgs/String.h>
#include <geometry_msgs//PoseStamped.h>
#include <iostream>
#include <map>
#include <SD.h>

/* 
================================= [CONTROL NOTE] =================================
 |
 | flight mode (ch5)
 |
    |
    | arming (ch6)
    |

  throttle (ch3)             pitch (ch2)
        |                       |
      —— —— yaw (ch4)         —— —— roll (ch1) 
        |                       |     
  Radio channel:
  channel 1; cyclic roll(遙控器右搖桿的左右)
  channel 2; cyclic pitch(遙控器右搖桿的上下)
  channel 3; throttle(遙控器左搖桿的上下)
  channel 4; cyclic yaw(遙控器左搖桿的左右)
  channel 5; flying mode
  channel 6; arming

  Functions:
  1. kbCallback() : get key from keyboard.
  2. radio_calibration(): get the initial full pulse and frequency of radio.
  3. RadioInput(): get the radio input from PIN 1~6.
  3. Output(): analogWrite the output array.
  4. 
====================================== [END] ======================================
*/


// ======================================== Define some variables =======================================
// PINs I/O dictionary
int action_percentage[13] = {35, 37, 40, 40, 42, 46, 50, 54, 58, 60, 60, 63, 65};
int action_percentage_length = sizeof(action_percentage) / sizeof(action_percentage[0]);
int middle_pos = action_percentage_length / 2; // 5

typedef struct { 
  int PIN;
  String name;
  int percentage_pos;
} Pin_dictionary;

Pin_dictionary pin_dict[]{
  {0, "null",     0},
  {1, "roll",     0},
  {2, "pitch",    0},
  {3, "throttle", 0},
  {4, "yaw",      0},
  {5, "mode",     0},
  {6, "arming",   0},
  {7, "roll",     middle_pos},
  {8, "pitch",    middle_pos},
  {9, "throttle", middle_pos},
  {10, "yaw",     middle_pos},
  {11, "mode",    middle_pos},
  {12, "arming",  middle_pos}
};

int PIN_roll_input = 1;
int PIN_pitch_input = 2;
int PIN_throttle_input = 3;
int PIN_yaw_input = 4;
int PIN_mode_input = 5;
int PIN_arming_input = 6;
int PIN_roll_output = 7;
int PIN_pitch_output = 8;
int PIN_throttle_output = 9;
int PIN_yaw_output = 10;
int PIN_mode_output = 11;
int PIN_arming_output = 12;

// Action index dictionary

typedef struct { 
  int right = 0;           
  int rrf = 1;            
  int right_forward = 2;  
  int frf = 3;            
  int forward = 4;       
  int flf = 5;      
  int left_forward = 6;   
  int llf = 7; 
  int left = 8;   
  int llb = 9;  
  int left_backward = 10;       
  int blb = 11;      
  int backward = 12;          
  int brb = 13;           
  int right_backward = 14;            
  int rrb = 15;  
  int up = 16;            
  int down = 17;            
  int turnLeft = 18;            
  int turnRight = 19;           
} Action_index;

const Action_index action_index;

// Action dictionary
typedef struct { 
  String name;
  int PIN[2];
  int direction[2];
} Action_dict;

const Action_dict action_dict[]{
  {"right",                   {PIN_roll_input, 0},               {2, 0}}, // 0: right
  {"right,right,forward",     {PIN_roll_input, PIN_pitch_input}, {2, -1}}, // 1: right right forward
  {"right,forward",           {PIN_roll_input, PIN_pitch_input}, {2, -2}}, // 2: right forward
  {"forward,right,forward",   {PIN_roll_input, PIN_pitch_input}, {1, -2}}, // 3: forward right forward
  {"foward",                  {PIN_pitch_input, 0},              {-2, 0}}, // 4: foward
  {"forward,left,forward",    {PIN_roll_input, PIN_pitch_input}, {-1, -2}}, // 5: forward left forward
  {"left,forward",            {PIN_roll_input, PIN_pitch_input}, {-2, -2}}, // 6: left forward
  {"left,left,forward",       {PIN_roll_input, PIN_pitch_input}, {-2, -1}}, // 7: left left forward
  {"left",                    {PIN_roll_input, 0},               {-2, 0}}, // 8: left
  {"left,left,backward",      {PIN_roll_input, PIN_pitch_input}, {-2, 1}}, // 9: left left backward
  {"left,backward",           {PIN_roll_input, PIN_pitch_input}, {-2, 2}}, // 10: left backward
  {"backward,left,backward",  {PIN_roll_input, PIN_pitch_input}, {-1, 2}}, // 11: backward left backward
  {"backward",                {PIN_pitch_input, 0},              {2, 0}}, // 12: backward
  {"backward,right,backward", {PIN_roll_input, PIN_pitch_input}, {1, 2}}, // 13: backward right backward
  {"right,backward",          {PIN_roll_input, PIN_pitch_input}, {2, 2}}, // 14: right backward
  {"right,right,backward",    {PIN_roll_input, PIN_pitch_input}, {2, 1}}, // 15: right right backward
  {"up",                      {PIN_throttle_input, 0},           {2, 0}}, // 16: up 
  {"down",                    {PIN_throttle_input, 0},           {-2, 0}}, // 17: down 
  {"turn left",               {PIN_yaw_input, 0},                {-2, 0}}, // 18: turn left
  {"turn right",              {PIN_yaw_input, 0},                {2, 0}}, // 19: turn right
};


// Some variables
int resolution = 12;
int full_pulse = 0;
int radio_input[13]; // idx 1 to 6 mean radio input pin 1~6 (channel 1~6)
int minmax_radio_input[13] = {0, 1500, 0, 1500, 0, 1500, 0, 1500, 0, 1500, 0, 1500, 0};
// int minmax_radio_input[13] = {0, 1100, 1921, 1109, 1930, 1109, 1930, 1110, 1930, 969, 2070, 969, 2070}; 
int radio_channel_range[7]; // idx 1 to 6 mean radio channel full range
int output[13]; // output array to Output() function, idx 7~12 mean pin 7~12 (channel 1~6) 
bool listen_to_radio = true;

// ROS node
String key = "0"; // initial key, set to "0"
// double x_current_pos, y_current_pos;
ros::NodeHandle node;
std_msgs::String string_msg;

typedef struct { 
  float x_now_pos = 0.0; // actual position
  float y_now_pos = 0.0;
  float x_current_pos = 0.0; // theory current position (if stable)
  float y_current_pos = 0.0;
  float x_target_pos = 0.0; // target position
  float y_target_pos = 0.0;
} Position_dict;

Position_dict position_dict;
float drift_radius = 0.0;
float drift_theta = 0.0;
float drift_threshold_low = 0.25; // m
// float drift_threshold_high = 0.5; // m
float path_threshold = 0.25; // m
bool print_pose = true;
String move_type = "Holding";
float move_length = 0.6; // m

// PID control
float Kp = 0.5;
float Ki = 0.5;
float Kd = 0.5;

// Record to SD
File file;
const char* data_name = "230915_1.txt";
bool record_flag = false;

// ======================================== Functions ======================================== 
// Subscribe keyboard typing
void kbCallback(const std_msgs::String & msg){
  key = msg.data;

  // print state
  // String state = "key: ";
  // state += key;
  // Serial.println(state);
  
  // record to SD
  if (record_flag == true){
      if (key != "0"){
      file.println(key);
    } 
  }
}
ros::Subscriber<std_msgs::String> kbSub("publisher_keyboard", kbCallback);

// Subscribe current pose

void poseCallback(const geometry_msgs::PoseStamped& msg){

    position_dict.x_now_pos = -msg.pose.position.x;
    position_dict.y_now_pos = -msg.pose.position.y;
    drift_radius = sqrt(pow((position_dict.x_now_pos - position_dict.x_current_pos), 2) +
                        pow((position_dict.y_now_pos - position_dict.y_current_pos), 2));

    drift_theta = atan2((position_dict.y_now_pos - position_dict.y_current_pos), 
                        (position_dict.x_now_pos - position_dict.x_current_pos)) * 180 / 3.1415926; // from current to now

    if ( print_pose == true){
      Serial.print("x_now: ");
      Serial.print(position_dict.x_now_pos);
      Serial.print(", y_now: ");
      Serial.print(position_dict.y_now_pos);
      Serial.print(", drift_radius: ");
      Serial.println(drift_radius);
    } 
}
ros::Subscriber<geometry_msgs::PoseStamped> poseSub("online_2d_robot_pose", poseCallback);

// setup radio frequency, resolution and record radio min/max, range
void radio_calibration(){
  
  // Setup Output pins frequency and resolution
  for (int i = 1; i <= 6; i++){
    full_pulse += pulseIn(i, LOW, 20000) + pulseIn(i, HIGH, 20000);
  }
  full_pulse = full_pulse / 6;
  int frequency = 1000000 / full_pulse;

  for (int pin = 7; pin <= 12; pin++){
    analogWriteFrequency(pin,  frequency);
  }
  analogWriteResolution(resolution);
  
  Serial.print("full_pulse: ");
  Serial.print(full_pulse);
  Serial.print(", frequency: ");
  Serial.println(frequency);
  
  // record radio min max
  for (int x = 1; x <= 20; x++){
    for (int pin = 1; pin <= 6; pin++) {
      radio_input[pin] = pulseIn(pin, HIGH, 20000);

      if (radio_input[pin] < minmax_radio_input[2*pin-1]){
        minmax_radio_input[2*pin-1] = radio_input[pin];
      }
      if (radio_input[pin] > minmax_radio_input[2*pin]){
        minmax_radio_input[2*pin] = radio_input[pin];
      }
      delay(120);
    }
    for (int idx = 1; idx <= 12; idx++){
      Serial.print(minmax_radio_input[idx]);
      Serial.print(", ");
    }
    Serial.println();
  }

  //Get radio channel range
  for (int pin=1 ; pin<=6 ; pin++){
    radio_channel_range[pin]=minmax_radio_input[2*pin]-minmax_radio_input[2*pin-1];
    Serial.print("Channel ");
    Serial.print(pin);
    Serial.print(": ");
    Serial.print(radio_channel_range[pin]);
    Serial.print(", ");
  }
  Serial.println();
  Serial.println(" ===================== Radio calibration end ===================== ");

}

void RadioInput(){
  for (int pin = 1; pin <= 6; pin++) {
    radio_input[pin] = pulseIn(pin, HIGH, 20000);
  }
}

void Output(int output[]){
  String state; 
  for (int pin = 7; pin <= 12; pin++) {
    analogWrite(pin, map(output[pin], 0, full_pulse, 0, pow(2, resolution)-1));

    // state += "Channel ";
    // state += String(pin);
    // state += ": ";
    // state += String(output[pin]);
    // state += " / ";
  }
  if (record_flag == true){
    file.println(state);
  }
}

int get_PWM(int PIN_input, int direction){

  int min = minmax_radio_input[2*PIN_input-1];
  int max = minmax_radio_input[2*PIN_input];
  int percentage;
  if (direction == 0){ // set to 50%: stay
    percentage = 50;
  }
  else{
    if (pin_dict[PIN_input+6].percentage_pos + direction >= 0 && pin_dict[PIN_input+6].percentage_pos + direction < action_percentage_length){
      percentage = action_percentage[pin_dict[PIN_input+6].percentage_pos + direction];
    }else{
      percentage = action_percentage[pin_dict[PIN_input+6].percentage_pos];
    }
  }
  return round(float(min) + float(max - min) * percentage / 100);
}

// void print_state(int PIN_intput_array[], int direction[], int size, bool Do){
//   String state = "[Action] ";
//   if (Do){
//     for (int idx = 0; idx < size; idx++){
//       // print state
//       state += pin_dict[PIN_intput_array[idx]].name;
//       state += " from ";
//       state += String(action_percentage[pin_dict[PIN_intput_array[idx]+6].percentage_pos]);
//       state += "% to ";
//       if (pin_dict[PIN_intput_array[idx]+6].percentage_pos + direction[idx] >= 0 && pin_dict[PIN_intput_array[idx]+6].percentage_pos + direction[idx] < action_percentage_length){
//         state += String(action_percentage[pin_dict[PIN_intput_array[idx]+6].percentage_pos + direction[idx]]);
//         // update percentage position
//         pin_dict[PIN_intput_array[idx]+6].percentage_pos += direction[idx];
//       }else{
//         state += String(action_percentage[pin_dict[PIN_intput_array[idx]+6].percentage_pos]);
//       }
//       state += "% / ";
//     }
//   }else{
//     state += "DO nothing";
//   }
//   Serial.println(state);
// }

void record_to_SD_file(){
  // Record to SD card
  if (SD.begin(BUILTIN_SDCARD)) {
    Serial.println(" --- SD initailization done ---");
  } else {
    Serial.println(" --- SD initailization fail ---");
    return;
  }
  SD.remove(data_name);
  file = SD.open(data_name, FILE_WRITE);
}

// ======================================== Actions ========================================
// moving flow -> for holding and waypoint control
void PID_holding(){

}
int inverse_action_idx;
void moving_flow(int action_idx, int inverse_action_idx, String move_type){

  print_pose = false;
  
  // check if drone out of control and need to listen to the radio
  RadioInput();
  output[PIN_yaw_output] = radio_input[PIN_yaw_input];
  output[PIN_throttle_output] = radio_input[PIN_throttle_input];
  if (radio_input[5] <= 1300 || radio_input[5] >= 1700){
    listen_to_radio = true;
    print_pose = true;
  }

  // print state
  Serial.print(" --- [");
  Serial.print(move_type);
  Serial.print("] Action --> ");
  Serial.println(action_dict[action_idx].name);

  // Move!!
  Action(action_idx);
  delay(600);
  Action(inverse_action_idx);
  delay(200);
  Stay();
  delay(50);
  Action(action_idx);
  delay(100);
  Action(inverse_action_idx);
  delay(50);
  Stay();

  node.spinOnce();
}
// Holding
void Holding(){
  move_type = "Holding";
  print_pose = false;
  while(listen_to_radio == false && drift_radius > drift_threshold_low){
    if (abs(drift_theta - 0) < 5){
      moving_flow(action_index.left, action_index.right, move_type);
    }
    else if (drift_theta >= 5 && drift_theta <= 40){
      moving_flow(action_index.llb, action_index.rrf, move_type);
    }
    else if (abs(drift_theta - 45) < 5){
      moving_flow(action_index.left_backward, action_index.right_forward, move_type);
    }
    else if (drift_theta >= 50 && drift_theta <= 85){
      moving_flow(action_index.blb, action_index.frf, move_type);
    }
    else if (abs(drift_theta - 90) < 5){
      moving_flow(action_index.backward, action_index.forward, move_type);
    }
    else if (drift_theta >= 95 && drift_theta <= 130){
      moving_flow(action_index.brb, action_index.flf, move_type);
    }
    else if (abs(drift_theta - 135) < 5){
      moving_flow(action_index.right_backward, action_index.left_forward, move_type);
    }
    else if (drift_theta >= 140 && drift_theta <= 175){
      moving_flow(action_index.rrb, action_index.llf, move_type);
    }
    else if (abs(drift_theta - 180) < 5){
      moving_flow(action_index.right, action_index.left, move_type);
    }
    else if (drift_theta >= -40 && drift_theta <= -5){
      moving_flow(action_index.llf, action_index.rrb, move_type);
    }
    else if (abs(drift_theta - (-45)) < 5){
      moving_flow(action_index.left_forward, action_index.right_backward, move_type);
    }
    else if (drift_theta >= -85 && drift_theta <= -50){
      moving_flow(action_index.flf, action_index.brb, move_type);
    }
    else if (abs(drift_theta - (-90)) < 5){
      moving_flow(action_index.forward, action_index.backward, move_type);
    }
    else if (drift_theta >= -130 && drift_theta <= -95){
      moving_flow(action_index.frf, action_index.blb, move_type);
    }
    else if (abs(drift_theta - (-135)) < 5){
      moving_flow(action_index.right_forward, action_index.left_backward, move_type);
    }
    else if (drift_theta >= -175 && drift_theta <= -140){
      moving_flow(action_index.rrf, action_index.llb, move_type);
    }
    else if (abs(drift_theta - (-180)) < 5){
      moving_flow(action_index.right, action_index.left, move_type);
    }      
  }

  Serial.println(" ===================== Holding finish ===================== ");
  print_pose = true;
  node.spinOnce();
}
// Waypoint control
void waypoint_control(float delta_x, float delta_y){
  move_type = "Waypoint moving";
  print_pose = false;
  float x_target_pos = position_dict.x_current_pos + delta_x;
  float y_target_pos = position_dict.y_current_pos + delta_y;

  // x-axis moving
  if (delta_x > 0){ // go right
    while (listen_to_radio == false && abs(position_dict.x_now_pos - x_target_pos) > path_threshold){
      moving_flow(action_index.right, action_index.left, move_type);
    }
  }else if (delta_x < 0){ // go left
    while (listen_to_radio == false && abs(position_dict.x_now_pos - x_target_pos) > path_threshold){
      moving_flow(action_index.left, action_index.right, move_type);
    }
  }
  // y-axis moving
  if (delta_y > 0){ // go forward
    while (listen_to_radio == false && abs(position_dict.y_now_pos - y_target_pos) > path_threshold){
      moving_flow(action_index.forward, action_index.backward, move_type);
    }
  }else if (delta_y < 0){ // go backward
    while (listen_to_radio == false && abs(position_dict.y_now_pos - y_target_pos) > path_threshold){
      moving_flow(action_index.backward, action_index.forward, move_type);
    }
  }

  Serial.println(" ===================== Waypoint move finish ===================== ");
  print_pose = true;

  // update current position
  position_dict.x_current_pos = x_target_pos;
  position_dict.y_current_pos = y_target_pos;
  node.spinOnce();
}

// Actions
void Action(int index){
  int PIN_intput_array[2];
  int direction[2];
  for(int idx = 0; idx < 2; idx++) {
      PIN_intput_array[idx] = action_dict[index].PIN[idx];
      direction[idx] = action_dict[index].direction[idx];
  }
  float step_rate = 0.25;

  if (PIN_intput_array[1] == 0){ // 1-way move

    int output_PIN = PIN_intput_array[0]+6;
    int start = output[output_PIN];
    int target = get_PWM(PIN_intput_array[0], direction[0]);
    int step = step_rate * (target - start); // + increase / - decrease
    int size = 1;
    bool loop = true;
    bool Do = true;

    // String state = "[Step] ====== ";
    // state += String(step);
    // Serial.println(state);

    while (loop == true){
      if (step > 0){
        if (output[output_PIN] + step > target){
          loop = false;
          break;
        }
      }else if (step < 0){
        if (output[output_PIN] + step < target){
          loop = false;
          break;
        }        
      }else if (step == 0){
        Serial.println("Step = 0");
        loop = false;
        Do = false;
        break;
      }
      output[output_PIN] = output[output_PIN] + step;
      output[PIN_throttle_output] = radio_input[PIN_throttle_input];
      Output(output);
    }
    // print_state(PIN_intput_array, direction, size, Do);
  }
  else if (PIN_intput_array[1] != 0){ // 2-way move

    int output_PIN_1 = PIN_intput_array[0]+6;
    int output_PIN_2 = PIN_intput_array[1]+6;
    int start_1 = output[output_PIN_1];
    int start_2 = output[output_PIN_2];
    int target_1 = get_PWM(PIN_intput_array[0], direction[0]);
    int target_2 = get_PWM(PIN_intput_array[1], direction[1]);
    int step_1 = step_rate * (target_1 - start_1);
    int step_2 = step_rate * (target_2 - start_2);
    int size = 2;
    bool loop = true;
    bool Do = true;

    // String state = "[Step] step 1: ";
    // state += String(step_1);
    // state += " / step 2: ";
    // state += String(step_2);
    // Serial.println(state);

    while (loop == true){

      if (step_1 > 0 && step_2 > 0){ // 往右後
        if (output[output_PIN_1] + step_1 > target_1 || output[output_PIN_2] + step_2 > target_2){
          loop = false;
          break;
        }
      }else if (step_1 > 0 && step_2 < 0){ // 往右前
        if (output[output_PIN_1] + step_1 > target_1 || output[output_PIN_2] + step_2 < target_2){
          loop = false;
          break;
        }
      }else if (step_1 < 0 && step_2 > 0){ // 往左後
        if (output[output_PIN_1] + step_1 < target_1 || output[output_PIN_2] + step_2 > target_2){
          loop = false;
          break;
        }
      }else if (step_1 < 0 && step_2 < 0){ // 往左前
        if (output[output_PIN_1] + step_1 < target_1 || output[output_PIN_2] + step_2 < target_2){
          loop = false;
          break;
        }
      }else if (step_1 == 0 || step_2 == 0) {
        loop = false;
        Do = false;
        break;
      }

      output[output_PIN_1] = output[output_PIN_1] + step_1;
      output[output_PIN_2] = output[output_PIN_2] + step_2;
      output[PIN_throttle_output] = radio_input[PIN_throttle_input];
      Output(output);
    }
    // print_state(PIN_intput_array, direction, size, Do);
  }

  // String state = " --- Action ";
  // state = state += action_dict[index].name;
  // state = state += " finish ---";
  // Serial.println(state);
}

void Stay(){ 

  output[PIN_roll_output] = get_PWM(PIN_roll_input, 0);
  output[PIN_pitch_output] = get_PWM(PIN_pitch_input, 0);
  output[PIN_yaw_output] = get_PWM(PIN_yaw_input, 0);

  pin_dict[PIN_roll_output].percentage_pos = middle_pos;
  pin_dict[PIN_pitch_output].percentage_pos = middle_pos;
  pin_dict[PIN_yaw_output].percentage_pos = middle_pos;

  // Serial.println("roll, pitch, yaw set to 50% !");
  output[PIN_throttle_output] = radio_input[PIN_throttle_input];
  Output(output);
}

// ======================================== Main code ========================================
void setup() {

  // Record to SD
  if (record_flag == true){
    record_to_SD_file();
  }

  // Subscribe to keyboard node in ROS
  node.initNode();
  node.subscribe(kbSub);
  node.subscribe(poseSub);
  
  // Setup Serial
  Serial.begin(115200);

  // Setup I/O pins
  pinMode(1, INPUT); 
  pinMode(2, INPUT); 
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT); 
  pinMode(6, INPUT);

  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  // Calibration
  radio_calibration(); 
}

void loop() {
  node.spinOnce(); 
  RadioInput();
  
  // Listen to "keyboard"
  if (1300 <= radio_input[5] && radio_input[5] <= 1700){
    listen_to_radio = false;
  }else{
    listen_to_radio = true;
    print_pose = true;
  }
  
  if (listen_to_radio == false) {
    // Serial.println("Listen to ROS");

    // if key not in [2, 4, 6, 8, 5], do Holding
    if (key != "2" && key != "4" && key != "6" && key != "8" && key != "5"){
      Holding();
    }
    // MAIN CODE of waypoint control
    else if (key == "6"){ // move +1 m in x-axis
      waypoint_control(move_length, 0);
    }
    else if (key == "4"){ // move -1 m in x-axis
      waypoint_control(-move_length, 0);
    }
    else if (key == "8"){ // move +1 m in y-axis
      waypoint_control(0, move_length);
    }
    else if (key == "2"){ // move -1 m in y-axis
      waypoint_control(0, -move_length);
    }
    else if (key == "5"){ // stay
      Stay();
    }


    // MAIN CODE of velocity control
    // if (key == "I" || key == "O"){
    //   // Start rotating & To Takeoff (I=initate , O=Operate)
    //   Takeoff(key);
    // }
    // else if (key == "L"){
    //   Landing();
    // }
    // if (key == "w"){
    //   Action(action_index.up);
    // }
    // else if (key=="s"){
    //   Action(action_index.down);
    // }
    // else if (key == "a"){
    //   Action(action_index.turnLeft);
    // }
    // else if (key == "d"){
    //   Action(action_index.turnRight);
    // }
    // else if (key == "8"){
    //   Action(action_index.forward);
    // }
    // else if (key == "2"){
    //   Action(action_index.backward);
    // }
    // else if (key == "4"){
    //   Action(action_index.left);
    // }
    // else if (key == "6"){
    //   Action(action_index.right);
    // }
    // else if (key == "9"){
    //   Action(action_index.right_forward);
    // }
    // else if (key == "3"){
    //   Action(action_index.right_backward);
    // }
    // else if (key == "7"){
    //   Action(action_index.left_forward);
    // }
    // else if (key == "1"){
    //   Action(action_index.left_backward);
    // }
    // else if (key == "5"){
    //   Stay();
    // }
    // else if (key == "Q"){
    //   if (record_flag == true){
    //     file.close();
    //     Serial.println(" --- save to SD ---");
    //   }
    // }
    
    // key = "0";
    output[PIN_throttle_output] = radio_input[PIN_throttle_input];
    output[PIN_yaw_output] = radio_input[PIN_yaw_input];

    Output(output);
  }

  // Listen to "Radio"
  else{
    key = "0";
    for (int pin = 1; pin <= 6;  pin++){
      output[pin+6] = radio_input[pin];
    }
    Output(output);
    Serial.println(" ===================== Listen to the radio ===================== ");
  }
  delay(10);
}  