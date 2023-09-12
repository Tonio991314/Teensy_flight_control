#include <ros.h>
#include <Arduino.h>
#include <std_msgs/String.h>
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
int action_percentage[11] = {35, 37, 39, 42, 46, 50, 54, 58, 61, 63, 65}; // 11 percentage: 34%, 36%, 38%, 41%, 45%, 50%, 55%, 59%, 62%, 64%, 66%
int action_percentage_length = sizeof(action_percentage) / sizeof(action_percentage[0]);
int middle_pos = action_percentage_length / 2;

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
  int up;             // 0 
  int down;           // 1
  int forward;        // 2
  int backward;       // 3
  int left;           // 4  
  int right;          // 5
  int right_forward;  // 6 
  int right_backward; // 7
  int left_forward;   // 8
  int left_backward;  // 9
  int turnLeft;       // 10
  int turnRight;      // 11
} Action_index;

const Action_index action_index[]{
  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
};

// Action dictionary
typedef struct { 
  String name;
  int PIN[2];
  int direction[2];
} Action_dict;

const Action_dict action_dict[]{
  {"up",             {PIN_throttle_input, 0},           {1, 0}}, // 0: up 
  {"down",           {PIN_throttle_input, 0},           {-1, 0}}, // 1: down 
  {"foward",         {PIN_pitch_input, 0},              {-1, 0}}, // 2: foward
  {"backward",       {PIN_pitch_input, 0},              {1, 0}}, // 3: backward
  {"left",           {PIN_roll_input, 0},               {-1, 0}}, // 4: left
  {"right",          {PIN_roll_input, 0},               {1, 0}}, // 5: right
  {"right,forward",  {PIN_roll_input, PIN_pitch_input}, {1, -1}}, // 6: right forward
  {"right,backward", {PIN_roll_input, PIN_pitch_input}, {1, 1}}, // 7: right backward
  {"left,forward",   {PIN_roll_input, PIN_pitch_input}, {-1, -1}}, // 8: left forward
  {"left,backward",  {PIN_roll_input, PIN_pitch_input}, {-1, 1}}, // 9: left backward
  {"turn left",      {PIN_yaw_input, 0},                {-1, 0}}, // 10: turn left
  {"turn right",     {PIN_yaw_input, 0},                {1, 0}}, // 11: turn right
};

// Some variables
int resolution = 12;
int full_pulse = 0;
int radio_input[13]; // idx 1 to 6 mean radio input pin 1~6 (channel 1~6)
int minmax_radio_input[13] = {0, 1500, 0, 1500, 0, 1500, 0, 1500, 0, 1500, 0, 1500, 0};
// int minmax_radio_input[13] = {0, 1100, 1921, 1109, 1930, 1109, 1930, 1110, 1930, 969, 2070, 969, 2070}; 
int radio_channel_range[7]; // idx 1 to 6 mean radio channel full range
int output[13]; // output array to Output() function, idx 7~12 mean pin 7~12 (channel 1~6) 

// ROS node 
String key = "0"; // initial key, set to "0"
ros::NodeHandle node;
std_msgs::String string_msg;

// // Record to SD
File file;
const char* data_name = "230908_1.txt";


// ======================================== Functions ======================================== 
// Subscribe keyboard typing
void kbCallback(const std_msgs::String & msg){
  key = msg.data;

  // print state
  // String state = "key: ";
  // state += key;
  // Serial.println(state);
  
  // record to SD
  // if (key != "0"){
  //   file.println(key);
  // } 
}
ros::Subscriber<std_msgs::String> kbSub("publisher_keyboard", kbCallback);

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
  Serial.print(" --- Radio calibration end ---");
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
    state += "Channel ";
    state += String(pin);
    state += ": ";
    state += String(output[pin]);
    state += " / ";
  }
  // Serial.println(state);
  file.println(state);
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

void print_state(int PIN_intput_array[], int direction[], int size, bool Do){

  String state = "[Action] ";
  if (Do){
    for (int idx = 0; idx < size; idx++){
      // print state
      state += pin_dict[PIN_intput_array[idx]].name;
      state += " from ";
      state += String(action_percentage[pin_dict[PIN_intput_array[idx]+6].percentage_pos]);
      state += "% to ";
      if (pin_dict[PIN_intput_array[idx]+6].percentage_pos + direction[idx] >= 0 && pin_dict[PIN_intput_array[idx]+6].percentage_pos + direction[idx] < action_percentage_length){
        state += String(action_percentage[pin_dict[PIN_intput_array[idx]+6].percentage_pos + direction[idx]]);

        // update percentage position
        pin_dict[PIN_intput_array[idx]+6].percentage_pos += direction[idx];
      }else{
        state += String(action_percentage[pin_dict[PIN_intput_array[idx]+6].percentage_pos]);
      }
      state += "% / ";
    }
  }else{
    state += "DO nothing";
  }
  Serial.println(state);
}

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
    print_state(PIN_intput_array, direction, size, Do);
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
    print_state(PIN_intput_array, direction, size, Do);
  }

  String state = " --- Action ";
  state = state += action_dict[index].name;
  state = state += " finish ---";
  Serial.println(state);
}

/*
void Takeoff(String key){
  int target_percentage = 60;
  float takeoff_rate;
  //Initate target value is 1465
  if (key == "I"){
    target_percentage = 60;
    takeoff_rate = 0.08;
  }
  //Operate target value is 1600
  else if (key == "O"){
    target_percentage = 63;
    takeoff_rate = 0.08;
  }
  int pin_array[] = {PIN_throttle_input};
  int size = sizeof(pin_array) / sizeof(pin_array[0]);
  int start = output[PIN_throttle_output];
  int target = get_PWM(minmax_radio_input[2*PIN_throttle_input-1], minmax_radio_input[2*PIN_throttle_input], target_percentage);
  int step = takeoff_rate * (target - start);
  Serial.print(output[PIN_throttle_output]);
  Serial.print("    /    ");
  Serial.print(target);

  while (output[PIN_throttle_output] + step < target && abs(step) > 0){
    output[PIN_throttle_output] = output[PIN_throttle_output] + step;
    Output(output);
    print_state(pin_array, size);
  }  
  key =="0";
  Serial.println(" --- Takeoff ready  ---");
}

void Landing(){
  float landing_rate = 0.1;
  int pin_array[] = {PIN_throttle_input};
  int size = sizeof(pin_array) / sizeof(pin_array[0]);
  int start = output[PIN_throttle_output];
  int target =  minmax_radio_input[2*PIN_throttle_input-1] + 10;
  int step = landing_rate * (target - start);
  while (output[PIN_throttle_output] + step > target && abs(step) > 0){
    output[PIN_throttle_output] = output[PIN_throttle_output] + step;
    Output(output);
    print_state(pin_array, size);
  }
}
*/
void Stay(){ 

  output[PIN_roll_output] = get_PWM(PIN_roll_input, 0);
  output[PIN_pitch_output] = get_PWM(PIN_pitch_input, 0);
  output[PIN_yaw_output] = get_PWM(PIN_yaw_input, 0);

  pin_dict[PIN_roll_output].percentage_pos = middle_pos;
  pin_dict[PIN_pitch_output].percentage_pos = middle_pos;
  pin_dict[PIN_yaw_output].percentage_pos = middle_pos;

  Serial.println("roll, pitch, yaw set to 50% !");
  output[PIN_throttle_output] = radio_input[PIN_throttle_input];
  Output(output);
}

/*



*/
// ======================================== Main code ========================================
void setup() {

  // Record to SD
  record_to_SD_file();

  // Subscribe to keyboard node in ROS
  node.initNode();
  node.subscribe(kbSub);
  
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
  if (1300 <= radio_input[PIN_mode_input] && radio_input[PIN_mode_input] <= 1700) {
    output[PIN_mode_output] = radio_input[PIN_mode_input];
    // Serial.println("Listen to ROS");
    
    // MAIN CODE
    // if (key == "I" || key == "O"){
    //   // Start rotating & To Takeoff (I=initate , O=Operate)
    //   Takeoff(key);
    // }
    // else if (key == "L"){
    //   Landing();
    // }
    if (key == "w"){
      Action(action_index[0].up);
    }
    else if (key=="s"){
      Action(action_index[0].down);
    }
    else if (key == "a"){
      Action(action_index[0].turnLeft);
    }
    else if (key == "d"){
      Action(action_index[0].turnRight);
    }
    else if (key == "8"){
      Action(action_index[0].forward);
    }
    else if (key == "2"){
      Action(action_index[0].backward);
    }
    else if (key == "4"){
      Action(action_index[0].left);
    }
    else if (key == "6"){
      Action(action_index[0].right);
    }
    else if (key == "9"){
      Action(action_index[0].right_forward);
    }
    else if (key == "3"){
      Action(action_index[0].right_backward);
    }
    else if (key == "7"){
      Action(action_index[0].left_forward);
    }
    else if (key == "1"){
      Action(action_index[0].left_backward);
    }
    else if (key == "5"){
      Stay();
    }
    else if (key == "Q"){
      file.close();
      Serial.println(" --- save to SD ---");
    }
    
    key = "0";
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
  }
  delay(10);
}  