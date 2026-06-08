// switch parts
int switch_pwm_A = 0;
int switch_pwm_B = 0;
bool usePIDCompute = true;
float spd_rate_A = 1.0;
float spd_rate_B = 1.0;
bool heartbeatStopFlag = false;

void movtionPinInit(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  ledcAttach(PWMA, freq, ANALOG_WRITE_BITS);
  ledcAttach(PWMB, freq, ANALOG_WRITE_BITS);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}


void switchEmergencyStop(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}


void switchPortCtrlA(float pwmInputA){
  int pwmIntA = round(pwmInputA * spd_rate_A);
  if(abs(pwmIntA) < 1e-6){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    return;
  }

  if(pwmIntA > 0){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA, pwmIntA);
  }
  else{
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWMA,-pwmIntA);
  }
}


void switchPortCtrlB(float pwmInputB){
  int pwmIntB = round(pwmInputB * spd_rate_B);
  if(abs(pwmIntB) < 1e-6){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    return;
  }

  if(pwmIntB > 0){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(PWMB, pwmIntB);
  }
  else{
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMB,-pwmIntB);
  }
}


void switchCtrl(int pwmIntA, int pwmIntB) {
    switch_pwm_A = pwmIntA;
    switch_pwm_B = pwmIntB;
    switchPortCtrlA(switch_pwm_A);
    switchPortCtrlB(switch_pwm_B);
}


void lightCtrl(int pwmIn) {
  switch_pwm_A = pwmIn;
  switchPortCtrlA(-abs(switch_pwm_A));
}


void setSpdRate(float inputL, float inputR) {
  inputL = abs(inputL);
  if (inputL > 1) {
    inputL = 1;
  }
  inputR = abs(inputR);
  if (inputR > 1) {
    inputR = 1;
  }
  spd_rate_A = inputL;
  spd_rate_B = inputR;
}


void getSpdRate() {
  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = CMD_GET_SPD_RATE;

  jsonInfoHttp["L"] = spd_rate_A;
  jsonInfoHttp["R"] = spd_rate_B;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}



// movtion parts.
// A-left, B-right

ESP32Encoder encoderA;
ESP32Encoder encoderB;

static unsigned long lastTime = 0;
static unsigned long lastLeftSpdTime = 0;
static unsigned long lastRightSpdTime = 0;
int64_t lastEncoderA = 0;
int64_t lastEncoderB = 0;

double speedGetA;
double speedGetB;

// Sample encoder velocity over a fixed interval and hold the last estimate
// between samples. Measuring every cooperative-loop tick makes the denominator
// tiny and pulse quantisation dominates; returning zero until a pulse threshold
// is reached makes the high-I PID over-ramp. A 50 ms sample gives about
// 13 pulses at 0.1 m/s on the UGV Rover, which is enough to damp cogging
// without adding much control latency.
const unsigned long SPEED_SAMPLE_INTERVAL_US = 50000;
const double SPEED_FILTER_ALPHA = 0.3;
double speedFilteredA = 0;
double speedFilteredB = 0;

double plusesRate = 3.14159265359 * WHEEL_D / ONE_CIRCLE_PLUSES;


void initEncoders() {
  encoderA.attachHalfQuad(AENCA, AENCB);
  encoderB.attachHalfQuad(BENCA, BENCB);
  encoderA.setCount(0);
  encoderB.setCount(0);
  unsigned long currentTime = micros();
  lastEncoderA = encoderA.getCount();
  lastEncoderB = encoderB.getCount();
  lastLeftSpdTime = currentTime;
  lastRightSpdTime = currentTime;
  speedGetA = 0;
  speedGetB = 0;
  speedFilteredA = 0;
  speedFilteredB = 0;
}

void resetLeftSpeedEstimate() {
  lastEncoderA = encoderA.getCount();
  lastLeftSpdTime = micros();
  speedGetA = 0;
  speedFilteredA = 0;
}

void resetRightSpeedEstimate() {
  lastEncoderB = encoderB.getCount();
  lastRightSpdTime = micros();
  speedGetB = 0;
  speedFilteredB = 0;
}

void getLeftSpeed() {
  unsigned long currentTime = micros();
  int64_t encoderPulsesA = encoderA.getCount();

  // Odometry from absolute count (unchanged behaviour)
  if (!SET_MOTOR_DIR) {
    en_odom_l = ((float)encoderPulsesA / ONE_CIRCLE_PLUSES) * WHEEL_D * 3.14159265359;
  } else {
    en_odom_l = - ((float)encoderPulsesA / ONE_CIRCLE_PLUSES) * WHEEL_D * 3.14159265359;
  }

  unsigned long elapsed = currentTime - lastLeftSpdTime;
  if (elapsed < SPEED_SAMPLE_INTERVAL_US) {
    return;
  }

  int64_t pulseDelta;
  if (!SET_MOTOR_DIR) {
    pulseDelta = encoderPulsesA - lastEncoderA;
  } else {
    pulseDelta = lastEncoderA - encoderPulsesA;
  }

  const double measuredSpeedA = (plusesRate * pulseDelta) / ((double)elapsed / 1000000);
  lastEncoderA = encoderPulsesA;
  lastLeftSpdTime = currentTime;

  speedFilteredA = SPEED_FILTER_ALPHA * measuredSpeedA + (1.0 - SPEED_FILTER_ALPHA) * speedFilteredA;
  speedGetA = speedFilteredA;  // control + T1001 feedback share the EMA-smoothed velocity (addresses #10)
}

void getRightSpeed() {
  unsigned long currentTime = micros();
  int64_t encoderPulsesB = encoderB.getCount();

  // Odometry from absolute count (unchanged behaviour)
  if (!SET_MOTOR_DIR) {
    en_odom_r = ((float)encoderPulsesB / ONE_CIRCLE_PLUSES) * WHEEL_D * 3.14159265359;
  } else {
    en_odom_r = - ((float)encoderPulsesB / ONE_CIRCLE_PLUSES) * WHEEL_D * 3.14159265359;
  }

  unsigned long elapsed = currentTime - lastRightSpdTime;
  if (elapsed < SPEED_SAMPLE_INTERVAL_US) {
    return;
  }

  int64_t pulseDelta;
  if (!SET_MOTOR_DIR) {
    pulseDelta = encoderPulsesB - lastEncoderB;
  } else {
    pulseDelta = lastEncoderB - encoderPulsesB;
  }

  const double measuredSpeedB = (plusesRate * pulseDelta) / ((double)elapsed / 1000000);
  lastEncoderB = encoderPulsesB;
  lastRightSpdTime = currentTime;

  speedFilteredB = SPEED_FILTER_ALPHA * measuredSpeedB + (1.0 - SPEED_FILTER_ALPHA) * speedFilteredB;
  speedGetB = speedFilteredB;  // control + T1001 feedback share the EMA-smoothed velocity (addresses #10)
}



// --- PID Controller ---

PID_v2 pidA(__kp, __ki, __kd, PID::Direct);
PID_v2 pidB(__kp, __ki, __kd, PID::Direct);

double outputA = 0;
double outputB = 0;
double setpointA = 0;
double setpointB = 0;

int setpoint_interval = 200;
unsigned long setpoint_cmd_recv = millis();
unsigned long setpoint_last_time = millis();
float setpointA_buffer;
float setpointB_buffer;
float setpointA_last;
float setpointB_last;
float change_offset = 0.005;
bool new_setpoint_flag = false;

void pidControllerInit() {
  pidA.Start(speedGetA,
             outputA,
             setpointA);
  pidA.SetOutputLimits(-255, 255);
  pidA.SetSampleTime(50);
  pidA.SetMode(PID::Automatic);

  pidB.Start(speedGetB,
             outputB,
             setpointB);
  pidB.SetOutputLimits(-255, 255);
  pidB.SetSampleTime(50);
  pidB.SetMode(PID::Automatic);
}

void leftCtrl(float pwmInputA){
  int pwmIntA = round(pwmInputA);
  if(SET_MOTOR_DIR){
    if(pwmIntA < 0){
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      ledcWrite(PWMA, abs(pwmIntA));
    }
    else{
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      ledcWrite(PWMA, abs(pwmIntA));
    }
  }else{
    if(pwmIntA < 0){
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      ledcWrite(PWMA, abs(pwmIntA));
    }
    else{
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      ledcWrite(PWMA, abs(pwmIntA));
    }
  }
}

void rightCtrl(float pwmInputB){
  int pwmIntB = round(pwmInputB);
  if(SET_MOTOR_DIR){
    if(pwmIntB < 0){
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      ledcWrite(PWMB, abs(pwmIntB));
    }
    else{
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      ledcWrite(PWMB, abs(pwmIntB));
    }
  }else{
    if(pwmIntB < 0){
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      ledcWrite(PWMB, abs(pwmIntB));
    }
    else{
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      ledcWrite(PWMB, abs(pwmIntB));
    }
  }
}

double motorFeedForward(double setpoint) {
  if (setpoint == 0) {
    return 0;
  }

  double direction = setpoint > 0 ? 1.0 : -1.0;
  double magnitude = MOTOR_MIN_FEEDFORWARD_PWM + (abs(setpoint) * MOTOR_SPEED_FEEDFORWARD_PWM);
  return direction * magnitude;
}

double clampMotorOutput(double output) {
  if (output > 255) {
    return 255;
  }
  if (output < -255) {
    return -255;
  }
  return output;
}

void setGoalSpeed(float inputLeft, float inputRight) {
  usePIDCompute = true;

  if(inputLeft < -2.0 || inputLeft > 2.0){
    return;
  }

  if(inputRight < -2.0 || inputRight > 2.0){
    return;
  }
  
  double nextSetpointA = inputLeft*spd_rate_A;
  double nextSetpointB = inputRight*spd_rate_B;

  if (nextSetpointA == 0 || (setpointA < 0 && nextSetpointA > 0) || (setpointA > 0 && nextSetpointA < 0)) {
    resetLeftSpeedEstimate();
  }
  if (nextSetpointB == 0 || (setpointB < 0 && nextSetpointB > 0) || (setpointB > 0 && nextSetpointB < 0)) {
    resetRightSpeedEstimate();
  }

  setpointA = nextSetpointA;
  setpointB = nextSetpointB;

  if (setpointA != setpointA_buffer) {
    pidA.Setpoint(setpointA);
    setpointA_buffer = setpointA;
  }

  if (setpointB != setpointB_buffer) {
    pidB.Setpoint(setpointB);
    setpointB_buffer = setpointB;
  }
}

void LeftPidControllerCompute() {
  if (!usePIDCompute) {
    return;
  }

  double pidOutput = pidA.Run(speedFilteredA);
  if (setpointA == 0) {
    outputA = 0;
  } else {
    outputA = clampMotorOutput(motorFeedForward(setpointA) + pidOutput);
  }
  leftCtrl(outputA);
}

void RightPidControllerCompute() {
  if (!usePIDCompute) {
    return;
  }

  double pidOutput = pidB.Run(speedFilteredB);
  if (setpointB == 0) {
    outputB = 0;
  } else {
    outputB = clampMotorOutput(motorFeedForward(setpointB) + pidOutput);
  }
  rightCtrl(outputB);
}

void setPID(float inputP, float inputI, float inputD, float inputLimits) {
  __kp = inputP;
  __ki = inputI;
  __kd = inputD;
  windup_limits = inputLimits;
  pidA.SetTunings(__kp, __ki, __kd);
  pidB.SetTunings(__kp, __ki, __kd);
}

void rosCtrl(float rosX, float rosZ) {
  // Pass locals into setGoalSpeed so its stop/reversal reset can compare the new
  // command against the PREVIOUS setpointA/setpointB before they're overwritten.
  double localA = rosX - (rosZ * TRACK_WIDTH / 2.0);
  double localB = rosX + (rosZ * TRACK_WIDTH / 2.0);
  // Snap float residue to exactly zero at the source: a wheel that should cancel
  // to zero must not reach motorFeedForward() as a tiny non-zero (which would
  // kick it to MOTOR_MIN_FEEDFORWARD_PWM and twitch a wheel meant to be still).
  if (fabs(localA) < 1e-4) localA = 0;
  if (fabs(localB) < 1e-4) localB = 0;
  setGoalSpeed(localA, localB);
}

void heartBeatCtrl() {
  if (currentTimeMillis - lastCmdRecvTime > HEART_BEAT_DELAY) {
    if (!heartbeatStopFlag) {
      heartbeatStopFlag = true;
      setGoalSpeed(0, 0);
    }
  }
}

void changeHeartBeatDelay(int inputCmd) {
  HEART_BEAT_DELAY = inputCmd;
}

void mm_settings(byte inputMain, byte inputModule) {
  mainType = inputMain;
  moduleType = inputModule;
  
  // mainType:01 RaspRover
  // #define WHEEL_D 0.0800
  // #define ONE_CIRCLE_PLUSES  2100
  // #define TRACK_WIDTH  0.125
  // #define SET_MOTOR_DIR false

  // mainType:02 UGV Rover
  // #define WHEEL_D 0.0800
  // #define ONE_CIRCLE_PLUSES  1650(v=0.90) -> 660(v>=0.93)
  // #define TRACK_WIDTH  0.172
  // #define SET_MOTOR_DIR false

  // mainType:03 UGV Beast
  // #define WHEEL_D  0.0523
  // #define ONE_CIRCLE_PLUSES  1092
  // #define TRACK_WIDTH  0.141
  // #define SET_MOTOR_DIR true

  if (mainType == 1) {
    WHEEL_D = 0.0800;
    ONE_CIRCLE_PLUSES = 2100;
    TRACK_WIDTH = 0.125;
    SET_MOTOR_DIR = false;
  } else if (mainType == 2) {
    WHEEL_D = 0.0800;
    ONE_CIRCLE_PLUSES = 660;
    TRACK_WIDTH = 0.172;
    SET_MOTOR_DIR = false;
  } else if (mainType == 3) {
    WHEEL_D = 0.0523;
    ONE_CIRCLE_PLUSES = 1092;
    TRACK_WIDTH = 0.141;
    SET_MOTOR_DIR = true;
  }
  plusesRate = 3.14159265359 * WHEEL_D / ONE_CIRCLE_PLUSES;

  if (mainType == 1) {
    screenLine_2 = "RaspRover";
  } else if (mainType == 2) {
    screenLine_2 = "UGV Rover";
  } else if (mainType == 3) {
    screenLine_2 = "UGV Beast";
  } 

  if (moduleType == 0) {
    screenLine_2 += " Null";
  } else if (moduleType == 1) {
    screenLine_2 += " Arm";
  } else if (moduleType == 2) {
    screenLine_2 += " PT";
  } 
}