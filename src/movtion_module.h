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

  ledcSetup(0, freq, ANALOG_WRITE_BITS);  // Channel 0 for motor A
  ledcAttachPin(PWMA, 0);
  ledcSetup(1, freq, ANALOG_WRITE_BITS);  // Channel 1 for motor B
  ledcAttachPin(PWMB, 1);

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
    ledcWrite(0, pwmIntA);
  }
  else{
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(0, -pwmIntA);
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
    ledcWrite(1, pwmIntB);
  }
  else{
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(1, -pwmIntB);
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
int lastEncoderA = 0;
int lastEncoderB = 0;

double speedGetA;
double speedGetB;

double plusesRate = 3.14159265359 * WHEEL_D / ONE_CIRCLE_PLUSES;


void initEncoders() {
  encoderA.attachHalfQuad(AENCA, AENCB);
  encoderB.attachHalfQuad(BENCA, BENCB);
  encoderA.setCount(0);
  encoderB.setCount(0);
}

void setGoalSpeed(float inputLeft, float inputRight);

void resetWheelOdom() {
  wheel_odom_x = 0;
  wheel_odom_y = 0;
  wheel_odom_dist = 0;
  wheel_odom_path = 0;
  wheel_odom_v = 0;
  wheel_odom_last_l = en_odom_l;
  wheel_odom_last_r = en_odom_r;
}

static double wrapAnglePi(double angle) {
  while (angle > PI) {
    angle -= 2.0 * PI;
  }
  while (angle < -PI) {
    angle += 2.0 * PI;
  }
  return angle;
}

void setDriveAnchorToCurrent() {
  drive_anchor_x = wheel_odom_x;
  drive_anchor_y = wheel_odom_y;
  drive_anchor_path = wheel_odom_path;
}

void stopDrivePlan() {
  drive_plan_active = false;
  drive_plan_turn_phase = false;
  drive_plan_leg_count = 0;
  drive_plan_leg_index = 0;
  drive_plan_leg_start_path = wheel_odom_path;
  drive_plan_total_target_dist = 0;
  drive_plan_completed_dist = 0;
  setGoalSpeed(0, 0);
}

bool startDrivePlan(uint8_t legCount, const double* yawDeg, const double* distM, bool resetAnchor) {
  if (legCount == 0 || legCount > DRIVE_PLAN_MAX_LEGS) {
    return false;
  }

  if (resetAnchor) {
    setDriveAnchorToCurrent();
  }

  drive_plan_leg_count = legCount;
  drive_plan_leg_index = 0;
  drive_plan_turn_phase = true;
  drive_plan_active = true;
  drive_plan_leg_start_path = wheel_odom_path;
  drive_plan_total_target_dist = 0;
  drive_plan_completed_dist = 0;

  for (uint8_t i = 0; i < legCount; i++) {
    drive_plan_leg_yaw[i] = yawDeg[i] * DEG_TO_RAD;
    drive_plan_leg_dist[i] = distM[i];
    drive_plan_total_target_dist += fabs(distM[i]);
  }

  heartbeatStopFlag = false;
  lastCmdRecvTime = millis();
  return true;
}

void updateDrivePlanController() {
  if (!drive_plan_active) {
    return;
  }

  if (drive_plan_leg_index >= drive_plan_leg_count) {
    stopDrivePlan();
    return;
  }

  heartbeatStopFlag = false;
  lastCmdRecvTime = millis();

  const uint8_t legIdx = drive_plan_leg_index;
  const double targetYaw = drive_plan_leg_yaw[legIdx];
  const double targetDist = fabs(drive_plan_leg_dist[legIdx]);
  const double currLegDist = fabs(wheel_odom_path - drive_plan_leg_start_path);
  const double yawErr = wrapAnglePi(targetYaw - icm_yaw);

  const double turnTolRad = 4.0 * DEG_TO_RAD;
  const double distTolM = 0.02;
  const double turnKp = 1.1;
  const double holdKp = 0.9;
  const double maxTurnSpeed = 0.38;
  const double cruiseSpeed = 0.30;
  const double minCruiseSpeed = 0.12;
  const double maxHoldCorr = 0.18;

  if (drive_plan_turn_phase) {
    if (fabs(yawErr) <= turnTolRad) {
      drive_plan_turn_phase = false;
      drive_plan_leg_start_path = wheel_odom_path;
      setGoalSpeed(0, 0);
      return;
    }

    double turnSpeed = constrain(turnKp * yawErr, -maxTurnSpeed, maxTurnSpeed);
    setGoalSpeed(-turnSpeed, turnSpeed);
    return;
  }

  if (currLegDist + distTolM >= targetDist) {
    drive_plan_completed_dist += targetDist;
    drive_plan_leg_index++;
    drive_plan_turn_phase = true;
    setGoalSpeed(0, 0);
    if (drive_plan_leg_index >= drive_plan_leg_count) {
      drive_plan_active = false;
      setGoalSpeed(0, 0);
    }
    return;
  }

  double remain = targetDist - currLegDist;
  double linearSpeed = cruiseSpeed;
  if (remain < 0.25) {
    linearSpeed = minCruiseSpeed + ((cruiseSpeed - minCruiseSpeed) * (remain / 0.25));
  }
  linearSpeed = constrain(linearSpeed, minCruiseSpeed, cruiseSpeed);
  if (drive_plan_leg_dist[legIdx] < 0) {
    linearSpeed = -linearSpeed;
  }

  double yawCorr = constrain(holdKp * yawErr, -maxHoldCorr, maxHoldCorr);
  double leftCmd = constrain(linearSpeed - yawCorr, -0.8, 0.8);
  double rightCmd = constrain(linearSpeed + yawCorr, -0.8, 0.8);
  setGoalSpeed(leftCmd, rightCmd);
}

void updateWheelOdom() {
  const double dl = en_odom_l - wheel_odom_last_l;
  const double dr = en_odom_r - wheel_odom_last_r;
  wheel_odom_last_l = en_odom_l;
  wheel_odom_last_r = en_odom_r;

  const double dc = 0.5 * (dl + dr);
  wheel_odom_path += fabs(dc);

  const double heading = icm_yaw;
  wheel_odom_x += dc * cos(heading);
  wheel_odom_y += dc * sin(heading);
  wheel_odom_dist = sqrt((wheel_odom_x * wheel_odom_x) + (wheel_odom_y * wheel_odom_y));

  wheel_odom_v = 0.5 * (speedGetA + speedGetB);
}

void getLeftSpeed() {
  unsigned long currentTime = micros();
  long encoderPulsesA = encoderA.getCount();
  if (!SET_MOTOR_DIR) {
    speedGetA = (plusesRate * (encoderPulsesA - lastEncoderA)) / ((double)(currentTime - lastLeftSpdTime) / 1000000);
    en_odom_l = ((float)encoderPulsesA / ONE_CIRCLE_PLUSES) * WHEEL_D * 3.14159265359;
  } else {
    speedGetA = (plusesRate * (lastEncoderA - encoderPulsesA)) / ((double)(currentTime - lastLeftSpdTime) / 1000000);
    en_odom_l = - ((float)encoderPulsesA / ONE_CIRCLE_PLUSES) * WHEEL_D * 3.14159265359;
  }
  lastEncoderA = encoderPulsesA;
  lastLeftSpdTime = currentTime;
}

void getRightSpeed() {
  unsigned long currentTime = micros();
  long encoderPulsesB = encoderB.getCount();
  if (!SET_MOTOR_DIR) {
    speedGetB = (plusesRate * (encoderPulsesB - lastEncoderB)) / ((double)(currentTime - lastRightSpdTime) / 1000000);
    en_odom_r = ((float)encoderPulsesB / ONE_CIRCLE_PLUSES) * WHEEL_D * 3.14159265359;
  } else {
    speedGetB = (plusesRate * (lastEncoderB - encoderPulsesB)) / ((double)(currentTime - lastRightSpdTime) / 1000000);
    en_odom_r = - ((float)encoderPulsesB / ONE_CIRCLE_PLUSES) * WHEEL_D * 3.14159265359;
  }
  lastEncoderB = encoderPulsesB;
  lastRightSpdTime = currentTime;
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
  pidA.SetMode(PID::Automatic);

  pidB.Start(speedGetB,
             outputB,
             setpointB);
  pidB.SetOutputLimits(-255, 255);
  pidB.SetMode(PID::Automatic);
}

void leftCtrl(float pwmInputA){
  int pwmIntA = round(pwmInputA);
  if(SET_MOTOR_DIR){
    if(pwmIntA < 0){
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      ledcWrite(0, abs(pwmIntA));
    }
    else{
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      ledcWrite(0, abs(pwmIntA));
    }
  }else{
    if(pwmIntA < 0){
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      ledcWrite(0, abs(pwmIntA));
    }
    else{
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      ledcWrite(0, abs(pwmIntA));
    }
  }
}

void rightCtrl(float pwmInputB){
  int pwmIntB = round(pwmInputB);
  if(SET_MOTOR_DIR){
    if(pwmIntB < 0){
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      ledcWrite(1, abs(pwmIntB));
    }
    else{
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      ledcWrite(1, abs(pwmIntB));
    }
  }else{
    if(pwmIntB < 0){
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      ledcWrite(1, abs(pwmIntB));
    }
    else{
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      ledcWrite(1, abs(pwmIntB));
    }
  }
}

void setGoalSpeed(float inputLeft, float inputRight) {
  usePIDCompute = true;

  if(inputLeft < -2.0 || inputLeft > 2.0){
    return;
  }

  if(inputRight < -2.0 || inputRight > 2.0){
    return;
  }
  
  setpointA = inputLeft*spd_rate_A;
  setpointB = inputRight*spd_rate_B;

  if (setpointA != setpointA_buffer) {
    pidA.Setpoint(setpointA);
    setpointA_buffer = inputLeft;
  }
  
  if (setpointB != setpointB_buffer) {
    pidB.Setpoint(setpointB);
    setpointB_buffer = inputRight;
  }
}

void LeftPidControllerCompute() {
  if (!usePIDCompute) {
    return;
  }

  outputA = pidA.Run(speedGetA);
  if (abs(outputA)<THRESHOLD_PWM) {
    outputA = 0;
  }
  if (setpointA == 0 && speedGetA == 0) {
    outputA = 0;
  }
  leftCtrl(outputA);
}

void RightPidControllerCompute() {
  if (!usePIDCompute) {
    return;
  }

  outputB = pidB.Run(speedGetB);
  if (abs(outputB)<THRESHOLD_PWM) {
    outputB = 0;
  }
  if (setpointB == 0 && speedGetB == 0) {
    outputB = 0;
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
  setpointA = rosX - (rosZ * TRACK_WIDTH / 2.0);
  setpointB = rosX + (rosZ * TRACK_WIDTH / 2.0);
  setGoalSpeed(setpointA, setpointB);
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