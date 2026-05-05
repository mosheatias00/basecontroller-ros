#define AD0_VAL 0
ICM_20948_I2C myICM;

void imu_init() {
}


void updateIMUData() {
}


// {"T":127}
// Explicit installation-bias update command:
// 1) boot and wait for startup IMU runtime calibration to settle (valid=true)
// 2) point robot to magnetic north
// 3) send T:127 to save installation heading bias to flash
void setIMUInstallationBias() {
  // Refuse bias save until startup runtime calibration converged.
  if (!imu_heading_valid) {
    jsonInfoHttp.clear();
    jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;
    jsonInfoHttp["calibrated"] = false;
    jsonInfoHttp["error"] = "IMU runtime calibration not settled (wait valid=true)";
    jsonInfoHttp["acc"] = imu_dmp_accuracy;
    jsonInfoHttp["valid"] = imu_heading_valid;
    String out; serializeJson(jsonInfoHttp, out); Serial.println(out);
    return;
  }

  imu_north_offset_rad = icm_yaw;
  imu_calibrated = true;

  // Save to flash so it persists across reboots.
  File file = LittleFS.open("/imu_cal.json", "w");
  if (file) {
    JsonDocument doc;
    // Use the new key; keep backward compatibility in loader.
    doc["install_bias"] = imu_north_offset_rad;
    serializeJson(doc, file);
    file.close();
    Serial.printf("[IMU bias] Saved install_bias=%.4f rad (%.1f deg)\n",
                  imu_north_offset_rad, imu_north_offset_rad * RAD_TO_DEG);
  } else {
    Serial.println("[IMU bias] ERROR: could not write /imu_cal.json");
  }

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;
  jsonInfoHttp["calibrated"] = true;
  jsonInfoHttp["install_bias_deg"] = imu_north_offset_rad * RAD_TO_DEG;
  jsonInfoHttp["north_offset_deg"] = imu_north_offset_rad * RAD_TO_DEG; // legacy key
  jsonInfoHttp["heading"] = getAlignedHeadingDeg();
  jsonInfoHttp["acc"] = imu_dmp_accuracy;
  jsonInfoHttp["valid"] = imu_heading_valid;
  String out;
  serializeJson(jsonInfoHttp, out);
  Serial.println(out);
}

// Backward-compatible command entry point (T:127).
void imuCalibration() {
  setIMUInstallationBias();
}

// Startup runtime calibration: reset convergence flags each boot.
// The loop sets valid=true once Quat9 is accurate and stable.
void startIMURuntimeCalibration() {
  imu_dmp_accuracy = 0;
  imu_heading_valid = false;
  imu_stable_count = 0;
  imu_yaw_prev_stable = 0.0;
  Serial.println("[IMU runtime] Startup calibration started; waiting for stable heading...");
}

// Load saved calibration from flash at boot.
// If no file found, defaults to 0 (uncalibrated).
void loadIMUCalibration() {
  File file = LittleFS.open("/imu_cal.json", "r");
  if (!file) {
    Serial.println("[IMU cal] No saved calibration found, defaulting to 0.");
    imu_north_offset_rad = 0.0;
    imu_calibrated = false;
    return;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, file);
  file.close();
  bool hasNew = doc["install_bias"].is<double>();
  bool hasOld = doc["north_offset"].is<double>();
  if (err || (!hasNew && !hasOld)) {
    Serial.println("[IMU cal] Corrupt calibration file, defaulting to 0.");
    imu_north_offset_rad = 0.0;
    imu_calibrated = false;
    return;
  }
  imu_north_offset_rad = hasNew ? doc["install_bias"].as<double>()
                                : doc["north_offset"].as<double>();
  imu_calibrated = true;
  Serial.printf("[IMU bias] Loaded install_bias=%.4f rad (%.1f deg)\n",
                imu_north_offset_rad, imu_north_offset_rad * RAD_TO_DEG);
}

// {"T":127}
void imuCalibration_bk() {
  bool bias_success  = (myICM.getBiasGyroX(&store.biasGyroX) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasGyroY(&store.biasGyroY) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasGyroZ(&store.biasGyroZ) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasAccelX(&store.biasAccelX) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasAccelY(&store.biasAccelY) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasAccelZ(&store.biasAccelZ) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasCPassX(&store.biasCPassX) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasCPassY(&store.biasCPassY) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasCPassZ(&store.biasCPassZ) == ICM_20948_Stat_Ok);

  if (!bias_success) {
    jsonInfoHttp.clear();
    jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;

    jsonInfoHttp["status"] = 0;

    String getInfoJsonString;
    serializeJson(jsonInfoHttp, getInfoJsonString);
    Serial.println(getInfoJsonString);
    return;
  }

  myICM.setBiasGyroX(store.biasGyroX);
  myICM.setBiasGyroY(store.biasGyroY);
  myICM.setBiasGyroZ(store.biasGyroZ);
  myICM.setBiasAccelX(store.biasAccelX);
  myICM.setBiasAccelY(store.biasAccelY);
  myICM.setBiasAccelZ(store.biasAccelZ);
  myICM.setBiasCPassX(store.biasCPassX);
  myICM.setBiasCPassY(store.biasCPassY);
  myICM.setBiasCPassZ(store.biasCPassZ);

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;

  jsonInfoHttp["status"] = 1;

  jsonInfoHttp["gx"] = store.biasGyroX;
  jsonInfoHttp["gy"] = store.biasGyroY;
  jsonInfoHttp["gz"] = store.biasGyroZ;

  jsonInfoHttp["ax"] = store.biasAccelX;
  jsonInfoHttp["ay"] = store.biasAccelY;
  jsonInfoHttp["az"] = store.biasAccelZ;

  jsonInfoHttp["cx"] = store.biasCPassX;
  jsonInfoHttp["cy"] = store.biasCPassY;
  jsonInfoHttp["cz"] = store.biasCPassZ;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}

// {"T":126}
void getIMUData() {
	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = FEEDBACK_IMU_DATA;

  const double rad_to_deg = 57.29577951308232;
  jsonInfoHttp["r"] = icm_roll * rad_to_deg;
  jsonInfoHttp["p"] = icm_pitch * rad_to_deg;
  jsonInfoHttp["y"] = getAlignedHeadingDeg();
  jsonInfoHttp["heading"] = getAlignedHeadingDeg();
  jsonInfoHttp["calibrated"] = imu_calibrated;
  jsonInfoHttp["acc"] = imu_dmp_accuracy;
  jsonInfoHttp["valid"] = imu_heading_valid;
  jsonInfoHttp["install_bias_deg"] = imu_north_offset_rad * RAD_TO_DEG;
  jsonInfoHttp["north_offset_deg"] = imu_north_offset_rad * RAD_TO_DEG;

  jsonInfoHttp["q0"] = q0;
  jsonInfoHttp["q1"] = q1;
  jsonInfoHttp["q2"] = q2;
  jsonInfoHttp["q3"] = q3;

  // Keep legacy keys but source them from wheel odometry.
  jsonInfoHttp["ix"] = wheel_odom_x;
  jsonInfoHttp["iy"] = wheel_odom_y;
  jsonInfoHttp["id"] = wheel_odom_dist;
  jsonInfoHttp["ivx"] = wheel_odom_v;
  jsonInfoHttp["ivy"] = 0;

	String getInfoJsonString;
	serializeJson(jsonInfoHttp, getInfoJsonString);
	Serial.println(getInfoJsonString);
}

// {"T":128}
// get and set qc0 ~ qc3 compensation quaternion
void getIMUOffset() {
    double halfRoll = -icm_roll / 2.0;
    double qr0 = cos(halfRoll);
    double qr1 = sin(halfRoll);
    double qr2 = 0.0;
    double qr3 = 0.0;

    double halfPitch = -icm_pitch / 2.0;
    double qp0 = cos(halfPitch);
    double qp1 = 0.0;
    double qp2 = sin(halfPitch);
    double qp3 = 0.0;

    qc0 = qr0 * qp0 - qr1 * qp1 - qr2 * qp2 - qr3 * qp3;
    qc1 = qr0 * qp1 + qr1 * qp0 + qr2 * qp3 - qr3 * qp2;
    qc2 = qr0 * qp2 - qr1 * qp3 + qr2 * qp0 + qr3 * qp1;
    qc3 = qr0 * qp3 + qr1 * qp2 - qr2 * qp1 + qr3 * qp0;

    jsonInfoHttp.clear();
    jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;
    jsonInfoHttp["qc0"] = qc0;
    jsonInfoHttp["qc1"] = qc1;
    jsonInfoHttp["qc2"] = qc2;
    jsonInfoHttp["qc3"] = qc3;
    String getInfoJsonString;
    serializeJson(jsonInfoHttp, getInfoJsonString);
    Serial.println(getInfoJsonString);
}

// {"T":128}
void getIMUOffset_bk() {
  bool bias_success = (myICM.getBiasGyroX(&store.biasGyroX) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasGyroY(&store.biasGyroY) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasGyroZ(&store.biasGyroZ) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasAccelX(&store.biasAccelX) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasAccelY(&store.biasAccelY) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasAccelZ(&store.biasAccelZ) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasCPassX(&store.biasCPassX) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasCPassY(&store.biasCPassY) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasCPassZ(&store.biasCPassZ) == ICM_20948_Stat_Ok);

  if (!bias_success) {
    return;
  }

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;

  jsonInfoHttp["gx"] = store.biasGyroX;
  jsonInfoHttp["gy"] = store.biasGyroY;
  jsonInfoHttp["gz"] = store.biasGyroZ;

  jsonInfoHttp["ax"] = store.biasAccelX;
  jsonInfoHttp["ay"] = store.biasAccelY;
  jsonInfoHttp["az"] = store.biasAccelZ;

  jsonInfoHttp["cx"] = store.biasCPassX;
  jsonInfoHttp["cy"] = store.biasCPassY;
  jsonInfoHttp["cz"] = store.biasCPassZ;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}

// {"T":129,"gx":0,"gy":0,"gz":0,"ax":0,"ay":0,"az":0,"cx":0,"cy":0,"cz":0}
void setIMUOffset(int32_t inGX, int32_t inGY, int32_t inGZ, int32_t inAX, int32_t inAY, int32_t inAZ, int32_t inCX, int32_t inCY, int32_t inCZ) {
  store.biasGyroX = inGX;
  store.biasGyroY = inGY;
  store.biasGyroZ = inGZ;

  store.biasAccelX = inAX;
  store.biasAccelY = inAY;
  store.biasAccelZ = inAZ;

  store.biasCPassX = inCX;
  store.biasCPassY = inCY;
  store.biasCPassZ = inCZ;

  myICM.setBiasGyroX(store.biasGyroX);
  myICM.setBiasGyroY(store.biasGyroY);
  myICM.setBiasGyroZ(store.biasGyroZ);
  myICM.setBiasAccelX(store.biasAccelX);
  myICM.setBiasAccelY(store.biasAccelY);
  myICM.setBiasAccelZ(store.biasAccelZ);
  myICM.setBiasCPassX(store.biasCPassX);
  myICM.setBiasCPassY(store.biasCPassY);
  myICM.setBiasCPassZ(store.biasCPassZ);

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;

  jsonInfoHttp["status"] = 1;

  jsonInfoHttp["gx"] = store.biasGyroX;
  jsonInfoHttp["gy"] = store.biasGyroY;
  jsonInfoHttp["gz"] = store.biasGyroZ;

  jsonInfoHttp["ax"] = store.biasAccelX;
  jsonInfoHttp["ay"] = store.biasAccelY;
  jsonInfoHttp["az"] = store.biasAccelZ;

  jsonInfoHttp["cx"] = store.biasCPassX;
  jsonInfoHttp["cy"] = store.biasCPassY;
  jsonInfoHttp["cz"] = store.biasCPassZ;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}