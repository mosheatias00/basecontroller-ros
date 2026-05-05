// --- --- --- Distance Measurement Handlers --- --- ---

bool distanceRefreshArmPosition() {
	if (moduleType != 1) {
		return false;
	}

	bool ok_base = false;
	bool ok_shoulder_drive = false;
	bool ok_shoulder_driven = false;
	bool ok_elbow = false;
	bool ok_hand = false;
	for (int retry = 0; retry < 5; retry++) {
		ok_base = ok_base || getFeedback(BASE_SERVO_ID, true);
		ok_shoulder_drive = ok_shoulder_drive || getFeedback(SHOULDER_DRIVING_SERVO_ID, true);
		ok_shoulder_driven = ok_shoulder_driven || getFeedback(SHOULDER_DRIVEN_SERVO_ID, true);
		ok_elbow = ok_elbow || getFeedback(ELBOW_SERVO_ID, true);
		ok_hand = ok_hand || getFeedback(GRIPPER_SERVO_ID, true);
		bool ok_shoulder = ok_shoulder_drive || ok_shoulder_driven;

		// XYZ only depends on base/shoulder/elbow, so do not fail capture on gripper read.
		bool ok_xyz = (ok_base && ok_shoulder && ok_elbow);
		if (ok_xyz) {
			break;
		}
		delay(12);
	}

	bool ok_shoulder = ok_shoulder_drive || ok_shoulder_driven;
	bool ok_xyz = (ok_base && ok_shoulder && ok_elbow);
	if (!ok_xyz) {
		return false;
	}

	radB = calculateRadByFeedback(servoFeedback[BASE_SERVO_ID - 11].pos, BASE_JOINT);
	if (ok_shoulder_drive) {
		radS = calculateRadByFeedback(servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].pos, SHOULDER_JOINT);
	} else {
		// Shoulder driven servo rotates in the opposite direction, so invert the sign.
		radS = -calculateRadByFeedback(servoFeedback[SHOULDER_DRIVEN_SERVO_ID - 11].pos, SHOULDER_JOINT);
	}
	radE = calculateRadByFeedback(servoFeedback[ELBOW_SERVO_ID - 11].pos, ELBOW_JOINT);
	radG = calculateRadByFeedback(servoFeedback[GRIPPER_SERVO_ID - 11].pos, EOAT_JOINT);

	RoArmM2_computePosbyJointRad(radB, radS, radE, radG);
	if (EEMode == 0 && ok_hand) {
		lastT = radG;
	}

	return true;
}

void distanceSetPointA() {
	const char* capture_source = "input";
	if (moduleType != 1 && !(jsonCmdReceive["x"].is<double>() && jsonCmdReceive["y"].is<double>() && jsonCmdReceive["z"].is<double>())) {
		jsonInfoHttp.clear();
		jsonInfoHttp["T"] = CMD_DISTANCE_SET_A;
		jsonInfoHttp["ok"] = 0;
		jsonInfoHttp["err"] = "No arm module. For base use T:149 then T:130";
		return;
	}

	// Check if explicit x, y, z values are provided
	if (jsonCmdReceive["x"].is<double>() && jsonCmdReceive["y"].is<double>() && jsonCmdReceive["z"].is<double>()) {
		dist_point_a_x = jsonCmdReceive["x"].as<double>();
		dist_point_a_y = jsonCmdReceive["y"].as<double>();
		dist_point_a_z = jsonCmdReceive["z"].as<double>();
	} else {
		// Use fresh arm feedback position (avoid stale cached lastX/lastY/lastZ)
		bool live_ok = distanceRefreshArmPosition();
		dist_point_a_x = lastX;
		dist_point_a_y = lastY;
		dist_point_a_z = lastZ;
		capture_source = live_ok ? "live" : "cached";
		if (!live_ok && InfoPrint == 1) {
			Serial.println("Distance A warning: servo feedback failed, using cached position.");
		}
	}
	dist_point_a_set = true;

	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = CMD_DISTANCE_SET_A;
	jsonInfoHttp["ok"] = 1;
	jsonInfoHttp["msg"] = "Point A set";
	jsonInfoHttp["source"] = capture_source;
	jsonInfoHttp["ax"] = dist_point_a_x;
	jsonInfoHttp["ay"] = dist_point_a_y;
	jsonInfoHttp["az"] = dist_point_a_z;
	
	if (InfoPrint == 1) {
		Serial.printf("Point A set: x=%.2f, y=%.2f, z=%.2f\n", dist_point_a_x, dist_point_a_y, dist_point_a_z);
	}
}

void distanceSetPointB() {
	const char* capture_source = "input";
	if (moduleType != 1 && !(jsonCmdReceive["x"].is<double>() && jsonCmdReceive["y"].is<double>() && jsonCmdReceive["z"].is<double>())) {
		jsonInfoHttp.clear();
		jsonInfoHttp["T"] = CMD_DISTANCE_SET_B;
		jsonInfoHttp["ok"] = 0;
		jsonInfoHttp["err"] = "No arm module. For base use T:149 then T:130";
		return;
	}

	if (!dist_point_a_set) {
		jsonInfoHttp.clear();
		jsonInfoHttp["T"] = CMD_DISTANCE_SET_B;
		jsonInfoHttp["ok"] = 0;
		jsonInfoHttp["err"] = "Point A not set";
		if (InfoPrint == 1) {
			Serial.println("Error: Point A not set. Use CMD_DISTANCE_SET_A first.");
		}
		return;
	}
	
	// Check if explicit x, y, z values are provided
	if (jsonCmdReceive["x"].is<double>() && jsonCmdReceive["y"].is<double>() && jsonCmdReceive["z"].is<double>()) {
		dist_point_b_x = jsonCmdReceive["x"].as<double>();
		dist_point_b_y = jsonCmdReceive["y"].as<double>();
		dist_point_b_z = jsonCmdReceive["z"].as<double>();
	} else {
		// Use fresh arm feedback position (avoid stale cached lastX/lastY/lastZ)
		bool live_ok = distanceRefreshArmPosition();
		dist_point_b_x = lastX;
		dist_point_b_y = lastY;
		dist_point_b_z = lastZ;
		capture_source = live_ok ? "live" : "cached";
		if (!live_ok && InfoPrint == 1) {
			Serial.println("Distance B warning: servo feedback failed, using cached position.");
		}
	}
	
	// Calculate distance: sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)
	double dx = dist_point_b_x - dist_point_a_x;
	double dy = dist_point_b_y - dist_point_a_y;
	double dz = dist_point_b_z - dist_point_a_z;
	dist_last_measurement = sqrt(dx*dx + dy*dy + dz*dz);
	
	// Update statistics
	dist_count++;
	dist_sum += dist_last_measurement;
	dist_sum_squares += dist_last_measurement * dist_last_measurement;
	
	if (dist_last_measurement < dist_min) {
		dist_min = dist_last_measurement;
	}
	if (dist_last_measurement > dist_max) {
		dist_max = dist_last_measurement;
	}

	double mean = dist_sum / dist_count;
	double variance = (dist_sum_squares / dist_count) - (mean * mean);
	double stddev = sqrt(variance > 0 ? variance : 0);

	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = CMD_DISTANCE_SET_B;
	jsonInfoHttp["ok"] = 1;
	jsonInfoHttp["msg"] = "Distance computed";
	jsonInfoHttp["source"] = capture_source;
	jsonInfoHttp["ax"] = dist_point_a_x;
	jsonInfoHttp["ay"] = dist_point_a_y;
	jsonInfoHttp["az"] = dist_point_a_z;
	jsonInfoHttp["bx"] = dist_point_b_x;
	jsonInfoHttp["by"] = dist_point_b_y;
	jsonInfoHttp["bz"] = dist_point_b_z;
	jsonInfoHttp["d"] = dist_last_measurement;
	jsonInfoHttp["count"] = dist_count;
	jsonInfoHttp["min"] = dist_min;
	jsonInfoHttp["max"] = dist_max;
	jsonInfoHttp["mean"] = mean;
	jsonInfoHttp["stddev"] = stddev;
	
	if (InfoPrint == 1) {
		Serial.printf("Point B set: x=%.2f, y=%.2f, z=%.2f\n", dist_point_b_x, dist_point_b_y, dist_point_b_z);
		Serial.printf("Distance A→B: %.2f mm\n", dist_last_measurement);
	}
}

void distanceReset() {
	// Reset all measurement data
	dist_point_a_set = false;
	dist_last_measurement = 0;
	dist_min = 9999999.0;
	dist_max = 0;
	dist_sum = 0;
	dist_sum_squares = 0;
	dist_count = 0;

	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = CMD_DISTANCE_RESET;
	jsonInfoHttp["ok"] = 1;
	jsonInfoHttp["msg"] = "Distance measurement reset";
	
	if (InfoPrint == 1) {
		Serial.println("Distance measurement reset.");
	}
}

void distanceReport() {
	if (moduleType != 1) {
		jsonInfoHttp.clear();
		jsonInfoHttp["T"] = CMD_DISTANCE_REPORT;
		jsonInfoHttp["ok"] = 0;
		jsonInfoHttp["err"] = "No arm module. For base use T:149 then T:130";
		jsonInfoHttp["hint"] = "Read wheel odom fields: imux, imuy, imud";
		return;
	}

	if (!dist_point_a_set) {
		jsonInfoHttp.clear();
		jsonInfoHttp["T"] = CMD_DISTANCE_REPORT;
		jsonInfoHttp["ok"] = 0;
		jsonInfoHttp["err"] = "Point A not set";
		if (InfoPrint == 1) {
			Serial.println("Error: Point A not set. Use CMD_DISTANCE_SET_A first.");
		}
		return;
	}

	bool live_ok = distanceRefreshArmPosition();
	if (!live_ok) {
		jsonInfoHttp.clear();
		jsonInfoHttp["T"] = CMD_DISTANCE_REPORT;
		jsonInfoHttp["ok"] = 0;
		jsonInfoHttp["err"] = "Live servo feedback failed";
		jsonInfoHttp["source"] = "no-live";
		jsonInfoHttp["ts"] = millis();
		jsonInfoHttp["base_ok"] = servoFeedback[BASE_SERVO_ID - 11].status ? 1 : 0;
		jsonInfoHttp["shoulder_ok"] = servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].status ? 1 : 0;
		jsonInfoHttp["elbow_ok"] = servoFeedback[ELBOW_SERVO_ID - 11].status ? 1 : 0;
		jsonInfoHttp["hand_ok"] = servoFeedback[GRIPPER_SERVO_ID - 11].status ? 1 : 0;
		if (InfoPrint == 1) {
			Serial.println("Distance report error: live servo feedback failed.");
		}
		return;
	}

	dist_point_b_x = lastX;
	dist_point_b_y = lastY;
	dist_point_b_z = lastZ;

	// Compute latest A->current distance on each report request.
	double dx = dist_point_b_x - dist_point_a_x;
	double dy = dist_point_b_y - dist_point_a_y;
	double dz = dist_point_b_z - dist_point_a_z;
	dist_last_measurement = sqrt(dx*dx + dy*dy + dz*dz);

	// Update rolling statistics from report-triggered measurements.
	dist_count++;
	dist_sum += dist_last_measurement;
	dist_sum_squares += dist_last_measurement * dist_last_measurement;
	if (dist_last_measurement < dist_min) {
		dist_min = dist_last_measurement;
	}
	if (dist_last_measurement > dist_max) {
		dist_max = dist_last_measurement;
	}
	
	double mean = dist_sum / dist_count;
	double variance = (dist_sum_squares / dist_count) - (mean * mean);
	double stddev = sqrt(variance > 0 ? variance : 0);

	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = CMD_DISTANCE_REPORT;
	jsonInfoHttp["ok"] = 1;
	jsonInfoHttp["msg"] = "Distance from A to current";
	jsonInfoHttp["source"] = live_ok ? "live" : "cached";
	jsonInfoHttp["ts"] = millis();
	jsonInfoHttp["pos11"] = servoFeedback[BASE_SERVO_ID - 11].pos;
	jsonInfoHttp["pos12"] = servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].pos;
	jsonInfoHttp["pos14"] = servoFeedback[ELBOW_SERVO_ID - 11].pos;
	jsonInfoHttp["pos15"] = servoFeedback[GRIPPER_SERVO_ID - 11].pos;
	jsonInfoHttp["ax"] = dist_point_a_x;
	jsonInfoHttp["ay"] = dist_point_a_y;
	jsonInfoHttp["az"] = dist_point_a_z;
	jsonInfoHttp["bx"] = dist_point_b_x;
	jsonInfoHttp["by"] = dist_point_b_y;
	jsonInfoHttp["bz"] = dist_point_b_z;
	jsonInfoHttp["d"] = dist_last_measurement;
	jsonInfoHttp["count"] = dist_count;
	jsonInfoHttp["min"] = dist_min;
	jsonInfoHttp["max"] = dist_max;
	jsonInfoHttp["mean"] = mean;
	jsonInfoHttp["stddev"] = stddev;
	
	Serial.println("\n=== Distance Measurement (A -> Current) ===");
	Serial.printf("Total Measurements: %d\n", dist_count);
	Serial.printf("Latest Distance:   %.2f mm\n", dist_last_measurement);
	Serial.printf("Minimum Distance:  %.2f mm\n", dist_min);
	Serial.printf("Maximum Distance:  %.2f mm\n", dist_max);
	Serial.printf("Mean Distance:     %.2f mm\n", mean);
	Serial.printf("Std Deviation:     %.2f mm\n", stddev);
	Serial.printf("Point A: (%.2f, %.2f, %.2f)\n", dist_point_a_x, dist_point_a_y, dist_point_a_z);
	Serial.printf("Current: (%.2f, %.2f, %.2f)\n", dist_point_b_x, dist_point_b_y, dist_point_b_z);
	Serial.println("====================================\n");
}

void baseDriveSetAnchor() {
	setDriveAnchorToCurrent();

	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = CMD_BASE_SET_ANCHOR;
	jsonInfoHttp["ok"] = 1;
	jsonInfoHttp["msg"] = "Anchor set";
	jsonInfoHttp["anchor_x"] = drive_anchor_x;
	jsonInfoHttp["anchor_y"] = drive_anchor_y;
}

void baseDriveAbort() {
	stopDrivePlan();

	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = CMD_BASE_DRIVE_ABORT;
	jsonInfoHttp["ok"] = 1;
	jsonInfoHttp["msg"] = "Drive plan aborted";
}

void baseDrivePlanStart() {
	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = CMD_BASE_DRIVE_PLAN;

	if (!jsonCmdReceive["legs"].is<JsonArrayConst>()) {
		jsonInfoHttp["ok"] = 0;
		jsonInfoHttp["err"] = "legs must be an array";
		return;
	}

	JsonArrayConst legs = jsonCmdReceive["legs"].as<JsonArrayConst>();
	if (legs.size() == 0 || legs.size() > DRIVE_PLAN_MAX_LEGS) {
		jsonInfoHttp["ok"] = 0;
		jsonInfoHttp["err"] = "legs count out of range";
		jsonInfoHttp["max"] = DRIVE_PLAN_MAX_LEGS;
		return;
	}

	double yawDeg[DRIVE_PLAN_MAX_LEGS];
	double distM[DRIVE_PLAN_MAX_LEGS];
	uint8_t legCount = 0;
	double carryYawDeg = icm_yaw * 57.29577951308232;

	for (JsonVariantConst legVar : legs) {
		if (!legVar.is<JsonObjectConst>()) {
			jsonInfoHttp["ok"] = 0;
			jsonInfoHttp["err"] = "each leg must be object";
			return;
		}

		JsonObjectConst legObj = legVar.as<JsonObjectConst>();
		bool has_yaw = legObj["yaw"].is<float>() || legObj["yaw"].is<int>();
		if (has_yaw) {
			carryYawDeg = legObj["yaw"].as<double>();
		}

		bool has_dist = legObj["dist"].is<float>() || legObj["dist"].is<int>();
		bool has_d = legObj["d"].is<float>() || legObj["d"].is<int>();
		if (!has_dist && !has_d) {
			jsonInfoHttp["ok"] = 0;
			jsonInfoHttp["err"] = "each leg needs dist or d";
			return;
		}

		yawDeg[legCount] = carryYawDeg;
		distM[legCount] = has_dist ? legObj["dist"].as<double>() : legObj["d"].as<double>();
		legCount++;
	}

	bool resetAnchor = true;
	if (jsonCmdReceive["anchor"].is<bool>()) {
		resetAnchor = jsonCmdReceive["anchor"].as<bool>();
	} else if (jsonCmdReceive["anchor"].is<int>()) {
		resetAnchor = (jsonCmdReceive["anchor"].as<int>() != 0);
	}

	if (!startDrivePlan(legCount, yawDeg, distM, resetAnchor)) {
		jsonInfoHttp["ok"] = 0;
		jsonInfoHttp["err"] = "failed to start plan";
		return;
	}

	jsonInfoHttp["ok"] = 1;
	jsonInfoHttp["msg"] = "Drive plan started";
	jsonInfoHttp["legs"] = legCount;
	jsonInfoHttp["anchor_reset"] = resetAnchor ? 1 : 0;
	jsonInfoHttp["target_m"] = drive_plan_total_target_dist;
}

void jsonCmdReceiveHandler(){
	int cmdType = jsonCmdReceive["T"].as<int>();
	switch(cmdType){
	// emergency stop.
	case CMD_EMERGENCY_STOP:
														stopDrivePlan();
												emergencyStopProcessing();
  											setGoalSpeed(0, 0);
												break;
	case CMD_SPEED_CTRL:	if (jsonCmdReceive["T"].is<int>() &&
														(jsonCmdReceive["L"].is<float>() || jsonCmdReceive["L"].is<int>()) &&
														(jsonCmdReceive["R"].is<float>() || jsonCmdReceive["R"].is<int>())){
														stopDrivePlan();
														heartbeatStopFlag = false;
														lastCmdRecvTime = millis();
														setGoalSpeed(
														jsonCmdReceive["L"],
														jsonCmdReceive["R"]);
												} break;
	case CMD_PWM_INPUT:		stopDrivePlan();
														usePIDCompute = false;
												heartbeatStopFlag = false;
												lastCmdRecvTime = millis();
												leftCtrl(jsonCmdReceive["L"]);
												rightCtrl(jsonCmdReceive["R"]);
												break;
	case CMD_ROS_CTRL:		stopDrivePlan();
														rosCtrl(
												jsonCmdReceive["X"],
												jsonCmdReceive["Z"]);break;
	case CMD_SET_MOTOR_PID:
												setPID(
												jsonCmdReceive["P"],
												jsonCmdReceive["I"],
												jsonCmdReceive["D"],
												jsonCmdReceive["L"]);break;
	case CMD_OLED_CTRL:		oledCtrl(
												jsonCmdReceive["lineNum"],
												jsonCmdReceive["Text"]);break;
	case CMD_OLED_DEFAULT:setOledDefault();break;
	case CMD_MODULE_TYPE:	changeModuleType(
												jsonCmdReceive["cmd"]);break;



	case CMD_GET_IMU_DATA:
												getIMUData();break;
	case CMD_CALI_IMU_STEP:
												imuCalibration();break;
	case CMD_SET_IMU_INSTALL_BIAS:
										setIMUInstallationBias();break;
	case CMD_GET_IMU_OFFSET:
												getIMUOffset();
												break;
	case CMD_SET_IMU_OFFSET:
												setIMUOffset(
												jsonCmdReceive["gx"],
												jsonCmdReceive["gy"],
												jsonCmdReceive["gz"],
                        jsonCmdReceive["ax"],
												jsonCmdReceive["ay"],
												jsonCmdReceive["az"],
                        jsonCmdReceive["cx"],
												jsonCmdReceive["cy"],
												jsonCmdReceive["cz"]);break;
	case CMD_BASE_FEEDBACK:
												baseInfoFeedback();break;
	case CMD_BASE_FEEDBACK_FLOW:
												setBaseInfoFeedbackMode(
												jsonCmdReceive["cmd"]);break;
	case CMD_FEEDBACK_FLOW_INTERVAL:
												setFeedbackFlowInterval(
												jsonCmdReceive["cmd"]);break;
	case CMD_UART_ECHO_MODE:
												setCmdEcho(
												jsonCmdReceive["cmd"]);break;
	case CMD_ODOM_RESET:
														resetWheelOdom();break;
	case CMD_BASE_DRIVE_PLAN:
														baseDrivePlanStart();break;
	case CMD_BASE_SET_ANCHOR:
														baseDriveSetAnchor();break;
	case CMD_BASE_DRIVE_ABORT:
														baseDriveAbort();break;
	case CMD_ARM_CTRL_UI: RoArmM2_uiCtrl(
												jsonCmdReceive["E"],
												jsonCmdReceive["Z"],
												jsonCmdReceive["R"]
												);break;
                        


	case CMD_LED_CTRL:		led_pwm_ctrl(
												jsonCmdReceive["IO4"],
												jsonCmdReceive["IO5"]);break;
	case CMD_GIMBAL_CTRL_SIMPLE:
												gimbalCtrlSimple(
												jsonCmdReceive["X"],
												jsonCmdReceive["Y"],
												jsonCmdReceive["SPD"],
												jsonCmdReceive["ACC"]);break;
	case CMD_GIMBAL_CTRL_MOVE:
												gimbalCtrlMove(
												jsonCmdReceive["X"],
												jsonCmdReceive["Y"],
												jsonCmdReceive["SX"],
												jsonCmdReceive["SY"]);break;
	case CMD_GIMBAL_CTRL_STOP:
												gimbalCtrlStop();break;
	case CMD_HEART_BEAT_SET:
												changeHeartBeatDelay(
												jsonCmdReceive["cmd"]);break;
	case CMD_GIMBAL_STEADY:
												gimbalSteadySet(
												jsonCmdReceive["s"],
												jsonCmdReceive["y"]);break;
	case CMD_SET_SPD_RATE:
												setSpdRate(
												jsonCmdReceive["L"],
												jsonCmdReceive["R"]);break;
	case CMD_GET_SPD_RATE:
												getSpdRate();break;
	case CMD_SAVE_SPD_RATE:
												saveSpdRate();break;
	case CMD_GIMBAL_USER_CTRL:
												gimbalUserCtrl(
												jsonCmdReceive["X"],
												jsonCmdReceive["Y"],
												jsonCmdReceive["SPD"]);break;




	// EoAT type settings.
	case CMD_EOAT_TYPE:		configEEmodeType(
												jsonCmdReceive["mode"]);break;
	case CMD_CONFIG_EOAT: configEoAT(
												jsonCmdReceive["pos"],
												jsonCmdReceive["ea"],
												jsonCmdReceive["eb"]
												);break;



	// it moves to goal position directly
	// with interpolation.
	case CMD_MOVE_INIT:		RoArmM2_moveInit();break;
	case CMD_SINGLE_JOINT_CTRL: 
												RoArmM2_singleJointAbsCtrl(
												jsonCmdReceive["joint"],
												jsonCmdReceive["rad"],
												jsonCmdReceive["spd"],
												jsonCmdReceive["acc"]
												);break;
	case CMD_JOINTS_RAD_CTRL: 
												RoArmM2_allJointAbsCtrl(
												jsonCmdReceive["base"],
												jsonCmdReceive["shoulder"],
												jsonCmdReceive["elbow"],
												jsonCmdReceive["hand"],
												jsonCmdReceive["spd"],
												jsonCmdReceive["acc"]
												);break;
	case CMD_SINGLE_AXIS_CTRL: 
												RoArmM2_singlePosAbsBesselCtrl(
												jsonCmdReceive["axis"],
												jsonCmdReceive["pos"],
												jsonCmdReceive["spd"]
												);break;
	case CMD_XYZT_GOAL_CTRL: 
												RoArmM2_allPosAbsBesselCtrl(
												jsonCmdReceive["x"],
											  jsonCmdReceive["y"],
											  jsonCmdReceive["z"],
											  jsonCmdReceive["t"],
											  jsonCmdReceive["spd"]
											  );break;
	case CMD_XYZT_DIRECT_CTRL:
												RoArmM2_baseCoordinateCtrl(
												jsonCmdReceive["x"],
												jsonCmdReceive["y"],
												jsonCmdReceive["z"],
												jsonCmdReceive["t"]);
												RoArmM2_goalPosMove();
												break;
	case CMD_SERVO_RAD_FEEDBACK:
												RoArmM2_getPosByServoFeedback();
												RoArmM2_infoFeedback();
												break;

	case CMD_EOAT_HAND_CTRL: 
												RoArmM2_handJointCtrlRad(1,
												jsonCmdReceive["cmd"],
												jsonCmdReceive["spd"],
												jsonCmdReceive["acc"]
												);break;
	case CMD_EOAT_GRAB_TORQUE:
												RoArmM2_handTorqueCtrl(
												jsonCmdReceive["tor"]
												);break;

	case CMD_SET_JOINT_PID:
												RoArmM2_setJointPID(
												jsonCmdReceive["joint"],
												jsonCmdReceive["p"],
												jsonCmdReceive["i"]
												);break;
	case CMD_RESET_PID:		RoArmM2_resetPID();break;

	// set a new x-axis.
	case CMD_SET_NEW_X: 	setNewAxisX(
												jsonCmdReceive["xAxisAngle"]
												);break;
	case CMD_DELAY_MILLIS:
												RoArmM2_delayMillis(
												jsonCmdReceive["cmd"]
												);break;
	case CMD_DYNAMIC_ADAPTATION: 
												RoArmM2_dynamicAdaptation(
												jsonCmdReceive["mode"],
												jsonCmdReceive["b"],
												jsonCmdReceive["s"],
												jsonCmdReceive["e"],
												jsonCmdReceive["h"]
												);break;
	// this two funcs are NOT for UGV.
	// case CMD_SWITCH_CTRL: switchCtrl(
	// 											jsonCmdReceive["pwm_a"],
	// 											jsonCmdReceive["pwm_b"]
	// 											);break;
	// case CMD_LIGHT_CTRL:	lightCtrl(
	// 											jsonCmdReceive["led"]
	// 											);break;
	case CMD_SWITCH_OFF:  switchEmergencyStop();break;
	case CMD_SINGLE_JOINT_ANGLE:
												RoArmM2_singleJointAngleCtrl(
												jsonCmdReceive["joint"],
												jsonCmdReceive["angle"],
												jsonCmdReceive["spd"],
												jsonCmdReceive["acc"]
												);break;
	case CMD_JOINTS_ANGLE_CTRL:
												RoArmM2_allJointsAngleCtrl(
												jsonCmdReceive["b"],
												jsonCmdReceive["s"],
												jsonCmdReceive["e"],
												jsonCmdReceive["h"],
												jsonCmdReceive["spd"],
												jsonCmdReceive["acc"]
												);break;
// constant ctrl
// m: 0 - angle
//    1 - xyzt
// cmd: 0 - stop
// 		  1 - increase
// 		  2 - decrease
// {"T":123,"m":0,"axis":0,"cmd":0,"spd":0}
	case CMD_CONSTANT_CTRL:
												constantCtrl(
												jsonCmdReceive["m"],
												jsonCmdReceive["axis"],
												jsonCmdReceive["cmd"],
												jsonCmdReceive["spd"]
												);break;




	// mission & steps edit & file edit.
	case CMD_SCAN_FILES:  scanFlashContents();
												break;
	case CMD_CREATE_FILE: createFile(
												jsonCmdReceive["name"],
												jsonCmdReceive["content"]
												);break;
	case CMD_READ_FILE:		readFile(
												jsonCmdReceive["name"]
												);break;
	case CMD_DELETE_FILE: deleteFile(
												jsonCmdReceive["name"]
												);break;
	case CMD_APPEND_LINE:	appendLine(
												jsonCmdReceive["name"],
												jsonCmdReceive["content"]
												);break;
	case CMD_INSERT_LINE: insertLine(
												jsonCmdReceive["name"],
												jsonCmdReceive["lineNum"],
												jsonCmdReceive["content"]
												);break;
	case CMD_REPLACE_LINE:
												replaceLine(
												jsonCmdReceive["name"],
												jsonCmdReceive["lineNum"],
												jsonCmdReceive["content"]
												);break;
	case CMD_READ_LINE:   readSingleLine(
												jsonCmdReceive["name"],
												jsonCmdReceive["lineNum"]
												);break;
	case CMD_DELETE_LINE: deleteSingleLine(
												jsonCmdReceive["name"],
												jsonCmdReceive["lineNum"]
												);break;


	case CMD_TORQUE_CTRL: servoTorqueCtrl(254,
												jsonCmdReceive["cmd"]);
												break;


	case CMD_CREATE_MISSION:
												createMission(
												jsonCmdReceive["name"],
												jsonCmdReceive["intro"]
												);break;
	case CMD_MISSION_CONTENT:
												missionContent(
												jsonCmdReceive["name"]
												);break;
	case CMD_APPEND_STEP_JSON: 
												appendStepJson(
												jsonCmdReceive["name"],
												jsonCmdReceive["step"]
												);break;
	case CMD_APPEND_STEP_FB:
												appendStepFB(
												jsonCmdReceive["name"],
												jsonCmdReceive["spd"]
												);break;
	case CMD_APPEND_DELAY:
												appendDelayCmd(
												jsonCmdReceive["name"],
												jsonCmdReceive["delay"]
												);break;
	case CMD_INSERT_STEP_JSON:
												insertStepJson(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"],
												jsonCmdReceive["step"]
												);break;
	case CMD_INSERT_STEP_FB:
												insertStepFB(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"],
												jsonCmdReceive["spd"]
												);break;
	case CMD_INSERT_DELAY:
												insertDelayCmd(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"],
												jsonCmdReceive["spd"]
												);break;
	case CMD_REPLACE_STEP_JSON:
												replaceStepJson(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"],
												jsonCmdReceive["step"]
												);break;
	case CMD_REPLACE_STEP_FB:
												replaceStepFB(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"],
												jsonCmdReceive["spd"]
												);break;
	case CMD_REPLACE_DELAY:
												replaceDelayCmd(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"],
												jsonCmdReceive["delay"]
												);break;
	case CMD_DELETE_STEP: deleteStep(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"]
												);break;

	case CMD_MOVE_TO_STEP:
												moveToStep(
												jsonCmdReceive["name"],
												jsonCmdReceive["stepNum"]
												);break;
	case CMD_MISSION_PLAY:
												missionPlay(
												jsonCmdReceive["name"],
												jsonCmdReceive["times"]
												);break;



	// esp-now settings.
  case CMD_BROADCAST_FOLLOWER:
  											changeBroadcastMode(
  											jsonCmdReceive["mode"],
  											jsonCmdReceive["mac"]
  											);break;
  case CMD_ESP_NOW_CONFIG:
  											changeEspNowMode(
  											jsonCmdReceive["mode"]
  											);break;
  case CMD_GET_MAC_ADDRESS: 
  											getThisDevMacAddress();
  											break;
  case CMD_ESP_NOW_ADD_FOLLOWER:
  											registerNewFollowerToPeer(
  											jsonCmdReceive["mac"]);break;
  case CMD_ESP_NOW_REMOVE_FOLLOWER:
  											deleteFollower(
  											jsonCmdReceive["mac"]);break;
  case CMD_ESP_NOW_GROUP_CTRL:
  											espNowGroupSend(
  											jsonCmdReceive["dev"],
  											jsonCmdReceive["b"],
  											jsonCmdReceive["s"],
  											jsonCmdReceive["e"],
  											jsonCmdReceive["h"],
  											jsonCmdReceive["cmd"],
  											jsonCmdReceive["megs"]
  											);break;
  case CMD_ESP_NOW_SINGLE:
  											espNowSingleDevSend(
  											jsonCmdReceive["mac"],
  											jsonCmdReceive["dev"],
  											jsonCmdReceive["b"],
  											jsonCmdReceive["s"],
  											jsonCmdReceive["e"],
  											jsonCmdReceive["h"],
  											jsonCmdReceive["cmd"],
  											jsonCmdReceive["megs"]
  											);break;



	// wifi settings.
	case CMD_WIFI_ON_BOOT: 
												configWifiModeOnBoot(
												jsonCmdReceive["cmd"]
												);break;
	case CMD_SET_AP: 			wifiModeAP(
									 			jsonCmdReceive["ssid"],
									 			jsonCmdReceive["password"]
									 			);break;
	case CMD_SET_STA: 		wifiModeSTA(
												jsonCmdReceive["ssid"],
												jsonCmdReceive["password"]
												);break;
	case CMD_WIFI_APSTA: 	wifiModeAPSTA(
										 	 	jsonCmdReceive["ap_ssid"],
											 	jsonCmdReceive["ap_password"],
											 	jsonCmdReceive["sta_ssid"],
											 	jsonCmdReceive["sta_password"]
											 	);break;
	case CMD_WIFI_INFO: 	wifiStatusFeedback();break;
	case CMD_WIFI_CONFIG_CREATE_BY_STATUS: 
												createWifiConfigFileByStatus();break;
	case CMD_WIFI_CONFIG_CREATE_BY_INPUT: 
												createWifiConfigFileByInput(
												jsonCmdReceive["mode"],
												jsonCmdReceive["ap_ssid"],
												jsonCmdReceive["ap_password"],
												jsonCmdReceive["sta_ssid"],
												jsonCmdReceive["sta_password"]
												);break;
	case CMD_WIFI_STOP: 	wifiStop();break;



	// servo settings.
	case CMD_SET_SERVO_ID:
												changeID(
												jsonCmdReceive["raw"],
												jsonCmdReceive["new"]
												);break;
	case CMD_SET_MIDDLE:  setMiddlePos(
												jsonCmdReceive["id"]
												);break;
	case CMD_SET_SERVO_PID: 
												setServosPID(
												jsonCmdReceive["id"],
												jsonCmdReceive["p"]
												);break;

	// esp-32 dev ctrl.
	case CMD_REBOOT: 			esp_restart();break;
	case CMD_FREE_FLASH_SPACE:
												freeFlashSpace();break;
	case CMD_BOOT_MISSION_INFO:
												missionContent("boot");break;
	case CMD_RESET_BOOT_MISSION:
												deleteFile("boot.mission");
												createFile("boot", "these cmds run automatically at boot.");
												break;
	case CMD_NVS_CLEAR:		nvs_flash_erase();
												delay(1000);
												nvs_flash_init();
												break;
	case CMD_INFO_PRINT:	configInfoPrint(
												jsonCmdReceive["cmd"]
												);break;

	// mainType & moduleType settings.
	case CMD_MM_TYPE_SET: mm_settings(
												jsonCmdReceive["main"],
												jsonCmdReceive["module"]
												);
												saveMainTypeModuleTpye(
												jsonCmdReceive["main"],
												jsonCmdReceive["module"]
												);
												break;

	// Distance measurement
	case CMD_DISTANCE_SET_A: 	distanceSetPointA();
												break;
	case CMD_DISTANCE_SET_B:	distanceSetPointB();
												break;
	case CMD_DISTANCE_RESET:	distanceReset();
												break;
	case CMD_DISTANCE_REPORT:	distanceReport();
												break;
	}
}


void serialCtrl() {
  static String receivedData;

  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    receivedData += receivedChar;

    // Detect the end of the JSON string based on a specific termination character
    if (receivedChar == '\n') {
      // Now we have received the complete JSON string
      DeserializationError err = deserializeJson(jsonCmdReceive, receivedData);
      if (err == DeserializationError::Ok) {
  			if (InfoPrint == 1 && uartCmdEcho) {
  				Serial.print(receivedData);
  			}
        jsonCmdReceiveHandler();
      } else {
        // Handle JSON parsing error here
      }
      // Reset the receivedData for the next JSON string
      receivedData = "";
    }
  }
}