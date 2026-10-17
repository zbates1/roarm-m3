// Added for roarm-m3 (not in Waveshare's original): compact high-rate telemetry
// for experiments. Off by default and not saved.
//
// {"T":118,"cmd":1}  start: after every control-loop pass reads the servos, print
//   @<ms>,<pos b>,<pos s>,<pos e>,<pos w>,<spd b>,<spd s>,<spd e>,<spd w>,<load s>,<load e>,<load w>,<mA s>,<mA e>
//   pos/spd are the servos' raw counts (4096 per turn) and counts/s; s = the
//   driving shoulder servo. ~60 characters per line, versus ~300 for T:1051.
// {"T":118,"cmd":0}  stop.

bool telemetryOn = false;

void telemetryConfig(int cmd) {
  telemetryOn = (cmd == 1);
}

void telemetryTick() {
  if (!telemetryOn) return;
  const int ids[4] = {BASE_SERVO_ID, SHOULDER_DRIVING_SERVO_ID, ELBOW_SERVO_ID, WRIST_SERVO_ID};
  char buf[160];
  int n = snprintf(buf, sizeof(buf), "@%lu", (unsigned long)millis());
  for (int k = 0; k < 4; k++) n += snprintf(buf + n, sizeof(buf) - n, ",%d", servoFeedback[ids[k] - 11].pos);
  for (int k = 0; k < 4; k++) n += snprintf(buf + n, sizeof(buf) - n, ",%d", servoFeedback[ids[k] - 11].speed);
  for (int k = 1; k < 4; k++) n += snprintf(buf + n, sizeof(buf) - n, ",%d", servoFeedback[ids[k] - 11].load);
  snprintf(buf + n, sizeof(buf) - n, ",%d,%d",
           (int)servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].current, (int)servoFeedback[ELBOW_SERVO_ID - 11].current);
  Serial.println(buf);
}
