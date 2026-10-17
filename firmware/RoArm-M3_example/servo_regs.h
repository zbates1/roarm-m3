// Added for roarm-m3 (not in Waveshare's original): read any servo register, and
// write a small whitelist of control registers so the servo's own loop can be
// tuned (Waveshare's T:108 only sets P and I).
//
// Read:  {"T":116,"id":12,"addr":22,"len":1}
//    ->  {"T":1161,"id":12,"addr":22,"len":1,"val":32,"ok":1}
// Write: {"T":117,"id":12,"addr":22,"len":1,"val":40}
//    ->  {"T":1171,"id":12,"addr":22,"len":1,"val":40,"ok":1}   (val is read back)
//
// Servo IDs: 11 base, 12 + 13 shoulder (driving + driven, set both), 14 elbow,
// 15 wrist, 16 roll, 17 gripper. Registers (Feetech STS table):
//   21 P, 22 D, 23 I                1 byte
//   24 minimum startup force        2 bytes
//   26 CW dead zone, 27 CCW dead zone   1 byte (encoder steps)
//   41 acceleration                 1 byte
// Anything else (ID, baud, limits, offsets, ...) is refused. Writes never unlock
// the servo's EEPROM (register 55), so nothing is saved by this command itself.

bool servoRegWritable(int addr, int len) {
  if (len == 1) return addr == 21 || addr == 22 || addr == 23 || addr == 26 || addr == 27 || addr == 41;
  if (len == 2) return addr == 24;
  return false;
}

int servoRegGet(int id, int addr, int len) {
  return len == 2 ? st.readWord(id, addr) : st.readByte(id, addr);
}

void servoRegReply(int type, int id, int addr, int len, int val, bool ok) {
  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = type;
  jsonInfoHttp["id"] = id;
  jsonInfoHttp["addr"] = addr;
  jsonInfoHttp["len"] = len;
  jsonInfoHttp["val"] = val;
  jsonInfoHttp["ok"] = ok ? 1 : 0;
  String out;
  serializeJson(jsonInfoHttp, out);
  Serial.println(out);
}

void servoRegRead(int id, int addr, int len) {
  if (id < 1 || id > 253 || addr < 0 || addr > 255 || (len != 1 && len != 2)) {
    servoRegReply(1161, id, addr, len, -1, false);
    return;
  }
  int v = servoRegGet(id, addr, len);
  servoRegReply(1161, id, addr, len, v, v >= 0);
}

void servoRegWrite(int id, int addr, int len, int val) {
  bool valid = id >= 11 && id <= 17 && servoRegWritable(addr, len)
               && val >= 0 && val <= (len == 2 ? 65535 : 255);
  if (!valid) {
    servoRegReply(1171, id, addr, len, -1, false);
    return;
  }
  if (len == 2) st.writeWord(id, addr, val);
  else st.writeByte(id, addr, val);
  int back = servoRegGet(id, addr, len);
  servoRegReply(1171, id, addr, len, back, back == val);
}
