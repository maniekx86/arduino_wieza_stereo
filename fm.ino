/* Wieza stereo
 * 
 * maniek86, 2023
 * 
 * 
 */

//#define TSA5523_ADDR (0x60)
#define TSA5523_ADDR (0xC2 >> 1)


void tsa5523_setFMFrequency(uint32_t freq_khz) {
  byte db1, db2, cb, pb;
  pb = 0xa4; // band A only
  cb = 0x88;

  freq_khz += 10700;
  freq_khz /= 50;
  db1 = (freq_khz >> 8) & 0x7F;
  db2 = freq_khz & 0xFF;

  Wire.beginTransmission(TSA5523_ADDR);
  Wire.write(db1);
  Wire.write(db2);
  Wire.write(cb);
  Wire.write(pb);
  Wire.endTransmission();
}

void tsa5523_setMute() {
  uint32_t freq_khz=87500;
  byte db1, db2, cb, pb;
  pb = 0b10101100; //0xa4; // band A only
  cb = 0b10001000;//0x88;
  
  freq_khz += 10700;
  freq_khz /= 50;
  db1 = (freq_khz >> 8) & 0x7F;
  db2 = freq_khz & 0xFF;

  Wire.beginTransmission(TSA5523_ADDR);
  Wire.write(db1);
  Wire.write(db2);
  Wire.write(cb);
  Wire.write(pb);
  Wire.endTransmission();
}


// READ register:
/* bit 4 - Tune
 * bit 0 - Stereo
 * 
 */

byte fm_readRegister() {
  Wire.requestFrom(TSA5523_ADDR,1);
  byte a=Wire.read();
  Wire.endTransmission();
  return a;
}
