void bmi160_write_reg(uint8_t reg, uint8_t value) {
  hspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(HSPI_CS, LOW);
  hspi.transfer(reg & 0x7F);
  hspi.transfer(value);
  digitalWrite(HSPI_CS, HIGH);
  hspi.endTransaction();
  delayMicroseconds(10);
  
}
uint8_t bmi160_read_reg(uint8_t reg){
  hspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(HSPI_CS, LOW);
  hspi.transfer(reg | 0x80);
  uint8_t value = hspi.transfer(0x00);
  digitalWrite(HSPI_CS, HIGH);
  hspi.endTransaction();
  return value;
}
bool bmi160_config_reg(uint8_t reg, uint8_t value, uint8_t expected, const char* reg_name, int retries) {
  for (int i = 0; i < retries; i++) {
    bmi160_write_reg(reg, value);
    delayMicroseconds(200);
    uint8_t read_value = bmi160_read_reg(reg);
    Serial.printf("%s (lần %d): 0x%02X (Mong muốn: 0x%02X)\n", reg_name, i + 1, read_value, expected);
    if (read_value == expected){
      return true;
    }
    Serial.printf("Thử lại %s...\n", reg_name);
    delay(5);
  }
  Serial.printf("Lỗi: Không thể cấu hình %s sau %d lần thử!\n", reg_name, retries);
  return false;
}
void READ_GYRO(){
  hspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(HSPI_CS, LOW);
  hspi.transfer(DATA_REG | 0x80);
  for (uint8_t i = 0; i < 12; i++) {
    data[i] = hspi.transfer(0x00);
  }
  digitalWrite(HSPI_CS, HIGH);
  hspi.endTransaction();
  for(int i = 0 ; i < 3 ; i ++){
    gyr[i] = (int16_t)(data[2 * i] | (data[2 * i + 1] << 8));
    gyr_f[i] = (1.0 - B_gyro) * gyr_f_prv[i] + B_gyro * gyr[i];
    gyr_dps[i] = gyr_f[i] * 0.0609756097560976f - gyr_bias[i] * 0.0609756097560976f;
    acc[i] = (int16_t)(data[2 * i + 6] | (data[2 * i + 7] << 8));
    acc_f[i] = (1.0 - B_accel) * acc_f_prv[i] + B_accel * acc[i];
    acc_mps[i] = acc_f[i] * 0.00048828125f - acc_bias[i] * 0.00048828125f;
  }
  //Convert to frame 
  gyr_fr_dps[0] = -gyr_dps[1];
  gyr_fr_dps[1] = gyr_dps[0];
  gyr_fr_dps[2] = gyr_dps[2];
  //Update 
  acc_f_prv[0] = acc_f[0];
  acc_f_prv[1] = acc_f[1];
  acc_f_prv[2] = acc_f[2];
  gyr_f_prv[0] = gyr_f[0];
  gyr_f_prv[1] = gyr_f[1];
  gyr_f_prv[2] = gyr_f[2];
}
void GYRO_CALIBRATE(bool cali_acc){
  int count = 0;
  float bias[6];
  uint32_t t = micros();
  if(cali_acc){
     while(count < 4000){
         READ_GYRO();
        for(int i = 0 ; i < 3 ; i ++){
          bias[i] += acc[i];
          bias[i + 3] += gyr[i];
        }
         count++;
         while(micros() - t < 2500);
     }
    for(int i = 0 ; i < 6 ; i ++){
       bias[i] /= count;
    }
    bias[2] -= 2048.0;
    for(int i = 0 ; i < 3 ; i ++){
            acc_bias[i] = bias[i];
            gyr_bias[i] = bias[i + 3];
    }
  }
}
void SENSOR_CONFIG(void){
  uint8_t acc_conf,acc_range,gyr_conf,gyr_range;
  uint8_t chip_id = bmi160_read_reg(CHIPID);
  Serial.printf("BMI160 Chip ID: 0x%02X (Expected: 0xD1)\n", chip_id);
  Serial.println("Soft reset...");
  bmi160_write_reg(CMD, 0xB6);
  delay(10);
  bool config_ok = true;
  config_ok &= bmi160_config_reg(ACC_CONF, 0x0C, 0x0C, "ACC_CONF", 3); // ODR 1600 Hz, OSR4
  config_ok &= bmi160_config_reg(ACC_RANGE, 0x0C, 0x0C, "ACC_RANGE", 3); // ±16g
  config_ok &= bmi160_config_reg(GYR_CONF, 0x0C, 0x0C, "GYR_CONF", 3); // ODR 1600 Hz, OSR4
  config_ok &= bmi160_config_reg(GYR_RANGE, 0x00, 0x00, "GYR_RANGE", 3); // ±2000 °/s
  if (!config_ok) {
    Serial.println("Lỗi: Một hoặc nhiều thanh ghi cấu hình thất bại!");
  }
  Serial.println("Bật gia tốc kế...");
  bmi160_write_reg(CMD, 0x11);
  delay(10);
  Serial.println("Bật con quay...");
  bmi160_write_reg(CMD, 0x15);
  delay(100);
  //CHECK CONFIG
  uint8_t pmu_status = bmi160_read_reg(PMU_STATUS);
  uint8_t acc_status = (pmu_status >> 4) & 0x03;
  uint8_t gyr_status = (pmu_status >> 2) & 0x03;
  Serial.printf("PMU Status: 0x%02X | Acc Mode: %s | Gyr Mode: %s\n",pmu_status,acc_status == 0x01 ? "Normal" : (acc_status == 0x00 ? "Suspend" : "Low Power"),gyr_status == 0x01 ? "Normal" : (gyr_status == 0x00 ? "Suspend" : gyr_status == 0x03 ? "Fast Start-up" : "Unknown"));
  acc_conf = bmi160_read_reg(ACC_CONF);
  acc_range = bmi160_read_reg(ACC_RANGE);
  gyr_conf = bmi160_read_reg(GYR_CONF);
  gyr_range = bmi160_read_reg(GYR_RANGE);
  Serial.printf("ACC_CONF: 0x%02X | ACC_RANGE: 0x%02X \n", acc_conf, acc_range);
  Serial.printf("GYR_CONF: 0x%02X | GYR_RANGE: 0x%02X \n", gyr_conf , gyr_range);
  uint8_t err_reg = bmi160_read_reg(ERR_REG);
  Serial.printf("ERR_REG: 0x%02X (0x00 = No Error)\n", err_reg);
  uint8_t status = bmi160_read_reg(STATUS);
  Serial.printf("STATUS: 0x%02X | Acc Data Ready: %d\n", status, (status >> 3) & 0x01);
}