  #include "gyroscope.h"

  static LSM6DSOXClass imu_gyro = LSM6DSOXClass(Wire, GYRO_IMU_I2C_ADDR);

  static float gyro_calibration_x, gyro_calibration_y, gyro_calibration_z = 0.0;
  static float accel_calibration_x, accel_calibration_y; // accel_calibration_z = 0.0;
  static float angle_sum_x = 0.0, angle_sum_y = 0.0, angle_sum_z= 0.0;

  void calibrate_gyro(){
    float sample_rate = imu_gyro.gyroscopeSampleRate();
    float x_sum = 0.0, y_sum = 0.0, z_sum = 0.0;
    float x = 0.0, y = 0.0, z = 0.0;
    
    for(int i = 0; i < SAMPLING_AMOUNT; i++){
      int valid_measurement = fetch_gyro_data(x, y ,z);
      if(valid_measurement == 1){
        x_sum += x;
        y_sum += y;
        z_sum += z;
        delay(10);
      }else{
        Serial.printf("Invalid measurement (%d): %.3f, %.3f, %.3f\n", i, x, y, z);
      }
    }

    gyro_calibration_x = x_sum / SAMPLING_AMOUNT;
    gyro_calibration_y = y_sum / SAMPLING_AMOUNT;
    gyro_calibration_z = z_sum / SAMPLING_AMOUNT;
  }

  /**
  Calibration of acceleration data

  Calculates the offset in measurements (mean of value).
  Measurement should occur, dependening on the current sample rate.
  Assumption is, that the robot is stationary at the moment of fetching.

  Z-Axis is not calibrated, since gravitational forces (g = 9.81m/s^2) applies
  at the moment of fetching samples (ASSUMPTION!).
  */
  void calibrate_accel(){
    float sample_rate = imu_gyro.accelerationSampleRate();
    float x_sum = 0.0, y_sum = 0.0; // z_sum = 0.0;
    float x = 0.0, y = 0.0, z = 0.0;

    for(int i = 0; i < SAMPLING_AMOUNT; i++){
      int valid_measurement = fetch_acceleration_data(x, y, z);
      if(valid_measurement == 1){
        x_sum += x;
        y_sum += y;
        //z_sum += z;
        delay(1/sample_rate);  //Sample Rate specific delay 
      }
    }

    accel_calibration_x = x_sum / SAMPLING_AMOUNT;
    accel_calibration_y = y_sum / SAMPLING_AMOUNT;
    //accel_calibration_z = z_sum / SAMPLING_AMOUNT;
  }

  void init_IMU(){
    imu_gyro.begin();
    delay(2500);
    calibrate_gyro();
    calibrate_accel();
  }


  bool fetch_gyro_data(float &x, float &y, float &z){
    bool is_available = imu_gyro.gyroscopeAvailable();

    if(is_available){
      int valid_measurement = imu_gyro.readGyroscope(x , y, z);
      if(valid_measurement == 1){
        x -= gyro_calibration_x;
        y -= gyro_calibration_y;
        z -= gyro_calibration_z;
      }
      else{
        Serial.printf("ERROR x: %.3f, y: %.3f, z: %.3f\n", x, y, z);
        return false;
      }
    }
    
    return is_available;
  }

  bool fetch_acceleration_data(float &x, float &y, float &z){
    bool is_available = imu_gyro.accelerationAvailable();

    if(is_available){
      int valid_measurement = imu_gyro.readAcceleration(x, y, z);
      if(valid_measurement == 1){
        x -= accel_calibration_x;
        y -= accel_calibration_y;
      }
      else{
        Serial.printf("ERROR x: %.3f, y: %.3f, z: %.3f\n", x, y, z);
        return false;
      }
    }
    return is_available;
  }

  void quartenion_algorithm(float &angle_x, float &angle_y, float &angle_z){
    float g_x = 0.0, g_y = 0.0, g_z = 0.0;

    //Local Variable, to check how often to sample in a given time interval
    int SAMPLE_STEPS = 10;
    
    unsigned long start_time = millis();
    unsigned long curr_time = 0;
    float dt = 0.0;
    for(int i = 0; i < SAMPLE_STEPS; i++){
      if(fetch_gyro_data(g_x, g_y, g_z)){
        curr_time = millis();
        dt = (curr_time - start_time) / 1000.0f;
        start_time = curr_time;
        
        angle_sum_x += g_x * dt;
        angle_sum_y += g_y * dt;
        angle_sum_z += g_z * dt;
      }
      delay(10);
    }

    angle_x = angle_sum_x * CONVERS_CONST_DEGREES_PER_SECOND_2_RADIANS_PER_SECOND;
    angle_y = angle_sum_y * CONVERS_CONST_DEGREES_PER_SECOND_2_RADIANS_PER_SECOND;
    angle_z = angle_sum_z * CONVERS_CONST_DEGREES_PER_SECOND_2_RADIANS_PER_SECOND;
  }

