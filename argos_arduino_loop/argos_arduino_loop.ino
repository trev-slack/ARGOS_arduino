/****************************************************************
 * ARGOS Arduino Loop
 * Team ARGOS - University of Colorado Boulder
 * Takes in sensor data and publishes to ROS, takes ROS Messages and controls motors/mast/camera
 * Necessary Sensors: IMU, 4x USRF, Temperature, 4x Motor encoders, 4x motor controllers
 * Author: Trevor Slack
 * Email: trevor.slack@colorado.edu
 * Date: 3/24/2021
 ***************************************************************/

#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
// critical for ROS
#define USE_USBCON

#include <ros.h>
#include <sensor_msgs/Imu.h>
// ros node
ros::NodeHandle  imu_node;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/argos/imu/data",&imu_msg);


//#define SERIAL_PORT Serial
#define WIRE_PORT Wire  // Your desired Wire port.     
#define AD0_VAL   1     // The value of the last bit of the I2C address.
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when
                        // the ADR jumper is closed the value becomes 0


ICM_20948_I2C myICM;  // create an ICM_20948_I2C object



void setup() {
  // ros node
  //imu_node.getHardware()->setBaud(115200);
  imu_node.initNode();
  imu_node.advertise(imu_pub);
  imu_msg.header.frame_id = "base_link";

  //SERIAL_PORT.begin(57600); // Start the serial console
  ////SERIAL_PORT.println(F("ICM-20948 Example"));

  delay(100);

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while( !initialized ){

    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
    myICM.begin( WIRE_PORT, AD0_VAL );

    ////SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
    //SERIAL_PORT.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      //SERIAL_PORT.println( F("Trying again...") );
      delay(500);
    }else{
      initialized = true;
    }
  }

  //SERIAL_PORT.println(F("Device connected!"));

  // The ICM-20948 is awake and ready but hasn't been configured. Let's step through the configuration
  // sequence from InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".

  bool success = true; // Use success to show if the configuration was successful

  // Configure clock source through PWR_MGMT_1
  // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  success &= (myICM.setClockSource(ICM_20948_Clock_Auto) == ICM_20948_Stat_Ok); // This is shorthand: success will be set to false if setClockSource fails

  // Enable accel and gyro sensors through PWR_MGMT_2
  // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
  success &= (myICM.setBank(0) == ICM_20948_Stat_Ok); // Select Bank 0
  uint8_t pwrMgmt2 = 0x40; // Set the reserved bit 6
  success &= (myICM.write(AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1) == ICM_20948_Stat_Ok); // Write one byte to the PWR_MGMT_2 register

  // Configure I2C_Master/Gyro/Accel in Low Power Mode (cycled) with LP_CONFIG
  success &= (myICM.setSampleMode( (ICM_20948_Internal_Mst | ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Cycled ) == ICM_20948_Stat_Ok);

  // Disable the FIFO
  success &= (myICM.enableFIFO(false) == ICM_20948_Stat_Ok);

  // Disable the DMP
  success &= (myICM.enableDMP(false) == ICM_20948_Stat_Ok);

  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
  ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  myFSS.a = gpm4;         // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                          // gpm2
                          // gpm4
                          // gpm8
                          // gpm16
  myFSS.g = dps2000;       // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                          // dps250
                          // dps500
                          // dps1000
                          // dps2000
  success &= (myICM.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS ) == ICM_20948_Stat_Ok);

  // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
  // If we see this interrupt, we'll need to reset the FIFO
  //success &= (myICM.intEnableOverflowFIFO( 0x1F ) == ICM_20948_Stat_Ok); // Enable the interrupt on all FIFOs

  // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
  // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
  success &= (myICM.setBank(0) == ICM_20948_Stat_Ok); // Select Bank 0
  uint8_t zero = 0;
  success &= (myICM.write(AGB0_REG_FIFO_EN_1, &zero, 1) == ICM_20948_Stat_Ok);
  // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
  success &= (myICM.write(AGB0_REG_FIFO_EN_2, &zero, 1) == ICM_20948_Stat_Ok);

  // Turn off data ready interrupt through INT_ENABLE_1
  success &= (myICM.intEnableRawDataReady(false) == ICM_20948_Stat_Ok);

  // Reset FIFO through FIFO_RST
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
  mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  myICM.setSampleRate( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt ); // ** Note: comment this line to leave the sample rates at the maximum **
  
  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  success &= (myICM.setDMPstartAddress() == ICM_20948_Stat_Ok); // Defaults to DMP_START_ADDRESS

  // Now load the DMP firmware
  success &= (myICM.loadDMPFirmware() == ICM_20948_Stat_Ok);

  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  success &= (myICM.setDMPstartAddress() == ICM_20948_Stat_Ok); // Defaults to DMP_START_ADDRESS

  // Set the Hardware Fix Disable register to 0x48
  success &= (myICM.setBank(0) == ICM_20948_Stat_Ok); // Select Bank 0
  uint8_t fix = 0x48;
  success &= (myICM.write(AGB0_REG_HW_FIX_DISABLE, &fix, 1) == ICM_20948_Stat_Ok);
  
  // Set the Single FIFO Priority Select register to 0xE4
  success &= (myICM.setBank(0) == ICM_20948_Stat_Ok); // Select Bank 0
  uint8_t fifoPrio = 0xE4;
  success &= (myICM.write(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1) == ICM_20948_Stat_Ok);
  
  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
  const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
  success &= (myICM.writeDMPmems(ACC_SCALE, 4, &accScale[0]) == ICM_20948_Stat_Ok); // Write accScale to ACC_SCALE DMP register
  // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
  const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  success &= (myICM.writeDMPmems(ACC_SCALE2, 4, &accScale2[0]) == ICM_20948_Stat_Ok); // Write accScale2 to ACC_SCALE2 DMP register

  // Configure Compass mount matrix and scale to DMP
  // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
  // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
  // Each compass axis will be converted as below:
  // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
  // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
  // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
  // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
  // 2^30 / 6.66666 = 161061273 = 0x9999999
  const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99}; // Value taken from InvenSense Nucleo example
  const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
  success &= (myICM.writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]) == ICM_20948_Stat_Ok);

  // Configure the B2S Mounting Matrix
  const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
  success &= (myICM.writeDMPmems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]) == ICM_20948_Stat_Ok);

  // Configure the DMP Gyro Scaling Factor
  // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
  //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
  //            10=102.2727Hz sample rate, ... etc.
  // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
  success &= (myICM.setGyroSF(19, 3) == ICM_20948_Stat_Ok); // 19 = 55Hz (see above), 3 = 2000dps (see above)
  
  // Configure the Gyro full scale
  // 2000dps : 2^28
  // 1000dps : 2^27
  //  500dps : 2^26
  //  250dps : 2^25
  const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
  success &= (myICM.writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScale[0]) == ICM_20948_Stat_Ok);

  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  //const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // InvenSense Nucleo example uses 225Hz
  success &= (myICM.writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]) == ICM_20948_Stat_Ok);
  
  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  //const unsigned char accelAlphaVar[4] = {0x06, 0x66, 0x66, 0x66}; // Value taken from InvenSense Nucleo example
  success &= (myICM.writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]) == ICM_20948_Stat_Ok);
  
  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  //const unsigned char accelAVar[4] = {0x39, 0x99, 0x99, 0x9A}; // Value taken from InvenSense Nucleo example
  success &= (myICM.writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]) == ICM_20948_Stat_Ok);
  
  // Configure the Accel Cal Rate
  const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
  success &= (myICM.writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]) == ICM_20948_Stat_Ok);

  // Configure the Compass Time Buffer. The compass (magnetometer) is set to 100Hz (AK09916_mode_cont_100hz)
  // in startupMagnetometer. We need to set CPASS_TIME_BUFFER to 100 too.
  const unsigned char compassRate[2] = {0x00, 0x64}; // 100Hz
  success &= (myICM.writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]) == ICM_20948_Stat_Ok);
  
  // Enable DMP interrupt
  // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
  //success &= (myICM.intEnableDMP(true) == ICM_20948_Stat_Ok);

  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP Game Rotation Vector sensor (Quat9)
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // Enable additional sensors / features
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to 20Hz
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to 20Hz
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to 20Hz
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to 1Hz
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 54) == ICM_20948_Stat_Ok); // Set to 1Hz
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 54) == ICM_20948_Stat_Ok); // Set to 1Hz

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if( success )
  {
    //SERIAL_PORT.println(F("DMP enabled!"));
  }
  else
  {
    //SERIAL_PORT.println(F("Enable DMP failed!"));
    //SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }
}

void loop()
{
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if(( myICM.status == ICM_20948_Stat_Ok ) || ( myICM.status == ICM_20948_Stat_FIFOMoreDataAvail )) // Was valid data available?
  {
    ////SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) //SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) //SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) //SERIAL_PORT.print( "0" );
    ////SERIAL_PORT.println( data.header, HEX );

    if ( (data.header & DMP_header_bitmap_Quat6) > 0 ) // Check for orientation data (Quat9)
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      ////SERIAL_PORT.print("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat9.Data.Q3);

      // Scale to +/- 1
      //double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      //double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      //double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt(1.0-((q1*q1)+(q2*q2)+(q3*q3))); // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      if (q0<=1) {
        imu_msg.orientation.x = q1;
        imu_msg.orientation.y = q2;
        imu_msg.orientation.z = q3;
        imu_msg.orientation.w = q0;        
      }
      
      
      //SERIAL_PORT.print(F("Q0:"));
      //SERIAL_PORT.print(q0, 3);
      //SERIAL_PORT.print(F(" Q1:"));
      //SERIAL_PORT.print(q1, 3);
      //SERIAL_PORT.print(F(" Q2:"));
      //SERIAL_PORT.print(q2, 3);
      //SERIAL_PORT.print(F(" Q3:"));
      //SERIAL_PORT.print(q3, 3);
      //SERIAL_PORT.print(F(" Accuracy:"));
      //SERIAL_PORT.println(data.Quat9.Data.Accuracy);
    }


    if ( (data.header & DMP_header_bitmap_Gyro) > 0 ) // Check for Gyro
    {
      float x = (float)data.Raw_Gyro.Data.X; // Extract the raw gyro data
      float y = (float)data.Raw_Gyro.Data.Y; 
      float z = (float)data.Raw_Gyro.Data.Z; 

      imu_msg.angular_velocity.x = x;
      imu_msg.angular_velocity.y = y;
      imu_msg.angular_velocity.z = z;
      
    
      //SERIAL_PORT.print(F("Gyro: X:"));
      //SERIAL_PORT.print(x);
      //SERIAL_PORT.print(F(" Y:"));
      //SERIAL_PORT.print(y);
      //SERIAL_PORT.print(F(" Z:"));
      //SERIAL_PORT.println(z);
    }
    imu_pub.publish(&imu_msg);


  }

  if ( myICM.status != ICM_20948_Stat_FIFOMoreDataAvail ) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
  //delay(1);
  imu_node.spinOnce();
}
