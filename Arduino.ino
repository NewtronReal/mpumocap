#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define DEBUG 1 //Set to 0 if you don't want to jam you serial port by debugging
#define BLENDER_3D //Don't know why they are given this didn't waste time for understating it
extern TwoWire Wire1; //externing another i2c wire protocol
MPU6050 MPU_A(0x69,&Wire);
MPU6050 MPU_B(0x68,&Wire);
MPU6050 MPU_C(0x68,&Wire1);
MPU6050 MPU_D(0x69,&Wire1);

bool mpuA_dmp_ready = false;
bool mpuB_dmp_ready = false;
//
bool mpuC_dmp_ready = false;
bool mpuD_dmp_ready = false;

/* Sensor connection status */
uint8_t mpuA_connected;
uint8_t mpuB_connected;
//
uint8_t mpuC_connected;
uint8_t mpuD_connected;


/* Sensor interrupt status */
uint8_t  mpuA_int_status;
uint8_t  mpuB_int_status;
//
uint8_t  mpuC_int_status;
uint8_t  mpuD_int_status;


/* Sensor DMP status */
uint8_t  mpuA_dma_status;
uint8_t  mpuB_dma_status;
//
uint8_t  mpuC_dma_status;
uint8_t  mpuD_dma_status;

/* Sensor packet size */
uint16_t mpuA_packet_size;
uint16_t mpuB_packet_size;
//
uint16_t mpuC_packet_size;
uint16_t mpuD_packet_size;

/* Sensor packet size */
uint16_t mpuA_fifo_count;
uint16_t mpuB_fifo_count;
//
uint16_t mpuC_fifo_count;
uint16_t mpuD_fifo_count;



/* Sensor FIFO buffers */
uint8_t  mpuA_fifo_buffer[64];
uint8_t  mpuB_fifo_buffer[64];
//
uint8_t  mpuC_fifo_buffer[64];
uint8_t  mpuD_fifo_buffer[64];

/* Sensor interrupt variables */
volatile bool mpuA_interrupt = false;
volatile bool mpuB_interrupt = false;
//
volatile bool mpuC_interrupt = false;
volatile bool mpuD_interrupt = false;

/* Sensor read data */
volatile bool mpuA_read_data = false;
volatile bool mpuB_read_data = false;
//
volatile bool mpuC_read_data = false;
volatile bool mpuD_read_data = false;

/* Quaternions */
Quaternion q1, q2, q3,q4; // [w, x, y, z]

/*  Sensor packet
    0  - $    - Begining of packet
    1  - 0x01 - Sensor 1 data [info]
    2  - Sensor A Qw high part
    3  - Sensor A Qw low  part
    4  - Sensor A Qx high part
    5  - Sensor A Qx low  part
    6  - Sensor A Qy high part
    7  - Sensor A Qy low  part
    8  - Sensor A Qz high part
    9  - Sensor A Qz low  part
    10 - 0x02  - Sensor 2 data [info]
    11 - Sensor B Qw high part
    12 - Sensor B Qw low  part
    13 - Sensor B Qx high part
    14 - Sensor B Qx low  part
    15 - Sensor B Qy high part
    16 - Sensor B Qy low  part
    17 - Sensor B Qz high part
    18 - Sensor B Qz low  part
    19 - 0x03  - Sensor 2 data [info]
    20 - Sensor C Qw high part
    21 - Sensor C Qw low  part
    22 - Sensor C Qx high part
    23 - Sensor C Qx low  part
    24 - Sensor C Qy high part
    25 - Sensor C Qy low  part
    26 - Sensor C Qz high part
    27 - Sensor C Qz low  part
    28 - 0x04  - Sensor 2 data [info]
    29 - Sensor D Qw high part
    30 - Sensor D Qw low  part
    31 - Sensor D Qx high part
    32 - Sensor D Qx low  part
    33 - Sensor D Qy high part
    34 - Sensor D Qy low  part
    35 - Sensor D Qz high part
    36 - Sensor D Qz low  part
    37 - 0x05  - Packet info data [info]
    38 - Packet counter
    39 - \r
    40 - \n
*/

uint8_t sensor_data_packet[41] = { '$', 0x01, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0x02, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0x03, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0x04, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0x05, 0x00, '\r', '\n'};

float mpu_b_X_accell_offset = -1839;
float mpu_b_Y_accell_offset = 3365;
float mpu_b_Z_accell_offset = 1463;
float mpu_b_X_gyro_offset   = 21;
float mpu_b_Y_gyro_offset   = 41;
float mpu_b_Z_gyro_offset   = -7;

float mpu_a_X_accell_offset = -2399;
float mpu_a_Y_accell_offset = 2042;
float mpu_a_Z_accell_offset = 1751;
float mpu_a_X_gyro_offset   = 70;
float mpu_a_Y_gyro_offset   = -43;
float mpu_a_Z_gyro_offset   = -37;

float mpu_c_X_accell_offset = -2399;
float mpu_c_Y_accell_offset = 2042;
float mpu_c_Z_accell_offset = 1751;
float mpu_c_X_gyro_offset   = 70;
float mpu_c_Y_gyro_offset   = -43;
float mpu_c_Z_gyro_offset   = -37;

float mpu_d_X_accell_offset = -1839;
float mpu_d_Y_accell_offset = 3365;
float mpu_d_Z_accell_offset = 1463;
float mpu_d_X_gyro_offset   = 21;
float mpu_d_Y_gyro_offset   = 41;
float mpu_d_Z_gyro_offset   = -7;

unsigned long reset_time;
unsigned long reset_prev_time = 0;
unsigned long reset_interval = 20000; //20s


//a useless function as of now
void dmpDataReady()
{
  //    mpuA_int_status = MPU_A.getIntStatus();
  //    mpuB_int_status = MPU_B.getIntStatus();
  //
  //    if(mpuA_int_status & 0x02)
  //    {
  //      mpuA_interrupt = true;
  //    }
  //
  //    if(mpuB_int_status > 0)
  //    {
  //      mpuB_interrupt = true;
  //    }
}

//checks whether all the sensors are properly connected or not
void check_sensor_connections()
{
  /* Checking sensor connections */

  Serial.println("Testing device connections");

  if (MPU_A.testConnection())
  {
    mpuA_connected = 1;
    Serial.println("Sensor A connected - TRUE");
  }
  else
  {
    mpuA_connected = 0;
    Serial.println("Sensor A connected - FALSE");
  }

  if (MPU_B.testConnection())
  {
    mpuB_connected = 1;
    Serial.println("Sensor B connected - TRUE");
  }
  else
  {
    mpuB_connected = 0;
    Serial.println("Sensor B connected - FALSE");
  }
  if (MPU_C.testConnection())
  {
    mpuC_connected = 1;
    Serial.println("Sensor C connected - TRUE");
  }
  else
  {
    mpuC_connected = 0;
    Serial.println("Sensor C connected - FALSE");
  }
  if (MPU_D.testConnection())
  {
    mpuD_connected = 1;
    Serial.println("Sensor D connected - TRUE");
  }
  else
  {
    mpuD_connected = 0;
    Serial.println("Sensor D connected - FALSE");
  }
}


//i guess initialization procedure involves caliberation and other stuff
//should be looking into i2c dev mpu6050 library for more details
void initialize_sensor_dma(uint8_t device)
{
  /* Sensor A */

  if (device == 0)
  {
    Serial.println("Sensor A DMP initialization");

    mpuA_dma_status = MPU_A.dmpInitialize();

    /* Set offsets */
    //MPU_A.setXAccelOffset(mpu_a_X_accell_offset);
    //MPU_A.setYAccelOffset(mpu_a_Y_accell_offset);
    MPU_A.setZAccelOffset(mpu_a_Z_accell_offset);
    MPU_A.setXGyroOffset(mpu_a_X_gyro_offset);
    MPU_A.setYGyroOffset(mpu_a_Y_gyro_offset);
    MPU_A.setZGyroOffset(mpu_a_Z_gyro_offset);

    if (mpuA_dma_status == 0)
    {
      Serial.println("Sensor A DMP initialized");
      Serial.println("Enabling DMP");

      MPU_A.setDMPEnabled(true);

      Serial.println("Sensor A Enabling interrupt detection");

      attachInterrupt(0, dmpDataReady, RISING);
      mpuA_int_status = MPU_A.getIntStatus();

      Serial.println("Sensor A DMP Ready, waiting for interrupt");
      mpuA_dmp_ready = true;

      mpuA_packet_size = MPU_A.dmpGetFIFOPacketSize();
    }
    else
    {
      Serial.println("Sensor A DMP initialization failed");
      if (mpuA_dma_status == 1)
      {
        Serial.println("Sensor A - Initial memory load failed!");
      }
      else if (mpuA_dma_status == 2)
      {
        Serial.println("Sensor A - DMP configuration updates failed!");
      }
    }
  }

  /* Sensor B */
  if (device == 1)
  {
    Serial.println("Sensor B DMP initialization");

    mpuB_dma_status = MPU_B.dmpInitialize();

    /* Set offsets */
    //MPU_B.setXAccelOffset(mpu_b_X_accell_offset);
    //MPU_B.setYAccelOffset(mpu_b_Y_accell_offset);
    MPU_B.setZAccelOffset(mpu_b_Z_accell_offset);
    MPU_B.setXGyroOffset(mpu_b_X_gyro_offset);
    MPU_B.setYGyroOffset(mpu_b_Y_gyro_offset);
    MPU_B.setZGyroOffset(mpu_b_Z_gyro_offset);

    if (mpuB_dma_status == 0)
    {
      Serial.println("Sensor B DMP initialized");
      Serial.println("Enabling DMP");

      MPU_B.setDMPEnabled(true);

      Serial.println("Sensor B Enabling interrupt detection");

      attachInterrupt(0, dmpDataReady, RISING);
      mpuB_int_status = MPU_B.getIntStatus();

      Serial.println("Sensor B DMP Ready, waiting for interrupt");
      mpuB_dmp_ready = true;

      mpuB_packet_size = MPU_B.dmpGetFIFOPacketSize();
    }
    else
    {
      Serial.println("Sensor B DMP initialization failed");
      if (mpuB_dma_status == 1)
      {
        Serial.println("Sensor B - Initial memory load failed!");
      }
      else if (mpuB_dma_status == 2)
      {
        Serial.println("Sensor B - DMP configuration updates failed!");
      }
    }
  }

  /* Sensor C */

  if (device == 2)
  {
    Serial.println("Sensor C DMP initialization");

    mpuC_dma_status = MPU_C.dmpInitialize();

    /* Set offsets */
    //MPU_A.setXAccelOffset(mpu_a_X_accell_offset);
    //MPU_A.setYAccelOffset(mpu_a_Y_accell_offset);
    MPU_C.setZAccelOffset(mpu_c_Z_accell_offset);
    MPU_C.setXGyroOffset(mpu_c_X_gyro_offset);
    MPU_C.setYGyroOffset(mpu_c_Y_gyro_offset);
    MPU_C.setZGyroOffset(mpu_c_Z_gyro_offset);

    if (mpuC_dma_status == 0)
    {
      Serial.println("Sensor C DMP initialized");
      Serial.println("Enabling DMP");

      MPU_C.setDMPEnabled(true);

      Serial.println("Sensor C Enabling interrupt detection");

      attachInterrupt(0, dmpDataReady, RISING);
      mpuC_int_status = MPU_C.getIntStatus();

      Serial.println("Sensor C DMP Ready, waiting for interrupt");
      mpuC_dmp_ready = true;

      mpuC_packet_size = MPU_C.dmpGetFIFOPacketSize();
    }
    else
    {
      Serial.println("Sensor C DMP initialization failed");
      if (mpuC_dma_status == 1)
      {
        Serial.println("Sensor C - Initial memory load failed!");
      }
      else if (mpuC_dma_status == 2)
      {
        Serial.println("Sensor C - DMP configuration updates failed!");
      }
    }
  }

  /* Sensor D */

  if (device == 3)
  {
    Serial.println("Sensor D DMP initialization");

    mpuD_dma_status = MPU_D.dmpInitialize();

    /* Set offsets */
    //MPU_A.setXAccelOffset(mpu_a_X_accell_offset);
    //MPU_A.setYAccelOffset(mpu_a_Y_accell_offset);
    MPU_D.setZAccelOffset(mpu_d_Z_accell_offset);
    MPU_D.setXGyroOffset(mpu_d_X_gyro_offset);
    MPU_D.setYGyroOffset(mpu_d_Y_gyro_offset);
    MPU_D.setZGyroOffset(mpu_d_Z_gyro_offset);

    if (mpuD_dma_status == 0)
    {
      Serial.println("Sensor D DMP initialized");
      Serial.println("Enabling DMP");

      MPU_D.setDMPEnabled(true);

      Serial.println("Sensor D Enabling interrupt detection");

      attachInterrupt(0, dmpDataReady, RISING);
      mpuD_int_status = MPU_D.getIntStatus();

      Serial.println("Sensor D DMP Ready, waiting for interrupt");
      mpuD_dmp_ready = true;

      Serial.println(true);

      mpuD_packet_size = MPU_D.dmpGetFIFOPacketSize();
    }
    else
    {
      Serial.println("Sensor D DMP initialization failed");
      if (mpuD_dma_status == 1)
      {
        Serial.println("Sensor D - Initial memory load failed!");
      }
      else if (mpuD_dma_status == 2)
      {
        Serial.println("Sensor D - DMP configuration updates failed!");
      }
    }
  }
}

//function sends data packet to serial port

void sensor_send_data_to_pc()
{
  if (mpuA_read_data && mpuB_read_data && mpuC_read_data && mpuD_read_data)
  {
    MPU_A.dmpGetQuaternion(&q1, mpuA_fifo_buffer);
    MPU_B.dmpGetQuaternion(&q2, mpuB_fifo_buffer);
    MPU_C.dmpGetQuaternion(&q3, mpuC_fifo_buffer);
    MPU_C.dmpGetQuaternion(&q3, mpuC_fifo_buffer);

#ifdef DEBUG
    Serial.print("Sensor A:\t");
    Serial.print(q1.w);
    Serial.print("\t");
    Serial.print(q1.x);
    Serial.print("\t");
    Serial.print(q1.y);
    Serial.print("\t");
    Serial.print(q1.z);
    Serial.print("\t");
    Serial.print("Sensor B:\t");
    Serial.print(q2.w);
    Serial.print("\t");
    Serial.print(q2.x);
    Serial.print("\t");
    Serial.print(q2.y);
    Serial.print("\t");
    Serial.println(q2.z);
    Serial.print("Sensor C:\t");
    Serial.print(q3.w);
    Serial.print("\t");
    Serial.print(q3.x);
    Serial.print("\t");
    Serial.print(q3.y);
    Serial.print("\t");
    Serial.print(q3.z);
    Serial.print("Sensor D:\t");
    Serial.print(q4.w);
    Serial.print("\t");
    Serial.print(q4.x);
    Serial.print("\t");
    Serial.print(q4.y);
    Serial.print("\t");
    Serial.print(q4.z);
    Serial.println("\t");
#endif

#ifdef BLENDER_3D

    /* Sensor A data */
    sensor_data_packet[2] = mpuA_fifo_buffer[0];
    sensor_data_packet[3] = mpuA_fifo_buffer[1];
    sensor_data_packet[4] = mpuA_fifo_buffer[4];
    sensor_data_packet[5] = mpuA_fifo_buffer[5];
    sensor_data_packet[6] = mpuA_fifo_buffer[8];
    sensor_data_packet[7] = mpuA_fifo_buffer[9];
    sensor_data_packet[8] = mpuA_fifo_buffer[12];
    sensor_data_packet[9] = mpuA_fifo_buffer[13];

    /* Sensor B data */
    sensor_data_packet[11] = mpuB_fifo_buffer[0];
    sensor_data_packet[12] = mpuB_fifo_buffer[1];
    sensor_data_packet[13] = mpuB_fifo_buffer[4];
    sensor_data_packet[14] = mpuB_fifo_buffer[5];
    sensor_data_packet[15] = mpuB_fifo_buffer[8];
    sensor_data_packet[16] = mpuB_fifo_buffer[9];
    sensor_data_packet[17] = mpuB_fifo_buffer[12];
    sensor_data_packet[18] = mpuB_fifo_buffer[13];

    /* Sensor C data */
    sensor_data_packet[20] = mpuC_fifo_buffer[0];
    sensor_data_packet[21] = mpuC_fifo_buffer[1];
    sensor_data_packet[22] = mpuC_fifo_buffer[4];
    sensor_data_packet[23] = mpuC_fifo_buffer[5];
    sensor_data_packet[24] = mpuC_fifo_buffer[8];
    sensor_data_packet[25] = mpuC_fifo_buffer[9];
    sensor_data_packet[26] = mpuC_fifo_buffer[12];
    sensor_data_packet[27] = mpuC_fifo_buffer[13];

    /* Sensor D data */
    sensor_data_packet[29] = mpuD_fifo_buffer[0];
    sensor_data_packet[30] = mpuD_fifo_buffer[1];
    sensor_data_packet[31] = mpuD_fifo_buffer[4];
    sensor_data_packet[32] = mpuD_fifo_buffer[5];
    sensor_data_packet[33] = mpuD_fifo_buffer[8];
    sensor_data_packet[34] = mpuD_fifo_buffer[9];
    sensor_data_packet[35] = mpuD_fifo_buffer[12];
    sensor_data_packet[36] = mpuD_fifo_buffer[13];

    Serial.write(sensor_data_packet, 41);

    /* Increment packet counter */
    sensor_data_packet[38]++;
    
#endif

    mpuA_read_data = false;
    mpuB_read_data = false;
    mpuC_read_data = false;
    mpuD_read_data = false;
  }
}

void setup() { 

  /* Initialize I2C */
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(22,23);//I2D connection 1 SCL0=D23 and SDA0=D22 on ESP32
  Wire1.begin(19,21);
  // TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  //not necassory for ESP ig usually the clock speed is set along with Wire.begin(sda,scl,freq)
  //in case of ESP32 boards ref(CanavarB's project)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  /* Setup UART */
  Serial.begin(115200);
  while (!Serial);

  /* Initialize sensors */
  Serial.println(F("Initializing I2C devices..."));
  MPU_A.initialize();
  MPU_B.initialize();
  MPU_C.initialize();
  MPU_D.initialize();

  /* Check connections */
  check_sensor_connections();

  /* Configure DMP */
  initialize_sensor_dma(0); //Sensor A
  initialize_sensor_dma(1); //Sensor B
  initialize_sensor_dma(2); //Sensor C
  initialize_sensor_dma(3); //Sensor D

  /* Sensors connection error handling */
  if (!mpuA_connected || !mpuB_connected || !mpuC_connected || !mpuD_connected)
  {
    Serial.println("ERROR: Both sensors must be connected!");

    while (1)
    {

    }
  }

  /* Sensors DMP error handling */
  if (!mpuA_dmp_ready || !mpuB_dmp_ready || !mpuC_dmp_ready || !mpuD_dmp_ready)
  {
    Serial.println("ERROR: Both sensors must have DMP initialized!");

    while (1)
    {

    }
  }
}

void loop() {

  /* Check interrupt status */
  mpuA_int_status = MPU_A.getIntStatus();
  mpuB_int_status = MPU_B.getIntStatus();
  mpuC_int_status = MPU_C.getIntStatus();
  mpuD_int_status = MPU_D.getIntStatus();

  /* Check FIFO count */
  mpuA_fifo_count = MPU_A.getFIFOCount();
  mpuB_fifo_count = MPU_B.getFIFOCount();
  mpuC_fifo_count = MPU_C.getFIFOCount();
  mpuD_fifo_count = MPU_D.getFIFOCount();

  /* MPU A read data */

  // FIFO overflow
  if ((mpuA_int_status & 0x10) || mpuA_fifo_count == 1024)
  {
    MPU_A.resetFIFO();
  }
  // New data
  else if (mpuA_int_status & 0x02)
  {
    //Wait for correct available data length, should be a very short wait
    while (mpuA_fifo_count < mpuA_packet_size) {
      mpuA_fifo_count = MPU_A.getFIFOCount();
    }

    MPU_A.getFIFOBytes(mpuA_fifo_buffer, mpuA_packet_size);

    //Track FIFO count here in case there is > 1 packet available
    mpuA_fifo_count -= mpuA_packet_size;

    mpuA_read_data = true;
  }

  /* MPU B read data */

  // FIFO overflow
  if ((mpuB_int_status & 0x10) || mpuB_fifo_count == 1024)
  {
    MPU_B.resetFIFO();
  }
  // New data
  else if (mpuB_int_status & 0x02)
  {
    //Wait for correct available data length, should be a very short wait
    while (mpuB_fifo_count < mpuB_packet_size) {
      mpuB_fifo_count = MPU_B.getFIFOCount();
    }

    MPU_B.getFIFOBytes(mpuB_fifo_buffer, mpuB_packet_size);

    //Track FIFO count here in case there is > 1 packet available
    mpuB_fifo_count -= mpuB_packet_size;

    mpuB_read_data = true;
  }

  /* MPU C read data */
  if ((mpuC_int_status & 0x10) || mpuC_fifo_count == 1024)
  {
    MPU_C.resetFIFO();
  }
  // New data
  else if (mpuC_int_status & 0x02)
  {
    //Wait for correct available data length, should be a very short wait
    while (mpuC_fifo_count < mpuC_packet_size) {
      mpuC_fifo_count = MPU_C.getFIFOCount();
    }

    MPU_C.getFIFOBytes(mpuC_fifo_buffer, mpuC_packet_size);

    //Track FIFO count here in case there is > 1 packet available
    mpuC_fifo_count -= mpuC_packet_size;

    mpuC_read_data = true;
  }

  /* MPU D read data */
  if ((mpuD_int_status & 0x10) || mpuD_fifo_count == 1024)
  {
    MPU_D.resetFIFO();
  }
  // New data
  else if (mpuD_int_status & 0x02)
  {
    //Wait for correct available data length, should be a very short wait
    while (mpuD_fifo_count < mpuD_packet_size) {
      mpuD_fifo_count = MPU_D.getFIFOCount();
    }

    MPU_D.getFIFOBytes(mpuD_fifo_buffer, mpuD_packet_size);

    //Track FIFO count here in case there is > 1 packet available
    mpuD_fifo_count -= mpuD_packet_size;

    mpuD_read_data = true;
  }

  /* Send data to PC */
  sensor_send_data_to_pc();

  /* Reset offsets if timeout */
  reset_time = millis();

  if(reset_time - reset_prev_time >= reset_interval)
  {
    reset_prev_time = reset_time;

    //MPU_A.resetFIFO();
    
  /* Set offsets */
    //MPU_A.setXAccelOffset(mpu_a_X_accell_offset);
    //MPU_A.setYAccelOffset(mpu_a_Y_accell_offset);
    MPU_A.setZAccelOffset(mpu_a_Z_accell_offset);
    MPU_A.setXGyroOffset(mpu_a_X_gyro_offset);
    MPU_A.setYGyroOffset(mpu_a_Y_gyro_offset);
    MPU_A.setZGyroOffset(mpu_a_Z_gyro_offset);

    //MPU_B.resetFIFO();
    
    /* Set offsets */
    //MPU_B.setXAccelOffset(mpu_b_X_accell_offset);
    //MPU_B.setYAccelOffset(mpu_b_Y_accell_offset);
    MPU_B.setZAccelOffset(mpu_b_Z_accell_offset);
    MPU_B.setXGyroOffset(mpu_b_X_gyro_offset);
    MPU_B.setYGyroOffset(mpu_b_Y_gyro_offset);
    MPU_B.setZGyroOffset(mpu_b_Z_gyro_offset);

    /* Set offsets */
    //MPU_B.setXAccelOffset(mpu_b_X_accell_offset);
    //MPU_B.setYAccelOffset(mpu_b_Y_accell_offset);
    MPU_C.setZAccelOffset(mpu_c_Z_accell_offset);
    MPU_C.setXGyroOffset(mpu_c_X_gyro_offset);
    MPU_C.setYGyroOffset(mpu_c_Y_gyro_offset);
    MPU_C.setZGyroOffset(mpu_c_Z_gyro_offset);

    /* Set offsets */
    //MPU_B.setXAccelOffset(mpu_b_X_accell_offset);
    //MPU_B.setYAccelOffset(mpu_b_Y_accell_offset);
    MPU_D.setZAccelOffset(mpu_d_Z_accell_offset);
    MPU_D.setXGyroOffset(mpu_d_X_gyro_offset);
    MPU_D.setYGyroOffset(mpu_d_Y_gyro_offset);
    MPU_D.setZGyroOffset(mpu_d_Z_gyro_offset);
  }
}
