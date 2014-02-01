//drv_mpu6000
#include "board.h"


// MPU_INT on PB0 
#define MPU_OFF            digitalHi(GPIOA, GPIO_Pin_4);
#define MPU_ON             digitalLo(GPIOA, GPIO_Pin_4);
// Experimental DMP support
// #define MPU6000_DMP

#define DMP_MEM_START_ADDR 0x6E
#define DMP_MEM_R_W 0x6F
////////////////////////////////////////
#define MPUREG_WHOAMI               0x75
#define MPUREG_SMPLRT_DIV           0x19
#define MPUREG_CONFIG               0x1A
#define MPUREG_GYRO_CONFIG          0x1B
#define MPUREG_ACCEL_CONFIG         0x1C
#define MPUREG_I2C_MST_CTRL         0x24
#define MPUREG_I2C_SLV0_ADDR        0x25
#define MPUREG_I2C_SLV0_REG         0x26
#define MPUREG_I2C_SLV0_CTRL        0x27
#define MPUREG_I2C_SLV4_ADDR        0x31
#define MPUREG_I2C_SLV4_REG         0x32
#define MPUREG_I2C_SLV4_DO          0x33
#define MPUREG_I2C_SLV4_CTRL        0x34
#define MPUREG_I2C_SLV4_DI          0x35
#define MPUREG_I2C_MST_STATUS       0x36
#define MPUREG_INT_PIN_CFG          0x37
#define MPUREG_INT_ENABLE           0x38 
#define MPUREG_ACCEL_XOUT_H         0x3B
#define MPUREG_ACCEL_XOUT_L         0x3C
#define MPUREG_ACCEL_YOUT_H         0x3D
#define MPUREG_ACCEL_YOUT_L         0x3E
#define MPUREG_ACCEL_ZOUT_H         0x3F
#define MPUREG_ACCEL_ZOUT_L         0x40
#define MPUREG_TEMP_OUT_H           0x41
#define MPUREG_TEMP_OUT_L           0x42
#define MPUREG_GYRO_XOUT_H          0x43
#define MPUREG_GYRO_XOUT_L          0x44
#define MPUREG_GYRO_YOUT_H          0x45
#define MPUREG_GYRO_YOUT_L          0x46
#define MPUREG_GYRO_ZOUT_H          0x47
#define MPUREG_GYRO_ZOUT_L          0x48
#define MPUREG_EXT_SENS_DATA_00     0x49 // Registers 0x49 to 0x60 - External Sensor Data
#define MPUREG_I2C_SLV0_DO          0x63 // This register holds the output data written into Slave 0 when Slave 0 is set to write mode.
#define MPUREG_I2C_MST_DELAY_CTRL   0x67 // I2C Master Delay Control
#define MPUREG_USER_CTRL            0x6A
#define MPUREG_PWR_MGMT_1           0x6B
#define MPUREG_PWR_MGMT_2           0x6C


// Configuration bits MPU 6000
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_AFS_2G                 0x00
#define BITS_AFS_4G                 0x08
#define BITS_AFS_8G                 0x10
#define BITS_AFS_16G                0x18
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_I2C_SLV0_EN             0x80

/////////////////////////////////////////


#define INV_MAX_NUM_ACCEL_SAMPLES      (8)
#define DMP_REF_QUATERNION             (0)
#define DMP_REF_GYROS                  (DMP_REF_QUATERNION + 4) // 4
#define DMP_REF_CONTROL                (DMP_REF_GYROS + 3)      // 7
#define DMP_REF_RAW                    (DMP_REF_CONTROL + 4)    // 11
#define DMP_REF_RAW_EXTERNAL           (DMP_REF_RAW + 8)        // 19
#define DMP_REF_ACCEL                  (DMP_REF_RAW_EXTERNAL + 6)       // 25
#define DMP_REF_QUANT_ACCEL            (DMP_REF_ACCEL + 3)      // 28
#define DMP_REF_QUATERNION_6AXIS       (DMP_REF_QUANT_ACCEL + INV_MAX_NUM_ACCEL_SAMPLES)        // 36
#define DMP_REF_EIS                    (DMP_REF_QUATERNION_6AXIS + 4)   // 40
#define DMP_REF_DMP_PACKET             (DMP_REF_EIS + 3)        // 43
#define DMP_REF_GARBAGE                (DMP_REF_DMP_PACKET + 1) // 44
#define DMP_REF_LAST                   (DMP_REF_GARBAGE + 1)    // 45

#define MPU_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     0x07
#define MPU_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     0x09
#define MPU_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     0x0B
#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     0x14
#define MPU_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     0x16
#define MPU_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     0x18
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FF_THR           0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_DMP_INT_STATUS   0x39
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

#define MPU6000_SMPLRT_DIV      0       //8000Hz
#define MPU6000_DLPF_CFG        0   // 256Hz
//#define MPU6000_DLPF_CFG   1        // 188Hz
//#define MPU6000_DLPF_CFG   2        // 98Hz
//#define MPU6000_DLPF_CFG   3        // 42Hz

#define MPU6000ES_REV_C4        0x14
#define MPU6000ES_REV_C5        0x15
#define MPU6000ES_REV_D6        0x16
#define MPU6000ES_REV_D7        0x17
#define MPU6000ES_REV_D8        0x18
#define MPU6000_REV_C4          0x54
#define MPU6000_REV_C5          0x55
#define MPU6000_REV_D6          0x56
#define MPU6000_REV_D7          0x57
#define MPU6000_REV_D8          0x58
#define MPU6000_REV_D9          0x59

#define MPU6000_SMPLRT_DIV      0       //8000Hz

#define MPU6000_LPF_256HZ       0
#define MPU6000_LPF_188HZ       1
#define MPU6000_LPF_98HZ        2
#define MPU6000_LPF_42HZ        3
#define MPU6000_LPF_20HZ        4
#define MPU6000_LPF_10HZ        5
#define MPU6000_LPF_5HZ         6

static uint8_t mpuLowPassFilter = MPU6000_LPF_42HZ;


static void mpu6000AccInit(void);
static void mpu6000AccRead(int16_t *accData);
static void mpu6000AccAlign(int16_t * accData);
static void mpu6000GyroInit(void);
static void mpu6000GyroRead(int16_t * gyroData);
static void mpu6000GyroAlign(int16_t * gyroData);
static void SPI1_Write(uint8_t Address, uint8_t Data );


extern uint16_t acc_1G;
static uint8_t mpuAccelHalf = 0;

/*static void SPI1_Read_REV(int16_t *acc_tmp);
		{
		uint8_t tmp[6];
		uint8_t i;
	  MPU_ON;
	  spi_writeByte(MPU_RA_XA_OFFS_H | 0x80);
			for (i = 0; i < 6; i++)
    buf[i] = spi_readByte();
    MPU_OFF;
		acc_tmp[0]=tmp[0];
		acc_tmp[1]=tmp[1];
		acc_tmp[2]=tmp[2];
		acc_tmp[3]=tmp[3];
		acc_tmp[4]=tmp[4];
		acc_tmp[5]=tmp[5];
		}
*/
static void SPI1_Write(uint8_t Address, uint8_t Data )
{ 
    MPU_ON;
    spi_writeByte(Address); 
    spi_writeByte(Data);
    MPU_OFF;
    delay(1);
}


/*static uint8_t SPI1_Read(uint8_t Address)
{
    uint8_t rv=0;
    MPU_ON;
    spi_writeByte(Address | 0x80); // Address with high bit set = Read operation
    rv = spi_readByte();
    MPU_OFF;
    return rv;
}
*/


bool mpu6000Detect(sensor_t * acc, sensor_t * gyro, uint16_t lpf, uint8_t *scale)
{
   // bool ack;
    uint8_t sig;//, rev
    //uint8_t tmp[6];

    delay(35);                  // datasheet page 13 says 30ms. other stuff could have been running meanwhile. but we'll be safe
    
    //sig = SPI1_Read( MPU_RA_WHO_AM_I);
	  MPU_ON;
    spi_writeByte(MPU_RA_WHO_AM_I | 0x80); // Address with high bit set = Read operation
    sig = spi_readByte();
    MPU_OFF;
    

    // So like, MPU6xxx has a "WHO_AM_I" register, that is used to verify the identity of the device.
    // The contents of WHO_AM_I are the upper 6 bits of the MPU-60X0’s 7-bit I2C address.
    // The least significant bit of the MPU-60X0’s I2C address is determined by the value of the AD0 pin. (we know that already).
    // But here's the best part: The value of the AD0 pin is not reflected in this register.
    
		if (sig !=0x68) //0x7e)
        return false;

     /*/determine product ID and accel revision
     SPI1_Read(int16_t *acc_tmp);
    rev = ((acc_tmp[5] & 0x01) << 2) | ((acc_tmp[3] & 0x01) << 1) | (acc_tmp[1] & 0x01);
    if (rev) {
        // Congrats, these parts are better. 
        if (rev == 1) {
            mpuAccelHalf = 1;
        } else if (rev == 2) {
            mpuAccelHalf = 0;
        } else {
            failureMode(5);
        }
    } else {
       sig = SPI1_Read( MPU_RA_PRODUCT_ID, 1, &sig);
        rev = sig & 0x0F;
        if (!rev) {
            failureMode(5);
        } else if (rev == 4) {
            mpuAccelHalf = 1;
        } else {
            mpuAccelHalf = 0;
        }
    
      }*/
			


    acc->init = mpu6000AccInit;
    acc->read = mpu6000AccRead;
    acc->align = mpu6000AccAlign;
    gyro->init = mpu6000GyroInit;
    gyro->read = mpu6000GyroRead;
    gyro->align = mpu6000GyroAlign;

    gyro->scale = (((32767.0f / 16.4f) * M_PI) / ((32767.0f / 4.0f) * 180.0f * 1000000.0f));

    // give halfacc (old revision) back to system
    if (scale)
        *scale = mpuAccelHalf;

    // default lpf is 42Hz
    switch (lpf) {
        case 256:
            mpuLowPassFilter = MPU6000_LPF_256HZ;
            break;
        case 188:
            mpuLowPassFilter = MPU6000_LPF_188HZ;
            break;
        case 98:
            mpuLowPassFilter = MPU6000_LPF_98HZ;
            break;
        default:
        case 42:
            mpuLowPassFilter = MPU6000_LPF_42HZ;
            break;
        case 20:
            mpuLowPassFilter = MPU6000_LPF_20HZ;
            break;
        case 10:
            mpuLowPassFilter = MPU6000_LPF_10HZ;
            break;
        case 5:
            mpuLowPassFilter = MPU6000_LPF_5HZ;
            break;
    }


    return true;
}

static void mpu6000AccInit(void)
{
   // if (mpuAccelHalf)
   //     acc_1G = 255;
   // else
        acc_1G = 512;
}

static void mpu6000AccRead(int16_t *accData)
{
    //uint8_t buf[6];
    //SPI1_Read_Buf( MPU_RA_ACCEL_XOUT_H, 6, buf);
	  uint8_t buf[6];
    uint8_t i;
	  MPU_ON;
	  spi_writeByte(MPU_RA_ACCEL_XOUT_H | 0x80);
			for (i = 0; i < 6; i++)
    buf[i] = spi_readByte();
    MPU_OFF;
    accData[0] = (int16_t)((buf[0] << 8) | buf[1]) / 8;
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]) / 8;
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]) / 8;
}

static void mpu6000AccAlign(int16_t *accData)
{
  /*  int16_t temp[2];
    temp[0] = accData[0];
    temp[1] = accData[1];

    // official direction is RPY
    accData[0] = temp[1];
    accData[1] = -temp[0];
    accData[2] = accData[2];*/
	accData[0] = -accData[0];
    accData[1] = -accData[1];
    accData[2] = accData[2];
}

static void mpu6000GyroInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // PB13 - MPU_INT output on rev4 hardware
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);


  /*  SPI1_Write(MPU_RA_PWR_MGMT_1, 0x80);      //PWR_MGMT_1    -- DEVICE_RESET 1
    delay(5);
    SPI1_Write( MPU_RA_SMPLRT_DIV, 0x00);      //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    SPI1_Write( MPU_RA_PWR_MGMT_1, 0x03);      //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    SPI1_Write( MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
    SPI1_Write( MPU_RA_CONFIG, MPU6000_DLPF_CFG);  //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    SPI1_Write( MPU_RA_GYRO_CONFIG, 0x18);      //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
*/
	 SPI1_Write(MPUREG_PWR_MGMT_1, BIT_H_RESET);
    delay(100);
    SPI1_Write(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);      // Set PLL source to gyro output
   // SPI1_Write(MPUREG_USER_CTRL, 0b00110000);                 // I2C_MST_EN
    SPI1_Write(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);             // Disable I2C bus
    SPI1_Write(MPUREG_SMPLRT_DIV, 0x04);                      // Sample rate = 200Hz    Fsample = 1Khz / (4 + 1) = 200Hz   
    SPI1_Write(MPUREG_CONFIG, 0); // BITS_DLPF_CFG_42HZ);            // Fs & DLPF Fs = 1kHz, DLPF = 42Hz (low pass filter)
    SPI1_Write(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);          // Gyro scale 2000?/s
    SPI1_Write(MPUREG_ACCEL_CONFIG, BITS_AFS_8G);             // Accel scale 4G
    SPI1_Write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);            // INT: Raw data ready
    SPI1_Write(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR);     // INT: Clear on any read
    // ACC Init stuff. Moved into gyro init because the reset above would screw up accel config. Oops.
    // Accel scale 8g (4096 LSB/g)
		SPI1_Write( MPU_RA_CONFIG, mpuLowPassFilter);
    SPI1_Write( MPU_RA_ACCEL_CONFIG, 2 << 3);

}

static void mpu6000GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
    uint8_t i;
	  MPU_ON;
	  spi_writeByte(MPU_RA_GYRO_XOUT_H | 0x80);
    //SPI1_Read_Buf(MPU_RA_GYRO_XOUT_H, 6, buf);
	  for (i = 0; i < 6; i++)
    buf[i] = spi_readByte();
    MPU_OFF;

    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]) / 4;
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]) / 4;
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]) / 4;

}

static void mpu6000GyroAlign(int16_t *gyroData)
{
	int16_t temp[2];
    temp[0] = gyroData[0];
    temp[1] = gyroData[1];

    // official direction is RPY
    gyroData[0] = temp[1];
    gyroData[1] = -temp[0];
   
    // official direction is RPY
    /*gyroData[0] = gyroData[0];
    gyroData[1] = gyroData[1];*/
    gyroData[2] = -gyroData[2];
}






