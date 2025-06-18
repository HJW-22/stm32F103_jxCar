// MPU6050 寄存器地址定义  
#define MPU6050_SMPLRT_DIV      0x19    // 采样率分频寄存器  
#define MPU6050_CONFIG          0x1A    // 配置寄存器  
#define MPU6050_GYRO_CONFIG     0x1B    // 陀螺仪配置寄存器  
#define MPU6050_ACCEL_CONFIG    0x1C    // 加速度计配置寄存器  

// 中断使能寄存器  
#define MPU6050_INT_ENABLE      0x38    // 中断使能寄存器  

// 电源管理寄存器  
#define MPU6050_PWR_MGMT_1      0x6B    // 电源管理寄存器1  
#define MPU6050_PWR_MGMT_2      0x6C    // 电源管理寄存器2  

// 数据寄存器  
#define MPU6050_ACCEL_XOUT_H    0x3B    // 加速度计X轴高8位  
#define MPU6050_ACCEL_XOUT_L    0x3C    // 加速度计X轴低8位  
#define MPU6050_ACCEL_YOUT_H    0x3D    // 加速度计Y轴高8位  
#define MPU6050_ACCEL_YOUT_L    0x3E    // 加速度计Y轴低8位  
#define MPU6050_ACCEL_ZOUT_H    0x3F    // 加速度计Z轴高8位  
#define MPU6050_ACCEL_ZOUT_L    0x40    // 加速度计Z轴低8位  

#define MPU6050_TEMP_OUT_H      0x41    // 温度高8位  
#define MPU6050_TEMP_OUT_L      0x42    // 温度低8位  

#define MPU6050_GYRO_XOUT_H     0x43    // 陀螺仪X轴高8位  
#define MPU6050_GYRO_XOUT_L     0x44    // 陀螺仪X轴低8位  
#define MPU6050_GYRO_YOUT_H     0x45    // 陀螺仪Y轴高8位  
#define MPU6050_GYRO_YOUT_L     0x46    // 陀螺仪Y轴低8位  
#define MPU6050_GYRO_ZOUT_H     0x47    // 陀螺仪Z轴高8位  
#define MPU6050_GYRO_ZOUT_L     0x48    // 陀螺仪Z轴低8位  

// 配置参数宏定义  
// 电源管理寄存器配置  
#define MPU6050_PWR_MGMT_1_RESET    0x80    // 复位  
#define MPU6050_PWR_MGMT_1_SLEEP    0x40    // 睡眠模式  
#define MPU6050_PWR_MGMT_1_CLKSEL   0x00    // 内部8MHz振荡器  

// 陀螺仪配置（满量程）  
#define MPU6050_GYRO_FS_250         0x00    // ±250°/s  
#define MPU6050_GYRO_FS_500         0x08    // ±500°/s  
#define MPU6050_GYRO_FS_1000        0x10    // ±1000°/s  
#define MPU6050_GYRO_FS_2000        0x18    // ±2000°/s  

// 加速度计配置（满量程）  
#define MPU6050_ACCEL_FS_2G         0x00    // ±2g  
#define MPU6050_ACCEL_FS_4G         0x08    // ±4g  
#define MPU6050_ACCEL_FS_8G         0x10    // ±8g  
#define MPU6050_ACCEL_FS_16G        0x18    // ±16g  

#define	MPU6050_WHO_AM_I		0x75
