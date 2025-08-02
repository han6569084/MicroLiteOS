#include "i2c.h"

void vTaskDelay(__IO uint32_t nCount);

/* I2C句柄 */
I2C_HandleTypeDef hi2c1;

/* HAL库回调函数 */
uint32_t I2Cx_TIMEOUT_UserCallback(uint8_t errorCode)
{
    /* 超时或错误处理 */
    return errorCode;
}

void I2cMaster_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能I2C和GPIO时钟 */
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 配置I2C GPIO引脚 */
    GPIO_InitStructure.Pin = SENSORS_I2C_SCL_GPIO_PIN | SENSORS_I2C_SDA_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Alternate = SENSORS_I2C_AF;

    HAL_GPIO_Init(SENSORS_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

    /* 配置I2C */
    hi2c1.Instance = SENSORS_I2C;
    hi2c1.Init.ClockSpeed = SENSORS_I2C_SPEED;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = I2C_OWN_ADDRESS;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_Init(&hi2c1);
}

/* 简化的HAL库I2C读写函数 */
static unsigned long ST_Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(&hi2c1, Address << 1, RegisterAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)RegisterValue, RegisterLen, 1000);
    return (status == HAL_OK) ? 0 : 1;
}

static unsigned long ST_Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c1, Address << 1, RegisterAddr, I2C_MEMADD_SIZE_8BIT, RegisterValue, RegisterLen, 1000);
    return (status == HAL_OK) ? 0 : 1;
}

static unsigned long ST_Sensors_I2C_WriteNoRegister(unsigned char Address, unsigned char RegisterAddr)
{
    HAL_StatusTypeDef status;
    uint8_t data = RegisterAddr;
    status = HAL_I2C_Master_Transmit(&hi2c1, Address << 1, &data, 1, 1000);
    return (status == HAL_OK) ? 0 : 1;
}

static unsigned long ST_Sensors_I2C_ReadNoRegister(unsigned char Address, unsigned short RegisterLen, unsigned char *RegisterValue)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Receive(&hi2c1, Address << 1, RegisterValue, RegisterLen, 1000);
    return (status == HAL_OK) ? 0 : 1;
}

int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len,
                                        const unsigned char *data_ptr)
{
  int ret = 0;
  ret = ST_Sensors_I2C_WriteRegister( slave_addr, reg_addr, len, data_ptr);
  return ret;
}

int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                      unsigned char reg_addr,
                                      unsigned short len,
                                      unsigned char *data_ptr)
{
  int ret = 0;
  ret = ST_Sensors_I2C_ReadRegister( slave_addr, reg_addr, len, data_ptr);
  return ret;
}

int Sensors_I2C_WriteNoRegister(unsigned char slave_addr, unsigned char reg_addr)
{
  int ret = 0;
  ret = ST_Sensors_I2C_WriteNoRegister( slave_addr, reg_addr);
  return ret;
}

int Sensors_I2C_ReadNoRegister(unsigned char slave_addr, unsigned short len, unsigned char *data_ptr)
{
  int ret = 0;
  ret = ST_Sensors_I2C_ReadNoRegister( slave_addr, len, data_ptr);
  return ret;
}

/* 简化的延时和重试函数 */
unsigned short Get_I2C_Retry(void)
{
    return 5;
}

void Set_I2C_Retry(unsigned short ml_sec)
{
    /* 简化实现 */
}
