#include "bsp_i2c.h"

#define BSP_I2C_TIMEOUT 10000U

/* This project carries an older CMSIS CM3 header that predates Arm Compiler
 * 6.  Under AC6, its __get_PRIMASK declaration is treated as an external
 * symbol.  Keep the timing-critical I2C sequences self-contained instead. */
static uint32_t BSP_I2C_SaveAndDisableInterrupts(void)
{
    uint32_t primask;

#if defined(__GNUC__) || defined(__clang__)
    __asm volatile("mrs %0, primask" : "=r"(primask)::"memory");
    __asm volatile("cpsid i" ::: "memory");
#else
    primask = __get_PRIMASK();
    __disable_irq();
#endif
    return primask;
}

static void BSP_I2C_RestoreInterrupts(uint32_t primask)
{
#if defined(__GNUC__) || defined(__clang__)
    __asm volatile("msr primask, %0" ::"r"(primask) : "memory");
#else
    if (primask == 0U) __enable_irq();
#endif
}

static void BSP_I2C_ShortDelay(void)
{
    volatile uint32_t delay = 32U;
    while (delay-- != 0U) {}
}

static void BSP_I2C_ClearErrors(I2C_TypeDef *I2Cx)
{
    uint16_t errors = I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF |
                      I2C_SR1_OVR | I2C_SR1_PECERR | I2C_SR1_TIMEOUT |
                      I2C_SR1_SMBALERT;
    I2Cx->SR1 &= (uint16_t)~errors;
}

static uint8_t BSP_I2C_Abort(I2C_TypeDef *I2Cx)
{
    I2C_GenerateSTOP(I2Cx, ENABLE);
    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current);
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    BSP_I2C_ClearErrors(I2Cx);
    return 0U;
}

static uint8_t BSP_I2C_WaitSR1Set(I2C_TypeDef *I2Cx, uint16_t flag)
{
    uint32_t timeout = BSP_I2C_TIMEOUT;

    while ((I2Cx->SR1 & flag) == 0U) {
        if ((I2Cx->SR1 & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF |
                          I2C_SR1_OVR | I2C_SR1_TIMEOUT)) != 0U) {
            return BSP_I2C_Abort(I2Cx);
        }
        if (timeout-- == 0U) return BSP_I2C_Abort(I2Cx);
    }
    return 1U;
}

static uint8_t BSP_I2C_WaitEvent(I2C_TypeDef *I2Cx, uint32_t event)
{
    uint32_t timeout = BSP_I2C_TIMEOUT;

    while (I2C_CheckEvent(I2Cx, event) != SUCCESS) {
        if ((I2Cx->SR1 & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF |
                          I2C_SR1_OVR | I2C_SR1_TIMEOUT)) != 0U) {
            return BSP_I2C_Abort(I2Cx);
        }
        if (timeout-- == 0U) return BSP_I2C_Abort(I2Cx);
    }
    return 1U;
}

static void BSP_I2C_ClearADDR(I2C_TypeDef *I2Cx)
{
    volatile uint16_t dummy;
    dummy = I2Cx->SR1;
    dummy = I2Cx->SR2;
    (void)dummy;
}

static void BSP_I2C_GetPins(I2C_TypeDef *I2Cx, uint8_t remap,
                            uint16_t *scl_pin, uint16_t *sda_pin)
{
    if (I2Cx == I2C1 && remap != 0U) {
        *scl_pin = GPIO_Pin_8;
        *sda_pin = GPIO_Pin_9;
    } else if (I2Cx == I2C1) {
        *scl_pin = GPIO_Pin_6;
        *sda_pin = GPIO_Pin_7;
    } else {
        *scl_pin = GPIO_Pin_10;
        *sda_pin = GPIO_Pin_11;
    }
}

/* GPIO is used only before peripheral initialization to release a slave that
 * held SDA low after an interrupted transfer.  Normal traffic is hardware I2C. */
static void BSP_I2C_RecoverBus(I2C_TypeDef *I2Cx, uint8_t remap)
{
    GPIO_InitTypeDef gpio_config;
    uint16_t scl_pin;
    uint16_t sda_pin;
    uint8_t pulse;

    BSP_I2C_GetPins(I2Cx, remap, &scl_pin, &sda_pin);
    I2C_Cmd(I2Cx, DISABLE);
    gpio_config.GPIO_Pin   = scl_pin | sda_pin;
    gpio_config.GPIO_Mode  = GPIO_Mode_Out_OD;
    gpio_config.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio_config);
    GPIO_SetBits(GPIOB, scl_pin | sda_pin);
    BSP_I2C_ShortDelay();

    for (pulse = 0U; pulse < 9U && GPIO_ReadInputDataBit(GPIOB, sda_pin) == Bit_RESET; ++pulse) {
        GPIO_ResetBits(GPIOB, scl_pin);
        BSP_I2C_ShortDelay();
        GPIO_SetBits(GPIOB, scl_pin);
        BSP_I2C_ShortDelay();
    }

    GPIO_ResetBits(GPIOB, sda_pin);
    BSP_I2C_ShortDelay();
    GPIO_SetBits(GPIOB, scl_pin);
    BSP_I2C_ShortDelay();
    GPIO_SetBits(GPIOB, sda_pin);
    BSP_I2C_ShortDelay();
}

uint8_t BSP_I2C_WaitIdle(I2C_TypeDef *I2Cx)
{
    uint32_t timeout = BSP_I2C_TIMEOUT;

    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) != RESET) {
        if (timeout-- == 0U) {
            return 0U;
        }
    }
    return 1U;
}

void BSP_I2C_Init(I2C_TypeDef *I2Cx, uint8_t remap, uint32_t clock_hz)
{
    I2C_InitTypeDef i2c_config;
    GPIO_InitTypeDef gpio_config;
    uint16_t scl_pin;
    uint16_t sda_pin;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd((I2Cx == I2C1) ? RCC_APB1Periph_I2C1 : RCC_APB1Periph_I2C2,
                           ENABLE);

    if (I2Cx == I2C1 && remap != 0U) {
        GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
    } else if (I2Cx == I2C1) {
        GPIO_PinRemapConfig(GPIO_Remap_I2C1, DISABLE);
    }

    BSP_I2C_RecoverBus(I2Cx, remap);
    BSP_I2C_GetPins(I2Cx, remap, &scl_pin, &sda_pin);
    gpio_config.GPIO_Pin   = scl_pin | sda_pin;
    gpio_config.GPIO_Mode  = GPIO_Mode_AF_OD;
    gpio_config.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio_config);

    I2C_DeInit(I2Cx);
    I2C_SoftwareResetCmd(I2Cx, ENABLE);
    I2C_SoftwareResetCmd(I2Cx, DISABLE);
    i2c_config.I2C_Mode                = I2C_Mode_I2C;
    i2c_config.I2C_ClockSpeed          = clock_hz;
    i2c_config.I2C_DutyCycle           = I2C_DutyCycle_2;
    i2c_config.I2C_Ack                 = I2C_Ack_Enable;
    i2c_config.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c_config.I2C_OwnAddress1         = 0U;
    I2C_Init(I2Cx, &i2c_config);
    BSP_I2C_ClearErrors(I2Cx);
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    I2C_Cmd(I2Cx, ENABLE);
}

uint8_t BSP_I2C_MemWrite(I2C_TypeDef *I2Cx, uint8_t address7,
                         uint8_t reg, const uint8_t *data, uint16_t count)
{
    uint16_t index;

    if (data == 0 || count == 0U || !BSP_I2C_WaitIdle(I2Cx)) return 0U;
    BSP_I2C_ClearErrors(I2Cx);

    I2C_GenerateSTART(I2Cx, ENABLE);
    if (!BSP_I2C_WaitEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) return 0U;
    I2C_Send7bitAddress(I2Cx, (uint8_t)(address7 << 1), I2C_Direction_Transmitter);
    if (!BSP_I2C_WaitEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 0U;
    I2C_SendData(I2Cx, reg);
    if (!BSP_I2C_WaitEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 0U;

    for (index = 0U; index < count; ++index) {
        I2C_SendData(I2Cx, data[index]);
        if (!BSP_I2C_WaitEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 0U;
    }
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 1U;
}

uint8_t BSP_I2C_MemRead(I2C_TypeDef *I2Cx, uint8_t address7,
                        uint8_t reg, uint8_t *data, uint16_t count)
{
    uint16_t index;
    uint32_t primask;

    if (data == 0 || count == 0U || !BSP_I2C_WaitIdle(I2Cx)) return 0U;
    BSP_I2C_ClearErrors(I2Cx);
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current);

    I2C_GenerateSTART(I2Cx, ENABLE);
    if (!BSP_I2C_WaitEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) return 0U;
    I2C_Send7bitAddress(I2Cx, (uint8_t)(address7 << 1), I2C_Direction_Transmitter);
    if (!BSP_I2C_WaitEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 0U;
    I2C_SendData(I2Cx, reg);
    if (!BSP_I2C_WaitEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 0U;

    I2C_GenerateSTART(I2Cx, ENABLE);
    if (!BSP_I2C_WaitEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) return 0U;
    I2C_Send7bitAddress(I2Cx, (uint8_t)(address7 << 1), I2C_Direction_Receiver);

    /* I2C_CheckEvent would read SR1/SR2 and clear ADDR too early here. */
    if (!BSP_I2C_WaitSR1Set(I2Cx, I2C_SR1_ADDR)) return 0U;
    if (count == 1U) {
        /* RM0008: ACK, ADDR clearing and STOP must not be interrupted. */
        primask = BSP_I2C_SaveAndDisableInterrupts();
        I2C_AcknowledgeConfig(I2Cx, DISABLE);
        BSP_I2C_ClearADDR(I2Cx);
        I2C_GenerateSTOP(I2Cx, ENABLE);
        BSP_I2C_RestoreInterrupts(primask);
        if (!BSP_I2C_WaitSR1Set(I2Cx, I2C_SR1_RXNE)) return 0U;
        data[0] = I2C_ReceiveData(I2Cx);
        I2C_AcknowledgeConfig(I2Cx, ENABLE);
        return 1U;
    }

    if (count == 2U) {
        /* POS makes the NACK apply to the second byte.  Clearing ADDR and ACK
         * is one timing-critical sequence on STM32F1. */
        I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Next);
        primask = BSP_I2C_SaveAndDisableInterrupts();
        I2C_AcknowledgeConfig(I2Cx, DISABLE);
        BSP_I2C_ClearADDR(I2Cx);
        BSP_I2C_RestoreInterrupts(primask);

        if (!BSP_I2C_WaitSR1Set(I2Cx, I2C_SR1_BTF)) return 0U;
        primask = BSP_I2C_SaveAndDisableInterrupts();
        I2C_GenerateSTOP(I2Cx, ENABLE);
        data[0] = I2C_ReceiveData(I2Cx);
        BSP_I2C_RestoreInterrupts(primask);
        data[1] = I2C_ReceiveData(I2Cx);
        I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current);
        I2C_AcknowledgeConfig(I2Cx, ENABLE);
        return 1U;
    }

    BSP_I2C_ClearADDR(I2Cx);
    index = 0U;
    while (index < (uint16_t)(count - 3U)) {
        if (!BSP_I2C_WaitSR1Set(I2Cx, I2C_SR1_RXNE)) return 0U;
        data[index++] = I2C_ReceiveData(I2Cx);
    }

    /* With three bytes left, BTF guarantees two bytes are buffered.  NACK the
     * final byte before draining N-2, then issue STOP while the last byte is
     * still in the shift register (RM0008 master receiver sequence). */
    if (!BSP_I2C_WaitSR1Set(I2Cx, I2C_SR1_BTF)) return 0U;
    primask = BSP_I2C_SaveAndDisableInterrupts();
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    data[index++] = I2C_ReceiveData(I2Cx);
    BSP_I2C_RestoreInterrupts(primask);
    if (!BSP_I2C_WaitSR1Set(I2Cx, I2C_SR1_BTF)) return 0U;

    primask = BSP_I2C_SaveAndDisableInterrupts();
    I2C_GenerateSTOP(I2Cx, ENABLE);
    data[index++] = I2C_ReceiveData(I2Cx);
    BSP_I2C_RestoreInterrupts(primask);
    data[index] = I2C_ReceiveData(I2Cx);
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    return 1U;
}