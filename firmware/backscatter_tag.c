/*
 * backscatter_tag.c
 * From-scratch standalone CC2652R1 PoC firmware for LAUNCHXL-CC26X2R1.
 *
 * Hardware pinout (direct single-pin DIO21 control — NO external AND gate):
 *   - DIO21 (BoosterPack pin 8): Direct RF-switch control via SN74LVC1T45 -> ADG902 J6 CTRL:
 *       * OPEN   (0): static LOW  (0.00 V -> ADG902 J6 CTRL = 0.00 V, Open/Reflective)
 *       * LOAD50 (1): static HIGH (3.30 V -> ADG902 J6 CTRL = 2.50 V, 50 Ohm J2 ON)
 *       * CW200K (2): continuous 200.000 kHz 50% PWM (0 / 2.50 V at J6 CTRL)
 *       * PKT    (3): software-gated 200.000 kHz PWM modulated by 1.000 kbps 88-bit frames
 *   - DIO24 (BoosterPack pin 6): Optional baseband monitor mirror (1.000 kbps data or static level)
 *   - DIO13 (BTN-1, Left): Switch RF load cycle:
 *       LOAD50 (2.50 V DC, Red ON / Green OFF)
 *       -> CW200K (200 kHz toggle, Red ON / Green ON)
 *       -> OPEN (0.00 V DC, Red OFF / Green OFF)
 *   - DIO14 (BTN-2, Right): Start/increment 1.0 kbps backscattering packet counter ("C0".."C9")
 *   - DIO6  (Red LED) / DIO7 (Green LED): Mode & transmission visual status
 *   - DIO2/3 (XDS110 UART2 @ 115200 8N1): Telemetry & optional PC command control
 *
 * Frame layout (88-bit cycle @ 1.000 kbps = 88.0 ms/packet, ~11.36 pkts/s):
 *   [Preamble 8b: 0xAA]
 * + [Sync Word 16b: 0xD391]
 * + [Magic 8b: 0xA5]
 * + [Seq 8b]
 * + [BTN2_Count 8b]
 * + [Payload 16b: big-endian, default 0x4F4B ("OK")]
 * + [CRC-8-CCITT 8b: poly=0x07, init=0x00 over 5 bytes (Magic..PayLo)]
 * + [Guard Gap 16b: 0x0000]
 */

#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/cpu.h)
#include DeviceFamily_constructPath(driverlib/gpio.h)
#include DeviceFamily_constructPath(driverlib/ioc.h)
#include DeviceFamily_constructPath(driverlib/osc.h)
#include DeviceFamily_constructPath(driverlib/timer.h)
#include DeviceFamily_constructPath(inc/hw_ints.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)

#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/power/PowerCC26X2.h>

#include "ti_drivers_config.h"

#ifndef INITIAL_MODE
#define INITIAL_MODE 3
#endif

#define SUBCARRIER_FREQ_HZ      200000U
#define BIT_TIMER_LOAD_COUNTS   (48000U - 1U)   /* 48.000 MHz / 48000 = 1.000 kHz */
#define DEFAULT_PAYLOAD_U16     0x4F4BU         /* "OK" */
#define FRAME_MAGIC_BYTE        0xA5U
#define FRAME_SYNC_HI           0xD3U
#define FRAME_SYNC_LO           0x91U
#define FRAME_PREAMBLE_BYTE     0xAAU
#define FRAME_TOTAL_BYTES       11U             /* 9 active bytes + 2 zero guard bytes */
#define FRAME_TOTAL_BITS        (FRAME_TOTAL_BYTES * 8U) /* 88 bits */
#define BTN_DEBOUNCE_MS         50U
#define RED_PULSE_MS            120U
#define TELEMETRY_PERIOD_MS     1000U

#define TAG_PIN_LOW             0U
#define TAG_PIN_HIGH            1U
#define TAG_PIN_PWM200K         2U

typedef enum {
    MODE_0_LOAD_OPEN = 0,
    MODE_1_LOAD_50OHM = 1,
    MODE_2_CW_SUBCARRIER = 2,
    MODE_3_PACKETS_1KBPS = 3
} TagMode_t;

typedef struct {
    uint_least8_t pin;
    bool lastRawPressed;
    bool stablePressed;
    uint32_t lastChangeMs;
} ButtonState_t;

static PWM_Handle gPwmHandle = NULL;
static UART2_Handle gUartHandle = NULL;
static HwiP_Struct gTimerHwi;

static volatile TagMode_t gMode = (TagMode_t)INITIAL_MODE;
static volatile uint32_t gTickMs = 0U;
static volatile uint16_t gNextPayload = DEFAULT_PAYLOAD_U16;
static volatile uint8_t gSeqCounter = 0U;
static volatile uint8_t gLastSentSeq = 0U;
static volatile uint8_t gBtn1Count = 0U;
static volatile uint8_t gBtn2Count = 0U;
static volatile uint32_t gPktTxCount = 0U;

/* ISR-owned frame buffer */
static uint8_t gIsrFrame[FRAME_TOTAL_BYTES];
static uint8_t gIsrBitIdx = 0U;

static uint32_t gRedPulseUntilMs = 0U;
static bool gForceTelemetry = false;

static uint8_t crc8Ccitt(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00U;
    for (size_t i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0U; b < 8U; ++b) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ 0x07U);
            } else {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }
    return crc;
}

static const char *modeToStr(TagMode_t mode)
{
    switch (mode) {
        case MODE_0_LOAD_OPEN:
            return "OPEN";
        case MODE_1_LOAD_50OHM:
            return "LOAD50";
        case MODE_2_CW_SUBCARRIER:
            return "CW200K";
        case MODE_3_PACKETS_1KBPS:
        default:
            return "PKT";
    }
}

static void buildNextIsrFrame(void)
{
    uint8_t seq = gSeqCounter++;
    uint8_t btn2 = gBtn2Count;
    uint16_t pay = gNextPayload;

    gIsrFrame[0] = FRAME_PREAMBLE_BYTE;
    gIsrFrame[1] = FRAME_SYNC_HI;
    gIsrFrame[2] = FRAME_SYNC_LO;
    gIsrFrame[3] = FRAME_MAGIC_BYTE;
    gIsrFrame[4] = seq;
    gIsrFrame[5] = btn2;
    gIsrFrame[6] = (uint8_t)(pay >> 8U);
    gIsrFrame[7] = (uint8_t)(pay & 0xFFU);
    gIsrFrame[8] = crc8Ccitt(&gIsrFrame[3], 5U);
    gIsrFrame[9] = 0x00U;
    gIsrFrame[10] = 0x00U;

    gLastSentSeq = seq;
    gPktTxCount++;
}

static inline void driveTagPin(uint8_t state)
{
    if (state == TAG_PIN_PWM200K) {
        IOCIOPortIdSet(CONFIG_GPIO_PWM_0, IOC_PORT_MCU_PORT_EVENT0);
    } else if (state == TAG_PIN_HIGH) {
        GPIO_writeDio(CONFIG_GPIO_PWM_0, 1U);
        IOCIOPortIdSet(CONFIG_GPIO_PWM_0, IOC_PORT_GPIO);
    } else {
        GPIO_writeDio(CONFIG_GPIO_PWM_0, 0U);
        IOCIOPortIdSet(CONFIG_GPIO_PWM_0, IOC_PORT_GPIO);
    }
}

static void bitTimerIsr(uintptr_t arg)
{
    (void)arg;
    TimerIntClear(GPT1_BASE, TIMER_TIMA_TIMEOUT);
    gTickMs++;

    TagMode_t mode = gMode;
    uint32_t outBit = 0U;

    if (mode == MODE_0_LOAD_OPEN) {
        outBit = 0U;
        gIsrBitIdx = 0U;
        driveTagPin(TAG_PIN_LOW);
    } else if (mode == MODE_1_LOAD_50OHM) {
        outBit = 1U;
        gIsrBitIdx = 0U;
        driveTagPin(TAG_PIN_HIGH);
    } else if (mode == MODE_2_CW_SUBCARRIER) {
        outBit = 1U;
        gIsrBitIdx = 0U;
        driveTagPin(TAG_PIN_PWM200K);
    } else {
        /* MODE_3_PACKETS_1KBPS */
        if (gIsrBitIdx == 0U) {
            buildNextIsrFrame();
        }
        uint8_t byteIdx = (uint8_t)(gIsrBitIdx >> 3U);
        uint8_t bitShift = (uint8_t)(7U - (gIsrBitIdx & 0x07U));
        outBit = (uint32_t)((gIsrFrame[byteIdx] >> bitShift) & 0x01U);

        driveTagPin((outBit != 0U) ? TAG_PIN_PWM200K : TAG_PIN_LOW);

        gIsrBitIdx++;
        if (gIsrBitIdx >= FRAME_TOTAL_BITS) {
            gIsrBitIdx = 0U;
        }
    }

    GPIO_writeDio(CONFIG_GPIO_DATA, outBit);
}

static void applyMode(TagMode_t newMode)
{
    uintptr_t key = HwiP_disable();
    gMode = newMode;
    gIsrBitIdx = 0U;
    if (newMode == MODE_0_LOAD_OPEN) {
        driveTagPin(TAG_PIN_LOW);
        GPIO_writeDio(CONFIG_GPIO_DATA, 0U);
    } else if (newMode == MODE_1_LOAD_50OHM) {
        driveTagPin(TAG_PIN_HIGH);
        GPIO_writeDio(CONFIG_GPIO_DATA, 1U);
    } else if (newMode == MODE_2_CW_SUBCARRIER) {
        driveTagPin(TAG_PIN_PWM200K);
        GPIO_writeDio(CONFIG_GPIO_DATA, 1U);
    }
    HwiP_restore(key);

    gForceTelemetry = true;
}

static void triggerBtn1LoadSwitch(void)
{
    gBtn1Count++;
    TagMode_t cur = gMode;
    if ((cur == MODE_3_PACKETS_1KBPS) || (cur == MODE_0_LOAD_OPEN)) {
        applyMode(MODE_1_LOAD_50OHM);
    } else if (cur == MODE_1_LOAD_50OHM) {
        applyMode(MODE_2_CW_SUBCARRIER);
    } else {
        applyMode(MODE_0_LOAD_OPEN);
    }
}

static void triggerBtn2StartCount(uint32_t nowMs)
{
    gBtn2Count++;
    uint8_t digit = (uint8_t)(gBtn2Count % 10U);
    gNextPayload = (uint16_t)(((uint16_t)'C' << 8U) | (uint16_t)('0' + digit));
    gRedPulseUntilMs = nowMs + RED_PULSE_MS;

    if (gMode != MODE_3_PACKETS_1KBPS) {
        uintptr_t key = HwiP_disable();
        gSeqCounter = 0U;
        gLastSentSeq = 0U;
        gPktTxCount = 0U;
        HwiP_restore(key);
        applyMode(MODE_3_PACKETS_1KBPS);
    } else {
        gForceTelemetry = true;
    }
}

static void updateLeds(uint32_t nowMs)
{
    TagMode_t mode = gMode;
    uint_least8_t red = CONFIG_GPIO_LED_OFF;
    uint_least8_t green = CONFIG_GPIO_LED_OFF;

    if (mode == MODE_1_LOAD_50OHM) {
        red = CONFIG_GPIO_LED_ON;
    } else if (mode == MODE_2_CW_SUBCARRIER) {
        red = CONFIG_GPIO_LED_ON;
        green = CONFIG_GPIO_LED_ON;
    } else if (mode == MODE_3_PACKETS_1KBPS) {
        if (((gPktTxCount >> 2U) & 1U) != 0U) {
            green = CONFIG_GPIO_LED_ON;
        }
    }

    if ((int32_t)(gRedPulseUntilMs - nowMs) > 0) {
        red = CONFIG_GPIO_LED_ON;
    }

    GPIO_write(CONFIG_GPIO_LED_RED, red);
    GPIO_write(CONFIG_GPIO_LED_GREEN, green);
}

static bool pollButtonEdge(ButtonState_t *btn, uint32_t nowMs)
{
    bool rawPressed = (GPIO_read(btn->pin) == 0U);
    if (rawPressed != btn->lastRawPressed) {
        btn->lastRawPressed = rawPressed;
        btn->lastChangeMs = nowMs;
    }

    if ((nowMs - btn->lastChangeMs) >= BTN_DEBOUNCE_MS) {
        if (rawPressed != btn->stablePressed) {
            btn->stablePressed = rawPressed;
            if (rawPressed) {
                return true;
            }
        }
    }
    return false;
}

static void uartSendRaw(const char *str)
{
    if ((gUartHandle == NULL) || (str == NULL)) {
        return;
    }
    size_t len = strlen(str);
    if (len == 0U) {
        return;
    }
    size_t written = 0U;
    (void)UART2_write(gUartHandle, str, len, &written);
}

static void emitTelemetry(void)
{
    static char txBuf[128];
    TagMode_t mode = gMode;
    uint8_t seq = gLastSentSeq;
    uint8_t b1 = gBtn1Count;
    uint8_t b2 = gBtn2Count;
    uint16_t pay = gNextPayload;
    uint32_t txCnt = gPktTxCount;

    (void)snprintf(
        txBuf,
        sizeof(txBuf),
        "POC_MCU|mode=%s|seq=%u|btn1=%u|btn2=%u|pay=%04X|tx=%lu\r\n",
        modeToStr(mode),
        (unsigned int)seq,
        (unsigned int)b1,
        (unsigned int)b2,
        (unsigned int)pay,
        (unsigned long)txCnt
    );
    uartSendRaw(txBuf);
}

static bool parseHex4(const char *s, uint16_t *outVal)
{
    if (strlen(s) < 4U) {
        return false;
    }
    uint16_t val = 0U;
    for (size_t i = 0U; i < 4U; ++i) {
        char c = (char)toupper((unsigned char)s[i]);
        uint16_t nib;
        if ((c >= '0') && (c <= '9')) {
            nib = (uint16_t)(c - '0');
        } else if ((c >= 'A') && (c <= 'F')) {
            nib = (uint16_t)(10 + (c - 'A'));
        } else {
            return false;
        }
        val = (uint16_t)((val << 4U) | nib);
    }
    *outVal = val;
    return true;
}

static void handleUartCommand(const char *line)
{
    char upper[64];
    size_t i = 0U;
    while ((i < (sizeof(upper) - 1U)) && (line[i] != '\0')) {
        upper[i] = (char)toupper((unsigned char)line[i]);
        ++i;
    }
    upper[i] = '\0';

    if ((strcmp(upper, "MODE:OPEN") == 0) || (strcmp(upper, "MODE:OFF") == 0)) {
        applyMode(MODE_0_LOAD_OPEN);
    } else if ((strcmp(upper, "MODE:LOAD50") == 0) || (strcmp(upper, "MODE:LOAD") == 0)) {
        applyMode(MODE_1_LOAD_50OHM);
    } else if ((strcmp(upper, "MODE:CW") == 0) || (strcmp(upper, "MODE:CW200K") == 0)) {
        applyMode(MODE_2_CW_SUBCARRIER);
    } else if ((strcmp(upper, "MODE:PKT") == 0) || (strcmp(upper, "MODE:COUNT") == 0)) {
        applyMode(MODE_3_PACKETS_1KBPS);
    } else if (strcmp(upper, "BTN1") == 0) {
        triggerBtn1LoadSwitch();
    } else if (strcmp(upper, "BTN2") == 0) {
        triggerBtn2StartCount(gTickMs);
    } else if (strncmp(upper, "PAY:", 4U) == 0) {
        uint16_t parsed = 0U;
        if (parseHex4(&upper[4], &parsed)) {
            gNextPayload = parsed;
            gRedPulseUntilMs = gTickMs + RED_PULSE_MS;
            gForceTelemetry = true;
        }
    } else if (strncmp(upper, "ASCII:", 6U) == 0) {
        /* Preserve original case from line for ASCII payload */
        uint8_t hi = (line[6] != '\0') ? (uint8_t)line[6] : (uint8_t)' ';
        uint8_t lo = ((line[6] != '\0') && (line[7] != '\0')) ? (uint8_t)line[7] : (uint8_t)' ';
        gNextPayload = (uint16_t)(((uint16_t)hi << 8U) | (uint16_t)lo);
        gRedPulseUntilMs = gTickMs + RED_PULSE_MS;
        gForceTelemetry = true;
    } else if (strcmp(upper, "STATUS?") == 0) {
        gForceTelemetry = true;
    }
}

static void pollUartCommands(void)
{
    static char cmdBuf[64];
    static size_t cmdLen = 0U;

    if (gUartHandle == NULL) {
        return;
    }

    for (uint8_t iter = 0U; iter < 32U; ++iter) {
        uint8_t ch = 0U;
        size_t bytesRead = 0U;
        int_fast16_t status = UART2_read(gUartHandle, &ch, 1U, &bytesRead);
        if ((status != UART2_STATUS_SUCCESS) || (bytesRead == 0U)) {
            break;
        }

        if ((ch == '\r') || (ch == '\n')) {
            if (cmdLen > 0U) {
                cmdBuf[cmdLen] = '\0';
                handleUartCommand(cmdBuf);
                cmdLen = 0U;
            }
        } else if (cmdLen < (sizeof(cmdBuf) - 1U)) {
            cmdBuf[cmdLen++] = (char)ch;
        }
    }
}

static void lockHighFreqCrystal(void)
{
    Power_setDependency(PowerCC26XX_XOSC_HF);
    Power_setConstraint(PowerCC26XX_DISALLOW_STANDBY);
    Power_setConstraint(PowerCC26XX_DISALLOW_IDLE);
    Power_setDependency(PowerCC26XX_PERIPH_GPT1);

    for (uint32_t waitCount = 0U; waitCount < 500000U; ++waitCount) {
        if (OSCClockSourceGet(OSC_SRC_CLK_HF) == OSC_XOSC_HF) {
            break;
        }
        CPUdelay(100U);
    }
}

static void initSubcarrierPwm(void)
{
    PWM_Params pwmParams;
    PWM_init();
    PWM_Params_init(&pwmParams);
    pwmParams.idleLevel = PWM_IDLE_LOW;
    pwmParams.periodUnits = PWM_PERIOD_HZ;
    pwmParams.periodValue = SUBCARRIER_FREQ_HZ;
    pwmParams.dutyUnits = PWM_DUTY_FRACTION;
    pwmParams.dutyValue = PWM_DUTY_FRACTION_MAX / 2U;

    gPwmHandle = PWM_open(CONFIG_PWM_SUBCARRIER, &pwmParams);
    if (gPwmHandle != NULL) {
        PWM_start(gPwmHandle);
    }
    GPIO_setOutputEnableDio(CONFIG_GPIO_PWM_0, GPIO_OUTPUT_ENABLE);
}

static void initUart(void)
{
    UART2_Params uartParams;
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200U;
    uartParams.readMode = UART2_Mode_NONBLOCKING;
    uartParams.writeMode = UART2_Mode_NONBLOCKING;
    uartParams.readReturnMode = UART2_ReadReturnMode_PARTIAL;

    gUartHandle = UART2_open(CONFIG_UART2_0, &uartParams);
    if (gUartHandle != NULL) {
        UART2_rxEnable(gUartHandle);
    }
}

static void initBitTimer(void)
{
    HwiP_Params hwiParams;

    TimerDisable(GPT1_BASE, TIMER_BOTH);
    TimerConfigure(GPT1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(GPT1_BASE, TIMER_A, BIT_TIMER_LOAD_COUNTS);

    HwiP_Params_init(&hwiParams);
    hwiParams.priority = 0x20U;
    HwiP_construct(&gTimerHwi, INT_GPT1A, bitTimerIsr, &hwiParams);

    TimerIntClear(GPT1_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(GPT1_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(GPT1_BASE, TIMER_A);
}

void *mainThread(void *arg0)
{
    (void)arg0;

    GPIO_init();
    GPIO_write(CONFIG_GPIO_DATA, 0U);
    GPIO_write(CONFIG_GPIO_LED_RED, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_GREEN, CONFIG_GPIO_LED_OFF);

    lockHighFreqCrystal();
    initSubcarrierPwm();
    initUart();
    initBitTimer();

    ButtonState_t btn1 = { CONFIG_GPIO_BTN1, false, false, 0U };
    ButtonState_t btn2 = { CONFIG_GPIO_BTN2, false, false, 0U };

    applyMode((TagMode_t)INITIAL_MODE);
    uartSendRaw("POC_MCU_BOOT|fw=backscatter_direct_dio21_1kbps|subcarrier=200kHz|frame=88b\r\n");

    uint32_t lastTelemetryMs = gTickMs;

    while (1) {
        uint32_t nowMs = gTickMs;

        pollUartCommands();

        if (pollButtonEdge(&btn1, nowMs)) {
            triggerBtn1LoadSwitch();
        }

        if (pollButtonEdge(&btn2, nowMs)) {
            triggerBtn2StartCount(nowMs);
        }

        updateLeds(nowMs);

        if (gForceTelemetry || ((nowMs - lastTelemetryMs) >= TELEMETRY_PERIOD_MS)) {
            gForceTelemetry = false;
            lastTelemetryMs = nowMs;
            emitTelemetry();
        }

        CPUdelay(2000U);
    }
}
