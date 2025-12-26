/**
  ******************************************************************************
  * @file    usbpd_bsp_pwr.c
  * @author  MCD Application Team
  * @brief   This file contains pwr control functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbpd_devices_conf.h"
#include "usbpd_bsp_pwr.h"
#ifdef _RTOS
#include "cmsis_os.h"
#endif /* _RTOS */

/* Private typedef -----------------------------------------------------------*/
#define USBPD_BSP_PWR_C

/* Definitions of HW boards specific connections for USBPD */
#if defined(USE_UNIBOARD)
/* On Uniboard QFP100 used for G474 and G431 validation, following setup has been used :
      MB1377 VBUS_ADC (VSENSE)      <=> Uniboard Pin 20  => PA0
      MB1377 VBUS_SRC (SOURCE_EN)   <=> Uniboard Pin 21  => PA1
      MB1377 VBUS_DSCHG (DISCHARGE) <=> Uniboard Pin 22  => PA2  */
#define VSENSE_GPIO_PORT                GPIOA
#define VSENSE_GPIO_PIN                 LL_GPIO_PIN_0
#define VSENSE_GPIO_ENABLE_CLOCK()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

#define SOURCE_EN_GPIO_PORT             GPIOA
#define SOURCE_EN_GPIO_PIN              LL_GPIO_PIN_1
#define SOURCE_EN_GPIO_ENABLE_CLOCK()   LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
#define SOURCE_EN_SET_OFF()             LL_GPIO_SetOutputPin(SOURCE_EN_GPIO_PORT, SOURCE_EN_GPIO_PIN);
#define SOURCE_EN_SET_ON()              LL_GPIO_ResetOutputPin(SOURCE_EN_GPIO_PORT, SOURCE_EN_GPIO_PIN);
#define IS_SOURCE_EN_SET_ON()           (LL_GPIO_IsOutputPinSet(SOURCE_EN_GPIO_PORT, SOURCE_EN_GPIO_PIN) == 0)

#define DISCHARGE_GPIO_PORT             GPIOA
#define DISCHARGE_GPIO_PIN              LL_GPIO_PIN_2
#define DISCHARGE_GPIO_ENABLE_CLOCK()   LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
#define DISCHARGE_SET_OFF()             LL_GPIO_ResetOutputPin(DISCHARGE_GPIO_PORT, DISCHARGE_GPIO_PIN);
#define DISCHARGE_SET_ON()              LL_GPIO_SetOutputPin(DISCHARGE_GPIO_PORT, DISCHARGE_GPIO_PIN);

/* PA0 is used as ADC12_IN1 input for ADC measurement of VBUS voltage :
   ADC1 (Common 12) Channel 1.
*/
#define VSENSE_ADC_INSTANCE             ADC1
#define VSENSE_ADC_COMMON               ADC12_COMMON
#define VSENSE_ADC_RANK                 LL_ADC_REG_RANK_1
#define VSENSE_ADC_CHANNEL              LL_ADC_CHANNEL_1

/* Enable ADC clock (core clock) */
#define VSENSE_ADC_ENABLE_CLOCK()    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

#elif defined(MB1397)
/* On EVAL STM32G474E_EVAL MB1397 REVA used for G474 validation, following setup has been used :
      VSENSE      => PC0
      SOURCE_EN   => PC11
      DISCHARGE   => PB2  */
#define VSENSE_GPIO_PORT                GPIOC
#define VSENSE_GPIO_PIN                 LL_GPIO_PIN_0
#define VSENSE_GPIO_ENABLE_CLOCK()      LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

#define SOURCE_EN_GPIO_PORT             GPIOC
#define SOURCE_EN_GPIO_PIN              LL_GPIO_PIN_11
#define SOURCE_EN_GPIO_ENABLE_CLOCK()   LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
#define SOURCE_EN_SET_OFF()             LL_GPIO_ResetOutputPin(SOURCE_EN_GPIO_PORT, SOURCE_EN_GPIO_PIN);
#define SOURCE_EN_SET_ON()              LL_GPIO_SetOutputPin(SOURCE_EN_GPIO_PORT, SOURCE_EN_GPIO_PIN);
#define IS_SOURCE_EN_SET_ON()           (LL_GPIO_IsOutputPinSet(SOURCE_EN_GPIO_PORT, SOURCE_EN_GPIO_PIN) == 1)

#define DISCHARGE_GPIO_PORT             GPIOB
#define DISCHARGE_GPIO_PIN              LL_GPIO_PIN_2
#define DISCHARGE_GPIO_ENABLE_CLOCK()   LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
#define DISCHARGE_SET_OFF()             LL_GPIO_ResetOutputPin(DISCHARGE_GPIO_PORT, DISCHARGE_GPIO_PIN);
#define DISCHARGE_SET_ON()              LL_GPIO_SetOutputPin(DISCHARGE_GPIO_PORT, DISCHARGE_GPIO_PIN);

/* PC0 is used as ADC12_IN6 input for ADC measurement of VBUS voltage :
   ADC1 (Common 12) Channel 1.
*/
#define VSENSE_ADC_INSTANCE             ADC1
#define VSENSE_ADC_COMMON               ADC12_COMMON
#define VSENSE_ADC_RANK                 LL_ADC_REG_RANK_1
#define VSENSE_ADC_CHANNEL              LL_ADC_CHANNEL_6

/* Enable ADC clock (core clock) */
#define VSENSE_ADC_ENABLE_CLOCK()    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

#else
#error "Please define configuration of HW setup for BSP PWR primitives"
#endif


/* Definitions of ADC hardware constraints delays */
/* Note: Only ADC IP HW delays are defined in ADC LL driver driver,           */
/*       not timeout values:                                                  */
/*       Timeout values for ADC operations are dependent to device clock      */
/*       configuration (system clock versus ADC clock),                       */
/*       and therefore must be defined in user application.                   */
/*       Refer to @ref ADC_LL_EC_HW_DELAYS for description of ADC timeout     */
/*       values definition.                                                   */

/* Timeout values for ADC operations. */
/* (calibration, enable settling time, disable settling time, ...)          */
/* Values defined to be higher than worst cases: low clock frequency,       */
/* maximum prescalers.                                                      */
/* Unit: ms                                                                 */
#define ADC_CHANNEL_CONF_RDY_TIMEOUT_MS  (   1U)
#define ADC_CALIBRATION_TIMEOUT_MS       (   1U)
#define ADC_ENABLE_TIMEOUT_MS            (   1U)
#define ADC_DISABLE_TIMEOUT_MS           (   1U)
#define ADC_STOP_CONVERSION_TIMEOUT_MS   (   1U)
#define ADC_CONVERSION_TIMEOUT_MS        (4000U)

/* Delay between ADC end of calibration and ADC enable.                     */
/* Delay estimation in CPU cycles: Case of ADC enable done                  */
/* immediately after ADC calibration, ADC clock setting slow                */
/* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
/* (CPU clock / ADC clock) is above 32.                                     */

#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)


#define VDDA_APPLI            3040U

#define VBUS_VOLTAGE_5V_IN_MV   5000u
#define VBUS_VOLTAGE_9V_IN_MV   9000u
#define VBUS_VOLTAGE_15V_IN_MV  15000u
#define VBUS_VOLTAGE_18V_IN_MV  18000u

/* Private Macro -----------------------------------------------------------*/
#define ABS_(_VALUE) ((_VALUE) < 0 ? -(_VALUE) : (_VALUE))

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void Configure_ADC(void);
static void Activate_ADC(void);
#if 0
static void MeasureVrefAnalog(void);
#endif


/**
  * @brief USB PD PPS values
  */
typedef enum
{
  VBUS_5_V         = 0,
  VBUS_9_V         = 1,
  VBUS_15_V        = 2,
  VBUS_18_V        = 3,
  VBUS_PPS_V       = 4,
  VBUS_LEVEL_NB    = 5
} VBUS_Level_TypeDef;

/**
  * @brief  Initialise VBUS power
  * @param  PortCount current port number
  * @retval None
  */
void BSP_PWR_VBUSInit(uint8_t PortCount)
{
  /* For Sink cases */
  Configure_ADC();
  Activate_ADC();
  /*  MeasureVrefAnalog(); */
  LL_ADC_REG_StartConversion(VSENSE_ADC_INSTANCE);

  /* Enable the peripheral clock of GPIO Port for SOURCE_EN */
  SOURCE_EN_GPIO_ENABLE_CLOCK();

  /* For SRC cases VBUS On/OFF */
  LL_GPIO_SetPinMode(SOURCE_EN_GPIO_PORT, SOURCE_EN_GPIO_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(SOURCE_EN_GPIO_PORT, SOURCE_EN_GPIO_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(SOURCE_EN_GPIO_PORT, SOURCE_EN_GPIO_PIN, LL_GPIO_PULL_NO);
  SOURCE_EN_SET_OFF();

  /* Enable the peripheral clock of GPIO Port for DISCHARGE */
  DISCHARGE_GPIO_ENABLE_CLOCK();

  /* VBUS Discharge */
  LL_GPIO_SetPinMode(DISCHARGE_GPIO_PORT, DISCHARGE_GPIO_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(DISCHARGE_GPIO_PORT, DISCHARGE_GPIO_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DISCHARGE_GPIO_PORT, DISCHARGE_GPIO_PIN, LL_GPIO_PULL_DOWN);
#if defined(MB1397)
  /* Need to keep DISCHARGE ON as soon as not powering VBUS on MB1397 */
  DISCHARGE_SET_ON();
#else
  DISCHARGE_SET_OFF();
#endif
}


/**
  * @brief  Initialise VBUS power
  * @param  PortCount current port number
  * @param  cc line 1 for cc1 2 for cc2
  * @retval None
  */
void BSP_PWR_VCONNInit(uint8_t PortNum, uint8_t cc)
{
}

/**
  * @brief  Get VBUS power voltage
  * @param  PortCount current port number
  * @retval VBUS voltage
  */
uint32_t BSP_PWR_VBUSGetVoltage(uint8_t PortCount)
{
  uint32_t val;

  val = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI,  LL_ADC_REG_ReadConversionData12(ADC1), LL_ADC_RESOLUTION_12B); /* mV */
#if defined(USE_UNIBOARD)
  /* MB1377 is used for VBUS sourcing on uniboard setup */
  /* Value is multiplied by 14.3 (MB1377A additional board used) for VBUS_ADC */
  val *= 143;
  val /= 10;
#elif defined(MB1397)
  /* STM32G474E_EVAL board is used */
  /* Value is multiplied by 7.613 (Divider R323/R244 (49.9K/330K) for VSENSE */
  val *= 7613;
  val /= 1000;
#else
#warning "Please ensure returned value has been computed properly according to your board setup"
#endif
  return val;
}

#if defined(_SRC) || defined(_DRP)
/**
 * @brief  Set a fixed/variable PDO and manage the power control.
 * @param  PortId Type-C port identifier
 *         This parameter can be take one of the following values:
 *         @arg TYPE_C_PORT_1
 * @param  VbusTargetInmv the vbus Target (in mV)
 * @param  OperatingCurrent the Operating Current (in mA)
 * @param  MaxOperatingCurrent the Max Operating Current (in mA)
 * @retval PD controller status
 */
PWR_StatusTypeDef BSP_PWR_VBUSSetVoltage_Fixed(uint32_t PortId,
                                               uint32_t VbusTargetInmv,
                                               uint32_t OperatingCurrent,
                                               uint32_t MaxOperatingCurrent)
{
  /* Set the power, the precision must be at 5% */
  PWR_StatusTypeDef retr = PWR_ERROR;
#warning "[YMA] To be implemented";

  /* Set the current limitation */
  /* not implemented */

  return retr;
}

/**
 * @brief  Set a fixed/variable PDO and manage the power control.
 * @param  PortId Type-C port identifier
 *         This parameter can be take one of the following values:
 *         @arg TYPE_C_PORT_1
 * @param  VbusTargetMinInmv the vbus Target min (in mV)
 * @param  VbusTargetMaxInmv the vbus Target max (in mV)
 * @param  OperatingCurrent the Operating Current (in mA)
 * @param  MaxOperatingCurrent the Max Operating Current (in mA)
 * @retval PD controller status
 */
PWR_StatusTypeDef BSP_PWR_VBUSSetVoltage_Variable(uint32_t PortId,
                                                  uint32_t VbusTargetMinInmv,
                                                  uint32_t VbusTargetMaxInmv,
                                                  uint32_t OperatingCurrent,
                                                  uint32_t MaxOperatingCurrent)
{
  /* Set the power, the precision must be at 5% */
  PWR_StatusTypeDef retr = PWR_ERROR;
#warning "[YMA] To be implemented";

  /* Set the current limitation */
  /* not implemented */

  return retr;
}
#endif /* _SRC || _DRP */

/**
  * @brief  Set VBUS voltage
  * @param  PortCount    Current port number
  * @param  VbusInmv     Voltage in mV
  * @param  Calibration  Request for ADC calibration
  * @param  Precision    Precision
  * @retval VBUS voltage in mV
  */
uint32_t BSP_PWR_VBUSSetVoltage(uint8_t PortCount, uint16_t VbusInmv, uint16_t Calibration, uint16_t Precision, uint32_t Timeout)
{
  uint32_t vbus_measured = 0;
  
  vbus_measured = VbusInmv;
  return (vbus_measured);
}

/**
  * @brief  Discharge VBUS
  * @param  PortCount     current port number
  * @param  VbusStopInmv  TBC
  * @param  VbusErrorInmv TBC
  * @param  DelayInms     TBC
  * @retval TBC
  */
uint32_t BSP_PWR_VBUSDischarge(uint8_t PortCount, uint32_t VbusStopInmv, uint32_t VbusErrorInmv, uint16_t DelayInms)
{
  uint32_t VbusInmv = 0;
  /* Release MOS */
  DISCHARGE_SET_ON();
  HAL_Delay(DelayInms);
  DISCHARGE_SET_OFF();
  return VbusInmv;
}

/**
  * @brief  Get Current value
  * @param  PortCount     current port number
  * @retval VBUS Current
  */
int32_t BSP_PWR_VBUSGetCurrent(uint8_t PortCount)
{
  return 0;
}


/**
  * @brief  Enable VBUS
  * @param  PortCount     current port number
  * @retval USBPD status
  */
PWR_StatusTypeDef BSP_PWR_VBUSOn(uint8_t PortCount)  /* add a parameter to indicte the powe level */
{
  PWR_StatusTypeDef status = PWR_OK;

  DISCHARGE_SET_OFF();
  SOURCE_EN_SET_ON();

  return status;
}

/**
  * @brief  Get VBUS state
  * @param  PortCount     current port number
  * @retval VBUS state (1 On or 0 off)
  */
uint8_t BSP_PWR_VBUSIsOn(uint8_t PortCount)
{
  uint8_t ret;

  if (IS_SOURCE_EN_SET_ON())
  {
    ret = 1;
  }
  else
  {
    ret = 0;
  }
  return ret;
}

/**
  * @brief  Disable VBUS
  * @param  PortCount     current port number
  * @retval USBPD status
  */
PWR_StatusTypeDef BSP_PWR_VBUSOff(uint8_t PortCount)
{
  PWR_StatusTypeDef status = PWR_OK;
  /* switch off VBUS */
  DISCHARGE_SET_ON();
  SOURCE_EN_SET_OFF();

#if defined(MB1397)
  /* Need to keep DISCHARGE ON as soon as not powering VBUS on MB1397 */
#else
  HAL_Delay(5);
  DISCHARGE_SET_OFF();
#endif

  return status;
}

/**
  * @brief  Enable VCONN on CC line
  * @param  PortCount current port number
  * @param  cc        TBC
  * @retval USBPD status
  */
PWR_StatusTypeDef BSP_PWR_VCONNOn(uint8_t PortCount, uint8_t cc)
{
  PWR_StatusTypeDef status = PWR_OK;
  return status;
}

/**
  * @brief  Disable VCONN on CC line
  * @param  PortCount current port number
  * @param  cc        TBC
  * @retval USBPD status
  */
PWR_StatusTypeDef BSP_PWR_VCONNOff(uint8_t PortCount, uint8_t cc)
{
  PWR_StatusTypeDef status = PWR_OK;
  return status;
}

/**
  * @brief  Configure ADC (ADC instance: ADC1) and GPIO used by ADC channels.
  * @note   In case re-use of this function outside of this example:
  *         This function includes checks of ADC hardware constraints before
  *         executing some configuration functions.
  *         - In this example, all these checks are not necessary but are
  *           implemented anyway to show the best practice usages
  *           corresponding to reference manual procedure.
  *           (On some STM32 series, setting of ADC features are not
  *           conditioned to ADC state. However, in order to be compliant with
  *           other STM32 series and to show the best practice usages,
  *           ADC state is checked anyway with same constraints).
  *           Software can be optimized by removing some of these checks,
  *           if they are not relevant considering previous settings and actions
  *           in user application.
  *         - If ADC is not in the appropriate state to modify some parameters,
  *           the setting of these parameters is bypassed without error
  *           reporting:
  *           it can be the expected behavior in case of recall of this
  *           function to update only a few parameters (which update fullfills
  *           the ADC state).
  *           Otherwise, it is up to the user to set the appropriate error
  *           reporting in user application.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
static void Configure_ADC(void)
{
  /*## Configuration of GPIO used by ADC channels ############################*/
  uint32_t wait_loop_index;

  /* Note: On this STM32 device, ADC1 channel 0 is mapped on GPIO pin PA.00 */

  /* Enable GPIO Clock */
  VSENSE_GPIO_ENABLE_CLOCK();

  /* Configure GPIO in analog mode to be used as ADC input */
  LL_GPIO_SetPinMode(VSENSE_GPIO_PORT, VSENSE_GPIO_PIN, LL_GPIO_MODE_ANALOG);

  /*## Configuration of ADC ##################################################*/

  /*## Configuration of ADC hierarchical scope: common to several ADC ########*/

  /* Enable ADC clock (core clock) */
  VSENSE_ADC_ENABLE_CLOCK();

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       All ADC instances of the ADC common group must be disabled.        */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  uint32_t IsEnabled = __LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE(VSENSE_ADC_COMMON);
  if (IsEnabled == 0)
  {
    /* Note: Call of the functions below are commented because they are       */
    /*       useless in this example:                                         */
    /*       setting corresponding to default configuration from reset state. */

    /* Set ADC clock (conversion clock) common to several ADC instances */
    /* Note: On this STM32 serie, ADC common clock asynchonous prescaler      */
    /*       is applied to each ADC instance if ADC instance clock is         */
    /*       set to clock source asynchronous                                 */
    /*       (refer to function "LL_ADC_SetClock()" below).                   */
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(VSENSE_ADC_INSTANCE), LL_ADC_CLOCK_SYNC_PCLK_DIV4);

    /* Set ADC measurement path to internal channels */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(VSENSE_ADC_INSTANCE), LL_ADC_PATH_INTERNAL_VREFINT);

    /*## Configuration of ADC hierarchical scope: multimode ####################*/

    /* Note: Feature not available on this STM32 serie */
  }

  /*## Configuration of ADC hierarchical scope: ADC instance #################*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled.                                              */
  if (LL_ADC_IsEnabled(VSENSE_ADC_INSTANCE) == 0)
  {
    /* Delay for internal voltage reference stabilization time.               */
    /* Compute number of CPU cycles to wait for, from delay in us.            */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    /* Note: If system core clock frequency is below 200kHz, wait time        */
    /*       is only a few CPU processing cycles.                             */
    wait_loop_index = ((LL_ADC_DELAY_VREFINT_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while (wait_loop_index != 0)
    {
      wait_loop_index--;
    }
  }

  /*## Configuration of ADC hierarchical scope: ADC group regular ############*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled or enabled without conversion on going        */
  /*       on group regular.                                                  */
  if ((LL_ADC_IsEnabled(VSENSE_ADC_INSTANCE) == 0)               ||
      (LL_ADC_REG_IsConversionOngoing(VSENSE_ADC_INSTANCE) == 0))
  {
    /* Set ADC group regular trigger source */
    LL_ADC_REG_SetTriggerSource(VSENSE_ADC_INSTANCE, LL_ADC_REG_TRIG_SOFTWARE);

    /* Set ADC group regular continuous mode */
    LL_ADC_REG_SetContinuousMode(VSENSE_ADC_INSTANCE, LL_ADC_REG_CONV_CONTINUOUS);

    /* Set ADC group regular overrun behavior */
    LL_ADC_REG_SetOverrun(VSENSE_ADC_INSTANCE, LL_ADC_REG_OVR_DATA_OVERWRITTEN);

    /* Set ADC group regular sequencer */
    /* Note: On this STM32 serie, ADC group regular sequencer has             */
    /*       two settings:                                                    */
    /*       - Sequencer configured to fully configurable:                    */
    /*         sequencer length and each rank                                 */
    /*         affectation to a channel are configurable.                     */
    /*         Channels selection is limited to channels 0 to 14.             */
    /*       - Sequencer configured to not fully configurable:                */
    /*         sequencer length and each rank affectation to a channel        */
    /*         are fixed by channel HW number.                                */
    /*         Channels selection is not limited (channels 0 to 18).          */
    /*       Refer to description of function                                 */
    /*       "LL_ADC_REG_SetSequencerConfigurable()".                         */

    /* Set ADC group regular sequence: channel on rank corresponding to       */
    /* channel number.                                                        */
    LL_ADC_REG_SetSequencerRanks(VSENSE_ADC_INSTANCE, VSENSE_ADC_RANK, VSENSE_ADC_CHANNEL);

  }

  /*## Configuration of ADC hierarchical scope: ADC group injected ###########*/

  /* Note: Feature available on this STM32 serie but not used */

  /*## Configuration of ADC hierarchical scope: channels #####################*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled or enabled without conversion on going        */
  /*       on either groups regular or injected.                              */
  if ((LL_ADC_IsEnabled(VSENSE_ADC_INSTANCE) == 0)                    ||
      ((LL_ADC_REG_IsConversionOngoing(VSENSE_ADC_INSTANCE) == 0) &&
       (LL_ADC_INJ_IsConversionOngoing(VSENSE_ADC_INSTANCE) == 0)))
  {
    /* Set ADC channels sampling time */
    LL_ADC_SetChannelSamplingTime(VSENSE_ADC_INSTANCE, VSENSE_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_247CYCLES_5);
  }

  /*## Configuration of ADC transversal scope: analog watchdog ###############*/

  /* Note: On this STM32 serie, there is only 1 analog watchdog available.    */

  /* Set ADC analog watchdog: channels to be monitored */
  LL_ADC_SetAnalogWDMonitChannels(VSENSE_ADC_INSTANCE, LL_ADC_AWD1, LL_ADC_AWD_ALL_CHANNELS_REG);

  /* Set ADC analog watchdog: thresholds */
  LL_ADC_ConfigAnalogWDThresholds(VSENSE_ADC_INSTANCE, LL_ADC_AWD1, 700, 600);

  /*## Configuration of ADC transversal scope: oversampling ##################*/

  /* Set ADC oversampling scope */
  LL_ADC_SetOverSamplingScope(VSENSE_ADC_INSTANCE, LL_ADC_OVS_GRP_REGULAR_CONTINUED);

  /* Set ADC oversampling parameters */
  LL_ADC_ConfigOverSamplingRatioShift(VSENSE_ADC_INSTANCE, LL_ADC_OVS_RATIO_16, LL_ADC_OVS_SHIFT_RIGHT_4);

  /*## Configuration of ADC interruptions ####################################*/
  /* Enable ADC analog watchdog 1 interruption */
  LL_ADC_EnableIT_AWD1(VSENSE_ADC_INSTANCE);
}

/**
  * @brief  Perform ADC activation procedure to make it ready to convert
  *         (ADC instance: VSENSE_ADC_INSTANCE).
  * @note   Operations:
  *         - ADC instance
  *           - Run ADC self calibration
  *           - Enable ADC
  *         - ADC group regular
  *           none: ADC conversion start-stop to be performed
  *                 after this function
  *         - ADC group injected
  *           Feature not available                                  (feature not available on this STM32 serie)
  * @param  None
  * @retval None
  */
void Activate_ADC(void)
{
  __IO uint32_t wait_loop_index = 0;
  __IO uint32_t backup_setting_adc_dma_transfer;
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0; /* Variable used for timeout management */
#endif /* USE_TIMEOUT */

  /*## Operation on ADC hierarchical scope: ADC instance #####################*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled.                                              */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if (LL_ADC_IsEnabled(VSENSE_ADC_INSTANCE) == 0)
  {
    /* Disable ADC deep power down mode */
    LL_ADC_DisableDeepPowerDown(VSENSE_ADC_INSTANCE);

    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(VSENSE_ADC_INSTANCE);

    /* Delay for ADC internal voltage regulator stabilization.                */
    /* Compute number of CPU cycles to wait for, from delay in us.            */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    /* Note: If system core clock frequency is below 200kHz, wait time        */
    /*       is only a few CPU processing cycles.                             */
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while (wait_loop_index != 0)
    {
      wait_loop_index--;
    }

    /* Disable ADC DMA transfer request during calibration */
    /* Note: Specificity of this STM32 serie: Calibration factor is           */
    /*       available in data register and also transfered by DMA.           */
    /*       To not insert ADC calibration factor among ADC conversion data   */
    /*       in DMA destination address, DMA transfer must be disabled during */
    /*       calibration.                                                     */
    backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(VSENSE_ADC_INSTANCE);
    LL_ADC_REG_SetDMATransfer(VSENSE_ADC_INSTANCE, LL_ADC_REG_DMA_TRANSFER_NONE);

    /* Run ADC self calibration */
    LL_ADC_StartCalibration(VSENSE_ADC_INSTANCE, LL_ADC_SINGLE_ENDED);

    /* Poll for ADC effectively calibrated */
#if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
#endif /* USE_TIMEOUT */

    while (LL_ADC_IsCalibrationOnGoing(VSENSE_ADC_INSTANCE) != 0)
    {
#if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if (Timeout-- == 0)
        {
          /* Time-out occurred. Set LED to blinking mode */
          LED_Blinking(LED_BLINK_ERROR);
        }
      }
#endif /* USE_TIMEOUT */
    }

    /* Restore ADC DMA transfer request after calibration */
    LL_ADC_REG_SetDMATransfer(VSENSE_ADC_INSTANCE, backup_setting_adc_dma_transfer);

    /* Delay between ADC end of calibration and ADC enable.                   */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
    while (wait_loop_index != 0)
    {
      wait_loop_index--;
    }

    /* Enable ADC */
    LL_ADC_Enable(VSENSE_ADC_INSTANCE);

    /* Poll for ADC ready to convert */
#if (USE_TIMEOUT == 1)
    Timeout = ADC_ENABLE_TIMEOUT_MS;
#endif /* USE_TIMEOUT */

    while (LL_ADC_IsActiveFlag_ADRDY(VSENSE_ADC_INSTANCE) == 0)
    {
#if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if (Timeout-- == 0)
        {
          /* Time-out occurred. Set LED to blinking mode */
          LED_Blinking(LED_BLINK_ERROR);
        }
      }
#endif /* USE_TIMEOUT */
    }

    /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
    /*       status afterwards.                                               */
    /*       This flag should be cleared at ADC Deactivation, before a new    */
    /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
  }

  /*## Operation on ADC hierarchical scope: ADC group regular ################*/
  /* Note: No operation on ADC group regular performed here.                  */
  /*       ADC group regular conversions to be performed after this function  */
  /*       using function:                                                    */
  /*       "LL_ADC_REG_StartConversion();"                                    */

  /*## Operation on ADC hierarchical scope: ADC group injected ###############*/
  /* Note: Feature not available on this STM32 serie */
}


/**
  * @brief  Measure voltage on device pin Vref+ provided at board level
  *         (usually connected to Vdda and Vdd)
  * @param  None
  * @retval None
  */
uint32_t VrefAnalog_CorrectionFactorx1000 = 0U;

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

