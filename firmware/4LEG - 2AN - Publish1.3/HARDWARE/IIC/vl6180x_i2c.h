#ifndef VL6180_I2C_H_
#define VL6180_I2C_H_
#include "include.h"

#define GPIO_0_PIN_PORT				    P1_0
#define GPIO_1_PIN_PORT		                    P1_1
#define Power_PIN_PORT		                    P1_2

#define GPIO_0_OUTPUT		                    (P1DIR |= BIT0)
#define GPIO_1_OUTPUT			            (P1DIR |= BIT1)
#define Power_PIN_OUTPUT			    (P1DIR |= BIT2)

#define DEV_ADDR                            0x29//        0x52//(0X29<<1)

//This read-only register contains the device identifier, set to B4
#define VL6180X_IDENTIFICATION_MODEL_ID_RETURN             0xB4

// VL6180X registers
#define VL6180X_WHO_AM_I                             0x0000   // should be 0xB4
#define VL6180X_IDENTIFICATION_MODEL_ID              0x0000
#define VL6180X_IDENTIFICATION_MODEL_REV_MAJOR       0x0001
#define VL6180X_IDENTIFICATION_MODEL_REV_MINOR       0x0002
#define VL6180X_IDENTIFICATION_MODULE_REV_MAJOR      0x0003
#define VL6180X_IDENTIFICATION_MODULE_REV_MINOR      0x0004
#define VL6180X_IDENTIFICATION_DATE_HI               0x0006
#define VL6180X_IDENTIFICATION_DATE_LO               0x0007
#define VL6180X_IDENTIFICATION_TIME_HI               0x0008
#define VL6180X_IDENTIFICATION_TIME_LO               0x0009
#define VL6180X_SYSTEM_MODE_GPIO0                    0x0010
#define VL6180X_SYSTEM_MODE_GPIO1                    0x0011
#define VL6180X_SYSTEM_HISTORY_CTRL                  0x0012
/*
[5:3] als_int_mode: Interrupt mode source for ALS readings:
0: Disabled
1: Level Low (value < thresh_low)
2: Level High (value > thresh_high)
3: Out Of Window (value < thresh_low OR value > thresh_high)
4: New sample ready

[2:0] range_int_mode: Interrupt mode source for Range readings:
0: Disabled
1: Level Low (value < thresh_low)
2: Level High (value > thresh_high)
3: Out Of Window (value
 */
#define VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO         0x0014
#define VL6180X_SYSTEM_INTERRUPT_CLEAR               0x0015
#define VL6180X_SYSTEM_FRESH_OUT_OF_RESET            0x0016
#define VL6180X_SYSTEM_GROUPED_PARAMETER_HOLD        0x0017

/*
[1] sysrange__mode_select: Device Mode select
0: Ranging Mode Single-Shot
1: Ranging Mode Continuous
[0] sysrange__startstop: StartStop trigger based on current mode and system configuration of
device_ready. FW clears register automatically.
Setting this bit to 1 in single-shot mode starts a single measurement.
Setting this bit to 1 in continuous mode will either start continuous operation (if stopped) or halt
continuous operation (if started).
This bit is auto-cleared in both modes of operation.
 */
#define VL6180X_SYSRANGE_START                       0x0018

#define VL6180X_SYSRANGE_THRESH_HIGH                 0x0019
#define VL6180X_SYSRANGE_THRESH_LOW                  0x001A
#define VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD     0x001B
#define VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME        0x001C
#define VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE 0x001E
#define VL6180X_SYSRANGE_CROSSTALK_VALID_HEIGHT      0x0021
#define VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE  0x0022
#define VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET   0x0024
#define VL6180X_SYSRANGE_RANGE_IGNORE_VALID_HEIGHT   0x0025
#define VL6180X_SYSRANGE_RANGE_IGNORE_THRESHOLD      0x0026
#define VL6180X_SYSRANGE_MAX_AMBIENT_LEVEL_MULT      0x002C
#define VL6180X_SYSRANGE_RANGE_CHECK_ENABLES         0x002D
#define VL6180X_SYSRANGE_VHV_RECALIBRATE             0x002E
#define VL6180X_SYSRANGE_VHV_REPEAT_RATE             0x0031

/*
[1] sysals__mode_select: Device Mode select
0: ALS Mode Single-Shot
1: ALS Mode Continuous
[0] VL6180X_SYSALS_STARTstop: Start/Stop trigger based on current mode and system configuration of
device_ready. FW clears register automatically.
Setting this bit to 1 in single-shot mode starts a single measurement.
Setting this bit to 1 in continuous mode will either start continuous operation (if stopped) or halt
continuous operation (if started).
This bit is auto-cleared in both modes of operation.
See 6.2.56: INTERLEAVED_MODE__ENABLE for combined ALS and Range operation.
 */
#define VL6180X_SYSALS_START                         0x038
#define VL6180X_SYSALS_THRESH_HIGH                   0x003A
#define VL6180X_SYSALS_THRESH_LOW                    0x003C
#define VL6180X_SYSALS_INTERMEASUREMENT_PERIOD       0x003E


/*
VL6180X_SYSALS_ANALOGUE_GAIN_light: ALS analogue gain (light channel)
0: ALS Gain = 20
1: ALS Gain = 10
2: ALS Gain = 5.0
3: ALS Gain = 2.5
4: ALS Gain = 1.67
5: ALS Gain = 1.25
6: ALS Gain = 1.0
7: ALS Gain = 40
Controls the “light?channel gain.
Note: Upper nibble should be set to 0x4 i.e. For ALS gain of 1.0 write 0x46.
 */
#define VL6180X_SYSALS_ANALOGUE_GAIN                 0x003F
#define VL6180X_SYSALS_INTEGRATION_PERIOD            0x0040
#define VL6180X_RESULT_RANGE_STATUS                  0x004D
#define VL6180X_RESULT_ALS_STATUS                    0x004E
/*
[7:6] result_int_error_gpio: Interrupt bits for Error:
0: No error reported
1: Laser Safety Error
2: PLL error (either PLL1 or PLL2)
[5:3] result_int_als_gpio: Interrupt bits for ALS:
0: No threshold events reported
1: Level Low threshold event
2: Level High threshold event
3: Out Of Window threshold event
4: New Sample Ready threshold event
[2:0] result_int_range_gpio: Interrupt bits for Range:
0: No threshold events reported
1: Level Low threshold event
2: Level High threshold event
3: Out Of Window threshold event
4: New Sample Ready threshold event
 */
#define VL6180X_RESULT_INTERRUPT_STATUS_GPIO         0x004F
#define VL6180X_RESULT_ALS_VAL                       0x0050
#define VL6180X_RESULT_HISTORY_BUFFER0               0x0052 // This is a FIFO buffer that can store 8 range values or 16 ALS values
#define VL6180X_RESULT_HISTORY_BUFFER1               0x0053 // It would be read in burst mode so all that is
#define VL6180X_RESULT_HISTORY_BUFFER2               0x0054 // needed would be to reference the first address
#define VL6180X_RESULT_HISTORY_BUFFER3               0x0055
#define VL6180X_RESULT_HISTORY_BUFFER4               0x0056
#define VL6180X_RESULT_HISTORY_BUFFER5               0x0057
#define VL6180X_RESULT_HISTORY_BUFFER6               0x0058
#define VL6180X_RESULT_HISTORY_BUFFER7               0x0059
#define VL6180X_RESULT_HISTORY_BUFFER8               0x0060  // end of FIFO

/*
[7:0] result__range_val: Final range result value presented to the user for use. Unit is in mm.
 */
#define VL6180X_RESULT_RANGE_VAL                     0x0062
#define VL6180X_RESULT_RANGE_RAW                     0x0064
#define VL6180X_RESULT_RANGE_RETURN_RATE             0x0066
#define VL6180X_RESULT_RANGE_REFERENCE_RATE          0x0068
#define VL6180X_RESULT_RANGE_RETURN_SIGNAL_COUNT     0x006C
#define VL6180X_RESULT_RANGE_REFERENCE_SIGNAL_COUNT  0x0070
#define VL6180X_RESULT_RANGE_RETURN_AMB_COUNT        0x0074
#define VL6180X_RESULT_RANGE_REFERENCE_AMB_COUNT     0x0078
#define VL6180X_RESULT_RANGE_RETURN_CONV_TIME        0x007C
#define VL6180X_RESULT_RANGE_REFERENCE_CONV_TIME     0x0080
#define VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD      0x010A
#define VL6180X_FIRMWARE_BOOTUP                      0x0119
#define VL6180X_FIRMWARE_RESULT_SCALER               0x0120
#define VL6180X_I2C_SLAVE_DEVICE_ADDRESS             0x0212
#define VL6180X_INTERLEAVED_MODE_ENABLE              0x02A3



#define START_SINGLE_MODE                            0x01
#define START_CONTINUOUS_MODE                        0x02


#define ALS_SINGLE_MODE_MASK                         0x38
#define ALS_SINGLE_MODE_READY                        0x20
#define RANGE_SINGLE_MODE_MASK                       0x07
#define RANGE_SINGLE_MODE_READY                      0x04

#define CLEAR_RANGE_INT                              0x01
#define CLEAR_ALS_INT                                0x02
#define CLEAR_ERROR_INT                              0x04

#define contRangeMode                                0
#define contALSMode                                  1
#define VL6180XMode                                  0

void VL6180X_Init(void);
uint8 RangePollingRead(void);
uint16 ligthPollingRead(void);
uint8 VL6180X_WriteBytes(uint8 I2C_addr,uint16 index,uint8 dat);
uint8 VL6180x_ReadBytes(uint8 I2C_addr, uint16 index) ;
uint16 VL6180x_Read_Two_Bytes(uint8 I2C_addr, uint16 index);

#endif

