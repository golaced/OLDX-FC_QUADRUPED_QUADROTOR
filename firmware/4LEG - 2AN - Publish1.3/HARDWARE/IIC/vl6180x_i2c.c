#include "vl6180x_i2c.h"
#include "iic_vl53.h"
int sel_iic=3;
uint8 data_reg=0;

uint8 VL6180X_WriteBytes(uint8 I2C_addr,uint16 index,uint8 dat)
{
	IIC_Write_1Byte_VL53(I2C_addr,index, dat,sel_iic);
}

uint8 VL6180x_ReadBytes(uint8 I2C_addr, uint16 index) {
	
u8 val;	
IIC_Read_1Byte_VL53(I2C_addr,index,&val,sel_iic);
return val;
}

uint16 VL6180x_Read_Two_Bytes(uint8 I2C_addr, uint16 index){
u8 gbuf[2];
IIC_Read_nByte_VL53(I2C_addr, index , 2, gbuf,sel_iic);	
return 	gbuf[0]<<8|gbuf[1];
}

uint8 VL6180X_CheckID(void)
{
    uint8 data_ID;
  	IIC_Read_1Byte_VL53(DEV_ADDR,VL6180X_IDENTIFICATION_MODEL_ID,&data_ID,sel_iic);
    //data_ID = VL6180x_ReadBytes(DEV_ADDR, VL6180X_IDENTIFICATION_MODEL_ID);
    return data_ID;
}

void VL6180X_Init(void)
{
    uint8 reset=0, timeOut=0;
//    GPIO_0_OUTPUT;
//    GPIO_1_OUTPUT;
////    Power_PIN_OUTPUT;
////    Power_PIN_PORT=0;
////    GPIO_0_PIN_PORT = 0;
////    GPIO_1_PIN_PORT = 0;
////    delay_ms(5);
////    Power_PIN_PORT=1;
////    delay_ms(5);
////    GPIO_0_PIN_PORT = 1;
////    GPIO_1_PIN_PORT = 1;
////    delay_ms(5);
//    GPIO_0_PIN_PORT = 0;
//    GPIO_1_PIN_PORT = 0;
//    delay_ms(10);
//    GPIO_0_PIN_PORT = 1;
//    GPIO_1_PIN_PORT = 1;
//    delay_ms(1);
    VL6180X_CheckID();
   // while(!(VL6180X_CheckID() == 0xB4));
   // delay_ms(5);
    
    VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x01);  
    while (reset!=1 && timeOut<100) {
        reset = VL6180x_ReadBytes(DEV_ADDR, VL6180X_SYSTEM_FRESH_OUT_OF_RESET);  // read fresh_out_of_reset bit
        if(reset == 1) {  // if if fresh_out_of_reset bit set, then device has been freshly initialized

            // SR03 settings AN4545 24/27 DocID026571 Rev 19 SR03 settings
            // http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
            // Below are the recommended settings required to be loaded onto the VL6180X during the
            // initialisation of the device (see Section 1.3).
            // Mandatory : private registers


            VL6180X_WriteBytes(DEV_ADDR, 0x0207, 0x01);
            data_reg = VL6180x_ReadBytes(DEV_ADDR, 0x0207);

            VL6180X_WriteBytes(DEV_ADDR, 0x0208, 0x01);
            VL6180X_WriteBytes(DEV_ADDR, 0x0096, 0x00);
            VL6180X_WriteBytes(DEV_ADDR, 0x0097, 0xFD);
            VL6180X_WriteBytes(DEV_ADDR, 0x00e3, 0x00);
            VL6180X_WriteBytes(DEV_ADDR, 0x00e4, 0x04);
            data_reg = VL6180x_ReadBytes(DEV_ADDR, 0x00e4);

            VL6180X_WriteBytes(DEV_ADDR, 0x00e5, 0x02);
            VL6180X_WriteBytes(DEV_ADDR, 0x00e6, 0x01);
            VL6180X_WriteBytes(DEV_ADDR, 0x00e7, 0x03);
            VL6180X_WriteBytes(DEV_ADDR, 0x00f5, 0x02);
            VL6180X_WriteBytes(DEV_ADDR, 0x00d9, 0x05);
            VL6180X_WriteBytes(DEV_ADDR, 0x00db, 0xce);
            VL6180X_WriteBytes(DEV_ADDR, 0x00dc, 0x03);
            VL6180X_WriteBytes(DEV_ADDR, 0x00dd, 0xf8);
            VL6180X_WriteBytes(DEV_ADDR, 0x009f, 0x00);
            VL6180X_WriteBytes(DEV_ADDR, 0x00a3, 0x3c);
            VL6180X_WriteBytes(DEV_ADDR, 0x00b7, 0x00);
            VL6180X_WriteBytes(DEV_ADDR, 0x00bb, 0x3c);
            VL6180X_WriteBytes(DEV_ADDR, 0x00b2, 0x09);
            VL6180X_WriteBytes(DEV_ADDR, 0x00ca, 0x09);
            VL6180X_WriteBytes(DEV_ADDR, 0x0198, 0x01);
            VL6180X_WriteBytes(DEV_ADDR, 0x01b0, 0x17);
            VL6180X_WriteBytes(DEV_ADDR, 0x01ad, 0x00);
            VL6180X_WriteBytes(DEV_ADDR, 0x00ff, 0x05);
            VL6180X_WriteBytes(DEV_ADDR, 0x0100, 0x05);
            VL6180X_WriteBytes(DEV_ADDR, 0x0199, 0x05);
            VL6180X_WriteBytes(DEV_ADDR, 0x01a6, 0x1b);
            VL6180X_WriteBytes(DEV_ADDR, 0x01ac, 0x3e);
            VL6180X_WriteBytes(DEV_ADDR, 0x01a7, 0x1f);
            VL6180X_WriteBytes(DEV_ADDR, 0x0030, 0x00);
            data_reg = VL6180x_ReadBytes(DEV_ADDR, 0x0030);
            

            // configure range measurement for low power
            // Specify range measurement interval in units of 10 ms from 0 (= 10 ms) - 254 (= 2.55 s)

            VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, 0x0A); // 100 ms interval in steps of 10 ms
            VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSRANGE_VHV_REPEAT_RATE, 0xFF); // sets number of range measurements after which autocalibrate is performed
            VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSRANGE_VHV_RECALIBRATE, 0x01); // perform temperature calibration of the ranging sensor

            // Set Early Convergence Estimate for lower power consumption
            VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x32); // set max convergence time to 50 ms (steps of 1 ms)
            VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSRANGE_RANGE_CHECK_ENABLES, 0x10 | 0x01);  // enable (0x01) early convergence estimate
            // This ECE is calculated as follows:
            // [(1 - % below threshold) x 0.5 x 15630]/ range max convergence time
            // This is ~123 ms for 50 ms max convergence time and 80% below threshold
            // This is a sixteen bit (2 byte) register with the first byte MSByte and the second LSByte
            VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, 0x00); // set early convergence estimate to 5%
            VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE + 1, 0x7B); // set early convergence estimate to 5%

            // Configure ALS
            VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 0x0A);   // set to 100 ms
            data_reg = VL6180x_ReadBytes(DEV_ADDR, VL6180X_SYSALS_INTERMEASUREMENT_PERIOD);
            
            // Following is a 16-bit register with the first MSByte reserved

            VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSALS_INTEGRATION_PERIOD, 0x00);
            VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSALS_INTEGRATION_PERIOD+1, 0x63);        // set ALS integration time to 100 ms in steps of 1 ms

            // The internal readout averaging sample period can be adjusted from 0 to 255. Increasing the sampling
            // period decreases noise but also reduces the effective max convergence time and increases power consumption:
            // Effective max convergence time = max convergence time - readout averaging period (see
            // Section 2.5: Range timing). Each unit sample period corresponds to around 64.5 ?s additional
            // processing time. The recommended setting is 48 which equates to around 4.3 ms
            VL6180X_WriteBytes(DEV_ADDR, VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30);  // compromise between low noise and increased execution time
            // Gain can be 0 = 20, 1 = 10, 2 = 5, 3 = 2.5, 4 = 1.67, 5 = 1.25, 6 = 1.0 and 7 = 40
            // These are value possible for the lower nibble. The upper nibble must be 4
            VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSALS_ANALOGUE_GAIN, 0x40 | 1);   // Sets light and dark gain (don't change upper nibble)
            data_reg = VL6180x_ReadBytes(DEV_ADDR, VL6180X_SYSALS_ANALOGUE_GAIN);
            
            // Scalar (1 - 32, bits 4:0) to multiply raw ALS count for additonal gain, if necessary
            VL6180X_WriteBytes(DEV_ADDR, VL6180X_FIRMWARE_RESULT_SCALER, 0x01);

            // Configure the interrupts
            VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSTEM_MODE_GPIO0, 0x00);                // set up GPIO 0 (set to high impedence for now)
            VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSTEM_MODE_GPIO1, 0x00);                // set up GPIO 1 (set to high impedence for now)
            VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x24);     // enable sample ready interrupt

#if 0
            // enable continuous range mode
            if(VL6180XMode == contRangeMode) {
                VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSRANGE_START, 0x03);  // start auto range mode

            }
            // enable continuous ALS mode
            if(VL6180XMode == contALSMode) {
                // Configure ALS
                VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 0x32);   // set to 100 ms
                // Following is a 16-bit register with the first MSByte reserved
                VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSALS_INTEGRATION_PERIOD+1, 0x32);        // set ALS integration time to 50 ms in steps of 1 ms
                VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSALS_START, 0x03);                     // start auto range mode
            }
#endif
            // Clear reset bit
            VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x00);        // reset fresh_out_of_reset bit to zero
        } else {
            timeOut++;
        }
    }
    reset=0;
   
}

void readRange(void) {
  // wait for device to be ready for range measurement
  while (! (VL6180x_ReadBytes(DEV_ADDR, 0x04d) & 0x01));

  // Start a range measurement
  VL6180X_WriteBytes(DEV_ADDR,0x018, 0x01);

  // Poll until bit 2 is set
  while (! (VL6180x_ReadBytes(DEV_ADDR,0x04f) & 0x04));

  // read range in mm
  uint8_t range = VL6180x_ReadBytes(DEV_ADDR,0x062);

  // clear interrupt
  VL6180X_WriteBytes(DEV_ADDR,0x015, 0x07);

  ls53[sel_iic].distance=range;
}


uint8 RangePollingRead(void)
{
    uint8 status;
    uint8  distance;
     readRange();/*

    do {
        status = VL6180x_ReadBytes(DEV_ADDR, VL6180X_RESULT_RANGE_STATUS);
    } while ((status & (1 << 0)) == 0);
    
    VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSRANGE_START, START_SINGLE_MODE);
    data_reg = VL6180x_ReadBytes(DEV_ADDR, VL6180X_SYSRANGE_START);
    
    status = VL6180x_ReadBytes(DEV_ADDR, VL6180X_RESULT_INTERRUPT_STATUS_GPIO);
    status = status & RANGE_SINGLE_MODE_MASK;
    

    while (status != RANGE_SINGLE_MODE_READY) {
        status = VL6180x_ReadBytes(DEV_ADDR, VL6180X_RESULT_INTERRUPT_STATUS_GPIO);
        status = status & RANGE_SINGLE_MODE_MASK;
        delay_ms(1);
    }
    
    delay_ms(10);
    distance = VL6180x_ReadBytes(DEV_ADDR, VL6180X_RESULT_RANGE_VAL);
    VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSTEM_INTERRUPT_CLEAR, CLEAR_ALS_INT);
		ls53[sel_iic].distance=distance;
    return distance;*/
}

uint16 ligthPollingRead(void) 
{
    uint8 status;
    uint16 alsraw;
    
    /* Wait for device ready. */
    do {
        status = VL6180x_ReadBytes(DEV_ADDR, VL6180X_RESULT_ALS_STATUS);
    } while ((status & (1 << 0)) == 0);
    
    VL6180X_WriteBytes(DEV_ADDR,VL6180X_SYSALS_START, START_SINGLE_MODE);
    data_reg = VL6180x_ReadBytes(DEV_ADDR, VL6180X_SYSALS_START);

    status = VL6180x_ReadBytes(DEV_ADDR,VL6180X_RESULT_INTERRUPT_STATUS_GPIO);
    status = status & ALS_SINGLE_MODE_MASK;

//     while (status != ALS_SINGLE_MODE_READY) {
//        status =  VL6180x_ReadBytes(DEV_ADDR, VL6180X_RESULT_INTERRUPT_STATUS_GPIO);
//        status = status & ALS_SINGLE_MODE_MASK;
//        delay_ms(1);
//    }

    alsraw = VL6180x_Read_Two_Bytes(DEV_ADDR, VL6180X_RESULT_ALS_VAL);
    //als = 0.32f * ((float) alsraw / 10.32) * (100.0f/100.0f);
    VL6180X_WriteBytes(DEV_ADDR, VL6180X_SYSTEM_INTERRUPT_CLEAR, CLEAR_ALS_INT);

    return alsraw;
}


