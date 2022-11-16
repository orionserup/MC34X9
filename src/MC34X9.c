/**
 * \file MC34X9.c
 * \author Orion Serup (orionserup@gmail.com)
 * \brief 
 * \version 0.1
 * \date 2022-10-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "MC34X9.h"

//Initialize the MC34X9 sensor and set as the default configuration
uint16_t MC34X9Start(MC34X9* const dev)
{

  //Init Reset
  if(MC34X9Reset(dev) != 1 || MC34X9SetMode(dev, MC34X9_MODE_STANDBY) != 1)
    return 0;

  /* Check I2C connection */
  uint8_t id;
  dev->config.read_reg(dev->address, MC34X9_REG_PROD, &id, 1);
  if (id != MC34X9_CHIP_ID)
    return 0;

  //Range: 8g
  MC34X9SetRangeCtrl(MC34X9_CFG_RANGE_DEFAULT);
  //Sampling Rate: 50Hz by default
  MC34X9SetSampleRate(MC34X9_CFG_SAMPLE_RATE_DEFAULT);
  //Mode: Active
  MC34X9SetMode(dev,MC34X9_MODE_CWAKE);

  delay(50);

  return true;
}

void MC34X9Wake()
{
  //Set mode as wake
  MC34X9SetMode(dev,MC34X9_MODE_CWAKE);
}

void MC34X9Stop()
{
  //Set mode as Sleep
  MC34X9SetMode(dev,MC34X9_MODE_STANDBY);
}

//Initial reset
void MC34X9reset()
{
  // Stand by mode
  MC34X9Write8(MC34X9_REG_MODE, MC34X9_MODE_STANDBY);

  delay(10);

  // power-on-reset
  MC34X9Write8(0x1c, 0x40);

  delay(50);

  // Disable interrupt
  MC34X9Write8(0x06, 0x00);
  delay(10);
  // 1.00x Aanalog Gain
  MC34X9Write8(0x2B, 0x00);
  delay(10);
  // DCM disable
  MC34X9Write8(0x15, 0x00);

}

//Set the operation mode
void MC34X9SetMode(MC34X9_mode_t mode)
{
  uint8_t value;

  value = MC34X9Read8(dev, MC34X9_REG_MODE);
  value &= 0b11110000;
  value |= mode;

  MC34X9Write8(MC34X9_REG_MODE, value);
}

//Set the range control
void MC34X9SetRangeCtrl(MC34X9_range_t range)
{
  uint8_t value;
  CfgRange = range;
  MC34X9SetMode(MC34X9_MODE_STANDBY);
  value = MC34X9Read8(dev, MC34X9_REG_RANGE_C);
  value &= 0b00000111;
  value |= (range << 4) & 0x70;
  MC34X9Write8(MC34X9_REG_RANGE_C, value);
}

//Set the sampling rate
void MC34X9SetSampleRate(MC34X9* const dev, const MC34X9SampleRate sample_rate)
{
  uint8_t value;
  MC34X9SetMode(dev,MC34X9_MODE_STANDBY);
  value = MC34X9Read8(dev, MC34X9_REG_SR);
  value &= 0b00000000;
  value |= sample_rate;
  MC34X9Write8(MC34X9_REG_SR, value);
}

// Set Motion feature
void MC34X9SetMotionCtrl(MC34X9* const dev, const bool tilt_ctrl,
                           bool flip_ctl,
                           bool anym_ctl,
                           bool shake_ctl,
                           bool tilt_35_ctl) {
  uint8_t CfgMotion = 0;

  if (tilt_ctrl || flip_ctl) {
    MC34X6SetTiltFlip(dev);
    CfgMotion |= (((tilt_ctrl || flip_ctl) & 0x01) << MC34X9_TILT_FEAT);
  }

  if (anym_ctl) {
    MC34X6SetAnym(dev);
    CfgMotion |= ((anym_ctl & 0x01) << MC34X9_ANYM_FEAT);
  }

  if (shake_ctl) {
    MC34X6SetShake(dev);
    // Also enable anyMotion feature
    CfgMotion |= ((shake_ctl & 0x01) << MC34X9_ANYM_FEAT) | ((shake_ctl & 0x01) << MC34X9_SHAKE_FEAT);
  }

  if (tilt_35_ctl) {
    MC34X6SetTilt35(dev);
    // Also enable anyMotion feature
    CfgMotion |= ((tilt_35_ctl & 0x01) << MC34X9_ANYM_FEAT) | ((tilt_35_ctl & 0x01) << MC34X9_TILT35_FEAT);
  }

  MC34X9Write8(MC34X9_REG_MOTION_CTRL, CfgMotion);
}

//Set FIFO feature
void MC34X9SetFIFOCtrl(MC34X9* const dev, const MC34X9FIFOControl fifo_ctl,
                         const MC34X9FIFOMode fifo_mode, const uint8_t fifo_thr)
{
  if (fifo_thr > 31)  //maximum threshold
    fifo_thr = 31;

  MC34X9SetMode(dev, MC34X9_MODE_STANDBY);

  CfgFifo = (MC34X9_COMB_INT_ENABLE << 3) | ((fifo_ctl << 5) | (fifo_mode << 6)) ;

  MC34X9Write8(dev, MC34X9_REG_FIFO_CTRL, CfgFifo);

  uint8_t CfgFifoThr = fifo_thr;
  MC34X9Write8(dev, MC34X9_REG_FIFO_TH, CfgFifoThr);
}

void MC34X9SetGerneralINTCtrl() {
  // Gerneral Interrupt setup
  uint8_t CfgGPIOINT = (((MC34X9_INTR_C_IAH_ACTIVE_LOW & 0x01) << 2) // int1
                        | ((MC34X9_INTR_C_IPP_MODE_OPEN_DRAIN & 0x01) << 3)// int1
                        | ((MC34X9_INTR_C_IAH_ACTIVE_LOW & 0x01) << 6)// int2
                        | ((MC34X9_INTR_C_IPP_MODE_OPEN_DRAIN & 0x01) << 7));// int2

  MC34X9Write8(dev, MC34X9_REG_GPIO_CTRL, CfgGPIOINT);
}

//Set interrupt control register
void MC34X9SetINTCtrl(MC34X9* const devbool tilt_int_ctrl,
                        const bool flip_int_ctl,
                        const bool anym_int_ctl,
                        const bool shake_int_ctl,
                        const bool tilt_35_int_ctl)
{
  MC34X9SetMode(dev, MC34X9_MODE_STANDBY);

  uint8_t CfgINT = (((tilt_int_ctrl & 0x01) << 0)
                    | ((flip_int_ctl & 0x01) << 1)
                    | ((anym_int_ctl & 0x01) << 2)
                    | ((shake_int_ctl & 0x01) << 3)
                    | ((tilt_35_int_ctl & 0x01) << 4)
                    | ((MC34X9_AUTO_CLR_ENABLE & 0x01) << 6));
  MC34X9Write8(dev, MC34X9_REG_INTR_CTRL, CfgINT);

  SetGerneralINTCtrl();
}

//Set FIFO interrupt control register
void MC34X9SetFIFOINTCtrl(MC34X9* const dev, const bool fifo_empty_int_ctl,
                            const bool fifo_full_int_ctl,
                            const bool fifo_thr_int_ctl)
{
  MC34X9SetMode(dev, MC34X9_MODE_STANDBY);

  CfgFifo = CfgFifo
            | (((fifo_empty_int_ctl & 0x01) << 0)
               | ((fifo_full_int_ctl & 0x01) << 1)
               | ((fifo_thr_int_ctl & 0x01) << 2));

  MC34X9Write8(dev, MC34X9_REG_FIFO_CTRL, CfgFifo);

  SetGerneralINTCtrl();
}

//Interrupt handler (clear interrupt flag)
void MC34X9INTHandler(MC34X9* const dev, MC34X9IntEvent* const ptINT_Event)
{
  uint8_t value;

  value = MC34X9Read8(dev, MC34X9_REG_INTR_STAT);

  ptINT_Event->bTILT           = ((value >> 0) & 0x01);
  ptINT_Event->bFLIP           = ((value >> 1) & 0x01);
  ptINT_Event->bANYM           = ((value >> 2) & 0x01);
  ptINT_Event->bSHAKE          = ((value >> 3) & 0x01);
  ptINT_Event->bTILT_35        = ((value >> 4) & 0x01);

  value &= 0x40;
  MC34X9Write8(dev, MC34X9_REG_INTR_STAT, value);
}

//FIFO Interrupt handler (clear interrupt flag)
void MC34X9FIFOINTHandler(MC34X9_fifo_interrupt_event_t *ptFIFO_INT_Event)
{
  uint8_t value;

  value = MC34X9Read8(dev, MC34X9_REG_FIFO_INTR);

  ptFIFO_INT_Event->bFIFO_EMPTY           = ((value >> 0) & 0x01);
  ptFIFO_INT_Event->bFIFO_FULL            = ((value >> 1) & 0x01);
  ptFIFO_INT_Event->bFIFO_THRESH          = ((value >> 2) & 0x01);
}

//Get the range control
MC34X9Range MC34X9GetRangeCtrl(MC34X9* const dev)
{
  // Read the data format register to preserve bits
  uint8_t value;
  value = MC34X9Read8(dev, MC34X9_REG_RANGE_C);
  value &= 0x70;
  return (MC34X9_range_t) (value >> 4);
}

//Get the output sampling rate
MC34X9SetSampleRate MC34X9GetSampleRate(MC34X9* const dev)
{
  // Read the data format register to preserve bits
  uint8_t value;
  value = MC34X9Read8(dev, MC34X9_REG_SR);
  value &= 0b00011111;
  return (MC34X9_sr_t) (value);
}

//Is FIFO empty
bool MC34X9IsFIFOEmpty(MC34X9* const dev)
{
  // Read the data format register to preserve bits
  uint8_t value;
  value = MC34X9Read8(dev, MC34X9_REG_FIFO_STAT);
  value &= 0x01;
  //Serial.println("FIFO_Status");
  //Serial.println(value, HEX);

  if (value ^ 0x01)
    return false;	//Not empty
  else {
    return true;  //Is empty
  }
}

//Read the raw counts and SI units measurement data
MC34X9Accel MC34X9ReadRawAccel(MC34X9* const dev)
{
  //{2g, 4g, 8g, 16g, 12g}
  float faRange[5] = { 19.614f, 39.228f, 78.456f, 156.912f, 117.684f};
  // 16bit
  float faResolution = 32768.0f;

  byte rawData[6];
  // Read the six raw data registers into data array
  mcube_read_regs(M_bSpi, M_chip_select, MC34X9_REG_XOUT_LSB, rawData, 6);
  x = (short)((((unsigned short)rawData[1]) << 8) | rawData[0]);
  y = (short)((((unsigned short)rawData[3]) << 8) | rawData[2]);
  z = (short)((((unsigned short)rawData[5]) << 8) | rawData[4]);

  AccRaw.XAxis = (short) (x);
  AccRaw.YAxis = (short) (y);
  AccRaw.ZAxis = (short) (z);
  AccRaw.XAxis_g = (float) (x) / faResolution * faRange[CfgRange];
  AccRaw.YAxis_g = (float) (y) / faResolution * faRange[CfgRange];
  AccRaw.ZAxis_g = (float) (z) / faResolution * faRange[CfgRange];

  return AccRaw;
}

// Write 8-bit to register
// ***MC34X9 dirver motion part***
void MC34X6SetTFThreshold(MC34X9* const dev, uint16_t threshold) {
  uint8_t _bFTThr[2] = {0};

  _bFTThr[0] = (threshold & 0x00ff);
  _bFTThr[1] = ((threshold & 0x7f00) >> 8 );

  // set threshold
  MC34X9Write8(dev, MC34X9_REG_TF_THRESH_LSB, _bFTThr[0]);
  MC34X9Write8(dev, MC34X9_REG_TF_THRESH_MSB, _bFTThr[1]);
}

void MC34X6SetTFDebounce(uint8_t debounce) {
  // set debounce
  MC34X9Write8(dev, MC34X9_REG_TF_DB, debounce);
}

void MC34X6SetANYMThreshold(uint16_t threshold) {
  uint8_t _bANYMThr[2] = {0};

  _bANYMThr[0] = (threshold & 0x00ff);
  _bANYMThr[1] = ((threshold & 0x7f00) >> 8 );

  // set threshold
  MC34X9Write8(dev, MC34X9_REG_AM_THRESH_LSB, _bANYMThr[0]);
  MC34X9Write8(dev, MC34X9_REG_AM_THRESH_MSB, _bANYMThr[1]);
}

void MC34X6SetANYMDebounce(uint8_t debounce) {
  MC34X9Write8(dev, MC34X9_REG_AM_DB, debounce);
}

void MC34X9SetShakeThreshold(uint16_t threshold) {
  uint8_t _bSHKThr[2] = {0};

  _bSHKThr[0] = (threshold & 0x00ff);
  _bSHKThr[1] = ((threshold & 0xff00) >> 8 );

  // set threshold
  MC34X9Write8(dev, MC34X9_REG_SHK_THRESH_LSB, _bSHKThr[0]);
  MC34X9Write8(dev, MC34X9_REG_SHK_THRESH_MSB, _bSHKThr[1]);
}

void M_DRV_MC34X6_SetShake_P2P_DUR_THRESH(uint16_t threshold, uint8_t shakeCount) {

  uint8_t _bSHKP2PDuration[2] = {0};

  _bSHKP2PDuration[0] = (threshold & 0x00ff);
  _bSHKP2PDuration[1] = ((threshold & 0x0f00) >> 8);
  _bSHKP2PDuration[1] |= ((shakeCount & 0x7) << 4);

  // set peak to peak duration and count
  MC34X9Write8(dev, MC34X9_REG_PK_P2P_DUR_THRESH_LSB, _bSHKP2PDuration[0]);
  MC34X9Write8(dev, MC34X9_REG_PK_P2P_DUR_THRESH_MSB, _bSHKP2PDuration[1]);
}

uint16_t MC34X6SetTILT35Threshold(MC34X9* const dev, uint16_t threshold) {
  MC34X6_SetTFThreshold(threshold);
}

void M_DRV_MC34X6_SetTILT35Timer(MC34X9* const dev, uint8_t timer) {
  uint8_t value;

  value = MC34X9Read8(dev, MC34X9_REG_TIMER_CTRL);
  value &= 0b11111000;
  value |= MC34X9_TILT35_2p0;

  MC34X9Write8(dev, MC34X9_REG_TIMER_CTRL, timer);
}

// Tilt & Flip
void MC34X6SetTiltFlip(MC34X9* const dev) {
  // set threshold
  MC34X6SetTFThreshold(s_bCfgFTThr);
  // set debounce
  M_DRV_MC34X6_SetTFDebounce(s_bCfgFTDebounce);
  return;
}

// AnyMotion
void _M_DRV_MC34X6_SetAnym() {
  // set threshold
  M_DRV_MC34X6_SetANYMThreshold(s_bCfgANYMThr);

  // set debounce
  M_DRV_MC34X6_SetANYMDebounce(s_bCfgANYMDebounce);
  return;
}

// Shake
void _M_DRV_MC34X6_SetShake() {
  // Config anymotion
  _M_DRV_MC34X6_SetAnym();

  // Config shake
  // set threshold
  M_DRV_MC34X6_SetShakeThreshold(s_bCfgShakeThr);

  // set peak to peak duration and count
  M_DRV_MC34X6_SetShake_P2P_DUR_THRESH(s_bCfgShakeP2PDuration, s_bCfgShakeCount);
  return;
}

// Tilt 35
void _M_DRV_MC34X6_SetTilt35() {
  // Config anymotion
  _M_DRV_MC34X6_SetAnym();

  // Config Tilt35
  // set threshold
  M_DRV_MC34X6_SetTILT35Threshold(s_bCfgTILT35Thr);

  //set timer
  M_DRV_MC34X6_SetTILT35Timer(MC34X9_TILT35_2p0);
  return;
}
