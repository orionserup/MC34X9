/******************************************************************************

   Copyright (c) 2020 mCube, Inc.  All rights reserved.

   This source is subject to the mCube Software License.
   This software is protected by Copyright and the information and source code
   contained herein is confidential. The software including the source code
   may not be copied and the information contained herein may not be used or
   disclosed except with the written permission of mCube Inc.

   All other rights reserved.
 *****************************************************************************/

/**
   @file    MC34X9.h
   @author  mCube
   @date    13 January 2020
   @brief   Driver interface header file for accelerometer mc34X9 series.
   @see     http://www.mcubemems.com
*/

#pragma once
#ifndef MC34X9_h
#define MC34X9_h

#include <stdint.h>
#include <stddef.h>

/******************************************************************************
 *** INFORMATION
 *****************************************************************************/
#define M_DRV_MC34X9_VERSION    "1.0.0"

/******************************************************************************
*** Motion threshold and debounce config
 *****************************************************************************/
#define s_bCfgFTThr               100
#define s_bCfgFTDebounce          50

#define s_bCfgANYMThr             200
#define s_bCfgANYMDebounce        100

#define s_bCfgShakeThr            300
#define s_bCfgShakeP2PDuration    10
#define s_bCfgShakeCount          1

#define s_bCfgTILT35Thr           20
#define s_bCfgTILT35Timer         MC34X9_TILT35_2p0

/******************************************************************************
 *** CONSTANT / DEFINE
 *****************************************************************************/
#define MC34X9_RETCODE_SUCCESS                 (0)
#define MC34X9_RETCODE_ERROR_BUS               (-1)
#define MC34X9_RETCODE_ERROR_NULL_POINTER      (-2)
#define MC34X9_RETCODE_ERROR_STATUS            (-3)
#define MC34X9_RETCODE_ERROR_SETUP             (-4)
#define MC34X9_RETCODE_ERROR_GET_DATA          (-5)
#define MC34X9_RETCODE_ERROR_IDENTIFICATION    (-6)
#define MC34X9_RETCODE_ERROR_NO_DATA           (-7)
#define MC34X9_RETCODE_ERROR_WRONG_ARGUMENT    (-8)
#define MC34X9_FIFO_DEPTH                        32
#define MC34X9_REG_MAP_SIZE                      64

/******************************************************************************
 *** CONSTANT / DEFINE
 *****************************************************************************/
#define MC34X9_INTR_C_IPP_MODE_OPEN_DRAIN    (0x00)
#define MC34X9_INTR_C_IPP_MODE_PUSH_PULL     (0x01)

#define MC34X9_INTR_C_IAH_ACTIVE_LOW         (0x00)
#define MC34X9_INTR_C_IAH_ACTIVE_HIGH        (0x01)

#define MC34X9_AUTO_CLR_DISABLE              (0x00)
#define MC34X9_AUTO_CLR_ENABLE               (0x01)
/******************************************************************************
 *** Register Map
 *****************************************************************************/
#define MC34X9_REG_DEV_STAT         (0x05)
#define MC34X9_REG_INTR_CTRL        (0x06)
#define MC34X9_REG_MODE             (0x07)
#define MC34X9_REG_SR               (0x08)
#define MC34X9_REG_MOTION_CTRL      (0x09)
#define MC34X9_REG_FIFO_STAT        (0x0A)
#define MC34X9_REG_FIFO_RD_P        (0x0B)
#define MC34X9_REG_FIFO_WR_P        (0x0C)
#define MC34X9_REG_XOUT_LSB         (0x0D)
#define MC34X9_REG_XOUT_MSB         (0x0E)
#define MC34X9_REG_YOUT_LSB         (0x0F)
#define MC34X9_REG_YOUT_MSB         (0x10)
#define MC34X9_REG_ZOUT_LSB         (0x11)
#define MC34X9_REG_ZOUT_MSB         (0x12)
#define MC34X9_REG_STATUS           (0x13)
#define MC34X9_REG_INTR_STAT        (0x14)
#define MC34X9_REG_PROD             (0x18)
#define MC34X9_REG_RANGE_C          (0x20)
#define MC34X9_REG_XOFFL            (0x21)
#define MC34X9_REG_XOFFH            (0x22)
#define MC34X9_REG_YOFFL            (0x23)
#define MC34X9_REG_YOFFH            (0x24)
#define MC34X9_REG_ZOFFL            (0x25)
#define MC34X9_REG_ZOFFH            (0x26)
#define MC34X9_REG_XGAIN            (0x27)
#define MC34X9_REG_YGAIN            (0x28)
#define MC34X9_REG_ZGAIN            (0x29)
#define MC34X9_REG_FIFO_CTRL        (0x2D)
#define MC34X9_REG_FIFO_TH          (0x2E)
#define MC34X9_REG_FIFO_INTR        (0x2F)
#define MC34X9_REG_FIFO_CTRL_SR2    (0x30)
#define MC34X9_REG_COMM_CTRL        (0x31)
#define MC34X9_REG_GPIO_CTRL        (0x33)
#define MC34X9_REG_TF_THRESH_LSB    (0x40)
#define MC34X9_REG_TF_THRESH_MSB    (0x41)
#define MC34X9_REG_TF_DB            (0x42)
#define MC34X9_REG_AM_THRESH_LSB    (0x43)
#define MC34X9_REG_AM_THRESH_MSB    (0x44)
#define MC34X9_REG_AM_DB            (0x45)
#define MC34X9_REG_SHK_THRESH_LSB   (0x46)
#define MC34X9_REG_SHK_THRESH_MSB   (0x47)
#define MC34X9_REG_PK_P2P_DUR_THRESH_LSB    (0x48)
#define MC34X9_REG_PK_P2P_DUR_THRESH_MSB    (0x49)
#define MC34X9_REG_TIMER_CTRL       (0x4A)

#define MC34X9_NULL_ADDR            (0)

#define MC34X9_CHIP_ID (0xA4)

#define s_bCfgFTThr 200
#define s_bCfgFTDebounce 50

typedef struct MC34X9ACCEL
{
  short XAxis;
  short YAxis;
  short ZAxis;
  float XAxis_g;
  float YAxis_g;
  float ZAxis_g;
} MC34X9Accel;

typedef enum MC34X9GAIN
{
  MC34X9_GAIN_5_24X      = 0b0011,
  MC34X9_GAIN_3_89X      = 0b0010,
  MC34X9_GAIN_DEFAULT_1X = 0b0000,
  MC34X9_GAIN_0_5X       = 0b0100,
  MC34X9_GAIN_0_33X      = 0b1100,
}   MC34X9Gain;

typedef enum MC34X9MODE
{
  MC34X9_MODE_SLEEP    = 0b000,
  MC34X9_MODE_CWAKE      = 0b001,
  MC34X9_MODE_RESERVED   = 0b010,
  MC34X9_MODE_STANDBY  = 0b011,
}   MC34X9Mode;

typedef enum MC34X9RANGE
{
  MC34X9_RANGE_2G    = 0b000,
  MC34X9_RANGE_4G    = 0b001,
  MC34X9_RANGE_8G    = 0b010,
  MC34X9_RANGE_16G   = 0b011,
  MC34X9_RANGE_12G   = 0b100,
  MC34X9_RANGE_END,
}   MC34X9Range;

typedef enum MC34X9SAMPLERATE
{
  MC34X9_SR_25Hz            = 0x10,
  MC34X9_SR_50Hz            = 0x11,
  MC34X9_SR_62_5Hz          = 0x12,
  MC34X9_SR_100Hz           = 0x13,
  MC34X9_SR_125Hz           = 0x14,
  MC34X9_SR_250Hz           = 0x15,
  MC34X9_SR_500Hz           = 0x16,
  MC34X9_SR_DEFAULT_1000Hz  = 0x17,
  MC34X9_SR_END,
}   MC34X9SampleRate;

typedef enum MC34X9MOTIONFEATURE
{
  MC34X9_TILT_FEAT = 0,
  MC34X9_ANYM_FEAT = 2,
  MC34X9_SHAKE_FEAT = 3,
  MC34X9_TILT35_FEAT = 4,
}   MC34X9MotionFeature;

typedef enum MC34X9FIFOCONTROL
{
  MC34X9_FIFO_CTL_DISABLE = 0,
  MC34X9_FIFO_CTL_ENABLE,
  MC34X9_FIFO_CTL_END,
}   MC34X9FIFOControl;

typedef enum MC34X9FIFOMODE
{
  MC34X9_FIFO_MODE_NORMAL = 0,
  MC34X9_FIFO_MODE_WATERMARK,
  MC34X9_FIFO_MODE_END,
}   MC34X9FIFOMode;

typedef enum MC34X9FIFOInt
{
  MC34X9_COMB_INT_DISABLE = 0,
  MC34X9_COMB_INT_ENABLE,
  MC34X9_COMB_INT_END,
}   MC34X9FIFOInt;

typedef struct MC34X9INTEVENT
{
  uint8_t    bTILT;
  uint8_t    bFLIP;
  uint8_t    bANYM;
  uint8_t    bSHAKE;
  uint8_t    bTILT_35;
  uint8_t    bRESV;
  uint8_t 	 bAUTO_CLR;
  uint8_t    bACQ;
}   MC34X9IntEvent;

typedef struct MC34X9FIFOIntEvent
{
  uint8_t    bFIFO_EMPTY;
  uint8_t    bFIFO_FULL;
  uint8_t    bFIFO_THRESH;
}   MC34X9FIFOIntEvent;

// ***MC34X9 dirver motion part***
typedef enum MC34X9TIMER35TILTDURATION
{
  MC34X9_TILT35_1p6           = 0b000,
  MC34X9_TILT35_1p8           = 0b001,
  MC34X9_TILT35_2p0           = 0b010,  
  MC34X9_TILT35_2p2           = 0b011,
  MC34X9_TILT35_2p4           = 0b100,
  MC34X9_TILT35_2p6           = 0b101,
  MC34X9_TILT35_2p8           = 0b110,
  MC34X9_TILT35_3p0           = 0b111
} MC34X9Tilt35TimerDuration;

typedef enum MC34X9Type {

  MC3419

} MC34X9Type;

typedef struct MC34X9CONFIG {

  uint16_t (*read_reg)(const uint8_t s_addr, const uint8_t reg_addr, void* const buffer, const uint16_t size);
  uint16_t (*write_reg)(const uint8_t s_addr, const uint8_t reg_addr, const void* const buffer, const uint16_t size);

  bool address_6;
  MC34X9Type type;

} MC34X9Config;

typedef struct MC34X9 {

  MC34X9Config config;
  uint8_t address;
  MC34X9Type type;
  MC34X9Accel acceleration;
  MC34X9Mode mode;

} MC34X9;

/**
 * \brief 
 * 
 * \param dev
 * \param config
 * \return MC34X9* 
 */
MC34X9* MC34X9Init(MC34X9* const dev, const MC34X9Config* const config);

/**
 * \brief 
 * 
 * \param dev
 * \param address
 * \param data
 * \return uint16_t 
 */
uint16_t MC34X9Write8(MC34X9* const dev, const uint8_t address, const uint8_t data);

/**
 * \brief 
 * 
 * \param dev
 * \param address
 * \return uint8_t 
 */
uint8_t MC34X9Read8(MC34X9* const dev, const uint8_t address);

/**
 * \brief 
 * 
 * \param dev
 * \return uint16_t 
 */
uint16_t MC34X9Start(MC34X9* const dev);

/**
 * \brief 
 * 
 * \param dev
 * \return uint16_t 
 */
uint16_t MC34X9Wake(MC34X9* const dev);

/**
 * \brief 
 * 
 * \param dev
 * \return uint16_t 
 */
uint16_t MC34X9Stop(MC34X9* const dev);

/**
 * \brief 
 * 
 * \param dev
 * \return uint16_t 
 */
uint16_t MC34X9Reset(MC34X9* const dev);

/**
 * \brief 
 * 
 * \param dev
 * \param mode
 * \return uint16_t 
 */
uint16_t MC3479SetMode(MC34X9* const dev, MC34X9Mode mode);

/**
 * \brief 
 * 
 * \param dev
 * \param range
 * \return uint16_t 
 */
uint16_t MC34X9SetRangeCtrl(MC34X9* const dev, MC34X9Range range);

/**
 * \brief 
 * 
 * \param dev
 * \param sample_rate
 * \return uint16_t 
 */
uint16_t MC34X9SetSampleRate(MC34X9* const dev, MC34X9SampleRate sample_rate);

/**
 * \brief 
 * 
 * \param dev
 * \param fifo_ctl
 * \param fifo_mode
 * \param fifo_thr
 * \return uint16_t 
 */
uint16_t MC34X9SetFIFOCtrl(MC34X9* const dev, MC34X9FIFOControl fifo_ctl, MC34X9FIFOMode fifo_mode, uint8_t fifo_thr);

/**
 * \brief 
 * 
 * \param dev
 * \param tilt_int_ctrl
 * \param flip_int_ctl
 * \param anym_int_ctl
 * \param shake_int_ctl
 * \param tilt_35_int_ctl
 * \return uint16_t 
 */
uint16_t MC34X9SetMotionCtrl(MC34X9* const dev, const bool tilt_int_ctrl, const bool flip_int_ctl,
                        const bool anym_int_ctl, const bool shake_int_ctl,
                        const bool tilt_35_int_ctl);

/**
 * \brief 
 * 
 * \param dev
 * \param tilt_int_ctrl
 * \param flip_int_ctl
 * \param anym_int_ctl
 * \param shake_int_ctl
 * \param tilt_35_int_ctl
 * \return uint16_t 
 */
uint16_t MC34X9SetINTCtrl(MC34X9* const dev, const bool tilt_int_ctrl, const bool flip_int_ctl,
                              const bool anym_int_ctl, const bool shake_int_ctl,
                              const bool tilt_35_int_ctl);

/**
 * \brief 
 * 
 * \param dev
 * \param fifo_empty_int_ctl
 * \param fifo_full_int_ctl
 * \param fifo_thr_int_ctl
 * \return uint16_t 
 */
uint16_t MCSetFIFOINTCtrl(MC34X9* const dev, const bool fifo_empty_int_ctl, const bool fifo_full_int_ctl, const bool fifo_thr_int_ctl);

/**
 * \brief 
 * 
 * \param dev
 * \return uint16_t 
 */
uint16_t MC34X9SetGerneralINTCtrl(MC34X9* const dev);

/**
 * \brief 
 * 
 * \param dev
 * \param ptINT_Event
 * \return uint16_t 
 */
uint16_t MC34X9INTHandler(MC34X9* const dev, MC34X9IntEvent* const ptINT_Event);

/**
 * \brief 
 * 
 * \param dev
 * \param ptFIFO_INT_Event
 * \return uint16_t 
 */
uint16_t MC34X9FIFOINTHandler(MC34X9* const dev, MC34X9FIFOIntEvent* const ptFIFO_INT_Event);

/**
 * \brief Get the Range Ctrl object
 * 
 * \param dev
 * \return MC34X9Range 
 */
MC34X9Range MC34X9GetRangeCtrl(MC34X9* const dev);

/**
 * \brief Get the Sample Rate object
 * 
 * \param dev
 * \return MC34X9SampleRate 
 */
MC34X9SampleRate MC34X9GetSampleRate(MC34X9* const dev);

/**
 * \brief 
 * 
 * \param dev
 * \return true 
 * \return false 
 */
bool MC34X9IsFIFOEmpty(MC34X9* const dev);

/**
 * \brief 
 * 
 * \param dev
 * \return MC34X9Accel 
 */
MC34X9Accel MC34X9ReadRawAccel(MC34X9* const dev);

/**
 * \brief 
 * 
 * \param threshold
 */
uint16_t MC34X6SetTFThreshold(MC34X9* const dev, const uint16_t threshold);

/**
 * \brief 
 * 
 * \param dev
 * \param debounce
 */
uint16_t MC34X6SetTFDebounce(MC34X9* const dev, const uint8_t debounce);

/**
 * \brief 
 * 
 * \param dev
 * \param threshold
 */
uint16_t MC34X6SetANYMThreshold(MC34X9* const dev, const uint16_t threshold);

/**
 * \brief 
 * 
 * \param dev
 * \param debounce
 */
uint16_t MC34X6SetANYMDebounce(MC34X9* const dev, const uint8_t debounce);

/**
 * \brief 
 * 
 * \param dev
 * \param threshold
 */
uint16_t MC34X6SetShakeThreshold(MC34X9* const dev, const uint16_t threshold);

/**
 * \brief 
 * 
 * \param dev
 * \param threshold
 * \param shakeCount
 */
uint16_t MC34X6SetShakeP2PDurThresh(MC34X9* const dev, const uint16_t threshold, const uint8_t shakeCount);

/**
 * \brief 
 * 
 * \param dev
 * \param threshold
 */
uint16_t MC34X6SetTilt35Threshold(MC34X9* const dev, const uint16_t threshold);

/**
 * \brief 
 * 
 * \param dev
 * \param timer
 */
uint16_t MC34X6SetTILT35Timer(MC34X9* const dev, const uint8_t timer);

/**
 * \brief 
 * 
 * \param dev
 */
uint16_t MC34X6SetTiltFlip(MC34X9* const dev);

/**
 * \brief 
 * 
 * \param dev
 */
uint16_t MC34X6SetAnym(MC34X9* const dev);

/**
 * \brief 
 * 
 * \param dev
 */
uint16_t MC34X6SetShake(MC34X9* const dev);

/**
 * \brief 
 * 
 * \param dev
 */
uint16_t MC34X6SetTilt35(MC34X9* const dev);

#endif
