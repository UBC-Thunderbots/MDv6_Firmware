/**
 ******************************************************************************
 * @file    interface.h
 * @author  UBC Thunderbots
 * @brief   This file contains all definitions and functions prototypes for the
 *          SPI interface module.
 ******************************************************************************
 */

#ifndef __INTERFACE_H
#define __INTERFACE_H

#define MESSAGE_SIZE 8
#define MESSAGE_DELIMITER 0xAA

typedef enum {
  NO_OP                        = 0x00,
  SET_TARGET_SPEED             = 0x01,
  SET_TARGET_TORQUE            = 0x02,
  SET_RESPONSE_TYPE            = 0x03,
  SET_PID_TORQUE_KP_KI         = 0x04,
  SET_PID_FLUX_KP_KI           = 0x05,
  SET_PID_SPEED_KP_KI          = 0x06,
  SET_SPEED_FEED_FORWARD_KA_KV = 0x07,
  SET_SPEED_FEED_FORWARD_KS    = 0x08,
} Opcode_t;

typedef enum {
  SPEED_AND_FAULTS          = 0x01,
  IQ_AND_ID                 = 0x02,
  VQ_AND_VD                 = 0x03,
  PHASE_CURRENT_AND_VOLTAGE = 0x04,
  IQ_AND_IQ_REF             = 0x05,
  ID_AND_ID_REF             = 0x06,
  SPEED_AND_SPEED_REF       = 0x07,
} ResponseType_t;

/**
 * For documentation on fault codes, visit ST MC SDK v6.2.0 documentation page
 * /group___m_c___type.html#fault_codes
 */
typedef enum {
  NO_FAULT     = 0x0000,
  DURATION     = 0x0001,
  OVER_VOLT    = 0x0002,
  UNDER_VOLT   = 0x0004,
  OVER_TEMP    = 0x0008,
  START_UP     = 0x0010,
  SPEED_FDBK   = 0x0020,
  OVER_CURR    = 0x0040,
  SW_ERROR     = 0x0080,
  SAMPLE_FAULT = 0x0100,
  OVERCURR_SW  = 0x0200,
  DP_FAULT     = 0x0400,
} FaultCode_t;

void Interface_Init(void);
void Interface_Loop(void);

#endif /* __INTERFACE_H */