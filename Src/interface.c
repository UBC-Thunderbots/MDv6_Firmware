/**
 ******************************************************************************
 * @file    interface.c
 * @author  UBC Thunderbots
 * @brief   SPI interface for motor control commands and telemetry
 * 
 * This module implements a simple SPI protocol for controlling the motor.
 * The master can send commands to set the target speed or torque, configure 
 * parameters, and select the type of telemetry response. The slave (this 
 * firmware) processes the commands and responds with telemetry data.
 ******************************************************************************
 */

#include "interface.h"

#include "stm32f0xx_hal.h"
#include "motorcontrol.h"
#include "speed_feed_forward_ctrl.h"
#include "crc.h"

#include <stdbool.h>
#include <string.h>

/* External variables --------------------------------------------------------*/

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

extern PID_Handle_t PIDSpeedHandle_M1;
extern PID_Handle_t PIDIqHandle_M1;
extern PID_Handle_t PIDIdHandle_M1;

/* Protocol buffering -------------------------------------------------------*/

#define RX_STREAM_CAPACITY (MESSAGE_SIZE * 4U)
#define TX_QUEUE_CAPACITY 4U

static uint8_t rx_stream[RX_STREAM_CAPACITY];
static size_t rx_stream_length;

static uint8_t tx_queue[TX_QUEUE_CAPACITY][MESSAGE_SIZE];
static size_t tx_queue_head;
static size_t tx_queue_tail;
static size_t tx_queue_count;

/* Private variables ---------------------------------------------------------*/

uint8_t rx_bufs[2][MESSAGE_SIZE];
uint8_t tx_bufs[2][MESSAGE_SIZE];

volatile bool txrx_half_completed;
volatile bool txrx_completed;

ResponseType_t response_type;

bool motor_enabled;

/* Private function prototypes -----------------------------------------------*/

static void ResetProtocolBuffers(void);
static void AppendRxBytes(const uint8_t *bytes, size_t length);
static bool PopNextFrame(uint8_t *frame);
static void QueueTxFrame(const uint8_t *frame);
static bool DequeueTxFrame(uint8_t *frame);
static void PopulateTxHeader(uint8_t *tx, uint8_t seq, ResponseType_t type);
static void PopulateTx_ResponseForCurrentType(uint8_t *tx, uint8_t seq);
static void ProcessSpiFrame(const uint8_t *rx);
static void HandleCompletedDmaHalf(uint8_t half_index);

void ProcessRx_SetTargetSpeed(const uint8_t *rx);
void ProcessRx_SetTargetTorque(const uint8_t *rx);
void ProcessRx_SetResponseType(const uint8_t *rx);
void ProcessRx_SetPidTorqueKpKi(const uint8_t *rx);
void ProcessRx_SetPidFluxKpKi(const uint8_t *rx);
void ProcessRx_SetPidSpeedKpKi(const uint8_t *rx);
void ProcessRx_SetSpeedFeedForwardKaKv(const uint8_t *rx);
void ProcessRx_SetSpeedFeedForwardKs(const uint8_t *rx);

void PopulateTx_ResponseSpeedAndFaults(uint8_t *tx, uint8_t seq);
void PopulateTx_ResponseIqAndId(uint8_t *tx, uint8_t seq);
void PopulateTx_ResponseVqAndVd(uint8_t *tx, uint8_t seq);
void PopulateTx_ResponsePhaseCurrentAndVoltage(uint8_t *tx, uint8_t seq);
void PopulateTx_ResponseIqAndIqRef(uint8_t *tx, uint8_t seq);
void PopulateTx_ResponseIdAndIdRef(uint8_t *tx, uint8_t seq);
void PopulateTx_ResponseSpeedAndSpeedRef(uint8_t *tx, uint8_t seq);

void SetMotorEnabled(bool enabled);

/* Function implementations --------------------------------------------------*/

/**
 * Initializes the SPI interface module.
 */
void Interface_Init(void)
{
  ResetProtocolBuffers();

  memset(rx_bufs, 0, sizeof(rx_bufs));
  memset(tx_bufs, 0, sizeof(tx_bufs));

  txrx_half_completed = false;
  txrx_completed = false;

  response_type = SPEED_AND_FAULTS;

  motor_enabled = false;

  PopulateTx_ResponseForCurrentType(tx_bufs[0], 0);
  PopulateTx_ResponseForCurrentType(tx_bufs[1], 0);
}

/**
 * Main loop for the SPI interface module. 
 * Continuously waits for SPI transactions to complete and processes them.
 */
void Interface_Loop(void)
{
  HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)tx_bufs, (uint8_t *)rx_bufs, MESSAGE_SIZE * 2);

  while (1) 
  {		
    if (txrx_half_completed) 
    {
      HandleCompletedDmaHalf(0);
      txrx_half_completed = false;
    }
    if (txrx_completed) 
    {
      HandleCompletedDmaHalf(1);
      txrx_completed = false;
    }
  }
}

/**
 * Resets the protocol stream and response queue.
 */
static void ResetProtocolBuffers(void)
{
  memset(rx_stream, 0, sizeof(rx_stream));
  rx_stream_length = 0;

  memset(tx_queue, 0, sizeof(tx_queue));
  tx_queue_head = 0;
  tx_queue_tail = 0;
  tx_queue_count = 0;
}

/**
 * Appends the given bytes to the sliding receive window.
 */
static void AppendRxBytes(const uint8_t *bytes, size_t length)
{
  if (length > RX_STREAM_CAPACITY)
  {
    bytes += length - RX_STREAM_CAPACITY;
    length = RX_STREAM_CAPACITY;
  }

  if (rx_stream_length + length > RX_STREAM_CAPACITY)
  {
    const size_t bytes_to_discard = rx_stream_length + length - RX_STREAM_CAPACITY;
    memmove(rx_stream, rx_stream + bytes_to_discard, rx_stream_length - bytes_to_discard);
    rx_stream_length -= bytes_to_discard;
  }

  memcpy(rx_stream + rx_stream_length, bytes, length);
  rx_stream_length += length;
}

/**
 * Extracts the next valid SPI frame from the sliding receive window.
 */
static bool PopNextFrame(uint8_t *frame)
{
  while (rx_stream_length >= MESSAGE_SIZE)
  {
    const uint8_t *delimiter = memchr(rx_stream, MESSAGE_DELIMITER, rx_stream_length);

    if (delimiter == NULL)
    {
      rx_stream_length = 0;
      return false;
    }

    const size_t delimiter_offset = (size_t)(delimiter - rx_stream);

    if (delimiter_offset > 0)
    {
      memmove(rx_stream, delimiter, rx_stream_length - delimiter_offset);
      rx_stream_length -= delimiter_offset;
      continue;
    }

    if (rx_stream_length < MESSAGE_SIZE)
    {
      return false;
    }

    if (rx_stream[MESSAGE_SIZE - 1] == CrcGenerateChecksum(rx_stream, MESSAGE_SIZE - 1))
    {
      memcpy(frame, rx_stream, MESSAGE_SIZE);
      memmove(rx_stream, rx_stream + MESSAGE_SIZE, rx_stream_length - MESSAGE_SIZE);
      rx_stream_length -= MESSAGE_SIZE;
      return true;
    }

    memmove(rx_stream, rx_stream + 1, rx_stream_length - 1);
    rx_stream_length--;
  }

  return false;
}

/**
 * Stores a response frame so it can be loaded into the next available TX DMA half.
 */
static void QueueTxFrame(const uint8_t *frame)
{
  memcpy(tx_queue[tx_queue_head], frame, MESSAGE_SIZE);
  tx_queue_head = (tx_queue_head + 1U) % TX_QUEUE_CAPACITY;

  if (tx_queue_count == TX_QUEUE_CAPACITY)
  {
    tx_queue_tail = (tx_queue_tail + 1U) % TX_QUEUE_CAPACITY;
  }
  else
  {
    tx_queue_count++;
  }
}

/**
 * Retrieves the oldest queued response frame.
 */
static bool DequeueTxFrame(uint8_t *frame)
{
  if (tx_queue_count == 0U)
  {
    return false;
  }

  memcpy(frame, tx_queue[tx_queue_tail], MESSAGE_SIZE);
  tx_queue_tail = (tx_queue_tail + 1U) % TX_QUEUE_CAPACITY;
  tx_queue_count--;
  return true;
}

/**
 * Populates the common prefix for a transmit frame.
 */
static void PopulateTxHeader(uint8_t *tx, uint8_t seq, ResponseType_t type)
{
  memset(tx, 0, MESSAGE_SIZE);
  tx[0] = MESSAGE_DELIMITER;
  tx[1] = seq;
  tx[2] = (uint8_t)type;
}

/**
 * Populates a response frame using the currently selected response type.
 */
static void PopulateTx_ResponseForCurrentType(uint8_t *tx, uint8_t seq)
{
  switch (response_type)
  {
    case SPEED_AND_FAULTS:
      PopulateTx_ResponseSpeedAndFaults(tx, seq);
      break;
    case IQ_AND_ID:
      PopulateTx_ResponseIqAndId(tx, seq);
      break;
    case VQ_AND_VD:
      PopulateTx_ResponseVqAndVd(tx, seq);
      break;
    case PHASE_CURRENT_AND_VOLTAGE:
      PopulateTx_ResponsePhaseCurrentAndVoltage(tx, seq);
      break;
    case IQ_AND_IQ_REF:
      PopulateTx_ResponseIqAndIqRef(tx, seq);
      break;
    case ID_AND_ID_REF:
      PopulateTx_ResponseIdAndIdRef(tx, seq);
      break;
    case SPEED_AND_SPEED_REF:
      PopulateTx_ResponseSpeedAndSpeedRef(tx, seq);
      break;
  }
}

/**
 * Processes a complete SPI frame by executing the received command and queuing
 * the matching response frame.
 */
static void ProcessSpiFrame(const uint8_t *rx)
{
  if (rx[0] != MESSAGE_DELIMITER)
  {
    return;
  }

  switch ((Opcode_t)rx[2]) 
  {
    case NO_OP:
      break;
    case SET_TARGET_SPEED:
      ProcessRx_SetTargetSpeed(rx);
      break;
    case SET_TARGET_TORQUE:
      ProcessRx_SetTargetTorque(rx);
      break;
    case SET_RESPONSE_TYPE:
      ProcessRx_SetResponseType(rx);
      break;
    case SET_PID_TORQUE_KP_KI:
      ProcessRx_SetPidTorqueKpKi(rx);
      break;
    case SET_PID_FLUX_KP_KI:
      ProcessRx_SetPidFluxKpKi(rx);
      break;
    case SET_PID_SPEED_KP_KI:
      ProcessRx_SetPidSpeedKpKi(rx);
      break;
    case SET_SPEED_FEED_FORWARD_KA_KV:
      ProcessRx_SetSpeedFeedForwardKaKv(rx);
      break;
    case SET_SPEED_FEED_FORWARD_KS:
      ProcessRx_SetSpeedFeedForwardKs(rx);
      break;
  }

  uint8_t tx[MESSAGE_SIZE];
  PopulateTx_ResponseForCurrentType(tx, rx[1]);
  QueueTxFrame(tx);
}

/**
 * Handles one completed DMA half-buffer by parsing any valid frames found in
 * the incoming byte stream and loading the next TX frame into the completed half.
 */
static void HandleCompletedDmaHalf(uint8_t half_index)
{
  AppendRxBytes(rx_bufs[half_index], MESSAGE_SIZE);

  uint8_t frame[MESSAGE_SIZE];
  while (PopNextFrame(frame))
  {
    ProcessSpiFrame(frame);
  }

  if (!DequeueTxFrame(tx_bufs[half_index]))
  {
    PopulateTx_ResponseForCurrentType(tx_bufs[half_index], 0);
  }
}

/**
 * Processes the received SPI frame to update the target speed for 
 * the motor and start/stop the motor based on the enable flag.
 *
 *  +--------+---------------+-------+
 *  | Enable |     Speed     |       |
 *  +--------+---------------+-------+
 *      1        2       3       4    
 */
void ProcessRx_SetTargetSpeed(const uint8_t *rx)
{
  const int16_t motor_target_speed = (int16_t)(((uint16_t)rx[4] << 8) | rx[5]);
  MC_ProgramSpeedRampMotor1(motor_target_speed, 0);
  SetMotorEnabled(rx[3] != 0);
}

/**
 * Processes the received SPI frame to update the target torque for 
 * the motor and start/stop the motor based on the enable flag.
 *
 *  +--------+---------------+-------+
 *  | Enable |    Torque     |       |
 *  +--------+---------------+-------+
 *      1        2       3       4    
 */
void ProcessRx_SetTargetTorque(const uint8_t *rx) 
{
  const int16_t motor_target_torque = (int16_t)(((uint16_t)rx[4] << 8) | rx[5]);
  MC_ProgramTorqueRampMotor1(motor_target_torque, 0);
  SetMotorEnabled(rx[3] != 0);
}

/**
 * Processes the received SPI frame to update the type of response 
 * that will be sent back to the master in subsequent transactions.
 *
 *  +--------+-----------------------+
 *  |  Type  |                       |
 *  +--------+-----------------------+
 *      1         2      3      4     
 */
void ProcessRx_SetResponseType(const uint8_t *rx) 
{
  response_type = (ResponseType_t)rx[3];
}

/**
 * Processes the received SPI frame to update the Kp and Ki 
 * parameters for the torque (Iq) PID controller.
 *
 *  +----------------+---------------+
 *  |       Kp       |       Ki      |
 *  +----------------+---------------+
 *      1       2        3       4    
 */
void ProcessRx_SetPidTorqueKpKi(const uint8_t *rx) 
{
  const int16_t kp = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);
  const int16_t ki = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);

  PID_SetKP(&PIDIqHandle_M1, kp);
  PID_SetKI(&PIDIqHandle_M1, ki);
}

/**
 * Processes the received SPI frame to update the Kp and Ki 
 * parameters for the flux (Id) PID controller.
 *
 *  +----------------+---------------+
 *  |       Kp       |       Ki      |
 *  +----------------+---------------+
 *      1       2        3       4    
 */
void ProcessRx_SetPidFluxKpKi(const uint8_t *rx) 
{
  const int16_t kp = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);
  const int16_t ki = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);
  
  PID_SetKP(&PIDIdHandle_M1, kp);
  PID_SetKI(&PIDIdHandle_M1, ki);
}

/**
 * Processes the received SPI frame to update the Kp and Ki 
 * parameters for the speed PID controller.
 *
 *  +----------------+---------------+
 *  |       Kp       |       Ki      |
 *  +----------------+---------------+
 *      1       2        3       4    
 */
void ProcessRx_SetPidSpeedKpKi(const uint8_t *rx) 
{
  const int16_t kp = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);
  const int16_t ki = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);
  
  PID_SetKP(&PIDSpeedHandle_M1, kp);
  PID_SetKI(&PIDSpeedHandle_M1, ki);
}

/**
 * Processes the received SPI frame to update the Ka and Kv 
 * parameters for the speed feedforward controller.
 *
 *  +----------------+---------------+
 *  |       Ka       |       Kv      |
 *  +----------------+---------------+
 *      1       2        3       4    
 */
void ProcessRx_SetSpeedFeedForwardKaKv(const uint8_t *rx) 
{
  const int16_t ka = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);
  const int16_t kv = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);

  SpeedFF_SetKaGain(&SpeedFF_M1, (int32_t)ka);
  SpeedFF_SetKvGain(&SpeedFF_M1, (int32_t)kv);
}

/**
 * Processes the received SPI frame to update the Ks 
 * parameter for the speed feedforward controller.
 *
 *  +----------------+---------------+
 *  |       Ks       |               |
 *  +----------------+---------------+
 *      1       2        3       4    
 */
void ProcessRx_SetSpeedFeedForwardKs(const uint8_t *rx) 
{
  const int16_t ks = (int16_t)(((uint16_t)rx[3] << 8) | rx[4]);

  SpeedFF_SetKsGain(&SpeedFF_M1, (int32_t)ks);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current measured speed and motor faults.
 *
 *  +---------------+----------------+
 *  |  Speed (RPM)  |     Faults     |
 *  +---------------+----------------+
 *      1       2        3      4     
 */
void PopulateTx_ResponseSpeedAndFaults(uint8_t *tx, uint8_t seq) 
{
  const int16_t motor_current_speed = MC_GetMecSpeedAverageMotor1();
  const int16_t motor_faults = MC_GetOccurredFaultsMotor1();

  PopulateTxHeader(tx, seq, SPEED_AND_FAULTS);
  tx[3] = (uint8_t)((motor_current_speed >> 8) & 0xFF);
  tx[4] = (uint8_t)(motor_current_speed & 0xFF);
  tx[5] = (uint8_t)((motor_faults >> 8) & 0xFF);
  tx[6] = (uint8_t)(motor_faults & 0xFF);
  tx[7] = CrcGenerateChecksum(tx, MESSAGE_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current Iq and Id values for the motor.
 *
 *  +---------------+----------------+
 *  |      Iq       |       Id       |
 *  +---------------+----------------+
 *      1       2        3      4     
 */
void PopulateTx_ResponseIqAndId(uint8_t *tx, uint8_t seq) 
{
  const qd_t motor_iqd = MC_GetIqdMotor1();

  PopulateTxHeader(tx, seq, IQ_AND_ID);
  tx[3] = (uint8_t)((motor_iqd.q >> 8) & 0xFF);
  tx[4] = (uint8_t)(motor_iqd.q & 0xFF);
  tx[5] = (uint8_t)((motor_iqd.d >> 8) & 0xFF);
  tx[6] = (uint8_t)(motor_iqd.d & 0xFF);
  tx[7] = CrcGenerateChecksum(tx, MESSAGE_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current Vq and Vd values for the motor.
 *
 *  +---------------+----------------+
 *  |      Vq       |       Vd       |
 *  +---------------+----------------+
 *      1       2        3      4     
 */
void PopulateTx_ResponseVqAndVd(uint8_t *tx, uint8_t seq) 
{
  const qd_t motor_vqd = MC_GetVqdMotor1();

  PopulateTxHeader(tx, seq, VQ_AND_VD);
  tx[3] = (uint8_t)((motor_vqd.q >> 8) & 0xFF);
  tx[4] = (uint8_t)(motor_vqd.q & 0xFF);
  tx[5] = (uint8_t)((motor_vqd.d >> 8) & 0xFF);
  tx[6] = (uint8_t)(motor_vqd.d & 0xFF);
  tx[7] = CrcGenerateChecksum(tx, MESSAGE_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * phase current and voltage amplitudes for the motor.
 *
 *  +---------------+----------------+
 *  | Phase Current |  Phase Voltage |
 *  +---------------+----------------+
 *      1       2        3      4     
 */
void PopulateTx_ResponsePhaseCurrentAndVoltage(uint8_t *tx, uint8_t seq) 
{
  const int16_t motor_phase_current = MC_GetPhaseCurrentAmplitudeMotor1();
  const int16_t motor_phase_voltage = MC_GetPhaseVoltageAmplitudeMotor1();

  PopulateTxHeader(tx, seq, PHASE_CURRENT_AND_VOLTAGE);
  tx[3] = (uint8_t)((motor_phase_current >> 8) & 0xFF);
  tx[4] = (uint8_t)(motor_phase_current & 0xFF);
  tx[5] = (uint8_t)((motor_phase_voltage >> 8) & 0xFF);
  tx[6] = (uint8_t)(motor_phase_voltage & 0xFF);
  tx[7] = CrcGenerateChecksum(tx, MESSAGE_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current Iq and reference Iq values for the motor.
 *
 *  +---------------+----------------+
 *  |      Iq       |     Iq Ref     |
 *  +---------------+----------------+
 *      1       2        3      4     
 */
void PopulateTx_ResponseIqAndIqRef(uint8_t *tx, uint8_t seq) 
{
  const qd_t motor_iqd = MC_GetIqdMotor1();
  const qd_t motor_iqd_ref = MC_GetIqdrefMotor1();

  PopulateTxHeader(tx, seq, IQ_AND_IQ_REF);
  tx[3] = (uint8_t)((motor_iqd.q >> 8) & 0xFF);
  tx[4] = (uint8_t)(motor_iqd.q & 0xFF);
  tx[5] = (uint8_t)((motor_iqd_ref.q >> 8) & 0xFF);
  tx[6] = (uint8_t)(motor_iqd_ref.q & 0xFF);
  tx[7] = CrcGenerateChecksum(tx, MESSAGE_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current Id and reference Id values for the motor.
 *
 *  +---------------+----------------+
 *  |      Id       |     Id Ref     |
 *  +---------------+----------------+
 *      1       2        3      4     
 */
void PopulateTx_ResponseIdAndIdRef(uint8_t *tx, uint8_t seq) 
{
  const qd_t motor_iqd = MC_GetIqdMotor1();
  const qd_t motor_iqd_ref = MC_GetIqdrefMotor1();

  PopulateTxHeader(tx, seq, ID_AND_ID_REF);
  tx[3] = (uint8_t)((motor_iqd.d >> 8) & 0xFF);
  tx[4] = (uint8_t)(motor_iqd.d & 0xFF);
  tx[5] = (uint8_t)((motor_iqd_ref.d >> 8) & 0xFF);
  tx[6] = (uint8_t)(motor_iqd_ref.d & 0xFF);
  tx[7] = CrcGenerateChecksum(tx, MESSAGE_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current measured speed and reference speed in RPM.
 *
 *  +---------------+----------------+
 *  |     Speed     |    Speed Ref   |
 *  +---------------+----------------+
 *      1       2        3      4     
 */
void PopulateTx_ResponseSpeedAndSpeedRef(uint8_t *tx, uint8_t seq) 
{
  const int16_t motor_speed = MC_GetMecSpeedAverageMotor1();
  const int16_t motor_speed_ref = MC_GetMecSpeedReferenceMotor1();

  PopulateTxHeader(tx, seq, SPEED_AND_SPEED_REF);
  tx[3] = (uint8_t)((motor_speed >> 8) & 0xFF);
  tx[4] = (uint8_t)(motor_speed & 0xFF);
  tx[5] = (uint8_t)((motor_speed_ref >> 8) & 0xFF);
  tx[6] = (uint8_t)(motor_speed_ref & 0xFF);
  tx[7] = CrcGenerateChecksum(tx, MESSAGE_SIZE - 1);
}

/**
 * Enables or disables the motor based on the provided flag.
 *
 * @param enabled Whether to enable or disable the motor.
 */
void SetMotorEnabled(bool enabled) 
{
  if (enabled && !motor_enabled) 
  {
    MC_StartMotor1();
  } 
  else if (!enabled && motor_enabled) 
  {
    MC_StopMotor1();
  }
  motor_enabled = enabled;
}

/* SPI callback handlers -----------------------------------------------------*/

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef* hspi)
{
  if (hspi->Instance == SPI1) 
  {
    txrx_half_completed = true;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
  if (hspi->Instance == SPI1) 
  {
    txrx_completed = true;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
  if (hspi->Instance == SPI1) 
  {
    // Do nothing for now
  }
}
