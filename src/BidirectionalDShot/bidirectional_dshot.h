/**
 * @file bidirectional_dshot.h
 * @brief DMA-enabled bidirectional dshot communication protocol manager class for multiple ESCs. Adapted from teensyshot.
 * @author Jared Boyer <jaredb@mit.edu>
 * @date 22 Jul 2021
 */

#ifndef BIDIRECTIONAL_DSHOT_H_
#define BIDIRECTIONAL_DSHOT_H_

#include <Arduino.h>
#include <util/atomic.h>
#include "DMAChannel.h"

#define DSHOT_DMA_LENGTH          18            // Number of steps of one DMA sequence (the two last values are zero)
#define DSHOT_DMA_MARGIN          2             // Number of additional bit duration to wait until checking if DMA is over
#define DSHOT_DSHOT_LENGTH        16            // Number of bits in a DSHOT sequence
#define DSHOT_BT_DURATION 3333 // DShot300 bit period, ns
#define DSHOT_LP_DURATION 2500 // DShot300 long pulse, ns
#define DSHOT_SP_DURATION 1250 // DShot300 short pulse, ns
#define MAX_ESC                   6             // Supports up to 6 ESCs
#define TX_WAIT 1000 // 1 kHz for the software signal-monitor test
#define RX_WAIT 130 // microseconds after TX DMA completion; includes turnaround and reply
#define RX_SIGNAL_LENGTH          21            // Number of bits in the dshot feedback received signal
#define RX_BIT_TICK_LENGTH (F_CPU_ACTUAL / 375000u) // reply bitrate = 1.25 * 300 kbit/s

// ITC: front-left 5, front-right 6, rear-left 8, rear-right 4.
#define DSHOT_ACTIVE_ESC 4
#define DSHOT_ATOMIC_RPM_SNAPSHOT 1
#define DSHOT_FRAME_CALLBACK_API 1
extern volatile uint32_t dshotTxCount;
extern volatile uint32_t dshotDmaCount;
extern volatile uint32_t dshotRxCount;
extern volatile uint32_t dshotEdgeCount;

class DshotManager
{
public:
	DshotManager();
	void start_tx();
    void set_frame_callback(void (*callback)());
	void set_throttle_esc(int i, uint16_t input);
	uint32_t average_eRPM(int i);
};

void DMA_init(int i);
void assemble_signal_esc(int i);
uint32_t decode_signal(int i);
void reset_array(int i);
void tx_ISR();
void rx_ISR();

#endif /* BIDIRECTIONAL_DSHOT_H_ */





