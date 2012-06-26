/**
 * Copyright (c) 2011-2012, Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the University of California, Berkeley nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Image capture device interface
 *
 * by Humphrey Hu
 *
 * v.1.0
 *
 * Revisions:
 *   Humphrey Hu    2011-10-26  Initial implementation
 *                  2012-01-31  Release
 *                  2012-02-16  Slight restructuring
 *                  2012-02-21  Changed to use pooled frames
 */

#include "timer.h"
#include "counter.h"
#include "ovcam.h"
#include "utils.h"
#include "carray.h"
#include <stdlib.h>
#include <string.h>
#include "cam.h"
#include "sys_clock.h"
#include "ov7660.h"

// Default camera capture timings for QQVGA no subsampling, 25 fps
#define ROW_ROW_TIME                    (32)
#define VSYNC_ROW_TIME                  (12800)
#define ROW_VSYNC_TIME                  (2800)
#define VSYNC_VSYNC_TIME                (25000)

// Amount of time before an event to trigger timer
#define ROW_ROW_OFFSET                  (6) // 384 cycles
#define VSYNC_ROW_OFFSET                (6) // 384 cycles
#define ROW_VSYNC_OFFSET                (8) // 512 cycles
#define VSYNC_VSYNC_OFFSET              (8) // 512 cycles

// The timer states describe what the timer is waiting for
// i.e. VSYNC state means timer is waiting for VSYNC event
// The NOT_SYNC state means that the timer has not been synchronized
// yet and is not ready for operation.
typedef enum {
    CT_NOT_SYNC,
    CT_WAIT_VSYNC,
    CT_WAIT_ROW,
    CT_SKIP_FRAME,
}CTimerState;

// ==== Function Stubs ========================================================
static void setupTimer7(void);
void _T7Interrupt(void);

void camCaptureRow(void);
static void processRow(void);

static CamFrame getEmptyFrame(void);
static void enqueueEmptyFrame(CamFrame frame);
static CamFrame getOldestFullFrame(void);
static void enqueueFullFrame(CamFrame frame);

// ==== Static Variables ======================================================
// Driver validity
static unsigned char is_ready;

// Asynchronous capture state
static CTimerState ct_state = CT_NOT_SYNC;
static unsigned long frame_start, frame_period;

// Asynchronous capture timing parameters
static unsigned int row_row_time = ROW_ROW_TIME;
static unsigned int vsync_row_time = VSYNC_ROW_TIME;
static unsigned int row_vsync_time = ROW_VSYNC_TIME;
static unsigned int vsync_vsync_time = VSYNC_VSYNC_TIME;

// Protected counters
static Counter frame_counter;
static Counter row_counter;

// Row capture buffer
static unsigned char row_buff[NATIVE_IMAGE_COLS];

// Frame buffering variables
static CamFrame current_frame;
static unsigned int next_row_index;
static unsigned char has_new_frame;
static CircArray empty_frame_pool, full_frame_pool;

// Driver config'd function pointers
static CamIrqHandler irq_handler;
static CamRowGetter row_getter;
static CamFrameWaiter frame_waiter;

// ==== Public functions ======================================================

void camSetup(CamFrame frames, unsigned int num_frames) {    

    unsigned int i;    
    
    setupTimer7();  // Set up timer peripheral

    next_row_index = 0;        
    has_new_frame = 0;
    current_frame = NULL;

    frame_counter = cntrCreate(); // Frame counter allocation
    if(frame_counter == NULL) { return; }
    row_counter = cntrCreate(); // Row counter allocation
    if(row_counter == NULL) { return; }

    empty_frame_pool = carrayCreate(num_frames); // Initialize frame pool
    if(empty_frame_pool == NULL) { return; }
    full_frame_pool = carrayCreate(num_frames); // Initialize frame pool
    if(full_frame_pool == NULL) { return; }

    for(i = 0; i < num_frames; i++) {
        camReturnFrame(&frames[i]);
    }
    
    ovcamSetup();   // Set up camera device
    irq_handler = NULL;   // Set up function pointers
    row_getter = &ovcamGetPixels;
    frame_waiter = &ovcamWaitForNewFrame;
    
    is_ready = 1;
    
    camRunCalib();  // Measure timing parameters

}

void camGetParams(CamParam params) {

    if(params == NULL) { return; }
    
    params->type = 0; // Not implemented yet!
    params->active = is_ready;
    params->frame_start = frame_start;
    params->frame_period = frame_period;

}

// Syncs the timer with the frame start event and begins the
// capture process.
void camStart(void) {

    if(!is_ready) { return; }

    DisableIntT7;				// Disable interrupt while syncing
    frame_waiter();             // Avoid clock drift    
    PR7 = VSYNC_ROW_TIME;		// Set wait time
    WriteTimer7(0);				// Reset timer	
    frame_start = sclockGetGlobalTicks();
    cntrSet(row_counter, 0);	// Reset row counter
    cntrSet(frame_counter, 0);	// Reset frame counter
    ct_state = CT_WAIT_ROW;		// Wait for first row
    EnableIntT7;				// Re-enable interrupt

}

void camStop(void) {

    DisableIntT7;

}

// Measures camera timing parameters
void camRunCalib(void) {

    unsigned int tic, capture_time, i;
    unsigned long t1, t2;

    if(!is_ready) { return; }

    // Approximately 8*pixels cycles per row
    // Using 64:1 prescale
    capture_time = (NATIVE_IMAGE_COLS)/(8);

    DisableIntT7;

    // VSYNC to VSYNC timing
    frame_waiter();
    WriteTimer7(0);
    t1 = sclockGetGlobalTicks();
    frame_waiter();
    tic = ReadTimer7();
    t2 = sclockGetGlobalTicks();
    vsync_vsync_time = tic - VSYNC_VSYNC_OFFSET;
    frame_period = t2 - t1;

    // VSYNC to row timing
    frame_waiter();
    WriteTimer7(0);
    for(i = 0; i < WINDOW_START_ROW; i++) {
        camCaptureRow();
    }
    camCaptureRow();
    tic = ReadTimer7();
    vsync_row_time = tic - capture_time - VSYNC_ROW_OFFSET;

    // row to row timing
    frame_waiter();
    camCaptureRow();
    WriteTimer7(0);
    for(i = 0; i < DS_ROW; i++) {
        camCaptureRow();
    }
    tic = ReadTimer7();
    row_row_time = tic - capture_time - ROW_ROW_OFFSET;

    // row to VSYNC timing
    frame_waiter();
    for(i = 0; i < WINDOW_END_ROW; i++) {
        camCaptureRow();
    }
    WriteTimer7(0);
    frame_waiter();
    tic = ReadTimer7();
    row_vsync_time = tic - ROW_VSYNC_OFFSET;

}

void camSetIrqHandler(CamIrqHandler irq) {
    irq_handler = irq;
}

unsigned char camHasNewFrame(void) {

    return has_new_frame;

}

CamFrame camGetFrame(void) {

    return getOldestFullFrame();

}

void camReturnFrame(CamFrame frame) {

    if(frame == NULL) { return; }
    enqueueEmptyFrame(frame);

}

unsigned int camGetFrameNum(void) {
    return cntrRead(frame_counter);
}

// =========== Private Functions ==============================================

// Interrupt handler for Timer 7
// Syncs frame timings and captures camera rows. This timer is
//  the highest priority with a medium execution time.
// TODO: Separate hardware and software downsampling
void __attribute__((interrupt, no_auto_psv)) _T7Interrupt(void) {

    if(ct_state == CT_WAIT_VSYNC) {
        frame_waiter();             // Avoid clock drift
        WriteTimer7(0);             // Reset timer
        
        cntrIncrement(frame_counter);
        cntrSet(row_counter, 0);    // Reset row counter

        if(cntrRead(frame_counter) % DS_FRAME == 0) {
            ct_state = CT_WAIT_ROW;         // Wait for first row
            PR7 = vsync_row_time;           // Set wait time
        } else {
            // ct_state == CT_WAIT_VSYNC (unchanged)
            PR7 = vsync_vsync_time;         // Wait for next frame
        }
        frame_start = sclockGetGlobalTicks();
        
    } else if(ct_state == CT_WAIT_ROW) {

        camCaptureRow();            // Capture row
        WriteTimer7(0);             // Reset timer
        processRow();               // Process row
        cntrAdd(row_counter, DS_ROW);    // Increment row count

        // Transition if captured last row
        if(cntrRead(row_counter) >= NATIVE_IMAGE_ROWS) {
            ct_state = CT_WAIT_VSYNC;
            PR7 = row_vsync_time;
            if(irq_handler != NULL) {
                irq_handler(CAM_IRQ_FRAME_DONE);
            }
        } else { // Else wait for next row
            PR7 = row_row_time;
            if(irq_handler != NULL) {
                irq_handler(CAM_IRQ_ROW_DONE);
            }
        }
    }

    _T7IF = 0;

}

void camCaptureRow(void) {

    CRITICAL_SECTION_START;    
    row_getter(row_buff, NATIVE_IMAGE_COLS);            
    CRITICAL_SECTION_END;

}

void processRow(void) {

    unsigned int i, j, k, acc;
    unsigned char *src_data, *dst_data;

    if(current_frame == NULL) {
        current_frame = getEmptyFrame(); // Load new frame
        if(current_frame == NULL) { return; }
    }
    
    dst_data = current_frame->pixels[next_row_index]; // Write into current frame
    src_data = row_buff;

    k = WINDOW_START_COL;
    for(i = 0; i < DS_IMAGE_COLS; i++) {
        acc = 0;
        for(j = 0; j < DS_COL; j++) {
            acc += row_buff[k++];
        }
        dst_data[i] = acc/DS_COL;
    }
        
    next_row_index++;    

    // If all rows are filled, add the frame to the full frame buffer
    if(next_row_index >= DS_IMAGE_ROWS) {
        current_frame->frame_num = cntrRead(frame_counter); // write frame number
        current_frame->timestamp = sclockGetLocalTicks();
        enqueueFullFrame(current_frame); // Add to output queue
        current_frame = NULL;
        next_row_index = 0;
    }

}

/**
 * Get the next available empty frame. If no frames are available, automatically
 * dequeues and returns the oldest full frame.
 *
 * @return Next available frame for writing
 */
static CamFrame getEmptyFrame(void) {

    CamFrame frame;

    frame = carrayPopHead(empty_frame_pool);
    if(frame == NULL) {
        frame = getOldestFullFrame(); // If no more empty frames, get oldest full
    }
    return frame;

}

/**
 * Enqueues a frame for writing into.
 *
 * @param frame CamFrame object to enqueue
 */
static void enqueueEmptyFrame(CamFrame frame) {

    carrayAddTail(empty_frame_pool, frame);

}

/**
 * Returns the oldest full frame in the outgoing buffer.
 *
 * @return Oldest full frame object
 */
static CamFrame getOldestFullFrame(void) {

    CamFrame frame;

    frame = carrayPopHead(full_frame_pool);
    if(carrayIsEmpty(full_frame_pool)) {
        has_new_frame = 0;
    }

    return frame;

}

/**
 * Enqueues a full frame object in the outgoing buffer.
 *
 * @param frame CamFrame object to enqueue
 */
static void enqueueFullFrame(CamFrame frame) {

    carrayAddTail(full_frame_pool, frame);
    has_new_frame = 1;

}

// Camera acquisition timer setup
static void setupTimer7(void) {

    unsigned int con_reg;

    con_reg =   T7_ON &             // Enable module
    T7_IDLE_CON &       // Continue running when idle
    T7_GATE_OFF &       // Time accumulation disable
    T7_PS_1_64 &        // Prescale 1:64
    T7_SOURCE_INT;      // Internal clock

    _T7IF = 0;
    OpenTimer7(con_reg, 0);         // Configure timer
    ConfigIntTimer7(T7_INT_PRIOR_6 & T7_INT_OFF);

}
