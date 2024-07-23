`timescale 1ns / 1ps
// ****************************************************
// * Initially based on Timothy Goddard's Verilog UART, 
// * but completely redesigned by Jeff Wilson of
// * S100Projects github and webpage, and S100Computers 
// * Google Group.  Nothing really remains of the
// * original Verilog code due to the requirements for
// * bit-level validtion, retiming on start bit edges
// * and the additionion of a receive FIFO.
// *
// * Improvements made:
// *  - Implemented Grant Searle's 50-clock RX Filtering
// *  - Implemented a revolving 16-Byte RX Receive FIFO
// *  - Reimplemented the TX/RX State Machines to be 
// *    non-blocking.
// *  - Reorganized TX/RX State Machines into State
// *    Clocking Sections, State Transition sections,
// *    and State Driven Output sections, for clearer
// *    differentiation of state machine functions.
// *
// ****************************************************

module uart(
    input  clk, 			    // The master clock for this module
    input  rst, 			    // Synchronous reset.
    input  rx, 				    // Incoming serial line
    output reg txd_out, 		// Outgoing serial line
    input  transmit, 			// Strobe to transmit
    input  [7:0] tx_byte, 		// Byte to transmit
    output received, 			// Copy of Data Ready
    output reg [7:0] rcvd_byte, // Byte received
    output is_receiving, 		// Low when receive line is idle.
    output is_transmitting,     // Low when transmit line is idle.
    output reg recv_error, 		// Indicates error in receiving packet.
    output data_ready,          // Indicates when the rcvd_byte[7:0] is ready
    input  data_read,           // Clears the data_ready flag
    output reg rcvr_overrun,    // Indicates when a Rx Data Overun condition occurs (cleared by data_read)
    output reg rcvr_fifo_full,  // Indicates when FIFO is full
    output reg rx_resyncing     // Indicates Receiver is resyncing
	);

// ****************************************************
// * FIFO Register File Interface
// ****************************************************
wire FIFO_WrEn;
reg  [7:0] FIFO_WrData;
wire [7:0] FIFO_RdData;

// ****************************************************
// * The Receive Counter is based on a 16 times over-sample of the baudrate.
// * Given the RX Filtering (1us High/Low filtering) the maximum data rate of the
// * uart interface will be 460,800 bits per second.
// * 
// * This 16-times the baudrate tick is used to time and evaluate the individual
// * bits and phasing of the received RX data.  The intial capture occurs on the
// * first falling start bit edge of the RX data and timing further bit-sampling
// * from there.  In the case where an input RX stream is slightly faster than
// * the RX State Machine Baudrate, the Stop Bit state allows for an early
// * start-bit edge detection and will transition directly to a new start bit
// * state (including reseting the sub-bit timing) to allow for the difference
// * in receiver clock skew.
// ****************************************************
//
// ****************************************************
// * Baudrate Table
// ****************************************************
// parameter BAUD_X16_DIV = 62500;      //      50 Baud     Error = 0.00%
// parameter BAUD_X16_DIV = 41667;      //      75 Baud     Error = 0.00%
// parameter BAUD_X16_DIV = 20833;      //     150 Baud     Error = 0.00%
// parameter BAUD_X16_DIV = 10417;      //     300 Baud     Error = 0.00%
// parameter BAUD_X16_DIV =  5208;      //     600 Baud     Error = 0.00%
// parameter BAUD_X16_DIV =  2604;      //   1,200 Baud     Error = 0.00%
// parameter BAUD_X16_DIV =  1302;      //   2,400 Baud     Error = 0.00%
// parameter BAUD_X16_DIV =   651;      //   4,800 Baud     Error = 0.00%
parameter BAUD_X16_DIV =   326;      //   9,600 Baud     Error = 0.01%
// parameter BAUD_X16_DIV =   163;      //  19,200 Baud     Error = 0.01%
// parameter BAUD_X16_DIV =    81;      //  38,400 Baud     Error = 0.04%
// parameter BAUD_X16_DIV =    54;      //  57,600 Baud     Error = 0.05%
// parameter BAUD_X16_DIV =    41;      //  76,800 Baud     Error = 0.07%
// parameter BAUD_X16_DIV =    27;      // 115,200 Baud     Error = 0.05%
// parameter BAUD_X16_DIV =    14;      // 230,400 Baud     Error = 3.20%
// parameter BAUD_X16_DIV =     7;      // 460,800 Baud     Error = 3.20%

// ****************************************************
// * Parameters for timing and validation of received serial bits
// ****************************************************
parameter  RX_BITS_VALID     = 11;      // The number of stable sub-bits that need to be valid
parameter  RX_EARLY_START    = 13;      // The latest sub-bit within the STOP bit that a Start bit is allowed
parameter  RX_BITS_LATCH     = 14;      // The latch point for the total high/low bit count
parameter  RX_BITS_OVF       = 15;      // The count where the RX sub-bit counter resets
parameter  RX_RESYNC_BITS    = 11*16;   // The number of IDLE (sub)bits needed to resync
parameter  TX_BITS_OVF       = 15;      // The count where the TX sub-bit counter resets

// ****************************************************
// * Baudrate Counters:
// * A clock divider of 16-bits allows for baudrates of 50 to 460,800 Baud
// * The rollover produces baud_tick_x16 enables at 16 x the baudrate
// ****************************************************
reg [15:0]  rx_baud_ctr_x16, tx_baud_ctr_x16;

// ****************************************************
// * States for the receiving state machine.
// ****************************************************
localparam RX_IDLE           = 0;
localparam RX_START_BIT      = 1;
localparam RX_SHIFT_BITS     = 2;
localparam RX_STOP_BIT       = 3;
localparam RX_RESYNC         = 4;

// ****************************************************
// * States for the data_read state machine.
// ****************************************************
localparam DR_IDLE           = 0;
localparam DR_READ           = 1;
localparam DR_INC_PTR        = 2;
localparam DR_WAIT_END       = 3;

// ****************************************************
// * States for the transmitting state machine.
// ****************************************************
localparam TX_IDLE           = 0;
localparam TX_START_ALIGN    = 1;
localparam TX_START_BIT      = 2;
localparam TX_SEND_DATA      = 3;
localparam TX_STOP_BIT       = 4;

// ****************************************************
// * FIFO Parameters
// ****************************************************
parameter FIFOBITDEPTH      = 4;    // Power of Two FIFO sizes...4 = 16 Bytes
parameter MAXFIFO           = 15;   // One count behind for wrap comparison

// ****************************************************
// * Receiver state registers and signals
// ****************************************************
reg [2:0]   rx_state, rx_state_next;
reg [3:0]   rx_sub_bit_ctr;
reg [3:0]   rx_sub_bitslow;
reg [3:0]   rx_sub_bitshigh;
reg         rx_valid_low;
reg         rx_valid_high;
reg [3:0]   rx_bit_count;
reg [7:0]   rx_data;
reg [6:0]   rx_fltr_ctr;
reg [7:0]   rx_resync_ctr;      // Up to 8-Bits of resyncing (i.e. - 16 RX Bits)
reg         rx_fltd;

// ****************************************************
// * Data read state machine and driven signals
// ****************************************************
reg [1:0]   dr_state, dr_state_next;
wire        data_read_active;

// ****************************************************
// * Rx FIFO Data and indexing pointers
// ****************************************************
reg     [(FIFOBITDEPTH-1):0]        RxRdPtr;
reg     [(FIFOBITDEPTH-1):0]        RxWrPtr;
reg     [(FIFOBITDEPTH-1):0]        RxCount;

// ****************************************************
// * Transmit state machine variables
// ****************************************************
reg [2:0] tx_state, tx_state_next;      // TX State Machine state registers
reg [3:0] tx_sub_bit_ctr;               // TX State Machine Sub-bit Counter
reg [3:0] tx_bit_count;                 // Transmit Bit Counter (TX_SEND_DATA)
reg [7:0] tx_data;                      // Latched and shifted TX Data

// ****************************************************
// * assign top-level flags for compatibility with FPGA Z80 SBC Builds
// ****************************************************
// Assign outputs for redundant signals
assign data_ready   =   (RxRdPtr == RxWrPtr) ? 1'b0 : 1'b1;
assign received     =   data_ready;
assign is_receiving =   data_ready;

// ****************************************************
// * Below are various combinatorial state transition enables or 
// * latching enables for various parts of the state machines.
// *
// * Assign baud rate, sub-bit and state transition ticks and enables
// ****************************************************
wire   rx_baud_tick_x16,    rx_baud_ctr0_tick,  tx_baud_tick_x16;
wire   rx_sub_ovf_tick,     tx_sub_ovf_tick,    tx_sub_start_tick;
wire   rx_next_state_en,    tx_next_state_en;
wire   rx_bit_valid_low,    rx_bit_valid_high;
wire   rx_baud_start_tick,  rx_sub_latch_tick;
wire   rx_early_start_en;
wire   rx_state_idle;
wire   rx_resync_done;
wire   set_error;
wire   set_overrun;
wire   FIFO_RdPtrInc;

// ****************************************************
// * Receiver tick or comparison enables (all combinatorial)
// ****************************************************
assign rx_baud_tick_x16     =   (rx_baud_ctr_x16 == BAUD_X16_DIV)   ? 1'b1 : 1'b0;
assign rx_baud_start_tick   =   (rx_baud_ctr_x16 == 0)              ? 1'b1 : 1'b0;
assign rx_sub_latch_tick    =   (rx_sub_bit_ctr  == RX_BITS_LATCH)  ? 1'b1 : 1'b0;
assign rx_sub_ovf_tick      =   (rx_sub_bit_ctr  == RX_BITS_OVF)    ? 1'b1 : 1'b0;
assign rx_bit_valid_low     =   (rx_sub_bitslow  >  RX_BITS_VALID)  ? 1'b1 : 1'b0;
assign rx_bit_valid_high    =   (rx_sub_bitshigh >  RX_BITS_VALID)  ? 1'b1 : 1'b0;
assign rx_early_start_en    =   (rx_sub_bit_ctr  >= RX_EARLY_START) ? 1'b1 : 1'b0;
assign rx_resync_done       =   (rx_resync_ctr   >= RX_RESYNC_BITS) ? 1'b1 : 1'b0;
assign rx_state_idle        =   (rx_state        == RX_IDLE)        ? 1'b1 : 1'b0;
assign rx_next_state_en     =   (rx_state != rx_state_next)         ? 1'b1 : 1'b0;

// ****************************************************
// * Transmitter tick or comparison enables (all combinatorial)
// ****************************************************
assign tx_baud_tick_x16     =   (tx_baud_ctr_x16 == BAUD_X16_DIV)   ? 1'b1 : 1'b0;
assign tx_sub_start_tick    =   (tx_sub_bit_ctr == 0)               ? 1'b1 : 1'b0;
assign tx_sub_ovf_tick      =   (tx_sub_bit_ctr == TX_BITS_OVF)     ? 1'b1 : 1'b0;
assign tx_next_state_en     =   (tx_state != tx_state_next)         ? 1'b1 : 1'b0;
assign is_transmitting      =   ((tx_state != TX_IDLE) |
                                  transmit)                         ? 1'b1 : 1'b0;

// ****************************************************
// * RX State Machine Synchronous logic and counters
// ****************************************************
always @(posedge clk) 
begin
    // ****************************************************
    // * Handle RX State Machines
    // ****************************************************
 	if (rst)
        rx_state    <= RX_IDLE;
    else
        rx_state    <= rx_state_next;

    // ****************************************************
    // * Handle Rx Baudrate Counter
    // ****************************************************
    if (rst | rx_state_idle | rx_baud_tick_x16)
        rx_baud_ctr_x16  <= 0;
    else
        rx_baud_ctr_x16  <= rx_baud_ctr_x16 + 1;
        
    // ****************************************************
    // * Handle RX Sub-Bit Counters (0-15) and Rx Bit High/Low Counters
    // *
    // * rx_sub_bitslow/high accumulate the number of low and
    // * high sub-bits within a single bit.  This info is used
    // * to qualify the quality of the received bit stream.
    // * rx_valid_low/high are status flags indicating that
    // * the received bits (low or high) meet the validity
    // * requirements.
    // ****************************************************
    // Reset Sub-Bit Counter with each RX state transition
    // Do not increment in RX_IDLE state (no sync yet)
    if  (rst | 
       ((rx_next_state_en & rx_baud_tick_x16) | 
        (rx_sub_ovf_tick & rx_baud_tick_x16)) |
         rx_state_idle)
    begin
        // Changing rx_state requires resetting rx_sub_bit_ctr
        rx_sub_bit_ctr  <= 0;
        rx_sub_bitslow  <= 0;
        rx_sub_bitshigh <= 0;
        rx_valid_low    <= 1'b0;
        rx_valid_high   <= 1'b0;
    end
    else
    begin
        // n.b. - Do not increment Sub-Bit Counter in IDLE State
        if (rx_baud_tick_x16 & !rx_state_idle)
        begin
            rx_sub_bit_ctr <= rx_sub_bit_ctr + 1;
            if (!rx_fltd)
                rx_sub_bitslow <= rx_sub_bitslow + 1;
            else
                rx_sub_bitslow <= rx_sub_bitslow;

            if (rx_fltd)
                rx_sub_bitshigh <= rx_sub_bitshigh + 1;
            else
                rx_sub_bitshigh <= rx_sub_bitshigh;

            // rx_fltd has a valid low during this bit period
            if (rx_bit_valid_low)
                rx_valid_low    <= 1'b1;
            else
                rx_valid_low    <= 1'b0;
            
            // rx_fltd has a valid high during this bit period
            if (rx_bit_valid_high)
                rx_valid_high   <= 1'b1;
            else
                rx_valid_high   <= 1'b0;
        end
    end
    
    // ****************************************************
    // * Handle Resync Counter
    // ****************************************************
    if ((rx_state != RX_RESYNC) | (rx_resync_done))
        rx_resync_ctr   <= 0;
    else if (rx_sub_ovf_tick & rx_baud_tick_x16)
        rx_resync_ctr   <= rx_resync_ctr + 1;
end


// ****************************************************
// * RX Data Filter 
// ****************************************************
always @(posedge clk)
begin
    // Filter the RX data (it must be stable for 50 clocks before changing state)
    if (rx & (rx_fltr_ctr == 50))
        rx_fltd = 1'b1;
    else if (rx & (rx_fltr_ctr != 50))
        rx_fltr_ctr = rx_fltr_ctr + 1;
        
    if (!rx & (rx_fltr_ctr == 0))
        rx_fltd = 1'b0;
    else if (!rx & (rx_fltr_ctr != 0))
        rx_fltr_ctr = rx_fltr_ctr - 1;
end


// ****************************************************
// * Next State Logic for RX state machine
// ****************************************************
always @(rx_state or rx_fltd or rx_sub_ovf_tick or rx_sub_bitslow or 
         rx_sub_bitshigh or rx_bit_count or rcvr_fifo_full or rx_data) 
begin
    case (rx_state)
        // Goto RX_START_BIT if low (start bit) detected
        RX_IDLE:        if (!rx_fltd)                                   rx_state_next = RX_START_BIT;
                        else                                            rx_state_next = RX_IDLE;

        // Goto RX_SHIFT_BITS if a valid start bit was detected
        // Goto RX_RESYNC if start bit is not low long enough
        RX_START_BIT:   if (rx_sub_ovf_tick & rx_baud_tick_x16)
                        begin
                            if (rx_valid_low)                           rx_state_next = RX_SHIFT_BITS;
                            else                                        rx_state_next = RX_RESYNC;
                        end
                        else                                            rx_state_next = RX_START_BIT;

        // Shift in 8 bits then goto RX_STOP_BIT
        RX_SHIFT_BITS:  if ((rx_sub_ovf_tick & rx_baud_tick_x16) &
                            (rx_bit_count == 7))                        rx_state_next = RX_STOP_BIT;
                            else                                        rx_state_next = RX_SHIFT_BITS;

        // Goto IDLE if an end of stop bit or an early start bit
        // is detected
        RX_STOP_BIT:    if ((rx_early_start_en & !rx_fltd) |                  
                            (rx_sub_ovf_tick & rx_baud_tick_x16 &
                             rx_fltd))                                  rx_state_next = RX_IDLE;
                        else                                            rx_state_next = RX_STOP_BIT;
                        
        // Stay in RX_RESYNC (rx is high) for specified # of cycles.
        RX_RESYNC:      if (rx_resync_done)                             rx_state_next = RX_IDLE;
                        else                                            rx_state_next = RX_RESYNC;
    endcase
end

// ****************************************************
// * Combinatorial Driven Signals for RX state machine
// ****************************************************
// Strobe the FIFO Write Enable but only if we have a 
// valid STOP bit and the FIFO is not full
assign FIFO_WrEn    =   ((rx_state == RX_STOP_BIT) &
                          rx_baud_start_tick &
                          rx_valid_high &
                          rx_sub_ovf_tick &
                          !rcvr_fifo_full)          ? 1'b1 : 1'b0;

// Strobe the FIFO Read Increment Enable.
assign FIFO_RdPtrInc =   (dr_state == DR_INC_PTR)   ? 1'b1 : 1'b0;

                          
// Set the overrun flag if the FIFO is full while trying
// to add a new rx byte
assign set_overrun  =   ((rx_state == RX_STOP_BIT) &
                          rx_valid_high &
                          rx_sub_ovf_tick & 
                          rx_baud_tick_x16 &
                          rcvr_fifo_full)           ? 1'b1 : 1'b0;

// set error if the STOP bit is not valid
assign set_error    =   ((rx_state == RX_STOP_BIT) &
                         !rx_valid_high &
                          rx_sub_ovf_tick & 
                          rx_baud_tick_x16)         ? 1'b1 : 1'b0;

// ****************************************************
// * Handle the rx state machine driven signals
// * n.b - These are registered logic.
// ****************************************************
always @(posedge clk)
begin
    if (rst)
    begin
        rx_bit_count    <= 0;
        RxWrPtr         <= 0;
        RxCount         <= 0;
        rcvr_fifo_full  <= 1'b0;
    end
    else
    begin
        // ****************************************************
        // * Handle FIFO Count and Flags.
        // *
        // * When RX/TX Pointers are equal, the FIFO is empty.
        // * Writes increment upto (but not matching) the
        // * current read pointer.
        // * Reads increment upto the current write pointer and
        // * when equal to the write pointer, the FIFO is empty.
        // ****************************************************
        // Compute Rx FIFO Data Size
        if (RxWrPtr == RxRdPtr)
            RxCount <= 0;
        else if (RxWrPtr > RxRdPtr)
            RxCount <= RxWrPtr - RxRdPtr;
        else
            RxCount <= (16 - RxRdPtr) + RxWrPtr;

        // Manage rcvr_fifo_full as a function of RxCount
        if (RxCount == MAXFIFO)
            rcvr_fifo_full <= 1;
        else
            rcvr_fifo_full <= 0;

        // ****************************************************
        // * Manage FIFO Pointers
        // ****************************************************
        // Manage FIFO Write Pointer (which is driven by the
        // receive state machine).
        if (FIFO_WrEn)
        begin
            if (RxWrPtr == MAXFIFO)
                RxWrPtr <= 0;
            else
                RxWrPtr <= RxWrPtr + 1;
        end
            
        // ****************************************************
        // * Manage state machine bit shifting and FIFO Write
        // ****************************************************
        if (rx_state == RX_START_BIT) 
        begin
            rx_bit_count    <= 0;
        end
        
        if (rx_state == RX_SHIFT_BITS)
        begin
            if (rx_sub_latch_tick & rx_baud_tick_x16)
            begin
                if (rx_valid_high)
                    rx_data    <= {1'b1, rx_data[7:1]};
                else if (rx_valid_low)
                    rx_data    <= {1'b0, rx_data[7:1]};
                // else ignore.  i.e.: set_error  <= 1'b1;
            end
            if (rx_sub_ovf_tick & rx_baud_tick_x16)
                rx_bit_count   <= rx_bit_count + 1;
        end
        
        if (rx_state == RX_STOP_BIT) 
        begin
            // Latch in the data to the FIFO Write Port in preparation
            // for strobing the data in
            if (rx_sub_latch_tick & rx_baud_tick_x16)
                FIFO_WrData    <= rx_data;
        end
        
        if (rx_state == RX_RESYNC)
        begin
            rx_resyncing    <= 1'b1;
        end
    end
end


// ****************************************************
// * Handle the RxRdPtr Increment
// ****************************************************
always @(posedge clk)
begin
    if (rst)
    begin
        RxRdPtr         <= 0;
        rcvr_overrun    <= 1'b0;
    end
    else 
    if (FIFO_RdPtrInc)
    begin
        if (RxWrPtr != RxRdPtr)
        begin
            if (RxRdPtr == MAXFIFO)
                RxRdPtr <= 0;
            else
                RxRdPtr <= RxRdPtr + 1;
            rcvr_overrun    <= 1'b0;
        end
        else
            rcvr_overrun    <= 1'b1;
    end
end

// ****************************************************
// * Handle data_read State Machine
// ****************************************************
always @(posedge clk)
begin
 	if (rst)
        dr_state    <= DR_IDLE;
    else
        dr_state    <= dr_state_next;
end

// ****************************************************
// * Handle data_read next state transitions
// ****************************************************
always @(dr_state or data_read)
begin
    case (dr_state)
        DR_IDLE:        if (data_read)              dr_state_next = DR_READ;
                        else                        dr_state_next = DR_IDLE;
                    
        DR_READ:                                    dr_state_next = DR_INC_PTR;
        
        DR_INC_PTR:                                 dr_state_next = DR_WAIT_END;
        
        DR_WAIT_END:    if (!data_read)             dr_state_next = DR_IDLE;
                        else                        dr_state_next = DR_WAIT_END;
                        
        //default                                     dr_state_next = DR_IDLE;
    endcase
end

// ****************************************************
// * Handle the data read state machine driven signals
// * n.b - These are registered logic.
// ****************************************************
assign data_read_active     =   (dr_state == DR_READ)   ? 1'b1 : 1'b0;

always @(posedge clk)
begin
    if (dr_state == DR_IDLE)
    begin
        // Always latch data in preparation for a read
        rcvd_byte <= FIFO_RdData;
    end
end


// ****************************************************
// * TX Synchronous counters and state machines
// ****************************************************
always @(posedge clk)
begin
    // ****************************************************
    // * Handle TX State Machine states
    // ****************************************************
    if (rst)
        tx_state    <= TX_IDLE;
    else 
        tx_state    <= tx_state_next;
    
    // ****************************************************
    // * Handle Tx Baudrate Counter
    // ****************************************************
    if ((rst) | (tx_baud_tick_x16))
        tx_baud_ctr_x16  <= 0;
    else
        tx_baud_ctr_x16  <= tx_baud_ctr_x16 + 1;
        
    // ****************************************************
    // * Handle TX Sub-Bit Counter (0-15)
    // ****************************************************
    if ((rst) | ((tx_sub_bit_ctr == 15) & tx_baud_tick_x16))
        tx_sub_bit_ctr  <= 0;
    else if (tx_baud_tick_x16)
        tx_sub_bit_ctr <= tx_sub_bit_ctr + 1;
    else
        tx_sub_bit_ctr <= tx_sub_bit_ctr;
end


// ****************************************************
// * Next State Logic for TX state machine (Combinatorial)
// ****************************************************
always @(posedge clk or tx_state_next or tx_sub_start_tick or
         tx_baud_tick_x16 or tx_sub_ovf_tick or tx_bit_count) 
begin
    // Transmit state machine
    case (tx_state)
        // Goto TX_START_BIT if transmit is high on a bit boundary
        // Goto TX_START_ALIGN if transmit is high but not on a 
        // bit boundary (to align it with a bit boundary
        TX_IDLE:        if (transmit)
                        begin
                            if (tx_sub_start_tick & 
                                tx_baud_tick_x16)           tx_state_next = TX_START_BIT;
                            else                            tx_state_next = TX_START_ALIGN;
                        end
                        else                                tx_state_next = TX_IDLE;

        // Goto TX_START_BIT if we are aligned to the
        // bit boundary.
        TX_START_ALIGN: if (tx_sub_ovf_tick &
                            tx_baud_tick_x16)               tx_state_next = TX_START_BIT;
                        else                                tx_state_next = TX_START_ALIGN;            
        
        // Goto TX_SEND_DATA when at the end of the 
        // start bit boundary.
        TX_START_BIT:   if (tx_sub_ovf_tick &
                            tx_baud_tick_x16)               tx_state_next = TX_SEND_DATA;
                        else                                tx_state_next = TX_START_BIT;  

        // Goto TX_STOP_BIT after sending 8-bits
        // (aligned to the bit bounary).
        TX_SEND_DATA:   if ((tx_sub_ovf_tick) &
                             tx_baud_tick_x16 &
                            (tx_bit_count == 7))            tx_state_next = TX_STOP_BIT;
                        else                                tx_state_next = TX_SEND_DATA;
                        
        // Goto TX_IDLE at the end of the stop bit.
        TX_STOP_BIT:    if (tx_sub_ovf_tick &
                            tx_baud_tick_x16)               tx_state_next = TX_IDLE;
                        else                                tx_state_next = TX_STOP_BIT;
                        
        default                                             tx_state_next = tx_state;
    endcase
end


// ****************************************************
// * Driven Signals for TX state machine
// ****************************************************
always @(posedge clk) 
begin
    if (rst)
    begin
        txd_out         <= 1'b1;            // idle marking bit
        tx_data         <= 0;
        tx_bit_count    <= 0;
    end
    else
    begin
        if (tx_state == TX_IDLE)
        begin
            txd_out         <= 1'b1;        // idle marking bit
            // Wait for 'transmit' signal...
            if (transmit)
                tx_data <= tx_byte;
        end
        
        // Leave prototype commented out here in case driven signals
        // are needed for this state in the future.
        // if (tx_state == TX_START_ALIGN)
        // begin
        // end

        if (tx_state == TX_START_BIT)
        begin
            // set start-bit
            txd_out <= 1'b0;
            if (tx_sub_ovf_tick)
                tx_bit_count  <= 0;
        end
        
        if (tx_state == TX_SEND_DATA)
        begin
            if (tx_sub_start_tick & tx_baud_tick_x16)
                txd_out <=       tx_data[0];
            if (tx_sub_ovf_tick & tx_baud_tick_x16)
            begin
                tx_data <=       {1'b0, tx_data[7:1]};
                tx_bit_count <=  tx_bit_count + 1;
            end
        end
        
        if (tx_state == TX_STOP_BIT)
            txd_out <= 1'b1;                         // set stop-bit
    end
end

RegisterFile    RxFIFO (
    .clk                (clk),		    // The master clock for this module 50MHz
    .writeEn            (FIFO_WrEn),
    .writeAddr          (RxWrPtr),
    .writeData          (FIFO_WrData),
    .readAddr           (RxRdPtr),
    .readData           (FIFO_RdData));

endmodule
