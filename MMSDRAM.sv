//
// MMSDRAM
//

// a memory-mapped SDRAM controller that talks to the PHY
// ADDR:
//   0x20-0x22 is address (3 bytes)
//   0x23 is control
//   0x30-0x3F is buffer (16 bytes)
// operation:
//   one reads and writes the 16 byte block buffer at addresses 0x30-0x3f
//   one sets the SDRAM address (24bits for 16M x 16-byte blocks) at addresses 0x20-0x22
//   one reads the control byte to determine whether the controller is ready (1) or not (0)
//   one writes 0 to the control byte to trigger a read, 1 to trigger a write
//   a read reads a 16-bytes block into the buffer, a write writes the buffer to SDRAM 


module MMSDRAM #(
    parameter simulation = 0,
    parameter CPU_DIVIDE = 1
)(
    input  wire logic CLKSYS,       // system 100MHz clock
    input  wire logic CLKREF,       // reference 200MHz clock
    input  wire logic RST_IN,       // reset-in
    output wire logic RST_OUT,      // reset-out
    
    output wire logic CLKD,         // divided /4 SDRAM clock eg ~80MHz
    output wire logic CLK0,         // CPU clock
    output wire logic CLK2,         // CPU phased clock
    
    input  wire logic MMAP,         // enable
    input  wire logic RW,           // read/write
    input  wire logic [4:0] ADDR,   // memory-mapped address
    input  wire logic [7:0] DIN,    // data-in
    output wire logic [7:0] DOUT,   // data-out
    
    output wire logic             DDR3_RST_N,    
    output wire logic             DDR3_CK_P,
    output wire logic             DDR3_CK_N,
    output wire logic             DDR3_CS_N,
    output wire logic             DDR3_RAS_N,
    output wire logic             DDR3_CAS_N,
    output wire logic             DDR3_WE_N,
    output wire logic [2:0]       DDR3_BA,
    output wire logic [13:0]      DDR3_ADDR,
    output wire logic             DDR3_CKE,
    output wire logic             DDR3_ODT,
    inout  wire logic [1:0]       DDR3_DM,
    inout  wire logic [15:0]      DDR3_DQ,
    inout  wire logic [1:0]       DDR3_DQS_P,
    inout  wire logic [1:0]       DDR3_DQS_N    
);

    // ignore warning for simulation but respect delay for synthesis
    // WARNING: 500 us is required after RST_N goes inactive before CKE goes active
    // clocked on clk at ~83MHz ie T~12ns, 500us is 500_000/12 = 41_666 ticks
    localparam CKE_PERIOD = simulation ? 4 : 42_000;

    // fast initialization else simulation takes forever
    localparam SIM_BYPASS_INIT_CAL = simulation ? "FAST" : "OFF";

    // see note in state machine / read state - works with this value
    localparam RD_OFFSET_FIX = 0; 

    wire logic clkd;
    assign CLKD = clkd;
    
    var logic [7:0] dout;
    assign DOUT = dout;
        
    wire             init_calib_complete;

    wire             idle;
    //wire             error;
    wire             rst_tg_mc;
    
    reg  [3:0]       mc_ras_n; // [nCK_PER_CLK-1:0]
    reg  [3:0]       mc_cas_n; // [nCK_PER_CLK-1:0]
    reg  [3:0]       mc_we_n; // [nCK_PER_CLK-1:0]
    reg  [55:0]      mc_address; // [nCK_PER_CLK*ROW_WIDTH-1:0]
    reg  [11:0]      mc_bank; // [nCK_PER_CLK*BANK_WIDTH-1:0]
    reg  [3:0]       mc_cke; // [nCK_PER_CLK-1:0]
    reg  [1:0]       mc_odt;
    reg  [3:0]       mc_cs_n; // [CS_WIDTH*nCS_PER_RANK*nCK_PER_CLK-1:0]
    wire             mc_reset_n;
    reg  [127:0]     mc_wrdata; // [2*nCK_PER_CLK*DQ_WIDTH-1:0]
    reg  [15:0]      mc_wrdata_mask; // [2*nCK_PER_CLK*DQ_WIDTH/8-1:0]
    reg              mc_wrdata_en;
    wire             mc_ref_zq_wip;
    wire             mc_cmd_wren;
    wire             mc_ctl_wren;
    reg  [2:0]       mc_cmd;
    reg  [1:0]       mc_cas_slot;
    reg  [5:0]       mc_data_offset_0;
    reg  [5:0]       mc_data_offset_1;
    reg  [5:0]       mc_data_offset_2;
    wire [3:0]       mc_aux_out0;
    wire [3:0]       mc_aux_out1;
    wire [1:0]       mc_rank_cnt;
    
    wire             phy_mc_ctl_full;
    wire             phy_mc_cmd_full;
    wire             phy_mc_data_full;
    wire [127:0]     phy_rd_data; // [2*nCK_PER_CLK*DQ_WIDTH-1:0]
    wire             phy_rddata_valid;
    
    wire [5:0]       calib_rd_data_offset_0;
    wire [5:0]       calib_rd_data_offset_1;
    wire [5:0]       calib_rd_data_offset_2;

    // TODO: wire the missing wires
    // TODO: implement a suitable controller!
    // TODO: handle CLKM vs CLKD collisions?
    
    // PHY command
    localparam
        CMD_NOP = 4,
        CMD_WR  = 1,
        CMD_RD  = 3;
    
    // commands (from datasheet table 87 p120)
    localparam
        //          = { CS#, RAS#, CAS#, WE# }              (condition)
        CMDX_MRS     = 4'b0000, // MODE REGISTER SET         (BA:register)
        CMDX_REF     = 4'b0001, // REFRESH                   (CKE:1->1)
        CMDX_SRE     = 4'b0001, // SELF-REFRESH ENTRY        (CKE:1->0)
        CMDX_SRX     = 4'b0111, // SELF-REFRESH EXIT (NOP)
        CMDX_PRE     = 4'b0010, // SINGLE-BANK PRECHARGE     (A10:0)
        CMDX_PREA    = 4'b0010, // PRECHARGE ALL BANKS       (A10:1)
        CMDX_ACT     = 4'b0011, // BANK ACTIVATE
        CMDX_WR      = 4'b0100, // WRITE                     (A10:0)
        CMDX_WRAP    = 4'b0100, // WRITE W/AUTO-PRECHARGE    (A10:1)
        CMDX_RD      = 4'b0101, // READ                      (A10:0)
        CMDX_RDAP    = 4'b0101, // READ W/AUTO-PRECHARGE     (A10:1)
        CMDX_NOP     = 4'b1111, // NOP - or 0111?
        CMDX_DES     = 4'b1000, // DEVICES DESELECTED
        CMDX_PDE     = 4'b0111, // POWER-DOWN ENTRY (NOP)    (CKE:1->0)
        CMDX_PDX     = 4'b0111, // POWER-DOWN EXIT (NOP)     (CKE:0->1)
        CMDX_ZQCL    = 4'b0110, // ZQ CALIBRATION LONG       (A10:1)
        CMDX_ZQCS    = 4'b0110; // ZQ CALIBRATION SHORT      (A10:0)
        
    // mc_cmdx is a composite signal that allows us to write meaningful commands:
    //   mc_cmdx <= { CMDX_NOP, CMDX_NOP, CMDX_NOP, CMDX_NOP };
    // and we derive the four CS/RAS/CAS/WE signals from it
    reg [15:0] mc_cmdx;
    always @* begin
        mc_cs_n  = { mc_cmdx[15], mc_cmdx[11], mc_cmdx[7], mc_cmdx[3] };
        mc_ras_n = { mc_cmdx[14], mc_cmdx[10], mc_cmdx[6], mc_cmdx[2] };
        mc_cas_n = { mc_cmdx[13], mc_cmdx[9],  mc_cmdx[5], mc_cmdx[1] };
        mc_we_n  = { mc_cmdx[12], mc_cmdx[8],  mc_cmdx[4], mc_cmdx[0] };
    end

    assign idle             = 0;
    assign mc_cmd_wren      = 1;
    assign mc_ctl_wren      = 1;
    assign mc_aux_out0      = 4'h0;
    assign mc_aux_out1      = 4'h0;
    assign mc_rank_cnt      = 2'h0;
    assign mc_cas_slot      = 2'b01;
    assign mc_rank_cnt      = 2'b00;
    assign mc_data_offset_2 = 6'h00;    
    assign mc_ref_zq_wip    = 0; // ??
    assign mc_reset_n       = 1;
    
    // PHY rst is "The rstdiv0 output from the infrastructure module 
    //   synchronized to the PHY_Clk domain." it is NOT directly
    //   sys_rst but is produced by INFRA which itself is driven by
    //   sys_rst_o which is produced by IODELAY_CTRL from sys_rst.
    // mc_reset_n (active low) is "input directly to the IOLOGIC without
    //   an OUT_FIFO." and UG586 p176 for PHY-only specifies it has to
    //   be tied to constant 1.
    // sys_rst "This is the main system reset (asynchronous). The reset
    //   signal must be applied for a minimum pulse width of 5 ns."
    
    var logic sdram_ready, sdram_locked;
    var logic [7:0] sdram_state, sdram_wstate;
    var logic [15:0] sdram_wcount; // wait count
    var logic [15:0] sdram_rcount; // refresh count
    var logic sdram_rq, sdram_op; // request & operation (read or write)
    var logic [127:0] sdram_buffer; //var logic [7:0] sdram_buffer [16];
    var logic [23:0] sdram_addr; //var logic [7:0] sdram_addr [3];

    // we have
    //      3-bits bank   = 8      banks
    //     14-bits row    = 16_384 rows
    //     10-bits column = 1_024  columns
    // which gives 8*16_384*1_024 = 128K-words = 256MB
    // which we are reading as 8-words bursts or 16-byte = 128-bit values
    // starting at a given column, so we can round the colummn numbers to 8-words boundary
    // -> a full address is bank+row+rounded_column ie 3+14+7 bits = 24 bits
    //
    // and then, we can use two schemes: [bank][row][column] or [row][bank][column]
    // depending on usage patterns and activate/precharge constraints... we need to pick one,
    // not sure exactly how to decide, going with [bank][row][column] here.

    var logic [2:0]  sdram_bank;
    var logic [13:0] sdram_row;
    var logic [9:0]  sdram_column;

    assign sdram_bank   = sdram_addr [23:21];
    assign sdram_row    = sdram_addr [20:7];
    assign sdram_column = { sdram_addr [6:0], 3'b000 }; // rounded to 8-words boundary
    
    // now we can define the mc_address values for PHY:
    // - for ACTIVATE and PRECHARGE, mc_address specifies the row
    // - for READ and WRITE, mc_address specifies the column + A10/A12 for options
    // A10 = 1 -> auto-precharge (used)
    // A12 = 1 -> BL8 (ignored, forced)

    var logic [13:0] sdram_addr_ap, sdram_addr_rw;
    assign sdram_addr_ap = sdram_row [13:0];
    assign sdram_addr_rw = { 1'b0, /*A12=*/ 1'b0, 1'b0, /*A10=*/1'b1, sdram_column [9:0] };

    assign RST_OUT = ~sdram_locked;

    always @(posedge clkd) begin
        if (~sdram_locked) begin
            mc_cmd      <= CMD_NOP;
            mc_cmdx     <= { CMDX_NOP, CMDX_NOP, CMDX_NOP, CMDX_NOP };
            mc_bank     <= { 3'h7,     3'h7,     3'h7,     3'h7 };
            mc_address  <= { 14'h3fff, 14'h3fff, 14'h3fff, 14'h3fff };
            mc_odt      <= 2'b00;
            mc_cke      <= 4'b0000;
            mc_data_offset_0 <= 6'h00;
            mc_data_offset_1 <= 6'h00;
            mc_wrdata <= 128'h0000000000000000;
            mc_wrdata_mask <= 16'h0000;
            mc_wrdata_en <= 0;
            
            sdram_ready <= 0;
            sdram_rcount <= 0;
            sdram_state  <= 0;
            sdram_rq <= 0;
        end
        else begin
        
            // refresh count
            sdram_rcount <= sdram_rcount + 1;
            
            // the MMAP clock is aligned with clk ie the CPU clock is produced
            // by dividing the MMAP clock and everything is synchronous.

            //var logic8 case_state;
            //assign case_state = MMAP ? : sdram_state;
            
            if (MMAP && CLK2) begin
                
                // ADDR is 6 lowest bits
                // 0x4020-0x4022 is address (3 bytes)
                // 0x4023 is control
                // 0x4030-0x403F is buffer (16 bytes)
                
                if (ADDR[4] == 1'b1) begin // 0x403x
                    if (RW)
                        sdram_buffer[8*ADDR[3:0]+:8] <= DIN;
                    else
                        dout <= sdram_buffer[8*ADDR[3:0]+:8];
                end
                else // 0x402x
                if (ADDR[3:2] == 2'b00) begin // 0x4020-0x4023
                    if (ADDR[1:0] == 2'b11) begin // 0x4023
                        if (RW) begin
                            // TODO this introduces a 1-cycle delay?
                            sdram_rq <= 1;
                            sdram_op <= DIN [0];
                        end
                        else dout <= { 7'b0, sdram_ready };
                    end
                    else begin
                        if (RW)
                            sdram_addr[8*ADDR[1:0]+:8] <= DIN;
                        else
                            dout <= sdram_addr[8*ADDR[1:0]+:8];
                    end
                end
                else // 0x4024-0x402f
                    dout <= 0;

            end
            
            case (sdram_state) // FIXME count states, reduce size!
                
                default: begin
                end
            
                // CKE DELAY SEQUENCE

                0: begin
                    sdram_state  <= 254;
                    sdram_wstate <= 1;
                    sdram_wcount <= CKE_PERIOD;
                end
                
                1: begin
                    mc_cke <= 4'b1111;
                    sdram_state <= 2;
                end
                
                // WAIT FOR CALIBRATION

                2: begin
                    if (init_calib_complete) begin
                        $display("at time %0t ZB: Calibration Done", $realtime);
                        $display("at time %0t ZB: Wait", $realtime);
                        sdram_state  <= 254;
                        sdram_wcount <= 32;
                        sdram_wstate <= 3;
                    end
                end

                // REFRESH SEQUENCE

                3: begin
                    $display("at time %0t ZB: Refresh", $realtime);
                    mc_cmd      <= CMD_NOP;
                    mc_cmdx     <= { CMDX_NOP, CMDX_NOP, CMDX_NOP, CMDX_REF };
                    sdram_state  <= 4;
                end
                
                4: begin
                    $display("at time %0t ZB: Clear", $realtime);
                    mc_cmd      <= CMD_NOP;
                    mc_cmdx     <= { CMDX_NOP, CMDX_NOP, CMDX_NOP, CMDX_NOP };
                    $display("at time %0t ZB: Wait", $realtime);
                    sdram_state  <= 254;
                    sdram_wcount <= 16;
                    sdram_wstate <= 60;
                    sdram_ready <= 1; 
                end
                
                // RUN                
                // wait for command, execute command, refresh periodically
                
                // FIXME that one makes no sense

                60: begin // back from refreshing / reading / writing / ...
                
                    // need to refresh periodically (see refresh sequence for details)
                    // 7.8us (0,064/8192) period with clk running at a period of tCK * nCK_PER_CLK
                    // -> every (0,064/8192) / (tCK*nCK_PER_CLK) = 7.8125us/12_308ps = 634 clk ticks
                    // TODO: go with a decrementing counter and test for zero? go with 512?
                    
                    if (sdram_rcount > 600) begin
                        sdram_ready <= 0;
                        sdram_state <= 120;
                    end
                    else if (sdram_rq) begin
                        sdram_ready <= 0;
                        sdram_state <= 80;
                        sdram_wstate <= sdram_op ? 90 : 100;
                        sdram_rq <= 0; // FIXME is going back up too soon ??
                    end
                    else begin
                        sdram_ready <= 1;
                    end
                end

                // ACTIVATE SEQUENCE
                //
                // The ACTIVATE command is used to open (or activate) a row in a particular bank for a
                // subsequent access. The value on the BA[2:0] inputs selects the bank, and the address
                // provided on inputs A[n:0] selects the row. This row remains open (or active) for accesses
                // until a PRECHARGE command is issued to that bank.
                // A PRECHARGE command must be issued before opening a different row in the same
                // bank.

                80: begin
                    $display("at time %0t ZB: Activate Bank", $realtime);
                    mc_cmd      <= CMD_NOP;
                    mc_cmdx     <= { CMDX_NOP, CMDX_NOP, CMDX_NOP, CMDX_ACT };       
                    mc_bank     <= { 3'h7,     3'h7,     3'h7,     sdram_bank };
                    mc_address  <= { 14'h3fff, 14'h3fff, 14'h3fff, sdram_addr_ap };
                    sdram_state  <= 81;
                end
                81: begin
                    $display("at time %0t ZB: Clear", $realtime);
                    mc_cmd      <= CMD_NOP;
                    mc_cmdx     <= { CMDX_NOP, CMDX_NOP, CMDX_NOP, CMDX_NOP };
                    mc_bank     <= { 3'h7,     3'h7,     3'h7,     3'h7 };
                    mc_address  <= { 14'h3fff, 14'h3fff, 14'h3fff, 14'h3fff };
                    $display("at time %0t ZB: Wait", $realtime);
                    // ACTIVATE to READ/WRITE minimum tRCD = 13500ps
                    // with clk running at a period of tCK * nCK_PER_CLK
                    // -> 13500/12308 ~= 2
                    sdram_state  <= 254;
                    sdram_wcount <= 2;
                    //test_wstate <= 70; // NO! set when triggering activate sequence
                end

                // WRITE SEQUENCE
                //
                // The WRITE command is used to initiate a burst write access to an active row. The value
                // on the BA[2:0] inputs selects the bank. The value on input A10 determines whether auto
                // precharge is used. The value on input A12 (if enabled in the MR) when the WRITE command
                // is issued determines whether BC4 (chop) or BL8 is used.
                // Input data appearing on the DQ is written to the memory array subject to the DM input
                // logic level appearing coincident with the data. If a given DM signal is registered LOW,
                // the corresponding data will be written to memory. If the DM signal is registered HIGH,
                // the corresponding data inputs will be ignored and a WRITE will not be executed to that
                // byte/column location.
                //
                // A10 = 1 -> auto-precharge
                // A12 = 1 -> BL8
                //
                // Burst length is defined by MR0[1:0]. Read and write accesses to the DDR3 SDRAM are
                // burst-oriented, with the burst length being programmable to 4 (chop mode), 8 (fixed
                // mode), or selectable using A12 during a READ/WRITE command (on-the-fly). The burst
                // length determines the maximum number of column locations that can be accessed for
                // a given READ or WRITE command.    
                //
                // PHY initializes with INFO: Load Mode 0 Burst Length =  8 so we can ignore A12
                // we don't want auto-precharge and keep A10 low
                //
                // about slot#: mc_cmdx uses 4 slots with numbers { 3, 2, 1, 0 }        

                90: begin
                    $display("at time %0t ZB: Write wo/Precharge", $realtime);
                    mc_cmd      <= CMD_WR;
                    mc_cmdx     <= { CMDX_NOP, CMDX_NOP, CMDX_WR,       CMDX_NOP }; 
                    mc_bank     <= { 3'h7,     3'h7,     sdram_bank,    3'h7 };
                    mc_address  <= { 14'h3fff, 14'h3fff, sdram_addr_rw, 14'h3fff }; // ! A12 and A10 have meaning
                    mc_wrdata   <= sdram_buffer;
                    
                    // XA51204:
                    // For nCK_PER_CLK = 4: Write Data Offset = CWL + 2 + slot number
                    mc_data_offset_0 <= 6'h08; // CWL (5) + 2 + slot# (1)
                    mc_data_offset_1 <= 6'h08; // CWL (5) + 2 + slot# (1)
                    
                    mc_wrdata_en <= 1;
                    mc_odt       <= 1;
                    sdram_state <= 91;            
                end
                91: begin
                    $display("at time %0t ZB: Clear", $realtime);
                    mc_cmd      <= CMD_NOP;
                    mc_cmdx     <= { CMDX_NOP, CMDX_NOP, CMDX_NOP, CMDX_NOP };         
                    mc_bank     <= { 3'h7,     3'h7,     3'h7,     3'h7 };
                    mc_address  <= { 14'h3fff, 14'h3fff, 14'h3fff, 14'h3fff };       
                    mc_data_offset_0 <= 6'h00;
                    mc_data_offset_1 <= 6'h00;
                    mc_wrdata_en <= 0;
                    sdram_state  <= 92;
                end
                92: begin
                    // prevent ERROR:  ODTH8 violation during ODT transition
                    // specs: ODT HIGH time with WRITE command and BL8, MIN = 6 CK
                    $display("at time %0t ZB: Clear (ODT)", $realtime);
                    mc_odt       <= 0;
                    $display("at time %0t ZB: Wait", $realtime);
                    // WRITE to READ minimum tWTR = 7500ps after last data sent 
                    // with clk running at a period of tCK * nCK_PER_CLK
                    // -> 7500/12308 ~= 1
                    sdram_state  <= 254;
                    sdram_wcount <= 1;
                    sdram_wstate <= 60;
                end
                
                // READ SEQUENCE
                //
                // The READ command is used to initiate a burst read access to an active row. The address
                // provided on inputs A[2:0] selects the starting column address, depending on the burst
                // length and burst type selected (see Burst Order table for additional information). The
                // value on input A10 determines whether auto precharge is used. If auto precharge is selected,
                // the row being accessed will be precharged at the end of the READ burst. If auto
                // precharge is not selected, the row will remain open for subsequent accesses. The value
                // on input A12 (if enabled in the mode register) when the READ command is issued determines
                // whether BC4 (chop) or BL8 is used. After a READ command is issued, the
                // READ burst may not be interrupted.
                //
                // A10 = 1 -> auto-precharge
                // A12 = 1 -> BL8
                //
                // Burst length is defined by MR0[1:0]. Read and write accesses to the DDR3 SDRAM are
                // burst-oriented, with the burst length being programmable to 4 (chop mode), 8 (fixed
                // mode), or selectable using A12 during a READ/WRITE command (on-the-fly). The burst
                // length determines the maximum number of column locations that can be accessed for
                // a given READ or WRITE command.    
                //
                // PHY initializes with INFO: Load Mode 0 Burst Length =  8 so we can ignore A12
                // we don't want auto-precharge and keep A10 low
                //
                // about slot#: mc_cmdx uses 4 slots with numbers { 3, 2, 1, 0 }        

                100: begin
                    $display("at time %0t ZB: Read wo/Precharge", $realtime);
                    mc_cmd              <= CMD_RD;
                    mc_cmdx             <= { CMDX_NOP, CMDX_NOP, CMDX_RD,       CMDX_NOP };         
                    mc_bank             <= { 3'h7,     3'h7,     sdram_bank,    3'h7 };
                    mc_address          <= { 14'h3fff, 14'h3fff, sdram_addr_rw, 14'h3fff }; // ! A12 and A10 have meaning
                    
                    // XA51204:
                    // Read Data Offset = Calibrated PHY read data offset (calib_rd_data_offset_*) + slot number
                    // and: "When using a custom controller, the data offset values used during calibration and
                    // normal operation reads may be different depending on the CWL. The values should match 
                    // for reads with even CWL, and be off by 1 for reads with odd CWL. This is because reads/writes
                    // are assigned to slot1 by the memory controller whereas slot0 is used for even CWL for the 
                    // MIG controller as only slot0 and slot1 are used for reads/writes."
                    // and: "As a general rule of thumb, the read data offset should be approximately..." ?!!?
                    //
                    // in our case CWL=5 is odd indeed, so we need a "rule of thumb" fix
                    mc_data_offset_0    <= calib_rd_data_offset_0 + RD_OFFSET_FIX + 1; // calib_rd_data_offset_0 + fix + slot# (1)
                    mc_data_offset_1    <= calib_rd_data_offset_1 + RD_OFFSET_FIX + 1; // calib_rd_data_offset_1 + fix + slot# (1)
                    
                    sdram_state          <= 101;
                end
                101: begin
                    $display("at time %0t ZB: Clear", $realtime);
                    mc_cmd              <= CMD_NOP;
                    mc_cmdx             <= { CMDX_NOP, CMDX_NOP, CMDX_NOP, CMDX_NOP };         
                    mc_bank             <= { 3'h7,     3'h7,     3'h7,     3'h7 };
                    mc_address          <= { 14'h3fff, 14'h3fff, 14'h3fff, 14'h3fff };       
                    mc_data_offset_0    <= 6'h00;
                    mc_data_offset_1    <= 6'h00;
                    mc_wrdata_en        <= 0;
                    mc_odt              <= 0;
                    $display("at time %0t ZB: Wait", $realtime);
                    sdram_state          <= 254;
                    sdram_wcount         <= 8;
                    sdram_wstate         <= 102;
                end
                102: begin
                    if (phy_rddata_valid) begin
                        sdram_buffer <= phy_rd_data;
                        sdram_state  <= 103;
                    end
                end
                103: begin
                    sdram_state <= 60;
                end

                // tRFC REFRESH SEQUENCE
                //
                // "The DRAM requires REFRESH cycles at an average interval of 7.8?s
                // (maximum when TC ? 85C or 3.9?s maximum when TC ? 95C). The REFRESH period
                // begins when the REFRESH command is registered and ends tRFC (MIN) later.
                // To allow for improved efficiency in scheduling and switching between tasks, some flexibility
                // in the absolute refresh interval is provided. A maximum of eight REFRESH commands
                // can be posted to any given DRAM, meaning that the maximum absolute interval
                // between any REFRESH command and the next REFRESH command is nine times the
                // maximum average interval refresh rate."
                //
                // also documented as: 64ms, 8192-cycle refresh up to 85C
                //                     32ms, 8192-cycle refresh up to 95C
                //
                // also: "The refresh period is 64ms when TC is less than or equal to 85C. 
                // This equates to an average refresh rate of 7.8125?s. However, nine REFRESH
                // commands should be asserted at least once every 70.3?s."
                //
                // tRFC = 160_000 ps
                // with clk running at a period of tCK * nCK_PER_CLK
                // -> 160_000/12_308 = 12 clk ticks

                120: begin
                    $display("at time %0t ZB: Time to refresh!", $realtime);
                    $display("at time %0t ZB: Precharge All Banks", $realtime);
                    mc_cmd      <= CMD_NOP;
                    mc_cmdx     <= { CMDX_NOP, CMDX_NOP, CMDX_NOP, CMDX_PREA };
                    sdram_state  <= 121;
                end
                
                121: begin
                    $display("at time %0t ZB: Clear", $realtime);
                    mc_cmd      <= CMD_NOP;
                    mc_cmdx     <= { CMDX_NOP, CMDX_NOP, CMDX_NOP, CMDX_NOP };
                    $display("at time %0t ZB: Wait", $realtime);
                    sdram_state  <= 254;
                    sdram_wcount <= 4;
                    sdram_wstate <= 122;
                end
                
                122: begin
                    $display("at time %0t ZB: Refresh", $realtime);
                    mc_cmd      <= CMD_NOP;
                    mc_cmdx     <= { CMDX_NOP, CMDX_NOP, CMDX_NOP, CMDX_REF };
                    sdram_state  <= 123;
                end
                
                123: begin
                    $display("at time %0t ZB: Clear", $realtime);
                    mc_cmd      <= CMD_NOP;
                    mc_cmdx     <= { CMDX_NOP, CMDX_NOP, CMDX_NOP, CMDX_NOP };
                    $display("at time %0t ZB: Wait", $realtime);
                    sdram_state  <= 254;
                    sdram_wcount <= 12; // tRFC
                    sdram_wstate <= 124;           
                end
                
                124: begin
                    $display("at time %0t ZB: Refreshed", $realtime);
                    sdram_state  <= 60; // back to idle
                    sdram_rcount <= 0;
                end
                                
                // wait for test_wcount clk ticks then jump to test_wstate
                254: begin
                    if (sdram_wcount != 0) sdram_wcount <= sdram_wcount - 1; 
                    else sdram_state <= sdram_wstate;
                end
                
            endcase
        end
    end    

    sdram #(
        // can we get proper numbers to generate 666 *and* 200 MHz here? no :(
        // 100 * 20/1 = 2000 - err: outside 800-1600MHz range!
        // see https://support.xilinx.com/s/article/43876
        // see https://forum.digilent.com/topic/20363-arty-a7-100-mig-route-design-clock-error/
        // The Arty board does not include the onboard oscillators that would be required to
        // directly use off-chip clocks to clock the MIG for highest DDR speeds. We *need* to
        // generate the 200MHz clock independently. 
        /*
        .CLKFBOUT_MULT  (20),
        .DIVCLK_DIVIDE  (1),
        .CLKOUT0_DIVIDE (3),   // 2000 / 3  = 666.67MHz 
        .CLKOUT1_DIVIDE (6),   // 2000 / 6  = 333.33MHz
        .CLKOUT2_DIVIDE (96),  // 2000 / 96 =  20.83MHz
        .CLKOUT3_DIVIDE (24)   // 2000 / 24 =  83.33MHz
        */
        
        .SIM_BYPASS_INIT_CAL (SIM_BYPASS_INIT_CAL),
        .CPU_DIVIDE (CPU_DIVIDE)
    )
    SDRAMi (
        .sys_rst (~RST_IN), // active-low
        .clk_ref_i (CLKREF),
        .sys_clk_i (CLKSYS),
        
        .clkd (clkd),
        .clk0 (CLK0),
        .clk2 (CLK2),
        .mmcm_locked (sdram_locked),
        .init_calib_complete (init_calib_complete),
    
        .idle (idle),
        .error (/*error*/),
        .rst_tg_mc (rst_tg_mc),
    
        .mc_ras_n (mc_ras_n),
        .mc_cas_n (mc_cas_n),
        .mc_we_n (mc_we_n),
        .mc_address (mc_address),
        .mc_bank (mc_bank),
        .mc_cke (mc_cke),
        .mc_odt (mc_odt),
        .mc_cs_n (mc_cs_n),
        .mc_reset_n (mc_reset_n),
        .mc_wrdata (mc_wrdata),
        .mc_wrdata_mask (mc_wrdata_mask),
        .mc_wrdata_en (mc_wrdata_en),
        .mc_ref_zq_wip (mc_ref_zq_wip),
        .mc_cmd_wren (mc_cmd_wren),
        .mc_ctl_wren (mc_ctl_wren),
        .mc_cmd (mc_cmd),
        .mc_cas_slot (mc_cas_slot),
        .mc_data_offset (mc_data_offset_0),
        .mc_data_offset_1 (mc_data_offset_1),
        .mc_data_offset_2 (mc_data_offset_2),
        .mc_aux_out0 (mc_aux_out0),
        .mc_aux_out1 (mc_aux_out1),
        .mc_rank_cnt (mc_rank_cnt),
        
        .phy_mc_ctl_full (phy_mc_ctl_full),
        .phy_mc_cmd_full (phy_mc_cmd_full),
        .phy_mc_data_full (phy_mc_data_full),
        .phy_rd_data (phy_rd_data),
        .phy_rddata_valid (phy_rddata_valid),
        
        .ddr3_reset_n (DDR3_RST_N),
        .ddr3_ck_p_fpga (DDR3_CK_P),
        .ddr3_ck_n_fpga (DDR3_CK_N),
        .ddr3_cs_n_fpga (DDR3_CS_N),
        .ddr3_ras_n_fpga (DDR3_RAS_N),
        .ddr3_cas_n_fpga (DDR3_CAS_N),
        .ddr3_we_n_fpga (DDR3_WE_N),
        .ddr3_ba_fpga (DDR3_BA),
        .ddr3_addr_fpga (DDR3_ADDR),
        .ddr3_cke_fpga (DDR3_CKE),
        .ddr3_odt_fpga (DDR3_ODT),
        .ddr3_dm_fpga (DDR3_DM),
        .ddr3_dq_fpga (DDR3_DQ),
        .ddr3_dqs_p_fpga (DDR3_DQS_P),
        .ddr3_dqs_n_fpga (DDR3_DQS_N),        
    
        .calib_rd_data_offset_0 (calib_rd_data_offset_0),
        .calib_rd_data_offset_1 (calib_rd_data_offset_1),
        .calib_rd_data_offset_2 (calib_rd_data_offset_2)
    );

endmodule