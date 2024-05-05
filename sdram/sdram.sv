`timescale 1ps / 1ps
//`default_nettype none

module sdram #(
    `include "sdram.vh"

    ,parameter CPU_DIVIDE = 1
)(
    input  wire             sys_rst,
    input  wire             clk_ref_i, // 200 MHz
    input  wire             sys_clk_i, // 100 MHz
    
    output wire             clkd,
    output wire             clk0,
    output wire             clk2,
    output wire             mmcm_locked,
    output wire             init_calib_complete,

    input  wire             idle,
    input  wire             error,
    output wire             rst_tg_mc,
    
    input wire [3:0]        mc_ras_n, // [nCK_PER_CLK-1:0]
    input wire [3:0]        mc_cas_n, // [nCK_PER_CLK-1:0]
    input wire [3:0]        mc_we_n, // [nCK_PER_CLK-1:0]
    input wire [55:0]       mc_address, // [nCK_PER_CLK*ROW_WIDTH-1:0]
    input wire [11:0]       mc_bank, // [nCK_PER_CLK*BANK_WIDTH-1:0]
    input wire [3:0]        mc_cke, // [nCK_PER_CLK-1:0]
    input wire [1:0]        mc_odt,
    input wire [3:0]        mc_cs_n, // [CS_WIDTH*nCS_PER_RANK*nCK_PER_CLK-1:0]
    input wire              mc_reset_n,
    input wire [127:0]      mc_wrdata, // [2*nCK_PER_CLK*DQ_WIDTH-1:0]
    input wire [15:0]       mc_wrdata_mask, // [2*nCK_PER_CLK*DQ_WIDTH/8-1:0]
    input wire              mc_wrdata_en,
    input wire              mc_ref_zq_wip,
    input wire              mc_cmd_wren,
    input wire              mc_ctl_wren,
    input wire [2:0]        mc_cmd,
    input wire [1:0]        mc_cas_slot,
    input wire [5:0]        mc_data_offset,
    input wire [5:0]        mc_data_offset_1,
    input wire [5:0]        mc_data_offset_2,
    input wire [3:0]        mc_aux_out0,
    input wire [3:0]        mc_aux_out1,
    input wire [1:0]        mc_rank_cnt,
    
    output wire             phy_mc_ctl_full,
    output wire             phy_mc_cmd_full,
    output wire             phy_mc_data_full,
    output wire [127:0]     phy_rd_data, // [2*nCK_PER_CLK*DQ_WIDTH-1:0]
    output wire             phy_rddata_valid,
    
    output wire [13:0]      ddr3_addr_fpga,
    output wire [2:0]       ddr3_ba_fpga,
    output wire             ddr3_ras_n_fpga,
    output wire             ddr3_cas_n_fpga,
    output wire             ddr3_we_n_fpga,
    output wire             ddr3_cs_n_fpga,
    output wire             ddr3_odt_fpga,
    output wire             ddr3_cke_fpga,
    output wire             ddr3_ck_p_fpga,
    output wire             ddr3_ck_n_fpga,
    inout  wire [15:0]      ddr3_dq_fpga,
    inout  wire [1:0]       ddr3_dm_fpga,
    inout  wire [1:0]       ddr3_dqs_p_fpga,
    inout  wire [1:0]       ddr3_dqs_n_fpga,
    
    output wire             ddr3_reset_n,
    
    output wire [5:0]       calib_rd_data_offset_0,
    output wire [5:0]       calib_rd_data_offset_1,
    output wire [5:0]       calib_rd_data_offset_2

    
`ifdef SKIP_CALIB
    ,output wire             calib_tap_req
    ,input  wire             calib_tap_load
    ,input  wire[6:0]        calib_tap_addr
    ,input  wire[7:0]        calib_tap_val
    ,input  wire             calib_tap_load_done
`endif

);

    localparam CLK_PERIOD = tCK * nCK_PER_CLK;
    localparam nCL  = CL;
    localparam CWL_T = (DRAM_TYPE == "DDR3") ? CWL : CL-1;
    localparam nCWL = CWL_T;
    localparam TAPSPERKCLK = (56*MMCM_MULT_F)/nCK_PER_CLK;
    localparam IODELAY_GRP = (tCK <= 1500)? IODELAY_GRP1 : IODELAY_GRP0;
    
`ifdef SKIP_CALIB
    localparam SKIP_CALIB = "TRUE";
`else
    localparam SKIP_CALIB = "FALSE";
`endif
   
    localparam TEMP_MON_EN = "ON";
    localparam XADC_CLK_PERIOD = 5000;
    localparam tTEMPSAMPLE = 10000000;
    localparam RESET_PERIOD = 200000; // ps
        
    localparam PRE_REV3ES = "OFF";   // Delay O/Ps using Phaser_Out fine dly
    
    localparam real REFCLK_PERIOD = (1000000.0/(2*REFCLK_FREQ));

    
    // --------
    
    
    wire ddr3_parity;               // PHY -> .
       
    //wire clk;                       // INFRA -> . -> TEMPMON
    wire rst;                       // INFRA -> . -> TEMPMON
    wire clk_div2;                  // INFRA -> . -> PHY
    wire rst_div2;                  // INFRA -> . -> PHY

    wire mem_refclk;                // INFRA -> . -> PHY
    wire freq_refclk;               // INFRA -> . -> PHY
    wire sync_pulse;                // INFRA -> . -> PHY
    wire mmcm_ps_clk;               // INFRA -> . -> PHY
    wire poc_sample_pd;             // INFRA -> . -> PHY
    wire pll_locked;                // INFRA -> . -> PHY
    wire iddr_rst;                  // INFRA -> . -> PHY
    wire ref_dll_lock;              // INFRA -> . -> PHY
    wire rst_phaser_ref;            // INFRA -> . -> PHY
    wire psdone;                    // INFRA -> . -> PHY    
    
    wire psen;                      // PHY -> . -> INFRA 
    wire psincdec;                  // PHY -> . -> INFRA

    wire mmcm_clk;                  // CLKIBUF -> . -> INFRA
    wire [1:0] iodelay_ctrl_rdy;    // IODELAY_CTRL -> . -> INFRA
    wire sys_rst_o;                 // IODELAY_CTRL -> . -> INFRA
    
    wire [1:0] clk_ref;             // IODELAY_CTRL -> . ->
                                    //                 .[0] -> TEMPMONi

    wire [11:0] device_temp;        // TEMPMON -> . -> PHY
    wire tempmon_sample_en;         // ?? -> . -> PHY
    
    // https://support.xilinx.com/s/article/52523
    // OFF for simulation unless we want to actually simulate the XADC
    assign tempmon_sample_en = TEMP_MON_EN == "OFF" ? 1'b0 : mc_ref_zq_wip;

    // these are used only ifdef SKIP_CALIB
    // but still, we need to define them 
    wire       calib_tap_req;       // PYH -> 
    reg        calib_tap_load;      // -> PHY
    reg  [6:0] calib_tap_addr;      // -> PHY
    reg  [7:0] calib_tap_val;       // -> PHY
    reg        calib_tap_load_done; // -> PHY
    
    wire clk_ref_p, clk_ref_n;      // const -> IODELAY_CTRL
    wire sys_clk_p, sys_clk_n;      // const -> CLKIBUF
    
    assign sys_clk_p = 1'b0;
    assign sys_clk_n = 1'b0;
    assign clk_ref_p = 1'b0;
    assign clk_ref_n = 1'b0;
    
    wire clk_ref_in;                // ( mmcm_clk | clk_ref_i ) -> IODELAY_CTRL

    generate
        if (REFCLK_TYPE == "USE_SYSTEM_CLOCK")
            assign clk_ref_in = mmcm_clk;
        else
            assign clk_ref_in = clk_ref_i;
    endgenerate
    
    wire init_wrcal_complete;       // (init_wrcal_complete_w)
    wire init_calib_complete_w;     // PHY -> . -> init_calib_complete
    wire init_wrcal_complete_w;     // PHY -> . -> init_wrcal_complete
    
    assign init_calib_complete = init_calib_complete_w;
    assign init_wrcal_complete = init_wrcal_complete_w;
    
    // -------- IODELAY_CTRL --------


    mig_7series_v4_2_iodelay_ctrl #(
        .TCQ                       (TCQ),
        .IODELAY_GRP0              (IODELAY_GRP0),
        .IODELAY_GRP1              (IODELAY_GRP1),
        .REFCLK_TYPE               (REFCLK_TYPE),
        .SYSCLK_TYPE               (SYSCLK_TYPE),
        .SYS_RST_PORT              (SYS_RST_PORT),
        .RST_ACT_LOW               (RST_ACT_LOW),
        .DIFF_TERM_REFCLK          (DIFF_TERM_REFCLK),
        .FPGA_SPEED_GRADE          (FPGA_SPEED_GRADE),
        .REF_CLK_MMCM_IODELAY_CTRL (REF_CLK_MMCM_IODELAY_CTRL)
    )
    IODELAY_CTRLi 
    (
        // in
        .clk_ref_p          (clk_ref_p),
        .clk_ref_n          (clk_ref_n),
        .clk_ref_i          (clk_ref_in),
        .sys_rst            (sys_rst),
        // out
        .iodelay_ctrl_rdy   (iodelay_ctrl_rdy),
        .sys_rst_o          (sys_rst_o),
        .clk_ref            (clk_ref)
    );


    // -------- IBUF --------


    mig_7series_v4_2_clk_ibuf #(
        .SYSCLK_TYPE        (SYSCLK_TYPE),
        .DIFF_TERM_SYSCLK   (DIFF_TERM_SYSCLK)
    )
    CLKIBUFi
    (
        // in
        .sys_clk_p          (sys_clk_p),
        .sys_clk_n          (sys_clk_n),
        .sys_clk_i          (sys_clk_i),
        // out
        .mmcm_clk           (mmcm_clk)
    );


    // -------- INFRA --------


    mig_7series_v4_2_infrastructure #(
        .TCQ                (TCQ),
        .nCK_PER_CLK        (nCK_PER_CLK),
        .CLKIN_PERIOD       (CLKIN_PERIOD),
        .SYSCLK_TYPE        (SYSCLK_TYPE),
        .CLKFBOUT_MULT      (CLKFBOUT_MULT),
        .DIVCLK_DIVIDE      (DIVCLK_DIVIDE),
        .CLKOUT0_PHASE      (CLKOUT0_PHASE),
        .CLKOUT0_DIVIDE     (CLKOUT0_DIVIDE),
        .CLKOUT1_DIVIDE     (CLKOUT1_DIVIDE),
        .CLKOUT2_DIVIDE     (CLKOUT2_DIVIDE),
        .CLKOUT3_DIVIDE     (CLKOUT3_DIVIDE),
        .MMCM_VCO           (MMCM_VCO),
        .MMCM_MULT_F        (MMCM_MULT_F),
        .MMCM_DIVCLK_DIVIDE (MMCM_DIVCLK_DIVIDE),
        .RST_ACT_LOW        (RST_ACT_LOW),
        .tCK                (tCK),
        .MEM_TYPE           (DRAM_TYPE),
        
        .MMCM_CLKOUT0_DIVIDE (CPU_DIVIDE)
    )
    INFRAi 
    (
        // in
        .mmcm_clk           (mmcm_clk),           // System clock diff input
        .sys_rst            (sys_rst_o),          // core reset from user application
        .iodelay_ctrl_rdy   (iodelay_ctrl_rdy),   // PLLE2/IDELAYCTRL lock status (),
        .ref_dll_lock       (ref_dll_lock),
        .psen               (psen),
        .psincdec           (psincdec),
        // out
        .clk                (clkd),               // fabric clock freq ; either half rate or quarter rate and is determined by  PLL parameters settings.
        .clk_div2           (clk_div2),           // mem_refclk divided by 2 for PI incdec
        .rst_div2           (rst_div2),           // reset in clk_div2 domain
        .mem_refclk         (mem_refclk),         // equal to  memory clock
        .freq_refclk        (freq_refclk),        // freq above 400 MHz:  set freq_refclk = mem_refclk
        .sync_pulse         (sync_pulse),         // exactly 1/16 of mem_refclk and the sync pulse is exactly 1 memref_clk wide
        .mmcm_ps_clk        (mmcm_ps_clk),        // Phase shift clock
        .poc_sample_pd      (poc_sample_pd),      // Tell POC when to sample phase detector output.
        .ui_addn_clk_0      (/**/),               // MMCM out0 clk
        .ui_addn_clk_1      (/**/),               // MMCM out1 clk
        .ui_addn_clk_2      (clk0),               // MMCM out2 clk
        .ui_addn_clk_3      (clk2),               // MMCM out3 clk
        .ui_addn_clk_4      (/**/),               // MMCM out4 clk
        .pll_locked         (pll_locked),         // locked output from PLLE2_ADV
        .mmcm_locked        (mmcm_locked),        // locked output from MMCME2_ADV
        .rstdiv0            (rst),                // Reset CLK and CLKDIV logic (incl I/O),
        .iddr_rst           (iddr_rst),
        .rst_phaser_ref     (rst_phaser_ref),
        .psdone             (psdone)
    );


    // -------- TEMPMON --------


    mig_7series_v4_2_tempmon #(
        .TCQ                (TCQ),              // Register delay (sim only)
        .TEMP_MON_CONTROL   (TEMP_MON_CONTROL), // XADC or user temperature source
        .XADC_CLK_PERIOD    (XADC_CLK_PERIOD),  // pS (default to 200 MHz refclk)
        .tTEMPSAMPLE        (tTEMPSAMPLE)       // ps (sample every 10 us)
    ) 
    TEMPMONi 
    (
        // in
        .clk                (clkd),
        .xadc_clk           (clk_ref[0]),
        .rst                (rst),
        .device_temp_i      (/**/), // not used when TEMP_MON_CONTROL is INTERNAL (uses XADC)
        // out
        .device_temp        (device_temp)
    );


    // -------- PHY --------
    
        
    
    mig_7series_v4_2_ddr_phy_top #(
        .TCQ                (TCQ),
        .DDR3_VDD_OP_VOLT   (VDD_OP_VOLT),
        .REFCLK_FREQ        (REFCLK_FREQ),
        .BYTE_LANES_B0      (BYTE_LANES_B0),
        .BYTE_LANES_B1      (BYTE_LANES_B1),
        .BYTE_LANES_B2      (BYTE_LANES_B2),
        .BYTE_LANES_B3      (BYTE_LANES_B3),
        .BYTE_LANES_B4      (BYTE_LANES_B4),
        .PHY_0_BITLANES     (PHY_0_BITLANES),
        .PHY_1_BITLANES     (PHY_1_BITLANES),
        .PHY_2_BITLANES     (PHY_2_BITLANES),
        .CA_MIRROR          (CA_MIRROR),
        .CK_BYTE_MAP        (CK_BYTE_MAP),
        .ADDR_MAP           (ADDR_MAP),
        .BANK_MAP           (BANK_MAP),
        .CAS_MAP            (CAS_MAP),
        .CKE_ODT_BYTE_MAP   (CKE_ODT_BYTE_MAP),
        .CKE_MAP            (CKE_MAP),
        .ODT_MAP            (ODT_MAP),
        .CKE_ODT_AUX        (CKE_ODT_AUX),
        .CS_MAP             (CS_MAP),
        .PARITY_MAP         (PARITY_MAP),
        .RAS_MAP            (RAS_MAP),
        .WE_MAP             (WE_MAP),
        .DQS_BYTE_MAP       (DQS_BYTE_MAP),
        .DATA0_MAP          (DATA0_MAP),
        .DATA1_MAP          (DATA1_MAP),
        .DATA2_MAP          (DATA2_MAP),
        .DATA3_MAP          (DATA3_MAP),
        .DATA4_MAP          (DATA4_MAP),
        .DATA5_MAP          (DATA5_MAP),
        .DATA6_MAP          (DATA6_MAP),
        .DATA7_MAP          (DATA7_MAP),
        .DATA8_MAP          (DATA8_MAP),
        .DATA9_MAP          (DATA9_MAP),
        .DATA10_MAP         (DATA10_MAP),
        .DATA11_MAP         (DATA11_MAP),
        .DATA12_MAP         (DATA12_MAP),
        .DATA13_MAP         (DATA13_MAP),
        .DATA14_MAP         (DATA14_MAP),
        .DATA15_MAP         (DATA15_MAP),
        .DATA16_MAP         (DATA16_MAP),
        .DATA17_MAP         (DATA17_MAP),
        .MASK0_MAP          (MASK0_MAP),
        .MASK1_MAP          (MASK1_MAP),
        .CALIB_ROW_ADD      (CALIB_ROW_ADD),
        .CALIB_COL_ADD      (CALIB_COL_ADD),
        .CALIB_BA_ADD       (CALIB_BA_ADD),
        .nCS_PER_RANK       (nCS_PER_RANK),
        .CS_WIDTH           (CS_WIDTH),
        .nCK_PER_CLK        (nCK_PER_CLK),
        .PRE_REV3ES         (PRE_REV3ES),
        .CKE_WIDTH          (CKE_WIDTH),
        .DATA_CTL_B0        (DATA_CTL_B0),
        .DATA_CTL_B1        (DATA_CTL_B1),
        .DATA_CTL_B2        (DATA_CTL_B2),
        .DATA_CTL_B3        (DATA_CTL_B3),
        .DATA_CTL_B4        (DATA_CTL_B4),
        .DDR2_DQSN_ENABLE   (DDR2_DQSN_ENABLE),
        .DRAM_TYPE          (DRAM_TYPE),
        .BANK_WIDTH         (BANK_WIDTH),
        .CK_WIDTH           (CK_WIDTH),
        .COL_WIDTH          (COL_WIDTH),
        .DM_WIDTH           (DM_WIDTH),
        .DQ_WIDTH           (DQ_WIDTH),
        .DQS_CNT_WIDTH      (DQS_CNT_WIDTH),
        .DQS_WIDTH          (DQS_WIDTH),
        .DRAM_WIDTH         (DRAM_WIDTH),
        .PHYCTL_CMD_FIFO    (PHYCTL_CMD_FIFO),
        .ROW_WIDTH          (ROW_WIDTH),
        .AL                 (AL),
        .ADDR_CMD_MODE      (ADDR_CMD_MODE),
        .BURST_MODE         (BURST_MODE),
        .BURST_TYPE         (BURST_TYPE),
        .CL                 (nCL),
        .CWL                (nCWL),
        .tRFC               (tRFC),
        .tREFI              (tREFI),
        .tCK                (tCK),
        .OUTPUT_DRV         (OUTPUT_DRV),
        .RANKS              (RANKS),
        .ODT_WIDTH          (ODT_WIDTH),
        .REG_CTRL           (REG_CTRL),
        .RTT_NOM            (RTT_NOM),
        .RTT_WR             (RTT_WR),
        .SLOT_1_CONFIG      (SLOT_1_CONFIG),
        .WRLVL              (WRLVL),
        .BANK_TYPE          (BANK_TYPE),
        .DATA_IO_PRIM_TYPE  (DATA_IO_PRIM_TYPE),
        .DATA_IO_IDLE_PWRDWN(DATA_IO_IDLE_PWRDWN),
        .IODELAY_GRP        (IODELAY_GRP),
        .FPGA_SPEED_GRADE   (FPGA_SPEED_GRADE),
        // Prevent the following simulation-related parameters from
        // being overridden for synthesis - for synthesis only the
        // default values of these parameters should be used
        // synthesis translate_off
        .SIM_BYPASS_INIT_CAL (SIM_BYPASS_INIT_CAL),
        // synthesis translate_on
        .USE_CS_PORT        (USE_CS_PORT),
        .USE_DM_PORT        (USE_DM_PORT),
        .USE_ODT_PORT       (USE_ODT_PORT),
        .MASTER_PHY_CTL     (PHY_CONTROL_MASTER_BANK),
        .DEBUG_PORT         (DEBUG_PORT),
        .IDELAY_ADJ         (IDELAY_ADJ),
        .FINE_PER_BIT       (FINE_PER_BIT),
        .CENTER_COMP_MODE   (CENTER_COMP_MODE),
        .PI_VAL_ADJ         (PI_VAL_ADJ),
        .TAPSPERKCLK        (TAPSPERKCLK),
        .SKIP_CALIB         (SKIP_CALIB),
        .FPGA_VOLT_TYPE     (FPGA_VOLT_TYPE)
    ) 
    PHYi 
    (
        // clocks.in
        .clk                        (clkd),
        .clk_div2                   (clk_div2),
        .rst_div2                   (rst_div2),
        .clk_ref                    (clk_ref[0]), 
        .freq_refclk                (freq_refclk),
        .mem_refclk                 (mem_refclk),
        .pll_lock                   (pll_locked),
        .sync_pulse                 (sync_pulse),
        .mmcm_ps_clk                (mmcm_ps_clk),
        .poc_sample_pd              (poc_sample_pd),

        // ctrl.in
        .rst                        (rst),
        .iddr_rst                   (iddr_rst),
        .idle                       (idle),
        .error                      (error),
        .slot_0_present             (SLOT_0_CONFIG),
        .slot_1_present             (SLOT_1_CONFIG),
        .rst_phaser_ref             (rst_phaser_ref),

        // ctrl.out
        .rst_tg_mc                  (rst_tg_mc),
        .ref_dll_lock               (ref_dll_lock),

        // temp.in
        .device_temp                (device_temp),
        .tempmon_sample_en          (tempmon_sample_en),

        // calib.in
`ifdef SKIP_CALIB
        .calib_tap_addr             (calib_tap_addr),
        .calib_tap_load             (calib_tap_load),
        .calib_tap_val              (calib_tap_val),
        .calib_tap_load_done        (calib_tap_load_done),
`else
        .calib_tap_load             (1'b0),
        .calib_tap_addr             (7'b0),
        .calib_tap_val              (8'b0),
        .calib_tap_load_done        (1'b0),
`endif

        // calib.out
`ifdef SKIP_CALIB
        .calib_tap_req              (calib_tap_req),
`else
        .calib_tap_req              (),
`endif
        .calib_rd_data_offset_0     (calib_rd_data_offset_0),
        .calib_rd_data_offset_1     (calib_rd_data_offset_1),
        .calib_rd_data_offset_2     (calib_rd_data_offset_2),
        .init_calib_complete        (init_calib_complete_w),
        .init_wrcal_complete        (init_wrcal_complete_w),

        // ddr.inout
        .ddr_dqs                    (ddr3_dqs_p_fpga),
        .ddr_dqs_n                  (ddr3_dqs_n_fpga),
        .ddr_dq                     (ddr3_dq_fpga),

        // ddr.out
        .ddr_ck                     (ddr3_ck_p_fpga),
        .ddr_ck_n                   (ddr3_ck_n_fpga),
        .ddr_addr                   (ddr3_addr_fpga),
        .ddr_ba                     (ddr3_ba_fpga),
        .ddr_ras_n                  (ddr3_ras_n_fpga),
        .ddr_cas_n                  (ddr3_cas_n_fpga),
        .ddr_we_n                   (ddr3_we_n_fpga),
        .ddr_cs_n                   (ddr3_cs_n_fpga),
        .ddr_cke                    (ddr3_cke_fpga),
        .ddr_odt                    (ddr3_odt_fpga),
        .ddr_reset_n                (ddr3_reset_n),
        .ddr_parity                 (ddr3_parity),
        .ddr_dm                     (ddr3_dm_fpga),

        // phase-shift.in
        .psdone                     (psdone),

        // phase-shift.out
        .psen                       (psen),
        .psincdec                   (psincdec),

        // mc.in
        .mc_address                 (mc_address),
        .mc_aux_out0                (mc_aux_out0),
        .mc_aux_out1                (mc_aux_out1),
        .mc_bank                    (mc_bank),
        .mc_cke                     (mc_cke),
        .mc_odt                     (mc_odt),
        .mc_cas_n                   (mc_cas_n),
        .mc_cmd                     (mc_cmd),
        .mc_cmd_wren                (mc_cmd_wren),
        .mc_cas_slot                (mc_cas_slot),
        .mc_cs_n                    (mc_cs_n),
        .mc_ctl_wren                (mc_ctl_wren),
        .mc_data_offset             (mc_data_offset),
        .mc_data_offset_1           (mc_data_offset_1),
        .mc_data_offset_2           (mc_data_offset_2),
        .mc_rank_cnt                (mc_rank_cnt),
        .mc_ras_n                   (mc_ras_n),
        .mc_reset_n                 (mc_reset_n),
        .mc_we_n                    (mc_we_n),
        .mc_wrdata                  (mc_wrdata),
        .mc_wrdata_en               (mc_wrdata_en),
        .mc_wrdata_mask             (mc_wrdata_mask),

        // mc.out
        .phy_rd_data                (phy_rd_data),
        .phy_rddata_valid           (phy_rddata_valid),
        .phy_mc_ctl_full            (phy_mc_ctl_full),
        .phy_mc_cmd_full            (phy_mc_cmd_full),
        .phy_mc_data_full           (phy_mc_data_full),

        // dbg
        .dbg_idel_up_all            (),
        .dbg_idel_down_all          (),
        .dbg_idel_up_cpt            (),
        .dbg_idel_down_cpt          (),
        .dbg_sel_idel_cpt           (),
        .dbg_sel_all_idel_cpt       (),
        .dbg_calib_top              (),
        .dbg_cpt_first_edge_cnt     (),
        .dbg_cpt_second_edge_cnt    (),
        .dbg_phy_rdlvl              (),
        .dbg_phy_wrcal              (),
        .dbg_final_po_fine_tap_cnt  (),
        .dbg_final_po_coarse_tap_cnt (),
        .dbg_rd_data_edge_detect    (),
        .dbg_rddata                 (),
        .dbg_rdlvl_done             (),
        .dbg_rdlvl_err              (),
        .dbg_rdlvl_start            (),
        .dbg_tap_cnt_during_wrlvl   (),
        .dbg_wl_edge_detect_valid   (),
        .dbg_wrlvl_done             (),
        .dbg_wrlvl_err              (),
        .dbg_wrlvl_start            (),
        .dbg_pi_phase_locked_phy4lanes (),
        .dbg_pi_dqs_found_lanes_phy4lanes (),
        .dbg_sel_pi_incdec          (),
        .dbg_sel_po_incdec          (),
        .dbg_byte_sel               (),
        .dbg_pi_f_inc               (),
        .dbg_po_f_inc               (),
        .dbg_po_f_stg23_sel         (),
        .dbg_pi_f_dec               (),
        .dbg_po_f_dec               (),
        .dbg_cpt_tap_cnt            (),
        .dbg_dq_idelay_tap_cnt      (),
        .dbg_rddata_valid           (),
        .dbg_wrlvl_fine_tap_cnt     (),
        .dbg_wrlvl_coarse_tap_cnt   (),
        .dbg_phy_wrlvl              (),
        .dbg_rd_data_offset         (),
        .dbg_phy_init               (),
        .dbg_prbs_rdlvl             (),
        .dbg_dqs_found_cal          (),
        .dbg_po_counter_read_val    (),
        .dbg_pi_counter_read_val    (),
        .dbg_pi_phaselock_start     (),
        .dbg_pi_phaselocked_done    (),
        .dbg_pi_phaselock_err       (),
        .dbg_pi_dqsfound_start      (),
        .dbg_pi_dqsfound_done       (),
        .dbg_pi_dqsfound_err        (),
        .dbg_wrcal_start            (),
        .dbg_wrcal_done             (),
        .dbg_wrcal_err              (),
        .dbg_phy_oclkdelay_cal      (),
        .dbg_oclkdelay_rd_data      (),
        .dbg_oclkdelay_calib_start  (),
        .dbg_oclkdelay_calib_done   (),
        .dbg_prbs_first_edge_taps   (),
        .dbg_prbs_second_edge_taps  (),
        .dbg_poc                    (),
        
        .prbs_final_dqs_tap_cnt_r   ()
    );
    
endmodule
