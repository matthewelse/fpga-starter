/* Copyright 2017 Matthew Else */
`define ENABLE_HPS

module toplevel(
      // Analogue-digital converter
      inout              ADC_CS_N,
      output             ADC_DIN,
      input              ADC_DOUT,
      output             ADC_SCLK,

      // Audio DAC
      input              AUD_ADCDAT,
      inout              AUD_ADCLRCK,
      inout              AUD_BCLK,
      output             AUD_DACDAT,
      inout              AUD_DACLRCK,
      output             AUD_XCK,

      // Clocks
      input              CLOCK_50,
      input              CLOCK2_50,
      input              CLOCK3_50,
      input              CLOCK4_50,

      // FPGA-side SDRAM
      output      [12:0] DRAM_ADDR,
      output      [1:0]  DRAM_BA,
      output             DRAM_CAS_N,
      output             DRAM_CKE,
      output             DRAM_CLK,
      output             DRAM_CS_N,
      inout       [15:0] DRAM_DQ,
      output             DRAM_LDQM,
      output             DRAM_RAS_N,
      output             DRAM_UDQM,
      output             DRAM_WE_N,

      // Fan control (unused on native board)
      output             FAN_CTRL,

      // FPGA I2C
      output             FPGA_I2C_SCLK,
      inout              FPGA_I2C_SDAT,

      // General purpose I/O
      inout     [35:0]         GPIO_0,

      // Hex LEDs
      output      [6:0]  HEX0,
      output      [6:0]  HEX1,
      output      [6:0]  HEX2,
      output      [6:0]  HEX3,
      output      [6:0]  HEX4,
      output      [6:0]  HEX5,
		
`ifdef ENABLE_HPS
      // ARM Cortex A9 Hard Processor System
      inout              HPS_CONV_USB_N,
      output      [14:0] HPS_DDR3_ADDR,
      output      [2:0]  HPS_DDR3_BA,
      output             HPS_DDR3_CAS_N,
      output             HPS_DDR3_CKE,
      output             HPS_DDR3_CK_N,
      output             HPS_DDR3_CK_P,
      output             HPS_DDR3_CS_N,
      output      [3:0]  HPS_DDR3_DM,
      inout       [31:0] HPS_DDR3_DQ,
      inout       [3:0]  HPS_DDR3_DQS_N,
      inout       [3:0]  HPS_DDR3_DQS_P,
      output             HPS_DDR3_ODT,
      output             HPS_DDR3_RAS_N,
      output             HPS_DDR3_RESET_N,
      input              HPS_DDR3_RZQ,
      output             HPS_DDR3_WE_N,
      output             HPS_ENET_GTX_CLK,
      inout              HPS_ENET_INT_N,
      output             HPS_ENET_MDC,
      inout              HPS_ENET_MDIO,
      input              HPS_ENET_RX_CLK,
      input       [3:0]  HPS_ENET_RX_DATA,
      input              HPS_ENET_RX_DV,
      output      [3:0]  HPS_ENET_TX_DATA,
      output             HPS_ENET_TX_EN,
      inout       [3:0]  HPS_FLASH_DATA,
      output             HPS_FLASH_DCLK,
      output             HPS_FLASH_NCSO,
      inout              HPS_GSENSOR_INT,
      inout              HPS_I2C1_SCLK,
      inout              HPS_I2C1_SDAT,
      inout              HPS_I2C2_SCLK,
      inout              HPS_I2C2_SDAT,
      inout              HPS_I2C_CONTROL,
      inout              HPS_KEY,
      inout              HPS_LED,
      inout              HPS_LTC_GPIO,
      output             HPS_SD_CLK,
      inout              HPS_SD_CMD,
      inout       [3:0]  HPS_SD_DATA,
      output             HPS_SPIM_CLK,
      input              HPS_SPIM_MISO,
      output             HPS_SPIM_MOSI,
      inout              HPS_SPIM_SS,
      input              HPS_UART_RX,
      output             HPS_UART_TX,
      input              HPS_USB_CLKOUT,
      inout       [7:0]  HPS_USB_DATA,
      input              HPS_USB_DIR,
      input              HPS_USB_NXT,
      output             HPS_USB_STP,
`endif /*ENABLE_HPS*/

      // Infra-red
      input              IRDA_RXD,
      output             IRDA_TXD,

      // Push buttons on DE1-SoC mainboard
      input       [3:0]  KEY,

      // Red LED row
      output      [9:0]  LEDR

      // PS2 port
      inout              PS2_CLK,
      inout              PS2_CLK2,
      inout              PS2_DAT,
      inout              PS2_DAT2,

      // Slide switches
      input       [9:0]  SW,

      // TMDS
      input              TD_CLK27,
      input      [7:0]  TD_DATA,
      input             TD_HS,
      output             TD_RESET_N,
      input             TD_VS,

      // VGA video
      output      [7:0]  VGA_B,
      output             VGA_BLANK_N,
      output             VGA_CLK,
      output      [7:0]  VGA_G,
      output             VGA_HS,
      output      [7:0]  VGA_R,
      output             VGA_SYNC_N,
      output             VGA_VS,
);

// code goes here
logic [27:0] count;

always @(posedge CLOCK_50) begin
	count <= count + 1;
end

assign LEDR[1:0] = SW[1:0];


starter u0 (
  .clk_clk                (CLOCK_50),                      //             clk.clk
  .leds_connection_export (LEDR[9:2]),                     // leds_connection.export
  .memory_mem_a           (HPS_DDR3_ADDR),                 //          memory.mem_a
  .memory_mem_ba          (HPS_DDR3_BA),                   //                .mem_ba
  .memory_mem_ck          (HPS_DDR3_CK_P),                 //                .mem_ck
  .memory_mem_ck_n        (HPS_DDR3_CK_N),                 //                .mem_ck_n
  .memory_mem_cke         (HPS_DDR3_CKE),                  //                .mem_cke
  .memory_mem_cs_n        (HPS_DDR3_CS_N),                 //                .mem_cs_n
  .memory_mem_ras_n       (HPS_DDR3_RAS_N),                //                .mem_ras_n
  .memory_mem_cas_n       (HPS_DDR3_CAS_N),                //                .mem_cas_n
  .memory_mem_we_n        (HPS_DDR3_WE_N),                 //                .mem_we_n
  .memory_mem_reset_n     (HPS_DDR3_RESET_N),              //                .mem_reset_n
  .memory_mem_dq          (HPS_DDR3_DQ),                   //                .mem_dq
  .memory_mem_dqs         (HPS_DDR3_DQS_P),                //                .mem_dqs
  .memory_mem_dqs_n       (HPS_DDR3_DQS_N),                //                .mem_dqs_n
  .memory_mem_odt         (HPS_DDR3_ODT),                  //                .mem_odt
  .memory_mem_dm          (HPS_DDR3_DM),                   //                .mem_dm
  .memory_oct_rzqin       (HPS_DDR3_RZQ),                  //                .oct_rzqin
  .reset_reset_n          (1),                             //           reset.reset_n
  .hps_io_hps_io_emac1_inst_TX_CLK (HPS_ENET_GTX_CLK),     //          hps_io.hps_io_emac1_inst_TX_CLK
  .hps_io_hps_io_emac1_inst_TXD0   (HPS_ENET_TX_DATA[0]),  //                .hps_io_emac1_inst_TXD0
  .hps_io_hps_io_emac1_inst_TXD1   (HPS_ENET_TX_DATA[1]),  //                .hps_io_emac1_inst_TXD1
  .hps_io_hps_io_emac1_inst_TXD2   (HPS_ENET_TX_DATA[2]),  //                .hps_io_emac1_inst_TXD2
  .hps_io_hps_io_emac1_inst_TXD3   (HPS_ENET_TX_DATA[3]),  //                .hps_io_emac1_inst_TXD3
  .hps_io_hps_io_emac1_inst_RXD0   (HPS_ENET_RX_DATA[0]),  //                .hps_io_emac1_inst_RXD0
  .hps_io_hps_io_emac1_inst_MDIO   (HPS_ENET_MDIO),        //                .hps_io_emac1_inst_MDIO
  .hps_io_hps_io_emac1_inst_MDC    (HPS_ENET_MDC),         //                .hps_io_emac1_inst_MDC
  .hps_io_hps_io_emac1_inst_RX_CTL (HPS_ENET_RX_DV),       //                .hps_io_emac1_inst_RX_CTL
  .hps_io_hps_io_emac1_inst_TX_CTL (HPS_ENET_TX_EN),       //                .hps_io_emac1_inst_TX_CTL
  .hps_io_hps_io_emac1_inst_RX_CLK (HPS_ENET_RX_CLK),      //                .hps_io_emac1_inst_RX_CLK
  .hps_io_hps_io_emac1_inst_RXD1   (HPS_ENET_RX_DATA[1]),  //                .hps_io_emac1_inst_RXD1
  .hps_io_hps_io_emac1_inst_RXD2   (HPS_ENET_RX_DATA[2]),  //                .hps_io_emac1_inst_RXD2
  .hps_io_hps_io_emac1_inst_RXD3   (HPS_ENET_RX_DATA[3]),  //                .hps_io_emac1_inst_RXD3
  .hps_io_hps_io_sdio_inst_CMD     (HPS_SD_CMD),           //                .hps_io_sdio_inst_CMD
  .hps_io_hps_io_sdio_inst_D0      (HPS_SD_DATA[0]),       //                .hps_io_sdio_inst_D0
  .hps_io_hps_io_sdio_inst_D1      (HPS_SD_DATA[1]),       //                .hps_io_sdio_inst_D1
  .hps_io_hps_io_sdio_inst_CLK     (HPS_SD_CLK),           //                .hps_io_sdio_inst_CLK
  .hps_io_hps_io_sdio_inst_D2      (HPS_SD_DATA[2]),       //                .hps_io_sdio_inst_D2
  .hps_io_hps_io_sdio_inst_D3      (HPS_SD_DATA[3]),       //                .hps_io_sdio_inst_D3
  .hps_io_hps_io_usb1_inst_D0      (HPS_USB_DATA[0]),      //                .hps_io_usb1_inst_D0
  .hps_io_hps_io_usb1_inst_D1      (HPS_USB_DATA[1]),      //                .hps_io_usb1_inst_D1
  .hps_io_hps_io_usb1_inst_D2      (HPS_USB_DATA[2]),      //                .hps_io_usb1_inst_D2
  .hps_io_hps_io_usb1_inst_D3      (HPS_USB_DATA[3]),      //                .hps_io_usb1_inst_D3
  .hps_io_hps_io_usb1_inst_D4      (HPS_USB_DATA[4]),      //                .hps_io_usb1_inst_D4
  .hps_io_hps_io_usb1_inst_D5      (HPS_USB_DATA[5]),      //                .hps_io_usb1_inst_D5
  .hps_io_hps_io_usb1_inst_D6      (HPS_USB_DATA[6]),      //                .hps_io_usb1_inst_D6
  .hps_io_hps_io_usb1_inst_D7      (HPS_USB_DATA[7]),      //                .hps_io_usb1_inst_D7
  .hps_io_hps_io_usb1_inst_CLK     (HPS_USB_CLKOUT),       //                .hps_io_usb1_inst_CLK
  .hps_io_hps_io_usb1_inst_STP     (HPS_USB_STP),          //                .hps_io_usb1_inst_STP
  .hps_io_hps_io_usb1_inst_DIR     (HPS_USB_DIR),          //                .hps_io_usb1_inst_DIR
  .hps_io_hps_io_usb1_inst_NXT     (HPS_USB_NXT),          //                .hps_io_usb1_inst_NXT
  .hps_io_hps_io_uart0_inst_RX     (HPS_UART_RX),          //                .hps_io_uart0_inst_RX
  .hps_io_hps_io_uart0_inst_TX     (HPS_UART_TX)           //                .hps_io_uart0_inst_TX
);



endmodule

  