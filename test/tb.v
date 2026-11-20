`default_nettype none
`timescale 1ns / 1ps

/* Testbench de TinyTapeout: instancia el wrapper tt_um_alexandersactic_spi,
   conecta los modelos de esclavo SPI (solo simulacion) y expone señales
   simples para que cocotb (test.py) las lea.

   Los modelos (models_spi.v) viven en test/, NO en src/ ni en info.yaml.
*/
module tb ();

  // Dump FST (gtkwave / surfer)
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end

  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  // ---- Bus SPI / control (mapeo de project.v) ----
  wire miso;
  wire sck     = uo_out[0];
  wire mosi    = uo_out[1];
  wire cs_rf_n = uo_out[2];
  wire cs_bm_n = uo_out[3];
  // uo_out[4] = RF_RESET (en 0, sin usar)
  wire active  = uo_out[5];

  // miso entra por ui_in[0]; el resto de ui_in queda en 0
  wire [7:0] ui_in = {7'b0, miso};

`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif

  tt_um_alexandersactic_spi user_project (
`ifdef GL_TEST
      .VPWR(VPWR),
      .VGND(VGND),
`endif
      .ui_in  (ui_in),
      .uo_out (uo_out),
      .uio_in (uio_in),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

  // ---- Modelos de los esclavos (mismos parametros que top_tb) ----
  rfm95_model  #(.TX_TIME(3000))   RF (.sck(sck), .cs_n(cs_rf_n), .mosi(mosi), .miso(miso));
  bme680_model #(.MEAS_TIME(2000)) BM (.sck(sck), .cs_n(cs_bm_n), .mosi(mosi), .miso(miso));

  // Datos que "mide" el BME680: A0..A7
  integer i;
  initial begin
    #1;
    for (i = 0; i < 8; i = i + 1) BM.meas[i] = 8'hA0 + i;
  end

  // ---- Señales de observacion para cocotb ----
  wire [31:0] rf_tx_count = RF.tx_count;
  wire [7:0]  rf_tx_len   = RF.tx_len[0];
  wire [7:0]  rf_opmode   = RF.mem[7'h01];
  wire [7:0]  rf_irqflags = RF.mem[7'h12];
  // paquete transmitido: byte 0 = ID, 1 = SEQ, 2..9 = datos
  wire [79:0] rf_pkt = {RF.txlog[9], RF.txlog[8], RF.txlog[7], RF.txlog[6], RF.txlog[5],
                        RF.txlog[4], RF.txlog[3], RF.txlog[2], RF.txlog[1], RF.txlog[0]};

endmodule