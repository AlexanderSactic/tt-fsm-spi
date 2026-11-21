/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

 //   ui_in[0]  MISO  (compartido: RFM95 y BME680)
//   ui_in[1]  DIO0  RFM95 (reservado, aun no usado)
//   ui_in[2]  WAKE
//   ui_in[7:3] reservados
//
//   uo_out[0] SCK
//   uo_out[1] MOSI
//   uo_out[2] CS_RF_N  (NSS del RFM95)
//   uo_out[3] CS_BM_N  (CS del BME680)
//   uo_out[4] RF_RESET (reservado, en 0; verificar polaridad en el datasheet)
//   uo_out[5] ACTIVE   (1 mientras el nodo no duerme; para medir consumo)
//   uo_out[7:6] reservados (UART TX de depuracion)

`default_nettype none

(* blackbox *) (* keep *)
module my_logo ();
endmodule


module tt_um_alexandersactic_spi (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    // reloj externo de 32.768 kHz, periodo de 10 minutos
    localparam CLK_HZ    = 32768;
    parameter PERIOD_S  = 600;
    localparam NODE_ID   = 8'h01;

    wire cs_rf_n, cs_bm_n, sck, mosi, active;

    node_fsm #(
        .CLK_HZ(CLK_HZ), .PERIOD_S(PERIOD_S), .CLK_DIV(1), .CS_GAP(2),
        .NODE_ID(NODE_ID), .PWRUP_CYC(400), .POLL_GAP(32), .POLL_MAX(200)
    ) u_node (
        .clk(clk), .rst_n(rst_n),
        .cs_rf_n(cs_rf_n), .cs_bm_n(cs_bm_n), .sck(sck), .mosi(mosi),
        .miso(ui_in[0]), .active(active)
    );

    (* keep *)
    my_logo logo();

    assign uo_out  = {2'b00, active, 1'b0, cs_bm_n, cs_rf_n, mosi, sck};
    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;           // todos los uio como entradas

    wire _unused = &{ena, ui_in[7:1], uio_in, 1'b0};
endmodule
