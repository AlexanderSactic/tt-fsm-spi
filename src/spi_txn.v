// Capa de transaccion SPI v3: maneja CS y encadena bytes sobre spi.
//   CS bajo -> byte "cmd" -> len bytes de datos -> CS alto
// Escritura: el byte de datos k sale de 'tx_byte' cuando tx_idx == k.
//   'tx_byte' debe ser una funcion COMBINACIONAL de 'tx_idx'.
// Lectura: cada byte recibido sale por rx_byte con rx_valid=1.
//
// REQUISITO (nuevo en v3, para ahorrar area): 'cmd' y 'len' deben
// mantenerse estables desde 'start' hasta 'done'. Ya no se copian a
// registros internos.  rx_byte es el dato del maestro (estable mientras
// rx_valid=1 y hasta el siguiente byte).
//   RFM95 (SX1276): escritura = 8'h80 | dir ; lectura = dir
//   BME680 (SPI)  : lectura   = 8'h80 | dir ; escritura = dir
module spi_txn #(
    parameter CLK_DIV = 4,
    parameter CS_GAP  = 4      // ciclos de clk entre CS y el primer/ultimo SCK
)(
    input            clk, rst_n,
    input            start,    // pulso de 1 ciclo, solo si busy=0
    input      [7:0] cmd,
    input      [3:0] len,      // bytes de datos despues de cmd (0..15)
    input      [7:0] tx_byte,  // dato a enviar (combinacional respecto a tx_idx)
    output     [3:0] tx_idx,   // indice del byte de datos que se esta cargando
    output reg       busy,
    output reg       done,     // pulso de 1 ciclo al terminar (CS ya en alto)
    output reg       rx_valid, // pulso por cada byte de datos recibido
    output     [7:0] rx_byte,
    output reg [3:0] rx_idx,
    output reg       cs_n,
    output           sck,
    output           mosi,
    input            miso
);
    localparam IDLE = 2'd0, SETUP = 2'd1, WAITB = 2'd2, HOLD = 2'd3;
    localparam GW = (CS_GAP > 1) ? $clog2(CS_GAP) : 1;
    /* verilator lint_off WIDTHTRUNC */
    localparam [GW-1:0] GAP_LAST = CS_GAP - 1;
    /* verilator lint_on WIDTHTRUNC */

    reg [1:0]    st;
    reg [3:0]    n;          // n = bytes de datos ya recibidos
    reg          is_cmd;
    reg [GW-1:0] gap;

    // Si acaba de terminar el comando toca el dato 0;
    // si acaba de terminar el dato n, toca el n+1.
    assign tx_idx = is_cmd ? 4'd0 : (n + 4'd1);

    reg        m_start;
    reg  [7:0] m_tx;
    wire [7:0] m_rx;
    wire       m_done;

    assign rx_byte = m_rx;

    spi #(.CLK_DIV(CLK_DIV)) M (
        .clk(clk), .rst_n(rst_n), .start(m_start), .tx_data(m_tx),
        /* verilator lint_off PINCONNECTEMPTY */
        .rx_data(m_rx), .busy(), .done(m_done),
        /* verilator lint_on PINCONNECTEMPTY */
        .sck(sck), .mosi(mosi), .miso(miso));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= IDLE; cs_n <= 1'b1; busy <= 1'b0; done <= 1'b0;
            rx_valid <= 1'b0; rx_idx <= 4'd0;
            m_start <= 1'b0; m_tx <= 8'h00;
            n <= 4'd0; is_cmd <= 1'b0; gap <= {GW{1'b0}};
        end else begin
            done <= 1'b0; rx_valid <= 1'b0; m_start <= 1'b0;
            case (st)
                IDLE: begin
                    cs_n <= 1'b1;
                    if (start) begin
                        n <= 4'd0; is_cmd <= 1'b1; gap <= {GW{1'b0}};
                        busy <= 1'b1; cs_n <= 1'b0; st <= SETUP;
                    end
                end
                SETUP: begin                       // CS ya bajo: esperar setup
                    if (gap == GAP_LAST) begin
                        gap <= {GW{1'b0}}; m_tx <= cmd; m_start <= 1'b1; st <= WAITB;
                    end else gap <= gap + 1'b1;
                end
                WAITB: if (m_done) begin
                    if (is_cmd) begin              // termino el byte de comando
                        is_cmd <= 1'b0;
                        if (len == 4'd0) begin gap <= {GW{1'b0}}; st <= HOLD; end
                        else begin m_tx <= tx_byte; m_start <= 1'b1; end   // dato 0
                    end else begin                 // termino un byte de datos
                        rx_idx <= n; rx_valid <= 1'b1;
                        n <= n + 4'd1;
                        if (n + 4'd1 == len) begin gap <= {GW{1'b0}}; st <= HOLD; end
                        else begin m_tx <= tx_byte; m_start <= 1'b1; end   // dato n+1
                    end
                end
                HOLD: begin                        // hold antes de subir CS
                    if (gap == GAP_LAST) begin
                        cs_n <= 1'b1; busy <= 1'b0; done <= 1'b1; st <= IDLE;
                    end else gap <= gap + 1'b1;
                end
            endcase
        end
    end
endmodule