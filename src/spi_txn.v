// Capa de transaccion SPI v2: maneja CS y encadena bytes sobre spi_master.
//   CS bajo -> byte "cmd" -> len bytes de datos -> CS alto
// Escritura: el byte de datos k sale de 'tx_byte' cuando tx_idx == k.
//   'tx_byte' debe ser una funcion COMBINACIONAL de 'tx_idx' (p. ej. buffer[tx_idx]).
//   Para escribir un solo registro basta tx_byte = valor constante y len = 1.
// Lectura: tx_byte = 0; cada byte recibido sale por rx_byte con rx_valid=1
//   y su posicion (0..len-1) en rx_idx.
// El byte 'cmd' lo arma quien llama (ya incluye el bit de lectura/escritura):
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
    output reg [7:0] rx_byte,
    output reg [3:0] rx_idx,
    output reg       cs_n,
    output           sck,
    output           mosi,
    input            miso
);
    localparam IDLE = 2'd0, SETUP = 2'd1, WAITB = 2'd2, HOLD = 2'd3;

    reg [1:0] st;
    reg [7:0] cmd_r;
    reg [3:0] len_r, n;       // n = bytes de datos ya recibidos
    reg       is_cmd;
    reg [7:0] gap;

    // Cuando se carga un byte de datos nuevo: si acaba de terminar el comando
    // toca el dato 0; si acaba de terminar el dato n, toca el n+1.
    assign tx_idx = is_cmd ? 4'd0 : (n + 4'd1);

    reg        m_start;
    reg  [7:0] m_tx;
    wire [7:0] m_rx;
    wire       m_busy, m_done;

    spi #(.CLK_DIV(CLK_DIV)) M (
        .clk(clk), .rst_n(rst_n), .start(m_start), .tx_data(m_tx),
        .rx_data(m_rx), .busy(m_busy), .done(m_done),
        .sck(sck), .mosi(mosi), .miso(miso));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= IDLE; cs_n <= 1'b1; busy <= 1'b0; done <= 1'b0;
            rx_valid <= 1'b0; rx_byte <= 8'h00; rx_idx <= 4'd0;
            m_start <= 1'b0; m_tx <= 8'h00;
            cmd_r <= 8'h00; len_r <= 4'd0; n <= 4'd0;
            is_cmd <= 1'b0; gap <= 8'd0;
        end else begin
            done <= 1'b0; rx_valid <= 1'b0; m_start <= 1'b0;
            case (st)
                IDLE: begin
                    cs_n <= 1'b1;
                    if (start) begin
                        cmd_r <= cmd; len_r <= len;
                        n <= 4'd0; is_cmd <= 1'b1; gap <= 8'd0;
                        busy <= 1'b1; cs_n <= 1'b0; st <= SETUP;
                    end
                end
                SETUP: begin                       // CS ya bajo: esperar setup
                    if (gap == CS_GAP-1) begin
                        gap <= 8'd0; m_tx <= cmd_r; m_start <= 1'b1; st <= WAITB;
                    end else gap <= gap + 8'd1;
                end
                WAITB: if (m_done) begin
                    if (is_cmd) begin              // termino el byte de comando
                        is_cmd <= 1'b0;
                        if (len_r == 4'd0) begin gap <= 8'd0; st <= HOLD; end
                        else begin m_tx <= tx_byte; m_start <= 1'b1; end   // dato 0
                    end else begin                 // termino un byte de datos
                        rx_byte <= m_rx; rx_idx <= n; rx_valid <= 1'b1;
                        n <= n + 4'd1;
                        if (n + 1 == len_r) begin gap <= 8'd0; st <= HOLD; end
                        else begin m_tx <= tx_byte; m_start <= 1'b1; end   // dato n+1
                    end
                end
                HOLD: begin                        // hold antes de subir CS
                    if (gap == CS_GAP-1) begin
                        cs_n <= 1'b1; busy <= 1'b0; done <= 1'b1; st <= IDLE;
                    end else gap <= gap + 8'd1;
                end
            endcase
        end
    end
endmodule