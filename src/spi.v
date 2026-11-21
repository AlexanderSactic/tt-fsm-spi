// Maestro SPI modo 0, un byte por transferencia.  SCK = clk / (2*CLK_DIV)
// Version optimizada en area: contador de divisor del ancho justo
// (sin contador cuando CLK_DIV = 1) y sin el bit 7 de sh_tx (nunca se leia).
module spi #(parameter CLK_DIV = 4)(
    input            clk, rst_n,
    input            start,        // pulso: inicia una transferencia (solo si !busy)
    input      [7:0] tx_data,
    output reg [7:0] rx_data,      // valido cuando done=1
    output reg       busy,
    output reg       done,         // pulso de 1 ciclo
    output reg       sck,
    output           mosi,
    input            miso
);
    localparam CW = (CLK_DIV > 1) ? $clog2(CLK_DIV) : 1;

    // 'tick' = fin de medio periodo de SCK
    wire tick;
    generate
        if (CLK_DIV > 1) begin : g_div
            /* verilator lint_off WIDTHTRUNC */
            localparam [CW-1:0] LAST = CLK_DIV - 1;
            /* verilator lint_on WIDTHTRUNC */
            reg [CW-1:0] cnt;
            assign tick = (cnt == LAST);
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)             cnt <= {CW{1'b0}};
                else if (!busy || tick) cnt <= {CW{1'b0}};
                else                    cnt <= cnt + 1'b1;
            end
        end else begin : g_nodiv
            assign tick = 1'b1;
        end
    endgenerate

    reg [3:0] bitcnt;
    reg [6:0] sh_tx;
    reg [7:0] sh_rx;
    reg       mosi_r;

    assign mosi = mosi_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck <= 1'b0; busy <= 1'b0; done <= 1'b0; bitcnt <= 4'd0;
            sh_tx <= 7'd0; sh_rx <= 8'd0; rx_data <= 8'd0; mosi_r <= 1'b0;
        end else begin
            done <= 1'b0;
            if (!busy) begin
                sck <= 1'b0; bitcnt <= 4'd0;
                if (start) begin
                    sh_tx  <= tx_data[6:0];
                    mosi_r <= tx_data[7];      // b7 listo antes del 1er flanco de subida
                    busy   <= 1'b1;
                end
            end else if (tick) begin
                sck <= ~sck;
                if (!sck) begin                              // flanco de SUBIDA
                    sh_rx  <= {sh_rx[6:0], miso};            // muestrea MISO
                    bitcnt <= bitcnt + 1'b1;
                end else begin                               // flanco de BAJADA
                    if (bitcnt == 4'd8) begin                // ya se muestrearon los 8
                        busy    <= 1'b0;
                        done    <= 1'b1;
                        rx_data <= sh_rx;
                    end else begin
                        mosi_r <= sh_tx[6];                  // siguiente bit
                        sh_tx  <= {sh_tx[5:0], 1'b0};
                    end
                end
            end
        end
    end
endmodule