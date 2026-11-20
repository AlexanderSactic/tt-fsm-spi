module spi #(parameter CLK_DIV = 4)(   // SCK = clk / (2*CLK_DIV)
    input            clk, rst_n,
    input            start,        // pulso: inicia una transferencia (solo si !busy)
    input      [7:0] tx_data,
    output reg [7:0] rx_data,      // válido cuando done=1
    output reg       busy,
    output reg       done,         // pulso de 1 ciclo
    output reg       sck,
    output           mosi,
    input            miso
);
    reg [15:0] cnt;
    reg [3:0]  bitcnt;
    reg [7:0]  sh_tx, sh_rx;
    reg        mosi_r;

    assign mosi = mosi_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck <= 0; busy <= 0; done <= 0; cnt <= 0; bitcnt <= 0;
            sh_tx <= 0; sh_rx <= 0; rx_data <= 0; mosi_r <= 0;
        end else begin
            done <= 1'b0;
            if (!busy) begin
                sck <= 1'b0; cnt <= 0; bitcnt <= 0;
                if (start) begin
                    sh_tx  <= tx_data;
                    mosi_r <= tx_data[7];      // b7 listo antes del 1er flanco de subida
                    busy   <= 1'b1;
                end
            end else begin
                if (cnt == CLK_DIV-1) begin
                    cnt <= 0;
                    sck <= ~sck;
                    if (!sck) begin                          // flanco de SUBIDA
                        sh_rx  <= {sh_rx[6:0], miso};        // muestrea MISO
                        bitcnt <= bitcnt + 1'b1;
                    end else begin                           // flanco de BAJADA
                        if (bitcnt == 4'd8) begin            // ya se muestrearon los 8
                            busy    <= 1'b0;
                            done    <= 1'b1;
                            rx_data <= sh_rx;
                        end else begin
                            mosi_r <= sh_tx[6];              // siguiente bit
                            sh_tx  <= {sh_tx[6:0], 1'b0};
                        end
                    end
                end else cnt <= cnt + 1'b1;
            end
        end
    end
endmodule