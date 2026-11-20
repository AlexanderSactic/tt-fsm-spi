// Esclavo SPI de comportamiento (solo para simulacion), modo 0, MSB primero.
// Primer byte = comando: bit7 distingue lectura/escritura segun WRITE_FLAG,
// bits [6:0] = direccion inicial. Auto-incremento de direccion por byte.
//   RFM95 : WRITE_FLAG = 1'b1  (bit7=1 escribe)
//   BME680: WRITE_FLAG = 1'b0  (bit7=1 lee)
module spi_slave_model #(parameter WRITE_FLAG = 1'b1)(
    input  sck, cs_n, mosi,
    output miso
);
    reg [7:0] mem [0:127];
    reg [7:0] rx_sh, tx_sh, tx_nxt;
    reg [2:0] bits;
    reg       got_cmd, is_wr, miso_r;
    reg [6:0] addr;
    integer   i;

    wire [7:0] rx_nxt = {rx_sh[6:0], mosi};
    assign miso = cs_n ? 1'bz : miso_r;

    initial begin
        for (i = 0; i < 128; i = i + 1) mem[i] = 8'h00;
        rx_sh = 0; tx_sh = 0; bits = 0; got_cmd = 0; is_wr = 0; miso_r = 0; addr = 0;
    end

    // flanco de subida: captura MOSI (CS alto reinicia la transaccion)
    always @(posedge sck or posedge cs_n) begin
        if (cs_n) begin
            bits <= 3'd0; got_cmd <= 1'b0;
        end else begin
            rx_sh <= rx_nxt;
            bits  <= bits + 3'd1;
            if (bits == 3'd7) begin                 // byte completo
                if (!got_cmd) begin
                    got_cmd <= 1'b1;
                    is_wr   <= (rx_nxt[7] == WRITE_FLAG);
                    addr    <= rx_nxt[6:0];
                end else begin
                    if (is_wr) begin
                        mem[addr] <= rx_nxt;
                        $display("[%0t] ESCLAVO: mem[%h] <= %h", $time, addr, rx_nxt);
                    end
                    addr <= addr + 7'd1;
                end
            end
        end
    end

    // flanco de bajada: presenta el siguiente bit de MISO
    always @(negedge sck) begin
        if (!cs_n) begin
            if (bits == 3'd0) begin                 // empieza un byte nuevo
                tx_nxt = (got_cmd && !is_wr) ? mem[addr] : 8'h00;
                miso_r <= tx_nxt[7];
                tx_sh  <= {tx_nxt[6:0], 1'b0};
            end else begin
                miso_r <= tx_sh[7];
                tx_sh  <= {tx_sh[6:0], 1'b0};
            end
        end
    end
endmodule