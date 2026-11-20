// Modelos de simulacion (NO sintetizables) del RFM95 y del BME680, modo SPI 0.
// spi_slave_core hace la parte serie; cada dispositivo agrega su comportamiento.

// ---------------------------------------------------------------- nucleo serie
// Primer byte = comando: bit7 distingue lectura/escritura segun WRITE_FLAG,
// bits [6:0] = direccion inicial (auto-incremento; NOINC0=1 deja la dir 0x00
// fija, como la FIFO del SX127x).
module spi_slave_core #(parameter WRITE_FLAG = 1'b1, parameter NOINC0 = 0)(
    input            sck, cs_n, mosi,
    output           miso,
    output reg       wr_evt,      // cambia en cada byte de datos escrito
    output reg [6:0] wr_addr,
    output reg [7:0] wr_data,
    output     [6:0] rd_addr,     // direccion del proximo byte a leer
    input      [7:0] rd_data      // contenido de rd_addr (combinacional)
);
    reg [7:0] rx_sh, tx_sh, tx_nxt;
    reg [2:0] bits;
    reg       got_cmd, is_wr, miso_r;
    reg [6:0] addr;

    wire [7:0] rx_nxt = {rx_sh[6:0], mosi};
    assign rd_addr = addr;
    assign miso    = cs_n ? 1'bz : miso_r;

    initial begin
        rx_sh = 0; tx_sh = 0; bits = 0; got_cmd = 0; is_wr = 0; miso_r = 0;
        addr = 0; wr_evt = 0; wr_addr = 0; wr_data = 0;
    end

    always @(posedge sck or posedge cs_n) begin
        if (cs_n) begin
            bits <= 3'd0; got_cmd <= 1'b0;
        end else begin
            rx_sh <= rx_nxt;
            bits  <= bits + 3'd1;
            if (bits == 3'd7) begin                       // byte completo
                if (!got_cmd) begin
                    got_cmd <= 1'b1;
                    is_wr   <= (rx_nxt[7] == WRITE_FLAG);
                    addr    <= rx_nxt[6:0];
                end else begin
                    if (is_wr) begin
                        wr_addr <= addr; wr_data <= rx_nxt; wr_evt <= ~wr_evt;
                    end
                    if (!(NOINC0 != 0 && addr == 7'd0)) addr <= addr + 7'd1;
                end
            end
        end
    end

    always @(negedge sck) begin
        if (!cs_n) begin
            if (bits == 3'd0) begin                       // empieza un byte nuevo
                tx_nxt = (got_cmd && !is_wr) ? rd_data : 8'h00;
                miso_r <= tx_nxt[7];
                tx_sh  <= {tx_nxt[6:0], 1'b0};
            end else begin
                miso_r <= tx_sh[7];
                tx_sh  <= {tx_sh[6:0], 1'b0};
            end
        end
    end
endmodule

// ----------------------------------------------------------------------- RFM95
// Modela lo minimo del SX1276 en modo LoRa que usa la FSM:
//  - FIFO (reg 0x00) con puntero 0x0D que se auto-incrementa
//  - RegOpMode (0x01): modo TX (011) -> tras TX_TIME levanta TxDone y vuelve a standby
//  - RegIrqFlags (0x12): escribir 1 limpia el bit; TxDone = bit 3
//  - Guarda cada paquete transmitido en txlog (16 bytes por paquete)
module rfm95_model #(parameter TX_TIME = 3000)(
    input sck, cs_n, mosi,
    output miso
);
    reg [7:0] mem  [0:127];
    reg [7:0] fifo [0:255];
    reg [7:0] txlog [0:63];
    reg [7:0] tx_len [0:3];
    integer   tx_count;
    integer   i;
    reg [7:0] plen;
    event     tx_go;

    wire       wr_evt;
    wire [6:0] wr_addr, rd_addr;
    wire [7:0] wr_data;
    wire [7:0] rd_data = mem[rd_addr];

    spi_slave_core #(.WRITE_FLAG(1'b1), .NOINC0(1)) C (
        .sck(sck), .cs_n(cs_n), .mosi(mosi), .miso(miso),
        .wr_evt(wr_evt), .wr_addr(wr_addr), .wr_data(wr_data),
        .rd_addr(rd_addr), .rd_data(rd_data));

    initial begin
        for (i = 0; i < 128; i = i + 1) mem[i] = 8'h00;
        for (i = 0; i < 256; i = i + 1) fifo[i] = 8'h00;
        mem[7'h0E] = 8'h80;      // FifoTxBaseAddr por defecto
        mem[7'h42] = 8'h12;      // RegVersion
        tx_count = 0;
    end

    always @(wr_evt) if (^wr_addr !== 1'bx) begin
        case (wr_addr)
            7'h00: begin fifo[mem[7'h0D]] = wr_data; mem[7'h0D] = mem[7'h0D] + 8'd1; end
            7'h12: mem[7'h12] = mem[7'h12] & ~wr_data;           // escribir 1 limpia
            7'h01: begin
                mem[7'h01] = wr_data;
                if (wr_data[2:0] == 3'b011) -> tx_go;            // modo TX
            end
            default: mem[wr_addr] = wr_data;
        endcase
    end

    always @(tx_go) begin
        plen = mem[7'h22];
        for (i = 0; i < plen && i < 16; i = i + 1)
            txlog[tx_count*16 + i] = fifo[mem[7'h0E] + i];
        tx_len[tx_count] = plen;
        #TX_TIME;
        mem[7'h12] = mem[7'h12] | 8'h08;                         // TxDone
        mem[7'h01] = 8'h81;                                      // vuelve a standby
        tx_count = tx_count + 1;
        $display("[%0t] RFM95: paquete #%0d transmitido (%0d bytes)", $time, tx_count, plen);
    end
endmodule

// ----------------------------------------------------------------------- BME680
// Modela: ctrl_meas (0x74) con modo forzado (01) inicia una medicion;
// meas_status_0 (0x1D): bit5 midiendo, bit7 new_data. Al terminar, los 8 bytes
// 'meas' aparecen en 0x1F..0x26 (presion 3, temperatura 3, humedad 2).
// NO modela la paginacion de memoria SPI (reg 0x73): confirmala con el datasheet.
module bme680_model #(parameter MEAS_TIME = 2000)(
    input sck, cs_n, mosi,
    output miso
);
    reg [7:0] mem  [0:127];
    reg [7:0] meas [0:7];
    integer   i;
    event     meas_go;

    wire       wr_evt;
    wire [6:0] wr_addr, rd_addr;
    wire [7:0] wr_data;
    wire [7:0] rd_data = mem[rd_addr];

    spi_slave_core #(.WRITE_FLAG(1'b0), .NOINC0(0)) C (
        .sck(sck), .cs_n(cs_n), .mosi(mosi), .miso(miso),
        .wr_evt(wr_evt), .wr_addr(wr_addr), .wr_data(wr_data),
        .rd_addr(rd_addr), .rd_data(rd_data));

    initial begin
        for (i = 0; i < 128; i = i + 1) mem[i] = 8'h00;
        for (i = 0; i < 8;   i = i + 1) meas[i] = 8'h00;
    end

    always @(wr_evt) if (^wr_addr !== 1'bx) begin
        mem[wr_addr] = wr_data;
        if (wr_addr == 7'h74 && wr_data[1:0] == 2'b01) -> meas_go;   // modo forzado
    end

    always @(meas_go) begin
        mem[7'h1D] = 8'h20;                                      // midiendo
        #MEAS_TIME;
        for (i = 0; i < 8; i = i + 1) mem[7'h1F + i] = meas[i];
        mem[7'h1D] = 8'h80;                                      // new_data
        mem[7'h74] = mem[7'h74] & 8'hFC;                         // vuelve a sleep
    end
endmodule