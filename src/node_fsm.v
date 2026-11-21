// FSM principal del nodo: BME680 (T, P, H crudos) -> paquete -> RFM95 (LoRa).
// Cada estado lanza UNA transaccion SPI (spi_txn) y espera 'done'.
//
// Arranque : espera power-up, escribe la tabla de configuracion del radio, duerme.
// Cada PERIODO (inicio a inicio): mide con el BME680 en modo forzado y despues,
//   con el radio en standby, arma el paquete [ID, SEQ, 8 bytes crudos] (10 bytes)
//   DIRECTAMENTE en la FIFO del RFM95: escribe [ID, SEQ] y luego, para cada
//   registro 0x1F..0x26 del BME680, lo lee y lo escribe en la FIFO (un solo byte
//   en transito; ya no hay buffer sens[] de 8 bytes). Despues transmite, espera
//   TxDone, limpia banderas y manda el radio a sleep.
//
// DEPENDE de que FifoAddrPtr del SX1276 se auto-incremente en cada escritura a
// la FIFO, tambien entre transacciones separadas (asi lo usan las librerias tipo
// arduino-LoRa). Confirmalo en el datasheet.
//
// Todos los contadores tienen el ancho minimo segun los parametros.
module node_fsm #(
    parameter CLK_HZ    = 32768,   // frecuencia de clk (para contar segundos)
    parameter PERIOD_S  = 600,     // periodo de envio (segundos)
    parameter CLK_DIV   = 1,       // SCK = clk / (2*CLK_DIV)
    parameter CS_GAP    = 2,
    parameter NODE_ID   = 8'h01,
    parameter PWRUP_CYC = 400,     // espera tras encender, antes del primer SPI
    parameter POLL_GAP  = 32,      // ciclos entre sondeos de estado
    parameter POLL_MAX  = 200      // sondeos maximos antes de abortar/continuar
)(
    input  clk, rst_n,
    output cs_rf_n, cs_bm_n, sck, mosi,
    input  miso,
    output active                  // 1 mientras no esta dormido
);
    // ---- anchos minimos
    localparam PRW  = (CLK_HZ   > 1) ? $clog2(CLK_HZ)   : 1;   // prescaler (0..CLK_HZ-1)
    localparam SCW  = (PERIOD_S > 1) ? $clog2(PERIOD_S) : 1;   // segundos  (0..PERIOD_S-1)
    localparam DMAX = (PWRUP_CYC > POLL_GAP) ? PWRUP_CYC : POLL_GAP;
    localparam DW   = $clog2(DMAX + 1);                        // retardo descendente
    localparam PW   = $clog2(POLL_MAX + 1);                    // sondeos
    localparam NBOOT = 9;

    /* verilator lint_off WIDTHTRUNC */
    localparam [PRW-1:0] PRE_LAST = CLK_HZ - 1;
    localparam [SCW-1:0] SEC_LAST = PERIOD_S - 1;
    localparam [DW-1:0]  PWRUP_V  = PWRUP_CYC;
    localparam [DW-1:0]  POLL_V   = POLL_GAP;
    localparam [PW-1:0]  PMAX     = POLL_MAX;
    localparam [3:0]     BOOT_LAST = NBOOT - 1;
    localparam [3:0]     PKT_LEN   = 4'd10;
    /* verilator lint_on WIDTHTRUNC */

    localparam S_PWRUP    = 5'd0,  S_BOOT     = 5'd1,  S_BOOT_NEXT = 5'd2,
               S_SLEEP    = 5'd3,  S_BME_PAGE = 5'd4,  S_BME_HUM   = 5'd5,
               S_BME_MEAS = 5'd6,  S_BME_POLL = 5'd7,  S_BME_CHK   = 5'd8,
               S_RF_STBY  = 5'd9,  S_RF_PTR   = 5'd10, S_RF_HDR    = 5'd11,
               S_BME_RD   = 5'd12, S_RF_PUT   = 5'd13, S_BYTE_NEXT = 5'd14,
               S_RF_LEN   = 5'd15, S_RF_TX    = 5'd16, S_RF_POLL   = 5'd17,
               S_RF_CHK   = 5'd18, S_RF_CLR   = 5'd19, S_RF_SLP    = 5'd20,
               S_GO       = 5'd21, S_WAIT     = 5'd22, S_DLY       = 5'd23;

    // origen del byte de datos de la transaccion
    localparam SRC_CONST = 2'd0,   // t_const
               SRC_HDR   = 2'd1,   // cabecera: idx0 = ID, idx1 = SEQ
               SRC_LAST  = 2'd2;   // ultimo byte recibido (last_rx)

    reg [4:0]     st, after, dly_ret;
    reg [3:0]     bi;               // indice de boot / de byte del sensor
    reg [PRW-1:0] pre;
    reg [SCW-1:0] secs;
    reg [DW-1:0]  dly;
    reg [PW-1:0]  poll_cnt;
    reg           wake_pending;
    reg [7:0]     seq, last_rx;

    // transaccion en curso
    reg        dev, t_start;
    reg [1:0]  t_src;
    reg [7:0]  t_cmd, t_const;
    reg [3:0]  t_len;
    wire       t_done, t_rxv, cs_n_i, t_busy;
    wire [7:0] t_rx;
    wire [3:0] t_ridx, t_tidx;

    wire [7:0] hdr_byte = t_tidx[0] ? seq : NODE_ID;
    wire [7:0] t_txb    = (t_src == SRC_HDR)  ? hdr_byte :
                          (t_src == SRC_LAST) ? last_rx  : t_const;

    spi_txn #(.CLK_DIV(CLK_DIV), .CS_GAP(CS_GAP)) TXN (
        .clk(clk), .rst_n(rst_n), .start(t_start), .cmd(t_cmd), .len(t_len),
        .tx_byte(t_txb), .tx_idx(t_tidx), .busy(t_busy), .done(t_done),
        .rx_valid(t_rxv), .rx_byte(t_rx), .rx_idx(t_ridx),
        .cs_n(cs_n_i), .sck(sck), .mosi(mosi), .miso(miso));

    assign cs_rf_n = dev ? 1'b1   : cs_n_i;
    assign cs_bm_n = dev ? cs_n_i : 1'b1;
    assign active  = (st != S_SLEEP);

    wire _unused = &{1'b0, t_ridx, t_tidx[3:1]};

    // tabla de configuracion del radio (se escribe una vez al arrancar)
    reg [7:0] boot_addr, boot_val;
    always @* begin
        case (bi)
            4'd0:    begin boot_addr = 8'h01; boot_val = 8'h80; end // OpMode: sleep + LoRa
            4'd1:    begin boot_addr = 8'h06; boot_val = 8'hE4; end // Frf MSB (915 MHz)
            4'd2:    begin boot_addr = 8'h07; boot_val = 8'hC0; end // Frf MID
            4'd3:    begin boot_addr = 8'h08; boot_val = 8'h00; end // Frf LSB
            4'd4:    begin boot_addr = 8'h09; boot_val = 8'h8B; end // PaConfig: PA_BOOST, ~13 dBm
            4'd5:    begin boot_addr = 8'h1D; boot_val = 8'h72; end // ModemConfig1: BW125, CR4/5
            4'd6:    begin boot_addr = 8'h1E; boot_val = 8'h74; end // ModemConfig2: SF7, CRC on
            4'd7:    begin boot_addr = 8'h26; boot_val = 8'h04; end // ModemConfig3: AGC auto
            default: begin boot_addr = 8'h0E; boot_val = 8'h00; end // FifoTxBaseAddr = 0
        endcase
    end

    // lanza una transaccion y vuelve a 'nxt' cuando termina
    task launch;
        input        d;     // 0 = RFM95, 1 = BME680
        input [7:0]  c;     // byte de comando (con bit de lectura/escritura)
        input [3:0]  l;     // bytes de datos
        input [7:0]  k;     // dato constante (escrituras de 1 registro)
        input [1:0]  p;     // origen del dato (SRC_*)
        input [4:0]  nxt;
        begin
            dev <= d; t_cmd <= c; t_len <= l; t_const <= k;
            t_src <= p; after <= nxt; st <= S_GO;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_PWRUP; after <= S_SLEEP; dly_ret <= S_SLEEP; bi <= 4'd0;
            pre <= {PRW{1'b0}}; secs <= {SCW{1'b0}};
            dly <= {DW{1'b0}}; poll_cnt <= {PW{1'b0}};
            wake_pending <= 1'b1;               // primer ciclo justo despues del arranque
            seq <= 8'd0; last_rx <= 8'd0;
            dev <= 1'b0; t_start <= 1'b0; t_src <= SRC_CONST;
            t_cmd <= 8'd0; t_const <= 8'd0; t_len <= 4'd0;
        end else begin
            t_start <= 1'b0;
            if (t_rxv) last_rx <= t_rx;

            case (st)
                // ---- arranque
                S_PWRUP: begin
                    dly <= PWRUP_V; dly_ret <= S_BOOT; st <= S_DLY;
                end
                S_BOOT:      launch(1'b0, 8'h80 | boot_addr, 4'd1, boot_val, SRC_CONST, S_BOOT_NEXT);
                S_BOOT_NEXT: begin
                    if (bi == BOOT_LAST) begin bi <= 4'd0; st <= S_SLEEP; end
                    else begin bi <= bi + 1'b1; st <= S_BOOT; end
                end

                // ---- reposo hasta el siguiente periodo
                S_SLEEP: if (wake_pending) begin
                    wake_pending <= 1'b0; st <= S_BME_PAGE;
                end

                // ---- medicion BME680 (modo forzado, sin gas)
                S_BME_PAGE: launch(1'b1, 8'h73, 4'd1, 8'h10, SRC_CONST, S_BME_HUM);   // spi_mem_page = 1
                S_BME_HUM:  launch(1'b1, 8'h72, 4'd1, 8'h01, SRC_CONST, S_BME_MEAS);  // ctrl_hum: osrs_h x1
                S_BME_MEAS: begin                                                      // ctrl_meas: T x1, P x1, forzado
                    poll_cnt <= {PW{1'b0}};
                    launch(1'b1, 8'h74, 4'd1, 8'h25, SRC_CONST, S_BME_POLL);
                end
                S_BME_POLL: launch(1'b1, 8'h9D, 4'd1, 8'h00, SRC_CONST, S_BME_CHK);   // leer 0x1D
                S_BME_CHK: begin
                    if (last_rx[7]) st <= S_RF_STBY;                   // new_data
                    else if (poll_cnt == PMAX) st <= S_SLEEP;          // timeout: aborta el ciclo
                    else begin
                        poll_cnt <= poll_cnt + 1'b1;
                        dly <= POLL_V; dly_ret <= S_BME_POLL; st <= S_DLY;
                    end
                end

                // ---- paquete directo a la FIFO del RFM95
                S_RF_STBY: launch(1'b0, 8'h81, 4'd1, 8'h81, SRC_CONST, S_RF_PTR);     // OpMode: standby LoRa
                S_RF_PTR:  launch(1'b0, 8'h8D, 4'd1, 8'h00, SRC_CONST, S_RF_HDR);     // FifoAddrPtr = 0
                S_RF_HDR:  launch(1'b0, 8'h80, 4'd2, 8'h00, SRC_HDR,   S_BME_RD);     // FIFO <- [ID, SEQ]
                S_BME_RD:  launch(1'b1, 8'h9F + {4'd0, bi}, 4'd1, 8'h00, SRC_CONST, S_RF_PUT); // leer 0x1F+bi
                S_RF_PUT:  launch(1'b0, 8'h80, 4'd1, 8'h00, SRC_LAST,  S_BYTE_NEXT);  // FIFO <- ese byte
                S_BYTE_NEXT: begin
                    if (bi == 4'd7) begin bi <= 4'd0; st <= S_RF_LEN; end
                    else begin bi <= bi + 1'b1; st <= S_BME_RD; end
                end

                // ---- transmision RFM95
                S_RF_LEN:  launch(1'b0, 8'hA2, 4'd1, {4'd0, PKT_LEN}, SRC_CONST, S_RF_TX); // PayloadLength
                S_RF_TX: begin                                                         // OpMode: TX
                    poll_cnt <= {PW{1'b0}};
                    launch(1'b0, 8'h81, 4'd1, 8'h83, SRC_CONST, S_RF_POLL);
                end
                S_RF_POLL: launch(1'b0, 8'h12, 4'd1, 8'h00, SRC_CONST, S_RF_CHK);     // leer IrqFlags
                S_RF_CHK: begin
                    if (last_rx[3] || poll_cnt == PMAX) st <= S_RF_CLR;                // TxDone (o timeout)
                    else begin
                        poll_cnt <= poll_cnt + 1'b1;
                        dly <= POLL_V; dly_ret <= S_RF_POLL; st <= S_DLY;
                    end
                end
                S_RF_CLR:  launch(1'b0, 8'h92, 4'd1, 8'hFF, SRC_CONST, S_RF_SLP);     // limpiar IrqFlags
                S_RF_SLP: begin                                                        // radio a sleep
                    seq <= seq + 8'd1;
                    launch(1'b0, 8'h81, 4'd1, 8'h80, SRC_CONST, S_SLEEP);
                end

                // ---- mecanica comun
                S_GO:   if (!t_busy) begin t_start <= 1'b1; st <= S_WAIT; end
                S_WAIT: if (t_done) st <= after;
                S_DLY:  if (dly == {DW{1'b0}}) st <= dly_ret; else dly <= dly - 1'b1;

                default: st <= S_SLEEP;
            endcase

            // temporizador de periodo (inicio a inicio), despues del case para que gane el set
            if (pre == PRE_LAST) begin
                pre <= {PRW{1'b0}};
                if (secs == SEC_LAST) begin secs <= {SCW{1'b0}}; wake_pending <= 1'b1; end
                else secs <= secs + 1'b1;
            end else pre <= pre + 1'b1;
        end
    end
endmodule