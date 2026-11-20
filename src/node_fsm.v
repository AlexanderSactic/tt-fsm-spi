// FSM principal del nodo: BME680 (T, P, H crudos) -> paquete -> RFM95 (LoRa).
// Cada estado lanza UNA transaccion SPI (spi_txn) y espera 'done'.
//
// Arranque : espera power-up, escribe la tabla de configuracion del radio, duerme.
// Cada PERIODO (inicio a inicio): mide con el BME680 en modo forzado, lee 8 bytes,
//   arma el paquete [ID, SEQ, 8 bytes crudos] (10 bytes), lo carga en la FIFO del
//   RFM95, transmite, espera TxDone, limpia banderas y manda el radio a sleep.
//
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
    localparam S_PWRUP   = 5'd0,  S_BOOT    = 5'd1,  S_BOOT_NEXT = 5'd2,
               S_SLEEP   = 5'd3,  S_BME_HUM = 5'd4,  S_BME_MEAS  = 5'd5,
               S_BME_POLL= 5'd6,  S_BME_CHK = 5'd7,  S_BME_READ  = 5'd8,
               S_RF_STBY = 5'd9,  S_RF_PTR  = 5'd10, S_RF_FIFO   = 5'd11,
               S_RF_LEN  = 5'd12, S_RF_TX   = 5'd13, S_RF_POLL   = 5'd14,
               S_RF_CHK  = 5'd15, S_RF_CLR  = 5'd16, S_RF_SLP    = 5'd17,
               S_GO      = 5'd18, S_WAIT    = 5'd19, S_DLY       = 5'd20;
    localparam PKT_LEN = 4'd10;
    localparam NBOOT   = 9;
    localparam S_BME_PAGE = 5'd21;

    reg [4:0]  st, after, dly_ret;
    reg [3:0]  bi;
    reg [15:0] pre, secs, dly, dly_tgt, poll_cnt;
    reg        wake_pending;
    reg [7:0]  seq, last_rx;
    reg [7:0]  sens [0:7];

    // transaccion en curso
    reg        dev, t_start, t_pkt, t_cap;
    reg [7:0]  t_cmd, t_const;
    reg [3:0]  t_len;
    wire       t_busy, t_done, t_rxv, cs_n_i;
    wire [7:0] t_rx;
    wire [3:0] t_ridx, t_tidx;

    // paquete: [0]=ID, [1]=SEQ, [2..9]=sensor
    wire [7:0] pkt_byte = (t_tidx == 4'd0) ? NODE_ID :
                          (t_tidx == 4'd1) ? seq : sens[t_tidx[2:0] - 3'd2];
    wire [7:0] t_txb    = t_pkt ? pkt_byte : t_const;

    spi_txn #(.CLK_DIV(CLK_DIV), .CS_GAP(CS_GAP)) TXN (
        .clk(clk), .rst_n(rst_n), .start(t_start), .cmd(t_cmd), .len(t_len),
        .tx_byte(t_txb), .tx_idx(t_tidx), .busy(t_busy), .done(t_done),
        .rx_valid(t_rxv), .rx_byte(t_rx), .rx_idx(t_ridx),
        .cs_n(cs_n_i), .sck(sck), .mosi(mosi), .miso(miso));

    assign cs_rf_n = dev ? 1'b1   : cs_n_i;
    assign cs_bm_n = dev ? cs_n_i : 1'b1;
    assign active  = (st != S_SLEEP);

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
        input        p;     // 1: los datos salen del paquete
        input        cp;    // 1: guardar lo recibido en sens[]
        input [4:0]  nxt;
        begin
            dev <= d; t_cmd <= c; t_len <= l; t_const <= k;
            t_pkt <= p; t_cap <= cp; after <= nxt; st <= S_GO;
        end
    endtask

    integer j;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_PWRUP; after <= S_SLEEP; dly_ret <= S_SLEEP; bi <= 4'd0;
            pre <= 16'd0; secs <= 16'd0; dly <= 16'd0; dly_tgt <= 16'd0; poll_cnt <= 16'd0;
            wake_pending <= 1'b1;               // primer ciclo justo despues del arranque
            seq <= 8'd0; last_rx <= 8'd0;
            dev <= 1'b0; t_start <= 1'b0; t_pkt <= 1'b0; t_cap <= 1'b0;
            t_cmd <= 8'd0; t_const <= 8'd0; t_len <= 4'd0;
            for (j = 0; j < 8; j = j + 1) sens[j] <= 8'd0;
        end else begin
            t_start <= 1'b0;
            if (t_rxv) begin
                last_rx <= t_rx;
                if (t_cap) sens[t_ridx[2:0]] <= t_rx;
            end

            case (st)
                // ---- arranque
                S_PWRUP: begin
                    dly <= 16'd0; dly_tgt <= PWRUP_CYC; dly_ret <= S_BOOT; st <= S_DLY;
                end
                S_BOOT:      launch(1'b0, 8'h80 | boot_addr, 4'd1, boot_val, 1'b0, 1'b0, S_BOOT_NEXT);
                S_BOOT_NEXT: begin
                    if (bi == NBOOT-1) begin bi <= 4'd0; st <= S_SLEEP; end
                    else begin bi <= bi + 4'd1; st <= S_BOOT; end
                end

                // ---- reposo hasta el siguiente periodo
                S_SLEEP: if (wake_pending) begin
                    wake_pending <= 1'b0; st <= S_BME_PAGE;
                end
                
                S_BME_PAGE: launch(1'b1, 8'h73, 4'd1, 8'h10, 1'b0, 1'b0, S_BME_HUM);  // spi_mem_page = 1

                // ---- medicion BME680 (modo forzado, sin gas)
                S_BME_HUM:  launch(1'b1, 8'h72, 4'd1, 8'h01, 1'b0, 1'b0, S_BME_MEAS); // ctrl_hum: osrs_h x1
                S_BME_MEAS: begin                                                      // ctrl_meas: T x1, P x1, forzado
                    poll_cnt <= 16'd0;
                    launch(1'b1, 8'h74, 4'd1, 8'h25, 1'b0, 1'b0, S_BME_POLL);
                end
                S_BME_POLL: launch(1'b1, 8'h9D, 4'd1, 8'h00, 1'b0, 1'b0, S_BME_CHK);  // leer 0x1D
                S_BME_CHK: begin
                    if (last_rx[7]) st <= S_BME_READ;                  // new_data
                    else if (poll_cnt == POLL_MAX) st <= S_SLEEP;      // timeout: aborta el ciclo
                    else begin
                        poll_cnt <= poll_cnt + 16'd1;
                        dly <= 16'd0; dly_tgt <= POLL_GAP; dly_ret <= S_BME_POLL; st <= S_DLY;
                    end
                end
                S_BME_READ: launch(1'b1, 8'h9F, 4'd8, 8'h00, 1'b0, 1'b1, S_RF_STBY);  // rafaga 0x1F..0x26

                // ---- transmision RFM95
                S_RF_STBY: launch(1'b0, 8'h81, 4'd1, 8'h81, 1'b0, 1'b0, S_RF_PTR);    // OpMode: standby LoRa
                S_RF_PTR:  launch(1'b0, 8'h8D, 4'd1, 8'h00, 1'b0, 1'b0, S_RF_FIFO);   // FifoAddrPtr = 0
                S_RF_FIFO: launch(1'b0, 8'h80, PKT_LEN, 8'h00, 1'b1, 1'b0, S_RF_LEN); // rafaga a la FIFO (reg 0x00)
                S_RF_LEN:  launch(1'b0, 8'hA2, 4'd1, {4'd0, PKT_LEN}, 1'b0, 1'b0, S_RF_TX); // PayloadLength
                S_RF_TX: begin                                                         // OpMode: TX
                    poll_cnt <= 16'd0;
                    launch(1'b0, 8'h81, 4'd1, 8'h83, 1'b0, 1'b0, S_RF_POLL);
                end
                S_RF_POLL: launch(1'b0, 8'h12, 4'd1, 8'h00, 1'b0, 1'b0, S_RF_CHK);    // leer IrqFlags
                S_RF_CHK: begin
                    if (last_rx[3] || poll_cnt == POLL_MAX) st <= S_RF_CLR;            // TxDone (o timeout)
                    else begin
                        poll_cnt <= poll_cnt + 16'd1;
                        dly <= 16'd0; dly_tgt <= POLL_GAP; dly_ret <= S_RF_POLL; st <= S_DLY;
                    end
                end
                S_RF_CLR:  launch(1'b0, 8'h92, 4'd1, 8'hFF, 1'b0, 1'b0, S_RF_SLP);    // limpiar IrqFlags
                S_RF_SLP: begin                                                        // radio a sleep
                    seq <= seq + 8'd1;
                    launch(1'b0, 8'h81, 4'd1, 8'h80, 1'b0, 1'b0, S_SLEEP);
                end

                // ---- mecanica comun
                S_GO:   if (!t_busy) begin t_start <= 1'b1; st <= S_WAIT; end
                S_WAIT: if (t_done) st <= after;
                S_DLY:  if (dly == dly_tgt) st <= dly_ret; else dly <= dly + 16'd1;

                default: st <= S_SLEEP;
            endcase

            // temporizador de periodo (inicio a inicio), despues del case para que gane el set
            if (pre == CLK_HZ-1) begin
                pre <= 16'd0;
                if (secs == PERIOD_S-1) begin secs <= 16'd0; wake_pending <= 1'b1; end
                else secs <= secs + 16'd1;
            end else pre <= pre + 16'd1;
        end
    end
endmodule