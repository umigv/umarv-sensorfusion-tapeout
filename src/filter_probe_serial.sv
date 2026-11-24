// AREA PROBE ONLY. Not a filter: the table below is random, so the numbers it produces mean
// nothing. It has the hardware a small microcoded single-precision filter would have, so the
// place-and-route flow can tell us what such a filter costs:
//   - 28 32-bit words kept in latches, one read port, the first six wired straight out
//   - one bit-serial fp32 add / subtract / multiply / divide unit (fp_serial, which is real)
//   - a 160-entry step table with two loop counters, as a filter's sequencer would have
//   - sensor words selectable as an operand, and byte-wide tuning writes from the SPI block
//   - the six output words double as the host-written state: SPI writes to 24-3B land in them,
//     so the SPI block needs no state registers of its own and the top level needs no selector
// Not here: the double-precision position difference, and sharing the lidar block's CORDIC.
module filter_probe_serial (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        enable,             // 0: the filter stays idle and the host owns the state words
    input  logic        meas_pulse,
    input  logic [31:0] beat_ts,
    input  logic [31:0] vn_yaw_f32, vn_rate_f32,
    input  logic [63:0] vn_lat_f64, vn_lon_f64,
    input  logic signed [27:0] dw_delta,
    input  logic [2:0]  spi_sel,            // the SPI block reads sensor words through the same selector
    output logic [31:0] meas_word,
    input  logic        cfg_we,
    input  logic [6:0]  cfg_addr,
    input  logic [7:0]  cfg_wdata,
    output logic        state_valid,
    output logic [31:0] state_ts, state_x, state_y, state_theta, state_omega, state_v
);
    localparam WORDS = 28;
    localparam STEPS = 160;
    localparam [5:0] CFG_BASE = WORDS - 8;      // tuning words sit at the top of the store

    // ---- step table ----
    logic [7:0]  pc;
    logic [26:0] rom;
    always_comb begin
        case (pc)
            8'd0: rom = 27'h00f80c6;
            8'd1: rom = 27'h0019487;
            8'd2: rom = 27'h0003611;
            8'd3: rom = 27'h0040006;
            8'd4: rom = 27'h00944d5;
            8'd5: rom = 27'h03c408b;
            8'd6: rom = 27'h00103cd;
            8'd7: rom = 27'h03024c8;
            8'd8: rom = 27'h5488688;
            8'd9: rom = 27'h008f6d1;
            8'd10: rom = 27'h008750c;
            8'd11: rom = 27'h00b9406;
            8'd12: rom = 27'h0082003;
            8'd13: rom = 27'h00563d2;
            8'd14: rom = 27'h0042495;
            8'd15: rom = 27'h005118a;
            8'd16: rom = 27'h020340a;
            8'd17: rom = 27'h508d207;
            8'd18: rom = 27'h004810f;
            8'd19: rom = 27'h0058694;
            8'd20: rom = 27'h0012311;
            8'd21: rom = 27'h008a494;
            8'd22: rom = 27'h00ba055;
            8'd23: rom = 27'h01ce414;
            8'd24: rom = 27'h0283153;
            8'd25: rom = 27'h00900c4;
            8'd26: rom = 27'h5a94191;
            8'd27: rom = 27'h001658b;
            8'd28: rom = 27'h00c8110;
            8'd29: rom = 27'h0097088;
            8'd30: rom = 27'h008604f;
            8'd31: rom = 27'h0007408;
            8'd32: rom = 27'h028e08e;
            8'd33: rom = 27'h033b189;
            8'd34: rom = 27'h0196396;
            8'd35: rom = 27'h4d50593;
            8'd36: rom = 27'h005218f;
            8'd37: rom = 27'h001a2ca;
            8'd38: rom = 27'h0098444;
            8'd39: rom = 27'h0084107;
            8'd40: rom = 27'h00c4688;
            8'd41: rom = 27'h0180547;
            8'd42: rom = 27'h0106053;
            8'd43: rom = 27'h0286552;
            8'd44: rom = 27'h56bc350;
            8'd45: rom = 27'h00db108;
            8'd46: rom = 27'h0088352;
            8'd47: rom = 27'h00c8409;
            8'd48: rom = 27'h008818f;
            8'd49: rom = 27'h00cc051;
            8'd50: rom = 27'h018f152;
            8'd51: rom = 27'h00823c4;
            8'd52: rom = 27'h03953cb;
            8'd53: rom = 27'h5b85251;
            8'd54: rom = 27'h00cb18b;
            8'd55: rom = 27'h00bd494;
            8'd56: rom = 27'h0095014;
            8'd57: rom = 27'h00002d5;
            8'd58: rom = 27'h0004490;
            8'd59: rom = 27'h0289550;
            8'd60: rom = 27'h02d2196;
            8'd61: rom = 27'h009158c;
            8'd62: rom = 27'h568a38e;
            8'd63: rom = 27'h0018409;
            8'd64: rom = 27'h00482c2;
            8'd65: rom = 27'h000c047;
            8'd66: rom = 27'h007e54a;
            8'd67: rom = 27'h00902d3;
            8'd68: rom = 27'h009b60f;
            8'd69: rom = 27'h03921d1;
            8'd70: rom = 27'h0094646;
            8'd71: rom = 27'h4dc53d5;
            8'd72: rom = 27'h00514c8;
            8'd73: rom = 27'h005168d;
            8'd74: rom = 27'h0050616;
            8'd75: rom = 27'h009b690;
            8'd76: rom = 27'h0056454;
            8'd77: rom = 27'h023f083;
            8'd78: rom = 27'h00934c8;
            8'd79: rom = 27'h0193191;
            8'd80: rom = 27'h57c1014;
            8'd81: rom = 27'h001a253;
            8'd82: rom = 27'h00843cd;
            8'd83: rom = 27'h001b5d1;
            8'd84: rom = 27'h00cb34b;
            8'd85: rom = 27'h001400c;
            8'd86: rom = 27'h015010e;
            8'd87: rom = 27'h030b4d1;
            8'd88: rom = 27'h03b8351;
            8'd89: rom = 27'h4c4a48e;
            8'd90: rom = 27'h00114c1;
            8'd91: rom = 27'h0051491;
            8'd92: rom = 27'h008528f;
            8'd93: rom = 27'h009a20b;
            8'd94: rom = 27'h0085013;
            8'd95: rom = 27'h014e20b;
            8'd96: rom = 27'h03903d5;
            8'd97: rom = 27'h038b4c9;
            8'd98: rom = 27'h51833d5;
            8'd99: rom = 27'h003944a;
            8'd100: rom = 27'h0087588;
            8'd101: rom = 27'h008c256;
            8'd102: rom = 27'h008c44c;
            8'd103: rom = 27'h00cf544;
            8'd104: rom = 27'h03900d4;
            8'd105: rom = 27'h0006150;
            8'd106: rom = 27'h034858c;
            8'd107: rom = 27'h4595289;
            8'd108: rom = 27'h000164e;
            8'd109: rom = 27'h008a28f;
            8'd110: rom = 27'h00fa34b;
            8'd111: rom = 27'h008c00f;
            8'd112: rom = 27'h00d3250;
            8'd113: rom = 27'h01c4194;
            8'd114: rom = 27'h03cd410;
            8'd115: rom = 27'h028d30e;
            8'd116: rom = 27'h52c7242;
            8'd117: rom = 27'h004d147;
            8'd118: rom = 27'h0052451;
            8'd119: rom = 27'h0086354;
            8'd120: rom = 27'h00d1308;
            8'd121: rom = 27'h003b08d;
            8'd122: rom = 27'h038c1cf;
            8'd123: rom = 27'h0290414;
            8'd124: rom = 27'h020040b;
            8'd125: rom = 27'h4b4d156;
            8'd126: rom = 27'h00490d5;
            8'd127: rom = 27'h009928a;
            8'd128: rom = 27'h009b613;
            8'd129: rom = 27'h00886c3;
            8'd130: rom = 27'h0042654;
            8'd131: rom = 27'h034708e;
            8'd132: rom = 27'h027c112;
            8'd133: rom = 27'h015140a;
            8'd134: rom = 27'h4f55016;
            8'd135: rom = 27'h008650d;
            8'd136: rom = 27'h0080011;
            8'd137: rom = 27'h0087546;
            8'd138: rom = 27'h008d3d3;
            8'd139: rom = 27'h0094496;
            8'd140: rom = 27'h03493c6;
            8'd141: rom = 27'h0216489;
            8'd142: rom = 27'h018c342;
            8'd143: rom = 27'h533d0d3;
            8'd144: rom = 27'h005b148;
            8'd145: rom = 27'h009914d;
            8'd146: rom = 27'h0010307;
            8'd147: rom = 27'h0048556;
            8'd148: rom = 27'h000b407;
            8'd149: rom = 27'h0081355;
            8'd150: rom = 27'h02051d5;
            8'd151: rom = 27'h00c91d4;
            8'd152: rom = 27'h4a8534f;
            8'd153: rom = 27'h0080250;
            8'd154: rom = 27'h00fe18b;
            8'd155: rom = 27'h0014241;
            8'd156: rom = 27'h0080342;
            8'd157: rom = 27'h0094283;
            8'd158: rom = 27'h001a484;
            8'd159: rom = 27'h00d0585;
            default: rom = 27'd0;
        endcase
    end
    wire        r_loop = rom[26];
    wire [3:0]  r_back = rom[25:22];
    wire [1:0]  r_f    = rom[21:20];
    wire [1:0]  r_op   = rom[19:18];
    wire [5:0]  r_a    = rom[17:12];
    wire [5:0]  r_b    = rom[11:6];
    wire [5:0]  r_d    = rom[5:0];

    logic [2:0] li, lj;                 // loop counters
    wire [5:0] addr_a = r_a + (r_f[0] ? {3'd0, li} : 6'd0);
    wire [5:0] addr_b = r_b + (r_f[1] ? {3'd0, lj} : 6'd0);
    wire [5:0] addr_d = r_d + (r_f[0] ? {3'd0, li} : 6'd0);

    // ---- storage ----
    logic        m_we;
    logic [5:0]  m_waddr, m_raddr;
    logic [31:0] m_wdata;
    wire  [31:0] m_rdata;
    wire  [191:0] m_first;
    filter_latch_mem #(.WORDS(WORDS)) u_mem (.clk(clk), .we(m_we), .waddr(m_waddr), .wdata(m_wdata),
                                             .raddr(m_raddr), .rdata(m_rdata), .first6(m_first));

    // ---- arithmetic ----
    logic        f_load_b, f_start;
    wire         f_busy, f_done;
    wire  [31:0] f_result;
    logic [31:0] ext;
    logic        f_load_a;
    wire  [2:0]  ext_sel = f_load_a ? r_a[2:0] : spi_sel;
    assign meas_word = ext;
    always_comb begin
        case (ext_sel)
            3'd0: ext = vn_yaw_f32;
            3'd1: ext = vn_rate_f32;
            3'd2: ext = vn_lat_f64[63:32];
            3'd3: ext = vn_lat_f64[31:0];
            3'd4: ext = vn_lon_f64[63:32];
            3'd5: ext = vn_lon_f64[31:0];
            3'd6: ext = {{4{dw_delta[27]}}, dw_delta};
            default: ext = beat_ts;
        endcase
    end
    wire [31:0] f_din = (f_load_a && r_a[5:3] == 3'd7) ? ext : m_rdata;
    fp_serial u_fp (.clk(clk), .rst_n(rst_n), .load_a(f_load_a), .load_b(f_load_b), .start(f_start), .op(r_op),
                    .din(f_din), .busy(f_busy), .done(f_done), .result(f_result));

    // ---- sequencer: read a, read b, run, write back, settle ----
    logic [2:0]  st;
    logic        running;
    logic [4:0]  init;                  // after reset the six state words are cleared, one every four clocks
    logic [23:0] asm;                   // tuning bytes from the SPI block are gathered into a word
    wire         cfg_state = (cfg_addr >= 7'h24) && (cfg_addr < 7'h3C);
    wire         cfg_tune  = (cfg_addr >= 7'h6C);
    wire  [4:0]  cfg_word  = cfg_addr[6:2] - 5'd9;

    assign f_load_a = running && (st == 3'd0);
    assign f_load_b = running && (st == 3'd1);
    assign f_start  = running && (st == 3'd2);
    assign m_raddr  = (st == 3'd1) ? addr_b : addr_a;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc <= 8'd0; st <= 3'd0; running <= 1'b0; li <= 3'd0; lj <= 3'd0;
            m_we <= 1'b0; m_waddr <= 6'd0; m_wdata <= 32'd0; asm <= 24'd0; state_valid <= 1'b0; init <= 5'd0;
        end else begin
            m_we <= 1'b0; state_valid <= 1'b0;
            if (init != 5'd24) begin
                init <= init + 5'd1;
                if (init[1:0] == 2'd0) begin m_we <= 1'b1; m_waddr <= {3'd0, init[4:2]}; m_wdata <= 32'd0; end
            end else
            if (cfg_we && (cfg_tune || cfg_state)) begin
                asm <= {cfg_wdata, asm[23:8]};
                if (!running && cfg_addr[1:0] == 2'd3) begin
                    m_we <= 1'b1; m_wdata <= {cfg_wdata, asm};
                    m_waddr <= cfg_state ? {1'b0, cfg_word} : CFG_BASE + {3'd0, cfg_addr[4:2]};
                end
            end
            if (init != 5'd24) begin
            end else if (meas_pulse && enable && !running) begin
                running <= 1'b1; pc <= 8'd0; st <= 3'd0; li <= 3'd0; lj <= 3'd0;
            end else if (running) begin
                case (st)
                    3'd0, 3'd1, 3'd2: st <= st + 3'd1;
                    3'd3: if (f_done) begin
                        m_we <= 1'b1; m_waddr <= addr_d; m_wdata <= f_result;
                        st <= 3'd4;
                    end
                    3'd4: st <= 3'd5;
                    default: begin
                        st <= 3'd0;
                        if (r_loop && li != 3'd4) begin
                            pc <= pc - {4'd0, r_back}; li <= li + 3'd1;
                        end else begin
                            if (r_loop) begin li <= 3'd0; lj <= (lj == 3'd4) ? 3'd0 : lj + 3'd1; end
                            if (pc == STEPS - 1) begin
                                running <= 1'b0; state_valid <= 1'b1;
                                m_we <= 1'b1; m_waddr <= 6'd0; m_wdata <= beat_ts;     // the time goes in as it is
                            end
                            else pc <= pc + 8'd1;
                        end
                    end
                endcase
            end
        end
    end

    assign state_ts    = m_first[31:0];
    assign state_x     = m_first[63:32];
    assign state_y     = m_first[95:64];
    assign state_theta = m_first[127:96];
    assign state_omega = m_first[159:128];
    assign state_v     = m_first[191:160];

    wire _unused = &{f_busy, 1'b0};
endmodule

// WORDS x 32 latch memory. Same write scheme as lidar_latch_mem: registered data, one
// falling-edge flip-flop per word as the latch enable, writes at least 3 clocks apart.
module filter_latch_mem #(
    parameter WORDS = 44
) (
    input  logic        clk,
    input  logic        we,
    input  logic [5:0]  waddr,
    input  logic [31:0] wdata,
    input  logic [5:0]  raddr,
    output logic [31:0] rdata,
    output logic [191:0] first6
);
    logic        we_q;
    logic [5:0]  wa_q;
    logic [31:0] wd_q;
    always_ff @(posedge clk) begin
        we_q <= we;
        if (we) begin
            wa_q <= waddr;
            wd_q <= wdata;
        end
    end

    logic [WORDS-1:0] gate;
    integer n;
    always_ff @(negedge clk) begin
        for (n = 0; n < WORDS; n = n + 1)
            gate[n] <= we_q && (wa_q == n[5:0]);
    end

    wire [31:0] q [0:63];
    assign rdata = q[raddr];

    genvar i;
    generate
        for (i = 0; i < 64; i = i + 1) begin : word
            if (i < WORDS) begin : real_
`ifdef SYNTHESIS
                genvar b;
                for (b = 0; b < 32; b = b + 1) begin : bit_
                    sky130_fd_sc_hd__dlxtp_1 u_latch (.D(wd_q[b]), .GATE(gate[i]), .Q(q[i][b]));
                end
`else
                reg [31:0] d;
                always_latch if (gate[i]) d = wd_q;
                assign q[i] = d;
`endif
            end else begin : none_
                assign q[i] = 32'd0;
            end
        end
        for (i = 0; i < 6; i = i + 1) begin : outw
            assign first6[32*i +: 32] = q[i];
        end
    endgenerate
endmodule
