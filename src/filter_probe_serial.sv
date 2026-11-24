// AREA PROBE ONLY. Not a filter: the table below is random, so the numbers it produces mean
// nothing. It has the hardware a small microcoded single-precision filter would have, so the
// place-and-route flow can tell us what such a filter costs:
//   - 48 32-bit words kept in latches, one read port, the first six wired straight out
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
    localparam WORDS = 48;
    localparam STEPS = 160;
    localparam [5:0] CFG_BASE = WORDS - 8;      // tuning words sit at the top of the store

    // ---- step table ----
    logic [7:0]  pc;
    logic [26:0] rom;
    always_comb begin
        case (pc)
            8'd0: rom = 27'h00f81c6;
            8'd1: rom = 27'h002508b;
            8'd2: rom = 27'h00163c7;
            8'd3: rom = 27'h00015a4;
            8'd4: rom = 27'h00dd24b;
            8'd5: rom = 27'h0040825;
            8'd6: rom = 27'h006a8a3;
            8'd7: rom = 27'h0020948;
            8'd8: rom = 27'h54a94e4;
            8'd9: rom = 27'h00913d2;
            8'd10: rom = 27'h00a1a87;
            8'd11: rom = 27'h00b9106;
            8'd12: rom = 27'h0089b44;
            8'd13: rom = 27'h008a0ca;
            8'd14: rom = 27'h03ca129;
            8'd15: rom = 27'h014158d;
            8'd16: rom = 27'h01aa4e3;
            8'd17: rom = 27'h522a091;
            8'd18: rom = 27'h00894cf;
            8'd19: rom = 27'h008291e;
            8'd20: rom = 27'h009f563;
            8'd21: rom = 27'h0082790;
            8'd22: rom = 27'h00baba7;
            8'd23: rom = 27'h02911d1;
            8'd24: rom = 27'h038784c;
            8'd25: rom = 27'h03aa402;
            8'd26: rom = 27'h45a544f;
            8'd27: rom = 27'h009cb8a;
            8'd28: rom = 27'h0019349;
            8'd29: rom = 27'h00823a6;
            8'd30: rom = 27'h002c519;
            8'd31: rom = 27'h0084956;
            8'd32: rom = 27'h030c19f;
            8'd33: rom = 27'h037b815;
            8'd34: rom = 27'h01a0691;
            8'd35: rom = 27'h4ac35a8;
            8'd36: rom = 27'h005f8e4;
            8'd37: rom = 27'h0088209;
            8'd38: rom = 27'h00ca245;
            8'd39: rom = 27'h002d510;
            8'd40: rom = 27'h00eaac9;
            8'd41: rom = 27'h010d0a1;
            8'd42: rom = 27'h028d617;
            8'd43: rom = 27'h029b52a;
            8'd44: rom = 27'h547c417;
            8'd45: rom = 27'h0099917;
            8'd46: rom = 27'h0087b1e;
            8'd47: rom = 27'h008d4df;
            8'd48: rom = 27'h0016714;
            8'd49: rom = 27'h008a619;
            8'd50: rom = 27'h001e659;
            8'd51: rom = 27'h038b644;
            8'd52: rom = 27'h02655dc;
            8'd53: rom = 27'h4b6a787;
            8'd54: rom = 27'h00aa187;
            8'd55: rom = 27'h00bd78a;
            8'd56: rom = 27'h006555c;
            8'd57: rom = 27'h0093adb;
            8'd58: rom = 27'h00d2912;
            8'd59: rom = 27'h0284893;
            8'd60: rom = 27'h0294757;
            8'd61: rom = 27'h000f41d;
            8'd62: rom = 27'h4742a88;
            8'd63: rom = 27'h006a25a;
            8'd64: rom = 27'h00966c5;
            8'd65: rom = 27'h0013425;
            8'd66: rom = 27'h00fe39c;
            8'd67: rom = 27'h0085a47;
            8'd68: rom = 27'h02e4390;
            8'd69: rom = 27'h038888a;
            8'd70: rom = 27'h014a826;
            8'd71: rom = 27'h4da3928;
            8'd72: rom = 27'h00e6708;
            8'd73: rom = 27'h00845e0;
            8'd74: rom = 27'h00269ca;
            8'd75: rom = 27'h008e9d2;
            8'd76: rom = 27'h00a1b1e;
            8'd77: rom = 27'h03ff0c1;
            8'd78: rom = 27'h00926db;
            8'd79: rom = 27'h03693c7;
            8'd80: rom = 27'h56da2ca;
            8'd81: rom = 27'h000d329;
            8'd82: rom = 27'h0060216;
            8'd83: rom = 27'h002675d;
            8'd84: rom = 27'h00ee5d8;
            8'd85: rom = 27'h009a5d4;
            8'd86: rom = 27'h0094957;
            8'd87: rom = 27'h000d8aa;
            8'd88: rom = 27'h02f8891;
            8'd89: rom = 27'h4e91a51;
            8'd90: rom = 27'h008a005;
            8'd91: rom = 27'h008f322;
            8'd92: rom = 27'h00ae2dc;
            8'd93: rom = 27'h00da865;
            8'd94: rom = 27'h00e57a4;
            8'd95: rom = 27'h03979cc;
            8'd96: rom = 27'h03a520c;
            8'd97: rom = 27'h03868a9;
            8'd98: rom = 27'h4d4455f;
            8'd99: rom = 27'h00b9b67;
            8'd100: rom = 27'h00998d2;
            8'd101: rom = 27'h00deae3;
            8'd102: rom = 27'h009c84c;
            8'd103: rom = 27'h009d141;
            8'd104: rom = 27'h0166551;
            8'd105: rom = 27'h028c407;
            8'd106: rom = 27'h0266188;
            8'd107: rom = 27'h5a1b51b;
            8'd108: rom = 27'h00a56ea;
            8'd109: rom = 27'h0058646;
            8'd110: rom = 27'h00ba998;
            8'd111: rom = 27'h00a73cf;
            8'd112: rom = 27'h0067722;
            8'd113: rom = 27'h02a1461;
            8'd114: rom = 27'h02a47da;
            8'd115: rom = 27'h024a2a0;
            8'd116: rom = 27'h4564883;
            8'd117: rom = 27'h008d6e2;
            8'd118: rom = 27'h002364a;
            8'd119: rom = 27'h00043de;
            8'd120: rom = 27'h00d83d9;
            8'd121: rom = 27'h00bb527;
            8'd122: rom = 27'h03aa119;
            8'd123: rom = 27'h010925e;
            8'd124: rom = 27'h01a1219;
            8'd125: rom = 27'h5226217;
            8'd126: rom = 27'h00af797;
            8'd127: rom = 27'h008a122;
            8'd128: rom = 27'h005c3ca;
            8'd129: rom = 27'h0089a05;
            8'd130: rom = 27'h0089653;
            8'd131: rom = 27'h014d560;
            8'd132: rom = 27'h023c314;
            8'd133: rom = 27'h00815e2;
            8'd134: rom = 27'h585a6e5;
            8'd135: rom = 27'h0095a67;
            8'd136: rom = 27'h006592a;
            8'd137: rom = 27'h0093787;
            8'd138: rom = 27'h0012b2a;
            8'd139: rom = 27'h001c915;
            8'd140: rom = 27'h038d0a1;
            8'd141: rom = 27'h009a2d1;
            8'd142: rom = 27'h00ee5c2;
            8'd143: rom = 27'h477d08e;
            8'd144: rom = 27'h00ab807;
            8'd145: rom = 27'h00a00d7;
            8'd146: rom = 27'h00810a0;
            8'd147: rom = 27'h0081b58;
            8'd148: rom = 27'h004e9e5;
            8'd149: rom = 27'h00d23e2;
            8'd150: rom = 27'h01902a0;
            8'd151: rom = 27'h0092a5a;
            8'd152: rom = 27'h49c2a59;
            8'd153: rom = 27'h00df061;
            8'd154: rom = 27'h007e6db;
            8'd155: rom = 27'h00d2181;
            8'd156: rom = 27'h00ee682;
            8'd157: rom = 27'h00a1b43;
            8'd158: rom = 27'h00a12c4;
            8'd159: rom = 27'h0029945;
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
