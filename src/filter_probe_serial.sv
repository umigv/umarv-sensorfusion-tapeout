// AREA PROBE ONLY. Not a filter: the table below is random, so the numbers it produces mean
// nothing. It has the hardware a small microcoded single-precision filter would have, so the
// place-and-route flow can tell us what such a filter costs:
//   - 40 32-bit words kept in latches, one read port, the first six wired straight out
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
    localparam WORDS = 40;
    localparam STEPS = 160;
    localparam [5:0] CFG_BASE = WORDS - 8;      // tuning words sit at the top of the store

    // ---- step table ----
    logic [7:0]  pc;
    logic [26:0] rom;
    always_comb begin
        case (pc)
            8'd0: rom = 27'h00f81c6;
            8'd1: rom = 27'h002509b;
            8'd2: rom = 27'h000658d;
            8'd3: rom = 27'h0001051;
            8'd4: rom = 27'h00de9d4;
            8'd5: rom = 27'h004b1e1;
            8'd6: rom = 27'h030f11b;
            8'd7: rom = 27'h0384156;
            8'd8: rom = 27'h46c44d5;
            8'd9: rom = 27'h00913da;
            8'd10: rom = 27'h005d85b;
            8'd11: rom = 27'h0039808;
            8'd12: rom = 27'h0015244;
            8'd13: rom = 27'h008a0c8;
            8'd14: rom = 27'h03ca117;
            8'd15: rom = 27'h014159e;
            8'd16: rom = 27'h0113747;
            8'd17: rom = 27'h4c8b44a;
            8'd18: rom = 27'h0089747;
            8'd19: rom = 27'h00d8595;
            8'd20: rom = 27'h00a5755;
            8'd21: rom = 27'h001e9a2;
            8'd22: rom = 27'h007a71d;
            8'd23: rom = 27'h0392449;
            8'd24: rom = 27'h035b1d6;
            8'd25: rom = 27'h0318402;
            8'd26: rom = 27'h4591261;
            8'd27: rom = 27'h009c108;
            8'd28: rom = 27'h008d0e2;
            8'd29: rom = 27'h0082396;
            8'd30: rom = 27'h00144d4;
            8'd31: rom = 27'h0025421;
            8'd32: rom = 27'h030c192;
            8'd33: rom = 27'h037b80d;
            8'd34: rom = 27'h01a068b;
            8'd35: rom = 27'h4ac3597;
            8'd36: rom = 27'h005f8d5;
            8'd37: rom = 27'h0088207;
            8'd38: rom = 27'h00ca245;
            8'd39: rom = 27'h0014286;
            8'd40: rom = 27'h00c524c;
            8'd41: rom = 27'h031950c;
            8'd42: rom = 27'h02a24d3;
            8'd43: rom = 27'h01a215e;
            8'd44: rom = 27'h4fbc661;
            8'd45: rom = 27'h00d1849;
            8'd46: rom = 27'h009134f;
            8'd47: rom = 27'h00d90d1;
            8'd48: rom = 27'h008e7cb;
            8'd49: rom = 27'h0093008;
            8'd50: rom = 27'h03936e0;
            8'd51: rom = 27'h0199782;
            8'd52: rom = 27'h029634b;
            8'd53: rom = 27'h579e05f;
            8'd54: rom = 27'h0086051;
            8'd55: rom = 27'h00bd10a;
            8'd56: rom = 27'h00d559e;
            8'd57: rom = 27'h0093950;
            8'd58: rom = 27'h00d290c;
            8'd59: rom = 27'h028489c;
            8'd60: rom = 27'h01d5810;
            8'd61: rom = 27'h039115e;
            8'd62: rom = 27'h548f411;
            8'd63: rom = 27'h0041647;
            8'd64: rom = 27'h000d243;
            8'd65: rom = 27'h00966d6;
            8'd66: rom = 27'h003e4ce;
            8'd67: rom = 27'h00a5391;
            8'd68: rom = 27'h0081950;
            8'd69: rom = 27'h01ca795;
            8'd70: rom = 27'h00498a0;
            8'd71: rom = 27'h596081c;
            8'd72: rom = 27'h009421c;
            8'd73: rom = 27'h00a4899;
            8'd74: rom = 27'h00dc091;
            8'd75: rom = 27'h00176a2;
            8'd76: rom = 27'h00269c8;
            8'd77: rom = 27'h01bf303;
            8'd78: rom = 27'h039f0c6;
            8'd79: rom = 27'h00926d0;
            8'd80: rom = 27'h51c1598;
            8'd81: rom = 27'h009a2c8;
            8'd82: rom = 27'h000d320;
            8'd83: rom = 27'h00e78dd;
            8'd84: rom = 27'h006021b;
            8'd85: rom = 27'h0083994;
            8'd86: rom = 27'h0292653;
            8'd87: rom = 27'h0192022;
            8'd88: rom = 27'h02b81d7;
            8'd89: rom = 27'h45e25d7;
            8'd90: rom = 27'h0054483;
            8'd91: rom = 27'h008b74b;
            8'd92: rom = 27'h002268d;
            8'd93: rom = 27'h005c460;
            8'd94: rom = 27'h0056960;
            8'd95: rom = 27'h039e796;
            8'd96: rom = 27'h0286861;
            8'd97: rom = 27'h0186795;
            8'd98: rom = 27'h55103dc;
            8'd99: rom = 27'h0039552;
            8'd100: rom = 27'h00a1592;
            8'd101: rom = 27'h008c995;
            8'd102: rom = 27'h0094716;
            8'd103: rom = 27'h0022741;
            8'd104: rom = 27'h010b54b;
            8'd105: rom = 27'h028c406;
            8'd106: rom = 27'h0246087;
            8'd107: rom = 27'h52954d8;
            8'd108: rom = 27'h00a4292;
            8'd109: rom = 27'h0080499;
            8'd110: rom = 27'h00fa4a0;
            8'd111: rom = 27'h00a73ca;
            8'd112: rom = 27'h0067719;
            8'd113: rom = 27'h03a0556;
            8'd114: rom = 27'h039941b;
            8'd115: rom = 27'h03d43e0;
            8'd116: rom = 27'h499a281;
            8'd117: rom = 27'h0064891;
            8'd118: rom = 27'h008d6d4;
            8'd119: rom = 27'h002365e;
            8'd120: rom = 27'h00c4120;
            8'd121: rom = 27'h007b61c;
            8'd122: rom = 27'h038f4e0;
            8'd123: rom = 27'h02a1854;
            8'd124: rom = 27'h020180b;
            8'd125: rom = 27'h4b5b2d6;
            8'd126: rom = 27'h0053195;
            8'd127: rom = 27'h009598a;
            8'd128: rom = 27'h009b78e;
            8'd129: rom = 27'h008a104;
            8'd130: rom = 27'h005c3de;
            8'd131: rom = 27'h020944a;
            8'd132: rom = 27'h01bc317;
            8'd133: rom = 27'h018d55d;
            8'd134: rom = 27'h50e15a2;
            8'd135: rom = 27'h004e8c6;
            8'd136: rom = 27'h001770d;
            8'd137: rom = 27'h001a6d5;
            8'd138: rom = 27'h0095956;
            8'd139: rom = 27'h006591e;
            8'd140: rom = 27'h03d3798;
            8'd141: rom = 27'h0012909;
            8'd142: rom = 27'h0199682;
            8'd143: rom = 27'h533d1d3;
            8'd144: rom = 27'h004b9c8;
            8'd145: rom = 27'h008a39f;
            8'd146: rom = 27'h0020607;
            8'd147: rom = 27'h0051806;
            8'd148: rom = 27'h00a00df;
            8'd149: rom = 27'h0082695;
            8'd150: rom = 27'h020b3a1;
            8'd151: rom = 27'h03c348d;
            8'd152: rom = 27'h5190293;
            8'd153: rom = 27'h00a100f;
            8'd154: rom = 27'h00be98c;
            8'd155: rom = 27'h00424c1;
            8'd156: rom = 27'h00816c2;
            8'd157: rom = 27'h00959c3;
            8'd158: rom = 27'h0024684;
            8'd159: rom = 27'h009a705;
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
