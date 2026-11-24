// AREA PROBE ONLY. Not a filter: the step table and the constants below are random, so the numbers
// it produces mean nothing. It has the hardware of a microcoded single-precision filter of the
// size the filter team plans (register map in Dev/pred_microcode.sv on UKF_jimpop), so the
// place-and-route flow can tell us what that costs:
//   - 64 32-bit words kept in latches, one read port, the first six wired straight out
//   - 48 fixed 32-bit constants (a table in logic, no storage)
//   - one bit-serial fp32 add / subtract / multiply / divide unit (fp_serial; it has no square root yet)
//   - a 192-entry step table with two loop counters
//   - sensor words selectable as an operand, and byte-wide tuning writes from the SPI block
//   - the six output words double as the host-written state
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
    localparam WORDS = 64;
    localparam STEPS = 192;
    localparam [5:0] CFG_BASE = WORDS - 8;      // tuning words sit at the top of the store

    // ---- step table ----
    logic [7:0]  pc;
    logic [29:0] rom;
    always_comb begin
        case (pc)
            8'd0: rom = 30'h007e2211;
            8'd1: rom = 30'h003ae5ab;
            8'd2: rom = 30'h000d19b0;
            8'd3: rom = 30'h00254728;
            8'd4: rom = 30'h00084d29;
            8'd5: rom = 30'h01ad8618;
            8'd6: rom = 30'h0121882d;
            8'd7: rom = 30'h00aae99f;
            8'd8: rom = 30'h2d4b0399;
            8'd9: rom = 30'h00431388;
            8'd10: rom = 30'h00147209;
            8'd11: rom = 30'h003e55a7;
            8'd12: rom = 30'h001a1181;
            8'd13: rom = 30'h000e4610;
            8'd14: rom = 30'h01045224;
            8'd15: rom = 30'h012dc192;
            8'd16: rom = 30'h01298c96;
            8'd17: rom = 30'h2c06ea29;
            8'd18: rom = 30'h000cce0e;
            8'd19: rom = 30'h001262b8;
            8'd20: rom = 30'h007a8aa1;
            8'd21: rom = 30'h005b1f0b;
            8'd22: rom = 30'h001e8cb7;
            8'd23: rom = 30'h00815726;
            8'd24: rom = 30'h011447a9;
            8'd25: rom = 30'h01a48b84;
            8'd26: rom = 30'h230e179b;
            8'd27: rom = 30'h0021810d;
            8'd28: rom = 30'h00259921;
            8'd29: rom = 30'h00065091;
            8'd30: rom = 30'h00585922;
            8'd31: rom = 30'h001b25b3;
            8'd32: rom = 30'h01820d1e;
            8'd33: rom = 30'h003ed297;
            8'd34: rom = 30'h000fd78f;
            8'd35: rom = 30'h2b215aa3;
            8'd36: rom = 30'h00090610;
            8'd37: rom = 30'h000e4796;
            8'd38: rom = 30'h00570b82;
            8'd39: rom = 30'h0027b496;
            8'd40: rom = 30'h00651c90;
            8'd41: rom = 30'h0026da1d;
            8'd42: rom = 30'h00a67512;
            8'd43: rom = 30'h00a11134;
            8'd44: rom = 30'h2b3f1218;
            8'd45: rom = 30'h0008ee91;
            8'd46: rom = 30'h004737af;
            8'd47: rom = 30'h005465a3;
            8'd48: rom = 30'h001368b8;
            8'd49: rom = 30'h0045b395;
            8'd50: rom = 30'h01ab4335;
            8'd51: rom = 30'h01838d05;
            8'd52: rom = 30'h0089989d;
            8'd53: rom = 30'h2b81c40f;
            8'd54: rom = 30'h002422a3;
            8'd55: rom = 30'h001f5d8f;
            8'd56: rom = 30'h005b9a12;
            8'd57: rom = 30'h00374c0a;
            8'd58: rom = 30'h0004ec97;
            8'd59: rom = 30'h01a31616;
            8'd60: rom = 30'h0175d1a3;
            8'd61: rom = 30'h01aa499a;
            8'd62: rom = 30'h2381418b;
            8'd63: rom = 30'h003a4697;
            8'd64: rom = 30'h002c0b05;
            8'd65: rom = 30'h00055820;
            8'd66: rom = 30'h003f81b1;
            8'd67: rom = 30'h003053aa;
            8'd68: rom = 30'h00414c20;
            8'd69: rom = 30'h0044dd18;
            8'd70: rom = 30'h01354214;
            8'd71: rom = 30'h28a7a81f;
            8'd72: rom = 30'h0038881d;
            8'd73: rom = 30'h0019ea8b;
            8'd74: rom = 30'h00200420;
            8'd75: rom = 30'h002e2d36;
            8'd76: rom = 30'h002d0e12;
            8'd77: rom = 30'h01bfdb84;
            8'd78: rom = 30'h0101dc24;
            8'd79: rom = 30'h01815214;
            8'd80: rom = 30'h2862b23a;
            8'd81: rom = 30'h002928b0;
            8'd82: rom = 30'h00282410;
            8'd83: rom = 30'h0022a5b1;
            8'd84: rom = 30'h001417a2;
            8'd85: rom = 30'h00268b2d;
            8'd86: rom = 30'h00471cb9;
            8'd87: rom = 30'h01a0cf33;
            8'd88: rom = 30'h015e13ab;
            8'd89: rom = 30'h2b020d1c;
            8'd90: rom = 30'h00276901;
            8'd91: rom = 30'h003187b2;
            8'd92: rom = 30'h002a08b3;
            8'd93: rom = 30'h0033d488;
            8'd94: rom = 30'h0037c235;
            8'd95: rom = 30'h008b5588;
            8'd96: rom = 30'h0120981e;
            8'd97: rom = 30'h01a2d58a;
            8'd98: rom = 30'h2d37c313;
            8'd99: rom = 30'h001e5a21;
            8'd100: rom = 30'h006edc23;
            8'd101: rom = 30'h002e9bb6;
            8'd102: rom = 30'h002ce592;
            8'd103: rom = 30'h002b5c83;
            8'd104: rom = 30'h00a74d9c;
            8'd105: rom = 30'h01c01b1a;
            8'd106: rom = 30'h000e18b4;
            8'd107: rom = 30'h2d08cd8a;
            8'd108: rom = 30'h002a2b0b;
            8'd109: rom = 30'h000a1d1f;
            8'd110: rom = 30'h001e9e96;
            8'd111: rom = 30'h00029321;
            8'd112: rom = 30'h005ae99d;
            8'd113: rom = 30'h00a6459a;
            8'd114: rom = 30'h002a1f98;
            8'd115: rom = 30'h0004dfab;
            8'd116: rom = 30'h228bf402;
            8'd117: rom = 30'h00211eb1;
            8'd118: rom = 30'h00060a13;
            8'd119: rom = 30'h0002b313;
            8'd120: rom = 30'h00651939;
            8'd121: rom = 30'h005ec810;
            8'd122: rom = 30'h00a9559f;
            8'd123: rom = 30'h002659a4;
            8'd124: rom = 30'h01891032;
            8'd125: rom = 30'h282d5bad;
            8'd126: rom = 30'h003324a7;
            8'd127: rom = 30'h0000a58a;
            8'd128: rom = 30'h002f0a99;
            8'd129: rom = 30'h0003cb04;
            8'd130: rom = 30'h0039f48b;
            8'd131: rom = 30'h014cc8b4;
            8'd132: rom = 30'h003f21ba;
            8'd133: rom = 30'h014a5b89;
            8'd134: rom = 30'h26012a19;
            8'd135: rom = 30'h004c2288;
            8'd136: rom = 30'h0035cb0f;
            8'd137: rom = 30'h00361eb8;
            8'd138: rom = 30'h000a5b2e;
            8'd139: rom = 30'h0000643a;
            8'd140: rom = 30'h01ea9a99;
            8'd141: rom = 30'h00a5d9b5;
            8'd142: rom = 30'h00174001;
            8'd143: rom = 30'h28bf5c1f;
            8'd144: rom = 30'h00058c87;
            8'd145: rom = 30'h000a468f;
            8'd146: rom = 30'h002b121b;
            8'd147: rom = 30'h0005cf0d;
            8'd148: rom = 30'h000f1f31;
            8'd149: rom = 30'h00228d8f;
            8'd150: rom = 30'h00ad970e;
            8'd151: rom = 30'h0027de9e;
            8'd152: rom = 30'h253b14ac;
            8'd153: rom = 30'h00253724;
            8'd154: rom = 30'h003f8e08;
            8'd155: rom = 30'h00146182;
            8'd156: rom = 30'h002f1587;
            8'd157: rom = 30'h002ce812;
            8'd158: rom = 30'h00a3140b;
            8'd159: rom = 30'h01204834;
            8'd160: rom = 30'h00ea82a7;
            8'd161: rom = 30'h2c178b2a;
            8'd162: rom = 30'h00041892;
            8'd163: rom = 30'h0039188c;
            8'd164: rom = 30'h00015110;
            8'd165: rom = 30'h003fc99f;
            8'd166: rom = 30'h000ba008;
            8'd167: rom = 30'h0089c034;
            8'd168: rom = 30'h00854e04;
            8'd169: rom = 30'h0185d587;
            8'd170: rom = 30'h293a879a;
            8'd171: rom = 30'h0042c78b;
            8'd172: rom = 30'h0003f2b1;
            8'd173: rom = 30'h0021cf07;
            8'd174: rom = 30'h0003069f;
            8'd175: rom = 30'h000b8bb7;
            8'd176: rom = 30'h009e0486;
            8'd177: rom = 30'h00a4c48c;
            8'd178: rom = 30'h002553a3;
            8'd179: rom = 30'h2641a4a2;
            8'd180: rom = 30'h006bce08;
            8'd181: rom = 30'h00290a03;
            8'd182: rom = 30'h000dc90c;
            8'd183: rom = 30'h001611a4;
            8'd184: rom = 30'h00238199;
            8'd185: rom = 30'h0180492e;
            8'd186: rom = 30'h010a0711;
            8'd187: rom = 30'h001e7081;
            8'd188: rom = 30'h00041982;
            8'd189: rom = 30'h00367203;
            8'd190: rom = 30'h00209e84;
            8'd191: rom = 30'h00330405;
            default: rom = 30'd0;
        endcase
    end
    wire        r_loop = rom[29];
    wire [3:0]  r_back = rom[28:25];
    wire [1:0]  r_f    = rom[24:23];
    wire [1:0]  r_op   = rom[22:21];
    wire [6:0]  r_a    = rom[20:14];
    wire [6:0]  r_b    = rom[13:7];
    wire [6:0]  r_d    = rom[6:0];

    logic [2:0] li, lj;                 // loop counters
    wire [6:0] addr_a = r_a + (r_f[0] ? {4'd0, li} : 7'd0);
    wire [6:0] addr_b = r_b + (r_f[1] ? {4'd0, lj} : 7'd0);
    wire [6:0] addr_d = r_d + (r_f[0] ? {4'd0, li} : 7'd0);

    // ---- storage ----
    logic        m_we;
    logic [5:0]  m_waddr;
    logic [6:0]  rd_addr;
    logic [31:0] m_wdata;
    wire  [31:0] m_rdata;
    wire  [191:0] m_first;
    filter_latch_mem #(.WORDS(WORDS)) u_mem (.clk(clk), .we(m_we), .waddr(m_waddr), .wdata(m_wdata),
                                             .raddr(rd_addr[5:0]), .rdata(m_rdata), .first6(m_first));

    // ---- constants ----
    logic [31:0] kconst;
    always_comb begin
        case (rd_addr[5:0])
            6'd0: kconst = 32'h3f0d492c;
            6'd1: kconst = 32'hbb6541b2;
            6'd2: kconst = 32'h3b962dce;
            6'd3: kconst = 32'hb7c7b7e1;
            6'd4: kconst = 32'hc0521f71;
            6'd5: kconst = 32'hb7669402;
            6'd6: kconst = 32'h323e13bc;
            6'd7: kconst = 32'h3dedbe49;
            6'd8: kconst = 32'hb3817fc4;
            6'd9: kconst = 32'hb332409a;
            6'd10: kconst = 32'h40a66969;
            6'd11: kconst = 32'hc3c89605;
            6'd12: kconst = 32'hb59dc2d7;
            6'd13: kconst = 32'h336463eb;
            6'd14: kconst = 32'hbcf31663;
            6'd15: kconst = 32'hbdbacf47;
            6'd16: kconst = 32'h41c80521;
            6'd17: kconst = 32'hbcdf527b;
            6'd18: kconst = 32'h397ad49a;
            6'd19: kconst = 32'hb292b5ca;
            6'd20: kconst = 32'hbc2a9b52;
            6'd21: kconst = 32'hc2546ad7;
            6'd22: kconst = 32'hbec05d65;
            6'd23: kconst = 32'h353511fb;
            6'd24: kconst = 32'hb9d8b83b;
            6'd25: kconst = 32'h3ac9e5cf;
            6'd26: kconst = 32'hc5a076c4;
            6'd27: kconst = 32'h36bce959;
            6'd28: kconst = 32'h412586a4;
            6'd29: kconst = 32'h321eb202;
            6'd30: kconst = 32'hc51a65b1;
            6'd31: kconst = 32'hb84ab8ce;
            6'd32: kconst = 32'hc4695f00;
            6'd33: kconst = 32'h4430ec31;
            6'd34: kconst = 32'hbeec4bcd;
            6'd35: kconst = 32'hb9b1fa0d;
            6'd36: kconst = 32'h3f009937;
            6'd37: kconst = 32'h414400b1;
            6'd38: kconst = 32'hbddc766b;
            6'd39: kconst = 32'h35517993;
            6'd40: kconst = 32'h38071d46;
            6'd41: kconst = 32'h44559893;
            6'd42: kconst = 32'hb94bd272;
            6'd43: kconst = 32'hbe49c967;
            6'd44: kconst = 32'h348c21fd;
            6'd45: kconst = 32'h3fbb1cf6;
            6'd46: kconst = 32'hb88aa444;
            6'd47: kconst = 32'hb8ac9ae6;
            default: kconst = 32'h3f800000;
        endcase
    end

    // ---- sensor words ----
    logic        f_load_a, f_load_b, f_start;
    logic [31:0] ext;
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
    wire [31:0] f_din = !rd_addr[6]              ? m_rdata :
                        (rd_addr[6:3] == 4'd15)  ? ext     : kconst;

    // ---- arithmetic ----
    wire         f_busy, f_done;
    wire  [31:0] f_result;
    fp_serial u_fp (.clk(clk), .rst_n(rst_n), .load_a(f_load_a), .load_b(f_load_b), .start(f_start), .op(r_op),
                    .din(f_din), .busy(f_busy), .done(f_done), .result(f_result));
    wire _unused_u = &{f_busy, 1'b0};

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
    assign rd_addr  = (st == 3'd1) ? addr_b : addr_a;

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
                        m_we <= 1'b1; m_waddr <= addr_d[5:0]; m_wdata <= f_result;
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

    wire _unused = &{addr_d[6], 1'b0};
endmodule

// WORDS x 32 latch memory. Same write scheme as lidar_latch_mem: registered data, one
// falling-edge flip-flop per word as the latch enable, writes at least 3 clocks apart.
module filter_latch_mem #(
    parameter WORDS = 64
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
