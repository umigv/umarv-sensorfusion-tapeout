// AREA PROBE ONLY. Not a filter. It has what a microcoded fp32 filter has in the way of hardware,
// so the place-and-route flow can tell us what such a filter costs:
//   - WORDS 32-bit registers with two read ports and one write port
//   - the team's fp32 multiply-accumulate and divide/square-root units
//   - a sequencer that keeps reading two registers, running a unit, and writing the result back
// A real filter also has a microcode table (a few hundred short entries), which is not here.
module filter_probe #(
    parameter WORDS = 88
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        meas_pulse,
    input  logic [31:0] beat_ts,
    input  logic [31:0] vn_yaw_f32, vn_rate_f32,
    input  logic [63:0] vn_lat_f64, vn_lon_f64,
    input  logic signed [27:0] dw_delta,
    input  logic        cfg_we,
    input  logic [6:0]  cfg_addr,
    input  logic [7:0]  cfg_wdata,
    output logic        state_valid,
    output logic [31:0] state_ts, state_x, state_y, state_theta, state_omega, state_v
);
    logic [31:0] regs [0:WORDS-1];
    logic [6:0]  pa, pb, pd;
    logic [9:0]  pc;
    logic [1:0]  st;                    // 0 issue, 1 wait, 2 write back
    wire         use_div = (pc[2:0] == 3'd7);

    wire [31:0] ra = regs[pa];
    wire [31:0] rb = regs[pb];

    logic        mac_start, ds_start;
    wire         mac_busy, mac_done, mac_valid, ds_busy, ds_done;
    wire  [31:0] mac_result, ds_result;
    wire  [4:0]  mac_flags;
    wire         ds_f0, ds_f1, ds_f2, ds_f3, ds_f4;

    fp32_mac u_mac (.clk(clk), .rst_n(rst_n), .start(mac_start), .clear(pc[1:0] == 2'd0), .last(pc[1:0] == 2'd3),
                    .a(ra), .b(rb), .busy(mac_busy), .done(mac_done), .result_valid(mac_valid), .result(mac_result),
                    .exception_flags(mac_flags));
    fp32_divsqrt u_ds (.clk(clk), .rst_n(rst_n), .start(ds_start), .mode(pc[3]), .a(ra), .b(rb), .busy(ds_busy), .done(ds_done),
                       .result(ds_result), .invalid(ds_f0), .divide_by_zero(ds_f1), .overflow(ds_f2), .underflow(ds_f3), .inexact(ds_f4));

    function automatic [6:0] next_ptr;
        input [6:0] p;
        input [6:0] step;
        reg [7:0] t;
        begin
            t = {1'b0, p} + {1'b0, step};
            next_ptr = (t >= WORDS) ? (t - WORDS) : t[6:0];
        end
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pa <= 7'd0; pb <= 7'd1; pd <= 7'd2; pc <= 10'd0; st <= 2'd0;
            mac_start <= 1'b0; ds_start <= 1'b0; state_valid <= 1'b0;
        end else begin
            mac_start <= 1'b0; ds_start <= 1'b0; state_valid <= 1'b0;
            if (meas_pulse) begin
                regs[0] <= vn_yaw_f32;  regs[1] <= vn_rate_f32;
                regs[2] <= vn_lat_f64[63:32]; regs[3] <= vn_lat_f64[31:0];
                regs[4] <= vn_lon_f64[63:32]; regs[5] <= vn_lon_f64[31:0];
                regs[6] <= {{4{dw_delta[27]}}, dw_delta}; regs[7] <= beat_ts;
                pc <= 10'd0; st <= 2'd0;
            end else if (cfg_we && cfg_addr >= 7'h6C) begin
                regs[8 + cfg_addr[3:2]][8*cfg_addr[1:0] +: 8] <= cfg_wdata;
            end else begin
                case (st)
                    2'd0: if (!mac_busy && !ds_busy) begin
                        if (use_div) ds_start <= 1'b1; else mac_start <= 1'b1;
                        st <= 2'd1;
                    end
                    2'd1: if (mac_done || ds_done) st <= 2'd2;
                    default: begin
                        if (pd >= 7'd12) regs[pd] <= use_div ? ds_result : mac_result;
                        pa <= next_ptr(pa, 7'd3); pb <= next_ptr(pb, 7'd5); pd <= next_ptr(pd, 7'd7);
                        pc <= pc + 10'd1;
                        state_valid <= (pc == 10'd300);
                        st <= 2'd0;
                    end
                endcase
            end
        end
    end

    assign state_ts    = regs[7];
    assign state_x     = regs[12];
    assign state_y     = regs[13];
    assign state_theta = regs[14];
    assign state_omega = regs[15];
    assign state_v     = regs[16];

    wire _unused = &{mac_valid, mac_flags, ds_f0, ds_f1, ds_f2, ds_f3, ds_f4, 1'b0};
endmodule
