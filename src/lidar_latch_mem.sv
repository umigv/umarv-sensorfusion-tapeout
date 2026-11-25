// 256 x 16 memory made of latches, for the lidar point buffer.
//
// A latch is about half the size of a flip-flop with a write enable, and the buffer is the
// largest thing in the lidar block.
//
// RULE FOR THE USER OF THIS BLOCK: writes must be at least 3 clocks apart. The point buffer
// writes once per lidar return, which is 12 clocks apart at the fastest.
//
// How a write stays safe, with no gated clocks:
//   rising edge 0     we, waddr and wdata are registered. The data register then holds its
//                     value until the next write, at least 3 clocks later.
//   falling edge 0.5  one flip-flop per word, clocked on the FALLING edge, takes the decoded
//                     word select. Its output is that word's latch enable: a clean, glitch-free
//                     level from a flip-flop, high for exactly one clock (falling edge 0.5 to
//                     falling edge 1.5).
//   The data has been steady for half a clock before the enable rises and stays steady for at
//   least a clock and a half after it falls. Both margins are there by construction; they do
//   not rely on the tools checking a latch's timing.
// A word holds its new value from falling edge 0.5, and must not be read before rising edge 2.
//
// Reading is combinational: rdata follows raddr.
//
// An earlier version used one clock gate per word. The place-and-route tools treated the 256
// gated clocks as clock nets, gave each of them wide wiring, and could not route the result.
//
// For synthesis the latches are Sky130 cells placed by name. Simulation without the cell
// library uses the equivalent behaviour below.
module lidar_latch_mem (
    input  logic        clk,
    input  logic        we,
    input  logic [7:0]  waddr,
    input  logic [15:0] wdata,
    input  logic [7:0]  raddr,
    output logic [15:0] rdata
);

    logic        we_q;
    logic [7:0]  wa_q;
    logic [15:0] wd_q;
    always_ff @(posedge clk) begin
        we_q <= we;
        if (we) begin
            wa_q <= waddr;
            wd_q <= wdata;
        end
    end

    // TRIAL: one enable pulse for the whole memory, from a single flip-flop on the falling edge,
    // ANDed with each word's select. The select is decoded from the held address, so it is steady
    // from half a clock before the pulse rises until at least a clock and a half after it falls;
    // the AND of a steady level and a clean pulse is a clean pulse. This replaces 256 flip-flops
    // (one per word) with 256 AND gates.
    logic pulse;
    always_ff @(negedge clk) pulse <= we_q;

    wire [255:0] sel, gate;
    wire [15:0] q [0:255];

    // Reading, in two levels. Each group of 16 words has its own selector, followed by a buffer
    // placed by name; a second selector picks the group, followed by an output buffer. Without
    // the named buffers the tools build one flat 256-word selector, and wherever its words end
    // up far apart a small gate in the middle of it is left driving millimetres of wire.
    wire [15:0] grp_pre [0:15];
    wire [15:0] grp     [0:15];
    wire [15:0] rdata_pre = grp[raddr[7:4]];
    genvar g, gb;
    generate
        for (g = 0; g < 16; g = g + 1) begin : rgrp
            assign grp_pre[g] = q[{g[3:0], raddr[3:0]}];
            for (gb = 0; gb < 16; gb = gb + 1) begin : rb
`ifdef SYNTHESIS
                sky130_fd_sc_hd__buf_4 u_rbuf (.A(grp_pre[g][gb]), .X(grp[g][gb]));
`else
                assign grp[g][gb] = grp_pre[g][gb];
`endif
            end
        end
        for (gb = 0; gb < 16; gb = gb + 1) begin : ob
`ifdef SYNTHESIS
            sky130_fd_sc_hd__buf_4 u_obuf (.A(rdata_pre[gb]), .X(rdata[gb]));
`else
            assign rdata[gb] = rdata_pre[gb];
`endif
        end
    endgenerate

    genvar i;
    generate
        for (i = 0; i < 256; i = i + 1) begin : word
            assign sel[i] = (wa_q == i);
`ifdef SYNTHESIS
            sky130_fd_sc_hd__and2_4 u_gate (.A(sel[i]), .B(pulse), .X(gate[i]));
`else
            assign gate[i] = sel[i] & pulse;
`endif
`ifdef SYNTHESIS
            genvar b;
            for (b = 0; b < 16; b = b + 1) begin : bit_
                sky130_fd_sc_hd__dlxtp_1 u_latch (.D(wd_q[b]), .GATE(gate[i]), .Q(q[i][b]));
            end
`else
            reg [15:0] d;
            always_latch if (gate[i]) d = wd_q;
            assign q[i] = d;
`endif
        end
    endgenerate

endmodule
