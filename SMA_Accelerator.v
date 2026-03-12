// ============================================================
// SMA_Accelerator.v  -- FULLY CORRECTED
// Accelerator-II: Symmetric-Matrix-based Accumulation (SMA)
// Tu et al., IEEE TCAS-II, Vol.72, No.1, Jan 2025
//
// All bugs fixed (verified against Algorithm 2 for N=4,H=2):
//
//  [FIX-A] CSR rotates RIGHT (not left) during computation
//          Right: CSR[k] = B[(k-i) mod N] after i cycles ✓
//          Left gave B[(k+i) mod N] - wrong B indexing
//
//  [FIX-B] Per-lane sign control for bc0 (Step-I)
//          sign_k = (d_cnt > k): lane k gets negative bc0
//          when d_cnt exceeds k (negacyclic boundary crossing)
//
//  [FIX-C] Step-II bc assignments corrected:
//          Step-II bc0 = CSR[k]           (no negate)
//          Step-II bc1 = sign_k ? -CSR[k+H] : +CSR[k+H]
//
//  [FIX-D] H[8:0]-1 underflow: use H_MINUS1 = 511
//
//  [FIX-E] pair_sum (33-bit) sign-extended to AC_WIDTH=42 bits
//
// Interface timing:
//   start → S_LOADB (N cycles b_valid)
//         → S_LOADD (N cycles d_valid)
//         → S_COMP  (Step-I: H cycles, Step-II: H cycles)
//         → S_OUTPUT (BUF2 drain: H cycles)
//         → S_DONE  (done=1)
//   BUF1 drains concurrently with Step-II (output overlap)
//   Latency from end of D load: 3H = 3*N/2 = 1536 cycles
// ============================================================
`timescale 1ns/1ps

module SMA_Accelerator (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire signed [1:0]  b_in,
    input  wire               b_valid,
    input  wire signed [31:0] d_in,
    input  wire               d_valid,
    output wire signed [31:0] w_out,
    output wire               w_valid,
    output reg                done
);

localparam N        = 1024;
localparam H        = N / 2;          // 512
localparam AC_WIDTH = 42;
localparam H_MINUS1 = H - 1;          // 511 - safe 9-bit terminal value [FIX-D]

localparam S_IDLE   = 3'd0;
localparam S_LOADB  = 3'd1;
localparam S_LOADD  = 3'd2;
localparam S_COMP   = 3'd3;
localparam S_OUTPUT = 3'd4;
localparam S_DONE   = 3'd5;
reg [2:0] state;

// ---- CSR: B coefficients (right-rotate during computation) -
(* ram_style = "distributed" *) reg signed [1:0] CSR [0:N-1];

// ---- D register file: dual-read access [D0=D[i], D1=D[i+H]]-
(* ram_style = "distributed" *) reg signed [31:0] D_reg [0:N-1];

// ---- Accumulators (no DSP) ---------------------------------
(* use_dsp = "no" *) reg signed [AC_WIDTH-1:0] AC1 [0:H-1];
(* use_dsp = "no" *) reg signed [AC_WIDTH-1:0] AC2 [0:H-1];

// ---- Output buffers ----------------------------------------
(* keep = "true" *) reg signed [31:0] BUF1 [0:H-1];
(* keep = "true" *) reg signed [31:0] BUF2 [0:H-1];

// ---- Drain counters (9-bit: 0..511) ----------------------
reg               buf1_valid, buf2_valid;
reg [8:0]         buf1_cnt;
reg [8:0]         buf2_cnt;

// ---- General counters -------------------------------------
reg [9:0] b_cnt;
reg [9:0] d_load;
reg [9:0] d_cnt;
reg       step;        // 0 = Step-I,  1 = Step-II

// ---- Simultaneous D0 / D1 read ----------------------------
wire signed [31:0] d0 = D_reg[d_cnt];
wire signed [31:0] d1 = D_reg[d_cnt + 10'd512];  // d_cnt + H

// ====================================================================
// Combinational: per-lane ternary products + addition-first pair_sum
//
// After i right-rotations of CSR: CSR[k] = B[(k-i) mod N]
//
// Step-I (step=0): computing Block-1 = W[0..H-1]
//   D0[i] paired with B[(k-i) mod N], sign = -1 when i > k
//   D1[i] paired with -B[(k-i+H) mod N] (always negated)
//
//   bc0[k] = (d_cnt > k) ? -CSR[k] : +CSR[k]   [FIX-B]
//   bc1[k] = -CSR[k+H]                           (always)
//
// Step-II (step=1): computing Block-2 = W[H..N-1]
//   D0[i] paired with B[(k+H-i) mod N] (always positive)
//   D1[i] paired with B[(k-i) mod N], sign = -1 when i > k
//
//   bc0[k] = +CSR[k]                             (always)  [FIX-C]
//   bc1[k] = (d_cnt > k) ? -CSR[k+H] : +CSR[k+H]         [FIX-C]
// ====================================================================

wire signed [31:0] prod0     [0:H-1];
wire signed [31:0] prod1     [0:H-1];
wire signed [32:0] pair_sum  [0:H-1];

genvar k;
generate
    for (k = 0; k < H; k = k+1) begin : LANE
        wire sign_k = (d_cnt > k[9:0]);  // per-lane negacyclic boundary [FIX-B]

        // Step-I: bc0 conditional, bc1 always negated
        // Step-II: bc0 always direct, bc1 conditional   [FIX-C]
        wire signed [1:0] bc0 = (!step && sign_k) ? (-CSR[k]) : CSR[k];
        wire signed [1:0] bc1 = ( step && !sign_k) ? CSR[k+H] : (-CSR[k+H]);

        // Ternary multiply - LUT-based, 0 DSP
        assign prod0[k] = (bc0 == 2'sb01) ?  d0 :
                          (bc0 == 2'sb11) ? -d0 : 32'sd0;
        assign prod1[k] = (bc1 == 2'sb01) ?  d1 :
                          (bc1 == 2'sb11) ? -d1 : 32'sd0;

        // Addition-first: add pair before accumulation → H ACs instead of N
        assign pair_sum[k] = $signed({prod0[k][31], prod0[k]})
                           + $signed({prod1[k][31], prod1[k]});
    end
endgenerate

// ---- Output ------------------------------------------------
assign w_valid = buf1_valid | buf2_valid;
assign w_out   = buf1_valid ? BUF1[buf1_cnt] : BUF2[buf2_cnt];

integer i;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state      <= S_IDLE;
        b_cnt      <= 10'd0; d_load <= 10'd0; d_cnt  <= 10'd0;
        step       <= 1'b0;
        buf1_cnt   <= 9'd0;  buf2_cnt  <= 9'd0;
        buf1_valid <= 1'b0;  buf2_valid <= 1'b0;
        done       <= 1'b0;
        for (i = 0; i < N; i = i+1) begin
            CSR[i]   <= 2'sd0;
            D_reg[i] <= 32'sd0;
        end
        for (i = 0; i < H; i = i+1) begin
            AC1[i]  <= {AC_WIDTH{1'b0}};  AC2[i]  <= {AC_WIDTH{1'b0}};
            BUF1[i] <= 32'sd0;            BUF2[i] <= 32'sd0;
        end
    end else begin

        // ---- BUF1 drains concurrently with Step-II ---------
        if (buf1_valid) begin
            if (buf1_cnt == H_MINUS1[8:0]) begin   // 9'd511 [FIX-D]
                buf1_valid <= 1'b0;
                buf1_cnt   <= 9'd0;
            end else
                buf1_cnt <= buf1_cnt + 9'd1;
        end

        case (state)

        S_IDLE: begin
            done <= 1'b0;
            if (start) state <= S_LOADB;
        end

        // Load N B-coefficients into CSR via left-shift (N cycles)
        // Left-shift load: CSR[j] = B[j] after N cycles
        S_LOADB: begin
            if (b_valid) begin
                for (i = 0; i < N-1; i = i+1)
                    CSR[i] <= CSR[i+1];
                CSR[N-1] <= b_in;
                if (b_cnt == 10'd1023) begin
                    b_cnt <= 10'd0;
                    state <= S_LOADD;
                end else
                    b_cnt <= b_cnt + 10'd1;
            end
        end

        // Load N D-coefficients into D_reg (N cycles)
        S_LOADD: begin
            if (d_valid) begin
                D_reg[d_load] <= d_in;
                if (d_load == 10'd1023) begin
                    d_load <= 10'd0;
                    state  <= S_COMP;
                end else
                    d_load <= d_load + 10'd1;
            end
        end

        // Step-I then Step-II: each H=512 cycles, auto-clocked
        S_COMP: begin
            // [FIX-E] sign-extend 33-bit pair_sum to AC_WIDTH=42 bits
            for (i = 0; i < H; i = i+1) begin
                if (step == 1'b0)
                    AC1[i] <= AC1[i]
                            + {{(AC_WIDTH-33){pair_sum[i][32]}}, pair_sum[i]};
                else
                    AC2[i] <= AC2[i]
                            + {{(AC_WIDTH-33){pair_sum[i][32]}}, pair_sum[i]};
            end

            // [FIX-A] CSR rotates RIGHT each cycle:
            // CSR[k] = B[(k-d_cnt) mod N] - correct negacyclic indexing
            begin : csr_rot_right
                reg signed [1:0] tail;
                tail = CSR[N-1];
                for (i = N-1; i > 0; i = i-1)
                    CSR[i] <= CSR[i-1];
                CSR[0] <= tail;
            end

            if (d_cnt == 10'd511) begin   // H-1 = 511 [FIX-D]
                d_cnt <= 10'd0;

                if (step == 1'b0) begin
                    // End of Step-I → latch Block-1 to BUF1, start drain
                    for (i = 0; i < H; i = i+1)
                        BUF1[i] <= AC1[i][31:0] + pair_sum[i][31:0];
                    for (i = 0; i < H; i = i+1)
                        AC1[i] <= {AC_WIDTH{1'b0}};
                    buf1_valid <= 1'b1;
                    buf1_cnt   <= 9'd0;
                    step       <= 1'b1;
                end else begin
                    // End of Step-II → latch Block-2 to BUF2
                    for (i = 0; i < H; i = i+1)
                        BUF2[i] <= AC2[i][31:0] + pair_sum[i][31:0];
                    for (i = 0; i < H; i = i+1)
                        AC2[i] <= {AC_WIDTH{1'b0}};
                    buf2_valid <= 1'b1;
                    buf2_cnt   <= 9'd0;
                    step       <= 1'b0;
                    state      <= S_OUTPUT;
                end
            end else
                d_cnt <= d_cnt + 10'd1;
        end

        // BUF2 drain (BUF1 already drained during Step-II)
        S_OUTPUT: begin
            if (buf2_valid) begin
                if (buf2_cnt == H_MINUS1[8:0]) begin   // 9'd511 [FIX-D]
                    buf2_valid <= 1'b0;
                    buf2_cnt   <= 9'd0;
                end else
                    buf2_cnt <= buf2_cnt + 9'd1;
            end else
                state <= S_DONE;
        end

        S_DONE: done <= 1'b1;

        default: state <= S_IDLE;
        endcase
    end
end

endmodule