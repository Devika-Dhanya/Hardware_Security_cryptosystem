// ============================================================
// PSC_Accelerator.v -- MUX-based PSC Architecture (Corrected)
// Tu et al., IEEE TCAS-II 2025, Algorithm 1 (PSC)
//
// Change ONLY "parameter V" to switch configurations:
// V=32 → U=32 (lightweight)
// V=64 → U=16
// V=128 → U=8
// V=256 → U=4
// ============================================================
`timescale 1ns/1ps

module PSC_Accelerator #(
parameter V = 64
) (
input wire clk,
input wire rst_n,
input wire start,
input wire signed [1:0] b_in,
input wire b_valid,
input wire signed [31:0] d_in,
input wire d_valid,
output wire signed [31:0] w_out,
output wire w_valid,
output reg done
);

// ---- Derived parameters (do not edit) ----------------------
localparam N = 1024;
localparam U = N / V;
localparam K = 32; // D/W bit-width
localparam AC_WIDTH = K + 10; // 42-bit: no overflow for N=1024
localparam LOG2V = $clog2(V); // shift amount: u_cnt<<LOG2V = u_cnt*V

// Parameterized counter widths
localparam BCW = $clog2(N); // 10 bits
localparam DCW = $clog2(N); // 10 bits
localparam UCW = (U > 1) ? $clog2(U) : 1;
localparam BUFCW = $clog2(V);

// ---- FSM states --------------------------------------------
localparam S_IDLE = 3'd0;
localparam S_LOAD = 3'd1;
localparam S_COMP = 3'd2;
localparam S_OUTPUT = 3'd3;
localparam S_DONE = 3'd4;
reg [2:0] state;

// ---- B storage: Distributed LUT-RAM ------------------------
(* ram_style = "distributed" *) reg signed [1:0] B [0:N-1];

// ---- V parallel accumulators (LUT adders, no DSP) ----------
(* use_dsp = "no" *)
reg signed [AC_WIDTH-1:0] AC [0:V-1];

// ---- Output buffer (keep all V entries) --------------------
(* keep = "true" *)
reg signed [K-1:0] BUF [0:V-1];

// ---- Control registers -------------------------------------
reg buf_valid;
reg [BCW-1:0] b_cnt;
reg [DCW-1:0] d_cnt;
reg [UCW-1:0] u_cnt;
reg [BUFCW-1:0] buf_cnt;

// ============================================================
// Combinational index + ternary multiply
//
// For round j (u_cnt), cycle i (d_cnt), unit g:
// raw = j*V + g - i (12-bit signed)
// neg = (raw < 0) negacyclic wrap → negate coefficient
// addr = raw mod N = raw[9:0] (exact: |raw| < N)
// prod = ±d_in or 0 based on B[addr] and neg
//
// u_cnt << LOG2V = u_cnt * V as a pure wire shift (no DSP)
// ============================================================
wire [10:0] u_times_V = {{(11-UCW){1'b0}}, u_cnt} << LOG2V;

// prod MUST be K=32 bits - carries full D value [bug was [11:0] here]
wire signed [K-1:0] prod [0:V-1];

genvar g;
generate
for (g = 0; g < V; g = g+1) begin : MAC_LANES
// raw = j*V + g - i (12-bit signed arithmetic)
wire signed [11:0] raw = $signed({1'b0, u_times_V})
+ $signed(12'd0 + g)
- $signed({2'b0, d_cnt});
wire neg = raw[11]; // sign bit = (raw < 0)
wire [9:0] addr = raw[9:0]; // mod 1024 (2's complement)

wire signed [1:0] b_raw = B[addr];
wire signed [1:0] bc = neg ? (-b_raw) : b_raw;

// Ternary multiply: LUT-based, no DSP
assign prod[g] = (bc == 2'sb01) ? d_in :
(bc == 2'sb11) ? -d_in :
{K{1'b0}};
end
endgenerate

// ---- Output ------------------------------------------------
assign w_valid = buf_valid;
assign w_out = BUF[buf_cnt];

// ============================================================
// FSM + Datapath
// ============================================================
integer i;
always @(posedge clk or negedge rst_n) begin
if (!rst_n) begin
state <= S_IDLE;
b_cnt <= {BCW{1'b0}};
d_cnt <= {DCW{1'b0}};
u_cnt <= {UCW{1'b0}};
buf_cnt <= {BUFCW{1'b0}};
buf_valid <= 1'b0;
done <= 1'b0;
for (i = 0; i < V; i = i+1) begin
AC[i] <= {AC_WIDTH{1'b0}};
BUF[i] <= {K{1'b0}};
end
end else begin

// ---- Output drain (overlaps with S_COMP) -----------
if (buf_valid) begin
if (buf_cnt == V[BUFCW:0] - 1) begin
buf_valid <= 1'b0;
buf_cnt <= {BUFCW{1'b0}};
end else begin
buf_cnt <= buf_cnt + 1'b1;
end
end

// ---- FSM -------------------------------------------
case (state)

S_IDLE: begin
done <= 1'b0;
if (start) state <= S_LOAD;
end

// Load N B-coefficients into LUT-RAM
S_LOAD: begin
if (b_valid) begin
B[b_cnt] <= b_in;
if (b_cnt == N[BCW:0] - 1) begin
b_cnt <= {BCW{1'b0}};
state <= S_COMP;
end else begin
b_cnt <= b_cnt + 1'b1;
end
end
end

// U rounds x N cycles (Algorithm 1, lines 3-8)
S_COMP: begin
if (d_valid) begin

// V parallel accumulations (Algorithm 1, line 5)
for (i = 0; i < V; i = i+1)
AC[i] <= AC[i]
+ {{(AC_WIDTH-K){prod[i][K-1]}}, prod[i]};

if (d_cnt == N[DCW:0] - 1) begin
// Round j complete: latch to BUF (Algorithm 1, line 7)
// AC holds cycles 0..N-2; add last prod here
for (i = 0; i < V; i = i+1)
BUF[i] <= AC[i][K-1:0] + prod[i];
// Clear ACs for next round
for (i = 0; i < V; i = i+1)
AC[i] <= {AC_WIDTH{1'b0}};

buf_valid <= 1'b1;
buf_cnt <= {BUFCW{1'b0}};
d_cnt <= {DCW{1'b0}};

if (u_cnt == U[UCW:0] - 1) begin
u_cnt <= {UCW{1'b0}};
state <= S_OUTPUT;
end else begin
u_cnt <= u_cnt + 1'b1;
end
end else begin
d_cnt <= d_cnt + 1'b1;
end
end
end

// Wait for last BUF to drain (Algorithm 1, line 9)
S_OUTPUT: begin
if (!buf_valid) state <= S_DONE;
end

S_DONE: begin
done <= 1'b1;
end

default: state <= S_IDLE;
endcase
end
end

endmodule