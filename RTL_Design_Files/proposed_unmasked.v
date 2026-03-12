`timescale 1ns/1ps
// =============================================================================
// LTE_PSC_IN_SMA  -  Hybrid: PSC-inside-SMA Polynomial Multiplier
// Compatible with V = 16/32/64/128 (and any V that divides N_HALF)
// Fix: dump_idx width scales with V (prevents watchdog timeout).
// =============================================================================

module proposed_unmasked #(
    parameter integer N      = 1024,
    parameter integer BW     = 2,
    parameter integer DW     = 32,
    parameter integer WW     = 32,
    parameter integer N_HALF = N/2,
    parameter integer V      = 64,
    parameter integer U_SEG  = N_HALF/V
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  load_en,
    input  wire signed [BW-1:0]  b_in,
    input  wire signed [DW-1:0]  d_in,
    input  wire                  start,
    output reg  signed [WW-1:0]  w_out,
    output reg                   w_valid,
    output reg                   done
);

    // ------------------------------------------------------------------------
    // Compile-time width helper (NO procedural loops)
    // Supports up to V=512; extend if needed.
    // ------------------------------------------------------------------------
    localparam integer DUMP_W =
        (V <= 2)   ? 1 :
        (V <= 4)   ? 2 :
        (V <= 8)   ? 3 :
        (V <= 16)  ? 4 :
        (V <= 32)  ? 5 :
        (V <= 64)  ? 6 :
        (V <= 128) ? 7 :
        (V <= 256) ? 8 : 9;

    // ------------------------------------------------------------------------
    // FSM States
    // ------------------------------------------------------------------------
    localparam S_IDLE     = 3'd0;
    localparam S_PIPE     = 3'd1;
    localparam S_CALC     = 3'd2;
    localparam S_FLUSH    = 3'd3;
    localparam S_DUMP_TOP = 3'd4;
    localparam S_DUMP_BOT = 3'd5;
    localparam S_DONE     = 3'd6;

    reg [2:0] state;

    // ------------------------------------------------------------------------
    // Coefficient Storage (Dual Distributed RAM)
    // ------------------------------------------------------------------------
    (* ram_style="distributed" *) reg signed [BW-1:0] B0_arr [0:N_HALF-1];
    (* ram_style="distributed" *) reg signed [BW-1:0] B1_arr [0:N_HALF-1];
    (* ram_style="distributed" *) reg signed [DW-1:0] D0_arr [0:N_HALF-1];
    (* ram_style="distributed" *) reg signed [DW-1:0] D1_arr [0:N_HALF-1];

    reg [10:0] load_ptr;
    wire       load_done = (load_ptr == N);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) load_ptr <= 11'd0;
        else if (state == S_DONE) load_ptr <= 11'd0;
        else if (load_en && !load_done) load_ptr <= load_ptr + 11'd1;
    end

    always @(posedge clk) begin
        if (load_en && !load_done) begin
            if (load_ptr < N_HALF) begin
                B0_arr[load_ptr[8:0]] <= b_in;
                D0_arr[load_ptr[8:0]] <= d_in;
            end else begin
                B1_arr[load_ptr[8:0]] <= b_in;
                D1_arr[load_ptr[8:0]] <= d_in;
            end
        end
    end

    // ------------------------------------------------------------------------
    // Control & Flattened Pipeline Registers
    // ------------------------------------------------------------------------
    reg [8:0]         k;
    reg [8:0]         seg_idx;
    reg [DUMP_W-1:0]  dump_idx;     // FIX: scalable width
    reg [1:0]         pipe_cnt;
    reg [1:0]         flush_cnt;

    reg mac_v0, mac_v1, mac_v2, mac_v3;

    reg signed [WW-1:0] d0_p0, d1_p0, d0_p1, d1_p1, d0_p2, d1_p2;
    reg [8:0]           k_p0;

    reg [(V*9)-1:0]   row_p0_flat;
    reg [(V*9)-1:0]   b_idx_p1_flat;
    reg [V-1:0]       wrap_p1_flat;
    reg [(V*BW)-1:0]  b0_p2_flat;
    reg [(V*BW)-1:0]  b1_p2_flat;
    reg [V-1:0]       wrap_p2_flat;

    reg signed [(V*WW)-1:0] acc_top_flat;
    reg signed [(V*WW)-1:0] acc_bot_flat;

    wire [(V*9)-1:0]   next_row_p0_flat;
    wire [(V*9)-1:0]   next_b_idx_p1_flat;
    wire [V-1:0]       next_wrap_p1_flat;
    wire [(V*BW)-1:0]  next_b0_p2_flat;
    wire [(V*BW)-1:0]  next_b1_p2_flat;
    wire [V-1:0]       next_wrap_p2_flat;
    wire signed [(V*WW)-1:0] next_acc_top_flat;
    wire signed [(V*WW)-1:0] next_acc_bot_flat;

    // ------------------------------------------------------------------------
    // Structural Datapath Generation
    // ------------------------------------------------------------------------
    genvar g;
    generate
        for(g=0; g<V; g=g+1) begin : MAC_LANES
            // Stage 0: row = seg_idx*V + g  (keep only 9 bits; N_HALF=512)
            wire [15:0] row_calc = (seg_idx * V) + g;
            assign next_row_p0_flat[g*9 +: 9] = row_calc[8:0];

            // Stage 1
            wire [8:0] curr_row = row_p0_flat[g*9 +: 9];
            assign next_wrap_p1_flat[g] = (curr_row < k_p0);
            assign next_b_idx_p1_flat[g*9 +: 9] = curr_row - k_p0;

            // Stage 2
            wire [8:0] curr_idx = b_idx_p1_flat[g*9 +: 9];
            assign next_b0_p2_flat[g*BW +: BW] = B0_arr[curr_idx];
            assign next_b1_p2_flat[g*BW +: BW] = B1_arr[curr_idx];
            assign next_wrap_p2_flat[g]        = wrap_p1_flat[g];

            // Stage 3: Ternary Math & Accumulation with your cross-coupling
            wire signed [BW-1:0] b0_raw = b0_p2_flat[g*BW +: BW];
            wire signed [BW-1:0] b1_raw = b1_p2_flat[g*BW +: BW];
            wire w = wrap_p2_flat[g];

            wire signed [BW-1:0] val_A     = w ? -b1_raw :  b0_raw;
            wire signed [BW-1:0] val_Bm    = w ? -b0_raw : -b1_raw;
            wire signed [BW-1:0] val_negBm = w ?  b0_raw :  b1_raw;

            wire signed [WW-1:0] t_td0 = (val_A     == 2'sb01) ? d0_p2 : ((val_A     == 2'sb11) ? -d0_p2 : {WW{1'b0}});
            wire signed [WW-1:0] t_td1 = (val_Bm    == 2'sb01) ? d1_p2 : ((val_Bm    == 2'sb11) ? -d1_p2 : {WW{1'b0}});
            wire signed [WW-1:0] t_bd0 = (val_negBm == 2'sb01) ? d0_p2 : ((val_negBm == 2'sb11) ? -d0_p2 : {WW{1'b0}});
            wire signed [WW-1:0] t_bd1 = (val_A     == 2'sb01) ? d1_p2 : ((val_A     == 2'sb11) ? -d1_p2 : {WW{1'b0}});

            wire signed [WW-1:0] curr_acc_top = acc_top_flat[g*WW +: WW];
            wire signed [WW-1:0] curr_acc_bot = acc_bot_flat[g*WW +: WW];

            assign next_acc_top_flat[g*WW +: WW] = curr_acc_top + t_td0 + t_td1;
            assign next_acc_bot_flat[g*WW +: WW] = curr_acc_bot + t_bd0 + t_bd1;
        end
    endgenerate

    // ------------------------------------------------------------------------
    // Synchronous Pipeline Logic
    // ------------------------------------------------------------------------
    wire clr_acc = (state == S_IDLE && start && load_done) ||
                   (state == S_DUMP_BOT && (dump_idx == (V-1)));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mac_v0 <= 1'b0; mac_v1 <= 1'b0; mac_v2 <= 1'b0; mac_v3 <= 1'b0;
            d0_p0 <= {WW{1'b0}}; d1_p0 <= {WW{1'b0}}; k_p0 <= 9'd0;
            d0_p1 <= {WW{1'b0}}; d1_p1 <= {WW{1'b0}};
            d0_p2 <= {WW{1'b0}}; d1_p2 <= {WW{1'b0}};

            row_p0_flat   <= {(V*9){1'b0}};
            b_idx_p1_flat <= {(V*9){1'b0}};
            wrap_p1_flat  <= {V{1'b0}};
            b0_p2_flat    <= {(V*BW){1'b0}};
            b1_p2_flat    <= {(V*BW){1'b0}};
            wrap_p2_flat  <= {V{1'b0}};
            acc_top_flat  <= {(V*WW){1'b0}};
            acc_bot_flat  <= {(V*WW){1'b0}};
        end else begin
            mac_v0 <= (state == S_CALC);
            mac_v1 <= mac_v0;
            mac_v2 <= mac_v1;
            mac_v3 <= mac_v2;

            d0_p0 <= {{(WW-DW){D0_arr[k][DW-1]}}, D0_arr[k]};
            d1_p0 <= {{(WW-DW){D1_arr[k][DW-1]}}, D1_arr[k]};
            k_p0  <= k;
            row_p0_flat <= next_row_p0_flat;

            d0_p1 <= d0_p0;
            d1_p1 <= d1_p0;
            wrap_p1_flat  <= next_wrap_p1_flat;
            b_idx_p1_flat <= next_b_idx_p1_flat;

            d0_p2 <= d0_p1;
            d1_p2 <= d1_p1;
            b0_p2_flat   <= next_b0_p2_flat;
            b1_p2_flat   <= next_b1_p2_flat;
            wrap_p2_flat <= next_wrap_p2_flat;

            if (clr_acc) begin
                acc_top_flat <= {(V*WW){1'b0}};
                acc_bot_flat <= {(V*WW){1'b0}};
            end else if (mac_v3) begin
                acc_top_flat <= next_acc_top_flat;
                acc_bot_flat <= next_acc_bot_flat;
            end
        end
    end

    // ------------------------------------------------------------------------
    // FSM
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            k         <= 9'd0;
            seg_idx   <= 9'd0;
            dump_idx  <= {DUMP_W{1'b0}};
            pipe_cnt  <= 2'd0;
            flush_cnt <= 2'd0;
            done      <= 1'b0;
            w_valid   <= 1'b0;
            w_out     <= 32'sd0;
        end else begin
            w_valid <= 1'b0;
            done    <= 1'b0;

            case (state)
            S_IDLE: begin
                if (start && load_done) begin
                    k        <= 9'd0;
                    seg_idx  <= 9'd0;
                    dump_idx <= {DUMP_W{1'b0}};
                    pipe_cnt <= 2'd0;
                    state    <= S_PIPE;
                end
            end

            S_PIPE: begin
                if (pipe_cnt == 2'd2) state <= S_CALC;
                else pipe_cnt <= pipe_cnt + 2'd1;
            end

            S_CALC: begin
                if (k == (N_HALF - 1)) begin
                    k         <= 9'd0;
                    flush_cnt <= 2'd0;
                    state     <= S_FLUSH;
                end else begin
                    k <= k + 9'd1;
                end
            end

            S_FLUSH: begin
                if (flush_cnt == 2'd3) begin
                    dump_idx <= {DUMP_W{1'b0}};
                    state    <= S_DUMP_TOP;
                end else begin
                    flush_cnt <= flush_cnt + 2'd1;
                end
            end

            S_DUMP_TOP: begin
                w_out   <= acc_top_flat[dump_idx * WW +: WW];
                w_valid <= 1'b1;
                if (dump_idx == (V-1)) begin
                    dump_idx <= {DUMP_W{1'b0}};
                    state    <= S_DUMP_BOT;
                end else begin
                    dump_idx <= dump_idx + {{(DUMP_W-1){1'b0}},1'b1};
                end
            end

            S_DUMP_BOT: begin
                w_out   <= acc_bot_flat[dump_idx * WW +: WW];
                w_valid <= 1'b1;

                if (dump_idx == (V-1)) begin
                    dump_idx <= {DUMP_W{1'b0}};
                    if (seg_idx == (U_SEG - 1)) begin
                        state <= S_DONE;
                    end else begin
                        seg_idx  <= seg_idx + 9'd1;
                        k        <= 9'd0;
                        pipe_cnt <= 2'd0;
                        state    <= S_PIPE;
                    end
                end else begin
                    dump_idx <= dump_idx + {{(DUMP_W-1){1'b0}},1'b1};
                end
            end

            S_DONE: begin
                done  <= 1'b1;
                state <= S_IDLE;
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule