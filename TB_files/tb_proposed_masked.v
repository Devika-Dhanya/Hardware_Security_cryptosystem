`timescale 1ns/1ps

module tb_proposed_masked;

parameter integer N             = 1024;
parameter integer BW            = 2;
parameter integer DW            = 32;
parameter integer WW            = 32;
parameter integer N_HALF        = N/2;

parameter integer V             = 16;  // 16/32/64/128
parameter integer U_SEG         = N_HALF / V;
parameter integer CLK_PERIOD_NS = 10;

localparam integer LANE_W =
    (V <= 2)   ? 1 :
    (V <= 4)   ? 2 :
    (V <= 8)   ? 3 :
    (V <= 16)  ? 4 :
    (V <= 32)  ? 5 :
    (V <= 64)  ? 6 :
    (V <= 128) ? 7 :
    (V <= 256) ? 8 : 9;

reg                  clk, rst_n, load_en, start;
reg  signed [BW-1:0] b_in;
reg  signed [DW-1:0] d_in;
wire signed [WW-1:0] w_out;
wire                 w_valid, done;

proposed_masked #(
    .N(N), .BW(BW), .DW(DW), .WW(WW),
    .N_HALF(N_HALF), .V(V), .U_SEG(U_SEG),
    .USE_REFRESH(1)
) dut (
    .clk(clk), .rst_n(rst_n), .load_en(load_en),
    .b_in(b_in), .d_in(d_in), .start(start),
    .w_out(w_out), .w_valid(w_valid), .done(done)
);

initial clk = 1'b0;
always #(CLK_PERIOD_NS/2) clk = ~clk;

integer cycle_counter;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cycle_counter = 0;
    else        cycle_counter = cycle_counter + 1;
end

integer gi, gj, gk;

reg signed [BW-1:0] B    [0:N-1];
reg signed [DW-1:0] D    [0:N-1];
reg signed [63:0]   GOLDW[0:N-1];
reg signed [WW-1:0] GOLD [0:N-1];

task compute_golden;
    begin
        for (gi = 0; gi < N; gi = gi + 1) GOLDW[gi] = 64'sd0;

        for (gi = 0; gi < N; gi = gi + 1) begin
            for (gj = 0; gj < N; gj = gj + 1) begin
                gk = gi + gj;
                if (gk < N) GOLDW[gk]   = GOLDW[gk]   + $signed(B[gi]) * $signed(D[gj]);
                else        GOLDW[gk-N] = GOLDW[gk-N] - $signed(B[gi]) * $signed(D[gj]);
            end
        end

        for (gi = 0; gi < N; gi = gi + 1)
            GOLD[gi] = GOLDW[gi][WW-1:0];
    end
endtask

integer errors, out_count, first_seen, run_id;
integer load_start_cycle, load_end_cycle, load_latency;
integer comp_start_cycle, comp_end_cycle, comp_latency;
integer first_out_cycle, last_out_cycle, first_out_latency, output_duration, total_latency;

reg [8:0]           seg_out;
reg                 half_out;     // 0->TOP, 1->BOT
reg [LANE_W-1:0]     lane_out;

reg [10:0]           widx;
reg signed [WW-1:0]  exp_val;

always @(posedge clk) begin
    if (w_valid) begin
        if (!first_seen) begin
            first_seen = 1;
            first_out_cycle = cycle_counter;
        end
        last_out_cycle = cycle_counter;

        widx = half_out ? (N_HALF + seg_out*V + lane_out) : (seg_out*V + lane_out);
        exp_val = GOLD[widx];

        if ($signed(w_out) !== $signed(exp_val)) begin
            $display("[ERROR] run=%0d seg=%0d half=%0d lane=%0d idx=%0d exp=%0d got=%0d",
                     run_id, seg_out, half_out, lane_out, widx, $signed(exp_val), $signed(w_out));
            errors = errors + 1;
        end

        out_count = out_count + 1;

        if (lane_out == (V-1)) begin
            lane_out = {LANE_W{1'b0}};
            if (half_out) begin
                half_out = 1'b0;
                seg_out  = seg_out + 9'd1;
            end else begin
                half_out = 1'b1;
            end
        end else begin
            lane_out = lane_out + {{(LANE_W-1){1'b0}},1'b1};
        end
    end
end

task print_latency_report;
    begin
        load_latency      = load_end_cycle - load_start_cycle;
        comp_latency      = comp_end_cycle - comp_start_cycle;
        first_out_latency = first_out_cycle - comp_start_cycle;
        output_duration   = last_out_cycle - first_out_cycle + 1;
        total_latency     = load_latency + comp_latency;

        $display("\n================================================================");
        $display("  MASKED-D DESIGN LATENCY REPORT  ---  RUN %0d  V=%0d", run_id, V);
        $display("================================================================");
        $display("  LOAD PHASE           : %0d cycles", load_latency);
        $display("  COMPUTE (start->done) : %0d cycles", comp_latency);
        $display("  First output latency : %0d cycles", first_out_latency);
        $display("  Output duration      : %0d cycles", output_duration);
        $display("  Total (load+compute) : %0d cycles", total_latency);
        $display("  Outputs received     : %0d / %0d", out_count, N);
        $display("  Errors               : %0d", errors);
        $display("================================================================\n");
    end
endtask

task run_test;
    input integer seed;
    integer wdog_cycles;
    begin
        errors = 0; out_count = 0; first_seen = 0;
        seg_out = 0; half_out = 1'b0; lane_out = {LANE_W{1'b0}};

        // deterministic but non-trivial vectors
        for (gi = 0; gi < N; gi = gi + 1) begin
            case ((gi + seed) % 3)
                0:       B[gi] =  1;
                1:       B[gi] = -1;
                default: B[gi] =  0;
            endcase
            D[gi] = $signed(gi) + $signed(seed);
        end

        compute_golden;

        // Load N samples at negedge
        @(negedge clk);
        load_en = 1'b1;
        load_start_cycle = cycle_counter;

        for (gi = 0; gi < N; gi = gi + 1) begin
            b_in = B[gi];
            d_in = D[gi];
            @(negedge clk);
        end

        load_en = 1'b0;
        b_in = 0;
        d_in = 0;
        load_end_cycle = cycle_counter;

        // Start pulse
        @(negedge clk);
        start = 1'b1;
        comp_start_cycle = cycle_counter;
        @(negedge clk);
        start = 1'b0;

        // Watchdog
        wdog_cycles = 2000 + 40 * (N_HALF/V) * N_HALF;

        fork
            begin : wait_done
                @(posedge done);
                comp_end_cycle = cycle_counter;
                disable watchdog;
            end
            begin : watchdog
                repeat(wdog_cycles) @(posedge clk);
                $display("[FATAL] Watchdog timeout! V=%0d waited %0d cycles", V, wdog_cycles);
                $finish;
            end
        join

        repeat(10) @(posedge clk);
        print_latency_report;

        if (out_count !== N) begin
            $display("[ERROR] Output count mismatch: got %0d expected %0d", out_count, N);
            $finish;
        end

        if (errors == 0) $display("[PASS] run=%0d seed=%0d", run_id, seed);
        else             $display("[FAIL] run=%0d seed=%0d errors=%0d", run_id, seed, errors);

        run_id = run_id + 1;
        repeat(5) @(posedge clk);
    end
endtask

initial begin
    $dumpfile("tb_LTE_PSC_IN_SMA_MASKED.vcd");
    $dumpvars(0, tb_LTE_PSC_IN_SMA_MASKED);

    run_id = 1;

    rst_n   = 0;
    load_en = 0;
    start   = 0;
    b_in    = 0;
    d_in    = 0;

    repeat(10) @(posedge clk);
    rst_n = 1;
    @(posedge clk);

    run_test(0);
    run_test(7);

    $finish;
end

endmodule
