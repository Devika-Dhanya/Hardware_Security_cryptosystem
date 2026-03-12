// ============================================================
// tb_psc_accelerator.v -- Self-Checking Testbench for PSC
// Verifies Negacyclic Convolution: W = B * D mod (x^N + 1)
// ============================================================
`timescale 1ns/1ps

module tb_psc_accelerator();

    // ---- Parameters ------------------------------------------
    localparam V = 64;           // Must match DUT parameter
    localparam N = 1024;         // Hardcoded in DUT
    localparam U = N / V;        // 16 rounds

    // ---- DUT Signals -----------------------------------------
    reg               clk;
    reg               rst_n;
    reg               start;
    reg signed [1:0]  b_in;
    reg               b_valid;
    reg signed [31:0] d_in;
    reg               d_valid;
    
    wire signed [31:0] w_out;
    wire               w_valid;
    wire               done;

    // ---- Golden Model Memory ---------------------------------
    reg signed [1:0]  B_mem [0:N-1];
    reg signed [31:0] D_mem [0:N-1];
    reg signed [31:0] W_exp [0:N-1];

    // ---- Instantiate DUT -------------------------------------
    psc_accelerator #(
        .V(V)
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .b_in(b_in),
        .b_valid(b_valid),
        .d_in(d_in),
        .d_valid(d_valid),
        .w_out(w_out),
        .w_valid(w_valid),
        .done(done)
    );

    // ---- Clock Generation ------------------------------------
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100 MHz clock
    end

    // ---- Test Variables --------------------------------------
    integer i, j, r;
    integer err_cnt = 0;
    integer out_idx = 0;
    integer rand_val;
    reg signed [63:0] prod; // Extended precision to prevent intermediate overflow

    // ---- Stimulus & Golden Model Generation ------------------
    initial begin
        // 1. Generate Random Inputs and Initialize Arrays
        for (i = 0; i < N; i = i + 1) begin
            // Generate ternary B values (-1, 0, 1)
            rand_val = {$random} % 3;
            if (rand_val == 0)      B_mem[i] = 2'sd0;
            else if (rand_val == 1) B_mem[i] = 2'sd1;
            else                    B_mem[i] = -2'sd1; // Represents 2'sb11

            // Limit D to 16-bit randoms to prevent overflow during accumulation
            D_mem[i] = $signed($random) >>> 16; 
            W_exp[i] = 0;
        end

        // 2. Compute Golden Negacyclic Convolution W = B * D mod (x^N + 1)
        for (i = 0; i < N; i = i + 1) begin
            for (j = 0; j < N; j = j + 1) begin
                if (j <= i) begin
                    prod = B_mem[j] * D_mem[i-j];
                    W_exp[i] = W_exp[i] + prod[31:0];
                end else begin
                    prod = B_mem[j] * D_mem[N+i-j];
                    W_exp[i] = W_exp[i] - prod[31:0]; // Wrap around boundary incurs negative sign
                end
            end
        end

        // 3. Reset Sequence
        rst_n = 0; start = 0;
        b_valid = 0; d_valid = 0;
        b_in = 0; d_in = 0;
        
        #100;
        @(negedge clk) rst_n = 1;
        #20;

        // 4. Send Start Pulse
        @(negedge clk) start = 1;
        @(negedge clk) start = 0;

        // 5. Stream N B-coefficients into LUT-RAM
        $display("[%0t] Loading %0d B coefficients...", $time, N);
        for (i = 0; i < N; i = i + 1) begin
            @(negedge clk);
            b_valid = 1;
            b_in = B_mem[i];
        end
        @(negedge clk) b_valid = 0;

        // 6. Stream N D-coefficients U times for parallel MACs
        $display("[%0t] Streaming D coefficients for %0d rounds...", $time, U);
        for (r = 0; r < U; r = r + 1) begin
            for (i = 0; i < N; i = i + 1) begin
                @(negedge clk);
                d_valid = 1;
                d_in = D_mem[i];
            end
        end
        @(negedge clk) d_valid = 0;

        // 7. Wait for processing to complete
        wait(done);
        #100;

        // 8. Final Report
        if (out_idx !== N) begin
            $display("ERROR: Expected %0d outputs, but got %0d", N, out_idx);
            err_cnt = err_cnt + 1;
        end

        $display("\n==================================================");
        if (err_cnt == 0)
            $display(">> SUCCESS! All %0d outputs matched golden model.", N);
        else
            $display(">> FAILED with %0d data mismatches.", err_cnt);
        $display("==================================================\n");

        $finish;
    end

    // ---- Self-Checking Output Monitor ------------------------
    always @(posedge clk) begin
        if (rst_n && w_valid) begin
            if (w_out !== W_exp[out_idx]) begin
                $display("[%0t] MISMATCH @ Index %0d: Expected %0d, Got %0d", 
                         $time, out_idx, W_exp[out_idx], w_out);
                err_cnt = err_cnt + 1;
            end
            out_idx = out_idx + 1;
        end
    end

    // ---- Safety Timeout --------------------------------------
    initial begin
        #5000000;
        $display("SIMULATION TIMEOUT! Deadlock detected.");
        $finish;
    end

    // ---- Optional: Generate Waveform File (VCD) --------------
    initial begin
        $dumpfile("psc_accelerator.vcd");
        $dumpvars(0, tb_psc_accelerator);
    end

endmodule