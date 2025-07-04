`timescale 1ns/1ns
`define VERBOSE
`include "femtosoc.v"

/*──────────────────────────────────────────────
 * Testbench de FemtoRV32
 *  – Con VERILATOR: pclk lo suministra sim_main.cpp;
 *    result y rbusy son puertos visibles para C++.
 *  – Sin VERILATOR (Icarus): generamos reloj aquí.
 *─────────────────────────────────────────────*/
`ifdef VERILATOR
module femtoRV32_bench (
    input  wire        pclk,
    output wire [63:0] result,
    output wire        rbusy
);
`else
module femtoRV32_bench;
    reg  pclk = 0;       // reloj local
    reg  clk = 0;        // reloj local para simulaciones sin Verilator
    wire [63:0] result;  // mismos nombres, pero wires internos
    wire        rbusy;
`endif

    /* Señales del multiplicador dentro del SoC */
    wire [31:0] A, B;
    wire        wstrb;
    wire [1:0]  sel;

    /* Instancia del SoC (unidad bajo prueba) */
    femtosoc uut (
        .pclk  (pclk),
        .clk   (clk),
        .RESET (1'b0),

        .A     (A),
        .B     (B),
        .result(result),   // usamos los mismos wires/puertos
        .wstrb (wstrb),
        .sel   (sel),
        .rbusy (rbusy)
    );

`ifndef VERILATOR
    /* Reloj de 500 MHz simulado (periodo 2 ns) */
    always #1 pclk = ~pclk;
    always #1 clk = ~clk;  // Generación del reloj clk para simulación
`endif


/*──────────────────────────────────────────────
 * Bloque de diagnóstico y volcado VCD
 *   – Activo cuando se compila sin -DBENCH_VERILATOR
 *─────────────────────────────────────────────*/
`ifndef BENCH_VERILATOR
    initial begin
        $dumpfile("wave.vcd");
        $dumpvars(0, femtoRV32_bench);

        /*  stop-sim tras 2 ms de tiempo simulado (≈ 1 000 000 ciclos)  */
        #2_000_000 begin
            $display("TIME-OUT: rbusy nunca se activó");
            $finish;
        end
    end
`endif

endmodule
