#include "VfemtoRV32_bench.h"
#include "verilated.h"

#include <cstdio>
#include <cstdint>
#include <inttypes.h>
#include <fenv.h>
#include <xmmintrin.h>

int main(int argc, char** argv, char** env) {
    // Configuración de coma flotante
    fesetround(FE_TOWARDZERO);
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);

    Verilated::commandArgs(argc, argv);

    VfemtoRV32_bench top;      // Instancia del testbench
    uint64_t main_time = 0;    // Contador de ciclos
    const uint64_t TIMEOUT = 1000000; // 1 000 000 ciclos de timeout

    bool shown     = false;    // Para imprimir una sola vez el resultado
    bool prev_busy = false;    // Para detectar flanco de bajada en rbusy

    // Inicializa el reloj pclk (el SoC genera 'clk' internamente vía PLL)
    top.pclk = 0;

    while (!Verilated::gotFinish() && main_time < TIMEOUT) {
        // Toggle de reloj
        top.pclk = !top.pclk;
        top.eval();
        ++main_time;

        // Detectar fin de multiplicación (flanco 1→0 en rbusy)
        if (!shown && prev_busy && !top.rbusy) {
            uint64_t prod = top.result;
            std::printf("Multiplicador: 7 x 9 = %" PRIu64 "\n", prod);
            shown = true;
        }
        prev_busy = top.rbusy;
    }

    if (!shown) {
        std::fprintf(stderr, "ERROR: rbusy nunca se desactivó (timeout tras %" PRIu64 " ciclos)\n", main_time);
        return 1;
    }

    return 0;
}
