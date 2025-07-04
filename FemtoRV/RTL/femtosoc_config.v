// Configuration file for femtosoc/femtorv32

`ifdef BENCH_VERILATOR
`define BENCH
`endif
// Definición de macros necesarias para la compilación
`define NRV_ARCH "rv32i"       // Definir la arquitectura (por ejemplo, RV32I o RV64I)
`define NRV_OPTIMIZE "-Os"     // Nivel de optimización, por ejemplo O2
`define NRV_ABI "ilp32"       // ABI, como ilp32 o lp64


// Comentamos las definiciones específicas de placas, ya que no las estamos usando
//`ifdef ULX3S
//`include "CONFIGS/ulx3s_config.v"
//`endif

//`ifdef ICE_STICK
//`include "CONFIGS/icestick_config.v"
//`endif

//`ifdef ICE_BREAKER
//`include "CONFIGS/icebreaker_config.v"
//`endif

//`ifdef ECP5_EVN
//`include "CONFIGS/ecp5evn_config.v"
//`endif

//`ifdef ARTY
//`include "CONFIGS/arty_config.v"
//`endif

//`ifdef ICE_SUGAR_NANO
//`include "CONFIGS/icesugarnano_config.v"
//`endif

//`ifdef CMODA7
//`include "CONFIGS/cmod_a7_config.v"
//`endif

// Usamos la configuración genérica si no se define ninguna de las anteriores
`ifndef NRV_CONFIGURED
`include "CONFIGS/generic_config.v"
`endif

/******************************************************************************/
// Configuración para el reset

// Deshabilitamos la configuración de reset para placas específicas
// Si no tienes un botón de reset activo, no defines NRV_NEGATIVE_RESET
//`ifdef ICE_STICK
//`define NRV_NEGATIVE_RESET 
//`endif

//`ifdef FOMU
//`define NRV_NEGATIVE_RESET
//`endif

// Definimos los módulos que vamos a usar. Aquí indicamos los módulos que pueden ser necesarios
`ifdef NRV_IO_SPI_FLASH
`define NRV_SPI_FLASH
`endif

`ifdef NRV_MAPPED_SPI_FLASH
`define NRV_SPI_FLASH
`endif

// Habilitamos el multiplicador como dispositivo

`define NRV_IO_MULTIPLIER

// En caso de que se use alguna de las placas ECP5, ULX3S o similares
`ifdef ECP5_EVN
`define NRV_NEGATIVE_RESET
`endif

/******************************************************************************************************************/
// Definición de procesadores

`define NRV_IS_IO_ADDR(addr) |addr[23:22] // Asserted if address is in IO space (then it needs additional wait states)

`include "PROCESSOR/utils.v"

// Definimos el procesador en función de la configuración
`ifdef NRV_FEMTORV32_QUARK
 `include "PROCESSOR/femtorv32_quark.v" // Minimalistic version of the processor for IceStick (RV32I)
`endif

`ifdef NRV_FEMTORV32_QUARK_BICYCLE
 `include "PROCESSOR/femtorv32_quark_bicycle.v" // Quark with Matthias's 2 CPI mode and barrel shifter (RV32I)
`endif

`ifdef NRV_FEMTORV32_TACHYON
 `include "PROCESSOR/femtorv32_tachyon.v" // Version for the IceStick with higher maxfreq (RV32I)
`endif

`ifdef NRV_FEMTORV32_ELECTRON
 `include "PROCESSOR/femtorv32_electron.v" // RV32IM with barrel shifter
`endif

`ifdef NRV_FEMTORV32_INTERMISSUM
 `include "PROCESSOR/femtorv32_intermissum.v" // RV32IM with barrel shifter and interrupts
`endif

`ifdef NRV_FEMTORV32_GRACILIS
 `include "PROCESSOR/femtorv32_gracilis.v" // RV32IMC with barrel shifter and interrupts
`endif

`ifdef NRV_FEMTORV32_PETITBATEAU
 `include "PROCESSOR/femtorv32_petitbateau.v" // under development, RV32IMFC
`endif

`ifdef NRV_FEMTORV32_TESTDRIVE
 `include "PROCESSOR/femtorv32_testdrive.v" // CPU under test
`endif

/******************************************************************************************************************/
