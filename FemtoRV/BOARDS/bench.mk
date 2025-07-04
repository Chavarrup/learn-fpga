################################################################################
#   testbench pseudo-board
################################################################################
.PHONY: BENCH BENCH.firmware_config BENCH.icarus BENCH.verilator BENCH.lint

BENCH: BENCH.verilator

BENCH.firmware_config:
	BOARD=testbench TOOLS/make_config.sh -DBENCH_VERILATOR
	(cd FIRMWARE; make libs)

BENCH.icarus:
	(cd RTL; iverilog -IPROCESSOR -IDEVICES femtosoc_bench.v \
         -o ../femtosoc_bench.vvp)
	vvp femtosoc_bench.vvp

BENCH.verilator:
	verilator -DBENCH_VERILATOR --top-module femtoRV32_bench \
         -IRTL -IRTL/PROCESSOR -IRTL/DEVICES -IRTL/PLL \
         -CFLAGS '-I../SIM' \
         -FI FPU_funcs.h \
         --cc --exe SIM/sim_main.cpp SIM/FPU_funcs.cpp \
         RTL/femtosoc_bench.v
	(cd obj_dir; make -f VfemtoRV32_bench.mk)
	obj_dir/VfemtoRV32_bench

BENCH.lint:
	verilator -DBENCH --lint-only --top-module femtoRV32_bench \
         -IRTL -IRTL/PROCESSOR -IRTL/DEVICES -IRTL/PLL RTL/femtosoc_bench.v
################################################################################
