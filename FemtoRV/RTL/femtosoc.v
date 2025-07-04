// femtorv32, a minimalistic RISC-V RV32I core
//    (minus SYSTEM and FENCE that are not implemented)
//
//       Bruno Levy, May-June 2020
//
// This file: the "System on Chip" that goes with femtorv32.

/*************************************************************************************/

`include "femtosoc_config.v"        // User configuration of processor and SOC.
`include "PLL/femtopll.v"           // The PLL (generates clock at NRV_FREQ)
`include "DEVICES/HardwareConfig.v" // Constant registers to query hardware config.
`include "DEVICES/multiplier.v"   
`include "PROCESSOR/femtorv32_quark.v"  // Asegúrate de que este archivo esté correctamente incluido

`ifndef NRV_RESET_ADDR
 `define NRV_RESET_ADDR 0
`endif

`ifndef NRV_ADDR_WIDTH
 `define NRV_ADDR_WIDTH 24
`endif

module femtosoc(
// Señales de entrada
   input RESET,          // Reset del sistema
   input clk,            // Reloj del sistema
   input pclk, 
   // Señales de salida para el multiplicador
   output [31:0] A,      // Operando A para la multiplicación
   output [31:0] B,      // Operando B para la multiplicación
   output [63:0] result, // Resultado de la multiplicación (64 bits)
   output wstrb,         // Señal de escritura para el multiplicador
   output [1:0] sel,     // Selector de registros (A o B)
   output rbusy          // Señal de ocupado que indica que el multiplicador está trabajando
);
   
  femtoPLL #(
    .freq(`NRV_FREQ)	     
  ) pll(
    .pclk(pclk), 
    .clk(clk)
  );

`ifdef ICE_STICK
  reg [11:0] reset_cnt = 0;   
`else   
  reg [15:0] reset_cnt = 0;
`endif   
  wire       reset = &reset_cnt;

/* verilator lint_off WIDTH */   
`ifdef NRV_NEGATIVE_RESET
   always @(posedge clk,negedge RESET) begin
      if(!RESET) begin
	 reset_cnt <= 0;
      end else begin
	 reset_cnt <= reset_cnt + !reset;
      end
   end
`else
   always @(posedge clk,posedge RESET) begin
      if(RESET) begin
	 reset_cnt <= 0;
      end else begin
	 reset_cnt <= reset_cnt + !reset;
      end
   end
`endif
/* verilator lint_on WIDTH */   
   
/*
 * Memory and memory interface
 * memory map:
 *   address[21:2] RAM word address (4 Mb max).
 *   address[23:22]   00: RAM
 *                    01: IO page (1-hot)  (starts at 0x400000)
 *                    10: SPI Flash page   (starts at 0x800000)
 */ 

   // The memory bus.
   wire [31:0] mem_address; // 24 bits are used internally. The two LSBs are ignored (using word addresses)
   wire  [3:0] mem_wmask;   // mem write mask and strobe /write Legal values are 000,0001,0010,0100,1000,0011,1100,1111
   wire [31:0] mem_rdata;   // processor <- (mem and peripherals) 
   wire [31:0] mem_wdata;   // processor -> (mem and peripherals)
   wire        mem_rstrb;   // mem read strobe. Goes high to initiate memory write.
   wire        mem_rbusy;   // processor <- (mem and peripherals). Stays high until a read transfer is finished.
   wire        mem_wbusy;   // processor <- (mem and peripherals). Stays high until a write transfer is finished.

   wire        mem_wstrb = |mem_wmask; // mem write strobe, goes high to initiate memory write (deduced from wmask)

   // IO bus.
`ifdef NRV_MAPPED_SPI_FLASH
   wire mem_address_is_ram       = (mem_address[23:22] == 2'b00);   
   wire mem_address_is_io        = (mem_address[23:22] == 2'b01);
   wire mem_address_is_spi_flash = (mem_address[23:22] == 2'b10);
   wire mapped_spi_flash_rbusy;
   wire [31:0] mapped_spi_flash_rdata;
   
   MappedSPIFlash mapped_spi_flash(
      .clk(clk),
      .rstrb(mem_rstrb && mem_address_is_spi_flash),
      .word_address(mem_address[21:2]),
      .rdata(mapped_spi_flash_rdata),
      .rbusy(mapped_spi_flash_rbusy),
      .CLK(spi_clk),
      .CS_N(spi_cs_n),
`ifdef SPI_FLASH_FAST_READ_DUAL_IO				   
      .IO({spi_miso,spi_mosi})
`else	
      .MISO(spi_miso),
      .MOSI(spi_mosi)
`endif				   
   );
`else   
   wire mem_address_is_io  =  mem_address[22];
   wire mem_address_is_ram = !mem_address[22];
`endif
      
   reg  [31:0] io_rdata; 
   wire [31:0] io_wdata = mem_wdata;
   wire        io_rstrb = mem_rstrb && mem_address_is_io;
   wire        io_wstrb = mem_wstrb && mem_address_is_io;
   wire [19:0] io_word_address = mem_address[21:2]; // word offset in io page
   wire	      io_rbusy; 
   wire        io_wbusy;
   
   assign      mem_rbusy = io_rbusy

`ifdef NRV_MAPPED_SPI_FLASH
    | mapped_spi_flash_rbusy			   
`endif 			   
    ;
   
   assign      mem_wbusy = io_wbusy; 

   wire [19:0] ram_word_address = mem_address[21:2];

// Using the 128 KBytes of SPRAM (single-ported RAM) embedded in the Ice40 UP5K   
`ifdef ICE40UP5K_SPRAM

   wire [31:0]  ram_rdata;
   wire 	spram_wr = mem_address_is_ram && !mem_address_is_vram;
   ice40up5k_spram RAM(
      .clk(clk),
      .wen({4{spram_wr}} & mem_wmask),
      .addr(ram_word_address[14:0]),
      .wdata(mem_wdata),
      .rdata(ram_rdata)		       
   );

`else // Synthethizing BRAM

   (* no_rw_check *)
   reg [31:0] RAM[0:(`NRV_RAM/4)-1];
   reg [31:0] ram_rdata;

   // Initialize the RAM with the generated firmware hex file.
   // The hex file is generated by the bundled elf-2-verilog converter (see TOOLS/FIRMWARE_WORDS_SRC)
`ifndef NRV_RUN_FROM_SPI_FLASH  
   initial begin
      $readmemh("FIRMWARE/firmware.hex",RAM); 
      $display("RAM[0]=%08x  RAM[1]=%08x", RAM[0], RAM[1]);
   end
`endif

   // The power of YOSYS: it infers BRAM primitives automatically ! (and recognizes
   // masked writes, amazing ...)
   /* verilator lint_off WIDTH */
   always @(posedge clk) begin
      if(mem_address_is_ram && !mem_address_is_vram) begin
	 if(mem_wmask[0]) RAM[ram_word_address][ 7:0 ] <= mem_wdata[ 7:0 ];
	 if(mem_wmask[1]) RAM[ram_word_address][15:8 ] <= mem_wdata[15:8 ];
	 if(mem_wmask[2]) RAM[ram_word_address][23:16] <= mem_wdata[23:16];
	 if(mem_wmask[3]) RAM[ram_word_address][31:24] <= mem_wdata[31:24];	 
      end 
      ram_rdata <= RAM[ram_word_address];
   end
   /* verilator lint_on WIDTH */
`endif
      
`ifdef NRV_MAPPED_SPI_FLASH
   assign mem_rdata = mem_address_is_io  ? io_rdata  : 
		      mem_address_is_ram ? ram_rdata : 
		      mapped_spi_flash_rdata;   
`else   
   assign mem_rdata = mem_address_is_io ? io_rdata : ram_rdata;
`endif   
   
/***************************************************************************************************
/*
 * Memory-mapped IO
 * Mapped IO uses "one-hot" addressing, to make decoder
 * simpler (saves a lot of LUTs), as in J1/swapforth,
 * thanks to Matthias Koch(Mecrisp author) for the idea !
 * The included files contains the symbolic constants that
 * determine which device uses which bit.
 */  

`include "DEVICES/HardwareConfig_bits.v"   

/*
 * Devices are components plugged to the IO memory bus.
 * A few words follow in case you want to write your own devices:
 *
 * Each device has one or several register(s). Each register 
 * can be optionally read or/and written.
 * - Each register is selected by a .sel_xxx signal (where xxx
 *   is the name of the register). With the 1-hot encoding that 
 *   I'm using, .sel_xxx is systematically one of the bits of the 
 *   IO word address (it is also possible to write a real
 *   address decoder, at the expense of eating-up a larger 
 *   number of LUTs).
 * - If the device requires wait cycles for writing and/or reading, 
 *   it can have a .wbusy and/or .rbusy signal(s). All the .wbusy
 *   and .rbusy signals of all the devices are ORed at the end of
 *   this file to form the .io_rbusy and .io_wbusy signals.
 * - If the device has read access, then it has a 32-bits .xxx_rdata
 *   signal, that returns 32'b0 if the device is not selected, or the
 *   read data otherwise. All the .xxx_rdata signals of all the devices
 *   are ORed at the end of this file to form the 32-bits io_rdata signal.
 * - Finally, of course, each device is plugged to some pins of the FPGA,
 *   the corresponding signals are in capital letters. 
 */   


/*********************** Hardware configuration ************/
/*
 * Three memory-mapped constant registers that make it easy for
 * client code to query installed RAM and configured devices
 * (this one does not use any pin, of course).
 * Uses some LUTs, a bit stupid, but more comfortable, so that
 * I do not need to change the software on the SDCard each time 
 * I test a different hardware configuration.
 */
`ifdef NRV_IO_HARDWARE_CONFIG   
wire [31:0] hwconfig_rdata;
HardwareConfig hwconfig(
   .clk(clk),			
   .sel_memory(io_word_address[IO_HW_CONFIG_RAM_bit]),
   .sel_devices(io_word_address[IO_HW_CONFIG_DEVICES_bit]),
   .sel_cpuinfo(io_word_address[IO_HW_CONFIG_CPUINFO_bit]),			
   .rdata(hwconfig_rdata)			 
);
`endif
   
`ifdef NRV_IO_MULTIPLIER
  // Internal connections to the multiplier peripheral
  wire [63:0] mult_rdata;  // Result of multiplication (64 bits)
  wire        mult_rbusy;  // Signal indicating if the multiplier is busy

  // Instantiate multiplier module
  multiplier multiplier_inst (
      .clk(clk),            // System clock
      .rst(reset),          // Synchronized reset signal
      .wstrb(io_wstrb),     // Write strobe signal
      .rstrb(io_rstrb),     // Read strobe signal
      .sel(io_word_address[IO_MULT_A_bit] | io_word_address[IO_MULT_B_bit] | io_word_address[IO_MULT_RESULT_bit]), // Register selection
      .wdata(io_wdata),     // Input data (32 bits)
      .rdata(mult_rdata),   // Result (64 bits)
      .rbusy(mult_rbusy)    // Busy signal
  );

  // Assign output signals for the multiplier
  assign A = io_word_address[IO_MULT_A_bit] ? io_wdata : 32'b0; // Assign A
  assign B = io_word_address[IO_MULT_B_bit] ? io_wdata : 32'b0; // Assign B
  assign wstrb  = io_wstrb;  // Assign write strobe
  assign sel    = io_word_address[IO_MULT_A_bit] | io_word_address[IO_MULT_B_bit] | io_word_address[IO_MULT_RESULT_bit]; // Register selection
  assign rbusy  = mult_rbusy; // Assign busy signal
  assign result = mult_rdata; // Assign multiplication result

`endif  
   
/********************* SPI SDCard  *********************************/
/*
 * This one has an output register directly wired to the CLK,MOSI,CS_N
 * and an input register directly wired to MISO. The software driver
 * implements the SPI protocol by bit-banging (see FIRMWARE/LIBFEMTORV32/spi_sd.c).
 * One day I'll replace it with a hardware driver... if I have time !
 * ... a generic SPI driver would be good to have also.
 */
`ifdef NRV_IO_SDCARD
   wire [31:0] sdcard_rdata;
   SDCard sdcard(
      .clk(clk),
      .rstrb(io_rstrb),
      .wstrb(io_wstrb), 
      .sel(io_word_address[IO_SDCARD_bit]),
      .wdata(io_wdata),
      .rdata(sdcard_rdata),
      .CLK(sd_clk),
      .MISO(sd_miso),		 
      .MOSI(sd_mosi),
      .CS_N(sd_cs_n)
   );
`endif

/************** io_rdata, io_rbusy and io_wbusy signals *************/

/*
 * io_rdata is latched. Not mandatory, but probably allow higher freq, to be tested.
 */
always @(posedge clk) begin
   io_rdata <= 0
`ifdef NRV_IO_HARDWARE_CONFIG	       
            | hwconfig_rdata
`endif	       
`ifdef NRV_IO_MULTIPLIER
   | (io_word_address[IO_MULT_RESULT_bit]      ? mult_rdata[31:0] :     // Lower part
      io_word_address[IO_MULT_RESULT_HI_bit]  ? mult_rdata[63:32] :    // Upper part
      io_word_address[IO_MULT_A_bit] || io_word_address[IO_MULT_B_bit] // Access to A or B
      ? {31'b0, mult_rbusy} : 32'b0)                                   // Show rbusy status
`endif
	    ;
end

   // For now, we got no device that has
   // blocking reads (SPI flash blocks on
   // write address and waits for read data).
   assign io_rbusy = 0 ; 
`ifdef NRV_IO_MULTIPLIER
    | mult_rbusy
`endif
;

   assign io_wbusy = 0
`ifdef NRV_IO_SSD1351_1331
	| SSD1351_wbusy
`endif
`ifdef NRV_IO_MAX7219
	| max7219_wbusy
`endif		   
`ifdef NRV_IO_SPI_FLASH
        | spi_flash_wbusy
`endif		   
; 

/****************************************************************/
/* And last but not least, the processor                        */
   
  reg error=1'b0;
   
  FemtoRV32 #(
     .ADDR_WIDTH(`NRV_ADDR_WIDTH),
     .RESET_ADDR(`NRV_RESET_ADDR)	      
  ) processor(
    .clk(clk),			
    .mem_addr(mem_address),
    .mem_wdata(mem_wdata),
    .mem_wmask(mem_wmask),
    .mem_rdata(mem_rdata),
    .mem_rstrb(mem_rstrb),
    .mem_rbusy(mem_rbusy),
    .mem_wbusy(mem_wbusy),

`ifdef NRV_INTERRUPTS
    .interrupt_request(1'b0),	      
`endif     
    .reset(reset)
  );
   
endmodule
