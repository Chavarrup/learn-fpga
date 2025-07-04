module multiplier (
  input  wire        clk,         // Reloj del sistema
  input  wire        rst,         // Reset del sistema
  input  wire        wstrb,       // Señal de escritura
  input  wire        rstrb,       // Señal de lectura
  input  wire [1:0]  sel,         // Selector de operación: 01->A, 10->B, 11->start/read
  input  wire [31:0] wdata,       // Datos de entrada
  output reg  [63:0] rdata,       // Resultado de la multiplicación
  output reg         rbusy        // Multiplicación en curso
);

  // Registros internos para almacenar los operandos
  reg [31:0] A;
  reg [31:0] B;
  reg [63:0] result;  // Registro de 64 bits para almacenar el resultado
  reg [5:0] counter;  // Contador para las iteraciones (hasta 32)

  // Señal de inicio
  wire start_mult = (sel == 2'b11 && wstrb && !rbusy); // Se activa cuando se inicia la multiplicación

  // Lógica para cargar los registros A y B, y realizar la multiplicación
  always @(posedge clk or posedge rst) begin
    if (rst) begin
        A         <= 32'b0;
        B         <= 32'b0;
        result    <= 64'b0;
        rdata     <= 64'b0;
        rbusy     <= 1'b0;
        counter <= 0;
        
    end else begin
      // Cargar los operandos A y B cuando se activa la señal de escritura
      if (wstrb) begin
        if (sel == 2'b01) begin
          A <= wdata;   // Cargar A con el dato de entrada
        end else if (sel == 2'b10) begin
          B <= wdata;   // Cargar B con el dato de entrada
        end
      end

      // Inicio de la multiplicación (cuando se recibe la señal de inicio)
      if (start_mult) begin
        result <= 64'b0;         // Inicializar el resultado
        counter <= 0;            // Inicializar el contador
        rbusy <= 1'b1;           // Señalizar que el multiplicador está ocupado
      end

      // Ejecución del algoritmo de multiplicación (Multiplicación de Booth o similar)
      if (rbusy) begin
        if (counter < 32) begin
          if (B[0] == 1) begin
            result <= result + {32'b0, A};  // Sumar A al resultado si el bit de B es 1
          end
          A       <= A << 1;  // Desplazar A a la izquierda (equivalente a multiplicar A por 2)
          B       <= B >> 1;  // Desplazar B a la derecha (equivalente a dividir B por 2)
          counter <= counter + 1;  // Incrementar el contador
        end else begin
          rbusy <= 1'b0;  // Multiplicación completada
        end
      end

      // Asignación de resultado cuando la lectura es activa (rstrb) y no está ocupado (rbusy)
      if (rstrb && (sel == 2'b11) && !rbusy) begin
        rdata <= result; // Asignar el resultado a rdata cuando no hay multiplicación en curso
      end
    end
  end

endmodule
