    .section .text
    .globl  main
    .type   main, @function

    # Importa IO_BASE y todas las macros IO_* definidas en HardwareConfig_bits.inc
    .include "femtorv32.inc"

main:
    # --------------------------------------
    # 1 Escribir operando A = 7
    # --------------------------------------
    la   t0, IO_MULT_A        # t0 = dirección de A
    li   t1, 7
    sw   t1, 0(t0)

    # --------------------------------------
    # 2 Escribir operando B = 9
    # --------------------------------------
    la   t0, IO_MULT_B        # t0 = dirección de B
    li   t1, 9
    sw   t1, 0(t0)

    # --------------------------------------
    # 3 Lanzar la multiplicación
    #    (escribiendo cualquier cosa en IO_MULT_RES_LO)
    # --------------------------------------
    la   t0, IO_MULT_RES_LO   # t0 = dirección parte baja resultado
    sw   zero, 0(t0)

wait_busy:
    # --------------------------------------
    # 4 Esperar a que rbusy==0 (bit0 de IO_MULT_RES_LO)
    # --------------------------------------
    lw   t2, 0(t0)            # lee parte baja (bit0 = busy)
    andi t3, t2, 1            # aisla bit0
    bnez t3, wait_busy        # si busy==1, sigue esperando

    # --------------------------------------
    # 5 Leer resultado de 64 bits
    # --------------------------------------
    # Parte baja
    lw   t1, 0(t0)

    # Parte alta
    la   t0, IO_MULT_RES_HI
    lw   t2, 0(t0)

    # --------------------------------------
    # 6 Guardar resultado en RAM en 0x10000
    # --------------------------------------
    li   t0, 0x10000
    sw   t1, 0(t0)            # baja en [0x10000]
    sw   t2, 4(t0)            # alta en [0x10004]

    # --------------------------------------
    # 7 Volver al entorno (ret)
    # --------------------------------------
    ret
