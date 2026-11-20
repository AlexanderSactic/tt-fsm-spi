# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

EXPECTED_DATA = [0xA0 + i for i in range(8)]


async def wait_first_tx(dut, max_cycles):
    """Espera hasta que el modelo del RFM95 registre la primera transmision."""
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        if int(dut.rf_tx_count.value) >= 1:
            return
    assert False, "TIMEOUT: el nodo no transmitio"


@cocotb.test()
async def test_primer_ciclo(dut):
    dut._log.info("Start")

    # Reloj de 32768 Hz (~30518 ns), el real del nodo
    clock = Clock(dut.clk, 30518, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ena.value = 1
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1

    # Primer ciclo: arranque + medicion + transmision
    await wait_first_tx(dut, 200_000)
    # Deja terminar la limpieza de IRQ y el paso a sleep
    await ClockCycles(dut.clk, 500)

    pkt = int(dut.rf_pkt.value)
    byte = lambda i: (pkt >> (8 * i)) & 0xFF

    assert int(dut.rf_tx_len.value) == 10, "largo del paquete"
    assert byte(0) == 0x01, f"ID = {byte(0):02x}"
    assert byte(1) == 0x00, f"SEQ = {byte(1):02x}"
    for i in range(8):
        assert byte(2 + i) == EXPECTED_DATA[i], \
            f"dato[{i}] = {byte(2 + i):02x} (esperado {EXPECTED_DATA[i]:02x})"

    assert int(dut.rf_opmode.value) == 0x80, "el radio no quedo en sleep"
    assert int(dut.rf_irqflags.value) == 0x00, "IrqFlags sin limpiar"
    assert int(dut.active.value) == 0, "el nodo no se durmio (active=1)"