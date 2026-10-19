"""A master call interrupted by reset is canceled, never reported as OKAY."""

import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import (
    ClockCycles, FallingEdge, RisingEdge, Timer, with_timeout,
)
from cocotb_tools.check_results import get_results
from cocotb_tools.runner import get_runner
from const import cfg
from test_ahb_lite_slave_reset import RecordingSlave

from cocotbext.ahb import AHBBus, AHBLiteMaster, AHBResp, AHBTrans


@cocotb.test(timeout_time=3000, timeout_unit="ns")
@cocotb.parametrize(
    reset_active_low=[True, False], write=[False, True], pipelined=[False, True]
)
async def reset_cancels_master(dut, reset_active_low, write, pipelined):
    asserted = int(not reset_active_low)
    released = int(reset_active_low)
    dut.hresetn.value = asserted
    bus = AHBBus.from_entity(dut)
    slave = RecordingSlave(
        bus, dut.hclk, dut.hresetn, reset_act_low=reset_active_low,
    )
    master = AHBLiteMaster(
        bus, dut.hclk, dut.hresetn, reset_act_low=reset_active_low,
        def_val="Z" if pipelined else 0,
    )
    clock = cocotb.start_soon(
        Clock(dut.hclk, 10, unit="ns").start(start_high=False)
    )
    await ClockCycles(dut.hclk, 3)
    await FallingEdge(dut.hclk)
    dut.hresetn.value = released
    await ClockCycles(dut.hclk, 2)
    await FallingEdge(dut.hclk)

    addresses = [0x10, 0x14, 0x18] if pipelined else [0x10]
    operation = (
        master.write(addresses, [0x11223344] * len(addresses), pip=pipelined)
        if write else master.read(addresses, pip=pipelined)
    )
    request = cocotb.start_soon(expect_cancellation(operation))
    for _ in range(20):
        await RisingEdge(dut.hclk)
        await Timer(1, unit="step")
        if slave.requests:
            break
    assert slave.requests, "Request did not reach the stalled slave"
    assert int(dut.hready.value) == 0
    await FallingEdge(dut.hclk)
    # Stop the clock too: reset cancellation must not depend on another edge.
    clock.cancel()
    dut.hresetn.value = asserted
    await with_timeout(request, 20, "ns")
    await Timer(1, unit="step")
    assert int(dut.htrans.value) == AHBTrans.IDLE
    assert slave.writes == []
    await expect_cancellation(master.read(0x20))

    slave.stall = False
    clock = cocotb.start_soon(
        Clock(dut.hclk, 10, unit="ns").start(start_high=False)
    )
    await ClockCycles(dut.hclk, 2)
    await FallingEdge(dut.hclk)
    dut.hresetn.value = released
    await ClockCycles(dut.hclk, 2)
    await FallingEdge(dut.hclk)
    response = await master.write(0x40, 0xAABBCCDD, sync=True)
    assert [item["resp"] for item in response] == [AHBResp.OKAY]
    assert slave.writes == [(0x40, 0xAABBCCDD)]
    response = await master.read(0x44, sync=True)
    assert int(response[0]["data"], 16) == 0xA5000044
    assert response[0]["resp"] == AHBResp.OKAY
    clock.cancel()


async def expect_cancellation(operation):
    try:
        await operation
    except ConnectionAbortedError:
        return
    raise AssertionError("Reset-interrupted transfer returned success")


def test_ahb_lite_master_reset():
    module = os.path.splitext(os.path.basename(__file__))[0]
    sim_build = os.path.join(
        cfg.TESTS_DIR, f"../run_dir/sim_build_{cfg.SIMULATOR}_{module}"
    )
    runner = get_runner(cfg.SIMULATOR)
    runner.build(
        sources=cfg.VERILOG_SOURCES, hdl_toplevel=cfg.TOPLEVEL,
        build_dir=sim_build, timescale=cfg.TIMESCALE,
        build_args=cfg.EXTRA_ARGS, waves=True,
    )
    results = runner.test(
        test_module=module, hdl_toplevel=cfg.TOPLEVEL, waves=True,
        extra_env=cfg.EXTRA_ENV, log_file=sim_build + "_run.log",
    )
    _, failures = get_results(results)
    assert failures == 0
