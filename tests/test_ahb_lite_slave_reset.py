"""Reset cancels pending slave phases without later model side effects."""

import os

import cocotb
from cocotb.triggers import Timer
from cocotb_tools.check_results import get_results
from cocotb_tools.runner import get_runner
from const import cfg

from cocotbext.ahb import AHBBus, AHBLiteSlave, AHBResp, AHBTrans


class RecordingSlave(AHBLiteSlave):
    """Observe model hooks without replacing the slave protocol service."""

    def __init__(self, *args, **kwargs):
        self.stall = True
        self.requests = []
        self.writes = []
        super().__init__(*args, bp=self.ready_pattern(), **kwargs)

    def ready_pattern(self):
        while True:
            yield not self.stall

    def _chk_wr(self, addr, size):
        self.requests.append(("write", int(addr)))
        return int(addr) != 0x100

    def _chk_rd(self, addr, size):
        self.requests.append(("read", int(addr)))
        return int(addr) != 0x100

    def _rd(self, addr, size):
        return 0xA5000000 ^ int(addr)

    def _wr(self, addr, size, value):
        self.writes.append((int(addr), int(value)))
        return 0


class ReferenceMaster:
    """Drive stable requests and record handshakes before each real edge."""

    def __init__(self, dut, reset_active_low):
        self.dut = dut
        self.reset_active_low = reset_active_low
        self.pending = None
        self.completed = []
        dut.hclk.value = 0
        self.drive()

    def drive(self, addr=0, data=0, write=False, trans=AHBTrans.IDLE):
        self.dut.haddr.value = addr
        self.dut.hwdata.value = data
        self.dut.hwrite.value = int(write)
        self.dut.htrans.value = trans
        self.dut.hsize.value = 2
        self.dut.hsel.value = 1
        self.dut.hready_in.value = 1

    async def cycle(self):
        self.dut.hclk.value = 0
        await Timer(5, unit="ns")
        reset = int(self.dut.hresetn.value) == int(not self.reset_active_low)
        if reset:
            self.pending = None
        elif int(self.dut.hready.value):
            if self.pending is not None:
                address, write = self.pending
                data = self.dut.hwdata if write else self.dut.hrdata
                self.completed.append(
                    (address, int(data.value), int(self.dut.hresp.value))
                )
            self.pending = (
                (int(self.dut.haddr.value), bool(self.dut.hwrite.value))
                if int(self.dut.htrans.value) & 2
                else None
            )
        self.dut.hclk.value = 1
        await Timer(5, unit="ns")


@cocotb.test(timeout_time=2000, timeout_unit="ns")
@cocotb.parametrize(
    reset_active_low=[True, False], interrupted=["write", "read", "error"]
)
async def reset_pending_phase(dut, reset_active_low, interrupted):
    asserted = int(not reset_active_low)
    released = int(reset_active_low)
    dut.hresetn.value = asserted
    master = ReferenceMaster(dut, reset_active_low)
    slave = RecordingSlave(
        AHBBus.from_entity(dut), dut.hclk, dut.hresetn,
        reset_act_low=reset_active_low,
    )
    for _ in range(3):
        await master.cycle()
    dut.hresetn.value = released
    await master.cycle()

    address = 0x100 if interrupted == "error" else 0x10
    master.drive(addr=address, write=interrupted == "write", trans=AHBTrans.NONSEQ)
    await master.cycle()
    assert slave.requests == [("write" if interrupted == "write" else "read", address)]
    assert master.completed == []
    assert int(dut.hready.value) == 0

    dut.hresetn.value = asserted
    master.drive(data=0x77778888)
    for _ in range(3):
        await master.cycle()
        assert slave.writes == [], "Canceled write reached the model during reset"
        assert int(dut.hready.value) == 1
        assert int(dut.hresp.value) == AHBResp.OKAY
        assert int(dut.hrdata.value) == 0

    # Even stale/non-idle bus qualifiers must not create a reset-time request.
    master.drive(addr=0x30, write=True, trans=AHBTrans.NONSEQ)
    for _ in range(2):
        await master.cycle()
    assert len(slave.requests) == 1, "Slave accepted a new request while reset was held"
    assert slave.writes == []
    assert master.completed == []

    master.drive()
    slave.stall = False
    dut.hresetn.value = released
    for _ in range(3):
        await master.cycle()
    assert slave.writes == [], "Canceled write completed after reset release"

    master.drive(addr=0x40, write=True, trans=AHBTrans.NONSEQ)
    await master.cycle()
    master.drive(data=0x11223344)
    await master.cycle()
    assert slave.writes == [(0x40, 0x11223344)]
    assert master.completed == [(0x40, 0x11223344, AHBResp.OKAY)]

    master.drive(addr=0x44, trans=AHBTrans.NONSEQ)
    await master.cycle()
    master.drive()
    await master.cycle()
    assert master.completed[-1] == (0x44, 0xA5000044, AHBResp.OKAY)
    assert slave.requests[-2:] == [("write", 0x40), ("read", 0x44)]


def test_ahb_lite_slave_reset():
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
