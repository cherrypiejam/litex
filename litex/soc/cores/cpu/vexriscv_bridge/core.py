#
# This file is part of LiteX.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2020 Dolu1990 <charles.papon.90@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

import os
import subprocess

from migen import *

from litex.gen import *

from litex import get_data_mod

from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr import *
from litex.soc.integration.soc import SoCRegion

from litex.soc.cores.cpu import CPU, CPU_GCC_TRIPLE_RISCV32

# VexRiscv Bridge Architecture ----------------------------------------------------------------------

class VexRiscvBridge(CPU):
    category             = "softcore"
    family               = "riscv"
    name                 = "vexriscv"
    human_name           = "VexRiscv Bridge"
    variants             = ["standard", "linux"]
    data_width           = 32
    endianness           = "little"
    gcc_triple           = CPU_GCC_TRIPLE_RISCV32
    linker_output_format = "elf32-littleriscv"
    nop                  = "nop"
    io_regions           = {0x8000_0000: 0x8000_0000} # Origin, Length.

    # Default parameters.
    cpu_count            = 1
    dcache_size          = 4096
    icache_size          = 4096
    dcache_ways          = 1
    icache_ways          = 1
    coherent_dma         = False
    litedram_width       = 32
    dcache_width         = 32
    icache_width         = 32
    aes_instruction      = False
    expose_time          = False
    out_of_order_decoder = True
    privileged_debug     = False
    hardware_breakpoints = 0
    wishbone_memory      = False
    wishbone_force_32b   = False
    with_fpu             = False
    cpu_per_fpu          = 4
    with_rvc             = False
    jtag_tap             = False
    dtlb_size            = 4
    itlb_size            = 4
    csr_base             = 0xf000_0000
    clint_base           = 0xf001_0000
    plic_base            = 0xf0c0_0000
    reset_vector         = 0
    with_mmu             = False
    with_formal          = False
    pmp_regions          = 16
    pmp_granularity      = 256
    with_supervisor      = False
    shared_region_base   = 0x4000_0000
    shared_region_size   = 0x1000_0000

    # Command line configuration arguments.
    @staticmethod
    def args_fill(parser):
        cpu_group = parser.add_argument_group(title="CPU options")
        cpu_group.add_argument("--cpu-count",                    default=1,            help="Number of CPU(s) in the cluster.", type=int)
        cpu_group.add_argument("--with-coherent-dma",            action="store_true",  help="Enable Coherent DMA Slave interface.")
        cpu_group.add_argument("--without-coherent-dma",         action="store_true",  help="Disable Coherent DMA Slave interface.")
        cpu_group.add_argument("--dcache-width",                 default=None,         help="L1 data cache bus width.")
        cpu_group.add_argument("--icache-width",                 default=None,         help="L1 instruction cache bus width.")
        cpu_group.add_argument("--dcache-size",                  default=None,         help="L1 data cache size in byte per CPU.")
        cpu_group.add_argument("--dcache-ways",                  default=None,         help="L1 data cache ways per CPU.")
        cpu_group.add_argument("--icache-size",                  default=None,         help="L1 instruction cache size in byte per CPU.")
        cpu_group.add_argument("--icache-ways",                  default=None,         help="L1 instruction cache ways per CPU")
        cpu_group.add_argument("--aes-instruction",              default=None,         help="Enable AES instruction acceleration.")
        cpu_group.add_argument("--without-out-of-order-decoder", action="store_true",  help="Reduce area at cost of peripheral access speed")
        cpu_group.add_argument("--with-wishbone-memory",         action="store_true",  help="Disable native LiteDRAM interface")
        cpu_group.add_argument("--with-privileged-debug",        action="store_true",  help="Enable official RISC-V debug spec")
        cpu_group.add_argument("--hardware-breakpoints",         default=1,            help="Number of hardware breapoints", type=int)
        cpu_group.add_argument("--wishbone-force-32b",           action="store_true",  help="Force the wishbone bus to be 32 bits")
        cpu_group.add_argument("--with-fpu",                     action="store_true",  help="Enable the F32/F64 FPU")
        cpu_group.add_argument("--cpu-per-fpu",                  default="4",          help="Maximal ratio between CPU count and FPU count. Will instanciate as many FPU as necessary.")
        cpu_group.add_argument("--with-rvc",                     action="store_true",  help="Enable RISC-V compressed instruction support")
        cpu_group.add_argument("--dtlb-size",                    default=4,            help="Data TLB size.")
        cpu_group.add_argument("--itlb-size",                    default=4,            help="Instruction TLB size.")
        cpu_group.add_argument("--expose-time",                  action="store_true",  help="Add CLINT time output.")
        cpu_group.add_argument("--csr-base",                     default="0xf0000000", help="CSR base address.")
        cpu_group.add_argument("--clint-base",                   default="0xf0010000", help="CLINT base address.")
        cpu_group.add_argument("--plic-base",                    default="0xf0c00000", help="PLIC base address.")
        cpu_group.add_argument("--jtag-tap",                     action="store_true",  help="Add the jtag tap instead of jtag instruction interface")
        cpu_group.add_argument("--with-formal",                  action="store_true",  help="Enable formal interface")
        cpu_group.add_argument("--shared-region-base",           default=None,         help="Shared memory region base address")
        cpu_group.add_argument("--shared-region-size",           default=None,         help="Shared memory region size")

    @staticmethod
    def args_read(args):
        VexRiscvBridge.cpu_count = args.cpu_count
        if int(args.cpu_count) != 1:
            VexRiscvBridge.icache_width = 64
            VexRiscvBridge.dcache_width = 64
            VexRiscvBridge.dcache_size  = 8192
            VexRiscvBridge.icache_size  = 8192
            VexRiscvBridge.dcache_ways  = 2
            VexRiscvBridge.icache_ways  = 2
            VexRiscvBridge.coherent_dma = True
        if(args.with_coherent_dma):            VexRiscvBridge.coherent_dma          = bool(True)
        if(args.without_coherent_dma):         VexRiscvBridge.coherent_dma          = bool(False)
        if(args.dcache_width):                 VexRiscvBridge.dcache_width          = int(args.dcache_width)
        if(args.icache_width):                 VexRiscvBridge.icache_width          = int(args.icache_width)
        if(args.dcache_size):                  VexRiscvBridge.dcache_size           = int(args.dcache_size)
        if(args.icache_size):                  VexRiscvBridge.icache_size           = int(args.icache_size)
        if(args.dcache_ways):                  VexRiscvBridge.dcache_ways           = int(args.dcache_ways)
        if(args.icache_ways):                  VexRiscvBridge.icache_ways           = int(args.icache_ways)
        if(args.aes_instruction):              VexRiscvBridge.aes_instruction       = bool(args.aes_instruction)
        if(args.expose_time):                  VexRiscvBridge.expose_time           = bool(args.expose_time)
        if(args.without_out_of_order_decoder): VexRiscvBridge.out_of_order_decoder  = False
        if(args.with_privileged_debug):        VexRiscvBridge.privileged_debug      = True
        if(args.hardware_breakpoints):         VexRiscvBridge.hardware_breakpoints  = args.hardware_breakpoints
        if(args.with_wishbone_memory):         VexRiscvBridge.wishbone_memory       = True
        if(args.wishbone_force_32b):           VexRiscvBridge.wishbone_force_32b    = True
        if(args.with_fpu):
            VexRiscvBridge.with_fpu     = True
            VexRiscvBridge.icache_width = 64
            VexRiscvBridge.dcache_width = 64 # Required for F64
        if(args.with_formal):
            VexRiscvBridge.with_formal = True
        if(args.cpu_per_fpu):
            VexRiscvBridge.cpu_per_fpu = args.cpu_per_fpu
        if(args.with_rvc):
            VexRiscvBridge.with_rvc = True
        if(args.dtlb_size):  VexRiscvBridge.dtlb_size  = int(args.dtlb_size)
        if(args.itlb_size):  VexRiscvBridge.itlb_size  = int(args.itlb_size)
        if(args.csr_base):   VexRiscvBridge.csr_base   = int(args.csr_base, 16)
        if(args.clint_base): VexRiscvBridge.clint_base = int(args.clint_base, 16)
        if(args.plic_base):  VexRiscvBridge.plic_base  = int(args.plic_base, 16)
        if(args.jtag_tap):   VexRiscvBridge.jtag_tap   = int(args.jtag_tap)
        if(args.shared_region_base):   VexRiscvBridge.shared_region_base = int(args.shared_region_base)
        if(args.shared_region_size):   VexRiscvBridge.shared_region_size = int(args.shared_region_size)

    # ABI.
    @staticmethod
    def get_abi():
        abi = "ilp32"
        if VexRiscvBridge.with_fpu:
            abi +="d"
        return abi

    # Arch.
    @staticmethod
    def get_arch():
        arch = "rv32i2p0_ma"
        if VexRiscvBridge.with_fpu:
            arch += "fd"
        if VexRiscvBridge.with_rvc:
            arch += "c"
        return arch

    # Memory Mapping.
    @property
    def mem_map(self):
        return {
            "rom":      0x0000_0000,
            "sram":     0x1000_0000,
            "main_ram": 0x4000_0000,
            "csr":      VexRiscvBridge.csr_base,
            "clint":    VexRiscvBridge.clint_base,
            "plic":     VexRiscvBridge.plic_base,
        }

    # GCC Flags.
    @property
    def gcc_flags(self):
        flags =  f" -march={VexRiscvBridge.get_arch()} -mabi={VexRiscvBridge.get_abi()}"
        flags += f" -D__vexriscv_bridge__"
        flags += f" -D__riscv_plic__"
        return flags

    # Reserved Interrupts.
    @property
    def reserved_interrupts(self):
        return {"noirq": 0}

    # Cluster Name Generation.
    @staticmethod
    def generate_cluster_name():
        ldw = f"Ldw{VexRiscvBridge.litedram_width}"
        VexRiscvBridge.cluster_name = f"VexRiscvBridgeLitexSmpCluster_" \
        f"{'R' + hex(VexRiscvBridge.reset_vector) if VexRiscvBridge.reset_vector else ''}"\
        f"Cc{VexRiscvBridge.cpu_count}"    \
        "_" \
        f"Iw{VexRiscvBridge.icache_width}" \
        f"Is{VexRiscvBridge.icache_size}"  \
        f"Iy{VexRiscvBridge.icache_ways}"  \
        "_" \
        f"Dw{VexRiscvBridge.dcache_width}" \
        f"Ds{VexRiscvBridge.dcache_size}"  \
        f"Dy{VexRiscvBridge.dcache_ways}"  \
        "_" \
        f"ITs{VexRiscvBridge.itlb_size}" \
        f"DTs{VexRiscvBridge.dtlb_size}" \
        f"{'_'+ldw if not VexRiscvBridge.wishbone_memory  else ''}" \
        f"{'_Cdma' if VexRiscvBridge.coherent_dma         else ''}" \
        f"{'_Aes'  if VexRiscvBridge.aes_instruction      else ''}" \
        f"{'_Time' if VexRiscvBridge.expose_time          else ''}" \
        f"{'_Ood'  if VexRiscvBridge.out_of_order_decoder else ''}" \
        f"{'_Wm'   if VexRiscvBridge.wishbone_memory      else ''}" \
        f"{'_Wf32' if VexRiscvBridge.wishbone_force_32b   else ''}" \
        f"{'_Fpu' + str(VexRiscvBridge.cpu_per_fpu) if VexRiscvBridge.with_fpu else ''}" \
        f"{'_Pd'   if VexRiscvBridge.privileged_debug else ''}" \
        f"{'_Hb' + str(VexRiscvBridge.hardware_breakpoints) if VexRiscvBridge.hardware_breakpoints > 0 else ''}" \
        f"{'_Rvc'   if VexRiscvBridge.with_rvc    else ''}" \
        f"{'_JtagT' if VexRiscvBridge.jtag_tap    else ''}" \
        "_" \
        f"Pr{VexRiscvBridge.pmp_regions}"                   \
        f"Pg{VexRiscvBridge.pmp_granularity}"               \
        "_" \
        f"Sr{hex(VexRiscvBridge.shared_region_base)}"                                    \
        f"_{hex(VexRiscvBridge.shared_region_base + VexRiscvBridge.shared_region_size)}" \
        f"{'_Fml'  if VexRiscvBridge.with_formal else ''}"  \
        f"{'_Supr' if VexRiscvBridge.with_supervisor else ''}"

    # Default Configs Generation.
    @staticmethod
    def generate_default_configs():
        # Sim
        # Single core
        VexRiscvBridge.cpu_count = 1

        # Cache parameters
        # Let's not worry about cache for now
        VexRiscvBridge.icache_width = 32
        VexRiscvBridge.dcache_width = 32
        # VexRiscvBridge.dcache_size  = 0
        # VexRiscvBridge.icache_size  = 0
        VexRiscvBridge.dcache_size  = 4096
        VexRiscvBridge.icache_size  = 4096
        VexRiscvBridge.dcache_ways  = 1
        VexRiscvBridge.icache_ways  = 1

        # DMA, memory, etc.
        VexRiscvBridge.wishbone_memory      = True
        VexRiscvBridge.with_rvc             = True
        VexRiscvBridge.with_mmu             = False
        VexRiscvBridge.coherent_dma         = False
        VexRiscvBridge.hardware_breakpoints = 0

        # With formal
        VexRiscvBridge.with_formal          = True

        # PMP
        VexRiscvBridge.pmp_regions          = 16
        VexRiscvBridge.pmp_granularity      = 256

        # Without supervisor
        VexRiscvBridge.with_supervisor      = False

        VexRiscvBridge.generate_cluster_name()
        VexRiscvBridge.generate_netlist()

        # Multi cores.
        for core_count in [2]:
            VexRiscvBridge.cpu_count = core_count
            VexRiscvBridge.generate_cluster_name()
            VexRiscvBridge.generate_netlist()

    # Netlist Generation.
    @staticmethod
    def generate_netlist():
        print(f"Generating cluster netlist")
        vdir = get_data_mod("cpu", "vexriscv_bridge").data_location
        gen_args = []
        if(VexRiscvBridge.coherent_dma):
            gen_args.append("--coherent-dma")
        gen_args.append(f"--cpu-count={VexRiscvBridge.cpu_count}")
        gen_args.append(f"--ibus-width={VexRiscvBridge.icache_width}")
        gen_args.append(f"--dbus-width={VexRiscvBridge.dcache_width}")
        gen_args.append(f"--dcache-size={VexRiscvBridge.dcache_size}")
        gen_args.append(f"--icache-size={VexRiscvBridge.icache_size}")
        gen_args.append(f"--dcache-ways={VexRiscvBridge.dcache_ways}")
        gen_args.append(f"--icache-ways={VexRiscvBridge.icache_ways}")
        gen_args.append(f"--litedram-width={VexRiscvBridge.litedram_width}")
        gen_args.append(f"--aes-instruction={VexRiscvBridge.aes_instruction}")
        gen_args.append(f"--expose-time={VexRiscvBridge.expose_time}")
        gen_args.append(f"--out-of-order-decoder={VexRiscvBridge.out_of_order_decoder}")
        gen_args.append(f"--privileged-debug={VexRiscvBridge.privileged_debug}")
        gen_args.append(f"--hardware-breakpoints={VexRiscvBridge.hardware_breakpoints}")
        gen_args.append(f"--wishbone-memory={VexRiscvBridge.wishbone_memory}")
        if(VexRiscvBridge.wishbone_force_32b): gen_args.append(f"--wishbone-force-32b={VexRiscvBridge.wishbone_force_32b}")
        gen_args.append(f"--fpu={VexRiscvBridge.with_fpu}")
        gen_args.append(f"--cpu-per-fpu={VexRiscvBridge.cpu_per_fpu}")
        gen_args.append(f"--rvc={VexRiscvBridge.with_rvc}")
        gen_args.append(f"--netlist-name={VexRiscvBridge.cluster_name}")
        gen_args.append(f"--netlist-directory={vdir}")
        gen_args.append(f"--dtlb-size={VexRiscvBridge.dtlb_size}")
        gen_args.append(f"--itlb-size={VexRiscvBridge.itlb_size}")
        gen_args.append(f"--mmu={VexRiscvBridge.with_mmu}")
        gen_args.append(f"--formal={VexRiscvBridge.with_formal}")
        gen_args.append(f"--pmp-regions={VexRiscvBridge.pmp_regions}")
        gen_args.append(f"--pmp-granularity={VexRiscvBridge.pmp_granularity}")
        gen_args.append(f"--supervisor={VexRiscvBridge.with_supervisor}")
        gen_args.append(f"--shared-region-start={VexRiscvBridge.shared_region_base}")
        gen_args.append(f"--shared-region-size={VexRiscvBridge.shared_region_size}")

        cmd = 'cd {path} && sbt "runMain vexriscv.demo.bridge.VexRiscvBridgeLitexSmpClusterCmdGen {args}"'.format(path=os.path.join(vdir, "ext", "VexRiscv"), args=" ".join(gen_args))
        subprocess.check_call(cmd, shell=True)

        cluster_filename = os.path.join(vdir,  VexRiscvBridge.cluster_name + ".v")
        def add_synthesis_define(filename):
            """Add SYNTHESIS define to verilog for toolchains requiring it, ex Gowin"""
            synthesis_define = "`define SYNTHESIS\n"
            # Read file.
            with open(filename, "r") as f:
                lines = f.readlines()
            # Modify file.
            with open(filename, "w") as f:
                if lines[0] != synthesis_define:
                    f.write(synthesis_define)
                for line in lines:
                    f.write(line)
        add_synthesis_define(cluster_filename)


    def __init__(self, platform, variant):
        self.platform         = platform
        self.variant          = variant
        self.human_name       = self.human_name + "-" + self.variant.upper()
        self.reset            = Signal()

        if VexRiscvBridge.jtag_tap:
            self.jtag_clk = Signal()
            self.jtag_tdo = Signal()
            self.jtag_tdi = Signal()
            self.jtag_tms = Signal()
        else:
            self.jtag_clk     = Signal()
            self.jtag_tdo     = Signal()
            self.jtag_tdi     = Signal()
            self.jtag_reset   = Signal()
            self.jtag_enable  = Signal()
            self.jtag_capture = Signal()
            self.jtag_shift   = Signal()
            self.jtag_update  = Signal()

        self.interrupt        = Signal(32)
        self.pbus_shared      = pbus_shared = wishbone.Interface(data_width={
            # Always 32-bit when using direct LiteDRAM interfaces.
            False : 32,
            # Else max of I/DCache-width.
            True  : max(VexRiscvBridge.icache_width, VexRiscvBridge.dcache_width),
        }[VexRiscvBridge.wishbone_memory and not VexRiscvBridge.wishbone_force_32b], addressing="word")
        self.pbus_locals      = pbus_locals = [
            wishbone.Interface(
                data_width={
                    # Always 32-bit when using direct LiteDRAM interfaces.
                    False : 32,
                    # Else max of I/DCache-width.
                    True  : max(VexRiscvBridge.icache_width, VexRiscvBridge.dcache_width),
                }[VexRiscvBridge.wishbone_memory and not VexRiscvBridge.wishbone_force_32b],
                addressing="word"
            )
            for _ in range(VexRiscvBridge.cpu_count)
        ]
        self.periph_buses     = [pbus_shared] + pbus_locals # Peripheral buses (Connected to main SoC's bus).
        # self.periph_buses = [pbus_shared]
        self.memory_buses     = []     # Memory buses (Connected directly to LiteDRAM).

        # # #

        self.cpu_params = dict(
            # Clk / Rst.
            i_debugCd_external_clk   = ClockSignal("sys"),
            i_debugCd_external_reset = ResetSignal("sys") | self.reset,

            # Interrupts.
            i_interrupts = self.interrupt,

            # Shared Peripheral Bus (Master).
            o_peripheral_CYC      = pbus_shared.cyc,
            o_peripheral_STB      = pbus_shared.stb,
            i_peripheral_ACK      = pbus_shared.ack,
            o_peripheral_WE       = pbus_shared.we,
            o_peripheral_ADR      = pbus_shared.adr,
            i_peripheral_DAT_MISO = pbus_shared.dat_r,
            o_peripheral_DAT_MOSI = pbus_shared.dat_w,
            o_peripheral_SEL      = pbus_shared.sel,
            i_peripheral_ERR      = pbus_shared.err,
            o_peripheral_CTI      = pbus_shared.cti,
            o_peripheral_BTE      = pbus_shared.bte
        ) | dict (
            # Local Peripheral Buses (Master).
            (k.format(cpu_index), v(pbus_locals[cpu_index]))
            for (k, v) in [
                ("o_peripheralLocals_{}_CYC"     ,   lambda p: p.cyc),
                ("o_peripheralLocals_{}_STB"     ,   lambda p: p.stb),
                ("i_peripheralLocals_{}_ACK"     ,   lambda p: p.ack),
                ("o_peripheralLocals_{}_WE"      ,   lambda p: p.we),
                ("o_peripheralLocals_{}_ADR"     ,   lambda p: p.adr),
                ("i_peripheralLocals_{}_DAT_MISO",   lambda p: p.dat_r),
                ("o_peripheralLocals_{}_DAT_MOSI",   lambda p: p.dat_w),
                ("o_peripheralLocals_{}_SEL"     ,   lambda p: p.sel),
                ("i_peripheralLocals_{}_ERR"     ,   lambda p: p.err),
                ("o_peripheralLocals_{}_CTI"     ,   lambda p: p.cti),
                ("o_peripheralLocals_{}_BTE"     ,   lambda p: p.bte),
            ]
            for cpu_index in range(VexRiscvBridge.cpu_count)
        )

        if VexRiscvBridge.jtag_tap:
            self.cpu_params.update(
                i_debugPort_tck     = self.jtag_clk,
                i_debugPort_tms     = self.jtag_tms,
                i_debugPort_tdi     = self.jtag_tdi,
                o_debugPort_tdo     = self.jtag_tdo
            )
        else:
            self.cpu_params.update(
                i_jtag_clk          = self.jtag_clk,
                i_debugPort_enable  = self.jtag_enable,
                i_debugPort_capture = self.jtag_capture,
                i_debugPort_shift   = self.jtag_shift,
                i_debugPort_update  = self.jtag_update,
                i_debugPort_reset   = self.jtag_reset,
                i_debugPort_tdi     = self.jtag_tdi,
                o_debugPort_tdo     = self.jtag_tdo
            )

        # DMA.
        if VexRiscvBridge.coherent_dma:
            self.dma_bus = dma_bus = wishbone.Interface(data_width=VexRiscvBridge.dcache_width, address_width=32, addressing="word")
            dma_bus_stall   = Signal()
            dma_bus_inhibit = Signal()
            self.cpu_params.update(
                # DMA Bus (Slave).
                i_dma_wishbone_CYC      = dma_bus.cyc,
                i_dma_wishbone_STB      = dma_bus.stb & ~dma_bus_inhibit,
                o_dma_wishbone_ACK      = dma_bus.ack,
                i_dma_wishbone_WE       = dma_bus.we,
                i_dma_wishbone_SEL      = dma_bus.sel,
                i_dma_wishbone_ADR      = dma_bus.adr,
                o_dma_wishbone_DAT_MISO = dma_bus.dat_r,
                i_dma_wishbone_DAT_MOSI = dma_bus.dat_w,
                o_dma_wishbone_STALL    = dma_bus_stall
            )
            self.sync += [
                If(dma_bus.stb & dma_bus.cyc & ~dma_bus_stall,
                    dma_bus_inhibit.eq(1),
                ),
                If(dma_bus.ack,
                   dma_bus_inhibit.eq(0)
                )
            ]

        # expose CLINT time
        if VexRiscvBridge.expose_time:
            self.clint_time = Signal(64)
            self.cpu_params.update(
                o_clint_time    = self.clint_time
            )

    def set_reset_address(self, reset_address):
        self.reset_address = reset_address
        VexRiscvBridge.reset_vector = reset_address

    def add_sources(self, platform):
        vdir = get_data_mod("cpu", "vexriscv_bridge").data_location
        print(f"VexRiscv cluster : {self.cluster_name}")
        if not os.path.exists(os.path.join(vdir, self.cluster_name + ".v")):
            self.generate_netlist()


        # Add RAM.

        # By default, use Generic RAM implementation.
        ram_filename = "Ram_1w_1rs_Generic.v"
        # On Altera/Intel platforms, use specific implementation.
        from litex.build.altera import AlteraPlatform
        if isinstance(platform, AlteraPlatform):
            ram_filename = "Ram_1w_1rs_Intel.v"
            # define SYNTHESIS verilog name to avoid issues with unsupported
            # functions
            platform.toolchain.additional_qsf_commands.append(
                'set_global_assignment -name VERILOG_MACRO "SYNTHESIS=1"')
        # On Efinix platforms, use specific implementation.
        from litex.build.efinix import EfinixPlatform
        if isinstance(platform, EfinixPlatform):
            ram_filename = "Ram_1w_1rs_Efinix.v"
        platform.add_source(os.path.join(vdir, ram_filename), "verilog")

        # Add Cluster.
        cluster_filename = os.path.join(vdir,  self.cluster_name + ".v")
        if not os.path.exists(cluster_filename):
            def add_synthesis_define(filename):
                """Add SYNTHESIS define to verilog for toolchains requiring it, ex Gowin"""
                synthesis_define = "`define SYNTHESIS\n"
                # Read file.
                with open(filename, "r") as f:
                    lines = f.readlines()
                    # Modify file.
                with open(filename, "w") as f:
                    if lines[0] != synthesis_define:
                        f.write(synthesis_define)
                    for line in lines:
                        f.write(line)
            add_synthesis_define(cluster_filename)
        platform.add_source(cluster_filename, "verilog")

    def add_jtag(self, pads):
        self.comb += [
            self.jtag_tms.eq(pads.tms),
            self.jtag_clk.eq(pads.tck),
            self.jtag_tdi.eq(pads.tdi),
            pads.tdo.eq(self.jtag_tdo),
        ]

    def add_soc_components(self, soc):
        if self.variant == "linux":
            # Set UART/Timer0 CSRs to the ones used by OpenSBI.
            soc.csr.add("uart",   n=2)
            soc.csr.add("timer0", n=3)

            # Add OpenSBI region.
            soc.bus.add_region("opensbi", SoCRegion(origin=self.mem_map["main_ram"] + 0x00f0_0000, size=0x8_0000, cached=True, linker=True))

        # Define number of CPUs
        soc.add_config("CPU_COUNT", VexRiscvBridge.cpu_count)
        soc.add_config("CPU_ISA",   VexRiscvBridge.get_arch())
        soc.add_config("CPU_MMU",   "sv32")

        # Constants for cache so we can add them in the DTS.
        if (VexRiscvBridge.dcache_size > 0):
            soc.add_config("CPU_DCACHE_SIZE", VexRiscvBridge.dcache_size)
            soc.add_config("CPU_DCACHE_WAYS", VexRiscvBridge.dcache_ways)
            soc.add_config("CPU_DCACHE_BLOCK_SIZE", 64) # hardwired?
        if (VexRiscvBridge.icache_size > 0):
            soc.add_config("CPU_ICACHE_SIZE", VexRiscvBridge.icache_size)
            soc.add_config("CPU_ICACHE_WAYS", VexRiscvBridge.icache_ways)
            soc.add_config("CPU_ICACHE_BLOCK_SIZE", 64) # hardwired?
        # Constants for TLB so we can add them in the DTS
        # full associative so only the size is described.
        if (VexRiscvBridge.dtlb_size > 0):
            soc.add_config("CPU_DTLB_SIZE", VexRiscvBridge.dtlb_size)
            soc.add_config("CPU_DTLB_WAYS", VexRiscvBridge.dtlb_size)
        if (VexRiscvBridge.itlb_size > 0):
            soc.add_config("CPU_ITLB_SIZE", VexRiscvBridge.itlb_size)
            soc.add_config("CPU_ITLB_WAYS", VexRiscvBridge.itlb_size)

        # Add PLIC as Bus Slave
        self.plicbus = plicbus  = wishbone.Interface(data_width=32, address_width=32, addressing="word")
        self.cpu_params.update(
            i_plicWishbone_CYC       = plicbus.cyc,
            i_plicWishbone_STB       = plicbus.stb,
            o_plicWishbone_ACK       = plicbus.ack,
            i_plicWishbone_WE        = plicbus.we,
            i_plicWishbone_ADR       = plicbus.adr,
            o_plicWishbone_DAT_MISO  = plicbus.dat_r,
            i_plicWishbone_DAT_MOSI  = plicbus.dat_w
        )
        soc.bus.add_slave("plic", self.plicbus, region=SoCRegion(origin=soc.mem_map.get("plic"), size=0x40_0000, cached=False))

        # Add CLINT as Bus Slave
        self.clintbus = clintbus = wishbone.Interface(data_width=32, address_width=32, addressing="word")
        self.cpu_params.update(
            i_clintWishbone_CYC      = clintbus.cyc,
            i_clintWishbone_STB      = clintbus.stb,
            o_clintWishbone_ACK      = clintbus.ack,
            i_clintWishbone_WE       = clintbus.we,
            i_clintWishbone_ADR      = clintbus.adr,
            o_clintWishbone_DAT_MISO = clintbus.dat_r,
            i_clintWishbone_DAT_MOSI = clintbus.dat_w,
        )
        soc.bus.add_slave("clint", clintbus, region=SoCRegion(origin=soc.mem_map.get("clint"), size=0x1_0000, cached=False))

    def add_memory_buses(self, address_width, data_width):
        VexRiscvBridge.litedram_width = data_width

        from litedram.common import LiteDRAMNativePort
        if(not VexRiscvBridge.wishbone_memory):
            ibus = LiteDRAMNativePort(mode="both", address_width=32, data_width=VexRiscvBridge.litedram_width)
            dbus = LiteDRAMNativePort(mode="both", address_width=32, data_width=VexRiscvBridge.litedram_width)
            self.memory_buses.append(ibus)
            self.memory_buses.append(dbus)
            self.cpu_params.update(
                # Instruction Memory Bus (Master).
                o_iBridge_dram_cmd_valid          = ibus.cmd.valid,
                i_iBridge_dram_cmd_ready          = ibus.cmd.ready,
                o_iBridge_dram_cmd_payload_we     = ibus.cmd.we,
                o_iBridge_dram_cmd_payload_addr   = ibus.cmd.addr,
                o_iBridge_dram_wdata_valid        = ibus.wdata.valid,
                i_iBridge_dram_wdata_ready        = ibus.wdata.ready,
                o_iBridge_dram_wdata_payload_data = ibus.wdata.data,
                o_iBridge_dram_wdata_payload_we   = ibus.wdata.we,
                i_iBridge_dram_rdata_valid        = ibus.rdata.valid,
                o_iBridge_dram_rdata_ready        = ibus.rdata.ready,
                i_iBridge_dram_rdata_payload_data = ibus.rdata.data,

                # Data Memory Bus (Master).
                o_dBridge_dram_cmd_valid          = dbus.cmd.valid,
                i_dBridge_dram_cmd_ready          = dbus.cmd.ready,
                o_dBridge_dram_cmd_payload_we     = dbus.cmd.we,
                o_dBridge_dram_cmd_payload_addr   = dbus.cmd.addr,
                o_dBridge_dram_wdata_valid        = dbus.wdata.valid,
                i_dBridge_dram_wdata_ready        = dbus.wdata.ready,
                o_dBridge_dram_wdata_payload_data = dbus.wdata.data,
                o_dBridge_dram_wdata_payload_we   = dbus.wdata.we,
                i_dBridge_dram_rdata_valid        = dbus.rdata.valid,
                o_dBridge_dram_rdata_ready        = dbus.rdata.ready,
                i_dBridge_dram_rdata_payload_data = dbus.rdata.data,
            )

    def do_finalize(self):
        assert hasattr(self, "reset_address")

        # When no Direct Memory Bus, do memory accesses through Wishbone Peripheral Bus.
        if len(self.memory_buses) == 0:
            if VexRiscvBridge.with_fpu and (not VexRiscvBridge.wishbone_memory and not VexRiscvBridge.wishbone_force_32b):
                raise ValueError("No Direct Memory Bus found, please add --with-wishbone-memory or --wishbone-force-32b to your build command.")
            else:
                VexRiscvBridge.wishbone_memory = True

        # Generate cluster name.
        VexRiscvBridge.generate_cluster_name()

        # Do verilog instance.
        self.specials += Instance(self.cluster_name, **self.cpu_params)

        # Add verilog sources
        self.add_sources(self.platform)

