package leesum.Core
import chisel3._
import chisel3.util.Valid
import leesum.Cache.{DCacheConnect, DCacheReq, DCacheResp, ICacheTop}
import leesum.ICache.DCacheTop
import leesum._
import leesum.axi4.{AXIMasterIO, AxiReadArbiter}
import leesum.dbg.DebugModuleCoreInterface
import leesum.fronten.IFUTop
import leesum.lsu.LSUTop
import leesum.mmu_sv39.MMU
import leesum.moniter.{DifftestPort, MonitorTop, PerfPort}
import leesum.muldiv.FuMulDiv

// create FishCore config
case class FishCoreConfig(
    muldiv_en: Boolean = true,
    rvc_en: Boolean = false,
    monitor_en: Boolean = true,
    boot_pc: Long = 0x80000000L,
    addr_map: Seq[(Long, Long, Boolean)] = Seq(
      (
        0,
        0xffffffffffffffffL,
        false
      ) // default addr map, 0x0 -> 0xffffffffffffffff, mmio = false
    ),
    reorder_buffer_size: Int = 8,
    itlb_size: Int = 8,
    dtlb_size: Int = 8,
    btb_ways: Int = 2,
    btb_nums: Int = 128,
    bim_nums: Int = 128
)

class FishCore(
    config: FishCoreConfig
) extends Module {

  val io = IO(new Bundle {
    val difftest = Output(Valid(new DifftestPort(2)))
    val perf_monitor = Output(new PerfPort)
    val axi_master = new AXIMasterIO(32, 64)
    val mtime = Input(UInt(64.W))
    val time_int = Input(Bool())
    val soft_int = Input(Bool())
    val mext_int = Input(Bool())
    val sext_int = Input(Bool())

    // debug
    val debug_core_interface = Flipped(new DebugModuleCoreInterface)
    val tohost_addr = Input(Valid(UInt(64.W))) // from  env  elf loader
  })

  // monitor
  val monitor = Module(new MonitorTop(2))
  io.difftest <> monitor.io.difftest
  io.perf_monitor <> monitor.io.perf

  // pipeline stage
  val ifu = Module(
    new IFUTop(
      config.boot_pc,
      config.rvc_en,
      btb_way_count = config.btb_ways,
      btb_nums = config.btb_nums,
      bim_nums = config.bim_nums
    )
  )
  val mmu = Module(new MMU(config.addr_map, config.itlb_size, config.dtlb_size))

  mmu.io.tohost_addr := io.tohost_addr

  val decode_stage = Seq.tabulate(2)(i => Module(new InstDecoder))

  val rob = Module(new ReOrderBuffer(16, 2, 2))
  val issue_stage_rob = Module(new IssueStageNew(2, 2))

  val commit_stage = Module(new CommitStage(2, 2, config.monitor_en))
  val reg_file = Module(new GPRs(3, 3, config.monitor_en))
  val csr_regs = Module(new CSRRegs(2, 2))

  // fu
  val alu_seq = Seq.fill(2)(Module(new FuAlu))
  val lsu = Module(new LSUTop())
  val bru = Module(new FuBranch(config.rvc_en))
  val mul_div = Module(new FuMulDiv(config.muldiv_en))
  val csr = Module(new FuCSR)

  val dcache_arb = Module(
    new ReqRespArbiter(4, new DCacheReq, new DCacheResp)
  )
  dcache_arb.io.flush := false.B

  val dcache = Module(new DCacheTop)
  val icache_top = Module(new ICacheTop)

  dcache_arb.io.req_arb <> dcache.io.req
  dcache_arb.io.resp_arb <> dcache.io.resp

  val axi_r_arb = Module(new AxiReadArbiter)

  axi_r_arb.io.in.foreach(axi => {
    axi.aw.noenq()
    axi.w.noenq()
    axi.b.nodeq()
  })

  axi_r_arb.io.out.aw.nodeq()
  axi_r_arb.io.out.w.nodeq()
  axi_r_arb.io.out.b.noenq()

  icache_top.io.mem_master.ar <> axi_r_arb.io.in(1).ar
  icache_top.io.mem_master.r <> axi_r_arb.io.in(1).r
  icache_top.io.mem_master.aw.nodeq()
  icache_top.io.mem_master.w.nodeq()
  icache_top.io.mem_master.b.noenq()

  dcache.io.mem_master.ar <> axi_r_arb.io.in(0).ar
  dcache.io.mem_master.r <> axi_r_arb.io.in(0).r

  io.axi_master.ar <> axi_r_arb.io.out.ar
  io.axi_master.r <> axi_r_arb.io.out.r
  io.axi_master.aw <> dcache.io.mem_master.aw
  io.axi_master.w <> dcache.io.mem_master.w
  io.axi_master.b <> dcache.io.mem_master.b

  // flush
  ifu.io.flush := commit_stage.io.flush
  issue_stage_rob.io.flush := commit_stage.io.flush
  rob.io.flush := commit_stage.io.flush
  lsu.io.flush := commit_stage.io.flush
  mmu.io.flush := commit_stage.io.flush
  dcache.io.flush := commit_stage.io.flush
  icache_top.io.flush := commit_stage.io.flush
  bru.io.flush := commit_stage.io.flush
  mul_div.io.flush := commit_stage.io.flush
  csr.io.flush := commit_stage.io.flush

  // fencei
  icache_top.io.fencei := commit_stage.io.icache_fencei
  dcache.io.fencei := commit_stage.io.dcache_fencei
  dcache.io.fencei_ack <> commit_stage.io.dcache_fencei_ack
  mmu.io.tlb_flush := commit_stage.io.tlb_flush

  // ifu <> icache
  ifu.io.icache_req <> icache_top.io.req
  ifu.io.icache_resp <> icache_top.io.resp

  // mmu <> icache
  mmu.io.fetch_req <> icache_top.io.mmu_req
  mmu.io.fetch_resp <> icache_top.io.mmu_resp

  // monitor <> icache
  monitor.io.perf_icache := icache_top.io.perf_icache
  monitor.io.perf_dcache := dcache.io.perf_dcache

  // mmu <> lsu
  mmu.io.lsu_req <> lsu.io.tlb_req
  mmu.io.lsu_resp <> lsu.io.tlb_resp

  // mmu <> csr
  mmu.io.satp := csr_regs.io.direct_read_ports.satp
  mmu.io.mstatus := csr_regs.io.direct_read_ports.mstatus

  // mmu <> commit stage
  mmu.io.cur_privilege := commit_stage.io.cur_privilege_mode

  // mmu <> dcache

  DCacheConnect.load_req_to_dcache_req(
    mmu.io.dcache_load_req,
    dcache_arb.io.req_vec(0)
  )
  DCacheConnect.dcache_resp_to_load_resp(
    dcache_arb.io.resp_vec(0),
    mmu.io.dcache_load_resp
  )

  // mmu <> monitor
  monitor.io.perf_dtlb := mmu.io.perf_dtlb
  monitor.io.perf_itlb := mmu.io.perf_itlb

  // ifu <> monitor
  monitor.io.perf_bp_f1 := ifu.io.perf_bp_f1

  // ifu <> decode stage

  for (i <- decode_stage.indices) {

    ifu.io.inst_fifo_pop(i).ready := decode_stage(i).io.in.ready
    decode_stage(i).io.in.valid := ifu.io.inst_fifo_pop(i).valid
    decode_stage(i).io.in.bits.inst := ifu.io.inst_fifo_pop(i).bits.inst
    decode_stage(i).io.in.bits.pc := ifu.io.inst_fifo_pop(i).bits.pc
    decode_stage(i).io.in.bits.is_rvc := ifu.io.inst_fifo_pop(i).bits.rvc
    decode_stage(i).io.in.bits.is_valid := ifu.io.inst_fifo_pop(i).bits.valid
    decode_stage(i).io.in.bits.inst_c := ifu.io.inst_fifo_pop(i).bits.inst_c
    decode_stage(i).io.in.bits.exception := ifu.io
      .inst_fifo_pop(i)
      .bits
      .exception
    decode_stage(i).io.in.bits.bp := ifu.io.inst_fifo_pop(i).bits.bp
  }

  // decode stage <> issue stage
  for (i <- decode_stage.indices) {
    decode_stage(i).io.out <> issue_stage_rob.io.push_port(i)
  }

  // issue stage <> scoreboard
  issue_stage_rob.io.pop_port <> rob.io.push_ports
  issue_stage_rob.io.alloc_trans_id <> rob.io.push_trans_id
  issue_stage_rob.io.operand_bypass_resp := rob.io.operand_bypass_resp
  rob.io.operand_bypass_req := issue_stage_rob.io.operand_bypass_req

  // issue stage <> fu
  require(issue_stage_rob.io.fu_port.alu.length == alu_seq.length)
  for (i <- alu_seq.indices) {
    issue_stage_rob.io.fu_port.alu(i) <> alu_seq(i).io.in
  }
  issue_stage_rob.io.fu_port.lsu_0 <> lsu.io.lsu_req
  issue_stage_rob.io.fu_port.branch_0 <> bru.io.in
  issue_stage_rob.io.fu_port.mul_0 <> mul_div.io.mul_req
  issue_stage_rob.io.fu_port.div_0 <> mul_div.io.div_req
  issue_stage_rob.io.fu_port.csr_0 <> csr.io.csr_req

  // csr <> csr_regs
  csr.io.csr_read_port <> csr_regs.io.read_ports(0)
  csr.io.csr_write_port <> csr_regs.io.write_ports(0)

  // issue stage <> reg file
  issue_stage_rob.io.gpr_read_port <> reg_file.io.read_ports.slice(0, 2)

  // rob <> fu
  require(rob.io.fu_alu_wb_port.length == alu_seq.length)
  for (i <- alu_seq.indices) {
    rob.io.fu_alu_wb_port(i) <> alu_seq(i).io.out
  }
  rob.io.fu_lsu_wb_port <> lsu.io.lsu_resp
  rob.io.fu_lsu_agu_wb_port <> lsu.io.agu_writeback
  rob.io.fu_branch_wb_port <> bru.io.out
  rob.io.fu_mul_div_wb_port <> mul_div.io.fu_div_mul_resp
  rob.io.fu_csr_wb_port <> csr.io.csr_resp
//  rob.io.fu_amo_wb_port <> lsu.io.amo_writeback

  // scoreboard <> commit stage
  rob.io.pop_ports <> commit_stage.io.rob_commit_ports

  // commit stage <> reg file
  commit_stage.io.gpr_commit_ports <> reg_file.io.write_ports.slice(0, 2)
  // commit stage <> csr file
  commit_stage.io.direct_read_ports <> csr_regs.io.direct_read_ports
  commit_stage.io.direct_write_ports <> csr_regs.io.direct_write_ports
  commit_stage.io.cur_privilege_mode <> csr_regs.io.cur_privilege_mode

  // commit stage <> fu
  commit_stage.io.store_commit <> lsu.io.store_commit
  commit_stage.io.mmio_commit <> lsu.io.mmio_commit
  commit_stage.io.amo_commit <> lsu.io.amo_commit
  commit_stage.io.store_queue_empty := lsu.io.store_queue_empty

  commit_stage.io.csr_commit <> csr.io.csr_commit

  // commit stage <> ifu
  ifu.io.commit_redirect_pc <> commit_stage.io.commit_redirect_pc

  ifu.io.cmt_update_pc := commit_stage.io.cmt_update_pc
  ifu.io.cmt_update_bim_en := commit_stage.io.cmt_update_bim_en
  ifu.io.cmt_update_btb_en := commit_stage.io.cmt_update_btb_en
  ifu.io.cmt_update_bim_data := commit_stage.io.cmt_update_bim_data
  ifu.io.cmt_update_btb_data := commit_stage.io.cmt_update_btb_data
  ifu.io.cmt_update_btb_way_sel := commit_stage.io.cmt_update_btb_way_sel

  // commit stage <> monitor
  commit_stage.io.commit_monitor.get <> monitor.io.commit_monitor
  commit_stage.io.perf_commit <> monitor.io.perf_commit
  commit_stage.io.perf_bp <> monitor.io.perf_bp

  // regfile <> monitor
  reg_file.io.gpr_monitor.get <> monitor.io.gpr_monitor

  // csr <> monitor
  csr_regs.io.direct_read_ports <> monitor.io.csr_monitor

  // csr <> core
  csr_regs.io.mtime := io.mtime
  csr_regs.io.time_int := io.time_int
  csr_regs.io.soft_int := io.soft_int
  csr_regs.io.mext_int := io.mext_int
  csr_regs.io.sext_int := io.sext_int

  // lsu <> dcache

  DCacheConnect.store_req_to_dcache_req(
    lsu.io.dcache_store_req,
    dcache_arb.io.req_vec(1)
  )
  DCacheConnect.dcache_resp_to_store_resp(
    dcache_arb.io.resp_vec(1),
    lsu.io.dcache_store_resp
  )

  DCacheConnect.load_req_to_dcache_req(
    lsu.io.dcache_load_req,
    dcache_arb.io.req_vec(2)
  )
  DCacheConnect.dcache_resp_to_load_resp(
    dcache_arb.io.resp_vec(2),
    lsu.io.dcache_load_resp
  )

  // debug port
  io.debug_core_interface.state_regs := commit_stage.io.debug_state_regs
  commit_stage.io.debug_halt_req := io.debug_core_interface.halt_req
  commit_stage.io.debug_resume_req := io.debug_core_interface.resume_req

  ifu.io.halted := commit_stage.io.debug_state_regs.is_halted

  io.debug_core_interface.gpr_read_port <> reg_file.io.read_ports.last
  io.debug_core_interface.gpr_write_port <> reg_file.io.write_ports.last
  io.debug_core_interface.csr_read_port <> csr_regs.io.read_ports.last
  io.debug_core_interface.csr_write_port <> csr_regs.io.write_ports.last
  io.debug_core_interface.dcache_req <> dcache_arb.io.req_vec(3)
  io.debug_core_interface.dcache_resp <> dcache_arb.io.resp_vec(3)
}

object gen_FishCore_verilog extends App {
  GenVerilogHelper(new FishCore(FishCoreConfig()))
}
