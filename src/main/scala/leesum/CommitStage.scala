package leesum
import chisel3._
import chisel3.util._
import leesum.Utils.SimLog
import leesum.bpu.BTBEntry
import leesum.dbg.DbgSlaveState
import leesum.mmu_sv39.SfenceVMABundle
import leesum.moniter.{CommitMonitorPort, PerfMonitorCounter}

class RedirectPC extends Bundle {
  val valid = Bool()
  val target = UInt(64.W)
}

class CommitStage(
    num_commit_port: Int,
    btb_way_count: Int = 2,
    monitor_en: Boolean = false
) extends Module {
  val io = IO(new Bundle {

    val rob_commit_ports =
      Vec(num_commit_port, Flipped(Decoupled(new ScoreBoardEntry)))
    val flush = Output(Bool())
    val icache_fencei =
      Output(Bool()) // when icache_fencei is true, flush must be true

    val dcache_fencei = Output(Bool())
    val dcache_fencei_ack = Input(Bool())

    // sfence.vma
    val tlb_flush = Output(Valid(new SfenceVMABundle))

    // gpr
    val gpr_commit_ports =
      Vec(num_commit_port, Flipped(new GPRsWritePort))
    // lsu
    val mmio_commit = Decoupled(Bool())
    val store_commit = Decoupled(Bool())
    val amo_commit = Decoupled(Bool())
    val store_queue_empty = Input(Bool())
    // csr
    val csr_commit = Decoupled(Bool())
    // branch
    val commit_redirect_pc = Output(new RedirectPC())
    // bpu commit update port
    val cmt_update_pc = Output(UInt(39.W))
    val cmt_update_btb_en =
      Output(Bool()) // btb only update when branch predict fail
    val cmt_update_bim_en = Output(Bool()) // bim update for all branch
    val cmt_update_btb_data = Output(new BTBEntry())
    val cmt_update_bim_data = Output(UInt(2.W))
    val cmt_update_btb_way_sel = Output(UInt(log2Ceil(btb_way_count).W))

    // csr regfile direct access
    val direct_read_ports = Input(new CSRDirectReadPorts)
    val direct_write_ports = Output(new CSRDirectWritePorts)

    // privilege mode
    val cur_privilege_mode = Output(UInt(2.W))

    // debug
    val debug_state_regs = Output(new DbgSlaveState())
    val debug_halt_req = Input(ValidIO(Bool()))
    val debug_resume_req = Input(ValidIO(Bool()))

    // performance monitor
    val perf_commit = Output(new PerfMonitorCounter)
    val perf_bp = Output(new PerfMonitorCounter)

    val commit_monitor = if (monitor_en) {
      Some(Output(Vec(num_commit_port, Valid(new CommitMonitorPort))))
    } else {
      None
    }
  })

  // -----------------------
  // performance monitor
  // -----------------------
  val commit_perf = RegInit(0.U.asTypeOf(new PerfMonitorCounter))
  val bp_perf = RegInit(0.U.asTypeOf(new PerfMonitorCounter))

  io.perf_commit := commit_perf
  io.perf_bp := bp_perf

  val rob_valid_seq = io.rob_commit_ports.map(_.valid)
  val rob_data_seq = io.rob_commit_ports.map(_.bits)
  val pop_ack = WireInit(VecInit(Seq.fill(num_commit_port)(false.B)))

  val flush_next = RegInit(false.B)
  val dfencei_next = RegInit(false.B)
  val ifencei_next = RegInit(false.B)
  val sfencevma_next = RegInit(false.B)
  val sfence_buf = RegInit(0.U.asTypeOf(new SfenceVMABundle))
  val redirect_next = RegInit(0.U.asTypeOf(new RedirectPC))
  val cmt_update_btb_en_next = RegInit(false.B)
  val cmt_update_bim_en_next = RegInit(false.B)
  val cmt_update_pc_next = RegInit(0.U(39.W))
  val cmt_update_btb_data_next = RegInit(0.U.asTypeOf(new BTBEntry()))
  val cmt_update_bim_data_next = RegInit(0.U(2.W))
  val cmt_update_btb_way_sel_next = RegInit(0.U(log2Ceil(btb_way_count).W))

  val privilege_mode = RegInit(3.U(2.W)) // machine mode

  val debug_state_regs = RegInit(0.U.asTypeOf(new DbgSlaveState()));

  io.debug_state_regs := debug_state_regs

  when(io.debug_halt_req.valid) {
    SimLog(desiredName, "halt req %d\n", io.debug_halt_req.bits)
    debug_state_regs.set_haltreq(io.debug_halt_req.bits)
  }

  when(io.debug_resume_req.valid) {
    SimLog(desiredName, "resume req\n")
    debug_state_regs.set_resumereq()
  }

  // interrupt
  val interrupt_inject_types = VecInit(
    Seq(
      FuType.Alu.asUInt,
      FuType.Mul.asUInt,
      FuType.Div.asUInt
//      FuType.Lsu.asUInt,
//      FuType.Br.asUInt
//      FuType.Csr.asUInt
    )
  )
  val has_exception = WireInit(false.B)
  val has_interrupt = WireInit(false.B)
  val interrupt_cause = Wire(ExceptionCause())
  val exception_cause = Wire(ExceptionCause())
  interrupt_cause := ExceptionCause.unknown
  exception_cause := ExceptionCause.unknown
  dontTouch(has_interrupt)
  dontTouch(has_exception)
  dontTouch(interrupt_cause)
  dontTouch(exception_cause)

  // when csr has side effect, flush the pipeline and rerun the current inst
  val csr_side_effect = RegInit(
    false.B
  )
  val csr_diff_skip = WireInit(false.B)

  io.cur_privilege_mode := privilege_mode

  when(cmt_update_bim_en_next) {
    cmt_update_bim_en_next := false.B
  }

  when(cmt_update_btb_en_next) {
    cmt_update_btb_en_next := false.B
  }

  when(flush_next) {
    flush_next := false.B
  }

  when(ifencei_next) {
    ifencei_next := false.B
  }
  when(sfencevma_next) {
    sfencevma_next := false.B
  }

  when(redirect_next.valid) {
    redirect_next.valid := false.B
  }

  io.flush := flush_next
  io.dcache_fencei := dfencei_next
  io.icache_fencei := ifencei_next
  io.tlb_flush.valid := sfencevma_next
  io.tlb_flush.bits := sfence_buf
  io.commit_redirect_pc := redirect_next

  io.cmt_update_pc := cmt_update_pc_next
  io.cmt_update_btb_en := cmt_update_btb_en_next
  io.cmt_update_bim_en := cmt_update_bim_en_next
  io.cmt_update_btb_data := cmt_update_btb_data_next
  io.cmt_update_bim_data := cmt_update_bim_data_next
  io.cmt_update_btb_way_sel := cmt_update_btb_way_sel_next

  assert(
    CheckOrder(pop_ack),
    "rob_commit_ports must be ordered"
  )

  def retire_alu(
      entry: ScoreBoardEntry,
      gpr_commit_port: GPRsWritePort,
      ack: Bool
  ): Unit = {
    val alu_mul_div_op =
      FuOP.is_alu(entry.fu_op) || FuOP.is_mul(entry.fu_op) || FuOP.is_div_rem(
        entry.fu_op
      )

    assert(alu_mul_div_op, "fu_op must be alu, mul or div")
    assert(entry.exception.valid === false.B)
    when(alu_mul_div_op) {
      gpr_commit_port.write(entry.rd_addr, entry.result)
      ack := true.B
    }
  }

  def retire_load(
      entry: ScoreBoardEntry,
      gpr_commit_port: GPRsWritePort,
      ack: Bool
  ): Unit = {
    assert(FuOP.is_lsu(entry.fu_op), "fu_op must be lsu")
    assert(entry.exception.valid === false.B)
    when(FuOP.is_load(entry.fu_op)) {
      assert(entry.complete === true.B, "load must be complete")
      gpr_commit_port.write(entry.rd_addr, entry.result)
      ack := true.B
    }
  }

  def retire_store(
      entry: ScoreBoardEntry,
      store_commit: DecoupledIO[Bool],
      ack: Bool
  ): Unit = {
    assert(FuOP.is_lsu(entry.fu_op), "fu_op must be lsu")
    assert(entry.exception.valid === false.B)
    assert(entry.complete === true.B, "store must be complete")
    when(FuOP.is_store(entry.fu_op)) {
      // valid rely on store_commit.ready
      store_commit.valid := true.B
      store_commit.bits := true.B
      when(store_commit.fire) {
        ack := true.B
      }
    }
  }

  def retire_branch(
      entry: ScoreBoardEntry,
      gpr_commit_port: GPRsWritePort,
      ack: Bool
  ): Unit = {
    assert(
      FuOP.is_branch(entry.fu_op) || FuOP.is_jalr(entry.fu_op) || FuOP.is_jal(
        entry.fu_op
      ),
      "fu_op must be branch"
    )
    assert(entry.exception.valid === false.B)
    val mis_predict = entry.bp.is_miss_predict
    val need_write_back = FuOP.is_jalr(entry.fu_op) || FuOP.is_jal(entry.fu_op)

    ack := true.B
    when(need_write_back) {
      gpr_commit_port.write(entry.rd_addr, entry.result)
    }

    val crossline_jump_inst = entry.pc(2, 0) === 6.U && !entry.is_rvc

    dontTouch(crossline_jump_inst)

    // if crossline_jump_inst is true, we use pc + 2 as the next pc
    val cmt_update_pc = Mux(
      crossline_jump_inst,
      entry.pc + 2.U,
      entry.pc
    )

    cmt_update_pc_next := cmt_update_pc

    // only update btb when branch predict fail
    cmt_update_btb_en_next := mis_predict
    cmt_update_btb_way_sel_next := entry.bp.sel_way

    // new btb data
    cmt_update_btb_data_next.target_pc := entry.bp.predict_pc
    cmt_update_btb_data_next.is_rvc := entry.is_rvc
    cmt_update_btb_data_next.bp_type := entry.bp.bp_type
    cmt_update_btb_data_next.offset := cmt_update_pc(2, 0)

    // always update bim
    cmt_update_bim_en_next := true.B
    cmt_update_bim_data_next := entry.bp.next_bim_value

    when(mis_predict) {
//      io.branch_commit.valid := true.B
//      io.branch_commit.bits.target := entry.bp.predict_pc

      redirect_next.valid := true.B
      redirect_next.target := entry.bp.predict_pc

      flush_next := true.B
      bp_perf.inc_miss(1.U)
    }.otherwise {
      bp_perf.inc_hit(1.U)
    }

  }

  def retire_none(entry: ScoreBoardEntry, ack: Bool): Unit = {
    assert(entry.exception.valid === false.B)

    val next_inst_pc = entry.pc + Mux(
      entry.is_rvc,
      2.U,
      4.U
    )

    // TODO: unimplemented
    when(entry.fu_op === FuOP.SFenceVMA) {
      val mstatus = new MstatusFiled(io.direct_read_ports.mstatus)
      val require_privi =
        Mux(mstatus.tvm, Privilegelevel.M.U, Privilegelevel.S.U)
//      printf("SFenceVMA at %x\n", entry.pc)

      when(privilege_mode >= require_privi) {
        // privi check pass
        flush_next := true.B
        sfencevma_next := true.B
        // TODO: unimplemented rs1 rs2
        sfence_buf.va := 0.U
        sfence_buf.asid := 0.U

        redirect_next.valid := true.B
        redirect_next.target := next_inst_pc

      }.otherwise {
        // privi check fail
        val vma_entry = WireInit(entry)
        vma_entry.exception.valid := true.B
        vma_entry.exception.cause := ExceptionCause.illegal_instruction
        vma_entry.exception.tval := vma_entry.inst // TODO: set tval 0

        retire_exception(vma_entry, ack)
      }
      ack := true.B

    }.elsewhen(entry.fu_op === FuOP.Fence) {
//      printf("Fence at %x\n", entry.pc)
      ack := true.B
    }.elsewhen(entry.fu_op === FuOP.FenceI) {
      dfencei_next := io.store_queue_empty

      when(dfencei_next) {
        assert(io.store_queue_empty, "store queue must be empty when fencei")
      }

      // clear icache after clear dcache
      when(io.dcache_fencei_ack && io.dcache_fencei) {
        flush_next := true.B
        ifencei_next := true.B
        dfencei_next := false.B
        ack := true.B

        redirect_next.valid := true.B
        redirect_next.target := next_inst_pc

      }
    }.elsewhen(entry.fu_op === FuOP.WFI) {
      // printf("WFI at %x\n", entry.pc)
      ack := true.B
    }

  }

  // TODO: exception
  def retire_exception(entry: ScoreBoardEntry, ack: Bool): Unit = {
    assert(entry.exception.valid === true.B)

    val mstatus = new MstatusFiled(io.direct_read_ports.mstatus)
    val medeleg = io.direct_read_ports.medeleg
    exception_cause := Mux(
      entry.fu_op === FuOP.Ecall,
      ExceptionCause.get_call_cause(privilege_mode),
      entry.exception.cause
    )

    val trap_to_smode = medeleg(
      exception_cause.asUInt(5, 0)
    ) && privilege_mode <= Privilegelevel.S.U

    has_exception := true.B
//    io.branch_commit.valid := true.B

    redirect_next.valid := true.B

    ack := true.B
    flush_next := true.B
    privilege_mode := Mux(
      trap_to_smode,
      Privilegelevel.S.U,
      Privilegelevel.M.U
    )

    when(trap_to_smode) {
      // s mode
      val stvec = new MtvecFiled(io.direct_read_ports.stvec)
      redirect_next.target := stvec.get_exception_pc

      io.direct_write_ports.mstatus.valid := true.B
      io.direct_write_ports.mstatus.bits := mstatus
        .get_smode_exception_mstatus(
          privilege_mode
        )
      io.direct_write_ports.sepc.valid := true.B
      io.direct_write_ports.sepc.bits := entry.pc
      io.direct_write_ports.scause.valid := true.B
      io.direct_write_ports.scause.bits := exception_cause.asUInt
      io.direct_write_ports.stval.valid := true.B
      io.direct_write_ports.stval.bits := entry.exception.tval

      SimLog(
        desiredName,
        "S Mode Exception at [%x], cause[%d], tval[%x]\n",
        entry.pc,
        exception_cause.asUInt,
        entry.exception.tval
      )

    }.otherwise {
      // m mode
      val mtvec = new MtvecFiled(io.direct_read_ports.mtvec)
//      io.branch_commit.valid := true.B
//      io.branch_commit.bits.target := mtvec.get_exception_pc

      redirect_next.valid := true.B
      redirect_next.target := mtvec.get_exception_pc

      io.direct_write_ports.mepc.valid := true.B
      io.direct_write_ports.mepc.bits := entry.pc
      io.direct_write_ports.mcause.valid := true.B
      io.direct_write_ports.mcause.bits := exception_cause.asUInt
      io.direct_write_ports.mtval.valid := true.B
      io.direct_write_ports.mtval.bits := entry.exception.tval
      io.direct_write_ports.mstatus.valid := true.B
      io.direct_write_ports.mstatus.bits := mstatus
        .get_mmode_exception_mstatus(
          privilege_mode
        )

      SimLog(
        desiredName,
        "M Mode Exception at pc[%x], cause[%d], tval[%x]\n",
        entry.pc,
        exception_cause.asUInt,
        entry.exception.tval
      )
    }

    // ------------------
    // assert
    // ------------------

    assert(
      entry.complete,
      "exception must be complete"
    )
  }

  def retire_mret(entry: ScoreBoardEntry, ack: Bool): Unit = {
    assert(entry.fu_op === FuOP.Mret, "fu_op must be mret")
    assert(entry.exception.valid === false.B)
    assert(entry.complete === true.B, "mret must be complete")

    val mstatus = new MstatusFiled(io.direct_read_ports.mstatus)
    val mepc = io.direct_read_ports.mepc

    // supposing xPP holds the value y
    val y = mstatus.mpp
    // the privilege mode is changed to xPP
    privilege_mode := y

    // (If y!=M, x RET also sets MPRV=0.)
    // reference to  https://github.com/riscv/riscv-isa-manual/pull/929
    val new_mstatus = mstatus.get_mret_mstatus(y =/= Privilegelevel.M.U(2.W))

//    io.branch_commit.valid := true.B
//    io.branch_commit.bits.target := mepc

    redirect_next.valid := true.B
    redirect_next.target := mepc

    io.direct_write_ports.mstatus.valid := true.B
    io.direct_write_ports.mstatus.bits := new_mstatus
    flush_next := true.B
    ack := true.B
  }

  def retire_sret(entry: ScoreBoardEntry, ack: Bool): Unit = {
    assert(entry.fu_op === FuOP.Sret, "fu_op must be sret")
    assert(entry.exception.valid === false.B)
    assert(entry.complete === true.B, "sret must be complete")

    val sstatus = new MstatusFiled(io.direct_read_ports.mstatus)
    val sepc = io.direct_read_ports.sepc

    // SRET should also raise an illegal instruction exception when TSR=1 in mstatus
    when(sstatus.tsr) {
      val sret_entry = WireInit(entry)
      sret_entry.exception.valid := true.B
      sret_entry.exception.cause := ExceptionCause.illegal_instruction
      sret_entry.exception.tval := entry.inst // TODO: set tval 0
      retire_exception(sret_entry, ack)
    }.otherwise {
      // supposing xPP holds the value y, 0: user mode 1: s mode
      val y = Mux(sstatus.spp, Privilegelevel.S.U(2.W), Privilegelevel.U.U(2.W))
      // the privilege mode is changed to xPP
      privilege_mode := y

      // (If y!=M, x RET also sets MPRV=0.)
      // reference to  https://github.com/riscv/riscv-isa-manual/pull/929
      val new_mstatus = sstatus.get_sret_mstatus(y =/= Privilegelevel.M.U(2.W))

//      io.branch_commit.valid := true.B
//      io.branch_commit.bits.target := sepc

      redirect_next.valid := true.B
      redirect_next.target := sepc

      io.direct_write_ports.mstatus.valid := true.B
      io.direct_write_ports.mstatus.bits := new_mstatus
      ack := true.B
      flush_next := true.B

    }

  }

  def retire_csr(
      entry: ScoreBoardEntry,
      gpr_commit_port: GPRsWritePort,
      csr_commit_port: DecoupledIO[Bool],
      ack: Bool
  ): Unit = {
    assert(FuOP.is_csr(entry.fu_op), "fu_op must be csr")
    assert(entry.exception.valid === false.B)

    val csr_addr = entry.inst(31, 20)

    val side_effect_csrs = Seq(
      CSRs.satp.asUInt
//      CSRs.mstatus.asUInt,
//      CSRs.sstatus.asUInt
    )

    val counter_csrs = Seq(
      CSRs.mcycle.asUInt,
      CSRs.minstret.asUInt,
      CSRs.mcycleh.asUInt,
      CSRs.minstreth.asUInt,
      CSRs.cycle.asUInt,
      CSRs.instret.asUInt,
      CSRs.cycleh.asUInt,
      CSRs.instreth.asUInt,
      CSRs.time.asUInt,
      CSRs.timeh.asUInt
    )

    val csr_next_pc = RegInit(0.U(64.W))

    // TODO: only write csr with side effect
    val has_side_effect = VecInit(side_effect_csrs).contains(csr_addr)
    val read_rdtime = VecInit(counter_csrs).contains(csr_addr)

    val sIdle :: sACK :: Nil = Enum(2)
    val state = RegInit(sIdle)
    state.suggestName("csr_state")

    switch(state) {
      is(sIdle) {
        assert(entry.complete === false.B, "csr must be not complete")
        csr_commit_port.bits := true.B
        csr_commit_port.valid := true.B
        when(csr_commit_port.fire) {
          csr_next_pc := entry.pc + 4.U
          state := sACK
        }
      }
      is(sACK) {
        assert(entry.complete === true.B, "csr must be complete")
        gpr_commit_port.write(entry.rd_addr, entry.result)

        ack := true.B
        csr_diff_skip := read_rdtime
        when(has_side_effect) {
//          printf("csr side effect at %x\n", entry.pc)
          csr_side_effect := true.B
        }
        state := sIdle
      }
    }
  }

  def retire_amo(
      entry: ScoreBoardEntry,
      gpr_commit_port: GPRsWritePort,
      amo_commit_port: DecoupledIO[Bool],
      ack: Bool
  ): Unit = {
    assert(entry.exception.valid === false.B)
    assert(FuOP.is_lsu(entry.fu_op), "fu_op must be lsu")
    when(FuOP.is_atomic(entry.fu_op)) {
      assert(entry.complete === true.B, "load must be complete")
      gpr_commit_port.write(entry.rd_addr, entry.result)
      ack := true.B
    }
  }

  io.mmio_commit.noenq()
  io.store_commit.noenq()
  io.amo_commit.noenq()
  io.csr_commit.noenq()
  io.gpr_commit_ports.foreach(gpr => {
    gpr.addr := 0.U
    gpr.wdata := 0.U
    gpr.wen := false.B
  })
  io.rob_commit_ports.foreach(_.nodeq())

  io.rob_commit_ports.zip(pop_ack).foreach { case (port, ack) =>
    port.ready := ack
  }

  io.direct_write_ports.clear()

  // -----------------------
  // retire logic
  // -----------------------

  // -------------- enter debug mode start----------------

  val debug_lock = RegInit(false.B)
  debug_lock := false.B

  val dcsr_field = new DCSRFiled(io.direct_read_ports.dcsr)
  val is_haltreq = debug_state_regs.haltreq_signal
  // 只有单步执行的指令执行完毕后，才能重新进入 debug 模式
  val is_stepreq = debug_state_regs.stepi_enter_debug_mode()

  val is_ebreakreq =
    rob_data_seq.head.fu_op === FuOP.Ebreak && rob_valid_seq.head && Seq(
      dcsr_field.ebreakm && privilege_mode === Privilegelevel.M.U,
      dcsr_field.ebreaks && privilege_mode === Privilegelevel.S.U,
      dcsr_field.ebreaku && privilege_mode === Privilegelevel.U.U
    ).reduce((_: Bool) || (_: Bool))

  val debug_cause = MuxCase(
    DebugCause.no_debug,
    Seq(
      is_haltreq -> DebugCause.halt_req,
      is_stepreq -> DebugCause.step,
      is_ebreakreq -> DebugCause.ebreak
    )
  )
  val debug_newpc = rob_data_seq.head.pc

  // 只有当不在debug模式下，且有debug_cause，且rob中有有效指令时，进入debug模式

  val will_enter_debug_mode = Seq(
    !debug_state_regs.halted(),
    debug_cause =/= DebugCause.no_debug,
    rob_valid_seq.head,
    !flush_next,
    !debug_lock
  ).reduce(_ && _)

  val sDebugIdle :: sEnterDebug :: sFlushCache :: Nil = Enum(3)
  val debug_state = RegInit(sDebugIdle)
  val debug_cause_buf = RegInit(DebugCause.no_debug)
  val debug_newpc_buf = RegInit(0.U(64.W))

  when(will_enter_debug_mode) {

    // 1. 首先检测是否需要进入 debug 模式
    // 如果上一条指令有 csr side effect，那么需要 flush (进入 debug 模式就已经 flush 了)
//    enter_debug_mode(debug_cause, debug_newpc)

    switch(debug_state) {
      is(sDebugIdle) {
        SimLog(
          desiredName,
          "Will enter DebugMode at PC:%x, Cause:%d\n",
          debug_newpc,
          debug_cause.asUInt
        )

        debug_state := sEnterDebug
        debug_cause_buf := debug_cause
        debug_newpc_buf := debug_newpc
      }
      is(sEnterDebug) {

        // set dcsr
        io.direct_write_ports.dcsr.valid := true.B
        io.direct_write_ports.dcsr.bits := dcsr_field.get_debug_dcsr(
          debug_cause_buf.asUInt,
          privilege_mode
        )

        // set dpc
        io.direct_write_ports.dpc.valid := true.B
        io.direct_write_ports.dpc.bits := debug_newpc_buf

        debug_state := sFlushCache
      }
      is(sFlushCache) {
        dfencei_next := io.store_queue_empty

        when(dfencei_next) {
          assert(io.store_queue_empty, "store queue must be empty when fencei")
        }

        // clear icache after clear dcache
        when(io.dcache_fencei_ack && io.dcache_fencei) {
          flush_next := true.B
          // icache only need one cycle to flush
          ifencei_next := true.B
          dfencei_next := false.B

          // The hart enters Debug Mode.
          debug_state_regs.is_halted := true.B

          debug_state_regs.haltreq_signal := false.B

          // debug mode is always performed in M mode
          privilege_mode := Privilegelevel.M.U
          debug_state := sDebugIdle

          // clear step
          debug_state_regs.clear_stepi()
        }
      }
    }

  }.elsewhen(csr_side_effect && rob_valid_seq.head) {
    // 2. 检查是否因为上一条指令的 csr side effect 而需要 flush
    assert(
      has_interrupt === false.B,
      "csr side effect and interrupt should not happen at the same time"
    )
    SimLog(
      desiredName,
      "csr side effect flush at PC[%x]\n",
      rob_data_seq.head.pc
    )

    // if csr side effect is true, we should flush the pipeline
    // and rerun the current inst
    csr_side_effect := false.B
    flush_next := true.B
    //    io.branch_commit.valid := true.B
    //    io.branch_commit.bits.target := rob_data_seq.head.pc

    redirect_next.valid := true.B
    redirect_next.target := rob_data_seq.head.pc
  }

  // -------------- enter debug mode end----------------

  // -------------- exit debug mode start----------------
  val will_exit_debug_mode =
    debug_state_regs.halted() && debug_state_regs.resumereq_flag
  val sResumeIdle :: sResumeFlush :: Nil = Enum(2)
  val resume_state = RegInit(sResumeIdle)

  when(will_exit_debug_mode) {
    assert(
      flush_next === false.B,
      "flush_next must be false when exit debug mode"
    )
    assert(
      debug_lock === false.B,
      "debug_lock must be false when exit debug mode"
    )

    switch(resume_state) {
      is(sResumeIdle) {
        resume_state := sResumeFlush
      }
      is(sResumeFlush) {
        dfencei_next := io.store_queue_empty

        when(dfencei_next) {
          assert(io.store_queue_empty, "store queue must be empty when fencei")
        }

        // clear icache after clear dcache
        when(io.dcache_fencei_ack && io.dcache_fencei) {
          flush_next := true.B
          // icache only need one cycle to flush
          ifencei_next := true.B
          dfencei_next := false.B

          // 1. pc changes to the value stored in dpc.
          redirect_next.valid := true.B
          redirect_next.target := io.direct_read_ports.dpc

          // 2. The current privilege mode and virtualization mode are changed to that specified by prv and v.
          // NOT support virtualization now
          val new_priv = dcsr_field.prv

          privilege_mode := new_priv

          // 3. When resuming from debug mode, clear mstatus.MPRV if the new privilege mode is less than M-mode
          val mstatus_field = new MstatusFiled(io.direct_read_ports.mstatus)
          io.direct_write_ports.mstatus.valid := new_priv < Privilegelevel.M.U
          io.direct_write_ports.mstatus.bits := mstatus_field
            .get_clear_mprv_mstatus()

          // 4. The debug mode is cleared.
          debug_state_regs.is_halted := false.B
          debug_state_regs.resumereq_flag := false.B
          debug_state_regs.resumeack_signal := true.B

          SimLog(
            desiredName,
            "Will exit debug mode at PC[%x] ,back to priv[%d], clear mprv[%d]\n",
            io.direct_read_ports.dpc,
            dcsr_field.prv,
            new_priv < Privilegelevel.M.U
          )

          // 5. check step, and set single step flag
          // use resume bits for step for test
          when(dcsr_field.step) {
            SimLog(
              desiredName,
              "Entry StepMode at PC[%x]\n",
              io.direct_read_ports.dpc
            )
            debug_state_regs.set_stepi()
          }

          resume_state := sResumeIdle
        }
      }
    }
  }
  // -------------- exit debug mode end----------------

  // -------------- first inst retire start----------------
  val can_retire_first_inst = rob_valid_seq.head && Seq(
    !flush_next,
    !csr_side_effect,
    !will_enter_debug_mode,
    !will_exit_debug_mode
  ).reduce(_ && _)

  when(can_retire_first_inst && rob_data_seq.head.complete) {
    when(rob_data_seq.head.exception.valid) {
      retire_exception(rob_data_seq.head, pop_ack.head)
    }.elsewhen(
      FuOP.is_xret(rob_data_seq.head.fu_op)
    ) {
      when(rob_data_seq.head.fu_op === FuOP.Mret) {
        retire_mret(rob_data_seq.head, pop_ack.head)
      }.otherwise {
        retire_sret(rob_data_seq.head, pop_ack.head)
      }
    }.otherwise {
      switch(rob_data_seq.head.fu_type) {
        is(FuType.Mul, FuType.Alu, FuType.Div) {
          retire_alu(
            rob_data_seq.head,
            io.gpr_commit_ports.head,
            pop_ack.head
          )
        }
        is(FuType.Lsu) {
          when(FuOP.is_load(rob_data_seq.head.fu_op)) {
            retire_load(
              rob_data_seq.head,
              io.gpr_commit_ports.head,
              pop_ack.head
            )
          }.elsewhen(FuOP.is_store(rob_data_seq.head.fu_op)) {
            retire_store(
              rob_data_seq.head,
              io.store_commit,
              pop_ack.head
            )
          }.elsewhen(FuOP.is_atomic(rob_data_seq.head.fu_op)) {
            retire_amo(
              rob_data_seq.head,
              io.gpr_commit_ports.head,
              io.amo_commit,
              pop_ack.head
            )
          }.otherwise {
            assert(false.B, "lsu op error")
          }
        }
        is(FuType.Br) {
          retire_branch(
            rob_data_seq.head,
            io.gpr_commit_ports.head,
            pop_ack.head
          )
        }
        is(FuType.Csr) {
          retire_csr(
            rob_data_seq.head,
            io.gpr_commit_ports.head,
            io.csr_commit,
            pop_ack.head
          )
        }
        is(FuType.None) {
          retire_none(rob_data_seq.head, pop_ack.head)
        }
      }
    }
  }.elsewhen(
    can_retire_first_inst && !rob_data_seq.head.complete
  ) {
    // for csr, mmio, amo the complete flag is not set
    assert(
      rob_data_seq.head.exception.valid === false.B,
      "rob entry must be not exception"
    )

    debug_lock := true.B

    switch(rob_data_seq.head.fu_type) {
      is(FuType.Lsu) {
        when(
          rob_data_seq.head.lsu_io_space && FuOP.is_load(
            rob_data_seq.head.fu_op
          )
        ) {
          io.mmio_commit.valid := true.B
          io.mmio_commit.bits := true.B
        }.elsewhen(FuOP.is_atomic(rob_data_seq.head.fu_op)) {
          io.amo_commit.valid := true.B
          io.amo_commit.bits := true.B
        }
      }
      is(FuType.Csr) {
        io.csr_commit.valid := true.B
        io.csr_commit.bits := true.B
      }
    }
  }

  // -------------- first inst retire end----------------

  when(rob_valid_seq.head && pop_ack.head && debug_state_regs.stepi_exec_flag) {
    debug_state_regs.exec_stepi()
    SimLog(
      desiredName,
      "StepMode Execute PC[%x],than re-enter debug mode\n",
      rob_data_seq.head.pc
    )
  }

  // ----------------------
  // interrupt
  // ----------------------

  val can_handle_interrupt = Seq(
    rob_valid_seq.head && pop_ack.head, // 第一条指令正常完成
    !rob_data_seq.head.exception.valid, // 该指令不能触发异常
    !debug_state_regs.stepi_exec_flag, // 不能是单步执行的指令
    can_retire_first_inst,
    interrupt_inject_types.contains(
      rob_data_seq.head.fu_type.asUInt
    ) // 该指令只能是 alu, mul, div 类型的指令
  ).reduce(_ && _)

  when(can_handle_interrupt) {
    val mip_and_mie = io.direct_read_ports.mip & io.direct_read_ports.mie
    dontTouch(mip_and_mie)

    val mstatus = new MstatusFiled(io.direct_read_ports.mstatus)
    val mideleg = io.direct_read_ports.mideleg

    val interrupt_mmode =
      mstatus.mie && privilege_mode === Privilegelevel.M.U || privilege_mode < Privilegelevel.M.U
    val mmode_pending = new MipFiled(mip_and_mie & (~mideleg).asUInt)
    val mmode_has_interrupt = mmode_pending.any_interrupt

    dontTouch(interrupt_mmode)
    dontTouch(mmode_has_interrupt)

    val interrupt_smode =
      mstatus.sie && privilege_mode === Privilegelevel.S.U || privilege_mode < Privilegelevel.S.U
    val smode_pending = new MipFiled(mip_and_mie & mideleg)
    val smode_has_interrupt = smode_pending.any_interrupt

    dontTouch(interrupt_smode)
    dontTouch(smode_has_interrupt)

    val normal_pc = rob_data_seq.head.pc + Mux(
      rob_data_seq.head.is_rvc,
      2.U,
      4.U
    )

    val br_pc = rob_data_seq.head.bp.predict_pc

    val new_pc = Mux(
      rob_data_seq.head.bp.is_miss_predict,
      br_pc,
      normal_pc
    )

    when(interrupt_mmode && mmode_has_interrupt) {
      val cause = mmode_pending.get_priority_interupt

      has_interrupt := true.B
      interrupt_cause := cause
      flush_next := true.B

      val mtvec = new MtvecFiled(io.direct_read_ports.mtvec)
      privilege_mode := Privilegelevel.M.U

      val interrupt_redirect_pc = mtvec.get_interrupt_pc(
        cause.asUInt(3, 0) //  0-11
      )

      redirect_next.valid := true.B
      redirect_next.target := interrupt_redirect_pc

      io.direct_write_ports.mepc.valid := true.B
      io.direct_write_ports.mepc.bits := new_pc

      io.direct_write_ports.mcause.valid := true.B

      io.direct_write_ports.mcause.bits := cause.asUInt

      io.direct_write_ports.mstatus.valid := true.B
      io.direct_write_ports.mstatus.bits := mstatus.get_mmode_exception_mstatus(
        privilege_mode
      )

      SimLog(
        desiredName,
        "M mode interrupt at pc[%x],cause[%x] to pc[%x]\n",
        new_pc,
        cause.asUInt,
        interrupt_redirect_pc
      )

    }.elsewhen(interrupt_smode && smode_has_interrupt) {
      val cause = smode_pending.get_priority_interupt

      has_interrupt := true.B
      interrupt_cause := cause
      flush_next := true.B
      val stvec = new MtvecFiled(io.direct_read_ports.stvec)

      flush_next := true.B
      privilege_mode := Privilegelevel.S.U

      val interrupt_redirect_pc = stvec.get_interrupt_pc(
        cause.asUInt(3, 0) //  0-11
      )

      redirect_next.valid := true.B
      redirect_next.target := interrupt_redirect_pc
      io.direct_write_ports.sepc.valid := true.B
      io.direct_write_ports.sepc.bits := new_pc

      io.direct_write_ports.scause.valid := true.B
      io.direct_write_ports.scause.bits := cause.asUInt

      io.direct_write_ports.mstatus.valid := true.B
      io.direct_write_ports.mstatus.bits := mstatus.get_smode_exception_mstatus(
        privilege_mode
      )
      SimLog(
        desiredName,
        "S mode interrupt at pc[%x],cause[%x] to pc[%x]\n",
        new_pc,
        cause.asUInt,
        interrupt_redirect_pc
      )
    }
  }

  // ---------------------- second inst retire start----------------

  val can_retire_second_inst = Seq(
    can_retire_first_inst && pop_ack.head, // 第一条指令正常完成
    rob_valid_seq(1) && rob_data_seq(1).complete, // 第二条指令可以提交
    !has_interrupt, // 不能有中断
    !debug_state_regs.stepi_exec_flag // 不能是单步执行的指令
  ).reduce(_ && _)

  when(can_retire_second_inst) {
    val constraint_fu_type = VecInit(
      Seq(
        FuType.Br.asUInt,
        FuType.Csr.asUInt,
        FuType.None.asUInt
      )
    )

    // TODO: more constraint on the second inst?
    when(
      pop_ack.head && !rob_data_seq.head.exception.valid
        && !constraint_fu_type.contains(rob_data_seq.head.fu_type.asUInt)
    ) {
      val retire_fu_type_seq = VecInit(
        Seq(
          FuType.Alu.asUInt,
          FuType.Mul.asUInt,
          FuType.Div.asUInt
        )
      )
      when(retire_fu_type_seq.contains(rob_data_seq(1).fu_type.asUInt)) {
        retire_alu(rob_data_seq(1), io.gpr_commit_ports(1), pop_ack(1))
      }
    }
  }

  // ---------------------- second inst retire end----------------

  // ---------------------
  // instret
  // ---------------------

  val commit_num = PopCountOrder(pop_ack)
  io.direct_write_ports.instret_inc.valid := commit_num =/= 0.U
  io.direct_write_ports.instret_inc.bits := commit_num

  // -----------------------
  // commit monitor
  // -----------------------

  if (monitor_en) {
    val commitMonitorSignals = io.commit_monitor.get

    commitMonitorSignals(0).bits.has_interrupt := has_interrupt
    commitMonitorSignals(1).bits.has_interrupt := false.B
    commitMonitorSignals(0).bits.csr_skip := csr_diff_skip
    commitMonitorSignals(1).bits.csr_skip := false.B

    for (idx <- 0 until num_commit_port) {
      commitMonitorSignals(idx).valid := io.rob_commit_ports(idx).fire
      commitMonitorSignals(idx).bits.pc := rob_data_seq(idx).pc
      commitMonitorSignals(idx).bits.is_rvc := rob_data_seq(idx).is_rvc
      commitMonitorSignals(idx).bits.inst := rob_data_seq(idx).inst
      commitMonitorSignals(idx).bits.fu_type := rob_data_seq(idx).fu_type
      commitMonitorSignals(idx).bits.fu_op := rob_data_seq(idx).fu_op
      commitMonitorSignals(idx).bits.exception := rob_data_seq(idx).exception
      commitMonitorSignals(idx).bits.is_mmio := rob_data_seq(idx).lsu_io_space

      if (idx == 0) {
        // override exception cause
        commitMonitorSignals(idx).bits.exception.cause := Mux(
          has_interrupt,
          interrupt_cause,
          exception_cause
        )
        assert(
          !(has_interrupt && has_exception),
          "interrupt and exception should not happen at the same time"
        )
      }
    }
  }

  // -----------------------
  // assert
  // -----------------------

  assert(
    CheckOrder(VecInit(rob_valid_seq)),
    "rob_commit_ports must be ordered"
  )

//  when(
//    rob_valid_seq.head && rob_data_seq.head.complete
//  ) {
//    assert(
//      rob_data_seq.head.pc =/= 0.U,
//      "pc must not be zero"
//    )
//  }

//  for (i <- 0 until num_commit_port) {
//    when(rob_valid_seq(i) && rob_data_seq(i).complete && pop_ack(i)) {
//      assert(
//        rob_data_seq(i).pc =/= 0.U,
//        "pc must not be zero"
//      )
//
//      when(rob_data_seq(i).bp.is_miss_predict) {
//        assert(
//          rob_data_seq(i).bp.predict_pc =/= 0.U,
//          "predict_pc must not be zero"
//        )
//        assert(rob_data_seq(i).fu_type === FuType.Br, "fu_type must be branch")
//      }
//    }
//  }
}

object gen_commit_stage_verilog extends App {
  GenVerilogHelper(
    new CommitStage(num_commit_port = 2, monitor_en = true)
  )
}
