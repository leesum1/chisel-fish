package leesum

import chisel3._
import chisel3.util._
import leesum.Cache.DcacheConst
import leesum.Utils.MultiPortFIFOUseMEM
import leesum.lsu.{AtomicOP, LSUReq}
import muldiv.DivReq

class FuReq extends Bundle {

  val alu = Vec(2, Decoupled(new AluReq()))
  val branch_0 = Decoupled(new FuBranchReq())
  val lsu_0 = Decoupled(new LSUReq)
  val mul_0 = Decoupled(new muldiv.MulReq)
  val div_0 = Decoupled(new DivReq)

  val csr_0 = Decoupled(new FuCsrReq)
}

class IssueStageNew(num_push_port: Int, num_pop_port: Int) extends Module {
  val io = IO(new Bundle {

    val push_port =
      Vec(num_push_port, Flipped(Decoupled(new ScoreBoardEntry())))
    val pop_port = Vec(num_pop_port, Decoupled(new ScoreBoardEntry()))
    val flush = Input(Bool())
    val alloc_trans_id = Input(Vec(num_pop_port, UInt(32.W)))
    val gpr_read_port = Vec(num_pop_port, new RegFileReadPort())

    val operand_bypass_req =
      Output(Vec(num_pop_port * 2, new OperandByPassReq()))
    val operand_bypass_resp =
      Input(Vec(num_pop_port * 2, new OperandByPassResp()))

    val fu_port = new FuReq()
  })

  require(num_pop_port == 2, "only support 2 pop port now")
  require(num_push_port == 2, "only support 2 push port now")

  // ----------------
  // issue fifo
  // ----------------

//  val issue_fifo2 = Module(
//    new DummyMultiPortFIFO(
//      gen = new ScoreBoardEntry,
//      size = 8,
//      num_push_port,
//      num_pop_port
//    )
//  )
//
//  issue_fifo2.io.in <> io.push_port
//  issue_fifo2.io.flush := io.flush
//  issue_fifo2.io.out <> io.pop_port
//
//  val issue_peek = Wire(Vec(num_pop_port, Valid(new ScoreBoardEntry)))
//
//  for (i <- 0 until num_pop_port) {
//    issue_peek(i).valid := issue_fifo2.io.out(i).valid
//    issue_peek(i).bits := issue_fifo2.io.out(i).bits
//  }

  val issue_fifo = new MultiPortFIFOUseMEM(
    new ScoreBoardEntry(),
    16,
    num_push_port,
    num_pop_port
//    use_mem = false,
//    with_valid = false
  )

  issue_fifo.push_pop_flush_cond(
    VecInit(io.push_port.map(_.fire)),
    VecInit(io.pop_port.map(_.fire)),
    io.flush,
    VecInit(io.push_port.map(_.bits))
  )

  val issue_peek = issue_fifo.peek()

  0.until(num_push_port)
    .foreach({ i =>
      io.push_port(i).ready := issue_fifo.free_entries > i.U
    })

  // ------------------
  // fu pipeline
  // ------------------

  val alu_pipe_vec = Wire(Vec(2, Decoupled(new AluReq())))
  val branch_0_pipe = Wire(Decoupled(new FuBranchReq()))
  val lsu_0_pipe = Wire(Decoupled(new LSUReq))
  val mul_0_pipe = Wire(Decoupled(new muldiv.MulReq))
  val div_0_pipe = Wire(Decoupled(new DivReq))

  val csr_0_pipe = Wire(Decoupled(new FuCsrReq))

  alu_pipe_vec.foreach(
    _.noenq()
  )
  branch_0_pipe.noenq()
  lsu_0_pipe.noenq()
  csr_0_pipe.noenq()
  mul_0_pipe.noenq()
  div_0_pipe.noenq()

  require(io.fu_port.alu.length == alu_pipe_vec.length)

  for (i <- 0 until io.fu_port.alu.length) {
    io.fu_port.alu(i) <> PipeLine(alu_pipe_vec(i), io.flush)
  }
  io.fu_port.branch_0 <> PipeLine(branch_0_pipe, io.flush)
  io.fu_port.lsu_0 <> PipeLine(lsu_0_pipe, io.flush)
  io.fu_port.csr_0 <> PipeLine(csr_0_pipe, io.flush)
  io.fu_port.mul_0 <> PipeLine(mul_0_pipe, io.flush)
  io.fu_port.div_0 <> PipeLine(div_0_pipe, io.flush)

  // ----------------
  // bypass logic
  // ----------------

  require(issue_peek.length * 2 == io.operand_bypass_req.length)

  // send rs1_addr and rs2_addr to bypass logic
  for (i <- 0 until issue_peek.length) {
    // rs1
    io.operand_bypass_req(i * 2).rs_addr := Mux(
      issue_peek(i).valid,
      issue_peek(i).bits.rs1_addr,
      0.U
    )
    // rs2
    io.operand_bypass_req(i * 2 + 1).rs_addr := Mux(
      issue_peek(i).valid,
      issue_peek(i).bits.rs2_addr,
      0.U
    )
  }

  // ---------------------------------
  //  stall logic
  // ---------------------------------

  val inst_stall = VecInit(false.B, false.B)

  class operandBundle extends Bundle {
    val rs1_data = UInt(64.W)
    val rs2_data = UInt(64.W)
    val pc = UInt(64.W)
    val imm = UInt(64.W)
  }
//
//  def check_raw_between_issue(
//      inst_scb_seq: Vec[ScoreBoardEntry],
//      stall_seq: Vec[Bool]
//  ): Unit = {
//    require(inst_scb_seq.length == stall_seq.length)
//    for (i <- inst_scb_seq.length - 1 to 0 by -1) {
//      for (j <- i - 1 to 0 by -1) {
//        val cur_inst = inst_scb_seq(i)
//        val pre_inst = inst_scb_seq(j)
//        when(
//          cur_inst.rs1_addr === pre_inst.rd_addr && cur_inst.rs1_addr =/= 0.U
//        ) {
//          stall_seq(i) := true.B
//        }
//        when(
//          cur_inst.rs2_addr === pre_inst.rd_addr && cur_inst.rs2_addr =/= 0.U
//        ) {
//          stall_seq(i) := true.B
//        }
//      }
//    }
//  }

  def check_raw_between_issue(
      inst_scb_seq: Vec[ScoreBoardEntry],
      stall_seq: Vec[Bool]
  ): Unit = {
    require(inst_scb_seq.length == stall_seq.length)
    require(inst_scb_seq.length == 2)
    val cur_inst = inst_scb_seq(1)
    val pre_inst = inst_scb_seq(0)

    when(
      pre_inst.rd_addr =/= 0.U &&
        (cur_inst.rs1_addr === pre_inst.rd_addr || cur_inst.rs2_addr === pre_inst.rd_addr)
    ) {
      stall_seq(1) := true.B
    }

//    for (i <- inst_scb_seq.length - 1 to 0 by -1) {
//      for (j <- i - 1 to 0 by -1) {
//        val cur_inst = inst_scb_seq(i)
//        val pre_inst = inst_scb_seq(j)
//
//        when(
//          pre_inst.rd_addr =/= 0.U &&
//            (cur_inst.rs1_addr === pre_inst.rd_addr || cur_inst.rs2_addr === pre_inst.rd_addr)
//        ) {
//          stall_seq(i) := true.B
//        }
//      }
//    }
  }

  def check_fu_hazard(
      fu_seq: Vec[UInt],
      stall_seq: Vec[Bool]
  ): Unit = {
    require(fu_seq.length == stall_seq.length)
    require(
      fu_seq
        .map(_.getWidth)
        .distinct
        .length == 1 && fu_seq.head.getWidth == FuType.width,
      "fu_seq should have same width"
    )
    val fu_hazard_seq = VecInit(Seq.fill(fu_seq.length)(false.B))

    for (i <- 0 until stall_seq.length) {
      fu_hazard_seq(i) := MuxLookup(fu_seq(i), false.B)(
        Seq(
          FuType.Alu -> false.B,
          FuType.Br -> false.B,
          FuType.None -> false.B,
          FuType.Lsu -> !lsu_0_pipe.ready,
          FuType.Mul -> !mul_0_pipe.ready,
          FuType.Div -> !div_0_pipe.ready,
          FuType.Csr -> !csr_0_pipe.ready
        )
      )
    }

    // when two inst use the same fu, stall the second inst, except multi fu unit
    val multi_fu_vec = VecInit(
      Seq(
        FuType.None.asUInt,
        FuType.Alu.asUInt
      )
    )

    when(fu_seq(0) === fu_seq(1)) {
      when(!multi_fu_vec.contains(fu_seq(0).asUInt)) {
        fu_hazard_seq(1) := true.B
      }
    }

    fu_hazard_seq.zipWithIndex.foreach { case (fu_hazard, i) =>
      when(fu_hazard) {
        stall_seq(i) := true.B
      }
    }
  }

  def check_operands(
      bypass_vec: Seq[OperandByPassResp],
      stall_seq: Vec[Bool]
  ): Unit = {
    require(bypass_vec.length == stall_seq.length * 2)
    bypass_vec.grouped(2).zip(stall_seq).foreach { case (bypass, stall) =>
      when(bypass(0).fwd_stall || bypass(1).fwd_stall) {
        stall := true.B
      }
    }
  }

  check_operands(io.operand_bypass_resp, inst_stall)
  check_fu_hazard(VecInit(issue_peek.map(_.bits.fu_type)), inst_stall)
  check_raw_between_issue(VecInit(issue_peek.map(_.bits)), inst_stall)

  // -----------------------
  // read operands logic
  // -----------------------

  val inst_op = WireInit(
    VecInit(Seq.fill(num_pop_port)(0.U.asTypeOf(new operandBundle)))
  )

  issue_peek
    .zip(io.gpr_read_port)
    .foreach(
      { case (scb, gpr_read_port) =>
        gpr_read_port.rs1_addr := scb.bits.rs1_addr
        gpr_read_port.rs2_addr := scb.bits.rs2_addr
      }
    )

  def read_operands(
      bypass_vec: Seq[OperandByPassResp],
      scb: ScoreBoardEntry,
      gpr: RegFileReadPort
  ): operandBundle = {
    require(bypass_vec.length == 2)

    val op_bundle = WireInit(0.U.asTypeOf(new operandBundle))

    when(bypass_vec(0).fwd_valid) {
      op_bundle.rs1_data := bypass_vec(0).rs_data
    }.otherwise {
      op_bundle.rs1_data := gpr.rs1_data
    }

    when(bypass_vec(1).fwd_valid) {
      op_bundle.rs2_data := bypass_vec(1).rs_data
    }.otherwise {
      op_bundle.rs2_data := gpr.rs2_data
    }

    op_bundle.pc := scb.pc
    op_bundle.imm := scb.result
    op_bundle
  }

  for (i <- 0 until issue_peek.length) {
    inst_op(i) := read_operands(
      io.operand_bypass_resp.grouped(2).toSeq(i),
      issue_peek(i).bits,
      io.gpr_read_port(i)
    )
  }

  // -----------------------
  // issue logic
  // -----------------------

  // TODO:  need optimize
  def check_issue(
      inst_seq: Vec[Valid[ScoreBoardEntry]],
      inst_stall: Vec[Bool]
  ) = {
    val allow_issue_tmp = WireInit(VecInit(false.B, false.B))
    for (i <- 0 until inst_seq.length) {
      when(inst_seq(i).valid) {
        when(!inst_stall(i)) {
          allow_issue_tmp(i) := true.B
        }
      }
    }
    GenOrderVec(allow_issue_tmp)
  }

  val allow_issue = check_issue(issue_peek, inst_stall)
  // -----------------------
  // dispatch to scoreboard
  // -----------------------
  assert(CheckOrder(allow_issue), "issue should be ordered")

  for (i <- 0 until issue_peek.length) {
    when(allow_issue(i)) {
      assert(issue_peek(i).valid, "fifo_data_seq should be valid")
      io.pop_port(i).bits := issue_peek(i).bits
      io.pop_port(i).valid := true.B
    }.otherwise {
      io.pop_port(i).bits := DontCare
      io.pop_port(i).valid := false.B
    }
  }

  val allow_issue_fire = io.pop_port.map(_.fire)

  // -------------------
  // alu logic
  // -------------------
  def dispatch_to_alu(
      alu: DecoupledIO[AluReq],
      scb: ScoreBoardEntry,
      alu_allow_dispatch: Bool,
      op_bundle: operandBundle,
      trans_id: UInt
  ): Unit = {
    when(scb.fu_type === FuType.Alu && alu_allow_dispatch) {
      val alu_op = alu.bits.convert_fuop2aluop(scb.fu_op)
      alu.valid := alu_allow_dispatch
      alu.bits.a := Mux(scb.use_pc, op_bundle.pc, op_bundle.rs1_data)
      alu.bits.b := Mux(scb.use_imm, op_bundle.imm, op_bundle.rs2_data)
      alu.bits.op := alu_op
      alu.bits.is_rv32 := scb.is_rv32
      alu.bits.trans_id := trans_id
    }
  }

  val alu_allow_dispatch = VecInit(
    allow_issue_fire.zip(issue_peek).map { case (allow, scb) =>
      allow && scb.bits.fu_type === FuType.Alu
    }
  )

  for (i <- 0.until(2)) {
    dispatch_to_alu(
      alu = alu_pipe_vec(i),
      scb = issue_peek(i).bits,
      alu_allow_dispatch = alu_allow_dispatch(i),
      op_bundle = inst_op(i),
      trans_id = io.alloc_trans_id(i)
    )
  }
  assert(
    PopCount(alu_allow_dispatch) <= 2.U,
    "alu unit can only dispatch two inst at one cycle"
  )

  // ------------------
  // br logic
  // ------------------
  val br_allow_dispatch = VecInit(
    allow_issue_fire.zip(issue_peek).map { case (allow, scb) =>
      allow && scb.bits.fu_type === FuType.Br
    }
  )

  assert(
    PopCount(br_allow_dispatch) <= 1.U,
    "branch unit can only dispatch one inst at one cycle"
  )

  def dispatch_to_br(
      br: DecoupledIO[FuBranchReq],
      scb: ScoreBoardEntry,
      br_allow_dispatch: Bool,
      op_bundle: operandBundle,
      trans_id: UInt
  ): Unit = {
    when(scb.fu_type === FuType.Br && br_allow_dispatch) {
      br.valid := true.B
      br.bits.fu_op := scb.fu_op
      br.bits.rs1 := op_bundle.rs1_data
      br.bits.rs2 := op_bundle.rs2_data
      br.bits.pc := op_bundle.pc
      br.bits.imm := op_bundle.imm
      br.bits.is_rvc := scb.is_rvc
      br.bits.trans_id := trans_id
      br.bits.bp := scb.bp
    }
  }

  for (i <- 0.until(num_pop_port)) {
    dispatch_to_br(
      branch_0_pipe,
      issue_peek(i).bits,
      br_allow_dispatch(i),
      op_bundle = inst_op(i),
      io.alloc_trans_id(i)
    )
  }

  // -------------------
  // lsu logic
  // -------------------
  val lsu_allow_dispatch = VecInit(
    allow_issue_fire.zip(issue_peek).map { case (allow, scb) =>
      allow && scb.bits.fu_type === FuType.Lsu
    }
  )
  assert(
    PopCount(lsu_allow_dispatch) <= 1.U,
    "load-store unit can only dispatch one inst at one cycle"
  )

  def dispatch_to_lsu(
      lsu: DecoupledIO[LSUReq],
      scb: ScoreBoardEntry,
      lsu_allow_dispatch: Bool,
      op_bundle: operandBundle,
      trans_id: UInt
  ) = {
    when(scb.fu_type === FuType.Lsu && lsu_allow_dispatch) {
      val is_atomic = FuOP.is_atomic(scb.fu_op)
      val atomic_size = Mux(scb.is_rv32, DcacheConst.SIZE4, DcacheConst.SIZE8)

      lsu.valid := true.B
      lsu.bits.op_a := op_bundle.rs1_data
      lsu.bits.op_b := op_bundle.imm
      lsu.bits.store_data := op_bundle.rs2_data // store data or amo rs2
      lsu.bits.is_store := FuOP.is_store(scb.fu_op)
      lsu.bits.is_load := FuOP.is_load(scb.fu_op)

      lsu.bits.atomic_op := AtomicOP.FuOP2AtomicOP(scb.fu_op)
      lsu.bits.size := Mux(is_atomic, atomic_size, FuOP.get_lsu_size(scb.fu_op))
      lsu.bits.sign_ext := FuOP.lsu_need_sign_ext(scb.fu_op)
      lsu.bits.trans_id := trans_id
    }
  }

  for (i <- 0.until(num_pop_port)) {
    dispatch_to_lsu(
      lsu_0_pipe,
      issue_peek(i).bits,
      lsu_allow_dispatch(i),
      op_bundle = inst_op(i),
      io.alloc_trans_id(i)
    )
  }

  // ----------------------
  // csr logic
  // ----------------------
  val csr_allow_dispatch = VecInit(
    allow_issue_fire.zip(issue_peek).map { case (allow, scb) =>
      allow && scb.bits.fu_type === FuType.Csr
    }
  )
  assert(
    PopCount(csr_allow_dispatch) <= 1.U,
    "csr unit can only dispatch one inst at one cycle"
  )

  def dispatch_to_csr(
      csr: DecoupledIO[FuCsrReq],
      scb: ScoreBoardEntry,
      csr_allow_dispatch: Bool,
      op_bundle: operandBundle,
      trans_id: UInt
  ) = {
    when(scb.fu_type === FuType.Csr && csr_allow_dispatch) {
      assert(FuOP.is_csr(scb.fu_op))
      val inst_base = new InstBase(scb.inst)
      csr.valid := true.B
      csr.bits.csr_addr := inst_base.csr_addr
      csr.bits.csr_op := scb.fu_op
      csr.bits.rs1_or_zimm :=
        Mux(scb.use_immz, inst_base.imm_z, op_bundle.rs1_data)
      csr.bits.trans_id := trans_id
      csr.bits.read_en := !(scb.fu_op === FuOP.CSRRW && scb.rd_addr === 0.U)
      csr.bits.write_en := !(scb.fu_op =/= FuOP.CSRRW && Mux(
        scb.use_immz,
        inst_base.imm_z === 0.U,
        scb.rs1_addr === 0.U
      ))
    }
  }

  for (i <- 0.until(num_pop_port)) {
    dispatch_to_csr(
      csr_0_pipe,
      issue_peek(i).bits,
      csr_allow_dispatch(i),
      op_bundle = inst_op(i),
      io.alloc_trans_id(i)
    )
  }

  // -------------------
  // mul logic
  // -------------------
  val mul_allow_dispatch = VecInit(
    allow_issue_fire.zip(issue_peek).map { case (allow, scb) =>
      allow && scb.bits.fu_type === FuType.Mul
    }
  )
  assert(
    PopCount(mul_allow_dispatch) <= 1.U,
    "mul unit can only dispatch one inst at one cycle"
  )

  def dispatch_to_mul(
      mul: DecoupledIO[muldiv.MulReq],
      scb: ScoreBoardEntry,
      mul_allow_dispatch: Bool,
      op_bundle: operandBundle,
      trans_id: UInt
  ) = {
    when(scb.fu_type === FuType.Mul && mul_allow_dispatch) {
      assert(FuOP.is_mul(scb.fu_op))
      mul.valid := true.B
      mul.bits.op_a := op_bundle.rs1_data
      mul.bits.op_b := op_bundle.rs2_data
      mul.bits.op_type := scb.fu_op
      mul.bits.is_rv32 := scb.is_rv32
      mul.bits.trans_id := trans_id
    }
  }

  for (i <- 0.until(num_pop_port)) {
    dispatch_to_mul(
      mul_0_pipe,
      issue_peek(i).bits,
      mul_allow_dispatch(i),
      op_bundle = inst_op(i),
      io.alloc_trans_id(i)
    )
  }

  // ------------------
  // div logic
  // ------------------
  val div_allow_dispatch = VecInit(
    allow_issue_fire.zip(issue_peek).map { case (allow, scb) =>
      allow && scb.bits.fu_type === FuType.Div
    }
  )
  assert(
    PopCount(div_allow_dispatch) <= 1.U,
    "div unit can only dispatch one inst at one cycle"
  )

  def dispatch_to_div(
      div: DecoupledIO[DivReq],
      scb: ScoreBoardEntry,
      div_allow_dispatch: Bool,
      op_bundle: operandBundle,
      trans_id: UInt
  ) = {
    when(scb.fu_type === FuType.Div && div_allow_dispatch) {
      assert(FuOP.is_div_rem(scb.fu_op))
      div.valid := true.B
      div.bits.op_a := op_bundle.rs1_data
      div.bits.op_b := op_bundle.rs2_data
      div.bits.op_type := scb.fu_op
      div.bits.is_rv32 := scb.is_rv32
      div.bits.trans_id := trans_id
    }
  }

  for (i <- 0.until(num_pop_port)) {
    dispatch_to_div(
      div_0_pipe,
      issue_peek(i).bits,
      div_allow_dispatch(i),
      op_bundle = inst_op(i),
      io.alloc_trans_id(i)
    )
  }

  // ------------------
  // assert
  // ------------------

  io.operand_bypass_resp.foreach(bypass_resp => {
    assert(
      PopCount(VecInit(bypass_resp.fwd_valid, bypass_resp.fwd_stall)) <= 1.U,
      "only one valid signal can be true"
    )
  })
}

object gen_IssueStageNew_verilog extends App {
  GenVerilogHelper(new IssueStageNew(2, 2))
}
