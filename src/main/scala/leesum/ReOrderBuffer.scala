package leesum

import chisel3._
import chisel3.util._
import leesum.Utils.MultiPortFIFOBase
import leesum.lsu.{AGUWriteBack, LSUResp}
import muldiv.FuMulDivResp

class OperandByPassReq extends Bundle {
  val rs_addr = UInt(5.W)
}

class OperandByPassResp extends Bundle {
  val rs_data = UInt(64.W)
  val fwd_valid = Bool()
  val fwd_stall = Bool()
}

class ReOrderBuffer(
    val entries: Int,
    val num_push_ports: Int,
    val num_pop_ports: Int
) extends Module {
  require(
    entries > 0,
    "ScoreBoard must have non-zero number of entries"
  )
  require(
    isPow2(entries),
    "ScoreBoard must have power-of-2 number of entries"
  )

  val genType = new ScoreBoardEntry()

  val io = IO(
    new Bundle {
      val push_ports = Vec(num_push_ports, Flipped(Decoupled(genType)))
      val pop_ports = Vec(num_pop_ports, Decoupled(genType))
      val flush = Input(Bool())
      // trans_id of the current push data(scoreboard-entry)
      val push_trans_id =
        Output(Vec(num_push_ports, UInt(log2Ceil(entries).W)))

      val operand_bypass_req =
        Input(Vec(num_push_ports * 2, new OperandByPassReq()))
      val operand_bypass_resp = Output(
        Vec(num_push_ports * 2, new OperandByPassResp())
      )

      // write-back port
      // alu
      val fu_alu_wb_port = Vec(2, Flipped(Decoupled(new AluResp)))
      // branch
      val fu_branch_wb_port = Flipped(Decoupled(new FuBranchResp))
      // lsu_port
      val fu_lsu_wb_port = Flipped(Decoupled(new LSUResp))
      val fu_lsu_agu_wb_port = Flipped(Decoupled(new AGUWriteBack))
      //  mul_div_port
      val fu_mul_div_wb_port = Flipped(Decoupled(new FuMulDivResp))
      //  csr_port
      val fu_csr_wb_port = Flipped(Decoupled(new FuCsrResp))
    }
  )

  // ---------------------------
  //  internal fifo
  // ---------------------------

  val rob = new MultiPortFIFOBase(
    gen = genType,
    entries,
    num_push_ports,
    num_pop_ports,
    use_mem = false,
    with_valid = true
  )

  // ---------------------------
  //  push pop ports logic
  // ---------------------------

  rob.push_pop_flush_cond(
    push_cond = VecInit(io.push_ports.map(_.fire)),
    pop_cond = VecInit(io.pop_ports.map(_.fire)),
    flush_cond = io.flush,
    entry = VecInit(io.push_ports.map(_.bits))
  )

  val pop_peek = rob.peek()

  // TODO: use complete signal?
  pop_peek.zipWithIndex.foreach({ case (pop, i) =>
    io.pop_ports(i).bits := pop.bits
    io.pop_ports(i).valid := pop.valid
  })

  // TODO: performance???
  0.until(num_push_ports)
    .foreach(i => io.push_ports(i).ready := rob.free_entries > i.U)

  // ---------------------------
  // allocate trans_id for push
  // ---------------------------
  0.until(num_push_ports)
    .foreach(i => {
      io.push_trans_id(i) := rob.push_ptrs(i)
    })

  // -------------------------
  // write-back ports logic
  // -------------------------

  // no exception for alu
  def alu_write_back(alu_resp: AluResp, en: Bool): Unit = {
    when(en) {
      val rob_idx = alu_resp.trans_id
      rob.content(rob_idx).complete := true.B
      rob.content(rob_idx).result := alu_resp.res
      rob.content(rob_idx).result_valid := true.B

      assert(
        rob.content_valid(rob_idx),
        "rob entry must be valid"
      )
    }
  }

  def lsu_write_back(lsu_resp: LSUResp, en: Bool): Unit = {
    when(en) {
      val rob_idx = lsu_resp.trans_id
      // normal write-back in LoadQueue, no exception
      rob.content(rob_idx).complete := true.B
      rob.content(rob_idx).result := lsu_resp.wb_data
      rob.content(rob_idx).result_valid := true.B
      assert(
        rob.content_valid(rob_idx),
        "rob entry must be valid"
      )
    }
  }

//  def amo_write_back(amo_resp: LSUResp, en: Bool): Unit = {
//    when(en) {
//      val rob_idx = amo_resp.trans_id
//      // normal write-back in LoadQueue, no exception
//      rob.content(rob_idx).complete := true.B
//      rob.content(rob_idx).result := amo_resp.wb_data
//      rob.content(rob_idx).result_valid := true.B
//      assert(
//        rob.content_valid(rob_idx),
//        "rob entry must be valid"
//      )
//    }
//  }

  def agu_write_back(agu_resp: AGUWriteBack, en: Bool) = {
    when(en) {
      val rob_idx = agu_resp.trans_id
      when(agu_resp.exception.valid) {
        // 1. exception happened in AGU
        rob.content(rob_idx).complete := true.B
        rob.content(rob_idx).exception := agu_resp.exception
      }.elsewhen(agu_resp.is_mmio) {
        // 2. detect mmio access in AGU
        // if op is store, then complete is true
        rob.content(rob_idx).complete := agu_resp.is_store
        rob.content(rob_idx).lsu_io_space := true.B
      }.otherwise {
        assert(agu_resp.is_store, "only store can reach here")
        rob.content(rob_idx).complete := true.B
      }
    }
  }

  def branch_write_back(branch_resp: FuBranchResp, en: Bool): Unit = {
    when(en) {
      val rob_idx = branch_resp.trans_id
      rob.content(rob_idx).complete := true.B
      rob.content(rob_idx).result := branch_resp.wb_data
      rob.content(rob_idx).result_valid := branch_resp.wb_valid
      rob
        .content(rob_idx)
        .bp
        .is_miss_predict := branch_resp.is_miss_predict
      rob.content(rob_idx).bp.predict_pc := branch_resp.redirect_pc
      rob.content(rob_idx).bp.bp_type := branch_resp.branch_type
      rob.content(rob_idx).exception := branch_resp.exception

      assert(
        rob.content_valid(rob_idx),
        "rob entry must be valid"
      )
      assert(
        !(branch_resp.exception.valid && branch_resp.wb_valid),
        "result and exception can't be valid at the same time"
      )
    }
  }

  // no exception for mul_div
  def mul_div_write_back(
      mul_div_resp: FuMulDivResp,
      en: Bool
  ): Unit = {
    when(en) {
      val rob_idx = mul_div_resp.trans_id
      rob.content(rob_idx).complete := true.B
      rob.content(rob_idx).result := mul_div_resp.data
      rob.content(rob_idx).result_valid := true.B

      assert(
        rob.content_valid(rob_idx),
        "rob entry must be valid"
      )
    }
  }

  def csr_write_back(csr_resp: FuCsrResp, en: Bool) = {

    when(en) {
      val rob_idx = csr_resp.trans_id
      rob.content(rob_idx).complete := true.B
      rob.content(rob_idx).result := csr_resp.data
      rob.content(rob_idx).result_valid := !csr_resp.exception.valid
      rob.content(rob_idx).exception := csr_resp.exception

      assert(
        rob.content_valid(rob_idx),
        "rob entry must be valid"
      )
    }

  }

  io.fu_alu_wb_port.foreach(_.ready := true.B)
  io.fu_branch_wb_port.ready := true.B
  io.fu_lsu_wb_port.ready := true.B
//  io.fu_amo_wb_port.ready := true.B
  io.fu_lsu_agu_wb_port.ready := true.B
  io.fu_mul_div_wb_port.ready := true.B
  io.fu_csr_wb_port.ready := true.B

  io.fu_alu_wb_port.foreach(alu_resp =>
    alu_write_back(alu_resp.bits, alu_resp.fire)
  )
  agu_write_back(io.fu_lsu_agu_wb_port.bits, io.fu_lsu_agu_wb_port.fire)
  lsu_write_back(io.fu_lsu_wb_port.bits, io.fu_lsu_wb_port.fire)
  branch_write_back(io.fu_branch_wb_port.bits, io.fu_branch_wb_port.fire)
  mul_div_write_back(io.fu_mul_div_wb_port.bits, io.fu_mul_div_wb_port.fire)
  csr_write_back(io.fu_csr_wb_port.bits, io.fu_csr_wb_port.fire)

  // ------------------------------------------
  // register rename logic, renaming in rob
  // ------------------------------------------

  // register rename table is used to rename rd_addr
  // the content of rename_table is the index of rob entry
  val rename_table = new RenameTable(
    set_ports_num = num_push_ports,
    clear_ports_num = num_pop_ports,
    rob_nums = entries
  )

  val rename_set_ports = Wire(
    Vec(num_push_ports, Valid(new RenameSetPort(entries)))
  )
  val rename_clear_ports = Wire(
    Vec(num_pop_ports, Valid(new RenameClearPort(entries)))
  )

  for (i <- 0 until num_push_ports) {
    val rd_addr = io.push_ports(i).bits.rd_addr
    rename_set_ports(i).valid := io.push_ports(i).fire && (rd_addr =/= 0.U)
    rename_set_ports(i).bits.rd_addr := rd_addr
    rename_set_ports(i).bits.rob_ptr := rob.push_ptrs(i)
  }
  for (i <- 0 until num_pop_ports) {
    val rd_addr = io.pop_ports(i).bits.rd_addr
    rename_clear_ports(i).valid := io.pop_ports(i).fire && (rd_addr =/= 0.U)
    rename_clear_ports(i).bits.rd_addr := rd_addr
    rename_clear_ports(i).bits.rob_ptr := rob.pop_ptrs(i)
  }

  rename_table.set_clear_flush(
    set_ports = rename_set_ports,
    clear_ports = rename_clear_ports,
    flush_port = io.flush
  )

  // ----------------------
  // bypass logic
  // ----------------------
  val fu_valid = VecInit(
    io.fu_alu_wb_port.map(_.fire) ++
      Seq(
        io.fu_branch_wb_port.fire,
        io.fu_lsu_wb_port.fire,
        io.fu_mul_div_wb_port.fire
      )
  )
  val fu_id = VecInit(
    io.fu_alu_wb_port.map(_.bits.trans_id) ++
      Seq(
        io.fu_branch_wb_port.bits.trans_id,
        io.fu_lsu_wb_port.bits.trans_id,
        io.fu_mul_div_wb_port.bits.trans_id
      )
  )
  val fu_result = VecInit(
    io.fu_alu_wb_port.map(_.bits.res) ++
      Seq(
        io.fu_branch_wb_port.bits.wb_data,
        io.fu_lsu_wb_port.bits.wb_data,
        io.fu_mul_div_wb_port.bits.data
      )
  )
  val fu_result_valid = VecInit(
    io.fu_alu_wb_port.map(_.fire) ++
      Seq(
        io.fu_branch_wb_port.bits.wb_valid,
        io.fu_lsu_wb_port.bits.wb_data_valid,
        io.fu_mul_div_wb_port.fire
      )
  )

  def check_operand_bypass(
      bypass_req: OperandByPassReq,
      bypass_resp: OperandByPassResp
  ) = {
    val rob_read = rename_table.crate_read_port(bypass_req.rs_addr)
    val rob_idx_valid = rob_read.valid
    val rob_idx = rob_read.bits
    val rob_entry = rob.content(rob_idx)
    val rob_entry_valid = rob.content_valid(rob_idx)

    when(rob_idx_valid) {
      val is_fu_ready = VecInit(
        fu_id.zip(fu_valid).zip(fu_result_valid).map {
          case ((id, valid), result_valid) =>
            id === rob_idx & valid & result_valid
        }
      )

      val is_any_fu_ready = is_fu_ready.reduce(_ | _)

      assert(
        PopCount(is_fu_ready) <= 1.U,
        "rs1 bypass error, fu write back conflict"
      )

      when(is_any_fu_ready) {
        // bypass from fu
        bypass_resp.fwd_valid := true.B
        bypass_resp.fwd_stall := false.B
        bypass_resp.rs_data := fu_result(is_fu_ready.indexWhere(_ === true.B))
      }.otherwise {
        // bypass from rob
        assert(
          rob_entry_valid,
          "rs1 bypass error, rob entry must be valid"
        )

        when(rob_entry.rd_data_valid()) {
          bypass_resp.fwd_valid := true.B
          bypass_resp.fwd_stall := false.B
          bypass_resp.rs_data := rob_entry.result
        }.otherwise {
          bypass_resp.fwd_valid := false.B
          bypass_resp.fwd_stall := true.B
        }
      }
    }
  }

  io.operand_bypass_resp.foreach(bypass_resp => {
    bypass_resp.fwd_valid := false.B
    bypass_resp.fwd_stall := false.B
    bypass_resp.rs_data := DontCare
  })

  require(io.operand_bypass_resp.length == io.operand_bypass_req.length)

  for (i <- 0 until io.operand_bypass_req.length) {
    check_operand_bypass(
      io.operand_bypass_req(i),
      io.operand_bypass_resp(i)
    )
  }

  // ----------------------
  // assert
  // ----------------------

  // if Write-back ID conflict, then return false
  def check_write_back_conflict(): Bool = {
    val is_distinct = WireInit(true.B)
    for (i <- fu_id.indices; j <- i + 1 until fu_id.length) {
      when(fu_valid(i) && fu_valid(j) && fu_id(i) === fu_id(j)) {
        is_distinct := false.B
      }
    }
    is_distinct
  }
  assert(
    check_write_back_conflict(),
    "write-back conflict"
  )

//  rob.content.foreach(entry => {
//    when(entry.valid) {
//      when(entry.bits.result_valid) {
//        assert(entry.bits.complete, "rob entry must be complete")
//      }
//
//      when(entry.bits.exception.valid) {
//        assert(entry.bits.complete, "rob entry must be complete")
//      }
//
//      assert(
//        !(entry.bits.exception.valid && entry.bits.result_valid),
//        "result and exception can't be valid at the same time"
//      )
//    }
//  })
}

object gen_ReOrderBuffer_verilog extends App {
  GenVerilogHelper(new ReOrderBuffer(8, 2, 2))
}
