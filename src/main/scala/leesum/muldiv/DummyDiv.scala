package leesum.muldiv

import chisel3._
import chisel3.util.{Decoupled, Enum, MuxLookup, is, switch}
import leesum.FuOP
import leesum.SignExt
import leesum.GenMaskOne
import leesum.PipeLine
import leesum.GenMaskZero
import leesum.GenVerilogHelper

class DivReq extends Bundle {
  private val Div_width = 64
  val op_a = UInt(Div_width.W)
  val op_b = UInt(Div_width.W)
  val trans_id = UInt(32.W)
  val op_type = UInt(FuOP.width.W)
  val is_rv32 = Bool()
}

class DivResp extends FuMulDivResp {}

class DummyDiv extends Module {
  val io = IO(new Bundle {
    val div_req = Flipped(Decoupled(new DivReq))
    val div_resp = Decoupled(new DivResp)
    val flush = Input(Bool())
  })

  val op_a = io.div_req.bits.op_a
  val op_b = io.div_req.bits.op_b

  require(op_a.getWidth == 64);
  require(op_b.getWidth == 64);

  val div_res = Wire(SInt(64.W))
  div_res := 0.S
  when(io.div_req.fire) {
    div_res := op_a.asSInt / op_b.asSInt
  }

  val rem_res = (op_a.asSInt % op_b.asSInt)
  // TODO: bug!!!! signed will became unsigned!!!
  dontTouch(rem_res)

  val divu_res = op_a / op_b
  val remu_res = op_a % op_b
  val divw_res = op_a(31, 0).asSInt / op_b(31, 0).asSInt
  val remw_res = op_a(31, 0).asSInt % op_b(31, 0).asSInt
  val divuw_res = op_a(31, 0) / op_b(31, 0)
  val remuw_res = op_a(31, 0) % op_b(31, 0)

  val div_rem_res = MuxLookup(
    io.div_req.bits.op_type.asUInt,
    0.U(64.W)
  )(
    Seq(
      FuOP.DivDiv.asUInt -> Mux(
        io.div_req.bits.is_rv32,
        SignExt(divw_res.asUInt, 32, 64),
        div_res(63, 0)
      ),
      FuOP.DivRem.asUInt -> Mux(
        io.div_req.bits.is_rv32,
        SignExt(remw_res.asUInt, 32, 64),
        rem_res.asUInt
      ),
      FuOP.DivDivu.asUInt -> Mux(
        io.div_req.bits.is_rv32,
        SignExt(divuw_res, 32, 64),
        divu_res(63, 0)
      ),
      FuOP.DivRemu.asUInt -> Mux(
        io.div_req.bits.is_rv32,
        SignExt(remuw_res, 32, 64),
        remu_res(63, 0)
      )
    )
  )
  require(div_rem_res.getWidth == 64)

  val div_by_zero_res = MuxLookup(
    io.div_req.bits.op_type.asUInt,
    0.U
  )(
    Seq(
      FuOP.DivDiv.asUInt -> GenMaskOne(64, 64),
      FuOP.DivRem.asUInt -> op_a,
      FuOP.DivDivu.asUInt -> Mux(
        io.div_req.bits.is_rv32,
        GenMaskOne(64, 32),
        GenMaskOne(64, 64)
      ),
      FuOP.DivRemu.asUInt -> op_a
    )
  )

  val overflow_res = MuxLookup(
    io.div_req.bits.op_type.asUInt,
    0.U
  )(
    Seq(
      FuOP.DivDiv.asUInt -> Mux(
        io.div_req.bits.is_rv32,
        GenMaskZero(64, 32 - 1),
        GenMaskZero(64, 64 - 1)
      ),
      FuOP.DivRem.asUInt -> 0.U
    )
  )

  val signed_op = VecInit(
    Seq(
      FuOP.DivDiv.asUInt,
      FuOP.DivRem.asUInt
    )
  )

  val final_res = Wire(UInt(64.W))
  when(Mux(io.div_req.bits.is_rv32, op_b(31, 0) === 0.U, op_b === 0.U)) {
    final_res := div_by_zero_res
  }.elsewhen(
    signed_op.contains(io.div_req.bits.op_type.asUInt) &&
      op_a === Mux(
        io.div_req.bits.is_rv32,
        GenMaskZero(64, 32 - 1),
        GenMaskZero(64, 64 - 1)
      ) &&
      op_b === GenMaskOne(64, 64)
  ) {
    final_res := overflow_res
  }.otherwise {
    final_res := div_rem_res
  }

  val final_res_ext = Mux(
    io.div_req.bits.is_rv32,
    SignExt(final_res, 32, 64),
    final_res
  )

  val div_pipe_in = Wire(Decoupled(new DivResp))
  val div_pipe_out = PipeLine(div_pipe_in, io.flush)
  io.div_resp <> div_pipe_out

  // -------------------
  // fsm
  // -------------------

  val sIdle :: sBusy :: sBusy2 :: Nil = Enum(3)
  val state = RegInit(sIdle)

  div_pipe_in.noenq()
  io.div_req.ready := false.B
  switch(state) {
    is(sIdle) {
      when(!io.flush) {
        io.div_req.ready := div_pipe_in.ready
        when(io.div_req.fire) {
          div_pipe_in.bits.data := final_res_ext
          div_pipe_in.bits.trans_id := io.div_req.bits.trans_id
          div_pipe_in.valid := io.div_req.valid
          state := sBusy
        }
      }
    }
    is(sBusy) {
      state := sBusy2
    }

    is(sBusy2) {
      state := sIdle
    }
  }

  // -------------------
  // assert
  // -------------------
  when(io.div_req.fire) {
    assert(
      FuOP.is_div_rem(io.div_req.bits.op_type),
      "DummyDiv: op_type is not Div"
    )
  }
}

object gen_dummy_Div_verilog extends App {
  GenVerilogHelper(new DummyDiv)
}
