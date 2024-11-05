package leesum.muldiv

import chisel3._
import chisel3.util.{Cat, Decoupled, Enum, MuxLookup, is, switch}
import leesum.PipeLine
import leesum.FuOP
import leesum.GenVerilogHelper
import leesum.SignExt

class MulReq extends Bundle {
  private val mul_width = 64
  val op_a = UInt(mul_width.W)
  val op_b = UInt(mul_width.W)
  val trans_id = UInt(32.W)
  val op_type = UInt(FuOP.width.W)
  val is_rv32 = Bool()
}

class MulResp extends FuMulDivResp {}

class DummyMul extends Module {
  val io = IO(new Bundle {
    val mul_req = Flipped(Decoupled(new MulReq))
    val mul_resp = Decoupled(new MulResp)
    val flush = Input(Bool())
  })

  val sign_info = FuOP.get_mul_signed_info(io.mul_req.bits.op_type)
  val op_a_ext =
    Cat(io.mul_req.bits.op_a(63) & sign_info(0), io.mul_req.bits.op_a)
  val op_b_ext =
    Cat(io.mul_req.bits.op_b(63) & sign_info(1), io.mul_req.bits.op_b)
  require(op_a_ext.getWidth == 65);
  require(op_b_ext.getWidth == 65);

  val mul_res_128 = op_a_ext.asSInt * op_b_ext.asSInt

  val mul_res = MuxLookup(
    io.mul_req.bits.op_type.asUInt,
    0.U
  )(
    Seq(
      FuOP.MulMul.asUInt -> Mux(
        io.mul_req.bits.is_rv32,
        SignExt(mul_res_128(31, 0), 32, 64),
        mul_res_128(63, 0)
      ),
      FuOP.MulMulh.asUInt -> mul_res_128(127, 64),
      FuOP.MulMulhsu.asUInt -> mul_res_128(127, 64),
      FuOP.MulMulhu.asUInt -> mul_res_128(127, 64)
    )
  )

  val mul_pipe_in = Wire(Decoupled(new MulResp))
  val mul_pipe_out = PipeLine(mul_pipe_in, io.flush)
  io.mul_resp <> mul_pipe_out

  mul_pipe_in.noenq()
  io.mul_req.ready := false.B

  // -------------------
  // fsm
  // -------------------
  val sIdle :: sBusy :: Nil = Enum(2)
  val state = RegInit(sIdle)

  switch(state) {
    is(sIdle) {
      when(!io.flush) {
        io.mul_req.ready := mul_pipe_in.ready
        when(io.mul_req.fire) {
          mul_pipe_in.bits.data := mul_res
          mul_pipe_in.bits.trans_id := io.mul_req.bits.trans_id
          mul_pipe_in.valid := io.mul_req.valid
          state := sBusy
        }
      }
    }
    is(sBusy) {
      state := sIdle
    }
  }

  // -------------------
  // assert
  // -------------------

  when(io.mul_req.fire) {
    assert(FuOP.is_mul(io.mul_req.bits.op_type), "DummyMul: op_type is not mul")
  }
}

object gen_dummy_mul_verilog extends App {
  GenVerilogHelper(new DummyMul)
}
