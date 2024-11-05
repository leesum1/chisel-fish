package leesum.muldiv
import chisel3._
import chisel3.util.{Decoupled, RRArbiter}
import leesum.GenVerilogHelper

class FuMulDivResp extends Bundle {
  val data = UInt(64.W)
  val trans_id = UInt(32.W)
}

class FuMulDiv(en: Boolean = true) extends Module {
  val io = IO(new Bundle {
    val mul_req = Flipped(Decoupled(new MulReq))
    val div_req = Flipped(Decoupled(new DivReq))

    val fu_div_mul_resp = Decoupled(new FuMulDivResp)

    val flush = Input(Bool())
  })

  if (en) {
    val dummy_mul = Module(new DummyMul)
    val dummy_div = Module(new DummyDiv)
    val mul_resp = Wire(Decoupled(new MulResp))
    val div_resp = Wire(Decoupled(new DivResp))
    dummy_mul.io.mul_req <> io.mul_req
    dummy_mul.io.mul_resp <> mul_resp
    dummy_mul.io.flush := io.flush
    dummy_div.io.div_req <> io.div_req
    dummy_div.io.div_resp <> div_resp
    dummy_div.io.flush := io.flush
    val resp_arb = Module(new RRArbiter(new FuMulDivResp, 2))
    resp_arb.io.in(0) <> mul_resp
    resp_arb.io.in(1) <> div_resp
    resp_arb.io.out <> io.fu_div_mul_resp
  } else {
    io.fu_div_mul_resp.valid := false.B
    io.fu_div_mul_resp.bits := DontCare
    io.mul_req.ready := false.B
    io.div_req.ready := false.B
  }
}

object gen_fu_mul_div_verilog extends App {
  GenVerilogHelper(new FuMulDiv)
}
