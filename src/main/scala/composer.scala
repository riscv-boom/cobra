package cobra

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._

import boom.ifu._
import boom.common._
import boom.util.{BoomCoreStringPrefix}


object ComposedPipelines {
  def swbpd(resp_in: BranchPredictionBankResponse, p: Parameters) = {
    val sw = Module(new SwBranchPredictorBank()(p))

    sw.io.resp_in(0) := resp_in

    (Seq(sw), sw.io.resp)
  }

  def tourney(resp_in: BranchPredictionBankResponse, p: Parameters) = {
    val btb = Module(new BTBBranchPredictorBank()(p))
    val gbim = Module(new HBIMBranchPredictorBank()(p))
    val lbim = Module(new HBIMBranchPredictorBank(CobraHBIMParams(useLocal=true))(p))
    val tourney = Module(new TourneyBranchPredictorBank()(p))
    val preds = Seq(lbim, btb, gbim, tourney)
    preds.map(_.io := DontCare)

    gbim.io.resp_in(0) := resp_in
    lbim.io.resp_in(0) := resp_in
    tourney.io.resp_in(0) := gbim.io.resp
    tourney.io.resp_in(1) := lbim.io.resp
    btb.io.resp_in(0)  := tourney.io.resp

    (preds, btb.io.resp)
  }

  def b2(resp_in: BranchPredictionBankResponse, p: Parameters) = {
    // gshare is just variant of TAGE with 1 table
    val gshare = Module(new TageBranchPredictorBank(
      CobraTageParams(tableInfo = Seq((256, 16, 7)))
    )(p))
    val btb = Module(new BTBBranchPredictorBank()(p))
    val bim = Module(new BIMBranchPredictorBank()(p))
    val preds = Seq(bim, btb, gshare)
    preds.map(_.io := DontCare)

    bim.io.resp_in(0)  := resp_in
    btb.io.resp_in(0)  := bim.io.resp
    gshare.io.resp_in(0) := btb.io.resp
    (preds, gshare.io.resp)
  }

  def tage(resp_in: BranchPredictionBankResponse, p: Parameters) = {
    val loop = Module(new LoopBranchPredictorBank()(p))
    val tage = Module(new TageBranchPredictorBank()(p))
    val btb = Module(new BTBBranchPredictorBank()(p))
    val bim = Module(new BIMBranchPredictorBank()(p))
    val ubtb = Module(new FAMicroBTBBranchPredictorBank()(p))
    val preds = Seq(loop, tage, btb, ubtb, bim)
    preds.map(_.io := DontCare)

    ubtb.io.resp_in(0)  := resp_in
    bim.io.resp_in(0)   := ubtb.io.resp
    btb.io.resp_in(0)   := bim.io.resp
    tage.io.resp_in(0)  := btb.io.resp
    loop.io.resp_in(0)  := tage.io.resp

    (preds, loop.io.resp)
  }
}

class ComposedBranchPredictorBank(implicit p: Parameters) extends BranchPredictorBank()(p)
{

  val (components, resp) = ComposedPipelines.swbpd(io.resp_in(0), p)
  io.resp := resp


  var metas = 0.U(1.W)
  var meta_sz = 0
  for (c <- components) {
    c.io.f0_valid  := io.f0_valid
    c.io.f0_pc     := io.f0_pc
    c.io.f0_mask   := io.f0_mask
    c.io.f1_ghist  := io.f1_ghist
    c.io.f1_lhist  := io.f1_lhist
    c.io.f3_fire   := io.f3_fire
    if (c.metaSz > 0) {
      metas = (metas << c.metaSz) | c.io.f3_meta(c.metaSz-1,0)
    }
    meta_sz = meta_sz + c.metaSz
  }
  require(meta_sz < bpdMaxMetaLength)
  io.f3_meta := metas


  var update_meta = io.update.bits.meta
  for (c <- components.reverse) {
    c.io.update := io.update
    c.io.update.bits.meta := update_meta
    update_meta = update_meta >> c.metaSz
  }

  val mems = components.map(_.mems).flatten

}
