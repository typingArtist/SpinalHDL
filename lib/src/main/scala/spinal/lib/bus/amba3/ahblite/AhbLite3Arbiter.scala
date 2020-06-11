/*                                                                           *\
**        _____ ____  _____   _____    __                                    **
**       / ___// __ \/  _/ | / /   |  / /   HDL Lib                          **
**       \__ \/ /_/ // //  |/ / /| | / /    (c) Dolu, All rights reserved    **
**      ___/ / ____// // /|  / ___ |/ /___                                   **
**     /____/_/   /___/_/ |_/_/  |_/_____/  MIT Licence                      **
**                                                                           **
** Permission is hereby granted, free of charge, to any person obtaining a   **
** copy of this software and associated documentation files (the "Software"),**
** to deal in the Software without restriction, including without limitation **
** the rights to use, copy, modify, merge, publish, distribute, sublicense,  **
** and/or sell copies of the Software, and to permit persons to whom the     **
** Software is furnished to do so, subject to the following conditions:      **
**                                                                           **
** The above copyright notice and this permission notice shall be included   **
** in all copies or substantial portions of the Software.                    **
**                                                                           **
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS   **
** OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF                **
** MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.    **
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY      **
** CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT **
** OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR  **
** THE USE OR OTHER DEALINGS IN THE SOFTWARE.                                **
\*                                                                           */
package spinal.lib.bus.amba3.ahblite

import spinal.core._
import spinal.lib._

/* don’t publish this supplemental class  */
case class AhbLite3AddrPhase(config: AhbLite3Config) extends Bundle{
  val HSEL      = Bool
  val HADDR     = UInt(config.addressWidth bits)
  val HWRITE    = Bool
  val HSIZE     = Bits(3 bits)
  val HBURST    = Bits(3 bits)
  val HPROT     = Bits(4 bits)
  val HTRANS    = Bits(2 bits)
  val HMASTLOCK = Bool

  def assignFromBus(bus: AhbLite3): Unit={
    HSEL      := True
    HADDR     := bus.HADDR
    HWRITE    := bus.HWRITE
    HSIZE     := bus.HSIZE
    HBURST    := bus.HBURST
    HPROT     := bus.HPROT
    HTRANS    := bus.HTRANS
    HMASTLOCK := bus.HMASTLOCK
  }

  def assignToBus(fromMaster: AhbLite3, toSlave: AhbLite3): Unit = {
    when(HSEL) {
      toSlave.HREADY     := True
      toSlave.HSEL       := HSEL
      toSlave.HADDR      := HADDR
      toSlave.HWRITE     := HWRITE
      toSlave.HSIZE      := HSIZE
      toSlave.HBURST     := HBURST
      toSlave.HPROT      := HPROT
      toSlave.HTRANS     := HTRANS
      toSlave.HMASTLOCK  := HMASTLOCK
    } otherwise {
      toSlave.HREADY     := fromMaster.HREADY
      toSlave.HSEL       := fromMaster.HSEL
      toSlave.HADDR      := fromMaster.HADDR
      toSlave.HWRITE     := fromMaster.HWRITE
      toSlave.HSIZE      := fromMaster.HSIZE
      toSlave.HBURST     := fromMaster.HBURST
      toSlave.HPROT      := fromMaster.HPROT
      toSlave.HTRANS     := fromMaster.HTRANS
      toSlave.HMASTLOCK  := fromMaster.HMASTLOCK
    }
  }
}

case class AhbLite3AddrPhaseFull(config: AhbLite3Config) extends AhbLite3AddrPhase(config) {
  val HREADY    = Bool
  val HSEL      = Bool

  def assignFromBus(bus: AhbLite3): Unit = {
    HREADY := bus.HREADY
    HSEL   := bus.HSEL
    super.assignFromBus(bus)
  }
}

/**
  * AHB Lite arbiter
  *
  * @param ahbLite3Config : Ahb bus configuration
  * @param inputsCount    : Number of inputs for the arbiter
  */
case class AhbLite3Arbiter(ahbLite3Config: AhbLite3Config, inputsCount: Int, roundRobinArbiter : Boolean = true) extends Component {

  val io = new Bundle {
    val inputs = Vec(slave(AhbLite3(ahbLite3Config)), inputsCount)
    val output = master(AhbLite3(ahbLite3Config))
  }

  val logic = if(inputsCount == 1) new Area {

    io.output << io.inputs.head

  }else new Area{

    /** on an input, the following functionality is important:
        * Accept and terminate IDLE requests. They should be terminated right on the input and
          shouldn't hit the output. If the output is selected, the IDLE requests
          of one of the inputs could hit the output, but there is little to no
          benefit in doing that instead of terminating all these requests on the
          input.
        * Accept NONSEQ requests (HREADY & HSEL & HTRANS == AhbLite3.NONSEQ). If
          the port is selected, forward the request. If either HMASTLOCK is asserted
          or it is to
          and
     */

    val dataPhaseActive = RegNextWhen(io.output.HTRANS(1), io.output.HREADY) init(False)
    val locked          = RegInit(False)
    val maskProposal    = Bits(inputsCount bits)
    val maskLocked      = Reg(Bits(inputsCount bits)) init(BigInt(1) << (inputsCount - 1))
    val maskRouted      = Mux(locked || dataPhaseActive, maskLocked, maskProposal)
    val requestIndex    = OHToUInt(maskRouted)

    /** Backup address phase */
    val addressPhasesStore = Vec(Reg(AhbLite3AddrPhase(ahbLite3Config)), inputsCount)

    val addressPhasesStore = io.inputs.map { input =>
      val store = Reg(Ahblite3AddrPhase(ahbLite3Config))
      store.HREADY.init(False)

      when(input.HREADY) {
        store.assignFromBus(input)
        when(input.HSEL & (input.HTRANS === Ahblite3.NONSEQ || input.HTRANS === AhbLite3.SEQ)) {

        }
      }

      store
    }

    val (addressPhasesStore, addressPhasesMuxed) = io.inputs.map { input =>
      val inputAddrData = AhbLite3AddrPhase(ahbLite3Config)
      inputAddrData.assignFromBus(input)
      inputAddrData.HREADY := input.HREADY

      val store = Reg(Ahblite3AddrPhase(ahbLite3Config))
      store.HREADY.init(False)
      val muxed = mux(store.HREADY, store, Ahblite3AddrPhase(ahbLite3Config).assignFromBus(input))
      val muxed = Ahblite3AddrPhase(ahbLite3Config)

      when(input.HREADY) {
        store.assignFromBus(input)
        when(input.HSEL & (input.HTRANS === Ahblite3.NONSEQ || input.HTRANS === AhbLite3.SEQ)) {

        }
      }


      (store, muxed)
    }

    val addressPhasesMuxed = Vec(AhbLite3AddrPhase(ahbLite3Config)), inputsCount)
    val addressPhaseData  = Vec(Reg(AhbLite3AddrPhase(ahbLite3Config)), io.inputs.length)
    val addressPhaseValid = Vec(RegInit(False), io.inputs.length)

    //
    for((input, index) <- io.inputs.zipWithIndex){
      when(input.HREADY){
        when(input.HTRANS === AhbLite3.IDLE || input.HTRANS === AhbLite3.BUSY) {
          // just terminate early.
          input.
        }
        addressPhaseData(index).assignFromBus(io.inputs(index))
        addressPhaseValid(index) := input.HSEL & (input.HTRANS === AhbLite3.NONSEQ || input.HTRANS === AhbLite3.SEQ)
      }
    }

    when(io.output.HSEL & io.output.HTRANS(1)) { // valid transaction
      maskLocked := maskRouted
      locked     := True
      addressPhaseValid(requestIndex) := False

      when(io.output.HREADY & io.output.last && !io.output.HMASTLOCK) { // End of burst and no lock
        locked := False
      }
    }

    /** Arbiter logic */
    val transactionOnHold = addressPhaseValid.reduce(_ || _)
    val requests = Mux(transactionOnHold, addressPhaseValid.asBits, io.inputs.map(bus => bus.HSEL & bus.HTRANS(1)).asBits)

    if(roundRobinArbiter) {
      maskProposal := OHMasking.roundRobin(requests, maskLocked(maskLocked.high - 1 downto 0) ## maskLocked.msb)
    }else{
      maskProposal := OHMasking.first(requests)
    }

    /** Multiplexer */
    val bufferAddrEnable = addressPhaseValid(requestIndex)

    io.output.HSEL      := bufferAddrEnable  ? True                                     | io.inputs(requestIndex).HSEL
    io.output.HADDR     := bufferAddrEnable  ? addressPhaseData(requestIndex).HADDR     | io.inputs(requestIndex).HADDR
    io.output.HREADY    := bufferAddrEnable  ? True                                     | io.inputs(requestIndex).HREADY
    io.output.HWRITE    := bufferAddrEnable  ? addressPhaseData(requestIndex).HWRITE    | io.inputs(requestIndex).HWRITE
    io.output.HSIZE     := bufferAddrEnable  ? addressPhaseData(requestIndex).HSIZE     | io.inputs(requestIndex).HSIZE
    io.output.HBURST    := bufferAddrEnable  ? addressPhaseData(requestIndex).HBURST    | io.inputs(requestIndex).HBURST
    io.output.HPROT     := bufferAddrEnable  ? addressPhaseData(requestIndex).HPROT     | io.inputs(requestIndex).HPROT
    io.output.HTRANS    := bufferAddrEnable  ? addressPhaseData(requestIndex).HTRANS    | io.inputs(requestIndex).HTRANS
    io.output.HMASTLOCK := bufferAddrEnable  ? addressPhaseData(requestIndex).HMASTLOCK | io.inputs(requestIndex).HMASTLOCK

    val dataIndex        = RegNextWhen(requestIndex, io.output.HSEL && io.output.HREADY)
    io.output.HWDATA    := io.inputs(dataIndex).HWDATA

    /** Drive input response signals  */
    for((input, requestRouted, onHold) <- (io.inputs, maskRouted.asBools, addressPhaseValid).zipped){

      val hreadyOut  = RegInit(True)

      when(!requestRouted & input.HSEL & input.HTRANS(1)){
        hreadyOut := False
      } elsewhen (requestRouted || !onHold){
        hreadyOut := True
      }

      input.HRDATA    := io.output.HRDATA
      input.HRESP     := io.output.HRESP & requestRouted
      input.HREADYOUT := (hreadyOut && io.output.HREADYOUT) || (hreadyOut && !requestRouted)
    }

  }
}
