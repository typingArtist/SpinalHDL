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

    /** AddressPhase
        Each input port owns a request buffer because the arbiter cannot deny a
        request from any port if it is busy handling traffic from another input.
        The request buffer accepts a request as if it was a slave, but delays the
        assertion of HREADYOUT until the arbitration has turned to that port, when
        the request can be finally terminated by the slave.
    */

    case class AddressPhase(withHREADY: Boolean) extends Bundle {
      val HREADY    = withHREADY generate Bool
      val HSEL      = Bool
      val HADDR     = UInt(ahbLite3Config.addressWidth bits)
      val HWRITE    = Bool
      val HSIZE     = Bits(3 bits)
      val HBURST    = Bits(3 bits)
      val HPROT     = Bits(4 bits)
      val HTRANS    = Bits(2 bits)
      val HMASTLOCK = Bool
    }

    val inputStages = io.inputs.map(input => new Area {
      // import: driven from outside of this area
      //   chosen: True during the address phase, at least in the last cycle of it
      val chosen = False

      val buffer = Reg(AddressPhase(withHREADY = false))
      buffer.HSEL.init(False)

      /** clear HSEL if this port is selected by the end of the address phase.
          This appears in any of these cases:
      1. there is a stored request in the buffer, so buffer.HSEL goes
         from True to False
      2. there is no stored request, so the request is accepted volley from
         the input. We keep buffer.HSEL at False
      3. there is no stored request and not even a non-IDLE request on any input, but
         this stage is selected, so keep buffer.HSEL at False
      */
      buffer.HSEL.clearWhen(chosen & io.output.HREADY)

      when(input.HREADY) {
        /** we don't store IDLE or BUSY. This is to avoid protocol errors at the
            beginning of a burst or HMASTLOCK sequence where arbitration might
            not succeed in the first cycle, which would lead to illegal non-zero
            wait state responses. Successive BUSY burst and IDLE HMASTLOCK transfers will
            stay together and will be forwarded to the slave */
        buffer.HSEL      := input.HSEL && (input.HTRANS =/= AhbLite3.IDLE) && (input.HTRANS =/= AhbLite3.BUSY)
        buffer.HADDR     := input.HADDR
        buffer.HWRITE    := input.HWRITE
        buffer.HSIZE     := input.HSIZE
        buffer.HBURST    := input.HBURST
        buffer.HPROT     := input.HPROT
        buffer.HTRANS    := input.HTRANS
        buffer.HMASTLOCK := input.HMASTLOCK
      }


      // export: used outside of this area
      val addressPhase = AddressPhase(withHREADY = true)
      val unInterruptible = addressPhase.HSEL && ((addressPhase.HTRANS === AhbLite3.SEQ) || (addressPhase.HTRANS === AhbLite3.BUSY) || addressPhase.HMASTLOCK)
      val requestTransfer = addressPhase.HREADY && addressPhase.HSEL && ((addressPhase.HTRANS === AhbLite3.NONSEQ) || unInterruptible)
      val dataPhaseActive = RegInit(False)

      dataPhaseActive.clearWhen(io.output.HREADYOUT).setWhen(chosen && addressPhase.HREADY && addressPhase.HSEL)

      addressPhase.HREADY    := Mux(buffer.HSEL, True,             input.HREADY)
      addressPhase.HSEL      := Mux(buffer.HSEL, True,             input.HSEL)
      addressPhase.HADDR     := Mux(buffer.HSEL, buffer.HADDR,     input.HADDR)
      addressPhase.HWRITE    := Mux(buffer.HSEL, buffer.HWRITE,    input.HWRITE)
      addressPhase.HSIZE     := Mux(buffer.HSEL, buffer.HSIZE,     input.HSIZE)
      addressPhase.HBURST    := Mux(buffer.HSEL, buffer.HBURST,    input.HBURST)
      addressPhase.HPROT     := Mux(buffer.HSEL, buffer.HPROT,     input.HPROT)
      addressPhase.HTRANS    := Mux(buffer.HSEL, buffer.HTRANS,    input.HTRANS)
      addressPhase.HMASTLOCK := Mux(buffer.HSEL, buffer.HMASTLOCK, input.HMASTLOCK)

      // data return path
      input.HRDATA    := io.output.HRDATA
      input.HRESP     := Mux(dataPhaseActive, io.output.HRESP,     False)
      input.HREADYOUT := Mux(dataPhaseActive, io.output.HREADYOUT, !buffer.HSEL)
    })

    val dataPhasePortMap = inputStages.map(_.dataPhaseActive)
    val dataPhaseIndex = OHToUInt(dataPhasePortMap)
    val dataPhaseValid = dataPhasePortMap.asBits.asUInt =/= 0



    // priority for uninterruptible requests
    val selectNewPort = False
    val dataPhaseUnInterruptible = Vec(inputStages.map(_.unInterruptible))(dataPhaseIndex)
    when(!dataPhaseValid) {
      selectNewPort := True
    } elsewhen(dataPhaseUnInterruptible) {
      selectNewPort := False
    } elsewhen(io.output.HREADYOUT) {
      selectNewPort := True
    }
    // val selectNewPort = !dataPhaseValid || (io.output.HREADYOUT && !dataPhaseUnInterruptible)
    val requestIndex = dataPhaseIndex

    when(selectNewPort){
      // the actual arbiter
      val requests = inputStages.map(_.requestTransfer).asBits
      if(roundRobinArbiter)
        maskProposal := OHMasking.roundRobin(requests, maskLocked(maskLocked.high - 1 downto 0) ## maskLocked.msb)
      else
        maskProposal := OHMasking.first(requests)

      // address forward path
      val requestIndex     = OHToUInt(maskRouted)
    }

    val chosen = Vec(inputStages.map(_.chosen))(requestIndex)
    chosen := True

    // address path
    val addressPhase = Vec(inputStages.map(_.addressPhase))(requestIndex)
    io.output.HSEL      := addressPhase.HSEL
    io.output.HREADY    := addressPhase.HREADY
    io.output.HADDR     := addressPhase.HADDR
    io.output.HWRITE    := addressPhase.HWRITE
    io.output.HSIZE     := addressPhase.HSIZE
    io.output.HBURST    := addressPhase.HBURST
    io.output.HPROT     := addressPhase.HPROT
    io.output.HTRANS    := addressPhase.HTRANS
    io.output.HMASTLOCK := addressPhase.HMASTLOCK

    // data forward path
    io.output.HWDATA    := MuxOH(dataPhasePortMap, io.inputs.map(_.HWDATA))

    def myRoundRobin(requests: Bits, lastMask : Bits) = {
      val ret = Bits(requests.getLength bits)
      val doubleRequests = requests ## requests
      val masking = Bits(2*requests.getLength bits)

      OHMasking.first()
    }

    when(!dataPhaseValid || io.output.HREADYOUT) {
      dataPhaseIndex := requestIndex
      dataPhaseValid := io.output.HSEL && io.output.HREADY
    }

    val dataPhaseActive = RegNextWhen(io.output.HTRANS(1), io.output.HREADY) init(False)
    val locked          = RegInit(False)
    val maskProposal    = Bits(inputsCount bits)
    val maskLocked      = Reg(Bits(inputsCount bits)) init(BigInt(1) << (inputsCount - 1))
    val maskRouted      = Mux(locked || dataPhaseActive, maskLocked, maskProposal)

    when(io.output.HSEL) { //valid
      maskLocked := maskRouted
      locked     := True

      when(io.output.HREADY){ //fire
        when(io.output.last && !io.output.HMASTLOCK) { //End of burst and no lock
          locked := False
        }
      }
    }

    // the actual arbiter
    val requests = io.inputs.map(_.HSEL).asBits
    if(roundRobinArbiter)
      maskProposal := OHMasking.roundRobin(requests, maskLocked(maskLocked.high - 1 downto 0) ## maskLocked.msb)
    else
      maskProposal := OHMasking.first(requests)

    // address forward path
    val requestIndex     = OHToUInt(maskRouted)
    io.output.HSEL      := (io.inputs, maskRouted.asBools).zipped.map(_.HSEL & _).reduce(_ | _)
    io.output.HADDR     := io.inputs(requestIndex).HADDR
    io.output.HREADY    := io.inputs(requestIndex).HREADY
    io.output.HWRITE    := io.inputs(requestIndex).HWRITE
    io.output.HSIZE     := io.inputs(requestIndex).HSIZE
    io.output.HBURST    := io.inputs(requestIndex).HBURST
    io.output.HPROT     := io.inputs(requestIndex).HPROT
    io.output.HTRANS    := io.output.HSEL ? io.inputs(requestIndex).HTRANS | B"00"
    io.output.HMASTLOCK := io.inputs(requestIndex).HMASTLOCK

    val dataIndex        = RegNextWhen(requestIndex, io.output.HSEL && io.output.HREADY)
    io.output.HWDATA    := io.inputs(dataIndex).HWDATA

    for((input,requestRouted) <- (io.inputs,maskRouted.asBools).zipped){
      input.HRDATA    := io.output.HRDATA
      input.HRESP     := io.output.HRESP
      input.HREADYOUT := (!requestRouted && !input.HSEL) || (requestRouted && io.output.HREADYOUT)
    }

  }
}
