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
import spinal.lib.bus.misc.SizeMapping


/**
  * Default Slave
  * Return an error when an operation occurs
  */
class DefaultAhbLite3Slave(config: AhbLite3Config) extends Component{

  val io = slave(AhbLite3(config))

  io.HREADYOUT := RegNext(True) init(True)
  io.HRESP     := RegNext(!io.HREADYOUT) init(False)
  io.HRDATA    := 0

  when(io.HREADY & io.HSEL) {
    io.HREADYOUT := io.isIdle
    io.HRESP     := !io.isIdle
  }
}



object AhbLite3Decoder{

  def apply(ahbLite3Config: AhbLite3Config, decodings: Seq[SizeMapping]): AhbLite3Decoder = {
    new AhbLite3Decoder(ahbLite3Config, decodings)
  }

  /**
    * @example {{{
    *
    *     val decoder = AhbLite3Decoder(
    *                      master = io.master_ahb,
    *                      slaves = List(
    *                           io.slave_ahb_1 -> (0x00000000, 1kB),
    *                           io.slave_ahb_2 -> (0x10000000, 1kB),
    *                           io.slave_ahb_3 -> (0x20000000, 1kB)
    *                      ))
    *         }}}
    *
    * A default slave can be added as follow
    *
    * @example {{{
    *
    *     val decoder = AhbLite3Decoder(
    *                      master = io.master_ahb,
    *                      slaves = List(
    *                           io.slave_ahb_1 -> (0x00000000, 1kB),
    *                           io.slave_ahb_2 -> (0x10000000, 1kB),
    *                           io.slave_ahb_3 -> (0x20000000, 1kB)
    *                      ),
    *                      defaultSlave = myDefautlSlave.io.ahb
    *                      )
    *         }}}
    *
    */
  def apply(master: AhbLite3, slaves: Seq[(AhbLite3, SizeMapping)], defaultSlave: AhbLite3 = null): AhbLite3Decoder = {

    val decoder = new AhbLite3Decoder(master.config, slaves.map(_._2), defaultSlave != null)

    decoder.io.input << master
    (slaves.map(_._1), decoder.io.outputs).zipped.map(_ << _)

    if(defaultSlave != null) defaultSlave << decoder.io.defaultSlave

    decoder
  }

}



/**
  * AHB lite decoder
  *
  * @param ahbLite3Config : AHB bus configuration
  * @param decodings      : Mapping list for all outputs
  */
class AhbLite3Decoder(ahbLite3Config: AhbLite3Config, decodings: Seq[SizeMapping], addDefaultSlaveInterface: Boolean = false) extends Component {

  assert(!SizeMapping.verifyOverlapping(decodings), "AhbLite3Decoder : overlapping found")

  val io = new Bundle {
    val input        = slave(AhbLite3(ahbLite3Config))
    val outputs      = Vec(master(AhbLite3(ahbLite3Config)), decodings.size)
    val defaultSlave = if(addDefaultSlaveInterface) master(AhbLite3(ahbLite3Config)).setPartialName("defaultSlave") else null
  }

  val defaultSlave = if(addDefaultSlaveInterface) null else new DefaultAhbLite3Slave(ahbLite3Config)

  // add the default slave to the output list
  def outputs : List[AhbLite3] = io.outputs.toList ++ List(if(addDefaultSlaveInterface) io.defaultSlave else defaultSlave.io)

  val decodesSlaves       = decodings.map(_.hit(io.input.HADDR)).asBits
  val decodeDefaultSlave  = decodesSlaves === 0

  val decodedSels  = decodeDefaultSlave ## decodesSlaves // !! reverse order compare to def outputs

  for((output, sel) <- (outputs, decodedSels.asBools).zipped){
    output.HREADY    := io.input.HREADY
    output.HSEL      := sel & io.input.HSEL
    output.HADDR     := io.input.HADDR
    output.HWRITE    := io.input.HWRITE
    output.HSIZE     := io.input.HSIZE
    output.HBURST    := io.input.HBURST
    output.HPROT     := io.input.HPROT
    output.HTRANS    := io.input.HTRANS
    output.HMASTLOCK := io.input.HMASTLOCK
    output.HWDATA    := io.input.HWDATA
  }

  val requestIndex = OHToUInt(outputs.map(_.HSEL))
  val dataIndex    = RegNextWhen(requestIndex, io.input.HREADY)

  val io.input.HRDATA    = outputs(dataIndex).HRDATA
  val io.input.HRESP     = outputs(dataIndex).HRESP
  val io.input.HREADYOUT = outputs(dataIndex).HREADYOUT
}
