/* -*- c++ -*- */
/*
 * Copyright 2014 <+YOU OR YOUR COMPANY+>.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <cmath>
#include <sys/ioctl.h>
#include "crash_usrp_source_impl.h"

namespace gr {
  // FIXME: Hack to prevent multiple CRASH instances from initializing the USRP repeatedly
  static int g_crash_init = 0;

  namespace crash {

    crash_usrp_source::sptr
    crash_usrp_source::make(const int xfer_size, const int decim_rate)
    {
      return gnuradio::get_initial_sptr
        (new crash_usrp_source_impl(xfer_size, decim_rate));
    }

    /*
     * Constructor
     */
    crash_usrp_source_impl::crash_usrp_source_impl(const int xfer_size, const int decim_rate)
      : gr::sync_block("crash_usrp_source",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, sizeof(gr_complex)))
    {
      double gain = 0.0;

      usrp_intf = crash_open(USRP_INTF_PLBLOCK_ID, READ);
      if (usrp_intf == 0) {
        throw std::runtime_error("Failed to allocate USRP interface plblock (Source)!\n");
        exit(-1);
      }

      if (g_crash_init == 0) {
        // Global Reset
        crash_reset(usrp_intf);

        // Wait for USRP DDR interface to finish calibrating (due to reset). This is necessary
        // as the next steps recalibrate the interface and are ignored if issued while it is
        // currently calibrating.
        while(!crash_get_bit(usrp_intf->regs,USRP_RX_CAL_COMPLETE));
        while(!crash_get_bit(usrp_intf->regs,USRP_TX_CAL_COMPLETE));

        // Set RX phase
        crash_write_reg(usrp_intf->regs,USRP_RX_PHASE_INIT,RX_PHASE_CAL);
        crash_set_bit(usrp_intf->regs,USRP_RX_RESET_CAL);
        printf("RX PHASE INIT: %d\n",crash_read_reg(usrp_intf->regs,USRP_RX_PHASE_INIT));
        while(!crash_get_bit(usrp_intf->regs,USRP_RX_CAL_COMPLETE));

        // Set TX phase
        crash_write_reg(usrp_intf->regs,USRP_TX_PHASE_INIT,TX_PHASE_CAL);
        crash_set_bit(usrp_intf->regs,USRP_TX_RESET_CAL);
        printf("TX PHASE INIT: %d\n",crash_read_reg(usrp_intf->regs,USRP_TX_PHASE_INIT));
        while(!crash_get_bit(usrp_intf->regs,USRP_TX_CAL_COMPLETE));

        // Set USRP Mode
        while(crash_get_bit(usrp_intf->regs,USRP_UART_BUSY));
        crash_write_reg(usrp_intf->regs,USRP_USRP_MODE_CTRL,TX_DAC_RAW_MODE + RX_ADC_DSP_MODE);
        while(crash_get_bit(usrp_intf->regs,USRP_UART_BUSY));

        g_crash_init = 1;
      }

      number_samples = (uint)xfer_size;

      crash_write_reg(usrp_intf->regs, USRP_AXIS_MASTER_TDEST, DMA_PLBLOCK_ID);   // Set tdest to ps_pl_interface
      crash_write_reg(usrp_intf->regs, USRP_RX_PACKET_SIZE, number_samples);      // Set packet size
      crash_clear_bit(usrp_intf->regs, USRP_RX_FIX2FLOAT_BYPASS);                 // Do not bypass fix2float
      if ((decim_rate == 0) || (decim_rate > 2047)) {
        throw std::runtime_error("Invalid decimation rate!\n");
        exit(-1);
      } else if (decim_rate == 1) {
        crash_set_bit(usrp_intf->regs, USRP_RX_CIC_BYPASS);                       // Bypass CIC Filter
        crash_set_bit(usrp_intf->regs, USRP_RX_HB_BYPASS);                        // Bypass HB Filter
        crash_write_reg(usrp_intf->regs, USRP_RX_GAIN, 1);                        // Set gain = 1
      } else if (decim_rate == 2) {
        crash_set_bit(usrp_intf->regs, USRP_RX_CIC_BYPASS);                       // Bypass CIC Filter
        crash_clear_bit(usrp_intf->regs, USRP_RX_HB_BYPASS);                      // Enable HB Filter
        crash_write_reg(usrp_intf->regs, USRP_RX_GAIN, 1);                        // Set gain = 1
      // Even, use both CIC and HB filters
      } else if ((decim_rate % 2) == 0) {
        crash_clear_bit(usrp_intf->regs, USRP_RX_CIC_BYPASS);                     // Enable CIC Filter
        crash_write_reg(usrp_intf->regs, USRP_RX_CIC_DECIM, decim_rate/2);        // Set CIC decimation rate (div by 2 as we are using HB filter)
        crash_clear_bit(usrp_intf->regs, USRP_RX_HB_BYPASS);                      // Enable HB Filter
        // Offset CIC bit growth. A 32-bit multiplier in the receive chain allows us
        // to scale the CIC output.
        gain = 32.0-3.0*log2(decim_rate/2);
        gain = (gain > 1.0) ? (ceil(pow(2.0,gain))) : (1.0);                      // Do not allow gain to be set to 0
        crash_write_reg(usrp_intf->regs, USRP_RX_GAIN, (uint32_t)gain);           // Set gain
      // Odd, use only CIC filter
      } else {
        crash_clear_bit(usrp_intf->regs, USRP_RX_CIC_BYPASS);                     // Enable CIC Filter
        crash_write_reg(usrp_intf->regs, USRP_RX_CIC_DECIM, decim_rate);          // Set CIC decimation rate
        crash_set_bit(usrp_intf->regs, USRP_RX_HB_BYPASS);                        // Bypass HB Filter
        //
        gain = 32.0-3.0*log2(decim_rate);
        gain = (gain > 1.0) ? (ceil(pow(2.0,gain))) : (1.0);                      // Do not allow gain to be set to 0
        crash_write_reg(usrp_intf->regs, USRP_RX_GAIN, (uint32_t)gain);           // Set gain
      }

      crash_set_bit(usrp_intf->regs, USRP_RX_ENABLE);                             // Enable RX
    }

    /*
     * Destructor
     */
    crash_usrp_source_impl::~crash_usrp_source_impl()
    {
      crash_clear_bit(usrp_intf->regs, USRP_RX_ENABLE);                           // Disable RX
      crash_close(usrp_intf);
    }

    int
    crash_usrp_source_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      gr_complex *out = (gr_complex *) output_items[0];
      int i = 0;
      float *sample = (float*)(usrp_intf->dma_buff);

      if (noutput_items < number_samples) {
        return 0;
      }

      if(crash_read(usrp_intf, USRP_INTF_PLBLOCK_ID, number_samples)) {
        throw std::runtime_error("DMA timeout!\n");
      }

      // Copy from DMA buffer to output buffer.
      // TODO: GNU Radio is in process of allowing us to specify our own output buffers, which will allow us to make
      //       this zero copy.
      for(i = 0; i < number_samples; i++) {
        out[i] = gr_complex(sample[2*i+1],sample[2*i]);
      }

      return number_samples;
    }

  } /* namespace crash */
} /* namespace gr */

