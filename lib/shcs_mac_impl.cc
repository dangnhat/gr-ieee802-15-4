/* -*- c++ -*- */
/* 
 * Copyright 2017 Real-time and Embedded Systems Laboratory, KAIST, South Korea.
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

/**
 * @file shcs_mac_impl.cpp
 * @author  Nhat Pham <nhatphd@kaist.ac.kr>, Real-time and Embedded Systems Lab
 * @version 1.0
 * @date 2017-03-13
 * @brief This is the implementation for SHCS MAC protocol.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/block_detail.h>
#include <gnuradio/thread/thread.h>
#include <boost/random/uniform_int.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/linear_congruential.hpp>
#include <iostream>
#include <iomanip>

#include "shcs_mac_impl.h"
#include "shcs_ieee802154.h"
#include "shcs_tasks_processor_base.hpp"
#include "shcs_tasks_processor_timers.hpp"

using namespace gr::ieee802_15_4;
using namespace tp_timers;
using namespace std;

#define dout d_debug && cout

// Part of tasks_processor class from
// tasks_processor_timers.hpp, that must be defined
// Somewhere in source file
tasks_processor&
tasks_processor::get ()
{
  static tasks_processor proc;
  return proc;
}

/*------------------------------------------------------------------------*/
shcs_mac::sptr
shcs_mac::make (bool debug, int nwk_dev_type, std::vector<uint8_t> mac_addr,
                int suc_id, int assoc_suc_id, int fft_len)
{
  return gnuradio::get_initial_sptr (
      new shcs_mac_impl (debug, nwk_dev_type, mac_addr, suc_id, assoc_suc_id,
                         fft_len));
}

/*------------------------------------------------------------------------*/
shcs_mac_impl::shcs_mac_impl (bool debug, int nwk_dev_type,
                              std::vector<uint8_t> mac_addr, int suc_id,
                              int assoc_suc_id, int fft_len) :
    sync_block ("shcs_mac",
                gr::io_signature::make (1, 1, sizeof(float) * fft_len),
                gr::io_signature::make (0, 0, 0)), d_msg_offset (0), d_seq_nr (
        0), d_debug (debug), d_num_packet_errors (0), d_num_packets_received (
        0), d_nwk_dev_type (nwk_dev_type), d_suc_id (suc_id), d_assoc_suc_id (
        assoc_suc_id), d_fft_len (fft_len)
{
  /* Print hello message and time stamp */
  dout << "Hello, this is SHCS MAC protocol implementation, version 0.3"
      << endl;
  if (nwk_dev_type == SUC) {
    dout << "NWK device type: SU Coordinator" << endl;
    dout << "SUC ID: " << hex << d_suc_id << dec << endl;
  }
  else if (nwk_dev_type == SUR) {
    dout << "NWK device type: SU Router" << endl;
    dout << "SUC ID: " << hex << d_suc_id << dec << endl;
    dout << "Assoc. SUC ID: " << hex << d_assoc_suc_id << dec << endl;
  }
  else {
    dout << "NWK device type: SU" << endl;
    dout << "Assoc. SUC ID: " << hex << d_assoc_suc_id << dec << endl;
  }

  if (mac_addr.size () != 2)
    throw std::invalid_argument ("MAC address has to consist of two integers");
  d_mac_addr[0] = mac_addr[0];
  d_mac_addr[1] = mac_addr[1];
  dout << "MAC address: ";
  printf ("%x.%x\n", d_mac_addr[0], d_mac_addr[1]);

  /* Register message port from NWK Layer */
  message_port_register_in (pmt::mp ("app in"));
  set_msg_handler (pmt::mp ("app in"),
                   boost::bind (&shcs_mac_impl::app_in, this, _1));

  /* Register message port from PHY Layer */
  message_port_register_in (pmt::mp ("pdu in"));
  set_msg_handler (pmt::mp ("pdu in"),
                   boost::bind (&shcs_mac_impl::mac_in, this, _1));

  /* Register message port to NWK Layer */
  message_port_register_out (pmt::mp ("app out"));

  /* Register message port to PHY Layer */
  message_port_register_out (pmt::mp ("pdu out"));

  /* Register command message ports to USRP blocks */
  message_port_register_out (pmt::mp ("usrp sink cmd"));
  message_port_register_out (pmt::mp ("usrp source cmd"));

  /* Change dout to scientific mode */
  dout << setprecision (3);
  dout << scientific;

  /* Initialize channels list */
  for (int count = 1; count < num_of_channels; count++) {
    center_freqs[count] = center_freqs[count - 1] + channel_step;
  }

  dout << "List of operating channels and center freq.: " << endl;
  for (int count = 0; count < num_of_channels; count++) {
    dout << (count + first_channel_index) << ": " << center_freqs[count]
        << " Hz" << endl;
  }

  /* Create control threads */
  d_control_thread_state = NULL_STATE;

  if (nwk_dev_type == SUC) {
    /* Create control thread for Coordinator */
    control_thread_ptr = boost::shared_ptr<gr::thread::thread> (
        new gr::thread::thread (
            boost::bind (&shcs_mac_impl::suc_control_thread, this)));
  }
  else if (nwk_dev_type == SUR) {
    /* Create control thread for Router */
    control_thread_ptr = boost::shared_ptr<gr::thread::thread> (
        new gr::thread::thread (
            boost::bind (&shcs_mac_impl::sur_control_thread, this)));
  }
  else {
    /* Create control thread for SU */
    control_thread_ptr = boost::shared_ptr<gr::thread::thread> (
        new gr::thread::thread (
            boost::bind (&shcs_mac_impl::su_control_thread, this)));
  }

  /* Create transmission thread */
  transmit_thread_ptr = boost::shared_ptr<gr::thread::thread> (
      new gr::thread::thread (
          boost::bind (&shcs_mac_impl::transmit_thread, this)));

  /* Reporting thread */
  reporting_thread_ptr = boost::shared_ptr<gr::thread::thread> (
      new gr::thread::thread (
          boost::bind (&shcs_mac_impl::reporting_thread_func, this)));
}

/*------------------------------------------------------------------------*/
shcs_mac_impl::~shcs_mac_impl ()
{
  dout << "Destructor called." << endl;
}

/*------------------------------------------------------------------------*/
int
shcs_mac_impl::work (int noutput_items, gr_vector_const_void_star &input_items,
                     gr_vector_void_star &output_items)
{
//      dout << setprecision (3) << fixed << "work: " << noutput_items << endl;

  if (d_control_thread_state != SPECTRUM_SENSING) {
    /* Not in spectrum sensing state, drop inputs */
    return noutput_items;
  }

  /* In spectrum sensing state */
  double mean = 0, avg_power = 0, delta;
  const float *in = (const float*) input_items[0];

  /* Calculate average power for 1 input item */
  for (int count = 0; count < noutput_items * d_fft_len; count++) {
    delta = in[count] - mean;
    mean += delta / (count + 1);
  }
  avg_power = mean / d_fft_len;

//      dout << "Avg power: " << avg_power << ": " << 10*log10(avg_power) + 30
//          << "dBm" << endl;

  /* Accumulate avg_power */
  delta = avg_power - d_avg_power;
  d_avg_power += delta / (++d_avg_power_count);

  return noutput_items;
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::channel_hopping (void)
{
  boost::posix_time::ptime time;
  time = boost::posix_time::microsec_clock::universal_time ();
  dout << endl;
  dout << time << ": Channel hopping" << endl;

  current_rand_seed = rng ();
  current_working_channel = current_rand_seed % num_of_channels;
  dout << scientific;
  dout << "-> new channel: " << current_working_channel + first_channel_index
      << ", " << center_freqs[current_working_channel] << endl;

  pmt::pmt_t command = pmt::cons (
      pmt::mp ("freq"), pmt::mp (center_freqs[current_working_channel]));

  message_port_pub (pmt::mp ("usrp sink cmd"), command);
  message_port_pub (pmt::mp ("usrp source cmd"), command);
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::spectrum_sensing (void)
{
  is_spectrum_sensing_completed = false;
  is_channel_available = false;
  d_avg_power = 0;
  d_avg_power_count = 0;
  d_control_thread_state = SPECTRUM_SENSING;

  boost::posix_time::ptime time;
  time = boost::posix_time::microsec_clock::universal_time ();
  dout << time << ": Spectrum sensing" << endl;

  /* Sleep in sensing duration */
  boost::this_thread::sleep_for (
      boost::chrono::milliseconds { Tss - d_guard_time });

  /* Stop spectrum sensing state */
  d_control_thread_state = NULL_STATE;

  /* calculate final average power */
  d_avg_power_dBm = 10 * log10 (d_avg_power) + 30;
  dout << setprecision (3) << fixed;
  dout << "Avg power (dBm): " << d_avg_power_dBm << endl;
  dout << "Threshold (dBm): " << d_ss_threshold_dBm << endl;

  if (d_avg_power_dBm < d_ss_threshold_dBm) {
    is_channel_available = true;
  }
  else {
    is_channel_available = false;
  }

  is_spectrum_sensing_completed = true;
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::beacon_duration (void)
{
  boost::posix_time::ptime time;
  time = boost::posix_time::microsec_clock::universal_time ();
  dout << time << ": Beacon duration" << endl;

  if (d_nwk_dev_type == SUC) {
    /* SUC: Broadcast beacon */
    d_control_thread_state = BEACON;
    is_beacon_received = true; // always true for SUC.

    if (!is_spectrum_sensing_completed) {
      dout << "Spectrum sensing is not completed, will not broadcast beacon."
          << endl;
      return;
    }

    if (!is_channel_available) {
      dout << "Channel is busy, will not broadcast beacon." << endl;
      return;
    }

    /* Broadcast beacon */
    uint8_t mhr[IEEE802154_MAX_HDR_LEN];
    uint8_t flags = IEEE802154_FCF_TYPE_BEACON | IEEE802154_FCF_SRC_ADDR_SHORT
        | IEEE802154_FCF_SRC_ADDR_VOID;
    d_msg_len = 0;
    le_uint16_t pan_id_le = byteorder_btols (byteorder_htons (d_suc_id));

    if ((d_msg_len = ieee802154_set_frame_hdr (mhr, (uint8_t*) &d_mac_addr, 2,
                                               (uint8_t*) &d_broadcast_addr, 2,
                                               pan_id_le, pan_id_le, flags,
                                               d_seq_nr++)) == 0) {
      dout << "Beacon header error." << endl;
    }
    else {
      /* Copy header to MAC frame */
      memcpy (d_msg, mhr, d_msg_len);

      /* Superframe Specification field */
      d_msg[d_msg_len++] = 0x00;
      d_msg[d_msg_len++] = 0xc0;

      /* GTS Specification field */
      d_msg[d_msg_len++] = 0;

      /* Pending Address Specification field */
      d_msg[d_msg_len++] = 0;

      /* Prepare the beacon payload */
      uint16_to_buffer (d_suc_id, &d_msg[d_msg_len]);
      d_msg_len += 2;

      uint16_to_buffer (Tss, &d_msg[d_msg_len]);
      d_msg_len += 2;

      uint32_to_buffer (current_rand_seed, &d_msg[d_msg_len]);
      d_msg_len += 4;

      /* Calculate FCS */
      uint16_t fcs = crc16 (d_msg, d_msg_len);

      /* Add FCS to frame */
      d_msg[d_msg_len++] = (uint8_t) fcs;
      d_msg[d_msg_len++] = (uint8_t) (fcs >> 8);

      /* Broadcast beacon */
      print_message (d_msg, d_msg_len);
      message_port_pub (
          pmt::mp ("pdu out"),
          pmt::cons (pmt::PMT_NIL, pmt::make_blob (d_msg, d_msg_len)));
    }
  }
  else {
    /* SU: wait for beacon */
    //is_beacon_received = false;
    // TODO: hard-coded for demo, always assume that we have received a beaoon.
    is_beacon_received = true;
    d_control_thread_state = BEACON;
  }
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::reporting_duration (void)
{
  //TODO: no need for now.

  boost::posix_time::ptime time;
  time = boost::posix_time::microsec_clock::universal_time ();
  dout << time << ": Reporting Duration" << endl;

  is_busy_signal_received = false;

  return;
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::data_duration (void)
{
  boost::posix_time::ptime time;
  time = boost::posix_time::microsec_clock::universal_time ();
  dout << time << ": Data Duration" << endl;

  if ((is_channel_available) && (is_beacon_received)
      && (!is_busy_signal_received)) {
    d_control_thread_state = DATA_TRANSMISSION;
    dout << "Channel is available: DATA_TRANSMISSION state, " << endl;

    /* Wake transmit_thread up */
    transmit_thread_ptr->interrupt ();
  }
  else {
    d_control_thread_state = NULL_STATE;
    dout << "Sleeping" << endl;
  }
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::reload_tasks (void)
{
  d_control_thread_state = RELOADING;

  /* Perform channel hopping */
  tasks_processor::get ().run_at (
      time_slot_start, boost::bind (&shcs_mac_impl::channel_hopping, this));

  /* Spectrum sensing */
  tasks_processor::get ().run_at (
      time_slot_start + boost::posix_time::milliseconds (Th),
      boost::bind (&shcs_mac_impl::spectrum_sensing, this));

  /* Beacon duration */
  if (d_nwk_dev_type == SUC) {
    tasks_processor::get ().run_at (
        time_slot_start + boost::posix_time::milliseconds (Th + Tss + Tb / 2),
        boost::bind (&shcs_mac_impl::beacon_duration, this));
  }
  else {
    tasks_processor::get ().run_at (
        time_slot_start + boost::posix_time::milliseconds (Th + Tss),
        boost::bind (&shcs_mac_impl::beacon_duration, this));
  }

  /* Reporting duration */
  tasks_processor::get ().run_at (
      time_slot_start + boost::posix_time::milliseconds (Th + Tss + Tb),
      boost::bind (&shcs_mac_impl::reporting_duration, this));

  /* Data duration */
  tasks_processor::get ().run_at (
      time_slot_start + boost::posix_time::milliseconds (Th + Tss + Tb + Tr),
      boost::bind (&shcs_mac_impl::data_duration, this));

  /* Reload tasks at the end of a time slot */
  tasks_processor::get ().run_at (
      time_slot_start + boost::posix_time::milliseconds (Ts),
      boost::bind (&shcs_mac_impl::reload_tasks, this));

  /* Calculate next time_slot_start */
  time_slot_start = time_slot_start + boost::posix_time::milliseconds (Ts);
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::suc_control_thread (void)
{
  dout << "Coordinator control thread created." << endl;

  /* Waiting for everything to settle */
  boost::this_thread::sleep_for (boost::chrono::seconds { 3 });

  time_slot_start = boost::posix_time::microsec_clock::universal_time ();

  /* Reload tasks */
  reload_tasks ();

  /* Start tasks_processor */
  tasks_processor::get ().start ();
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::su_control_thread (void)
{
  dout << "SU control thread created." << endl;

  /* Waiting for everything to settle */
  boost::this_thread::sleep_for (boost::chrono::seconds { 3 });

  /* Bootstrapping */
  su_bootstrapping ();

  /* Reload tasks */
  reload_tasks ();

  /* Start tasks_processor */
  tasks_processor::get ().start ();

}
/*------------------------------------------------------------------------*/
void
shcs_mac_impl::su_bootstrapping (void)
{
  d_control_thread_state = SU_BOOTSTRAPPING;

  while (1) {
    /* Choose a random channel and wait for a time frame */
    channel_hopping ();

    dout << "Wait for beacon within a time frame, " << Tf << " ms" << endl;

    try {
      boost::this_thread::sleep_for (boost::chrono::milliseconds { Tf });
    }
    catch (boost::thread_interrupted&) {
      dout << "Interrupted, beacon received." << endl;
      d_control_thread_state = NULL_STATE;
      return;
    }
  }
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::sur_control_thread (void)
{
  dout << "SUR control thread created." << endl;

  /* Waiting for everything to settle */
  boost::this_thread::sleep_for (boost::chrono::seconds { 3 });

  /* Bootstrapping */
  su_bootstrapping ();

  /* Reload tasks */
  reload_tasks ();

  /* Start tasks_processor */
  tasks_processor::get ().start ();

}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::mac_in (pmt::pmt_t msg)
{
  pmt::pmt_t blob;
  uint8_t* frame_p = NULL;
  size_t data_index = 0;

  boost::posix_time::ptime time_slot_start_tmp;
  uint16_t suc_id_tmp, Tss_tmp;
  uint32_t rand_seed_tmp;

  if (pmt::is_pair (msg)) {
    blob = pmt::cdr (msg);
  }
  else {
    assert(false);
  }

  /* Take timestamp */
  time_slot_start_tmp = boost::posix_time::microsec_clock::universal_time ()
      + boost::posix_time::milliseconds (Ts)
      - boost::posix_time::milliseconds (Tss)
      - boost::posix_time::milliseconds (Th)
      - boost::posix_time::milliseconds (Tb / 2);

  size_t frame_len = pmt::blob_length (blob);
  if (frame_len < 11) {
    dout << "MAC: frame too short. Dropping!" << endl;
    return;
  }

  uint16_t crc = crc16 ((uint8_t*) pmt::blob_data (blob), frame_len);
  d_num_packets_received++;
  if (crc) {
    d_num_packet_errors++;
    dout << "MAC: wrong crc. Dropping packet!" << endl;
    return;
  }

  dout << "MAC: correct crc, new packet!" << endl;
  d_num_bytes_received += frame_len;

  /* Beacon packet */
  frame_p = (uint8_t*) pmt::blob_data (blob);

  if ((frame_p[0] & 0x03) == 0) {
    /* Beacon found */
    dout << "MAC: Found a beacon." << endl;

    switch (d_control_thread_state) {
      case SU_BOOTSTRAPPING:
        /* Store SUC_ID, sensing time, current random seed, time frame start time */
        data_index = ieee802154_get_frame_hdr_len (frame_p);

        /* Skip superframe, GTS and pending address fields */
        data_index += 4;

        suc_id_tmp = buffer_to_uint16 (&frame_p[data_index]);
        data_index += 2;
        Tss_tmp = buffer_to_uint16 (&frame_p[data_index]);
        data_index += 2;
        rand_seed_tmp = buffer_to_uint32 (&frame_p[data_index]);
        data_index += 4;

        /* Check predefined associate SUC_ID */
        if ((d_assoc_suc_id != 0xFFFF) && (suc_id_tmp != d_assoc_suc_id)) {
          dout << "Predefined assoc. suc id " << d_assoc_suc_id
              << " != received suc id, " << suc_id_tmp << " drop beacon! "
              << endl;
          return;
        }

        /* update suc id, sensing time, random seed, and time slot start */
        d_assoc_suc_id = suc_id_tmp;
        Tss = Tss_tmp;
        time_slot_start = time_slot_start_tmp;
        current_rand_seed = rand_seed_tmp;
        rng.seed (current_rand_seed);
        d_su_connected = true;

        dout << "Received SUC ID: " << hex << d_assoc_suc_id << dec << endl;
        dout << "Received Tss: " << Tss << endl;
        dout << "Received random seed: " << current_rand_seed << endl;
        dout << "Next time slot start: " << time_slot_start << endl;

        /* Wake up control thread */
        control_thread_ptr->interrupt ();

        break;

      case BEACON:
        if (d_nwk_dev_type == SU) {
          /* Check SUC ID */
          data_index = ieee802154_get_frame_hdr_len (frame_p);

          /* Skip superframe, GTS and pending address fields */
          data_index += 4;

          uint16_t recv_suc_id = buffer_to_uint16 (&frame_p[data_index]);

          if (d_assoc_suc_id == recv_suc_id) {
            time_slot_start = time_slot_start_tmp;
            is_beacon_received = true;

            dout << "MAC: Beacon state. => is_beacon_received = true." << endl;
            dout << "Recalculate time_slot_start: " << time_slot_start << endl;
          }
        }

        break;

      default:
        dout << "MAC: Drop beacon!" << endl;

        break;
    }

    return;
  }

  /* Other packet types */
  /* Check destination address */
  uint8_t dest[8];
  le_uint16_t dest_pan;
  int dest_addr_len = 0;

  dout << "MAC: Data/ACK packet." << endl;

  dest_addr_len = ieee802154_get_dst (frame_p, dest, &dest_pan);
  if (dest_addr_len != 2) {
    dout << "MAC: Something wrong here, dest_addr_len: " << dest_addr_len
        << endl;
    return;
  }

  if ((dest[0] != d_mac_addr[0]) || (dest[1] != d_mac_addr[1])) {
    dout << "MAC: not for me " << int (dest[0]) << "." << int (dest[1])
        << ", dropping packet!" << endl;
    return;
  }

  /* Forward to RIME layer */
  dout << "MAC: forward packet to RIME layer." << endl;
  print_message (frame_p, frame_len);

  size_t fcs_len = ieee802154_get_frame_hdr_len (frame_p);
  pmt::pmt_t mac_payload = pmt::make_blob ((char*) frame_p + fcs_len,
                                           frame_len - fcs_len - 2);

  message_port_pub (pmt::mp ("app out"), pmt::cons (pmt::PMT_NIL, mac_payload));
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::app_in (pmt::pmt_t msg)
{
  pmt::pmt_t blob;
  if (pmt::is_eof_object (msg)) {
    dout << "MAC: exiting" << endl;
    detail ().get ()->set_done (true);
    return;
  }
  else if (pmt::is_blob (msg)) {
    blob = msg;
  }
  else if (pmt::is_pair (msg)) {
    blob = pmt::cdr (msg);
  }
  else {
    dout << "MAC: unknown input" << endl;
    return;
  }

  dout << "MAC: received new message from APP of length "
      << pmt::blob_length (blob) << endl;

  /* If SU is not connected, drop all packets */
  if (d_nwk_dev_type == SU && !d_su_connected) {
    dout << "MAC: SU is not connected, drop packet!" << endl;
    return;
  }

  /* Push to transmit queue */
//      print_message ((uint8_t*) pmt::blob_data (blob), pmt::blob_length (blob));
  if (!transmit_queue.push (blob)) {
    dout << "MAC: push packet to transmit_queue failed." << endl;
  }
  else {
    /* Wake transmit_thread up */
    transmit_thread_ptr->interrupt ();
  }
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::transmit_thread (void)
{
  pmt::pmt_t blob;

  dout << "Transmit thread created." << endl;

  while (1) {
    try {
      boost::this_thread::sleep_for (boost::chrono::milliseconds { Tf });
    }
    catch (boost::thread_interrupted&) {
      dout << "Transmit_t: Interrupted, woke up." << endl;

      /* Check current control state */
      if (d_control_thread_state == DATA_TRANSMISSION) {
        /* Transmit data in transmit_queue */
        while (transmit_queue.pop (blob)) {
          print_message ((uint8_t*) pmt::blob_data (blob),
                         pmt::blob_length (blob));
          generate_mac ((const uint8_t*) pmt::blob_data (blob),
                        pmt::blob_length (blob));
//              print_message (d_msg, d_msg_len);
          message_port_pub (
              pmt::mp ("pdu out"),
              pmt::cons (pmt::PMT_NIL, pmt::make_blob (d_msg, d_msg_len)));
        }
      }
    }
  }/* end while */
}

/*------------------------------------------------------------------------*/
uint16_t
shcs_mac_impl::crc16 (uint8_t *buf, int len)
{
  uint16_t crc = 0;

  for (int i = 0; i < len; i++) {
    for (int k = 0; k < 8; k++) {
      int input_bit = (!!(buf[i] & (1 << k)) ^ (crc & 1));
      crc = crc >> 1;
      if (input_bit) {
        crc ^= (1 << 15);
        crc ^= (1 << 10);
        crc ^= (1 << 3);
      }
    }
  }

  return crc;
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::generate_mac (const uint8_t *buf, int len)
{
  uint8_t mhr[IEEE802154_MAX_HDR_LEN];
  uint8_t flags = IEEE802154_FCF_TYPE_DATA | IEEE802154_FCF_SRC_ADDR_SHORT;
  d_msg_len = 0;
  le_uint16_t pan_id_le;
  if (d_nwk_dev_type == SUC) {
    pan_id_le = byteorder_btols (byteorder_htons (d_suc_id));
  }
  else if (d_nwk_dev_type == SU) {
    pan_id_le = byteorder_btols (byteorder_htons (d_assoc_suc_id));
  }

  /* Peak to get src addr and dest addr */
  /* Test */
  cout << "MAC: TEST: buf: " << endl;
  for (int count = 0; count < len; count++) {
    cout << buf[count] << " ";
  }
  count << endl;

  const uint8_t* d_dest_addr = &buf[0];
  if ((d_msg_len = ieee802154_set_frame_hdr (mhr, (uint8_t*) &d_mac_addr, 2,
                                             (uint8_t*) &d_dest_addr, 2,
                                             pan_id_le, pan_id_le, flags,
                                             d_seq_nr++)) == 0) {
    dout << "MAC: header error." << endl;
  }
  else {
    /* Copy header to MAC frame */
    memcpy (d_msg, mhr, d_msg_len);

    /* Prepare the data payload */
    memcpy (&d_msg[d_msg_len], buf + 2, len - 2);
    d_msg_len += len - 2;

    /* Calculate FCS */
    uint16_t fcs = crc16 (d_msg, d_msg_len);

    /* Add FCS to frame */
    d_msg[d_msg_len++] = (uint8_t) fcs;
    d_msg[d_msg_len++] = (uint8_t) (fcs >> 8);
  }
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::print_message (uint8_t *buf, int len)
{
  for (int i = 0; i < len; i++) {
    dout << setfill ('0') << setw (2) << hex << ((unsigned int) buf[i] & 0xFF)
        << dec << " ";
    if (i % 16 == 19) {
      dout << endl;
    }
  }
  dout << endl;
}

/*------------------------------------------------------------------------*/
int
shcs_mac_impl::get_num_packet_errors ()
{
  return d_num_packet_errors;
}

/*------------------------------------------------------------------------*/
int
shcs_mac_impl::get_num_packets_received ()
{
  return d_num_packets_received;
}

/*------------------------------------------------------------------------*/
float
shcs_mac_impl::get_packet_error_ratio ()
{
  return float (d_num_packet_errors) / d_num_packets_received;
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::reporting_thread_func (void)
{
  int count = 0;

  while (1) {
    d_num_bytes_received = 0;

    /* Sleep for 5s  */
    boost::this_thread::sleep_for (boost::chrono::seconds (5));

    /* Reporting */
    std::cout << "MAC: Reports #" << count << ", avg data rate: "
        << d_num_bytes_received * 8 / 1024 / 5 << " kbit/s" << std::endl;

    count++;
  }
}

/*----------------------------------------------------------------------------*/
uint16_t
shcs_mac_impl::buffer_to_uint16 (uint8_t * buffer)
{
  uint16_t ret_val;

  ret_val = (*buffer) | (*(buffer + 1) << 8);

  return ret_val;
}

/*----------------------------------------------------------------------------*/
uint32_t
shcs_mac_impl::buffer_to_uint32 (uint8_t * buffer)
{
  uint32_t ret_val;

  ret_val = (*buffer) | (*(buffer + 1) << 8) | (*(buffer + 2) << 16)
      | (*(buffer + 3) << 24);

  return ret_val;
}

/*----------------------------------------------------------------------------*/
void
shcs_mac_impl::uint16_to_buffer (uint16_t data, uint8_t* buffer)
{
  *buffer = (uint8_t) data;
  *(++buffer) = (uint8_t) (data >> 8);
}

/*----------------------------------------------------------------------------*/
void
shcs_mac_impl::uint32_to_buffer (uint32_t data, uint8_t* buffer)
{
  *buffer = (uint8_t) data;
  *(++buffer) = (uint8_t) (data >> 8);
  *(++buffer) = (uint8_t) (data >> 16);
  *(++buffer) = (uint8_t) (data >> 24);
}

/*----------------------------------------------------------------------------*/
float
shcs_mac_impl::buffer_to_float (uint8_t* buffer)
{
  float ret_val;
  uint16_t dec;
  uint16_t frac;

  dec = buffer_to_uint16 (buffer);
  buffer += 2;
  frac = buffer_to_uint16 (buffer);

  ret_val = (float) dec;
  ret_val += ((float) frac) / 100;

  return ret_val;
}

/*----------------------------------------------------------------------------*/
void
shcs_mac_impl::float_to_buffer (float data, uint8_t* buffer)
{
  uint16_t dec, frac;

  dec = (uint16_t) data;
  frac = ((uint16_t) (data * 100)) % 100;

  uint16_to_buffer (dec, buffer);
  buffer += 2;
  uint16_to_buffer (frac, buffer);
}

