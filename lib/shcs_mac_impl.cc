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
 * @author Nhat Pham <nhatphd@kaist.ac.kr>, Real-time and Embedded Systems Lab
 * @version 1.0
 * @date 2018-07-28
 * @brief This is the implementation for MSHCS MAC protocol.
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
#include <numeric>
#include <string>
#include <functional>
#include <time.h>

#include "shcs_mac_impl.h"
#include "shcs_ieee802154.h"
#include "shcs_tasks_processor_base.hpp"
#include "shcs_tasks_processor_timers.hpp"

#include <gnuradio/block_registry.h>
#include <boost/pointer_cast.hpp>

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
  cout << "Hello, this is MSHCS-MAC protocol implementation, version 1.0"
      << endl;
  if (nwk_dev_type == SUC) {
    cout << "NWK device type: SU Coordinator" << endl;
    cout << "SUC ID: " << hex << d_suc_id << dec << endl;
  }
  else if (nwk_dev_type == SUR) {
    cout << "NWK device type: SU Router" << endl;
    cout << "SUC ID: " << hex << d_suc_id << dec << endl;
    cout << "Assoc. SUC ID: " << hex << d_assoc_suc_id << dec << endl;
  }
  else {
    cout << "NWK device type: SU" << endl;
    cout << "Assoc. SUC ID: " << hex << d_assoc_suc_id << dec << endl;
  }

  if (mac_addr.size () != 2)
    throw std::invalid_argument ("MAC address has to consist of two integers");
  d_mac_addr[0] = mac_addr[0];
  d_mac_addr[1] = mac_addr[1];
  cout << "MAC address: " << int (d_mac_addr[0]) << "." << int (d_mac_addr[1])
      << endl;
  cout << "Ts (ms): " << Ts << endl;
  cout << "Tss (ms): " << Tss << endl;
  cout << "Tb (ms): " << Tb << endl;
  cout << "Tr (ms): " << Tr << endl;

  /* Time sync */
  ref_point_ptime = boost::posix_time::time_from_string (ref_point_c);
  cout << "Reference point: " << ref_point_ptime << endl;

  /* RBS */
  rbs_delay_acc_ptr = boost::shared_ptr<MeanAccumulator> (
      new MeanAccumulator (bt::rolling_window::window_size =
          rbs_delay_window_size));

  rbs_t_locals_ptr = boost::shared_ptr<boost::circular_buffer<int64_t>> (
      new boost::circular_buffer<int64_t> (rbs_offset_max_samples));
  rbs_t_refs_ptr = boost::shared_ptr<boost::circular_buffer<int64_t>> (
      new boost::circular_buffer<int64_t> (rbs_offset_max_samples));

  /* USRP GPIO */
  usrp_gpio_init ();
  usrp_gpio_off (0);
  usrp_gpio_off (1);
  usrp_gpio_off (2);
  usrp_gpio_off (3);

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

  /* Create transmission thread */
  transmit_thread_ptr[TX_THREAD_LOCAL] = boost::shared_ptr<gr::thread::thread> (
      new gr::thread::thread (
          boost::bind (&shcs_mac_impl::transmit_thread_local, this)));

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

    /* Create additional transmit thread for Router */
    transmit_thread_ptr[TX_THREAD_PARENT] =
        boost::shared_ptr<gr::thread::thread> (
            new gr::thread::thread (
                boost::bind (&shcs_mac_impl::transmit_thread_parent, this)));
  }
  else {
    /* Create control thread for SU */
    control_thread_ptr = boost::shared_ptr<gr::thread::thread> (
        new gr::thread::thread (
            boost::bind (&shcs_mac_impl::su_control_thread, this)));
  }

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

  if (d_control_thread_state != SPECTRUM_SENSING && !d_cca_state) {
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
  uint32_t seed_tmp;
  boost::posix_time::ptime time;

  time = boost::posix_time::microsec_clock::universal_time ();
  dout << endl;
  dout << time << ": Channel hopping" << endl;

  /* Toogle pin 1 */
//  usrp_gpio_toggle (0);
  /* Increase Ts_counter */
  Ts_counter++;

  /* RBS sync period */
  if ((Ts_counter % (rbs_sync_period / Ts) == 0)
      || (Ts_counter % (rbs_sync_period / Ts) == 1)) {
    rbs_sync_period_flag = true;
  }
  else {
    rbs_sync_period_flag = false;
  }

  current_rand_seed = rng ();
  seed_tmp = current_rand_seed;

  if (d_nwk_dev_type == SUR) {
    dout << "SUR: ";
    current_rand_seed_local = rng_local ();

    if (d_sur_state == IN_PARENT_NWK) {
      dout << "In parent nwk" << endl;
    }
    else {
      dout << "In local nwk" << endl;
      seed_tmp = current_rand_seed_local;
    }
  }

  current_working_channel = get_current_working_channel_from_seed (seed_tmp);
  dout << scientific;
  dout << "-> new channel: " << current_working_channel + first_channel_index
      << ", " << center_freqs[current_working_channel] << endl;

  pmt::pmt_t command = pmt::cons (
      pmt::mp ("freq"), pmt::mp (center_freqs[current_working_channel]));

  message_port_pub (pmt::mp ("usrp sink cmd"), command);
  message_port_pub (pmt::mp ("usrp source cmd"), command);
}

/*------------------------------------------------------------------------*/
uint32_t
shcs_mac_impl::get_current_working_channel_from_seed (uint32_t seed)
{
  uint32_t temp = seed % max_prio;
  uint32_t count;

  for (count = 0; count < num_of_channels - 1; count++) {
    if (temp <= channel_prios[count]) {
      return count;
    }
  }

  return count;
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::spectrum_sensing (void)
{
  is_spectrum_sensing_completed = false;
  is_channel_available = false;
  d_avg_power = 0;
  d_avg_power_count = 0;
  double avg_power_dBm = 0;
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
  avg_power_dBm = 10 * log10 (d_avg_power) + 30;
  dout << setprecision (3) << fixed;
  dout << "Avg power (dBm): " << avg_power_dBm << endl;
  dout << "Threshold (dBm): " << d_ss_threshold_dBm << endl;

  if (avg_power_dBm < d_ss_threshold_dBm) {
    is_channel_available = true;
  }
  else {
    is_channel_available = false;
  }
  /* For demo */
  if (!is_sleeping_needed) {
    is_channel_available = true; // Channel is always available.
  }

  is_spectrum_sensing_completed = true;
}

/*------------------------------------------------------------------------*/
bool
shcs_mac_impl::cca (void)
{
  d_avg_power = 0;
  d_avg_power_count = 0;
  double avg_power_dBm;

  /* Start ED */
  d_cca_state = true;

  boost::this_thread::sleep_for (boost::chrono::milliseconds { cca_time });

  /* Stop ED */
  d_cca_state = false;

  /* calculate final average power */
  avg_power_dBm = 10 * log10 (d_avg_power) + 30;

  if (avg_power_dBm < cca_threshold) {
    return true;
  }

  return false;
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::phy_transmit (const uint8_t *buf, int len)
{
  message_port_pub (pmt::mp ("pdu out"),
                    pmt::cons (pmt::PMT_NIL, pmt::make_blob (buf, len)));
}

/*------------------------------------------------------------------------*/
bool
shcs_mac_impl::csma_ca_send (uint16_t transmit_state, bool wait_for_beacon,
                             bool suc_wait_for_sur, const uint8_t *buf, int len)
{
  int num_of_backoffs = 0, backoff_exp = 0;
  int backoff_time, backoff_time_max;

  while (1) {
    if (transmit_state != NULL_STATE) {
      /* Wait until transmit state */
      gr::thread::scoped_lock lock (d_tx_mutex);
      while (d_control_thread_state != transmit_state
          || (wait_for_beacon && !is_beacon_received && !d_ext_op_sender
              && d_ext_op_recv_wait_time_left <= 0)
          || (suc_wait_for_sur && d_assoc_current_sur_state != IN_PARENT_NWK
              && !d_ext_op_sender && d_ext_op_recv_wait_time_left <= 0)) {
        d_tx_condvar.wait (lock);
      }
      lock.unlock ();
    }

    /* Calculate backoff time */
    backoff_time_max = (int) (pow (2, backoff_exp) - 1);

    if (backoff_time_max != 0) {
      backoff_time = ((rand () % backoff_time_max) + 1) * csma_ca_backoff_unit;

//      dout << "CSMA: backoff time max " << backoff_time_max << endl;
      dout << "CSMA: backoff " << backoff_time << "ms" << endl;
      boost::this_thread::sleep_for (
          boost::chrono::milliseconds { backoff_time });

      if ((d_control_thread_state != transmit_state)
          && (transmit_state != NULL_STATE)) {
        /* Cancel current backoff and do it again in next time slot */
        continue;
      }
    }

    /* Perform CCA */
    if (cca ()) {
      /* Channel is idle, transmit data */
      phy_transmit (buf, len);
      dout << "CMSA: transmitted" << endl;
      return true;
    }

    /* Channel is busy */
    backoff_exp = min (backoff_exp + 1, max_csma_ca_be);
    num_of_backoffs++;
    if (num_of_backoffs > max_csma_ca_backoffs) {
      return false;
    }
  }
}

/*------------------------------------------------------------------------*/
bool
shcs_mac_impl::csma_ca_rsend (uint8_t transmit_thread_id,
                              uint16_t transmit_state, bool wait_for_beacon,
                              bool suc_wait_for_sur, const uint8_t *buf,
                              int len)
{
  if (!csma_with_ack) {
    return csma_ca_send (transmit_state, wait_for_beacon, suc_wait_for_sur, buf,
                         len);
  }

  uint8_t seqno, backup_buf[256];
//  uint8_t dest_addr[8];
//  le_uint16_t dest_panid;

  /* Backup buffer */
  memcpy (backup_buf, buf, len);

  /* Get dest address and seqno*/
//  ieee802154_get_dst (backup_buf, dest_addr, &dest_panid);
  seqno = ieee802154_get_seq (backup_buf);

  for (int count = 0; count < max_retries + 1; count++) { /* first time transmit is not a retry */
    d_ack_recv_seq_nr[transmit_thread_id] = seqno; /* Expecting seqno */

    /* Send data */
    dout << "RSend: Send #" << int (seqno) << " try " << count << endl;
//    csma_ca_send (transmit_state, wait_for_beacon, suc_wait_for_sur, buf, len);
    csma_ca_send (transmit_state, false, suc_wait_for_sur, buf, len); /* Test maximum throughput */

    /* Wait for ACK */
    gr::thread::scoped_lock lock (d_ack_m[transmit_thread_id]);
    d_is_ack_received[transmit_thread_id] = false;

    d_ack_received_cv[transmit_thread_id].timed_wait (
        lock, boost::posix_time::milliseconds (max_retry_timeout));

    if (d_is_ack_received[transmit_thread_id]) {
      /* ACK received */
      dout << "RSend: Received ack for #" << int (seqno) << endl;
      return true;
    }
  }

  dout << "RSend: failed" << endl;
  return false;
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::beacon_duration (void)
{
  boost::posix_time::ptime time;
  time = boost::posix_time::microsec_clock::universal_time ();
  rbs_last_beacon_send_timestamp = time;
  dout << time << ": Beacon duration" << endl;
  uint8_t buf[256];
  int len;

  if ((d_nwk_dev_type == SUC)
      || ((d_nwk_dev_type == SUR) && (d_sur_state == IN_LOCAL_NWK))) {
    /* SUC: Broadcast beacon */
    d_control_thread_state = BEACON;

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
    len = 0;
    le_uint16_t pan_id_le = byteorder_btols (byteorder_htons (d_suc_id));

    if ((len = ieee802154_set_frame_hdr (mhr, d_mac_addr, 2, d_broadcast_addr,
                                         2, pan_id_le, pan_id_le, flags,
                                         d_seq_nr++)) == 0) {
      dout << "Beacon header error." << endl;
    }
    else {
      /* Copy header to MAC frame */
      memcpy (buf, mhr, len);

      /* Superframe Specification field */
      buf[len++] = 0x00;
      buf[len++] = 0xc0;

      /* GTS Specification field */
      buf[len++] = 0;

      /* Pending Address Specification field */
      buf[len++] = 0;

      /* Prepare the beacon payload */
      /* Control byte */
      uint8_t control_byte = 0;
      control_byte |= d_assoc_current_sur_state << ASSOC_CURRENT_SUR_STATE_POS;
      buf[len] = control_byte;
      len++;

      /* SUC ID */
      uint16_to_buffer (d_suc_id, &buf[len]);
      len += 2;

      /* Sensing time */
      uint16_to_buffer (Tss, &buf[len]);
      len += 2;

      /* Current rand seed */
      if (d_nwk_dev_type == SUC) {
        uint32_to_buffer (current_rand_seed, &buf[len]);
      }
      else {
        /* SUR */
        uint32_to_buffer (current_rand_seed_local, &buf[len]);
      }
      len += 4;

      /* Calculate FCS */
      uint16_t fcs = crc16 (buf, len);

      /* Add FCS to frame */
      buf[len++] = (uint8_t) fcs;
      buf[len++] = (uint8_t) (fcs >> 8);

      /* Broadcast beacon */
      print_message (buf, len);
      phy_transmit (buf, len);
    }
  }
  else {
    /* SU: wait for beacon */
    is_beacon_received = false;
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

  if ((is_channel_available) && (!is_busy_signal_received)) {
    /* RBS: Send timestamps packet */
    if (rbs_send_timestamps_packet && rbs_sync_period_flag) {
      generate_rbs_timestamps_packet (rbs_last_beacon_send_timestamp,
                                      rbs_last_beacon_rcv_timestamp);
    }
    rbs_send_timestamps_packet = false;

    /* Wake transmit_thread up */
    gr::thread::scoped_lock lock (d_tx_mutex);
    if ((d_nwk_dev_type == SUR) && (d_sur_state == IN_PARENT_NWK)) {
      d_control_thread_state = DATA_TRANSMISSION_PARENT;
      dout << "Channel is available: DATA_TRANSMISSION_PARENT state, " << endl;

//      /* TPSN test */
//      /* Request TPSN ack */
//      generate_tpsn_req ((uint8_t*) &d_assoc_suc_id);
    }
    else {
      d_control_thread_state = DATA_TRANSMISSION_LOCAL;
      dout << "Channel is available: DATA_TRANSMISSION_LOCAL state, " << endl;
    }

    lock.unlock ();
    d_tx_condvar.notify_all ();
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
  /* SUR: switch network */
  if (d_nwk_dev_type == SUR) {
    d_sur_state = !d_sur_state;
  }

  /* SUC: switch current SUR should-be state */
  if (d_nwk_dev_type == SUC) {
    d_assoc_current_sur_state = !d_assoc_current_sur_state;
  }

  /* Check d_ex_op_wait_time_left */
  if ((!d_ext_op_sender) && (d_ext_op_recv_wait_time_left <= 0)) {
    d_control_thread_state = RELOADING;

    /* Perform channel hopping */
    tasks_processor::get ().run_at (
        time_slot_start, boost::bind (&shcs_mac_impl::channel_hopping, this));

    /* Spectrum sensing */
    tasks_processor::get ().run_at (
        time_slot_start + boost::posix_time::milliseconds (Th),
        boost::bind (&shcs_mac_impl::spectrum_sensing, this));

    /* Beacon duration */
    if ((d_nwk_dev_type == SUC)
        || ((d_nwk_dev_type == SUR) && (d_sur_state == IN_LOCAL_NWK))) {
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
  }
  else {
    /* Extended operation */
    dout << "MAC: Reload task: in EXT_OP" << endl;

    current_rand_seed = rng ();

    if (d_nwk_dev_type == SUR) {
      current_rand_seed_local = rng_local ();
    }

    gr::thread::scoped_lock lock (d_ext_op_recv_wait_time_mutex);
    if (d_ext_op_recv_wait_time_left > 0) {
      d_ext_op_recv_wait_time_left--;
      dout << "MAC: EXT_OP wait time " << d_ext_op_recv_wait_time_left << endl;
    }
    lock.unlock ();
  }

  /* Reload tasks at the end of a time slot */
  tasks_processor::get ().run_at (
      time_slot_start + boost::posix_time::milliseconds (Ts - d_guard_time),
      boost::bind (&shcs_mac_impl::reload_tasks, this));

  /* Calculate next time_slot_start */
  time_slot_start = time_slot_start + boost::posix_time::milliseconds (Ts);
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::suc_control_thread (void)
{
  dout << "Coordinator control thread created." << endl;

  /* Seed RNGs */
  uint32_t seed = seed_gen ();
  rng.seed (seed);
  dout << "RNG1 seed: " << seed << endl;

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

  /* Seed RNGs */
  uint32_t seed = seed_gen ();
  rng.seed (seed);
  dout << "RNG1 seed: " << seed << endl;
  seed = seed_gen ();
  rng_local.seed (seed);
  dout << "RNG2 seed: " << seed << endl;

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
  size_t frame_len = 0;
  size_t data_index = 0;

  boost::posix_time::ptime received_timestamp, time_slot_start_tmp;
  uint8_t control_byte_tmp;
  uint16_t suc_id_tmp, Tss_tmp;
  uint32_t rand_seed_tmp;

  uint16_t recv_suc_id;

  /* Take timestamp */
  received_timestamp = boost::posix_time::microsec_clock::universal_time ();

//  time_slot_start_tmp = received_timestamp
//      + boost::posix_time::milliseconds (Ts)
//      - boost::posix_time::milliseconds (Tss)
//      - boost::posix_time::milliseconds (Th)
//      - boost::posix_time::milliseconds (Tb / 2); // old calculation

//  int64_t time_until_next_ts = static_cast<int64_t> (static_cast<double> ((Tr
//      + Tdata + Tb / 2) * 1000 - rbs_avg_delay) * rbs_modifier);

  int64_t time_until_next_ts = (Tr + Tdata + Tb / 2) * 1000 - rbs_avg_delay;
  time_slot_start_tmp = received_timestamp
      + boost::posix_time::microseconds (time_until_next_ts);

  if (pmt::is_pair (msg)) {
    blob = pmt::cdr (msg);
  }
  else {
    assert(false);
  }

  frame_len = pmt::blob_length (blob);
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

  /* New packet */
  frame_p = (uint8_t*) pmt::blob_data (blob);

  if ((frame_p[0] & IEEE802154_FCF_TYPE_MASK) == IEEE802154_FCF_TYPE_BEACON) {
    /* Beacon found */
    dout << "MAC: Found a beacon." << endl;

    /* RBS testing */
//    uint8_t recv_seqno = ieee802154_get_seq (frame_p);
//
//    if (recv_seqno != d_last_recv_seqno_rbs+1) {
//      d_last_recv_seqno_rbs++;
//      while (d_last_recv_seqno_rbs != recv_seqno) {
//        cout << (int) d_last_recv_seqno_rbs << endl;
//        d_last_recv_seqno_rbs++;
//      }
//    }
//
//    cout << (int) recv_seqno << ", "
//        << (received_timestamp - ref_point_ptime).total_microseconds () << endl;
//    d_last_recv_seqno_rbs = recv_seqno;
    switch (d_control_thread_state) {
      case SU_BOOTSTRAPPING:
        /* Store SUC_ID, sensing time, current random seed, time frame start time */
        data_index = ieee802154_get_frame_hdr_len (frame_p);

        /* Skip superframe, GTS and pending address fields */
        data_index += 4;

        control_byte_tmp = frame_p[data_index];
        data_index++;
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

        /* update suc id, random seed, and time slot start */
        d_assoc_suc_id = suc_id_tmp;
        time_slot_start = time_slot_start_tmp;
        current_rand_seed = rand_seed_tmp;
        rng.seed (current_rand_seed);

        if (d_nwk_dev_type == SUR) {
          d_sur_state =
              (bool) ((control_byte_tmp >> ASSOC_CURRENT_SUR_STATE_POS) & 0x01);
          dout << "SUR current state: ";
          if (d_sur_state == IN_PARENT_NWK) {
            dout << "IN_PARENT_NWK" << endl;
          }
          else {
            dout << "IN_LOCAL_NWK" << endl;
          }
        }

        d_su_connected = true;

        dout << "Control byte: 0x" << hex << int (control_byte_tmp) << dec
            << endl;
        dout << "Received SUC ID: 0x" << hex << d_assoc_suc_id << dec << endl;
        dout << "Received Tss: " << Tss << endl;
        dout << "Received random seed: " << current_rand_seed << endl;
        dout << "Next time slot start: " << time_slot_start << endl;
        cout << "\nSUR/SU: connected to 0x" << hex << d_assoc_suc_id << dec
            << endl;

        /* Wake up control thread */
        control_thread_ptr->interrupt ();

        break;

      case BEACON:
        /* Get SUC ID */
        data_index = ieee802154_get_frame_hdr_len (frame_p);

        /* Skip superframe, GTS, pending address and control_byte fields */
        data_index += 5;

        recv_suc_id = buffer_to_uint16 (&frame_p[data_index]);

        /* RBS: save last beacon time stamp */
        rbs_last_beacon_rcv_timestamp = received_timestamp;

        if (d_nwk_dev_type == SU
            || (d_nwk_dev_type == SUR && d_sur_state == IN_PARENT_NWK)) {

          if (d_assoc_suc_id == recv_suc_id) {
            /* Update the starting time of next timeslot */
            time_slot_start = time_slot_start_tmp;
            is_beacon_received = true;

            dout << "MAC: Beacon state. => is_beacon_received = true." << endl;
            dout << "Updated time_slot_start: " << time_slot_start << endl;
          }
        }
        else if (d_nwk_dev_type == SUC
            || (d_nwk_dev_type == SUR && d_sur_state == IN_LOCAL_NWK)) {
          /* RBS: check whether we receive our own beacon or not */
          if (d_suc_id == recv_suc_id) {
            /* RBS: my beacon, signal to send RBS timestamp packet */
            dout << "MAC: received my own beacon." << endl;
            rbs_send_timestamps_packet = true;
          }
        }

        break;

      default:
        dout << "MAC: Drop beacon!" << endl;

        break;
    }

    return;
  }

  if ((frame_p[0] & IEEE802154_FCF_TYPE_MASK)
      == IEEE802154_FCF_TYPE_RBS_TIMESTAMPS) {
    /* RBS timestamps */
    dout << "MAC: RBS timestamps received." << endl;

    if (d_nwk_dev_type == SU
        || (d_nwk_dev_type == SUR && d_sur_state == IN_PARENT_NWK)) {
      /* Check whether a beacon has been received in the last beacon duration */
      if (!is_beacon_received) {
        dout << "MAC: Didn't receive beacon, drop rbs timestamps." << endl;
        return;
      }

      /* Get SUC ID */
      data_index = ieee802154_get_frame_hdr_len (frame_p);
      recv_suc_id = buffer_to_uint16 (&frame_p[data_index]);
      data_index += 2;

      if (d_assoc_suc_id == recv_suc_id) {
        /* Get timestamps */
        int64_t ref_beacon_rcv_time = (int64_t) buffer_to_uint64 (
            &frame_p[data_index]);
        data_index += 8;
        int64_t ref_delay = (int64_t) buffer_to_uint64 (&frame_p[data_index]);
        data_index += 8;

//        dout << "MAC: SUC ID: " << recv_suc_id << ", current_offset: "
//            << (rbs_last_beacon_rcv_timestamp - ref_point_ptime).total_microseconds ()
//                - ref_beacon_rcv_time << ", ref_delay: " << ref_delay << endl;

        /* Calculate avg_delay */
        (*rbs_delay_acc_ptr) (ref_delay);
        rbs_avg_delay = (int64_t) ba::rolling_mean (*rbs_delay_acc_ptr);
        dout << "MAC: rolling delay: " << rbs_avg_delay << endl;

        /* Save timestamps to ring buffers */
        rbs_t_locals_ptr->push_back (
            (rbs_last_beacon_rcv_timestamp - ref_point_ptime).total_microseconds ());
        rbs_t_refs_ptr->push_back (ref_beacon_rcv_time);
        dout << "MAC: push (local, ref): ("
            << (rbs_last_beacon_rcv_timestamp - ref_point_ptime).total_microseconds ()
            << ", " << ref_beacon_rcv_time << ") to buffers" << endl;
        rbs_new_samples_counter++;

        if (rbs_new_samples_counter >= rbs_linear_regression_min_new_samples) {
          /* Do linear regression */
          double t_locals_buf[rbs_offset_max_samples],
              t_refs_buf[rbs_offset_max_samples];

          /* Change variables: Tlc' = Tlc - Tlc0, Tr' = Tr - Tr0, Tlc = f(Tr), Tlc' = f'(Tr') */
          rbs_t_local0 = (*rbs_t_locals_ptr)[0];
          rbs_t_ref0 = (*rbs_t_refs_ptr)[0];

//          cout << endl << "(Tlc, Tr):" << endl;
          for (int count = 0; count < rbs_t_locals_ptr->size (); count++) {
//            // TEST
//            cout << "(" << (*rbs_t_locals_ptr)[count] << ", "
//                << (*rbs_t_refs_ptr)[count] << ") ";

            /* Buffers of Tlc' and Tr' */
            t_locals_buf[count] =
                (static_cast<double> ((*rbs_t_locals_ptr)[count] - rbs_t_local0));
            t_refs_buf[count] = (static_cast<double> ((*rbs_t_refs_ptr)[count]
                - rbs_t_ref0));
          }
          cout << endl;

          /* Calculate f': a', b' */
          linear_regression (t_refs_buf, t_locals_buf,
                             rbs_t_locals_ptr->size (), &rbs_linear_regess);

          rbs_modifier = rbs_linear_regess.a;
          rbs_current_offset = rbs_linear_regess.b;
          cout << "\nMAC: (Tlocal-Tlocal0) = " << rbs_linear_regess.a
              << " * (Tref-Tref0) + " << rbs_linear_regess.b << endl;
          rbs_new_samples_counter = 0;
        }
      }
      else {
        dout << "MAC: recv_suc_id != d_assoc_suc_id: " << recv_suc_id << ", "
            << d_assoc_suc_id << endl;
      }
    }
    else {
      dout << "MAC: SUC, or SUR in LOCAL_NWK, don't care RBS timestamps."
          << endl;
    }

    return;
  }

  /* DATA/ACK */
  /* Check destination address */
  uint8_t dest[8];
  le_uint16_t dest_pan;
  int dest_addr_len = 0;

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

  /* ACK */
  if ((frame_p[0] & IEEE802154_FCF_TYPE_MASK) == IEEE802154_FCF_TYPE_ACK) {
    /* ACK, check seqno */
    uint8_t recv_seqno = ieee802154_get_seq (frame_p);

    for (int count = 0; count < max_transmit_threads; count++) {
      if (d_ack_recv_seq_nr[count] == recv_seqno) {
        {
          gr::thread::scoped_lock lock (d_ack_m[count]);
          d_is_ack_received[count] = true;
        }
        d_ack_received_cv[count].notify_one ();
        dout << "MAC: notified tx thead #" << count << endl;
      }
    }

    return;
  }

  /* DATA */
  if ((frame_p[0] & IEEE802154_FCF_TYPE_MASK) == IEEE802154_FCF_TYPE_DATA) {
    uint8_t recv_src_addr[8], recv_seqno;
    le_uint16_t recv_src_pan;
    uint8_t ack_frame_buf[IEEE802154_MAX_HDR_LEN];
    int ack_frame_len;

    recv_seqno = ieee802154_get_seq (frame_p);

    /* Check duplicate frame */
    if (!d_is_first_recv_data_frame) {
      if (recv_seqno == d_last_recv_seqno) {
        /* Duplicated, drop frame */
        dout << "MAC: duplicated frame #" << int (d_last_recv_seqno)
            << ", dropping." << endl;

        /* Send ACK back to sender */
        ieee802154_get_src (frame_p, recv_src_addr, &recv_src_pan);

        generate_ack_frame (recv_src_addr, recv_seqno, ack_frame_buf,
                            ack_frame_len);
        phy_transmit (ack_frame_buf, ack_frame_len);
        dout << "MAC: Sent ack #" << int (recv_seqno) << endl;
        return;
      }
    }
    else {
      /* first received data frame */
      d_is_first_recv_data_frame = false;
    }

    /* Handle new data frame */
    dout << "MAC: new DATA frame #" << int (recv_seqno) << endl;

    /* Check extended operation request */
    if ((frame_p[0] & IEEE802154_SHCS_EXT_OPERATION)
        == IEEE802154_SHCS_EXT_OPERATION) {
      /* turn on extended operation */
      dout << "MAC: EXT_OP is on." << endl;
      gr::thread::scoped_lock lock (d_ext_op_recv_wait_time_mutex);
      d_ext_op_recv_wait_time_left = max_ext_op_recv_wait_time;
      lock.unlock ();
    }

    /* Send ACK back to sender */
    ieee802154_get_src (frame_p, recv_src_addr, &recv_src_pan);

    generate_ack_frame (recv_src_addr, recv_seqno, ack_frame_buf,
                        ack_frame_len);
    phy_transmit (ack_frame_buf, ack_frame_len);
    dout << "MAC: Sent ack #" << int (recv_seqno) << endl;
    d_last_recv_seqno = recv_seqno;

    /* Forward to RIME layer */
    dout << "MAC: forward packet to RIME layer." << endl;
    print_message (frame_p, frame_len);

    size_t fcs_len = ieee802154_get_frame_hdr_len (frame_p);
    pmt::pmt_t mac_payload = pmt::make_blob ((char*) frame_p + fcs_len,
                                             frame_len - fcs_len - 2);

    message_port_pub (pmt::mp ("app out"),
                      pmt::cons (pmt::PMT_NIL, mac_payload));

    return;
  }

  /* TPSN REQ */
  if ((frame_p[0] & IEEE802154_FCF_TYPE_MASK) == IEEE802154_FCF_TYPE_TPSN_REQ) {
    uint8_t recv_src_addr[8], recv_seqno;
    le_uint16_t recv_src_pan;

    dout << "MAC: received TPSN REQ" << endl;

    /* Get seqno */
    recv_seqno = ieee802154_get_seq (frame_p);

    /* Get src address */
    ieee802154_get_src (frame_p, recv_src_addr, &recv_src_pan);

    generate_tpsn_ack (recv_src_addr, recv_seqno, received_timestamp);

    /* print message */
//    print_message(tpsn_ack_frame_p, tpsn_ack_frame_len);
    return;
  }

  /* TPSN ACK */
  if ((frame_p[0] & IEEE802154_FCF_TYPE_MASK) == IEEE802154_FCF_TYPE_TPSN_ACK) {
    /* Check seqno */
    if (d_tpsn_waiting_seqno != ieee802154_get_seq (frame_p)) {
      /* Duplicated TPSN ACK */
      dout << "MAC: duplicated TPSN ACK, dropped!" << endl;
      return;
    }

    /* print message */
//    print_message(frame_p, frame_len);
    /* Handle TPSN ACK */
    int64_t t2, t3, t4;
    size_t fcs_len = ieee802154_get_frame_hdr_len (frame_p);

    t2 = buffer_to_uint64 (&frame_p[fcs_len]);
    t3 = buffer_to_uint64 (&frame_p[fcs_len + 8]);
    t4 = (received_timestamp - ref_point_ptime).total_microseconds ();

    offset = ((t2 - t1) - (t4 - t3)) / 2;
    delay = ((t2 - t1) + (t4 - t3)) / 2;

    dout << "MAC: TPSN\n" << "t1: " << t1 << "\n" << "t2: " << t2 << "\n"
        << "t3: " << t3 << "\n" << "t4: " << t4 << "\n" << "offset: " << offset
        << "\n" << "delay: " << delay << "\n";
    cout << offset << ", " << delay << endl;

    return;
  }
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

  dout << "MAC: new RIME msg from APP, len: " << pmt::blob_length (blob)
      << endl;

  /* If SU is not connected, drop all packets */
  if ((d_nwk_dev_type == SU || d_nwk_dev_type == SUR) && !d_su_connected) {
    //dout << "MAC: SU/SUR is not connected, drop packet!" << endl;
    return;
  }

  /* Push to transmit queue */
  print_message ((uint8_t*) pmt::blob_data (blob), pmt::blob_length (blob));
  uint8_t dest_suc_id = ((uint8_t*) pmt::blob_data (blob))[0];
  if (d_nwk_dev_type == SUR && dest_suc_id == d_assoc_suc_id) {
//    dout << "SUR: push to transmit queue parent, dest: " << int (dest_suc_id)
//        << endl;

    if (!transmit_queue[TX_THREAD_PARENT].push (blob)) {
//      dout << "MAC: push packet to transmit_queue failed." << endl;
    }
  }
  else {
    if (!transmit_queue[TX_THREAD_LOCAL].push (blob)) {
//      dout << "MAC: push packet to transmit_queue failed." << endl;
    }
  }
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::transmit_thread_local (void)
{
  pmt::pmt_t blob, next_blob;
  uint8_t *mac_payload_p, *dest_addr_p, *next_dest_addr_p;
  int mac_payload_len, frame_len;
  uint8_t frame_buf[256];
  bool wait_for_beacon = false, suc_wait_for_sur = false, rsend_result,
      retransmit_last_frame = false;

  dout << "Transmit thread local created." << endl;

  while (1) {
    /* Waiting for the right state */
    gr::thread::scoped_lock lock (d_tx_mutex);
    while (d_control_thread_state != DATA_TRANSMISSION_LOCAL) {
      d_tx_condvar.wait (lock);
    }
    lock.unlock ();

    if (!retransmit_last_frame) {
      if (transmit_queue[TX_THREAD_LOCAL].read_available () > 0) {
        /* There is data in transmit queue, pop it out and transmit */
        blob = transmit_queue[TX_THREAD_LOCAL].front ();
        transmit_queue[TX_THREAD_LOCAL].pop ();
        dest_addr_p = (uint8_t*) pmt::blob_data (blob);
        mac_payload_p = (uint8_t*) pmt::blob_data (blob) + 2;
        mac_payload_len = pmt::blob_length (blob) - 2;
        print_message (mac_payload_p, mac_payload_len);

        generate_data_frame (dest_addr_p, d_ext_op_sender, mac_payload_p,
                             mac_payload_len, frame_buf, frame_len);
      }
      else {
        /* No data in the queue */
        continue;
      }
    }
    else {
      dout << "TX_THREAD_LOCAL: Retransmit last frame." << endl;
      retransmit_last_frame = false;
    }

    /* Check dest_address to decide whether we need to wait for beacon or not*/
    wait_for_beacon = false;
    if ((d_nwk_dev_type == SU || d_nwk_dev_type == SUR)
        && (dest_addr_p[0] == (uint8_t) d_assoc_suc_id)
        && (dest_addr_p[1] == 0)) {
      wait_for_beacon = true;
      dout << "TX_THREAD_LOCAL: need to wait for beacon." << endl;
    }

    /* Check dest_address to decide whether SUC needs to wait for SUR to return or not */
    suc_wait_for_sur = false;
    if (d_nwk_dev_type == SUC && (dest_addr_p[1] == 0)) {
      suc_wait_for_sur = true;
      dout << "TX_THREAD_LOCAL: SUC needs to wait for SUR." << endl;
    }

    rsend_result = csma_ca_rsend (TX_THREAD_LOCAL, DATA_TRANSMISSION_LOCAL,
                                  wait_for_beacon, suc_wait_for_sur, frame_buf,
                                  frame_len);

    /* Check resend result when we are in extended operation */
    if (d_ext_op_sender && !rsend_result) {
      /* We should return to common hopping channel */
      d_ext_op_sender = false;
      retransmit_last_frame = true;
      d_control_thread_state = NULL_STATE;
      dout << "TX_THREAD_LOCAL: Returning to CHC." << endl;
      continue;
    }

    /* Check next packet to see whether we should turn on extended operation */
    /* Only check when the last rsend was successful and in correct state */
    if (d_is_ext_op_needed) {
      if (rsend_result && d_control_thread_state == DATA_TRANSMISSION_LOCAL) {
        if (transmit_queue[TX_THREAD_LOCAL].read_available () > 0) {
          next_blob = transmit_queue[TX_THREAD_LOCAL].front ();

          next_dest_addr_p = (uint8_t*) pmt::blob_data (next_blob);
          if (dest_addr_p[0] == next_dest_addr_p[0]
              && dest_addr_p[1] == next_dest_addr_p[1]) {
            dout << "TX_THREAD_LOCAL: turn on EXT_OP." << endl;
            d_ext_op_sender = true;
          }
          else {
            dout << "TX_THREAD_LOCAL: turn off EXT_OP (to different addr)."
                << endl;
            d_ext_op_sender = false;
          }
        }
        else {
          dout << "TX_THREAD_LOCAL: turn off EXT_OP (no data in queue)."
              << endl;
          d_ext_op_sender = false;
        }
      }/* End if EXT_OP */
    }
  }/* End while (1) */

}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::transmit_thread_parent (void)
{
  pmt::pmt_t blob, next_blob;
  uint8_t *mac_payload_p, *dest_addr_p, *next_dest_addr_p;
  int mac_payload_len, frame_len;
  uint8_t frame_buf[256];
  bool wait_for_beacon = false, rsend_result, retransmit_last_frame = false;

  dout << "Transmit thread parent created." << endl;

  while (1) {
    /* Waiting for the right state */
    gr::thread::scoped_lock lock (d_tx_mutex);
    while (d_control_thread_state != DATA_TRANSMISSION_PARENT) {
      d_tx_condvar.wait (lock);
    }
    lock.unlock ();

    if (!retransmit_last_frame) {
      if (transmit_queue[TX_THREAD_PARENT].read_available () > 0) {
        /* There is data in transmit queue, pop it out and transmit */
        blob = transmit_queue[TX_THREAD_PARENT].front ();
        transmit_queue[TX_THREAD_PARENT].pop ();
        dest_addr_p = (uint8_t*) pmt::blob_data (blob);
        mac_payload_p = (uint8_t*) pmt::blob_data (blob) + 2;
        mac_payload_len = pmt::blob_length (blob) - 2;
        print_message (mac_payload_p, mac_payload_len);

        generate_data_frame (dest_addr_p, d_ext_op_sender, mac_payload_p,
                             mac_payload_len, frame_buf, frame_len);
      }
      else {
        /* No data in the queue */
        continue;
      }
    }
    else {
      dout << "TX_THREAD_PARENT: Retransmit last frame." << endl;
      retransmit_last_frame = false;
    }

    /* Check dest_address to decide whether we need to wait for beacon or not*/
    wait_for_beacon = false;
    if ((d_nwk_dev_type == SU || d_nwk_dev_type == SUR)
        && (dest_addr_p[0] == (uint8_t) d_assoc_suc_id)
        && (dest_addr_p[1] == 0)) {
      wait_for_beacon = true;
      dout << "TX_THREAD_PARENT: need to wait for beacon." << endl;
    }

    rsend_result = csma_ca_rsend (TX_THREAD_PARENT, DATA_TRANSMISSION_PARENT,
                                  wait_for_beacon, false, frame_buf, frame_len);

    /* Check resend result when we are in extended operation */
    if (d_ext_op_sender && !rsend_result) {
      /* We should return to common hopping channel */
      d_ext_op_sender = false;
      retransmit_last_frame = true;
      d_control_thread_state = NULL_STATE;
      dout << "TX_THREAD_PARENT: Returning to CHC." << endl;
      continue;
    }

    /* Check next packet to see whether we should turn on extended operation */
    /* Only check when the last rsend was successful and in correct state */
    if (d_is_ext_op_needed) {
      if (rsend_result && d_control_thread_state == DATA_TRANSMISSION_PARENT) {
        if (transmit_queue[TX_THREAD_PARENT].read_available () > 0) {
          next_blob = transmit_queue[TX_THREAD_PARENT].front ();

          next_dest_addr_p = (uint8_t*) pmt::blob_data (next_blob);
          if (dest_addr_p[0] == next_dest_addr_p[0]
              && dest_addr_p[1] == next_dest_addr_p[1]) {
            dout << "TX_THREAD_PARENT: turn on EXT_OP." << endl;
            d_ext_op_sender = true;
          }
          else {
            dout << "TX_THREAD_PARENT: turn off EXT_OP (to different addr)."
                << endl;
            d_ext_op_sender = false;
          }
        }
        else {
          dout << "TX_THREAD_PARENT: turn off EXT_OP (no data in queue)."
              << endl;
          d_ext_op_sender = false;
        }
      } /* End if EXT_OP */
    }
  } /* End while (1) */
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
shcs_mac_impl::generate_data_frame (const uint8_t *dest_addr, const bool ext_op,
                                    const uint8_t *data_payload,
                                    int payload_len, uint8_t *obuf, int &olen)
{
  uint8_t mhr[IEEE802154_MAX_HDR_LEN];
  uint8_t flags = IEEE802154_FCF_TYPE_DATA;
  le_uint16_t pan_id_le;

  if (ext_op) {
    flags |= IEEE802154_SHCS_EXT_OPERATION;
  }

  if (d_nwk_dev_type == SUC) {
    pan_id_le = byteorder_btols (byteorder_htons (d_suc_id));
  }
  else if (d_nwk_dev_type == SU) {
    pan_id_le = byteorder_btols (byteorder_htons (d_assoc_suc_id));
  }

  /* Peak to get src addr and dest addr */
  if ((olen = ieee802154_set_frame_hdr (mhr, d_mac_addr, 2, dest_addr, 2,
                                        pan_id_le, pan_id_le, flags, d_seq_nr++))
      == 0) {
    dout << "MAC: header error." << endl;
  }
  else {
    /* Copy header to MAC frame */
    memcpy (obuf, mhr, olen);

    /* Prepare the data payload */
    memcpy (&obuf[olen], data_payload, payload_len);
    olen += payload_len;

    /* Calculate FCS */
    uint16_t fcs = crc16 (obuf, olen);

    /* Add FCS to frame */
    obuf[olen++] = (uint8_t) fcs;
    obuf[olen++] = (uint8_t) (fcs >> 8);
  }
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::generate_ack_frame (const uint8_t *dest_addr, int seqno,
                                   uint8_t *obuf, int &olen)
{
  uint8_t mhr[IEEE802154_MAX_HDR_LEN];
  uint8_t flags = IEEE802154_FCF_TYPE_ACK;
  le_uint16_t pan_id_le;
  if (d_nwk_dev_type == SUC) {
    pan_id_le = byteorder_btols (byteorder_htons (d_suc_id));
  }
  else if (d_nwk_dev_type == SU) {
    pan_id_le = byteorder_btols (byteorder_htons (d_assoc_suc_id));
  }

  /* Peak to get src addr and dest addr */
  if ((olen = ieee802154_set_frame_hdr (mhr, d_mac_addr, 2, dest_addr, 2,
                                        pan_id_le, pan_id_le, flags, seqno))
      == 0) {
    dout << "MAC: header error." << endl;
  }
  else {
    /* Copy header to MAC frame */
    memcpy (obuf, mhr, olen);

    /* Calculate FCS */
    uint16_t fcs = crc16 (obuf, olen);

    /* Add FCS to frame */
    obuf[olen++] = (uint8_t) fcs;
    obuf[olen++] = (uint8_t) (fcs >> 8);

  }
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::generate_tpsn_req (const uint8_t *dest_addr)
{
  uint8_t obuf[IEEE802154_MAX_HDR_LEN + 2];
  int olen;
  uint8_t mhr[IEEE802154_MAX_HDR_LEN];
  uint8_t flags = IEEE802154_FCF_TYPE_TPSN_REQ;
  le_uint16_t pan_id_le;

  if (d_nwk_dev_type == SUC) {
    pan_id_le = byteorder_btols (byteorder_htons (d_suc_id));
  }
  else if (d_nwk_dev_type == SU) {
    pan_id_le = byteorder_btols (byteorder_htons (d_assoc_suc_id));
  }

  d_tpsn_waiting_seqno = d_seq_nr;

  if ((olen = ieee802154_set_frame_hdr (mhr, d_mac_addr, 2, dest_addr, 2,
                                        pan_id_le, pan_id_le, flags, d_seq_nr++))
      == 0) {
    dout << "MAC: header error." << endl;
  }
  else {
    /* Copy header to MAC frame */
    memcpy (obuf, mhr, olen);

    /* Calculate FCS */
    uint16_t fcs = crc16 (obuf, olen);

    /* Add FCS to frame */
    obuf[olen++] = (uint8_t) fcs;
    obuf[olen++] = (uint8_t) (fcs >> 8);
  }

  /* take timestamp */
  boost::posix_time::time_duration t1_td;
  t1_td = boost::posix_time::microsec_clock::universal_time ()
      - ref_point_ptime;
  t1 = t1_td.total_microseconds ();

  phy_transmit (obuf, olen);

  dout << "MAC: sent TPSN req to " << hex << d_assoc_suc_id << dec << ", t1: "
      << t1 << endl;
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::generate_tpsn_ack (const uint8_t *dest_addr, int seqno,
                                  boost::posix_time::ptime &received_timestamp)
{
  uint8_t obuf[IEEE802154_MAX_HDR_LEN + 16];
  int olen;
  uint8_t mhr[IEEE802154_MAX_HDR_LEN];
  uint8_t flags = IEEE802154_FCF_TYPE_TPSN_ACK;
  le_uint16_t pan_id_le;

  if (d_nwk_dev_type == SUC) {
    pan_id_le = byteorder_btols (byteorder_htons (d_suc_id));
  }
  else if (d_nwk_dev_type == SU) {
    pan_id_le = byteorder_btols (byteorder_htons (d_assoc_suc_id));
  }

  if ((olen = ieee802154_set_frame_hdr (mhr, d_mac_addr, 2, dest_addr, 2,
                                        pan_id_le, pan_id_le, flags, seqno))
      == 0) {
    dout << "MAC: header error." << endl;
  }
  else {
    /* Copy header to MAC frame */
    memcpy (obuf, mhr, olen);

    /* Prepare the data payload */
    boost::posix_time::time_duration t2_td, t3_td;
    t2_td = received_timestamp - ref_point_ptime;
    t3_td = boost::posix_time::microsec_clock::universal_time ()
        - ref_point_ptime;

    uint64_to_buffer ((uint64_t) t2_td.total_microseconds (), &obuf[olen]);
    olen += 8;
    uint64_to_buffer ((uint64_t) t3_td.total_microseconds (), &obuf[olen]);
    olen += 8;

    /* Calculate FCS */
    uint16_t fcs = crc16 (obuf, olen);

    /* Add FCS to frame */
    obuf[olen++] = (uint8_t) fcs;
    obuf[olen++] = (uint8_t) (fcs >> 8);

    /* Transmit */
    phy_transmit (obuf, olen);

    dout << "MAC: TPSN ACK" << "\n" << " t2: " << t2_td.total_microseconds ()
        << "\n" << " t3: " << t3_td.total_microseconds () << "\n";
  }
}

/*------------------------------------------------------------------------*/
void
shcs_mac_impl::generate_rbs_timestamps_packet (
    boost::posix_time::ptime &beacon_sent_timestamp,
    boost::posix_time::ptime &beacon_received_timestamp)
{
  uint8_t obuf[255];
  int olen;
  uint8_t mhr[IEEE802154_MAX_HDR_LEN];
  uint8_t flags = IEEE802154_FCF_TYPE_RBS_TIMESTAMPS
      | IEEE802154_FCF_SRC_ADDR_SHORT | IEEE802154_FCF_SRC_ADDR_VOID;
  le_uint16_t pan_id_le;

  if (d_nwk_dev_type == SUC) {
    pan_id_le = byteorder_btols (byteorder_htons (d_suc_id));
  }
  else if (d_nwk_dev_type == SU) {
    pan_id_le = byteorder_btols (byteorder_htons (d_assoc_suc_id));
  }

  if ((olen = ieee802154_set_frame_hdr (mhr, d_mac_addr, 2, d_broadcast_addr, 2,
                                        pan_id_le, pan_id_le, flags, d_seq_nr++))
      == 0) {
    dout << "MAC: header error." << endl;
  }
  else {
    /* Copy header to MAC frame */
    memcpy (obuf, mhr, olen);

    /* Prepare the data payload */
    uint16_to_buffer (d_suc_id, &obuf[olen]);
    olen += 2;

    boost::posix_time::time_duration beacon_recv_td, delay_td;
    beacon_recv_td = beacon_received_timestamp - ref_point_ptime;
    delay_td = beacon_received_timestamp - beacon_sent_timestamp;

    uint64_to_buffer ((uint64_t) beacon_recv_td.total_microseconds (),
                      &obuf[olen]);
    olen += 8;
    uint64_to_buffer ((uint64_t) delay_td.total_microseconds (), &obuf[olen]);
    olen += 8;

    /* Calculate FCS */
    uint16_t fcs = crc16 (obuf, olen);

    /* Add FCS to frame */
    obuf[olen++] = (uint8_t) fcs;
    obuf[olen++] = (uint8_t) (fcs >> 8);

    /* Transmit */
    csma_ca_send (NULL_STATE, false, false, obuf, olen);

    dout << "MAC: Sent RBS timestamps packet " << "\n" << " beacon_recv: "
        << beacon_recv_td.total_microseconds () << "\n" << " delay: "
        << delay_td.total_microseconds () << "\n";
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
//void
//shcs_mac_impl::reporting_thread_func (void)
//{
//  int count = 0;
//
//  while (1) {
//    d_num_bytes_received = 0;
//
//    /* Sleep for 10s  */
//    boost::this_thread::sleep_for (boost::chrono::seconds (d_reporting_period));
//
//    /* Reporting */
//    std::cout << "MAC: Reports #" << count << ", avg data rate: "
//        << d_num_bytes_received * 8 / 1024 / d_reporting_period << " kbit/s"
//        << std::endl;
//
//    count++;
//  }
//}
/*------------------------------------------------------------------------*/
// TODO: not great! so buggy, changed to channel hopping for now.
void
shcs_mac_impl::reporting_thread_func (void)
{
  boost::posix_time::ptime cur_time, next_sec_ptime;
  int64_t cur_time_us, cur_time_s;
  static bool is_first_time = true;

  cur_time = boost::posix_time::microsec_clock::universal_time ();
  cur_time_us = (cur_time - ref_point_ptime).total_microseconds ();
  /* Round to the nearest second */
  cur_time_s = ((cur_time_us + 1000000 / 2) / 1000000);

  if (is_first_time) {
    //dout << "MAC: Reporting: skip first run!." << endl;
    cur_time_s++;
    next_sec_ptime = ref_point_ptime + boost::posix_time::seconds (cur_time_s);
    is_first_time = false;

    tasks_processor::get ().run_at (
        next_sec_ptime,
        boost::bind (&shcs_mac_impl::reporting_thread_func, this));
    return;
  }

  /* Turn on LED on even seconds, off on odd seconds of Ref time */
  if (will_gpio_be_on) {
    usrp_gpio_on (0);
    cout << "\n" << cur_time << ": GPIO 0 on." << endl;
  }
  else {
    usrp_gpio_off (0);
    cout << "\n" << cur_time << ": GPIO 0 off." << endl;
  }

  /* Run at the next second */
  if (d_nwk_dev_type == SUC || rbs_t_local0 <= 0) {
    cur_time_s++;
    next_sec_ptime = ref_point_ptime + boost::posix_time::seconds (cur_time_s);
    if (cur_time_s % 2 == 0) {
      will_gpio_be_on = true;
    }
    else {
      will_gpio_be_on = false;
    }
  }
  else {
    int64_t cur_local_time_since_ref0 = cur_time_us - rbs_t_local0;
    double cur_ref_time_since_ref0_db =
        (static_cast<double> (cur_local_time_since_ref0) - rbs_current_offset)
            / rbs_modifier;
    int64_t cur_ref_time_us = static_cast<int64_t> (cur_ref_time_since_ref0_db)
        + rbs_t_ref0;
    int64_t next_sec_ref_time_s = ((cur_ref_time_us + 1000000 / 2) / 1000000)
        + 1;
    int64_t time_until_next_sec_ref_since_ref0_us = next_sec_ref_time_s
        * 1000000 - rbs_t_ref0;
    double time_until_next_sec_local_since_ref0_us =
        static_cast<double> (time_until_next_sec_ref_since_ref0_us)
            * rbs_modifier + rbs_current_offset;

    next_sec_ptime =
        ref_point_ptime
            + boost::posix_time::microseconds (
                rbs_t_local0
                    + static_cast<int64_t> (time_until_next_sec_local_since_ref0_us));
    if (next_sec_ref_time_s % 2 == 0) {
      will_gpio_be_on = true;
    }
    else {
      will_gpio_be_on = false;
    }
  }

  tasks_processor::get ().run_at (
      next_sec_ptime,
      boost::bind (&shcs_mac_impl::reporting_thread_func, this));
}

/*----------------------------------------------------------------------------*/
void
shcs_mac_impl::usrp_gpio_init (void)
{
  /* This is definitively not good code. Messange passing will be much cleaner.
   * However, since message passing is not supported yet (July 20, 2018), this
   * is the only choice to access GPIOs on USRP device */
  basic_block_sptr blk;

// call print print self.uhd_usrp_source_0.alias() to get this alias.
  const string usrp_alias = "gr uhd usrp source0";
  try {
    blk = global_block_registry.block_lookup (pmt::intern (usrp_alias));
    d_usrp = boost::dynamic_pointer_cast<gr::uhd::usrp_source> (blk);
  }
  catch (const std::runtime_error& err) {
    dout << "MAC: Unable to find USRP instance in block registry." << endl;
    exit (0);
  }

  dout << "MAC: Found USRP" << endl;

// This is to find the name of GPIO banks
//    std::vector<std::string> gpio_banks;
//    gpio_banks = d_usrp->get_gpio_banks(0);
//    dout << "MAC: GPIO banks:" << endl;
//    for (string i : gpio_banks) {
//      dout << i << endl;
//    }

// set up our masks, defining the pin numbers
  const uint32_t MAN_GPIO_MASK = 0x0F;
// set up our values for ATR control: 1 for ATR, 0 for manual
  const uint32_t ATR_CONTROL = ~MAN_GPIO_MASK;
// set up the GPIO directions: 1 for output, 0 for input
  const uint32_t GPIO_DDR = MAN_GPIO_MASK;

// now, let's do the basic ATR setup
  d_usrp->set_gpio_attr ("FP0", "CTRL", ATR_CONTROL, MAN_GPIO_MASK);
  d_usrp->set_gpio_attr ("FP0", "DDR", GPIO_DDR, MAN_GPIO_MASK);
}

/*----------------------------------------------------------------------------*/
void
shcs_mac_impl::usrp_gpio_on (int pin)
{
  uint32_t mask = 1 << pin;
  d_usrp->set_gpio_attr ("FP0", "OUT", 1 << pin, mask);
}

/*----------------------------------------------------------------------------*/
void
shcs_mac_impl::usrp_gpio_off (int pin)
{
  uint32_t mask = 1 << pin;
  d_usrp->set_gpio_attr ("FP0", "OUT", 0, mask);
}

/*----------------------------------------------------------------------------*/
void
shcs_mac_impl::usrp_gpio_toggle (int pin)
{
  uint32_t mask = 1 << pin;
  uint32_t retval = d_usrp->get_gpio_attr ("FP0", "OUT");

  if ((retval & mask) == 0) {
    usrp_gpio_on (pin);
  }
  else {
    usrp_gpio_off (pin);
  }
}

/*----------------------------------------------------------------------------*/
double
shcs_mac_impl::lr_slope (const vector<double>& x, const vector<double>& y)
{
  if (x.size () != y.size ()) {
    dout << "MAC: lr_slope: x.size != y.size, return 0";
  }
  double n = x.size ();

  // Print x, y
//  dout << "x: ";
//  for (int i : x) {
//    dout << i << ", ";
//  }
//  dout << endl;
//
//  dout << "y: ";
//  for (int i : y) {
//    dout << i << ", ";
//  }
//  dout << endl;

  double avgX = accumulate (x.begin (), x.end (), 0.0) / n;
  double avgY = accumulate (y.begin (), y.end (), 0.0) / n;

  double numerator = 0.0;
  double denominator = 0.0;

  for (int i = 0; i < n; ++i) {
    numerator += (x[i] - avgX) * (y[i] - avgY);
    denominator += (x[i] - avgX) * (x[i] - avgX);
  }

  if (denominator == 0) {
    dout << "MAC: lr_slope: denominator = 0, return 0";
    return 0;
  }

  return numerator / denominator;
}

/*----------------------------------------------------------------------------*/
uint16_t
shcs_mac_impl::buffer_to_uint16 (uint8_t * buffer)
{
  uint16_t ret_val;

  ret_val = (*(buffer + 1)) | (*(buffer) << 8);

  return ret_val;
}

/*----------------------------------------------------------------------------*/
uint32_t
shcs_mac_impl::buffer_to_uint32 (uint8_t * buffer)
{
  uint32_t ret_val;

  ret_val = (*(buffer + 3)) | (*(buffer + 2) << 8) | (*(buffer + 1) << 16)
      | (*(buffer) << 24);

  return ret_val;
}

/*----------------------------------------------------------------------------*/
uint64_t
shcs_mac_impl::buffer_to_uint64 (uint8_t * buffer)
{
  uint64_t ret_val = 0;

  for (int count = 0; count < 8; count++) {
    ret_val |= (uint64_t) *(buffer + count) << ((7 - count) * 8);
  }

  return ret_val;
}

/*----------------------------------------------------------------------------*/
void
shcs_mac_impl::uint16_to_buffer (uint16_t data, uint8_t* buffer)
{
  *buffer = (uint8_t) (data >> 8);
  *(++buffer) = (uint8_t) (data);
}

/*----------------------------------------------------------------------------*/
void
shcs_mac_impl::uint32_to_buffer (uint32_t data, uint8_t* buffer)
{
  *buffer = (uint8_t) (data >> 24);
  *(++buffer) = (uint8_t) (data >> 16);
  *(++buffer) = (uint8_t) (data >> 8);
  *(++buffer) = (uint8_t) (data);
}

/*----------------------------------------------------------------------------*/
void
shcs_mac_impl::uint64_to_buffer (uint64_t data, uint8_t* buffer)
{
  *buffer = (uint8_t) (data >> 56);
  *(++buffer) = (uint8_t) (data >> 48);
  *(++buffer) = (uint8_t) (data >> 40);
  *(++buffer) = (uint8_t) (data >> 32);
  *(++buffer) = (uint8_t) (data >> 24);
  *(++buffer) = (uint8_t) (data >> 16);
  *(++buffer) = (uint8_t) (data >> 8);
  *(++buffer) = (uint8_t) (data);
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

