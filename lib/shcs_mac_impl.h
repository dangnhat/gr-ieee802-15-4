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
 * @file shcs_mac_impl.h
 * @author Nhat Pham <nhatphd@kaist.ac.kr>, Real-time and Embedded Systems Lab
 * @version 1.0
 * @date 2018-07-28
 * @brief This is the implementation for MSHCS MAC protocol.
 */

#ifndef INCLUDED_IEEE802_15_4_SHCS_MAC_IMPL_H
#define INCLUDED_IEEE802_15_4_SHCS_MAC_IMPL_H

#include <ieee802_15_4/shcs_mac.h>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/asio.hpp>
#include <vector>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/circular_buffer.hpp>

#include "linearregression.h"

/* USRP GPIO */
#include <gnuradio/uhd/usrp_source.h>

/* RBS delay moving average */
namespace ba = boost::accumulators;
namespace bt = ba::tag;
typedef ba::accumulator_set<int64_t, ba::stats<bt::rolling_mean> > MeanAccumulator;

namespace gr
{
  namespace ieee802_15_4
  {
    enum NWK_TYPE
    {
      SU = 0, SUC = 1, SUR = 2,
    };

    enum control_thread_state_e
    {
      NULL_STATE,
      SU_BOOTSTRAPPING,
      SPECTRUM_SENSING,
      BEACON,
      REPORTING,
      DATA_TRANSMISSION_LOCAL,
      DATA_TRANSMISSION_PARENT,
      RELOADING
    };

    enum sur_state_e
    {
      IN_LOCAL_NWK, IN_PARENT_NWK,
    };

    enum transmit_thread_id_e
    {
      TX_THREAD_LOCAL = 0, TX_THREAD_PARENT = 1,
    };

    class shcs_mac_impl : public shcs_mac
    {
    public:
      /**
       * @brief   Constructor.
       *
       * @param[in]   debug, turn on/off debugging messages.
       */
      shcs_mac_impl (bool debug, int nwk_dev_type,
                     std::vector<uint8_t> mac_addr, int suc_id,
                     int assoc_suc_id, int fft_len);

      /**
       * @brief   Destructor.
       *
       * @param[in]   debug, turn on/off debugging messages.
       */
      ~shcs_mac_impl ();

      /**
       * @brief   Work function overload. It's used for spectrum sensing.
       *
       * \param noutput_items  number of output and input items to write on each in/out stream
       * \param input_items vector of pointers to the input items, one entry per input stream
       * \param output_items  vector of pointers to the output items, one entry per output stream
       *
       * \returns number of items actually written to each output stream, or -1 on EOF.
       * It is OK to return a value less than noutput_items.  -1 <= return value <= noutput_items
       */
      int
      work (int noutput_items, gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);

      /**
       * @brief   Return number of error packets.
       *
       * @return  Number of error packets.
       */
      int
      get_num_packet_errors ();

      /**
       * @brief   Return number of received packets.
       *
       * @return  Number of received packets.
       */
      int
      get_num_packets_received ();

      /**
       * @brief   Return error ratio.
       *
       * @return  Error ratio.
       */
      float
      get_packet_error_ratio ();

      /*--------------------------------- Private -----------------------------*/
    private:
      bool d_debug;
      int d_msg_offset;
      uint8_t d_seq_nr;
      gr::thread::mutex d_seq_nr_m;
      int d_fft_len;

      int d_num_packet_errors;
      int d_num_packets_received;

      /* network device type */
      int d_nwk_dev_type;

      /* wireless channel configuration */
      static const int num_of_channels = 4; // channel 23 -> 26: [2.465, ..., 2.480] GHz,
      const double channel_step = 5e6; // 5MHz step between 2 channels.
      const int first_channel_index = 24;
      double center_freqs[num_of_channels] = { 2.465e9 }; // channel 23: 2.465GHz.
      int max_prio = 100;
      int channel_prios[num_of_channels - 1] = { 25, 50, 75 }; /* CDF style */

      const double bandwidth = 2e6;      // Hz, constant for LR-WPAN.
      const double sampling_rate = 4e6;  // Hz,

      static const uint32_t Ts = 300; // ms, slot duration (i.e. dwelling time of a channel hop).
      static const uint32_t Tf = Ts * num_of_channels; // ms, frame duration.
      static const uint32_t Th = 5; // ms, channel hopping duration.
      static const uint32_t Tss = 10; // ms, sensing duration.
      static const uint32_t d_guard_time = 3; // ms, guard time at the end of sensing duration.
      static const uint32_t Tb = 10; // ms, beacon duration.
      static const uint32_t Tr = 5; // ms, reporting duration.
      static const uint32_t Tdata = Ts - Th - Tss - Tb - Tr;

      uint32_t Ts_counter = 0;

      /* csma_with_ack for reliable unicast */
      static const bool csma_with_ack = false;

      uint16_t d_suc_id = 0xFFFF;
      uint16_t d_assoc_suc_id = 0xFFFF; // 0xFFFF means it can be changed after getting beacon.
      const uint8_t d_broadcast_addr[2] = { 0xFF, 0xFF };
      uint8_t d_mac_addr[2] = { 0x0, 0x0 };

      bool d_assoc_current_sur_state = IN_PARENT_NWK; // Only be used in SUC's beacon,
      // should always be IN_PARENT_NWK for SUR's beacon
      const uint8_t EXT_OP_POS = 0;
      const uint8_t ASSOC_CURRENT_SUR_STATE_POS = 1;

      /* SUR state */
      bool d_sur_state = IN_PARENT_NWK;

      /* Time frame related variables */
      boost::random::mt19937 seed_gen;
      boost::random::minstd_rand rng;
      uint32_t current_rand_seed;
      boost::random::minstd_rand rng_local; /* rng_local, and current_rand_seed_local are for SUR only */
      uint32_t current_rand_seed_local;
      boost::posix_time::ptime time_slot_start;

      /* Channel hopping */
      uint32_t current_working_channel;

      /* Spectrum sensing */
      bool is_spectrum_sensing_completed = false;
      bool is_channel_available = false;
      double d_ss_threshold_dBm = 10; // dBm, 50% of 20dBm.
      double d_avg_power = 0; // W.
      uint32_t d_avg_power_count = 0;

      /* Beacon duration */
      bool is_beacon_received = false; // SU only.
      bool is_busy_signal_received = false;

      /* Control thread */
      boost::shared_ptr<gr::thread::thread> control_thread_ptr;
      uint16_t d_control_thread_state = NULL_STATE;

      /* Transmission thread */
      bool d_su_connected = false;
      static const int max_transmit_threads = 2;

      /* Transmit queue */
      static const long unsigned int d_transmit_queue_size = 128;
      boost::shared_ptr<gr::thread::thread> transmit_thread_ptr[max_transmit_threads];
      boost::lockfree::spsc_queue<pmt::pmt_t,
          boost::lockfree::capacity<d_transmit_queue_size>> transmit_queue[max_transmit_threads];

      /* CCA */
      const int cca_time = 1; // ms
      const int cca_threshold = 7; // dBm
      bool d_cca_state = false;

      /* CSMA-CA */
      const int max_csma_ca_backoffs = 10;
      const int max_csma_ca_be = 5; /* maximum backoff exponential */
      const int csma_ca_backoff_unit = 1; /* in ms */
      gr::thread::condition_variable d_tx_condvar;
      gr::thread::mutex d_tx_mutex;

      /* CSMA-CA rsend */
      static const int max_retries = 3;
      static const int max_retry_timeout = 2 * Ts; /* ms */
      gr::thread::condition_variable d_ack_received_cv[max_transmit_threads];
      gr::thread::mutex d_ack_m[max_transmit_threads];
      bool d_is_ack_received[max_transmit_threads] = { false, false };
      //uint8_t d_ack_src_addr[max_transmit_threads];
      uint8_t d_ack_recv_seq_nr[max_transmit_threads];
      uint8_t d_last_recv_seqno = 0;
      bool d_is_first_recv_data_frame = true;

      /* Extended operation */
      bool d_is_ext_op = false; // true to turn on extended operation.

      /* Sender side */
      bool d_ext_op_sender = false;

      /* Receiving side */
      static const int max_ext_op_recv_wait_time = (max_retries
          * max_retry_timeout) / Ts + 1; /* in number of time slots */
//      static const int max_ext_op_recv_wait_time = 3; /* in number of time slots */
      int d_ext_op_recv_wait_time_left = 0;
      gr::thread::mutex d_ext_op_recv_wait_time_mutex;

      /* Reporting thread */
      const int d_reporting_period = 10; /* s */
      uint64_t d_num_bytes_received = 0;
      boost::shared_ptr<gr::thread::thread> reporting_thread_ptr;

      /* Time synchronization */
      const char ref_point_c[64] = "1970-01-01 00:00:00.000";
      boost::posix_time::ptime ref_point_ptime;

      /* Modified version of RBS */
      const int rbs_sync_period = 1000; //ms
      uint8_t d_last_recv_seqno_rbs = 0;
      boost::posix_time::ptime rbs_last_beacon_send_timestamp;
      boost::posix_time::ptime rbs_last_beacon_rcv_timestamp;
      bool rbs_send_timestamps_packet = false; // signal to send timestamps in data duration.
      bool rbs_sync_period_flag = false;
      /* RBS moving average delay calculation */
      const int rbs_delay_window_size = 100;
      boost::shared_ptr<MeanAccumulator> rbs_delay_acc_ptr;
      /* RBS linear regression for offsets */
      const int rbs_offset_max_samples = 10;
      const int rbs_linear_regression_min_new_samples = 5;
      uint32_t rbs_new_samples_counter = 0;
      boost::shared_ptr<boost::circular_buffer<int64_t>> rbs_t_locals_ptr,
          rbs_t_refs_ptr;
      lin_reg rbs_linear_regess {1.0, 0.0};
      int64_t rbs_t_local_ref = 0;

      double rbs_modifier = 1.0, rbs_current_offset = 0.0; // time in local clock / time in ref clock.
      int64_t rbs_avg_delay = 0; // in us.

      /* TPSN */
      uint8_t d_tpsn_waiting_seqno = 0;
      int64_t t1, offset, delay; /* in us */

      /* USRP GPIO */
      gr::uhd::usrp_source::sptr d_usrp; // pointer to USRP object.

      /**
       * @brief   Control thread for Coordinator.
       */
      void
      suc_control_thread (void);

      /**
       * @brief   Control thread for SU.
       */
      void
      su_control_thread (void);

      /**
       * @brief   Control thread for Router.
       */
      void
      sur_control_thread (void);

      /**
       * @brief   Transmission thread.
       */
      void
      transmit_thread_local (void);

      /**
       * @brief   Transmission thread in local network (for SUR).
       */
      void
      transmit_thread_parent (void);

      /**
       * @brief   Handle package from PHY layer and forward processed package
       *          to upper layer.
       *
       * @param[in]   msg, message demodulated by PHY layer.
       */
      void
      mac_in (pmt::pmt_t msg);

      /**
       * @brief   Handle package from NETWORK layer and forward processed
       *          package to PHY layer.
       *
       * @param[in]   msg, message received from NETWORK layer.
       */
      void
      app_in (pmt::pmt_t msg);

      /**
       * @brief   Generate data frame from a received buffer and store frame to
       *          output buffer.
       *
       * @param[in]   dest_addr, destination address (2 bytes).
       * @param[in]   ext_op, extended operation.
       * @param[in]   data_payload, data payload buffer.
       * @param[in]   payload_len, data payload buffer length.
       * @param[out]  obuf, output buffer.
       * @param[in]   olen, output buffer length.
       *
       * Used private vars:
       * - d_seq_nr
       * - d_ext_operation
       */
      void
      generate_data_frame (const uint8_t *dest_addr, const bool ext_op,
                           const uint8_t *data_payload, int payload_len,
                           uint8_t *obuf, int &olen);

      /**
       * @brief   Generate ACK frame to a buffer.
       *
       * @param[in]   dest_addr, destination address (2 bytes).
       * @param[in]   seqno, sequence number to ack.
       * @param[out]  obuf, output buffer.
       * @param[in]   olen, output buffer length.
       *
       * Used private vars:
       * - d_seq_nr
       * - d_ext_operation
       */
      void
      generate_ack_frame (const uint8_t *dest_addr, int seqno, uint8_t *obuf,
                          int &olen);

      /**
       * @brief   Calculate CRC16 checksum for a buffer.
       *
       * @param[in]   buf, buffer.
       * @param[in]   len, buffer length.
       *
       * @return      crc16.
       */
      uint16_t
      crc16 (uint8_t *buf, int len);

      /**
       * @brief   Print message in buffer.
       */
      void
      print_message (uint8_t* buf, int len);

      /**
       * @brief   Channel hopping. Generate a new seed, and move to new channel
       * IN:
       * - rng.
       * OUT:
       * - rng.
       * - current_rand_seed.
       * - current_working_channel.
       */
      void
      channel_hopping (void);

      /**
       * @brief   Convert seed to current working channel.
       * (new channel = (current seed % max_prio -> channel_prios mapping)).
       * IN:
       * - channel_prios.
       * - seed.
       * OUT:
       * - current_working_channel.
       */
      uint32_t
      get_current_working_channel_from_seed (uint32_t seed);

      /**
       * @brief   Perform local spectrum sensing within sensing time period.
       * IN:
       * - Tss.
       * OUT:
       * - is_spectrum_sensing_completed.
       * - is_channel_available.
       */
      void
      spectrum_sensing (void);

      /**
       * @brief   Clear channel assessment. Return whether channel is busy or not
       * after cca_time.
       *
       * @param[in] cca_time (ms), time to measure channel's availability.
       * @param[in] ed_threshold (in dBm), threshold for energy detection.
       *
       * @return    true when channel is available. False, otherwise.
       *
       * Used private vars:
       * - d_avg_power
       * - d_cca_state
       * - d_avg_power_count
       * - work function will output average channel power to d_avg_power while
       * d_cca_state is being true.
       * - cca_time, cca_threshold
       */
      bool
      cca (void);

      /**
       * @brief   PHY transmit. Send data in buffer to physical layer.
       *
       */
      void
      phy_transmit (const uint8_t *buf, int len);

      /**
       * @brief   Perform CCA before sending packet. Calling thread will sleep
       * when it's waiting. It will only transmit when control_thread_state is
       * at a specified state.
       *
       * @param[in] wait_for_beacon. true if it needs to wait for beacon
       *            before transmitting.
       * @param[in] wait_for_sur. true if SUC needs to wait for SUR to arrive
       *            before transmitting.
       * @param[in] transmit_state. It will wait until control thread is in
       *            this state before sending. NULL_STATE to skip it.
       *
       * @return    true when successful. False, otherwise.
       *
       * Used private vars:
       * - max_csma_ca_backoffs
       * - max_csma_ca_be
       * - csma_ca_backoff_unit
       * - d_msg, d_msg_len: packet we need to send.
       * - d_control_thread_state
       * - is_beacon_received when wait_for_beacon = true.
       */
      bool
      csma_ca_send (uint16_t transmit_state, bool wait_for_beacon,
                    bool wait_for_sur, const uint8_t *buf, int len);

      /**
       * @brief   Perform reliable CSMA CA unicast with Idle RQ.
       *
       * @param[in] wait_for_beacon. true if it needs to wait for beacon
       *            before transmitting.
       * @param[in] transmit_thread_id. It's used in mac_in function to wake up
       *            correct waiting thread.
       * @param[in] transmit_state. It will wait until control thread is in
       *            this state before sending.
       * @param[in] buf. buffer holding data. It will be backed up at the
       *            beginning of this function.
       * @param[in] len. Buffer length.
       *
       * @return    true when successful. False, otherwise.
       *
       * Used private vars:
       * - max_retries, max_retry_timeout
       * - d_ack_received_cv, d_ack_m
       * - d_ack_src_addr[2], d_ack_recv_seq_nr
       * - is_beacon_received when wait_for_beacon = true.
       */
      bool
      csma_ca_rsend (uint8_t transmit_thread_id, uint16_t transmit_state,
                     bool wait_for_beacon, bool suc_wait_for_sur,
                     const uint8_t *buf, int len);

      /**
       * @brief   Safely increase seqno with mutex.
       */
      inline void
      safe_inc_seqno (void)
      {
        gr::thread::scoped_lock lock (d_seq_nr_m);
        d_seq_nr++;
      }

      /**
       * @brief   SUC: broadcast beacon if spectrum sensing returns channel
       * is available.
       *          SU: wait for beacon. Set is_received_beacon accordingly.
       */
      void
      beacon_duration (void);

      /**
       * @brief   both SU and SUC: wait for busy signal. Set is_busy_signal_received
       * accordingly.
       */
      void
      reporting_duration (void);

      /**
       * @brief   both SUC, and SU: if the channel is busy (either
       * is_channel_available = false, is_beacon_received = false, is_signal_received = true),
       * set the state to SLEEPING (drop all packets). Otherwise, set the state
       * to DATA_TRANSMISSION.
       */
      void
      data_duration (void);

      /**
       * @brief   reload tasks at the end of time slot to begin a new one.
       */
      void
      reload_tasks (void);

      /**
       * @brief   SU only, bootstrapping for SU. It will choose a random channel
       * in channels list and wait for beacon for 1 Time frame duration (Tf).
       * When an active beacon is received, SU will do the following tasks upon
       * receiving a beacon:
       * - store SUC ID
       * - store current random seed for common hopping sequence.
       * - store sensing duration time (not needed, will be the same for every nodes)
       * - synchronize time slot starting time.
       * - control_thread_state will be changed to NULL_STATE upon returning.
       * Otherwise, current channel is occupied with PU activity so it will be
       * removed from channels list (currently, it's not implemented).
       * SU will choose another random channel from current channels list.
       */
      void
      su_bootstrapping (void);

      /**
       * @brief   Reporting performance every 1s.
       */
      void
      reporting_thread_func (void);

      /**
       * @brief   Generate and send TPSN request frame. Client side.
       *
       * @param[in]   dest_addr, destination address (2 bytes).
       *
       * Used private vars:
       * - d_seq_nr
       * - ref_point_ptime
       * - t1
       */
      void
      generate_tpsn_req (const uint8_t *dest_addr);

      /**
       * @brief   Generate and send TPSN ack frame. Sever side.
       *
       * @param[in]   dest_addr, destination address (2 bytes).
       * @param[in]   received_timestamp, received timestamp (t2).
       *
       * Used private vars:
       * - d_seq_nr
       * - ref_point_ptime
       */
      void
      generate_tpsn_ack (const uint8_t *dest_addr, int seqno,
                         boost::posix_time::ptime &received_timestamp);

      /**
       * @brief   Generate and send RBS timestamps frame.
       *
       * @param[in]   beacon_sent_timestamp, sent beacon timestamp.
       * @param[in]   beacon_received_timestamp, received beacon timestamp.
       *
       * Used private vars:
       * - d_seq_nr
       * - ref_point_ptime
       */
      void
      generate_rbs_timestamps_packet (
          boost::posix_time::ptime &beacon_sent_timestamp,
          boost::posix_time::ptime &beacon_received_timestamp);

      /**
       * @brief   Init GPIO 0, 1, 2, 3 for manual control as output pins.
       *
       * Used private vars:
       * - d_usrp
       */
      void
      usrp_gpio_init (void);

      /**
       * @brief   Turn on, off, and toggle a gpio pin.
       *
       * Used private vars:
       * - d_usrp
       */
      void
      usrp_gpio_toggle (int pin);
      void
      usrp_gpio_on (int pin);
      void
      usrp_gpio_off (int pin);

      /**
       * @brief   Calculate linear regression slope.
       */
      double lr_slope(const std::vector<double>& x, const std::vector<double>& y);

      /**
       * @brief    Buffer related functions.
       */
      uint16_t
      buffer_to_uint16 (uint8_t* buffer); // LSByte first
      uint32_t
      buffer_to_uint32 (uint8_t* buffer); // LSByte first
      uint64_t
      buffer_to_uint64 (uint8_t* buffer); // LSByte first
      void
      uint16_to_buffer (uint16_t data, uint8_t* buffer); // LSByte fisrt
      void
      uint32_to_buffer (uint32_t data, uint8_t* buffer); // LSByte fisrt
      void
      uint64_to_buffer (uint64_t data, uint8_t* buffer); // LSByte fisrt
      float
      buffer_to_float (uint8_t* buffer); // dec-2byte, frac-2byte (2 digits)
      void
      float_to_buffer (float data, uint8_t* buffer); // dec-2byte, frac-2byte (2 digits)
    };

  } // namespace ieee802_15_4
} // namespace gr

#endif /* INCLUDED_IEEE802_15_4_SHCS_MAC_IMPL_H */

