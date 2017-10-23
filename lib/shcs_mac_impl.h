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

#ifndef INCLUDED_IEEE802_15_4_SHCS_MAC_IMPL_H
#define INCLUDED_IEEE802_15_4_SHCS_MAC_IMPL_H

#include <ieee802_15_4/shcs_mac.h>
#include <boost/lockfree/spsc_queue.hpp>

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
      DATA_TRANSMISSION,
      RELOADING
    };

    enum sur_state_e {
      IN_LOCAL_NWK = false,
      IN_PARENT_NWK = true,
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
      int d_msg_len;
      uint8_t d_seq_nr;
      uint8_t d_msg[256];
      int d_fft_len;

      int d_num_packet_errors;
      int d_num_packets_received;

      /* network device type */
      int d_nwk_dev_type;

      /* wireless channel configuration */
      static const int num_of_channels = 4; // channel 23 -> 26: [2.465, ..., 2.480] GHz,
      const double channel_step = 5e6; // 5MHz step between 2 channels.
      const int first_channel_index = 23;
      double center_freqs[num_of_channels] = { 2.465e9 }; // channel 23: 2.465GHz.

      const double bandwidth = 2e6;      // Hz, constant for LR-WPAN.
      const double sampling_rate = 4e6;  // Hz,

      const uint32_t Ts = 500; // ms, slot duration (i.e. dwelling time of a channel hop).
      const uint32_t Tf = Ts * num_of_channels; // ms, frame duration.
      const uint16_t Th = 5; // ms, channel hopping duration.
      uint16_t Tss = 20; // ms, sensing duration.
      const uint16_t Tb = 20; // ms, beacon duration.
      const uint16_t Tr = 5; // ms, reporting duration.
      const uint16_t d_guard_time = 1; // ms, guard time at the end of each duration
                                       // if needed.

      uint16_t d_suc_id = 0xFFFF;
      uint16_t d_assoc_suc_id = 0xFFFF; // 0xFFFF means it can be changed after getting beacon.
      const uint8_t d_broadcast_addr[2] = {0xFF, 0xFF};
      uint8_t d_mac_addr[2] = {0x0, 0x0};

      /* SUR state */
      bool d_sur_state = IN_PARENT_NWK;

      /* TODO: only for demo */
      bool d_su_transmit_state = true;

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
      double d_avg_power_dBm = 0; // dBm
      uint32_t d_avg_power_count = 0;

      /* Beacon duration */
      bool is_beacon_received = false; // SU only.
      bool is_busy_signal_received = false;

      /* Control thread */
      boost::shared_ptr<gr::thread::thread> control_thread_ptr;
      uint16_t d_control_thread_state = NULL_STATE;

      /* Transmission thread */
      const long unsigned int d_transmit_queue_size = 128;

      boost::shared_ptr<gr::thread::thread> transmit_thread_ptr;
      boost::lockfree::spsc_queue<pmt::pmt_t> transmit_queue {
          d_transmit_queue_size };
      bool d_su_connected = false;

      /* For SUR only */
      boost::shared_ptr<gr::thread::thread> transmit_thread_local_ptr;
      boost::lockfree::spsc_queue<pmt::pmt_t> transmit_queue_local {
          d_transmit_queue_size };

      /* Reporting thread */
      const int d_reporting_period = 10000; /* ms */
      uint32_t d_num_bytes_received = 0;
      boost::shared_ptr<gr::thread::thread> reporting_thread_ptr;

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
      transmit_thread (void);

      /**
       * @brief   Transmission thread in local network (for SUR).
       */
      void
      transmit_thread_local (void);

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
       * @brief   Generate MAC frame from a received buffer.
       *
       * @param[in]   buf, buffer.
       * @param[in]   len, buffer length.
       */
      void
      generate_mac (const uint8_t *buf, int len);

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
       * (new channel = current seed % number of channels).
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
       * @brief    Buffer related functions.
       */
      uint16_t
      buffer_to_uint16 (uint8_t* buffer); // LSByte first
      uint32_t
      buffer_to_uint32 (uint8_t* buffer); // LSByte first
      void
      uint16_to_buffer (uint16_t data, uint8_t* buffer); // LSByte fisrt
      void
      uint32_to_buffer (uint32_t data, uint8_t* buffer); // LSByte fisrt
      float
      buffer_to_float (uint8_t* buffer); // dec-2byte, frac-2byte (2 digits)
      void
      float_to_buffer (float data, uint8_t* buffer); // dec-2byte, frac-2byte (2 digits)
    };

  } // namespace ieee802_15_4
} // namespace gr

#endif /* INCLUDED_IEEE802_15_4_SHCS_MAC_IMPL_H */

