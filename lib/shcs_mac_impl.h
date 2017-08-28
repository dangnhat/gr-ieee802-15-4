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


namespace gr {
  namespace ieee802_15_4 {

    enum control_thread_state_e {
      NULL_STATE,
      BOOTSTRAPPING,
      SPECTRUM_SENSING,
      BEACON,
      REPORTING,
      RELOADING
    };


    class shcs_mac_impl : public shcs_mac
    {
     public:
      /**
       * @brief   Constructor.
       *
       * @param[in]   debug, turn on/off debugging messages.
       */
      shcs_mac_impl(bool debug, bool nwk_dev_type, int suc_id, int mac_addr);

      /**
       * @brief   Destructor.
       *
       * @param[in]   debug, turn on/off debugging messages.
       */
      ~shcs_mac_impl();

      /**
       * @brief   Return number of error packets.
       *
       * @return  Number of error packets.
       */
      int get_num_packet_errors();

      /**
       * @brief   Return number of received packets.
       *
       * @return  Number of received packets.
       */
      int get_num_packets_received();

      /**
       * @brief   Return error ratio.
       *
       * @return  Error ratio.
       */
      float get_packet_error_ratio();

     /*--------------------------------- Private -----------------------------*/
     private:
       bool        d_debug;
       int         d_msg_offset;
       int         d_msg_len;
       uint8_t     d_seq_nr;
       uint8_t     d_msg[256];

       int d_num_packet_errors;
       int d_num_packets_received;

       /* network device type */
       bool       d_nwk_dev_type;

       /* wireless channel configuration */
       static const int num_of_channels = 16;  // channel 11 -> 26: [2.405, ..., 2.480] GHz,
       const double channel_step = 5e6; // 5MHz step between 2 channels.
       const int first_channel_index = 11;
       double center_freqs[num_of_channels] = {2.405e9}; // channel 11: 2.405GHz.

       const double bandwidth = 2e6;      // Hz, constant for LR-WPAN.
       const double sampling_rate = 4e6;  // Hz,

       const uint32_t Ts = 2000; // ms, slot duration (i.e. dwelling time of a channel hop).
       const uint32_t Tf = Ts*num_of_channels; // ms, frame duration.
       uint16_t Tss = 1000; // ms, sensing duration.
       const uint16_t Tb = 10; // ms, beacon duration.
       const uint16_t Tr = 10; // ms, reporting duration.
       const uint16_t Th = 10; // ms, channel hopping duration.


       uint16_t d_suc_id = 0; // just a random number, for now.
       uint16_t d_mac_addr = 0; // Short mac address (16 bits).

       /* Time frame related variables */
       boost::random::minstd_rand rng;
       uint32_t current_rand_seed = 0;
       boost::posix_time::ptime time_slot_start;

       /* Channel hopping */
       uint32_t current_working_channel;

       /* Spectrum sensing */
       bool is_spectrum_sensing_completed = false;
       bool is_channel_available = false;
       uint32_t spectrum_sensing_dectection_count = 0;

       /* Control thread */
       boost::shared_ptr<gr::thread::thread> control_thread_ptr;
       uint16_t control_thread_state = NULL_STATE;

       /**
        * @brief   Control thread for Coordinator.
        */
       void coor_control_thread(void);

       /**
         * @brief   Control thread for SU.
         */
       void su_control_thread(void);

       /**
        * @brief   Handle package from PHY layer and forward processed package
        *          to upper layer.
        *
        * @param[in]   msg, message demodulated by PHY layer.
        */
       void mac_in(pmt::pmt_t msg);

      /**
       * @brief   Handle package from NETWORK layer and forward processed
       *          package to PHY layer.
       *
       * @param[in]   msg, message received from NETWORK layer.
       */
      void app_in(pmt::pmt_t msg);

      /**
       * @brief   Generate MAC frame from a received buffer.
       *
       * @param[in]   buf, buffer.
       * @param[in]   len, buffer length.
       */
      void generate_mac(const uint8_t *buf, int len);

      /**
       * @brief   Calculate CRC16 checksum for a buffer.
       *
       * @param[in]   buf, buffer.
       * @param[in]   len, buffer length.
       *
       * @return      crc16.
       */
      uint16_t crc16(uint8_t *buf, int len);

      /**
       * @brief   Print message in buffer.
       */
      void print_message();

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
      void channel_hopping(void);

      /**
       * @brief   Perform local spectrum sensing within sensing time period.
       * IN:
       * - Tss.
       * OUT:
       * - is_spectrum_sensing_completed.
       * - is_channel_available.
       */
      void spectrum_sensing(void);

      /**
       * @brief   Call back when signals map is received from signal detector
       *          module.
       *
       * @param[in]   msg, signal map from signal detector module.
       */
      void spectrum_sensing_callback(pmt::pmt_t msg);

      /**
       * @brief   SUC only, broadcast beacon if spectrum sensing returns channel
       * is available.
       */
      void beacon_broadcasting(void);

      /**
       * @brief   SUC only, reload tasks at the end of time slot to begin a new one.
       */
      void reload_tasks(void);

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
      void su_bootstrapping(void);

      /**
       * @brief    Buffer related functions.
       */
      uint16_t buffer_to_uint16(uint8_t* buffer); // LSByte first
      uint32_t buffer_to_uint32(uint8_t* buffer); // LSByte first
      void uint16_to_buffer(uint16_t data, uint8_t* buffer); // LSByte fisrt
      void uint32_to_buffer(uint32_t data, uint8_t* buffer); // LSByte fisrt
      float buffer_to_float(uint8_t* buffer); // dec-2byte, frac-2byte (2 digits)
      void float_to_buffer(float data, uint8_t* buffer); // dec-2byte, frac-2byte (2 digits)
    };

  } // namespace ieee802_15_4
} // namespace gr

#endif /* INCLUDED_IEEE802_15_4_SHCS_MAC_IMPL_H */

