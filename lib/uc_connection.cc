/*
 * Copyright (C) 2013 Christoph Leitner <c.leitner@student.uibk.ac.at>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "uc_connection.h"
#include "bc_connection.h"

using namespace gr::ieee802_15_4;
using namespace std;

uc_connection::uc_connection (rime_stack *block, uint16_t channel,
                              pmt::pmt_t inport, pmt::pmt_t outport,
                              const uint8_t rime_add_mine[2]) :
    rime_connection (block, channel, inport, outport, rime_add_mine)
{

}

std::array<uint8_t, 256>
uc_connection::make_msgbuf (uint16_t channel, const uint8_t src[2],
                            const uint8_t dest[2],
                            const uint8_t next_hop_mac[2])
{
  std::array<uint8_t, 256> buf;

  buf[0] = next_hop_mac[0];
  buf[1] = next_hop_mac[1];
  buf[2] = channel & 0xff;
  buf[3] = (channel >> 8) & 0xff;
  buf[4] = dest[0];
  buf[5] = dest[1];
  buf[6] = src[0];
  buf[7] = src[1];

  return buf;
}

bool
uc_connection::rime_add_from_string (std::string &to_parse, uint8_t addr[2])
{
  unsigned long rime_zero_long = std::strtoul (to_parse.data (), nullptr, 10);
  size_t index = to_parse.find (".");
  to_parse.erase (0, index + 1);
  unsigned long rime_one_long = std::strtoul (to_parse.data (), nullptr, 10);
  index = to_parse.find_first_not_of ("0123456789");
  if (to_parse.at (index) == ' ') {
    to_parse.erase (0, index + 1);
  }
  else {
    to_parse.erase (0, index);
  }
  if (rime_zero_long > 255 || rime_one_long > 255
      || (rime_zero_long == 0 && rime_one_long == 0)) {
    return false;
  }
  addr[0] = static_cast<uint8_t> (rime_zero_long);
  addr[1] = static_cast<uint8_t> (rime_one_long);
  return true;
}

void
uc_connection::pack (pmt::pmt_t msg)
{
  if (pmt::is_eof_object (msg)) {
    d_block->message_port_pub (d_mac_outport, pmt::PMT_EOF);
    d_block->detail ().get ()->set_done (true);
    return;
  }

  std::string tmp = rime_connection::msg_to_string (msg);

  uint8_t dest[2];
  if (!uc_connection::rime_add_from_string (tmp, dest)) {
    std::cerr
        << "#RIME: Warning: invalid target RIME-Address for unicast on channel ";
    std::cerr << static_cast<unsigned> (d_channel);
    std::cerr << ". Message will not be sent." << std::endl;
    return;
  }

  /* Routing */
  uint8_t next_hop_mac_addr[2];
  if (get_next_hop_mac_addr (d_rime_add_mine, dest, next_hop_mac_addr) == -1) {
    printf (
        "#RIME: can't find next hop mac address (src: %d.%d, dest: %d.%d)\n",
        d_rime_add_mine[0], d_rime_add_mine[1], dest[0], dest[1]);
    cout << "Dropping packet!" << endl;
    return;
  }
  else {
    printf (
        "#RIME: found next hop mac address (src: %d.%d, dest: %d.%d, next_hop_mac: %d.%d)",
        d_rime_add_mine[0], d_rime_add_mine[1], dest[0], dest[1],
        next_hop_mac_addr[0], next_hop_mac_addr[1]);
  }

  std::array<uint8_t, 256> buf = uc_connection::make_msgbuf (d_channel,
                                                             d_rime_add_mine,
                                                             dest,
                                                             next_hop_mac_addr);

  size_t data_len = tmp.length ();
  assert(data_len);
  assert(data_len < 256 - header_length - 2);

  std::memcpy (buf.data () + header_length + 2, tmp.data (), data_len);
  pmt::pmt_t rime_msg = pmt::make_blob (buf.data (),
                                        data_len + header_length + 2);

  d_block->message_port_pub (d_mac_outport, pmt::cons (pmt::PMT_NIL, rime_msg));
}

void
uc_connection::unpack (pmt::pmt_t msg)
{
  unsigned char buf[256];
  size_t data_len = pmt::blob_length (msg);
  std::memcpy (buf, pmt::blob_data (msg), data_len);

  //this block is not the destination of the message
  if (buf[2] != d_rime_add_mine[0] || buf[3] != d_rime_add_mine[1]) {
    std::cout << "#RIME: wrong rime add " << int (buf[2]) << "." << int (buf[3])
        << std::endl;

    /* Forwarding */
    cout << "#RIME: Forwarding..." << endl;

    /* Routing */
    uint8_t next_hop_mac_addr[2];
    uint8_t* dest = &buf[2];
    uint8_t* src = &buf[4];
    uint8_t* channel = &buf[0];
    if (get_next_hop_mac_addr (src, dest, next_hop_mac_addr) == -1) {
      printf (
          "#RIME: can't find next hop mac address (src: %d.%d, dest: %d.%d)\n",
          src[0], src[1], dest[0], dest[1]);
      cout << "Dropping packet!" << endl;
      return;
    }
    else {
      printf (
          "#RIME: found next hop mac address (src: %d.%d, dest: %d.%d, next_hop_mac: %d.%d)",
          src[0], src[1], dest[0], dest[1],
          next_hop_mac_addr[0], next_hop_mac_addr[1]);
    }

//
//    std::array < uint8_t, 256 > buf = uc_connection::make_msgbuf (
//        (channel[0] << 8) | channel[1], src, dest, next_hop_mac_addr);
//
//    size_t data_len = tmp.length ();
//    assert(data_len);
//    assert(data_len < 256 - header_length - 2);
//
//    std::memcpy (buf.data () + header_length + 2, tmp.data (), data_len);
//    pmt::pmt_t rime_msg = pmt::make_blob (buf.data (),
//                                          data_len + header_length + 2);
//
//    d_block->message_port_pub (d_mac_outport,
//                               pmt::cons (pmt::PMT_NIL, rime_msg));
//
    return;
  }

  pmt::pmt_t rime_payload = pmt::make_blob (buf + header_length,
                                            data_len - header_length);
  d_block->message_port_pub (d_outport, pmt::cons (pmt::PMT_NIL, rime_payload));
}

/*----------------------- Routing table --------------------------------------*/
const int routing_table_max_rows = 16;
const uint8_t sur1_routing_table[routing_table_max_rows][4] = { // dest RIME address, next hop MAC address.
    { 12, 34, 1, 0 }, };

const uint8_t sur2_routing_table[routing_table_max_rows][4] = { // dest RIME address, next hop MAC address.
    { 12, 34, 2, 0 }, { 12, 35, 2, 0 }, };

int
uc_connection::get_next_hop_mac_addr (const uint8_t* rime_src,
                                      const uint8_t* rime_dest,
                                      uint8_t* next_hop_mac)
{
  uint8_t** routing_table;
  int count;

  if ((rime_src[0] == 12) && (rime_src[1] == 35)) {
    routing_table = sur1_routing_table;
  }
  else if ((rime_src[0] == 12) && (rime_src[1] == 36)) {
    routing_table = sur2_routing_table;
  }
  else {
    return -1;
  }

  for (count = 0; count < routing_table_max_rows; count++) {
    if ((rime_dest[0] == routing_table[count][0])
        && (rime_dest[1] == routing_table[count][1])) {
      /* Found RIME dest address in routing table */
      next_hop_mac[0] = routing_table[count][2];
      next_hop_mac[1] = routing_table[count][3];

      return 0;
    }
  }

  return -1;
}
