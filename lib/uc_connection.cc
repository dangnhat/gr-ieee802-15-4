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

#define dout d_debug && cout

uc_connection::uc_connection (rime_stack *block, uint16_t channel,
                              pmt::pmt_t inport, pmt::pmt_t outport,
                              const uint8_t rime_add_mine[2]) :
    rime_connection (block, channel, inport, outport, rime_add_mine)
{

}

array<uint8_t, 256>
uc_connection::make_msgbuf (uint16_t channel, const uint8_t src[2],
                            const uint8_t dest[2],
                            const uint8_t next_hop_mac[2])
{
  array<uint8_t, 256> buf;

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
uc_connection::rime_add_from_string (string &to_parse, uint8_t addr[2])
{
  unsigned long rime_zero_long = strtoul (to_parse.data (), nullptr, 10);
  size_t index = to_parse.find (".");
  to_parse.erase (0, index + 1);
  unsigned long rime_one_long = strtoul (to_parse.data (), nullptr, 10);
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

  string tmp = rime_connection::msg_to_string (msg);

  uint8_t dest[2];
  if (!uc_connection::rime_add_from_string (tmp, dest)) {
    dout << "#RIME: invalid target RIME-Address" << endl;
    return;
  }

  /* Routing */
  uint8_t next_hop_mac_addr[2];
  if (get_next_hop_mac_addr (d_rime_add_mine, dest, next_hop_mac_addr) == -1) {
    dout << "#RIME: Can't find next hop for: " << int (dest[0]) << "."
        << int (dest[1]) << endl;
    return;
  }

  array<uint8_t, 256> buf = uc_connection::make_msgbuf (d_channel,
                                                        d_rime_add_mine, dest,
                                                        next_hop_mac_addr);

  size_t data_len = tmp.length ();
  assert(data_len);
  assert(data_len < 256 - header_length - 2);

  memcpy (buf.data () + header_length + 2, tmp.data (), data_len);
  pmt::pmt_t rime_msg = pmt::make_blob (buf.data (),
                                        data_len + header_length + 2);

  d_block->message_port_pub (d_mac_outport, pmt::cons (pmt::PMT_NIL, rime_msg));
}

void
uc_connection::unpack (pmt::pmt_t msg)
{
  unsigned char buf[256];
  memset(buf, 0, 256);
  size_t data_len = pmt::blob_length (msg);
  memcpy (buf + 2, pmt::blob_data (msg), data_len);

  uint8_t* next_hop_mac_addr = &buf[0];
  uint8_t* channel = &buf[2];
  uint8_t* dest = &buf[4];
  uint8_t* src = &buf[6];

  //this block is not the destination of the message
  if (dest[0] != d_rime_add_mine[0] || dest[1] != d_rime_add_mine[1]) {
    dout << "#RIME: wrong rime add " << int (dest[0]) << "." << int (dest[1])
        << endl;

    /* Forwarding */
    /* Routing */
    if (get_next_hop_mac_addr (src, dest, next_hop_mac_addr) == -1) {
      dout << "#RIME: Can't find next hop for: " << int (dest[0]) << "."
          << int (dest[1]) << endl;
      return;
    }
    dout << "#RIME: Forward -> next_hop_mac: " << int (next_hop_mac_addr[0])
        << "." << int (next_hop_mac_addr[1]) << endl;

    /* Send packet to MAC layer */
    pmt::pmt_t to_mac_msg = pmt::make_blob (buf, data_len + 2);

    d_block->message_port_pub (d_mac_outport,
                               pmt::cons (pmt::PMT_NIL, to_mac_msg));
    return;
  }

  cout << "#RIME: Forward to application: " << buf + 2 + header_length << endl;
  pmt::pmt_t rime_payload = pmt::make_blob (buf + 2 + header_length,
                                            data_len - header_length);
  d_block->message_port_pub (d_outport, pmt::cons (pmt::PMT_NIL, rime_payload));
}

/*----------------------- Routing table --------------------------------------*/
const int routing_table_max_rows = 16;
const int routing_table_max_cols = 4;
typedef uint8_t routing_table_t[routing_table_max_rows][routing_table_max_cols];

const routing_table_t suc_routing_table = { // dest RIME address, next hop MAC address.
    { 12, 35, 1, 0 }, { 12, 36, 1, 0 }, { 12, 37, 1, 0 }, { 12, 38, 1, 0 },};

const routing_table_t sur1_routing_table = { // dest RIME address, next hop MAC address.
    { 12, 34, 0, 0 },
    { 12, 36, 2, 0 }, { 12, 37, 2, 0 }, { 12, 38, 2, 0 }, };

const routing_table_t sur2_routing_table = { // dest RIME address, next hop MAC address.
    { 12, 34, 1, 0 }, { 12, 35, 1, 0 },
    { 12, 37, 3, 0 }, { 12, 38, 3, 0 }, };

const routing_table_t sur3_routing_table = { // dest RIME address, next hop MAC address.
    { 12, 34, 2, 0 }, { 12, 35, 2, 0 }, { 12, 36, 2, 0 },
    { 12, 38, 3, 1 }, };

int
uc_connection::get_next_hop_mac_addr (const uint8_t* rime_src,
                                      const uint8_t* rime_dest,
                                      uint8_t* next_hop_mac)
{
  const routing_table_t* routing_table;
  int count;

  if ((d_rime_add_mine[0] == 12) && (d_rime_add_mine[1] == 34)) {
    routing_table = &suc_routing_table;
  }
  else if ((d_rime_add_mine[0] == 12) && (d_rime_add_mine[1] == 35)) {
    routing_table = &sur1_routing_table;
  }
  else if ((d_rime_add_mine[0] == 12) && (d_rime_add_mine[1] == 36)) {
    routing_table = &sur2_routing_table;
  }
  else if ((d_rime_add_mine[0] == 12) && (d_rime_add_mine[1] == 37)) {
    routing_table = &sur3_routing_table;
  }
  else if ((d_rime_add_mine[0] == 12) && (d_rime_add_mine[1] == 38)) {
    /* TODO: hard coded default route for SU1 */
    next_hop_mac[0] = 3;
    next_hop_mac[1] = 0;

    return 0;
  }
  else if ((d_rime_add_mine[0] == 52) && (d_rime_add_mine[1] == 35)) {
    /* TODO: hard coded default route for SU1 (perf test) */
    next_hop_mac[0] = 5;
    next_hop_mac[1] = 0;

    return 0;
  }
  else {
    return -1;
  }

  for (count = 0; count < routing_table_max_rows; count++) {
    if ((rime_dest[0] == (*routing_table)[count][0])
        && (rime_dest[1] == (*routing_table)[count][1])) {
      /* Found RIME dest address in routing table */
      next_hop_mac[0] = (*routing_table)[count][2];
      next_hop_mac[1] = (*routing_table)[count][3];

      return 0;
    }
  }

  return -1;
}
