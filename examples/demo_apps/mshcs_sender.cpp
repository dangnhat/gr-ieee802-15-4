//============================================================================
// Name        : ngrc_su_app.cpp
// Author      : Nhat Pham
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <string>
#include <sys/socket.h>
#include <cstdio>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <sstream>
#include <boost/chrono.hpp>
#include <signal.h>

using namespace std;
using namespace boost::chrono;

/* UDP server info */
string addr = "127.0.0.1";
int send_port = 52001, recv_port = 52002;
int send_socket_fd, recv_socket_fd;

const string recv_addr_s = "12.34";
const string send_addr_s = "12.35";
const int wait_time_range[2] = { 2, 10 };
//const string message = "Hello World.\n";

void
intHandler (int dummy)
{
  close (recv_socket_fd);
  close (send_socket_fd);
  exit (0);
}

int
main ()
{
  struct sockaddr_in myaddr;
  struct sockaddr_in dest_addr, recv_addr;
  unsigned int recv_addr_len;
  const uint32_t ack_buffer_len = 256;
  char ack_buffer[ack_buffer_len];
  int ack_recv_buffer_len;

  ostringstream message;
  uint64_t seqno = 0, recv_seqno = 0;
  uint64_t numAckedPackets = 0;
  system_clock::time_point send_time, ack_time;
  int rand_wait_time;

  cout << "This is MSHCS round trip delay test - sender side." << endl;

  /* Added signal handle for Ctrl-C */
  signal (SIGINT, intHandler);

  /* Create receiving socket */
  if ((recv_socket_fd = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    cout << "cannot create recv_socket" << endl;
    return 0;
  }

  memset ((char *) &myaddr, 0, sizeof(myaddr));
  myaddr.sin_family = AF_INET;
  myaddr.sin_addr.s_addr = htonl (INADDR_ANY);
  myaddr.sin_port = htons (recv_port);

  if (bind (recv_socket_fd, (struct sockaddr *) &myaddr, sizeof(myaddr)) < 0) {
    cout << "bind failed" << endl;
    return 0;
  }

  /* Set timeout for socket 20s */
  struct timeval tv;
  tv.tv_sec = 20;
  tv.tv_usec = 0;
  if (setsockopt (recv_socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv))
      < 0) {
    cout << "Set timeout error" << endl;
  }

  while (1) {
    /* Generate su_status message */
    message.clear ();
    message.str ("");
    message << recv_addr_s << " " << send_addr_s << " " << seqno << endl;

    /* Create a socket */
    if ((send_socket_fd = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
      cout << "cannot create socket" << endl;
      return 0;
    }

    /* periodically send status to Coordinator */
    dest_addr.sin_family = AF_INET;
    inet_aton (addr.c_str (), &dest_addr.sin_addr);
    dest_addr.sin_port = htons (send_port);

    /* Wait to receive ACK for a random time */
    rand_wait_time = rand () % (wait_time_range[1] - wait_time_range[0] + 1)
        + wait_time_range[0];
    cout << "Random wait time: " << rand_wait_time << endl;
    sleep (rand_wait_time);

    cout << "Sending: " << message.str () << endl;
    sendto (send_socket_fd, message.str ().c_str (), message.str ().length (),
            0, (struct sockaddr *) &dest_addr, sizeof(dest_addr));
    close (send_socket_fd);

    /* Get current time */
    send_time = system_clock::now ();

    /* Wait for ack */
    while (1) {
      ack_recv_buffer_len = recvfrom (recv_socket_fd, ack_buffer,
                                      ack_buffer_len, 0,
                                      (struct sockaddr *) &recv_addr,
                                      &recv_addr_len);
      if (ack_recv_buffer_len < 0) {
        cout << "Recvfrom error or timeout" << endl;

        seqno++;
        break;
      }
      else {
        /* Get current time */
        ack_time = system_clock::now ();

        ack_buffer[ack_recv_buffer_len] = '\0';
        cout << "Received: " << ack_buffer << endl;

        string s (ack_buffer);
        stringstream ack_string (s);
        ack_string >> recv_seqno;
        if (recv_seqno == seqno) {
          numAckedPackets++;
          cout << "ACKed for #" << seqno << ", acked: " << numAckedPackets
              << ", RTT: " << duration_cast<milliseconds> (ack_time - send_time)
              << endl;

          seqno++;
          break;
        }
      }
    }

  }

  return 0;
}

