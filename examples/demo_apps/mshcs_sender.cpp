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

/*
 Simple udp client
 */

#define SERVER "127.0.0.1"
#define BUFLEN 512  //Max length of buffer
#define PORT 52001   //The port on which to send data

const int wait_time_range[2] = { 2, 10 };

int socket_fd;

void
intHandler (int dummy)
{
  close (socket_fd);
  exit (0);
}

void
die (const char *s)
{
  cout << s << endl;
  close (socket_fd);
  exit (1);
}

int
main (void)
{
  struct sockaddr_in si_other;
  int i;
  unsigned int slen = sizeof(si_other);
  char ack_buf[BUFLEN];

  ostringstream message;
  uint64_t seqno = 0, recv_seqno = 0;
  uint64_t num_acked = 0;
  system_clock::time_point send_time, ack_time;
  int rand_wait_time;

  string recv_addr_s, send_addr_s;

  cout << "This is MSHCS round trip delay test - sender side." << endl;
  cout << "Enter sender address:" << endl;
  cin >> send_addr_s;
  cout << "Enter receiver address:" << endl;
  cin >> recv_addr_s;

  /* Added signal handle for Ctrl-C */
  signal (SIGINT, intHandler);

  /* Create socket */
  if ((socket_fd = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    die ("socket");
  }

  /* Set timeout for socket 20s */
  struct timeval tv;
  tv.tv_sec = 20;
  tv.tv_usec = 0;
  if (setsockopt (socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    die ("setsockopt()");
  }

  /* Set address to server */
  memset ((char *) &si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons (PORT);

  if (inet_aton (SERVER, &si_other.sin_addr) == 0) {
    fprintf (stderr, "inet_aton() failed\n");
    exit (1);
  }

  while (1) {
    /* Generate message */
    message.clear ();
    message.str ("");
    message << recv_addr_s << " " << send_addr_s << " " << seqno << endl;

    /* Wait for a random time */
    rand_wait_time = rand () % (wait_time_range[1] - wait_time_range[0] + 1)
        + wait_time_range[0];
    cout << "Random wait time: " << rand_wait_time << endl;
    sleep (rand_wait_time);

    cout << "Sending: " << message.str () << endl;

    //send the message
    if (sendto (socket_fd, message.str ().c_str (), message.str ().length (), 0,
                (struct sockaddr *) &si_other, slen) == -1) {
      die ("sendto()");
    }

    /* Get send time */
    send_time = system_clock::now ();

    /* Wait for ack */
    while (1) {
      //receive a reply and print it
      //clear the buffer by filling null, it might have previously received data
      memset (ack_buf, '\0', BUFLEN);
      //try to receive some data, this is a blocking call
      if (recvfrom (socket_fd, ack_buf, BUFLEN, 0,
                    (struct sockaddr *) &si_other, &slen) == -1) {
        cout << "Recvfrom error or timeout" << endl;

        seqno++;
        break;
      }
      else {
        /* Get current time */
        ack_time = system_clock::now ();

        cout << "Received: " << ack_buf << endl;

        string ack_s (ack_buf);
        stringstream ack_ss (ack_s);
        ack_ss >> recv_seqno;
        if (recv_seqno == seqno) {
          num_acked++;
          cout << "ACKed for #" << seqno << ", num_acked: " << num_acked
              << ", RTT: " << duration_cast<milliseconds> (ack_time - send_time)
              << endl;

          seqno++;
          break;
        }
      }
    }
  }

  close (socket_fd);
  return 0;
}
