//============================================================================
// Name        : ngrc_suc_app.cpp
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
#include <signal.h>
#include <stdlib.h>

using namespace std;

/* SUC UDP server info */
string addr = "127.0.0.1";
int suc_udp_server_port = 52002, send_udp_port = 52001;
const uint32_t buffer_len = 256;
char buffer[buffer_len];

int suc_socket_fd, send_socket_fd;

void
intHandler (int dummy)
{
  close (suc_socket_fd);
  close (send_socket_fd);
  exit (0);
}

int
main ()
{
  unsigned int client_addr_len;
  struct sockaddr_in my_addr, client_addr;
  struct sockaddr_in send_addr;
  int received_buf_len;

  cout << "This is MSHCS round trip delay test - Receiver side." << endl;

  /* Added signal handle for Ctrl-C */
  signal (SIGINT, intHandler);

  /* Create SUC UDP server socket */
  if ((suc_socket_fd = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    cout << "cannot create socket" << endl;
    return 0;
  }

  memset ((char *) &my_addr, 0, sizeof(my_addr));
  my_addr.sin_family = AF_INET;
  my_addr.sin_addr.s_addr = htonl (INADDR_ANY);
  my_addr.sin_port = htons (suc_udp_server_port);

  if (bind (suc_socket_fd, (struct sockaddr *) &my_addr, sizeof(my_addr)) < 0) {
    cout << "bind failed" << endl;
    return 0;
  }

  /* Wait for message from SUs and forward them to control center */
  while (1) {
    /* Waiting for messages */
    cout << "Waiting for message..." << endl;
    received_buf_len = recvfrom (suc_socket_fd, (void *) buffer, buffer_len, 0,
                                 (sockaddr *) &client_addr, &client_addr_len);
    if (received_buf_len == -1) {
      cout << "recvfrom error." << endl;
      continue;
    }

    buffer[received_buf_len] = '\0';
    cout << "\nReceived: " << buffer << endl;

    /* Echo back to client */
    if ((send_socket_fd = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
      cout << "cannot create socket" << endl;
      return 0;
    }

    memset ((char *) &send_addr, 0, sizeof(send_addr));
    send_addr.sin_family = AF_INET;
    inet_aton (addr.c_str (), &send_addr.sin_addr);
    send_addr.sin_port = htons (send_udp_port);

    sendto (send_socket_fd, (void *) buffer, received_buf_len, 0,
            (sockaddr *) &send_addr, sizeof(send_addr));
    close (send_socket_fd);
  }

  return 0;
}

