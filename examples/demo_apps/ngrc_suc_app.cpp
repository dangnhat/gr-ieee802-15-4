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

/* Control center UDP server info */
string control_center_addr_s = "192.168.5.2";
int control_center_port = 40000;

/* SUC UDP server info */
int suc_udp_server_port = 52001;
const uint32_t buffer_len = 256;
char buffer[buffer_len];

int suc_socket_fd, control_center_sock_fd;

void
intHandler (int dummy)
{
  close (suc_socket_fd);
  close (control_center_sock_fd);
  exit (0);
}

int
main ()
{
  unsigned int client_addr_len;
  struct sockaddr_in myaddr, client_addr;
  struct sockaddr_in control_center_addr;
  int received_buf_len;

  cout << "This is NGRC project SUC demo application" << endl;

  /* Added signal handle for Ctrl-C */
  signal (SIGINT, intHandler);

  /* Create SUC UDP server socket */
  if ((suc_socket_fd = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    cout << "cannot create socket" << endl;
    return 0;
  }

  memset ((char *) &myaddr, 0, sizeof(myaddr));
  myaddr.sin_family = AF_INET;
  myaddr.sin_addr.s_addr = htonl (INADDR_ANY);
  myaddr.sin_port = htons (suc_udp_server_port);

  if (bind (suc_socket_fd, (struct sockaddr *) &myaddr, sizeof(myaddr)) < 0) {
    cout << "bind failed" << endl;
    return 0;
  }

  /* Create to control center UDP socket */
  if ((control_center_sock_fd = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    cout << "cannot create socket" << endl;
    return 0;
  }

  memset ((char *) &myaddr, 0, sizeof(myaddr));
  myaddr.sin_family = AF_INET;
  myaddr.sin_addr.s_addr = htonl (INADDR_ANY);
  myaddr.sin_port = htons (0);

  if (bind (control_center_sock_fd, (struct sockaddr *) &myaddr, sizeof(myaddr)) < 0) {
    cout << "bind failed" << endl;
    return 0;
  }

  /* Wait for message from SUs and forward them to control center */
  while (1) {
    /* Waiting for messages */
	received_buf_len = recvfrom (suc_socket_fd, (void *) buffer, buffer_len, 0,
            (sockaddr *) &client_addr, &client_addr_len);
    if (received_buf_len == -1) {
      cout << "recvfrom error." << endl;
      continue;
    }

    cout << "Received: " << buffer << endl;

    /* Forward to control center server */
    control_center_addr.sin_family = AF_INET;
    inet_aton (control_center_addr_s.c_str (), &control_center_addr.sin_addr);
    control_center_addr.sin_port = htons (control_center_port);

    cout << "Forwarding to control center" << endl;
    sendto (control_center_sock_fd, buffer, received_buf_len, 0,
            (struct sockaddr *) &control_center_addr, sizeof(control_center_addr));
  }

  return 0;
}

