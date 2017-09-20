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

/* UDP server info */
string control_center_addr = "127.0.0.1";
int control_center_port = 52001;
const uint32_t buffer_len = 256;
char buffer[buffer_len];

int socket_fd;

void intHandler(int dummy) {
    close(socket_fd);
    exit(0);
}

int
main ()
{
  unsigned int client_addr_len;
  struct sockaddr_in myaddr;
  struct sockaddr_in client_addr;
  string su_status;

  cout << "This is NGRC project SUC demo application" << endl;

  /* Added signal handle for Ctrl-C */
  signal(SIGINT, intHandler);

  /* Create a socket */

  if ((socket_fd = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    cout << "cannot create socket" << endl;
    return 0;
  }

  memset ((char *) &myaddr, 0, sizeof(myaddr));
  myaddr.sin_family = AF_INET;
  myaddr.sin_addr.s_addr = htonl (INADDR_ANY);
  myaddr.sin_port = htons (control_center_port);

  if (bind (socket_fd, (struct sockaddr *) &myaddr, sizeof(myaddr)) < 0) {
    cout << "bind failed" << endl;
    return 0;
  }

  while (1) {
    /* Waiting for messages */
    if (recvfrom (socket_fd, (void *) buffer, buffer_len, 0,
                  (sockaddr *) &client_addr, &client_addr_len) == -1) {
      cout << "recvfrom error." << endl;
      close (socket_fd);
      return 0;
    }

    cout << "Received: " << buffer << endl;
  }

  return 0;
}

