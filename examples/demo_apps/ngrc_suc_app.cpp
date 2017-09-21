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
#include "restclient-cpp/restclient.h"

using namespace std;

/* UDP server info */
string control_center_addr = "143.248.194.81:38888";
int control_center_port = 52001;
const uint32_t buffer_len = 256;
char buffer[buffer_len];

int socket_fd;

void
intHandler (int dummy)
{
  close (socket_fd);
  exit (0);
}

int
main ()
{
  unsigned int client_addr_len;
  struct sockaddr_in myaddr;
  struct sockaddr_in client_addr;
  string http_string, mid, sid, sname, rank, position, wjam, gjam, gstate;
  size_t pos1, pos2;
  int count;

  cout << "This is NGRC project SUC demo application" << endl;

  /* Added signal handle for Ctrl-C */
  signal (SIGINT, intHandler);

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
      continue;
    }

    cout << "Received: " << buffer << endl;
    /* Parse data from received string */
    string received_status (buffer);
    pos1 = received_status.find_first_of ("&");
    count = 0;
    while (pos1 != string::npos) {
      pos2 = received_status.find_first_of ("&", pos1 + 1);
      if (pos2 == string::npos) {
        break;
      }

      switch (count) {
        case 0:
          mid = received_status.substr (pos1 + 1, pos2 - pos1 - 1);
          break;
        case 1:
          sid = received_status.substr (pos1 + 1, pos2 - pos1 - 1);
          break;
        case 2:
          sname = received_status.substr (pos1 + 1, pos2 - pos1 - 1);
          break;
        case 3:
          rank = received_status.substr (pos1 + 1, pos2 - pos1 - 1);
          break;
        case 4:
          position = received_status.substr (pos1 + 1, pos2 - pos1 - 1);
          break;
        case 5:
          wjam = received_status.substr (pos1 + 1, pos2 - pos1 - 1);
          break;
        case 6:
          gjam = received_status.substr (pos1 + 1, pos2 - pos1 - 1);
          break;
        case 7:
          gstate = received_status.substr (pos1 + 1, pos2 - pos1 - 1);
          break;
        default:
          break;
      }

      count++;
      pos1 = pos2;
    }/* end while */

    http_string = "http://" + control_center_addr + "/node-update.php?mid="
        + mid + "&sid=" + sid + "&sname=" + sname + "&rank=" + rank
        + "&position=" + position + "&wjam=" + wjam + "&gjam=" + gjam
        + "&gstate=" + gstate;
    cout << "HTTP GET: " << http_string << endl;

    RestClient::Response r = RestClient::get (http_string);
    cout << "Respone code: " << r.code << endl;
  }

  return 0;
}

