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

using namespace std;

/* UDP server info */
string addr = "127.0.0.1";
int port = 52002;

/* Soldier info */
//string su1_status =
//    "mid=K05%2E001&sid=12345&sname=Kim&rank=sergeant&position=36.368487%127.359918&wjam=1&gjam=0&gstate=100%3A130%3A150";
//string su2_status =
//    "mid=K05%2E002&sid=11234&sname=Hae&rank=officer&position=36.371955%127.365539&wjam=1&gjam=1&gstate=100%3A130%3A150";
//string su3_status =
//    "mid=K05%2E003&sid=00121&sname=Park&rank=private&position=36.374028%127.365228&wjam=0&gjam=0&gstate=100%3A130%3A150";
string su1_status =
    "usrp&1111111113&59:90&5&0&100:130:150&";
string su2_status =
    "usrp&1111111117&59:71&5&0&100:130:150&";
string su3_status =
    "usrp&1301381719&50:55&5&0&100:130:150&";
string su4_status =
    "usrp&1302102032&70:50&5&0&100:130:150&";


int
main ()
{
  int socket_fd, su_id;
  struct sockaddr_in myaddr;
  struct sockaddr_in dest_addr;
  string su_status;

  cout << "This is NGRC project SU demo application" << endl;
  cout << "What's SU ID?" << endl;
  cin >> su_id;
  switch (su_id) {
    case 1:
      su_status = su1_status;
      break;
    case 2:
      su_status = su2_status;
      break;
    case 3:
      su_status = su3_status;
      break;
    case 4:
      su_status = su4_status;
      break;
  }
  cout << "SU status string" << endl;
  cout << su_status << endl;

  while (1) {
    /* Create a socket */

    if ((socket_fd = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
      cout << "cannot create socket" << endl;
      return 0;
    }

    memset ((char *) &myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = htonl (INADDR_ANY);
    myaddr.sin_port = htons (0);

    if (bind (socket_fd, (struct sockaddr *) &myaddr, sizeof(myaddr)) < 0) {
      cout << "bind failed" << endl;
      return 0;
    }

    /* periodically send status to Coordinator */
    dest_addr.sin_family = AF_INET;
    inet_aton (addr.c_str (), &dest_addr.sin_addr);
    dest_addr.sin_port = htons (port);

    cout << "Sending status to Coor" << endl;
    sendto (socket_fd, su_status.c_str (), su_status.length (), 0,
            (struct sockaddr *) &dest_addr, sizeof(dest_addr));
    close (socket_fd);

    sleep (1);
  }

  return 0;
}

