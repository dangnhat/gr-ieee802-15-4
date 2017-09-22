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

typedef struct soldier_info {
    int sid;
    int gps_lat, gps_long;
    int wjam, gjam;
    int gstate1, gstate2, gstate3;
} soldier_infor_t;

const int mov_range = 5;
const string status_header = "usrp";

int main ()
{
  int socket_fd, su_id;
  struct sockaddr_in myaddr;
  struct sockaddr_in dest_addr;
  ostringstream su_status;
  int mov_lat, mov_long;

  /* Soldiers info */
  soldier_infor_t soldiers[4];

  soldiers[0].sid = 1111111113;
  soldiers[0].gps_lat = 59;
  soldiers[0].gps_long = 80;
  soldiers[0].wjam = 5;
  soldiers[0].gjam = 0;
  soldiers[0].gstate1 = 100;
  soldiers[0].gstate2 = 130;
  soldiers[0].gstate3 = 150;

  soldiers[1].sid = 1111111117;
  soldiers[1].gps_lat = 59;
  soldiers[1].gps_long = 71;
  soldiers[1].wjam = 5;
  soldiers[1].gjam = 0;
  soldiers[1].gstate1 = 100;
  soldiers[1].gstate2 = 130;
  soldiers[1].gstate3 = 150;

  soldiers[2].sid = 1301381719;
  soldiers[2].gps_lat = 50;
  soldiers[2].gps_long = 55;
  soldiers[2].wjam = 5;
  soldiers[2].gjam = 0;
  soldiers[2].gstate1 = 100;
  soldiers[2].gstate2 = 130;
  soldiers[2].gstate3 = 150;

  cout << "This is NGRC project SU demo application" << endl;
  cout << "What's SU ID?" << endl;
  cin >> su_id;

  while (1) {
    /* Random movement */
    mov_lat = soldiers[su_id-1].gps_lat + rand()%(2*mov_range) - mov_range;
    mov_long = soldiers[su_id-1].gps_long + rand()%(2*mov_range) - mov_range;

    /* Generate su_status message */
    /* i.e. "usrp&1111111113&59:90&5&0&100:130:150&" */
    su_status.clear();
    su_status.str("");
    su_status << status_header << "&" << soldiers[su_id-1].sid << "&" << mov_lat
            << ":" << mov_long << "&"
            << soldiers[su_id-1].wjam << "&" << soldiers[su_id-1].gjam << "&"
            << soldiers[su_id-1].gstate1 << ":" << soldiers[su_id-1].gstate2 << ":"
            << soldiers[su_id-1].gstate3 << "&";
    cout << "SU status: " << su_status.str() << endl;

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
    sendto (socket_fd, su_status.str().c_str (), su_status.str().length (), 0,
            (struct sockaddr *) &dest_addr, sizeof(dest_addr));
    close (socket_fd);

    sleep (1);
  }

  return 0;
}

