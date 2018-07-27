Daejeon, South Korea, July 28, 2018.
Real-time and Embedded System Lab (http://www.resl.kaist.ac.kr/), Korea Advanced Institute of Science and Technology
Nhat Pham (nhatphd@kaist.ac.kr)

Hi!

This is the implementation of "MSHCS-MAC: A MAC protocol for Multi-hop cognitive radio networks based on Slow Hopping and Cooperative Sensing approach".
It also include time synchronization based on a modified version of Reference Broadcast Time Synchronization with active loopback and linear regression.
Our implementation is based on Bastibl's gr-ieee802-15-4.

## Install script
```
cd examples
./GalaxyS8+_linux_deploy.sh
```
The script will install needed libraries and packages, and compile the MAC protocol.
After that, compile the gnuradio flowgraph (*.grc) and run it.

Have fun!
