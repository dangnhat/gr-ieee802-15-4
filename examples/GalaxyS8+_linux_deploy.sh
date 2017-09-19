# Deploy Ubuntu with > 4GB image file, grapic: VNC, 800x400, 100 dpi, pass: resl.

# Install git and clone source code:
# https://github.com/dangnhat/gr-foo
# https://github.com/dangnhat/gr-ieee802-15-4

# Install dependencies:
DEPS="gnuradio swig pkgconf usbutils libuhd* libusb-dev"
echo "Install $DEP"
sudo apt-get install git gnuradio swig pkgconf usbutils libuhd* libusb-dev cmake

# Setup udev rules for USRPs
UHD_UDEV_LOC=/usr/lib/uhd/utils

echo "Setup udev rules for USRPs..."
sudo cp $UHD_UDEV_LOC/uhd-usrp.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Build gr-foo and gr-ieee802-15-4
NTHREADS=8

cd ~
git clone https://github.com/dangnhat/gr-foo
git clone https://github.com/dangnhat/gr-ieee802-15-4

echo "Building gr-foo with $NTHREADS threads..."
cd gr-foo
mkdir build
cd build
cmake ../
make -j$NTHREAD
sudo make install
sudo ldconfig

cd ~
echo "Building gr-ieee802-15-4 with $NTHREADS threads..."
cd gr-ieee802-15-4
mkdir build
cd build
cmake ../
make -j$NTHREAD
sudo make install
sudo ldconfig

# Run gnuradio companion to generate .py files. or use grcc:
# Usage: grcc: [options] filename
