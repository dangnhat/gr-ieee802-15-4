# Deploy Ubuntu with > 4GB image file, grapic: VNC, 800x400, 100 dpi, pass: resl.

# Install git and clone source code:
# https://github.com/dangnhat/gr-foo
# https://github.com/dangnhat/gr-ieee802-15-4

# Install dependencies:
DEPS="git gnuradio gnuradio-dev swig pkgconf usbutils libuhd* libusb-dev cmake autotools-dev autoconf libboost-all-dev"
echo
echo
echo "Installing $DEPS"
sudo apt-get install $DEPS

# Setup udev rules for USRPs
UHD_UDEV_LOC=/usr/lib/uhd/utils

echo
echo
echo "Setup udev rules for USRPs..."
sudo cp $UHD_UDEV_LOC/uhd-usrp.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo /usr/lib/uhd/utils/uhd_images_downloader.py

# Build gr-foo and gr-ieee802-15-4
NTHREADS=1

echo
echo
echo "Cloning gr-foo, gr-ieee802-15-4, and restclient-cpp"
cd ~
git clone https://github.com/dangnhat/gr-foo
git clone https://github.com/dangnhat/gr-ieee802-15-4

echo
echo
echo "Building gr-foo with $NTHREADS threads..."
cd gr-foo
mkdir build
cd build
cmake ../
make
sudo make install
sudo ldconfig

cd ~
echo
echo
echo "Building gr-ieee802-15-4 with $NTHREADS threads..."
cd gr-ieee802-15-4
git checkout shcs_mac_multi_hops
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make
sudo make install
sudo ldconfig
cd ../examples
grcc ieee802_15_4_OQPSK_PHY.grc

# Run gnuradio companion to generate .py files. or use grcc:
# Usage: grcc: [options] filename
