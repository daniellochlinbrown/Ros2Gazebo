#!/usr/bin/env bash
set -e

echo
echo "Installing Micro XRCE-DDS Agent ..."

# Clone Micro XRCE-DDS Agent Repository and build
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
# Install Micro XRCE-DDS Agent
sudo make install
sudo ldconfig /usr/local/lib/

echo
echo "[DONE] Micro XRCE-DDS Agent installed!"
