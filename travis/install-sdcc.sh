#!/bin/sh
set -ex
cd /tmp/
wget https://downloads.sourceforge.net/sourceforge/sdcc/sdcc-src-3.9.0.tar.bz2
tar -xf sdcc-src-3.9.0.tar.bz2
cd sdcc-3.9.0
./configure --disable-pic14-port --disable-pic16-port
make
sudo make install
