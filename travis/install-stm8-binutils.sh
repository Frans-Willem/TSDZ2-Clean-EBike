#!/bin/sh
set -ex
cd /tmp/
wget https://netix.dl.sourceforge.net/project/stm8-binutils-gdb/stm8-binutils-gdb-sources-2018-03-04.tar.gz
tar -xf stm8-binutils-gdb-sources-2018-03-04.tar.gz
cd stm8-binutils-gdb-sources
./patch_binutils.sh
./configure_binutils.sh
cd binutils-2.30
make
sudo make install
