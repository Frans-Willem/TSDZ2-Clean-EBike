#!/bin/sh
set -ex
rm -rf ./package
mkdir ./package
cp ./src/controller/main.hex ./package/TSDZ2.hex
cp ./src/display/KT-LCD3/main.hex ./package/KT-LCD3.hex
cp ./README.md ./package/
echo -n "Current version: " > ./package/version.txt
git describe --always --dirty >> ./package/version.txt
echo --- >> ./package/version.txt
echo History: >> ./package/version.txt
git log -n30 >> ./package/version.txt
unix2dos package/*.txt
rm -rf package.zip
cd ./package
zip -r ../package.zip ./
