#!/bin/bash

# Script to flash the firmware
# usage: flash_linux.sh main.hex in src/controller directory
# The Script resets the memory before the new firmware is written

PROGRAMMER=stlinkv2
#PROGRAMMER=stlink
TARGET=stm8s105?6

EMPTY_PROGRAM=../../releases/empty_memory/empty_program.hex
EMPTY_DATA=../../releases/empty_memory/empty_data.hex
DEFAULT_OPTION=../../releases/empty_memory/default_option.bin

HEX_FILE=$1

#echo $HEX_FILE

#
# clear memory
#

echo -e "\nERASING FLASH\n"
stm8flash -c $PROGRAMMER -p $TARGET -s flash -w $EMPTY_PROGRAM
if (test $? -ne 0); then echo "clearing flash failed"; exit 1; fi 

echo -e "\nERASING EEPROM\n"
stm8flash -c $PROGRAMMER -p $TARGET -s eeprom -w $EMPTY_DATA
if (test $? -ne 0); then echo "clearing eeprom failed"; exit 1; fi 

echo -e "\nERASING OPTION BYTE\n"
stm8flash -c $PROGRAMMER -p $TARGET -s opt -w $DEFAULT_OPTION
if (test $? -ne 0); then echo "clearing option failed"; exit 1; fi 

#
# write hex file
#

if [ -f "$HEX_FILE" ]; then echo $HEX_FILE; else echo -e "\nhex file not found\n"; exit 1; fi

echo -e "\n"
stm8-size $HEX_FILE
if (test $? -ne 0); then echo "stm8-size failed"; exit 1; fi 

echo -e "\nWRITING PROGRAM\n"
stm8flash -c $PROGRAMMER -p $TARGET -s flash -w $HEX_FILE
if (test $? -ne 0); then echo "writing hex file failed"; exit 1; fi 

echo -e "\nVERIFYING PROGRAM\n"
stm8flash -c $PROGRAMMER -p $TARGET -s flash -v $HEX_FILE
if (test $? -ne 0); then echo "verification of hex file failed"; exit 1; fi 




exit 0
