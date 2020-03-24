#!/bin/bash

PROGRAMMER=stlinkv2
#PROGRAMMER=stlink
TARGET=stm8s105?6

EMPTY_PROGRAM=empty_memory/empty_program.hex
EMPTY_DATA=empty_memory/empty_data.hex
DEFAULT_OPTION=empty_memory/default_option.bin

ORIGINAL_PROGRAM=original_firmware/program.s19
ORIGINAL_DATA=original_firmware/data.s19
ORIGINAL_OPTION=original_firmware/option_bytes_tongsheng.bin

HEX_FILE=$1

echo $HEX_FILE

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
# write hex files
#

if test ! -f $HEX_FILE; then echo "hex file does not exist"; exit 1; fi

echo -e "\nWRITING PROGRAM\n"
stm8flash -c $PROGRAMMER -p $TARGET -s flash -w $ORIGINAL_PROGRAM
if (test $? -ne 0); then echo "writing hex file failed"; exit 1; fi 

echo -e "\nVERIFYING PROGRAM\n"
stm8flash -c $PROGRAMMER -p $TARGET -s flash -v $ORIGINAL_PROGRAM
if (test $? -ne 0); then echo "verification of hex file failed"; exit 1; fi 


echo -e "\nWRITING DATA\n"
stm8flash -c $PROGRAMMER -p $TARGET -s eeprom -w $ORIGINAL_DATA
if (test $? -ne 0); then echo "writing hex file failed"; exit 1; fi 

echo -e "\nVERIFYING DATA\n"
stm8flash -c $PROGRAMMER -p $TARGET -s eeprom -v $ORIGINAL_DATA
if (test $? -ne 0); then echo "verification of hex file failed"; exit 1; fi 


echo -e "\nWRITING OPTION BYTE\n"
stm8flash -c $PROGRAMMER -p $TARGET -s opt -w $ORIGINAL_OPTION
if (test $? -ne 0); then echo "writing hex file failed"; exit 1; fi 

echo -e "\nVERIFYING OPTION BYTE\n"
stm8flash -c $PROGRAMMER -p $TARGET -s opt -v $ORIGINAL_OPTION
if (test $? -ne 0); then echo "verification of hex file failed"; exit 1; fi 



exit 0
