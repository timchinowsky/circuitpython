#!/bin/bash

DIR=$(pwd)

git config --global --add safe.directory /root/circuitpython
git config --global --add safe.directory /root/circuitpython/ports/espressif/esp-idf

cd /root/circuitpython

make fetch-tags

. /root/esp/esp-idf/export.sh

cd /root/circuitpython/ports/$1

# cd $DIR

make BOARD=$2 $3
