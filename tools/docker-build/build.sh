#!/bin/bash

git config --global --add safe.directory $CP_DIR

git config --global --add safe.directory $CP_DIR/ports/espressif/esp-idf

cd $CP_DIR

make fetch-tags

. /tools/esp/esp-idf/export.sh

cd $CP_DIR/ports/$1

make BOARD=${@:2}
