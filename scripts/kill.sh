#!/bin/bash

cd ~/rex_ws
source devel/setup.bash

rosnode kill --all&
sudo pkill roscore&
sudo pkill rosmaster&