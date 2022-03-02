#!/bin/bash
# - setup of Robobot
sudo apt update
sudo apt -y dist-upgrade
# hostname
echo "robobot" >hostname
pi@raspberrypi:~ $ sudo cp hostname /etc
rm hostname
#
sudo adduser local
# requires password and other settings manually
sudo usermod -a --groups adm,cdrom,sudo,audio,video,plugdev,games,users,netdev,input,spi,gpio,i2c,dialout local 
#
su - local
# requires password
