#!/bin/bash

sudo apt-get install python-flake8 -y
sudo apt-get install python-pylint -y
sudo apt-get install python-autoflake -y
sudo apt-get install autopep8 -y
sudo apt-get install python-examples -y
sudo apt-get install python-dev libffi-dev libssl-dev -y
sudo apt-get install python-logilab-common -y
sudo apt-get install wiringpi -y
sudo apt-get install git-core -y
sudo apt-get install libcwiid1 lswm wmgui wminput -y
sudo apt-get install cloc -y
sudo apt-get install doxygen -y
sudo apt-get install googletest -y
sudo apt-get install samba -y
sudo apt remove wolfram-engine -y

sudo pip install --upgrade pip
sudo pip install --upgrade ndg-httpsclient 
sudo pip install --upgrade setuptools
sudo pip install pep8ify
sudo pip install yapf
sudo pip install pytest
sudo pip install pytest-cov
sudo pip install colorlog
sudo pip install coverage
sudo pip install pyzmq
sudo pip install autoflake
sudo pip install pylint
sudo pip install flake8
sudo pip install autopep8

cd ~/code
git clone https://github.com/google/protobuf
cd protobuf
./autogen.sh
./configure
make
sudo make install
sudo ldconfig


cd ~/code
git clone https://github.com/jedisct1/libsodium.git
cd libsodium
./autogen.sh 
./configure
make
sudo make install
sudo ldconfig


cd ~/code
wget http://download.zeromq.org/zeromq-4.1.4.tar.gz
tar -zxvf zeromq-4.1.4.tar.gz
cd zeromq-4.1.4/
./configure
make
sudo make install
sudo ldconfig


#cd code
#git clone git://git.drogon.net/wiringPi
#cd wiringPi
#git pull origin
#sudo ./build
#cd wiringPi
#make static
#sudo make install-static
#cd ../..
