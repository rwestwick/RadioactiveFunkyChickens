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

#cd code
#git clone git://git.drogon.net/wiringPi
#cd wiringPi
#git pull origin
#sudo ./build
#cd wiringPi
#make static
#sudo make install-static
#cd ../..
