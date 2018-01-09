#!/bin/bash

sudo apt-get install python-flake8 -y
sudo apt-get install python-pylint -y
sudo apt-get install python-autoflake -y
sudo apt-get install autopep8 -y
sudo apt-get install python-examples -y
sudo apt-get install python-dev libffi-dev libssl-dev -y
sudo apt-get install python-logilab-common -y
sudo pip install --upgrade pip
sudo pip install --upgrade ndg-httpsclient 
sudo pip install --upgrade setuptools
sudo pip install pep8ify
sudo pip install yapf
sudo pip install pytest
sudo pip install pytest-cov
sudo pip install colorlog
sudo apt-get purge wiringpi
sudo apt-get install git-core
cd code
git clone git://git.drogon.net/wiringPi
cd wiringPi
git pull origin
sudo ./build
cd wiringPi
make static
sudo make install-static
cd ../..
sudo apt-get install libcwiid1 lswm wmgui wminput
#sudo pip install tesseract
#sudo pip install lept
sudo apt install libgtk2.0-dev
