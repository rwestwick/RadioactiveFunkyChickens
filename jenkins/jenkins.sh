#!/bin/bash
export PATH=$PATH:/home/bodgeit/.local/bin/

rm -f *.xml
rm -f *.sc
rm -f pylint.txt
rm -f flake8.txt


#--------------------------------------------------------------
# Run the pep8 utilities
#--------------------------------------------------------------
echo "**************** $(date) - Running pylint ****************"
find . -name \*.py ! -path '*test*' | xargs pylint --rcfile=jenkins/pylint.cfg > pylint.txt
more pylint.txt | grep -v "Relative import" | grep -v "Unable to import" > pylint2.txt
mv pylint2.txt pylint.txt
echo "**************** $(date) - pylint complete ****************"


echo "**************** $(date) - Running flake8 ****************"
find . -name \*.py ! -path '*test*' | xargs flake8 > flake8.txt
echo "**************** $(date) - flake8 complete ****************"


#--------------------------------------------------------------
# Run the cloc utility
#--------------------------------------------------------------
echo "**************** $(date) - Running cloc ****************"

cloc --by-file \
     --xml \
     --out=./cloc.xml \
     --exclude-dir=.git,.svn,.idea \
	 .

echo "**************** $(date) - cloc complete ****************"


#--------------------------------------------------------------
# Create coverage report
#--------------------------------------------------------------
coverage erase
pytest --junit-xml unittest.xml -s --cov=. --cov-config ./jenkins/.coveragerc \
       --cov-report=xml --cov-report=html --cov-report term-missing \
       MotorController_test.py \
       DualMotorController_test.py \
       MecanumController_test.py \
       UltrasonicSensor_test.py \
       UltrasonicSensorThread_test.py \
       SwitchingGPIO_test.py \
       IRSensor_test.py \
       LineFollowerSensor_test.py \
       ServoController_test.py
