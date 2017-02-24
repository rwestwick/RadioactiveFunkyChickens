#!/bin/bash

rm -f *.xml
rm -f *.sc
rm -f pylint.txt
rm -f flake8.txt


#--------------------------------------------------------------
# Run the pep8 utilities
#--------------------------------------------------------------
echo "**************** $(date) - Running pylint ****************"
find . -name \*.py ! -path '*test*' | xargs pylint --rcfile=jenkins/pylint.cfg > pylint.txt
echo "**************** $(date) - pylint complete ****************"


echo "**************** $(date) - Running flake8 ****************"
find . -name \*.py ! -path '*test*' | xargs flake8 > flake8.txt
echo "**************** $(date) - flake8 complete ****************"


#--------------------------------------------------------------
# Run the sloccount utility
#--------------------------------------------------------------
echo "**************** $(date) - Running cloc ****************"

cloc --by-file \
     --xml \
     --out=./cloc.xml \
     --exclude-dir=.git,.svn,.idea \
	 .

echo "**************** $(date) - cloc complete ****************"