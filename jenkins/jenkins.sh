#!/bin/bash

rm -f *.xml
rm -f *.sc
rm -f pylint.txt
rm -f flake8.txt


#--------------------------------------------------------------
# Run the pep8 utilities 
#--------------------------------------------------------------
echo "**************** $(date) - Running pylint ****************"
find . -name \*.py | xargs pylint > pylint.txt
echo "**************** $(date) - pylint complete ****************"


echo "**************** $(date) - Running flake8 ****************"
find . -name \*.py | xargs flake8 > flake8.txt
echo "**************** $(date) - flake8 complete ****************"


#--------------------------------------------------------------
# Run the sloccount utility
#--------------------------------------------------------------
echo "**************** $(date) - Running cloc ****************"

cloc --by-file \
     --xml \
     --out=./cloc.xml \
     --exclude-dir=.git,.svn \

echo "**************** $(date) - cloc complete ****************"
