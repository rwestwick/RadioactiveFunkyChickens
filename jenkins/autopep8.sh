#!/bin/bash

echo "Running yapf"
yapf -i -r *.py

echo "Running reindent"
find . -type f -name "*.py" | xargs python /usr/share/doc/python2.7/examples/Tools/scripts/reindent.py -r -n

echo "Running autopep8"
autopep8 -i -r -aa *.py

echo "Running autoflake"
autoflake --remove-all-unused-imports --remove-unused-variables -r -i *.py

echo "Running pep8ify"
pep8ify -n -w .