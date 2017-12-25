#!/bin/bash

echo "Running autopep8"
autopep8 --in-place -r -a -v *.py
echo "Running autoflake"
autoflake --remove-all-unused-imports --remove-unused-variables -r --in-place *.py
echo "Running reindent"
find . -type f -name "*.py" | xargs python /usr/share/doc/python2.7/examples/Tools/scripts/reindent.py -r -v --nobackup