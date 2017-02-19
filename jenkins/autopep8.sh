#!/bin/bash

autopep8 --in-place -a -v *.py
autoflake --remove-all-unused-imports --remove-unused-variables -r --in-place *.py
