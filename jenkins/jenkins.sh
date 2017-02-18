find . -name \*.py | xargs pylint > pylint.txt
find . -name \*.py | xargs flake8 > flake8.txt
