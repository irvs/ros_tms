# /bin/sh

cd $(dirname $0)
find . -type f -regex ".*\.\(c\|cpp\|cxx\|h\|hpp\|hxx\)$" | xargs clang-format-3.6 -i -style=file $1
# find . -type f -regex ".*\.\(c\|cpp\|cxx\|h\|hpp\|hxx\)$" | xargs clang-format-3.6 -i -style="{BasedOnStyle: Google, ColumnLimit: 100, IndentWidth: 2, UseTab: Never}"
find . -type f -regex ".*\.\(py\)$"|while read f;do autopep8 -i --max-line-length 120 $f;done
