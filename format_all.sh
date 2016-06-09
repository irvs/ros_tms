# /bin/sh

cd $(dirname $0)
alias beautify='clang-format-3.6 -style="{BasedOnStyle: Google, ColumnLimit: 100, IndentWidth: 2, UseTab: Never}" -i'
find . -type f -regex ".*\.\(c\|cpp\|cxx\|h\|hpp\|hxx\)$"|while read f
do
    beautify $f
done
