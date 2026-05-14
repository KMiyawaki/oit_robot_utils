#!/bin/bash
# sudo pip install autoflake
# sudo pip install isort

function main(){
    cd "$(dirname "$0")"
    lst=$(find . -type f -name "*.py"|sort)
    for f in ${lst}; do
        autopep8 --in-place ${f}
        autoflake --in-place --remove-all-unused-imports ${f}
        isort ${f}
    done
}

main "$@"
