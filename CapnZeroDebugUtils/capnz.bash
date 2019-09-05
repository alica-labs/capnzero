#!/usr/bin/env bash
CAPNZ_LOC="${BASH_SOURCE::-10}"

function capnzero () {
    if [ "$1" = "echo" ]; then
        source "$CAPNZ_LOC/env/bin/activate"
        shift
        python3 "$CAPNZ_LOC/echo.py" "$@"
        deactivate
    fi
}
