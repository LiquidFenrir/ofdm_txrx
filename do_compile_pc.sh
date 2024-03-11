#!/bin/bash

if [ -z "$1" ]
then
    export CUR_BUILD_TYPE="Debug"
    export CUR_EXTRA_ARGS=""
else
    export CUR_BUILD_TYPE="$1"
    export CUR_EXTRA_ARGS="${@:2}"
fi

export CC=gcc
export CXX=g++
export USING_CMAKE="cmake"

./do_compile_inner.sh "$CUR_BUILD_TYPE" $CUR_EXTRA_ARGS
