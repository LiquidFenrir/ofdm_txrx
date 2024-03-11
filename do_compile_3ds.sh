#!/bin/bash

if [ -z "$1" ]
then
    export CUR_BUILD_TYPE="Debug"
    export CUR_EXTRA_ARGS=""
else
    export CUR_BUILD_TYPE="$1"
    export CUR_EXTRA_ARGS="${@:2}"
fi

export USING_CMAKE="/opt/devkitpro/portlibs/3ds/bin/arm-none-eabi-cmake"
export BUILDDIR_SUFFIX="-3ds"

./do_compile_inner.sh "$CUR_BUILD_TYPE" $CUR_EXTRA_ARGS
