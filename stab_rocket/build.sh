#!/usr/bin/env bash

echo "raspi-stabilization stab_rocket build.sh:"

if [[ $# -eq 0 ]] ; then
    echo "No arguments supplied, defaulting to 'all'"
    BUILD="all"
else
    BUILD=$1
fi

if [ $BUILD == "all" ] ; then
    echo "Generating python bindings..."
    ./gen-types.sh
    echo "Generating .jar file..."
    ./createjar.sh
    echo "Types built!"
elif [ $BUILD == "clean" ] ; then
    echo "Cleaning build artifacts..."
    rm my_types.jar
    rm -rf rocket
    echo "Artifacts cleaned!"
else
    echo "Invalid parameter, candidates are: all clean"
fi

