#!/usr/bin/env bash

echo "raspi-stabilization stab_rocket build.sh:"

if [[ $# -eq 0 ]] ; then
    echo "No arguments supplied, defaulting to 'all'"
    BUILD="all"
else
    BUILD=$1
fi

if [ $BUILD == "all" ] ; then
    echo "Generating language bindings..."
    ./gen-types.sh
    echo "Generating .jar file..."
    ./createjar.sh
    echo "Types built!"
    echo "Calling make in src"
    cd src
    make $BUILD
    cd ../
    cp src/tenDOF .
elif [ $BUILD == "clean" ] ; then
    echo "Calling make clean in src"
    cd src
    make $BUILD
    cd ../
    echo "Cleaning build artifacts..."
    rm -f my_types.jar
    rm -rf rocket
    rm -f tenDOF
    echo "Artifacts cleaned!"
else
    echo "Invalid parameter, candidates are: all clean"
fi

