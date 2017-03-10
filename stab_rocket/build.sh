#!/usr/bin/env bash

echo "raspi-stabilization stab_rocket build.sh:"

. ./config.sh

if [[ $DEBUG -eq 0 ]] ; then
    DBG= 
elif [[ $DEBUG -eq 1 ]] ; then
    DBG=-DDEBUG
else
    DBG=-DDEBUG_V -DDEBUG
fi

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
    make $BUILD DEBUG=$DBG
    cd ../
    cp src/tenDOF .
elif [ $BUILD == "remake" ] ; then
    echo "Cleaning build artifacts..."
    rm -f my_types.jar
    rm -rf rocket
    rm -f tenDOF
    echo "Artifacts cleaned!"
    echo "Generating language bindings..."
    ./gen-types.sh
    echo "Generating .jar file..."
    ./createjar.sh
    echo "Types built!"
    echo "Calling make remake in src"
    cd src
    make $BUILD DEBUG=$DBG
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

