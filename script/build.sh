# !/bin/bash

if [ $1 = "b" ]; then
    cmake -B ./build -S .
    cd ./build
    make
elif [ $1 = "r" ]; then
    ./build/BasicService ./conf/app.yaml ./conf/busi.yaml
elif [ $1 = "c" ]; then
    rm -rf ./build
fi
