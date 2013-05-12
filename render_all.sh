#!/bin/bash

function check_error {
    if [ "$?" -ne "0" ]; then
        echo "An error occurred!"
        exit 1
    fi
}

echo "test01: Animated Transformations"
./view -i -t 2.5 scenes/test01.json
check_error

echo "test02: Skinned Mesh"
./view -i -t 2.5 scenes/test02.json
check_error

echo "test03: Particles"
./view -i -t 2.5 scenes/test03.json
check_error

echo "All completed successfully!"
