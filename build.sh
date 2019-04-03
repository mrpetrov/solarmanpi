#!/bin/bash

if [ ! -e solard.c ]
then
    echo Refusing to start from non-source-root dir...
    echo ERROR: No solard.c found in `pwd`!
    exit 2
fi

if [ -e solard ]
then
    rm solard
fi
date
time gcc -D_FORTIFY_SOURCE=2 -DSDGV=\" `git describe`\" -Wall -Wno-unused-result -O3 -o solard solard.c
ls -l

#EOF
