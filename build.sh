#!/bin/bash

builddir=/home/pi/compile/solard

cd $builddir
if [ -e solard ]
then
    rm solard
fi
date
time gcc -D_FORTIFY_SOURCE=2 -Wall -Wno-unused-result -O3 -o solard solard.c
ls -l

builddir=
#EOF
