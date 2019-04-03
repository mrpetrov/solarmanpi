#!/bin/bash

echo "Checking if solard.c is present..."
if [ ! -e solard.c ]
then
    echo "$(tput setaf 1)$(tput setab 7)Refusing to start from non-source-root dir...$(tput sgr 0)"
    echo "$(tput setaf 1)$(tput setab 7)ERROR: No solard.c found in $(tput smso)`pwd`$(tput rmso)!$(tput sgr 0)"
    exit 2
fi
echo "$(tput setaf 2)OK.$(tput sgr 0)"

if [ -e solard ]
then
    rm solard
	echo "$(tput setaf 3)Previous compile result was still present. It is now removed.$(tput sgr 0)"
fi
if ! gcc -D_FORTIFY_SOURCE=2 -DSOLARDVERSION=\"`git describe --tag`\" -Wall -Wno-unused-result -O3 -o solard solard.c; then
	echo "$(tput setaf 1)$(tput setab 7)ERROR: Compilation failed!$(tput sgr 0)"
else
	echo "$(tput setaf 2)Successfully compiled solard version $(tput smso)`git describe --tag`$(tput rmso) at: $(tput setaf 4)`date`$(tput sgr 0)"
fi

#EOF
