#!/bin/bash
daemon_name=solard
echo "$(tput setaf 3)Starting $(tput setaf 6)$daemon_name.c$(tput setaf 3) compilation...$(tput sgr0)"
if [ ! -e $daemon_name.c ]
then
    echo "$(tput setaf 7)$(tput setab 1)ERROR: No $(tput smso) $daemon_name.c $(tput rmso) found in current working dir: "\
             "$(tput smso) `pwd` $(tput rmso)!!!$(tput sgr0)"
    exit 2
fi
echo "$(tput setaf 2)Source file found!$(tput sgr0)"

git describe --tag 1>/dev/null 2>&1
if (( $? > 0 ))
then
    daemon_ver="STANDALONE-`date '+%F--%T'`"
    echo "Working out version number to use... $(tput setaf 3)CUSTOM$(tput sgr0)."
else
    daemon_ver=`git describe --tag`
    echo "Working out version number to use... $(tput setaf 3)GIT-TAG based$(tput sgr0)."
fi

if [ -e $daemon_name ]
then
#    echo "$(tput setaf 3)Previous compile result: still present.$(tput sgr0)"
    mv $daemon_name $daemon_name.prev
#    echo "$(tput setaf 3)Previous compile result: renamed for now.$(tput sgr0)"
fi

gcc -D_FORTIFY_SOURCE=2 -DPGMVER=\"$daemon_ver\" -Wall -Wno-unused-result -O3 -o $daemon_name $daemon_name.c
if (( $? > 0 ))
then
    mv $daemon_name.prev $daemon_name
    echo "$(tput setaf 3)Previous compile result: RESTORED.$(tput sgr0)"
    echo "$(tput setaf 7)$(tput setab 1)ERROR: Compilation failed!$(tput sgr0)"
else
    rm $daemon_name.prev
    echo "$(tput setaf 3)Previous compile result: removed.$(tput sgr0)"
    echo "$(tput setaf 2)$(tput smso)Compilation SUCCESS!$(tput rmso)$(tput sgr0)"
fi
#EOF