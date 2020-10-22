#!/bin/bash
#

if [ "$EUID" -ne 0 ]
then
  echo "Please run as root!"
  exit 1
fi

# copy rc.solard_sender to /etc/rc.solard_sender
cp ./rc.solard_sender /etc/rc.solard_sender
if (( $? > 0 ))
then
    echo "$(tput setaf 7)$(tput setab 1)ERROR: Could not create /etc/rc.solard_sender!$(tput sgr0)"
    exit 1
fi
chmod +x /etc/rc.solard_sender

# backup the the original rc.local we find
mv /etc/rc.local /etc/rc.local.bak -f

# then replace it with our version
cp ./solard_data_sender_starter.sh /etc/rc.local
if (( $? > 0 ))
then
    echo "$(tput setaf 7)$(tput setab 1)ERROR: Could NOT replace /etc/rc.local!$(tput sgr0)"
    exit 2
fi
chmod +x /etc/rc.local

echo "$(tput setaf 2)$(tput smso)Install went OK!$(tput rmso)$(tput sgr0)"

exit 0

#EOF
