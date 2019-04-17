#!/bin/bash
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.

# Print the IP address
_IP=$(hostname -I) || true
if [ "$_IP" ]; then
  printf "My IP address is %s\n" "$_IP"
fi

################################
#
# Custom part from solarmanpi package to complement it by starting another
# custom script tasked with exporting solard data to emoncms from the
# openenergymonitor.org package
#
/etc/rc.solard_sender >>/run/shm/sender_log &

exit 0

#EOF
