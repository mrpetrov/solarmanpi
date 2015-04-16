# solarmanpi
Small daemon designed to run on a raspberry pi with custom hardware acting as a solar controller for drain-back system
integrated in a home central heating system.

The daemon collects data from DS18B20 1-wire temperature sensors, and acts on it by controlling 4 relays via GPIO.

The collected data gets logged in 2 files - one is human readable CSV file, the other - continuously rewritten every
10 seconds - for reading by data gatharing daemon like collectd. Other software may be used to create pretty graphs
from gathered data.
