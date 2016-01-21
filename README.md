# solarmanpi

## What is this?
solard is a small daemon designed to run on a raspberry pi with custom hardware acting as a solar controller for drain-back system
integrated in a home central heating system. The custom hardware is actually a kind of breadboard to allow to connect physical devices to the RPi with the proper pull-up and pull-down resistors. The devices connected are: at least 4 1-wire digital thermomethers, and five relays - 4 controlled, and one being read.

The daemon collects data from the 4 DS18B20 1-wire temperature sensors, and acts on it by controlling 4 relays via GPIO. Data is being read sequentially from the 4 temp sensors, then the one input GPIO, a decision is made what to turn on or off by the set parameters, all this is logged, and the remainder of a 10 seconds interval is slept.

The collected data gets logged in 2 files - one is human readable CSV file, the other - continuously rewritten every
10 seconds - for reading by data gathering daemon like collectd. Other software may be used to create pretty graphs
from gathered data. Also, work is done on a web interface for near-real-time display and authorized configuration, which uses the continuously rewritten data, but it is being developed separately.
