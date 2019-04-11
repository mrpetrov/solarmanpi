#!/bin/bash
service_name=solard
src_dir=/home/pi/$service_name

service $service_name stop
cp $src_dir/$service_name /usr/bin
chown root:root /usr/bin/$service_name
chmod a+x /usr/bin/$service_name
sleep 6
service $service_name start

