#!/bin/sh
# Grid Voltage shot be recieved, normally in range 230-240Volt
#237.2
#237.2
#236.5
#236.2

mosquitto_sub -h localhost -t "PV18/grid/voltage"
