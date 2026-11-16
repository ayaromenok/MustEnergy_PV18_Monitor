#!/bin/sh
sudo systemctl stop pv18_monitor_py
sudo systemctl start pv18_monitor_py
sudo systemctl status pv18_monitor_py