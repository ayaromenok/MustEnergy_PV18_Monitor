#!/bin/sh
sudo systemctl stop pv18_monitor_py
sudo systemctl status pv18_monitor_py
sudo systemctl disable pv18_monitor_py
sudo rm /etc/systemd/system/pv18_monitor_py.service
