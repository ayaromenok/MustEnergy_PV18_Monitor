#!/bin/sh
sudo cp ./pv18_monitor_py.service /etc/systemd/system/
sudo systemctl start pv18_monitor_py
sudo systemctl status pv18_monitor_py
sudo systemctl enable pv18_monitor_py