#!/bin/bash

sudo kill -9 $(ps -elf|grep -v grep|grep python3|grep alphabetically|awk '{print $4}') >/dev/null 2>&1
sudo systemctl restart alphabetically.service  

sudo kill -9 $(ps -elf|grep -v grep|grep python3|grep waste_classification|awk '{print $4}') >/dev/null 2>&1
sudo systemctl restart waste_classification.service

sudo kill -9 $(ps -elf|grep -v grep|grep python3|grep object_tracking|awk '{print $4}') >/dev/null 2>&1
sudo systemctl restart object_tracking.service

sudo kill -9 $(ps -elf|grep -v grep|grep python3|grep color_sorting|awk '{print $4}') >/dev/null 2>&1
sudo systemctl restart color_sorting.service


