#!/bin/bash

sudo ./clear_environment.sh

colcon build --packages-select pid_debug_interfaces uart


