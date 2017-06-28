#!/bin/bash
## A script that cleans up as much pof the other script's mess as it can.
## You can safely ignore most of the error messages that happen when running this script.

rosnode kill /ur_follow_trajectory
kill `pgrep sns-pblend`
kill `pgrep sns-watchdog`
kill `pgrep ur_realtime_` # pgrep doesn' work with long names?
sns stop

