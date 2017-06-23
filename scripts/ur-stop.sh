#!/bin/bash
## A script that cleans up as much pof the other script's mess as it can.

rosnode kill /ur_follow_trajectory
kill `pgrep sns-pblend`
kill `pgrep sns-watchdog`
sns stop

