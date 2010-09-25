#!/bin/bash
LOG_FILE=mem_leak.log
valgrind --leak-check=full --show-reachable=yes --log-file=$LOG_FILE $@
echo "See log file '$LOG_FILE'"
$EDITOR $LOG_FILE

