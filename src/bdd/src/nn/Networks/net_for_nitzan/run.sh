#!/bin/bash
set -x
set -e

for ((i=0; i<=1000; i++)); do
    python2.7 Train.py --epoch $i --use-states 6 "$@" 
done
