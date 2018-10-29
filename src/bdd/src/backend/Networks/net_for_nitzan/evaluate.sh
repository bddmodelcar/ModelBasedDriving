#!/bin/bash
set -x
set -e

# Adjust for the actual no of max epochs or just let it crash
for ((i=1; i<=20; i++)); do
   python2.7 Evaluate.py --gpu 0 --data-path ~/testmount/TrainingData/processed_h5py/validation --epoch $i --save-path save --data-logfile `basename $PWD`
done
