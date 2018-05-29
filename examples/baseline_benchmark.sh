#!/bin/bash

# Default configuration to get the center of a mouse via background subtraction
# Background subtraction uses the first image of SRC
# $1: source directory with pngs
# $2: target directory for output data
# $3: first frame, default: 0
# $4: last frame, default: 100000
#
# The script assumes to be run in the same directory where also the mousetrack binary is


APP=./mousetrack

SRC=$1
OUT=$2
FRAMES="--first-frame=${3-0} --last-frame=${4-100000}"

$APP -c -s $SRC -l trace --pipeline-timer --pipeline-timer-log $OUT ${FRAMES} --pipeline-frame-window-filtering=background-subtraction --background-subtraction-cage-directory=$SRC --pipeline-clustering=single-cluster 


