#!/bin/bash
# Extracts a subset of frames from input folder into output folder
# $1: Input folder
# $2: Output folder
# $3: Start frame
# $4: Step
# $5: End frame
# $6: Strart stream (optional, default=1)
# $7: End stream (optional, default=4)

# Checks if file exists, copies if yes, throws warning if not.
function check_and_copy {
if [ -f $1 ]
then
  cp $1 $2
else
  echo "WARNING:" $1 "not found"
fi
}

# Check correct number of arguments
if [ $# -lt 5 ]
then
  echo "Usage: $0 INPUT_DIR OUTPUT_DIR START_FRAME STEP END_FRAME [START-STREAM] [END-STREAM]"
  exit 1
fi

# If output directory doesn't exist, create it.
if [ ! -d $2 ]
then
   mkdir $2
fi

check_and_copy $1/params_R.csv $2/
check_and_copy $1/params_camchain.csv $2/

disp=disparity

for i in `seq $3 $4 $5`;
do
  check_and_copy $1/params_f_$i.csv $2/
  for j in `seq ${6-1} ${7-4}`;
  do
    if [ -f $1/depth_s_${j}_f_$i.png ]
    then
      disp=depth
    fi
	  check_and_copy $1/${disp}_normalized_s_${j}_f_$i.png $2/
	  check_and_copy $1/${disp}_s_${j}_f_$i.png $2/
	  check_and_copy $1/pic_s_${j}_f_$i.png $2/
  done
done
