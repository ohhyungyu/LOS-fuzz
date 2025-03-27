#!/bin/bash
file_name="fastrtps_"
file_dir="/dev/shm"

sequence="0 1 2 3 4 5 6 7 8 9 a b c d e f"

for i in $sequence
do
    for j in $sequence
    do
        echo "GOOD ${i} ${j}"
        rm -rf $file_dir/$file_name$i$j*
    done
done

rm -rf $file_dir/*