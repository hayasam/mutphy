#!/bin/bash

for mut_dir in $(ls -d mut*/); do
		cd $mut_dir
		platformio test -e uno | tee "output.txt"
		cd ..
done

egrep -lir --include=*.txt "FAILED" . > "failed.txt"
