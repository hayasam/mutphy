#!/bin/bash

rm -f mut*/output.txt
cd original
platformio run -t clean
platformio run -t upload
python3 -m pytest ../run_test.py | tee "output.txt"
cd ..

for mut_dir in $(ls -d mut*/); do
                echo $mut_dir
		cd $mut_dir
                platformio run -t clean
		platformio run -t upload
                python3 -m pytest ../run_test.py -x| tee "output.txt"
		cd ..
                mv $mut_dir executed
done

egrep -lir --include=*.txt "FAILED" . > "failed.txt"
