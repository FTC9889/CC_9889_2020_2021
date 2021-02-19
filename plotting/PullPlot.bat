set arg1=%1
@echo off
echo cd platform-tools
adb pull /sdcard/saved_data/%arg1%.csv
move %arg1%.csv ../.
cd ../.
py plot.py %arg1%