@echo off
cd /d C:\Users\jaech\STM32F103RB_Modbus_ADC

echo ---------------------------------------------
echo Git Add
git add .

echo ---------------------------------------------
echo Git Commit
git commit -m "Add ADC Exponential Moving Average Filter"

echo ---------------------------------------------
echo Git Push
git push origin main

echo ---------------------------------------------
echo Push Completed!
pause
