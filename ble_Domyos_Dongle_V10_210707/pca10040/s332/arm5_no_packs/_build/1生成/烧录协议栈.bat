@echo off

nrfjprog.exe --eraseall -f NRF52
nrfjprog.exe --program ANT_s332_nrf52_7.0.1.hex --verify -f NRF52

pause