@echo off
nrfjprog.exe --eraseall -f NRF52
nrfjprog.exe --program ble_Domyos_Dongle_V10_yymmdd.hex --verify -f NRF52
nrfjprog.exe --debugreset -f NRF52
