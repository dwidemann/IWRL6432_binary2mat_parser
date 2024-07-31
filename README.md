This script reads in a folder of binary files created by TI's IWRL6432 sensor and parses them to create
.mat files that are read in by python.

Example usage:

Open matlab
open the file `bin2fHist_looping_timestamps.m`

edit line 12:
binFilePath = "/Users/davidwidemann/Documents/IWRL6432/data/07_29_2024_10_23_28";
 
to point to the directory that has your data. 

run the script at the matlab prompt:

> bin2fHist_looping_timestamps