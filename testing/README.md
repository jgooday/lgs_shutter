# Shutter testing

I needed a way to repetitively test the shutter, so I used a spare Arduino Uno as a "virtual control system". All it does it send open and close signals to the shutter in a cycle (the CMD signal). It also records the FAULT signal using an interrupt. Whenever CMD or FAULT changes, it prints their value, along with a timestamp, to the console. This can be used with `run_test.bat` which will save all console output to a file for analysis.
