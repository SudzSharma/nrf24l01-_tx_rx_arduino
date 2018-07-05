# Transmit and Recieve through NRF24l01+ using Arduino.

## Description

This repository constains the RX and TX codes for NRF24l01+ Communication based on the RF24 Library 
[Link](https://github.com/sudo-sh/RF24)
The code sends the Roll, Pitch and Yaw through MPu6050 developed for a particular work.
However, It can be modified easily according to the need. 
The code is commented extensively to enhance the reusability. 

## How to use?
```
paste the files in the arduino sketch after installing the discrepancies.
Read the advice section once.

Change the Pin Assignment according to the Board and the Connections.

```

## Discrepancies
-Arduino IDE
-RF24 Library 
-Wire Library-(If MPU is used) (Remove the usage from the code if not used)
-SPI Library



## Advice and some sayings from Experience

-When using nrf first of all make sure that the connections are perfect, check everything.
  The power levels pin connections etc.
  
-Make sure that the nrf is working, test the demo code from the library.

-If you have the chineese nrfs, change the modules and try again.

-Sometimes nrf doesnt work well because of the power level, Any functuation can leead to its malfunctioning, and 
it take suitable amount of current, so make sure you power it with proper power source.

-The fake onces are less reliable. Range of Communication can change anytime cause problems.


Please contact me if you have any doubts.

Thanks
