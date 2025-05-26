Source codes for COER Board V1 (wih ESP32) with data sent over Wifi (ESP NOW protocol) to an ESP32 receiver.
Folder structure:
  . ESP_NOWCapdac contains the arduino source file for the COER Board (transimtter)
  . ESPNow_Receiver contains the soruce file for the receiver
  . ESPNow_MACaddress contains the source file for finding the MAC address of the baords
  . ArduinoLibraies contains the libraries for the sensors. [The library provided by Protocentral for FDC1004 has a bug. It has been corrected in the library uploaded here.]

Known issues:
  It is recommended to remove the daughter board before flashing the main board since this may cause failure to upload.
