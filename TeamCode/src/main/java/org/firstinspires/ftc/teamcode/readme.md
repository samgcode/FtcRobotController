# Important info for working on this project

# Connecting to the robot over wifi
1.) make sure the hub has power and plug it into the computer through USB C
2.) the device should show up in the top bar next to the play button
3.) connect to the robot wifi on wifi 2
4.) click the "debug" button next to the play button
5.) wait for the "launch succeded" to show up
6.) in the top bar go to "Tools" > "External Tools" and select "Enable ADB over TCP/IP"
7.) once that is completed go to "Tools" > "External Tools" and select "Connect to ADB over WiFi Direct"
8.) you can now unplug the robot from the computer
9.) whenever you click debug it will upload the code to the robot

#Common issues:
"more than one device/emulator"
- close and open android studio

"no device/emulator" or "could not find the file specified"
- close android studio
- turn the robot off and on again
- open android studio


# FTC dashboard
1.) connect to the robots wifi
2.) go to http://192.168.43.1:8080/dash in a browser