# nRF52840-zigbee-color-light-bulb
This is a zigbee-ble multiprotocol example demonstrating a dimmable color light device on a nRF52840 dongle. After compiling and installing the FW to the nRF52840 dongle, the user can control the color LED over Zigbee or BLE.

Currently, this project only supports Segger Embedded Studio as IDE.

## Hardware Requirement
- At least one nRF52840 Dongle to run as Color Dimmable Light Device
- Amazon Echo+ or other Zigbee coordinator that support Color Dimmable Light Device
- [nRF Toolbox](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Toolbox) to control color LED over BLE NUS

## How to Compile
1. Download the [nRF5 SDK for Thread and Zigbee](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK-for-Thread-and-Zigbee) v2.0
2. Clone the project to examples\multiprotocol\ble_zigbee
3. Open SES project at examples\multiprotocol\ble_zigbee\nRF52840-zigbee-color-light-bulb\pca10059\s140\ses
4. Compile

## How to Install
1. Follow the guide of [Program application using nRF Connect Programmer](https://devzone.nordicsemi.com/tutorials/b/getting-started/posts/nrf52840-dongle-programming-tutorial) to program soft device s140 v6.1.0 to nRF52840 dongle
2. Program compiled application hex to nRF52840 dongle
3. If both SoftDevice and Application are programmed successfully, the LED1 and LED2 will start blinking:
    - LED1 start blinking when BLE advertising
    - LED2 start blinking in green when it is trying to join a Zigbee network

## How to Test

1. Test with Amazon Echo+
    1. Make sure LED2 is blinking in green
    2. Open Amazon Alexa app on mobile
    3. Device -> "+" icon -> Add Device -> Light -> Other -> DISCOVER DEVICES
    4. After 1 ~ 2 minutes, a new light bulb will be discovered.
    5. Now you can control the LED2 of Alexa app
        - Turn on/off light
        - Change Brightness
        - Set Colour
    6. You can also voice control your new light. For example: 
        - "Turn on First Light"
        - "Turn First Light to 20 percent"
        - "Turn First Light to Blue"

2. Test with [nRF Toolbox](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Toolbox)
    1. Open nRF Toolbox
    2. Open UART
    2. Connect to device named "Zigbee_UART"
    3. Press "EDIT" to configure buttons
        1. The supported commands are:
            - on
            - off
            - white
            - red
            - yellow
            - green
            - cyan
            - blue
            - purple
        2. Select EOL as "LF"
        3. Press OK to finish button setting
    4. Press "DONE" to finish configure
    5. Now you can control the LED with your configured buttons

## How to Reset Color Light Bulb
To reset the ligh bulb, hold the Button (SW1) when power on the dongle.





