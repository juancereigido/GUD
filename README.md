# GUD: A motion-controlled table lamp

## Description
This project uses an MPU6050 sensor to control a table lamp in the form of a cube.

### How it Works
- **Switching ON/OFF:** 
    - Tilting the lamp in the X axis forward switches it on with a smooth fade-in effect.
    - Tilting it backward switches it off with a fade-out effect.
  
- **Adjusting Brightness:**
    - Tilting the lamp in the Y axis **while holding** to the right increases the brightness.
    - Tilting it to the left decreases the brightness.

Additionally, the system includes a basic filter in the readings of the MPU6050. This helps in smoothing the values to ensure functions do not get triggered by minor noise or micro-movements.

## Libraries & Dependencies
- Arduino
- Wire (for I2C communication)
- FastLED (to control LEDs)
- EEPROM (to save settings)

## Constants Configuration
- **MPU6050 Sensor Address:** `0x68`
- **LED Matrix Configuration:** 
    - Number of LEDs: `64`
    - LED Pin: `4`
- **Tilt Detection:** 
    - Tilt Threshold: `7`
- **Brightness Settings:**
    - Fade Speed: `5`
    - Max Brightness: `255`
    - Min Brightness: `0`
    - Brightness Increment: `1`
    - Brightness Threshold: `50` (to avoid sudden LED changes)

## Debug Mode
Set `DEBUG` to `1` to enable debugging. This will output debug messages over Serial. Otherwise, set it to `0` to disable.

## Calibration
On startup, the MPU is calibrated, ensuring that even if the lamp is turned on a tilted table, it will start with all axes at zero.

## Brightness Memory
The system remembers the last brightness level set by the user. This setting is saved in EEPROM and retrieved every time the lamp is powered up.

## Further Development
While the core functionalities (tilt to power on/off, adjust brightness) are in place, this platform can be extended with more interactive features or different sensors. For example, integrating color sensors can allow for automatic ambient color matching.

## License
[MIT License](LICENSE)

## Acknowledgements
Thanks to ChatGPT for writing this README.
