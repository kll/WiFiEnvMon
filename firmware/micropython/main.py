import machine
import ssd1306
import ads1x15
import bme280
import tsl2561

SCL_PIN = 5
SDA_PIN = 4
I2C_FREQ = 100000
ADC_ADDR = 0x48
DISPLAY_WIDTH = 128
DISPLAY_HEIGHT = 64

def main():
    """The main loop of the program."""
    oled, adc = init()

    while True:
        gas = adc.read(0)
        noise = adc.read(1)

        oled.fill(0)
        oled.text(str.format("Gas: {}", gas), 0, 0)
        oled.text(str.format("Noise: {}", noise), 0, 10)
        oled.show()


def init():
    """Initialize required IO objects."""
    i2c = machine.I2C(scl=machine.Pin(SCL_PIN), sda=machine.Pin(SDA_PIN), freq=I2C_FREQ)
    oled = ssd1306.SSD1306_I2C(DISPLAY_WIDTH, DISPLAY_HEIGHT, i2c)
    adc = ads1x15.ADS1115(i2c, ADC_ADDR)
    return oled, adc


if __name__ == '__main__':
    main()
