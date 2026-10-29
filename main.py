#main.py running on ESP32 that acts as a bridge between sensors and lego hub

from machine import Pin, I2C, UART
from time import sleep
import time
from pupremote import PUPRemoteSensor, SPIKE_ULTRASONIC


# ============================================================
# UART to OpenMV
# ============================================================

# ESP32 TX=14 -> OpenMV RX
# ESP32 RX=13 <- OpenMV TX
openmv_uart = UART(1, baudrate=115200, tx=14, rx=13)
openmv_uart.init(115200, bits=8, parity=None, stop=1)


# ============================================================
# I2C pins
# ============================================================

SCL_PIN = 32
SDA_PIN = 33

i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100000)


# ============================================================
# TCS34725 color sensor
# ============================================================

ADDR = 0x29
CMD = 0x80

REG_ENABLE = 0x00
REG_ATIME = 0x01
REG_CONTROL = 0x0F
REG_ID = 0x12
REG_CDATAL = 0x14


# ============================================================
# Sensitivity settings
# ============================================================

C_SCALE = 1
ATIME_VALUE = 0xFC
GAIN_VALUE = 0x01

last_c = 0
sensor_ok = False


# ============================================================
# Helpers
# ============================================================

def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def write8(reg, value):
    i2c.writeto_mem(ADDR, CMD | reg, bytes([value]))


def read8(reg):
    return i2c.readfrom_mem(ADDR, CMD | reg, 1)[0]


def read16(reg):
    data = i2c.readfrom_mem(ADDR, CMD | reg, 2)
    return data[0] | (data[1] << 8)


def init_sensor():
    global sensor_ok

    print("Scanning I2C...")
    devices = i2c.scan()
    print("Found devices:", [hex(d) for d in devices])

    if ADDR not in devices:
        print("No color sensor found at 0x29.")
        sensor_ok = False
        return

    try:
        sensor_id = read8(REG_ID)
        print("Sensor ID:", hex(sensor_id))

        write8(REG_ATIME, ATIME_VALUE)
        write8(REG_CONTROL, GAIN_VALUE)

        write8(REG_ENABLE, 0x01)
        sleep(0.01)

        write8(REG_ENABLE, 0x03)
        sleep(0.1)

        sensor_ok = True
        print("Color sensor ready.")

    except Exception as e:
        print("Sensor init error:", e)
        sensor_ok = False


def read_c_raw():
    return read16(REG_CDATAL)


# ============================================================
# line()
# ============================================================

def line():
    global last_c, sensor_ok

    if not sensor_ok:
        return last_c

    try:
        c_raw = read_c_raw()
        c = c_raw // C_SCALE

        c = clamp(c, -32768, 32767)
        last_c = c

        print("C raw:", c_raw, "C sent:", c)

        return c

    except Exception as e:
        print("Read error:", e)
        sensor_ok = False
        return last_c


# ============================================================
# scan()
# ============================================================

def scan(dummy=0):
    try:
        # Clear old UART data first.
        while openmv_uart.any():
            openmv_uart.read()

        print("Sending R to OpenMV")
        openmv_uart.write(b"R\n")

        start = time.ticks_ms()

        while time.ticks_diff(time.ticks_ms(), start) < 700:
            msg = openmv_uart.readline()

            if msg:
                try:
                    text = msg.decode().strip()
                    print("Decoded OpenMV scan reply:", text)

                    parts = text.split(",")

                    if len(parts) == 12:
                        values = tuple(int(x) for x in parts)
                        print("Scan successful:", values)
                        return values

                    print("Wrong number of values:", len(parts), text)
                    return (0,) * 12

                except Exception as e:
                    print("Parse error:", e)
                    print("Raw message:", msg)
                    return (0,) * 12

            time.sleep_ms(10)

        print("Timeout waiting for OpenMV reply")
        return (0,) * 12

    except Exception as e:
        print("openmv_uart error:", e)
        return (0,) * 12


# ============================================================
# PUPRemote setup
# ============================================================

p = PUPRemoteSensor(
    sensor_id=SPIKE_ULTRASONIC,
    power=True,
    max_packet_size=16
)

p.add_command("line", "h")
p.add_command("scan", to_hub_fmt="bbbbbbbbbbbb", from_hub_fmt="b")


# ============================================================
# Start
# ============================================================

init_sensor()

print("ESP32 original line + scan bridge ready.")

while True:
    p.process()
