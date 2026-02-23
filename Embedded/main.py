from machine import Pin, I2C, ADC
import time
# Initialize I2C
i2c = I2C(scl=Pin(9), sda=Pin(8), freq=400000)
# Scan for I2C devices
devices = i2c.scan()
if devices:
    print("I2C devices found:", [hex(device) for device in devices])
else:
    print("No I2C devices found.")

## ------- MAX30102 Configuration
# Reset
i2c.writeto_mem(0x57, 0x09, b'\x40')
time.sleep(0.1)

# Clear FIFO pointers
i2c.writeto_mem(0x57, 0x04, b'\x00')  # write pointer
i2c.writeto_mem(0x57, 0x05, b'\x00')  # overflow counter
i2c.writeto_mem(0x57, 0x06, b'\x00')  # read pointer

# FIFO config (no averaging)
i2c.writeto_mem(0x57, 0x08, b'\x0f')

# SpO2 config (100Hz, 18-bit)
i2c.writeto_mem(0x57, 0x0A, b'\x27')

# LED current
i2c.writeto_mem(0x57, 0x0C, b'\x07')  # IR 25.4 mA
i2c.writeto_mem(0x57, 0x0D, b'\x07')  # RED 25.4 mA

# Mode: SpO2 mode
i2c.writeto_mem(0x57, 0x09, b'\x03')

## ------- END OF CONFIGURATION
def samples_to_read(i2c: I2C):
    # Read FIFO pointers
    write_ptr = i2c.readfrom_mem(0x57, 0x04, 1)[0]
    read_ptr = i2c.readfrom_mem(0x57, 0x06, 1)[0]
    if write_ptr == read_ptr:
        return 0 
    if (write_ptr > read_ptr):
        return write_ptr - read_ptr
    return (32 - read_ptr) + write_ptr

gsr_adc = ADC(Pin(27))


while True:
    try:
        # Read 6 bytes (3 samples) from FIFO
        num_samples = samples_to_read(i2c)
        if num_samples > 0:
            for i in range(num_samples):
                # 1st byte will be the first byte of the ir value, ir value has 3 bytes
                # 4th 5th and 6th bytes will be the red value.
                data = i2c.readfrom_mem(0x57, 0x07, 6)
                # 01110000 00000000 00000000
                #          10101010 00000000
                #                   01010101
                ir = (data[0] << 16) | (data[1] << 8) | data[2] # 24 bit value for IR and RED
                red = (data[3] << 16) | (data[4] << 8) | data[5]
                t = time.ticks_ms()
                gsr_value = gsr_adc.read_u16()
                # gsr_value is a measure of resistance
                # so gsr_value will decrease as skin conductance increases
                print(f">IR:{ir}:{t}")
                print(f">RED:{red}:{t}")
                print(f">GSR:{gsr_value}:{t}")
        
    except Exception as e:
        print("Error reading from I2C device:", e)
    time.sleep(0.01)
