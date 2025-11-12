from machine import ADC, Pin
import time


ADC_PIN_NUMBER = 26      # Configure ADC on GP26 (ADC0)
SAMPLE_INTERVAL_SECONDS = 0.5
ADC_MAX_READING = 65535  
VREF_VOLTS = 3.3         # Pico reference voltage

# Calibration: distance (cm) = voltage (V) * CM_PER_VOLT
CM_PER_VOLT = 100.0


def read_distance_cm(distance_adc: ADC) -> float:
    raw = distance_adc.read_u16()
    voltage = (raw / ADC_MAX_READING) * VREF_VOLTS
    distance_cm = voltage * CM_PER_VOLT
    return distance_cm


def main():
    distance_adc = ADC(Pin(ADC_PIN_NUMBER))
    print("URM09 analog distance reader (every 0.5s).")
    while True:
        try:
            distance = read_distance_cm(distance_adc)
            print("Distance: {:.1f} cm".format(distance))
            time.sleep(SAMPLE_INTERVAL_SECONDS)
        except KeyboardInterrupt:
            print("\nStopping distance reader.")
            break


if __name__ == "__main__":
    main()

