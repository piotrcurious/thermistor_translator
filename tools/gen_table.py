import math
import argparse

def ntc_resistance(temp_c, r0, t0_c, b):
    t0_k = t0_c + 273.15
    t_k = temp_c + 273.15
    return r0 * math.exp(b * (1.0/t_k - 1.0/t0_k))

def voltage_divider(r_ntc, r_pullup, v_in):
    return v_in * r_ntc / (r_ntc + r_pullup)

def adc_value(v_out, v_ref, resolution=1024):
    return int(v_out / v_ref * resolution + 0.5)

def main():
    parser = argparse.ArgumentParser(description="Generate Arduino PROGMEM lookup table for NTC thermistor.")
    parser.add_argument("--r0", type=float, default=10000, help="Resistance at T0 (default: 10000)")
    parser.add_argument("--t0", type=float, default=25, help="T0 in Celsius (default: 25)")
    parser.add_argument("--b", type=float, default=3950, help="B parameter (default: 3950)")
    parser.add_argument("--rp", type=float, default=10000, help="Pull-up resistor (default: 10000)")
    parser.add_argument("--vin", type=float, default=5.0, help="Input voltage (default: 5.0)")
    parser.add_argument("--vref", type=float, default=5.0, help="Reference voltage (default: 5.0)")
    parser.add_argument("--points", type=int, default=10, help="Number of table points (default: 10)")
    parser.add_argument("--tmin", type=float, default=0, help="Min temperature (default: 0)")
    parser.add_argument("--tmax", type=float, default=100, help="Max temperature (default: 100)")
    parser.add_argument("--pwm-max", type=int, default=255, help="Max PWM value (default: 255)")

    args = parser.parse_args()

    print(f"// Table for R0={args.r0}, B={args.b}, Rp={args.rp}")
    print(f"const uint16_t adcPoints[] PROGMEM = {{")

    adc_vals = []
    pwm_vals = []

    for i in range(args.points):
        t = args.tmin + (args.tmax - args.tmin) * i / (args.points - 1)
        r = ntc_resistance(t, args.r0, args.t0, args.b)
        v = voltage_divider(r, args.rp, args.vin)
        adc = adc_value(v, args.vref)
        # For simplicity, we'll map temperature linearly to PWM for this tool's default
        # In a real scenario, you'd have a target curve.
        pwm = int(i * args.pwm_max / (args.points - 1))

        adc_vals.append(adc)
        pwm_vals.append(pwm)

    # Tables in Arduino usually expect sorted ADC values.
    # NTC resistance decreases with T, so Vout decreases with T, so ADC decreases with T.
    # We should reverse if needed.
    if adc_vals[0] > adc_vals[-1]:
        adc_vals.reverse()
        pwm_vals.reverse()

    print("  " + ", ".join(map(str, adc_vals)))
    print("};")
    print(f"const uint8_t pwmPoints[] PROGMEM = {{")
    print("  " + ", ".join(map(str, pwm_vals)))
    print("};")

if __name__ == "__main__":
    main()
