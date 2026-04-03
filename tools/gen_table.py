import math
import argparse

def ntc_resistance(temp_c, r0, t0_c, b):
    t0_k = t0_c + 273.15
    t_k = temp_c + 273.15
    return r0 * math.exp(b * (1.0/t_k - 1.0/t0_k))

def inverse_ntc_temp(r_ntc, r0, t0_c, b):
    t0_k = t0_c + 273.15
    inv_t = (1.0 / t0_k) + (1.0 / b) * math.log(r_ntc / r0)
    return (1.0 / inv_t) - 273.15

def voltage_divider(r_var, r_fixed, v_in):
    return v_in * r_var / (r_var + r_fixed)

def inverse_voltage_divider(v_out, r_fixed, v_in):
    if v_out >= v_in: return 1e9
    if v_out <= 0: return 0
    return r_fixed * v_out / (v_in - v_out)

def main():
    parser = argparse.ArgumentParser(description="Advanced Arduino PROGMEM lookup table generator for thermistor translation.")
    # Source Thermistor (Input)
    parser.add_argument("--sr0", type=float, default=10000, help="Source R0 (default: 10000)")
    parser.add_argument("--st0", type=float, default=25, help="Source T0 (default: 25)")
    parser.add_argument("--sb", type=float, default=3950, help="Source B (default: 3950)")
    parser.add_argument("--srp", type=float, default=10000, help="Source Pull-up (default: 10000)")

    # Target Thermistor (Output)
    parser.add_argument("--tr0", type=float, default=10000, help="Target R0 (default: 10000)")
    parser.add_argument("--tt0", type=float, default=25, help="Target T0 (default: 25)")
    parser.add_argument("--tb", type=float, default=3950, help="Target B (default: 3950)")
    parser.add_argument("--trp", type=float, default=10000, help="Target Pull-up (default: 10000)")

    # System Params
    parser.add_argument("--vin", type=float, default=5.0, help="System Voltage (default: 5.0)")
    parser.add_argument("--points", type=int, default=16, help="Table points (default: 16)")
    parser.add_argument("--tmin", type=float, default=-20, help="Min Temp C (default: -20)")
    parser.add_argument("--tmax", type=float, default=120, help="Max Temp C (default: 120)")
    parser.add_argument("--pwm-res", type=int, default=255, help="PWM Resolution (default: 255)")

    # RC Filter Params (for info only in this tool)
    parser.add_argument("--rc-r", type=float, default=10000, help="RC Filter R Ohms")
    parser.add_argument("--rc-c", type=float, default=10e-6, help="RC Filter C Farads")
    parser.add_argument("--fpwm", type=float, default=490, help="PWM Freq Hz")

    args = parser.parse_args()

    print(f"/*")
    print(f" * Generated Table for Thermistor Translation")
    print(f" * Source: R0={args.sr0}, B={args.sb}, Rp={args.srp}")
    print(f" * Target: R0={args.tr0}, B={args.tb}, Rp={args.trp}")
    print(f" * RC Filter Info: R={args.rc_r}, C={args.rc_c}, Fpwm={args.fpwm}")
    print(f" */")

    adc_vals = []
    pwm_vals = []

    for i in range(args.points):
        # We sweep temperature
        t = args.tmin + (args.tmax - args.tmin) * i / (args.points - 1)

        # 1. Calculate Source ADC value for this temperature
        r_src = ntc_resistance(t, args.sr0, args.st0, args.sb)
        v_src = voltage_divider(r_src, args.srp, args.vin)
        adc = int(v_src / args.vin * 1023 + 0.5)

        # 2. We want the target system to see the same temperature 't'
        #    Calculate what resistance the target system should see
        r_tgt_needed = ntc_resistance(t, args.tr0, args.tt0, args.tb)

        # 3. Calculate what voltage the target system expects for that resistance
        v_tgt_expected = voltage_divider(r_tgt_needed, args.trp, args.vin)

        # 4. Calculate the PWM duty cycle needed to produce that voltage (after RC filter)
        #    Assuming Vout_filtered = Vref * DutyCycle
        duty = v_tgt_expected / args.vin
        pwm = int(duty * args.pwm_res + 0.5)
        pwm = max(0, min(args.pwm_res, pwm))

        adc_vals.append(adc)
        pwm_vals.append(pwm)

    # Ensure ADC values are sorted ascending for the firmware search logic
    combined = sorted(zip(adc_vals, pwm_vals))
    adc_vals, pwm_vals = zip(*combined)

    print(f"const uint16_t adcPoints[] PROGMEM = {{")
    print("  " + ", ".join(map(str, adc_vals)))
    print("};")
    print(f"const uint{8 if args.pwm_res <= 255 else 16}_t pwmPoints[] PROGMEM = {{")
    print("  " + ", ".join(map(str, pwm_vals)))
    print("};")

    # Ripple calculation at 50% duty
    ripple = (args.vin * 0.5 * 0.5) / (args.fpwm * args.rc_r * args.rc_c)
    print(f"// Predicted Max Ripple: {ripple*1000:.2f} mV")

if __name__ == "__main__":
    main()
