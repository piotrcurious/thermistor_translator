import csv
import sys
import os

def generate_svg(csv_path, svg_path, title, x_col, y_col, x_label, y_label):
    data = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                x = float(row[x_col])
                y = float(row[y_col])
                # Filter out error values from TargetSystem
                if y > 500 or y < -200: continue
                data.append((x, y))
            except (ValueError, KeyError):
                continue

    if not data:
        return

    min_x = min(row[0] for row in data)
    max_x = max(row[0] for row in data)
    min_y = min(row[1] for row in data)
    max_y = max(row[1] for row in data)

    # Add some padding to ranges
    dx = max_x - min_x if max_x != min_x else 1
    dy = max_y - min_y if max_y != min_y else 1

    width = 600
    height = 400
    padding = 60

    def scale_x(x):
        return padding + (x - min_x) * (width - 2 * padding) / dx

    def scale_y(y):
        return height - padding - (y - min_y) * (height - 2 * padding) / dy

    with open(svg_path, 'w') as f:
        f.write(f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">\n')
        f.write(f'<rect width="100%" height="100%" fill="#f9f9f9" />\n')
        f.write(f'<text x="{width/2}" y="30" text-anchor="middle" font-family="Arial" font-size="18" font-weight="bold">{title}</text>\n')

        # Grid lines
        for i in range(5):
            val_y = min_y + i * dy / 4
            sy = scale_y(val_y)
            f.write(f'<line x1="{padding}" y1="{sy}" x2="{width-padding}" y2="{sy}" stroke="#ddd" stroke-dasharray="4" />\n')
            f.write(f'<text x="{padding-10}" y="{sy+5}" text-anchor="end" font-family="Arial" font-size="10">{val_y:.1f}</text>\n')

        for i in range(5):
            val_x = min_x + i * dx / 4
            sx = scale_x(val_x)
            f.write(f'<line x1="{sx}" y1="{padding}" x2="{sx}" y2="{height-padding}" stroke="#ddd" stroke-dasharray="4" />\n')
            f.write(f'<text x="{sx}" y="{height-padding+20}" text-anchor="middle" font-family="Arial" font-size="10">{val_x:.1f}</text>\n')

        # Axes
        f.write(f'<line x1="{padding}" y1="{height-padding}" x2="{width-padding}" y2="{height-padding}" stroke="black" stroke-width="2" />\n')
        f.write(f'<line x1="{padding}" y1="{padding}" x2="{padding}" y2="{height-padding}" stroke="black" stroke-width="2" />\n')

        # Data points
        points = " ".join([f"{scale_x(x)},{scale_y(y)}" for x, y in data])
        f.write(f'<polyline points="{points}" fill="none" stroke="red" stroke-width="2" />\n')

        # Labels
        f.write(f'<text x="{width/2}" y="{height-10}" text-anchor="middle" font-family="Arial" font-size="12">{x_label}</text>\n')
        f.write(f'<text x="20" y="{height/2}" text-anchor="middle" font-family="Arial" font-size="12" transform="rotate(-90 20,{height/2})">{y_label}</text>\n')

        f.write('</svg>')

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 plot_results.py input.csv output_prefix")
    else:
        csv_path = sys.argv[1]
        prefix = sys.argv[2]
        title_base = os.path.basename(csv_path).replace('.csv', '').replace('data_', '').upper()

        # 1. Translation Accuracy: Input Temp vs Output Temp
        generate_svg(csv_path, f"{prefix}_accuracy.svg", f"{title_base}: Translation Accuracy",
                     'in_temp', 'out_temp', "Input Temperature (C)", "Interpreted Output Temperature (C)")

        # 2. Transfer Curve: ADC vs PWM
        generate_svg(csv_path, f"{prefix}_transfer.svg", f"{title_base}: Transfer Curve",
                     'adc', 'pwm', "ADC Input (Raw)", "PWM Output (Duty)")

        # 3. Voltage Curve: Input Temp vs Output Voltage
        generate_svg(csv_path, f"{prefix}_voltage.svg", f"{title_base}: Voltage Response",
                     'in_temp', 'out_volt', "Input Temperature (C)", "Filtered Output Voltage (V)")
