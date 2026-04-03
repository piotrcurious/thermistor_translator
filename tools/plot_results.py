import csv
import sys

def generate_svg(csv_path, svg_path, title):
    data = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append((float(row['adc']), float(row['pwm'])))

    if not data:
        return

    # Sort by ADC to draw a proper line
    data.sort()

    min_x = 0
    max_x = 1023
    min_y = 0
    max_y = max(row[1] for row in data)
    if max_y < 255: max_y = 255

    width = 500
    height = 300
    padding = 40

    def scale_x(x):
        return padding + (x - min_x) * (width - 2 * padding) / (max_x - min_x)

    def scale_y(y):
        return height - padding - (y - min_y) * (height - 2 * padding) / (max_y - min_y)

    with open(svg_path, 'w') as f:
        f.write(f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">\n')
        f.write(f'<rect width="100%" height="100%" fill="white" />\n')
        f.write(f'<text x="{width/2}" y="20" text-anchor="middle" font-family="Arial" font-size="16">{title}</text>\n')

        # Axes
        f.write(f'<line x1="{padding}" y1="{height-padding}" x2="{width-padding}" y2="{height-padding}" stroke="black" />\n')
        f.write(f'<line x1="{padding}" y1="{padding}" x2="{padding}" y2="{height-padding}" stroke="black" />\n')

        # Data points
        points = " ".join([f"{scale_x(x)},{scale_y(y)}" for x, y in data])
        f.write(f'<polyline points="{points}" fill="none" stroke="blue" stroke-width="2" />\n')

        # Labels
        f.write(f'<text x="{padding}" y="{height-padding+20}" text-anchor="middle" font-family="Arial" font-size="12">0</text>\n')
        f.write(f'<text x="{width-padding}" y="{height-padding+20}" text-anchor="middle" font-family="Arial" font-size="12">1023</text>\n')
        f.write(f'<text x="{width/2}" y="{height-padding+35}" text-anchor="middle" font-family="Arial" font-size="12">ADC Input</text>\n')

        f.write(f'<text x="{padding-10}" y="{height-padding}" text-anchor="end" font-family="Arial" font-size="12">0</text>\n')
        f.write(f'<text x="{padding-10}" y="{padding}" text-anchor="end" font-family="Arial" font-size="12">{int(max_y)}</text>\n')
        f.write(f'<text x="15" y="{height/2}" text-anchor="middle" font-family="Arial" font-size="12" transform="rotate(-90 15,{height/2})">PWM Output</text>\n')

        f.write('</svg>')

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python3 plot_results.py input.csv output.svg title")
    else:
        generate_svg(sys.argv[1], sys.argv[2], sys.argv[3])
