import subprocess
import os

test_files = [
    "test_translator.cpp",
    "test_translator_uno_precise.cpp",
    "test_translator_v2.cpp",
    "test_translator_v3_switchable.cpp",
    "test_translator_v3_switchable2.cpp",
    "test_adc_to_pwm.cpp",
    "test_noise_robustness.cpp"
]

def run_test(test_file):
    print(f"--- Running {test_file} ---")
    binary = f"./tests/{test_file.replace('.cpp', '')}"
    compile_cmd = [
        "g++",
        "-I", "tests/mock_arduino",
        "-I", "tests/mock_arduino/avr",
        f"tests/{test_file}",
        "-o", binary
    ]

    try:
        subprocess.run(compile_cmd, check=True)
        subprocess.run([binary], check=True)
        print(f"Successfully ran {test_file}\n")
    except subprocess.CalledProcessError as e:
        print(f"Failed to run {test_file}: {e}\n")
    finally:
        if os.path.exists(binary):
            os.remove(binary)

if __name__ == "__main__":
    for test_file in test_files:
        run_test(test_file)
