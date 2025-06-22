import subprocess
import os
import argparse

def test_first_question():
    
    env = os.environ.copy()
    env["PYTHONPATH"] = "projeto_robotica"

    command = ["python", "tests/first_question.py"]

    result = subprocess.run(command, capture_output=True, text=True, env=env)

    if result.returncode == 0:
        print(result.stdout)
    else:
        print("Error:", result.stderr)

def test_second_question():
    
    env = os.environ.copy()
    env["PYTHONPATH"] = "projeto_robotica"

    command = ["python", "tests/second_question.py"]

    result = subprocess.run(command, capture_output=True, text=True, env=env)

    if result.returncode == 0:
        print(result.stdout)
    else:
        print("Error:", result.stderr)

def start():

    # Set up argument parser
    parser = argparse.ArgumentParser(description="Start the main script.")
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    
    # Parse the arguments
    args = parser.parse_args()

    command = ["python", "projeto_robotica/main.py"]
    env = os.environ.copy()
    env["DEBUG"] = "True" if args.debug else "False"

    print(env["DEBUG"])
    print(args)

    result = subprocess.run(command, capture_output=True, text=True, env=env)

    if result.returncode == 0:
        print(result.stdout)
    else:
        print("Error:", result.stderr)
