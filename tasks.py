import subprocess
import os

def test_first_question():
    
    # Set the PYTHONPATH environment variable
    env = os.environ.copy()  # Copy the current environment
    env["PYTHONPATH"] = "projeto_robotica"  # Set the new PYTHONPATH

    # Command to run the test script
    command = ["python3", "tests/first_question.py"]

    # Execute the command with the modified environment
    result = subprocess.run(command, capture_output=True, text=True, env=env)

    # Print the output
    if result.returncode == 0:
        print(result.stdout)
    else:
        print("Error:", result.stderr)

def test_second_question():
    
    # Set the PYTHONPATH environment variable
    env = os.environ.copy()  # Copy the current environment
    env["PYTHONPATH"] = "projeto_robotica"  # Set the new PYTHONPATH

    # Command to run the test script
    command = ["python3", "tests/second_question.py"]

    # Execute the command with the modified environment
    result = subprocess.run(command, capture_output=True, text=True, env=env)

    # Print the output
    if result.returncode == 0:
        print(result.stdout)
    else:
        print("Error:", result.stderr)