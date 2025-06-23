# Robotic Arm Project

This repository contains code for a mechanical arm robotics project.

## Description

This project focuses on developing and simulating a robotic arm with various trajectory planning and control functions. It includes one-dimensional and two-dimensional movement capabilities, as well as specific motor control functions.

## Project Structure

- `projeto_robotica/`: Main source code directory
  - `drivers/`: Contains drivers for simulation
  - `functions/`: Contains movement and trajectory planning functions
  - `main.py`: Main entry point for the application
  - `tasks.py`: Task definitions for the project

## Setup

### Prerequisites

- Python 3.7.17
- Poetry (for dependency management)

### Installation

1. Clone this repository:
   ```
   git clone https://github.com/yourusername/projeto-robotica.git
   cd projeto-robotica
   ```

2. Install dependencies using Poetry:
   ```
   poetry install
   ```

Alternatively, you can use the requirements.txt file:
```
pip install -r requirements.txt
```

### Docker Setup

The project includes Docker support:

```
docker-compose run app
```

After that, you can use the terminal to run the commands.

## Running the Project

The project includes several predefined scripts that can be run using Poetry:

```
# Run the first question test
poetry run first_question

# Run the second question test
poetry run second_question

# Start the main application
poetry run start
```

## Authors

- Ian Karlo (iankarlots@gmail.com)
- Enzo Gabriel (egb2@cin.ufpe.br)
- Ekistoclecio Heleno (ehdl@cin.ufpe.br)

## License

This project is licensed under the MIT License.


## Results

### First Part
In this part, we implemented the two functions required in the assignment:

1. `SE2_xy(x, y)`: Creates a 2D homogeneous transformation matrix for translation along x and y axes.
2. `SE2_theta(theta)`: Creates a 2D homogeneous transformation matrix for rotation around the origin.

These functions are implemented in the `projeto_robotica/functions/one_dim.py` file. The implementation uses NumPy for matrix operations and follows the mathematical formulations for 2D transformations in homogeneous coordinates.

#### Tests
The tests in `first_question.py` validate the functionality of the transformation functions using a point \( P \) located at coordinates (0.5, 0.5) in the reference frame \( R_1 \). The tests calculate the coordinates of point \( P \) in two different reference frames, \( R_1 \) and \( R_2 \), after applying translations and rotations:

1. **Test 1**: Calculates the coordinates of point \( P \) after translating it by (1, 0.25) in reference frame \( R_1 \).
2. **Test 2**: Computes the coordinates of point \( P \) in reference frame \( R_2 \) by applying the inverse of the translation.
3. **Test 3**: Determines the coordinates of point \( P \) in reference frame \( R_1 \) after applying a rotation of 45 degrees.
4. **Test 4**: Finds the coordinates of point \( P \) in reference frame \( R_2 \) after applying both translation and rotation.

These tests ensure that the transformation functions work correctly for both translations and rotations in 2D space.

### Second Part
In this section, we address the implementation of a planar robotic arm with two rotational joints. We use the functions `fx(theta1, theta2)` and `ik(x, y)` to calculate the position and orientation of the robot's end effector.

1. **Function `fx(theta1, theta2)`**:
   - **Input**: 
     - `theta1`: Angle of the first joint (rad).
     - `theta2`: Angle of the second joint (rad).
   - **Output**: 
     - `(x, y)`: Coordinates of the end effector.
     - `orientation`: Total angle (rad) of the end effector.

   This function uses direct kinematics to calculate the position of the end effector based on the angles of the joints.

2. **Function `ik(x, y)`**:
   - **Input**: 
     - `x`: x-coordinate of the end effector (m).
     - `y`: y-coordinate of the end effector (m).
   - **Output**: 
     - `(theta1, theta2)`: Angles of the joints corresponding to the point (x, y).

   The `ik` function calculates the joint angles necessary to reach a specific position of the end effector, using inverse kinematics.

#### Tests
The tests for the functions `fx` and `ik` are implemented in `second_question.py` and validate the functionality of these kinematic calculations. 

For the `fx` function, the following test cases are used:
- (θ₁ = 0, θ₂ = π/2)  
- (θ₁ = π/2, θ₂ = −π/2)  
- (θ₁ = π/2, θ₂ = π/2)  
- (θ₁ = −π, θ₂ = π)

These cases check the end effector's position and orientation for various joint angles.

For the `ik` function, the following test points are evaluated:

- ( x = 1, y = 1 )
- ( x = 1, y = -1 )
- ( x = -1, y = 1 )
- ( x = -1, y = -1 )
- ( x = 2, y = 1 )
- ( x = 2, y = 0 )
- ( x = 0, y = 2 )
- ( x = -2, y = 0 )
- ( x = -2, y = -2 )

These tests ensure that the inverse kinematics function can accurately compute the necessary joint angles to reach specified coordinates in the workspace.

These functions are implemented in the file `projeto_robotica/functions/two_dim.py` and are fundamental for the simulation and control of the robotic arm.

