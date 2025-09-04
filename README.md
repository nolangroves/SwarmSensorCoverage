# Swarm Robot Simulation

An implementation of a multi-robot sensor coverage algorithm, based on the paper [&#34;Distributed Control of Multiple Robots for Sensor Coverage&#34; (Springer, 2009)](https://link.springer.com/chapter/10.1007/978-4-431-65941-9_30). The focus is on maximizing the area covered by the robots' sensors in a 2D environment. Robots use local sensing and distributed control to spread out and optimize coverage, demonstrating principles of swarm intelligence.


https://github.com/user-attachments/assets/f39b7bac-5e65-4f2b-a0f5-2922185b24e4


## Algorithm Overview

Robots are initialized in a bounded environment and use local sensor readings to determine their movement. The control law encourages robots to spread out, maximizing sensor coverage while avoiding obstacles and other robots. The simulation visualizes the coverage process and robot distribution over time.

## Features

- Multi-robot simulation in a 2D environment
- Robots use sensors to detect walls and other robots
- Physics-inspired control for obstacle avoidance and swarm behavior
- Environment loaded from an image file (`building.png`)
- Visualization using Pygame

## Installation

1. **Clone the repository:**

   ```powershell
   git clone <your-repo-url>
   cd Swarm-project
   ```
2. **Install dependencies:**
   This project uses [Pipenv](https://pipenv.pypa.io/en/latest/). Install Pipenv if you don't have it:

   ```powershell
   pip install pipenv
   ```

   Then install dependencies:

   ```powershell
   pipenv install
   ```

## Usage

1. Place your environment image as `building.png` in the project root.
2. Run the simulation:
   ```powershell
   pipenv run python main.py
   ```
3. The Pygame window will open, showing robots navigating the environment.

## File Structure

- `main.py`: Entry point, runs the simulation loop
- `robot.py`: Robot class and control logic
- `environment.py`: Environment setup and sensor logic
- `building.png`: Environment map (black/white image)
- `Pipfile`, `Pipfile.lock`: Dependency management

## Requirements

- Python 3.12+
- Pygame
- Matplotlib
- Numpy
