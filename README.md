# Uri - Reverse Balance Game with UR5e
This repository provides a foundation for controlling the UR5e robot, named "Uri," in playing the Reverse Balance Game. Uri's objective in the game is to carefully remove all pawns from a board, one by one, without causing the board to tip over.
## Project Overview
- **Robot:** UR5e - a collaborative robot arm from Universal Robots
- **Game:** Reverse Balance Game - Uri attempts to clear a set of pawns from a game board without flipping it over.
- **Vision:** Utilizes the ZED 2 camera for real-time board visualization used normal and angles calculation.

## Installation
Follow these steps to set up the project environment.
1. **Clone the Repository**
```bash
git clone https://github.com/alonzTau/ReverseBalanceGame.git
cd reverse-balance-game
```
2. **Install Dependencies:** Make sure to install all required libraries and packages. Refer to requirements.txt for a list of dependencies.

## Usage
1. **Set up** the board and pawns in the lab
2. **Start the Camera:** Initialize the ZED 2 camera for board visualization.
3. **Run the Game Script:** Execute the main script to start the game.
4. **Enter the locations:** enter the pawn locations in the GUI robot will use vision feedback to remove pawns without flipping the board.
