import tkinter as tk
import math
import controluri

# Parameters for hexagon layout
BUTTON_SIZE = 50
HEX_HEIGHT = math.sqrt(3) * BUTTON_SIZE / 2 
HEX_WIDTH = BUTTON_SIZE 
PADDING = 10 

# List to store clicked hexagons
clicked_hexagons = []


def on_hex_click(button, i, j):
    """event of clicking on button in GUI

    Args:
        button (tk.Button): button that was clicked
        i (int): row in the grid
        j (int): collumn in the grid
    """
    if (i, j) not in clicked_hexagons:
        clicked_hexagons.append((i, j))
        print(f"Hexagon clicked at position: ({i}, {j}), added to list")
        button.config(bg="green")
    else:
        clicked_hexagons.remove((i, j))
        print(f"Hexagon at position: ({i}, {j}) reverted, removed from list")

        button.config(bg="lightblue")


def start_game():
    """
    called when pressing start game. starts the game
    """
    global clicked_hexagons,hexagons
    print("Game started!")

    controluri.play_game(clicked_hexagons)  
    for i,j in clicked_hexagons:
        button = hexagons[i][j]
        button.config(bg="lightblue")
    clicked_hexagons = []
root = tk.Tk()
root.title("Hexagon Grid")

start_button = tk.Button(root, text="Start Game", command=start_game)
start_button.pack(pady=10)


hexagons = []
for i in range(7):
    hex_row = []
    k = abs(3 - i)
    for j in range(7 - k):  # Ensure 7 hexagons in the middle row and tapering toward the edges
        x_offset = k * HEX_WIDTH / 2  # Offset even rows by half a hexagon width
        x = (j + k) * (HEX_WIDTH + PADDING) - x_offset
        y = i * (HEX_HEIGHT + PADDING) + 50  # Shift hexagons down to make space for the Start button
        
        hex_button = tk.Button(root, text=f"({i},{j})", bg="lightblue", width=6)
        hex_button.place(x=x, y=y, width=BUTTON_SIZE, height=BUTTON_SIZE)
        
        # Bind the button click event to toggle color and store clicked hexagon
        hex_button.config(command=lambda button=hex_button, i=i, j=j: on_hex_click(button, i, j))
        
        hex_row.append(hex_button)
    hexagons.append(hex_row)

# Set the window size and run the main event loop
root.geometry("430x500")
root.mainloop()
