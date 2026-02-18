"""
    A manual control script for the Hexapod
"""

from controller import Hexapod
import sys

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
bot = Hexapod(port, step_time=0.25)
bot.stand()
while True:
    print("directions: forward, backward, left, right. Steps (a positive int), amplitude (10-100)")
    command = input("Type command in format direction, steps, amplitude\n")
    try:
        message = command.split(',')
        direction = message[0]
        # default amplitude/steps
        steps = 2
        amplitude = 60
        if (len(message) > 1):
            steps = int(message[1])
        if (len(message) > 2):
            amplitude = int(message[2])

        if (direction == "forward"):
            bot.forward(amplitude=amplitude, steps=steps)
        if (direction == "backward"):
            bot.backward(amplitude=amplitude, steps=steps)
        if (direction == "left"):
            bot.turn_left(amplitude=amplitude, steps=steps)
        if (direction == "right"):
            bot.turn_right(amplitude=amplitude, steps=steps)
    except Exception as error:
        print(f"Error with command {command} has error {error}")
