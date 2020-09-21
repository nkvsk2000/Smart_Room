# Create a CommandSet for your remote control
# GPIO for the IR receiver: 23
# GPIO for the IR transmitter: 22
from ircodec.command import CommandSet
# controller = CommandSet(emitter_gpio=16, receiver_gpio=20, description='AC Remote for RPi', name='AC Remote')

controller = CommandSet.load('ac.json')

while True:
    print("Enter input name")
    s = input()
    if s == "end":
        break

    # Add the volume up key
    # controller.add(s)
    # Connected to pigpio
    # Detecting IR command...
    # Received.

    # print("Ready IR receiver")
    # input()

    # Send the volume up command
    controller.emit(s)

    # Remove the volume up command
    # controller.remove('volume_up')

    # Examine the contents of the CommandSet
    # controller
    # CommandSet(emitter=22, receiver=23, description="TV remote")
    # {}

    # Save to JSON
    # controller.save_as('temp.json')

# Load from JSON
# new_controller = CommandSet.load('tv.json')
