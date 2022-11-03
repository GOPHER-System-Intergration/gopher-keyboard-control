#! /usr/bin/env python

from __future__ import print_function

# import rospy
from pynput import mouse
# from geometry_msgs.msg import Twist
from screeninfo import get_monitors




class MouseControl():

    def __init__(self):
        self.screen_center_x = 0
        self.screen_center_y = 0
        self.update_screen_center()
    
    def on_move(self, x_curser_position, y_curser_position):
        x_pos = x_curser_position - self.screen_center_x
        y_pos = y_curser_position - self.screen_center_y

        print('Point moved to {0}'.format((x_pos, y_pos)))

        # print('Point moved to {0}'.format((x_curser_position, y_curser_position)))

    def on_click(self, x, y, button, pressed):
        print('{0} at {1}'.format('Pressed' if pressed else 'Released', (x, y)))

    def on_scroll(self, x, y, dx, dy):
        print('scrolled {0}'.format((x, y)))

    def update_screen_center(self):
        # gets the center of the screen
        # assumes that there is only one screen.
        #   if there are more screens, we will look only at the resolution of the "first" identified screen

        first_monitor = get_monitors()[0]
        self.screen_center_x = int(first_monitor.width/2.0)
        self.screen_center_y = int(first_monitor.height/2.0)

        

# Lets just print out the position of the mouse position

if __name__ == '__main__':
    
    computer_mouse = MouseControl()

    mouse_listener = mouse.Listener(
        on_move = computer_mouse.on_move, 
        on_click= None, 
        on_scroll= None)
    mouse_listener.start()

    while mouse_listener.is_alive():
        None