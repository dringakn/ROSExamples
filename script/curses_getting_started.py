#!/usr/bin/env python3

import curses
import time
import random

class MyWindow:
    def __init__(self, window, title, height, width, y, x):
        self.window = window
        self.title = title
        self.height = height
        self.width = width
        self.y = y
        self.x = x
        self.data = []

    def update(self):
        self.window.clear()
        self.window.border(1)
        self.window.addstr(0, 2, self.title)
        for i, line in enumerate(self.data):
            self.window.addstr(i + 1, 1, line[:self.width - 2])

    def set_data(self, data):
        self.data = data

    def handle_input(self, key):
        pass  # Override this method in subclasses to handle input

class App:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        curses.curs_set(0)
        curses.start_color()
        curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLUE)
        self.stdscr.attron(curses.color_pair(1))

        self.windows = [
            MyWindow(stdscr, "Window 1", 5, 20, 1, 1),
            MyWindow(stdscr, "Window 2", 5, 20, 1, 10),
        ]

        self.running = True
        self.current_window_index = 0
        self.fill_windows_with_data()

    def fill_windows_with_data(self):
        for window in self.windows:
            data = [f"Data {i}" for i in range(1, 10)]
            window.set_data(data)

    def run(self):
        while self.running:
            self.update_windows()
            self.handle_input()
            time.sleep(0.1)

    def update_windows(self):
        for i, window in enumerate(self.windows):
            if i == self.current_window_index:
                window.window.attron(curses.A_BOLD)
            window.update()
            if i == self.current_window_index:
                window.window.attroff(curses.A_BOLD)
            window.window.refresh()

    def handle_input(self):
        key = self.stdscr.getch()
        if key == ord('q'):
            self.running = False
        elif key == curses.KEY_RIGHT:
            self.current_window_index = (self.current_window_index + 1) % len(self.windows)
        elif key == curses.KEY_LEFT:
            self.current_window_index = (self.current_window_index - 1) % len(self.windows)
        else:
            self.windows[self.current_window_index].handle_input(key)

def main(stdscr):
    app = App(stdscr)
    app.run()

if __name__ == "__main__":
    curses.wrapper(main)
