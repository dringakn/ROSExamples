#!/usr/bin/env python3

import curses


class Window:
    def __init__(self, height, width, start_y, start_x):
        self.window = curses.newwin(height, width, start_y, start_x)

    def addstr(self, y, x, text, attr=curses.A_NORMAL):
        try:
            self.window.addstr(y, x, text, attr)
        except curses.error:
            pass

    def refresh(self):
        self.window.border(0)
        self.window.refresh()

    def getch(self):
        key = self.window.getch()
        if key != -1:
            return key
        else:
            return None

    def enable_scrolling(self):
        self.window.scrollok(True)

    def scroll(self, nlines):
        self.window.scroll(nlines)

    def keypad(self, state):
        """ Enable special keys like function keys and arrow keys """
        self.window.keypad(state)
        # self.stdscr.keypad(state)

    def echo(self, state):
        """ Enable/Disable automatic echoing of keys to the screen """
        curses.echo(state)

    def enter(self, state):
        """ React to keys instantly (no Enter key needed) """
        curses.cbreak(state)

    def enable_non_blocking(self):
        """ Set the timeout to 0 to make getch non-blocking """
        self.timeout(0)

    def timeout(self, milliseconds=0):
        """ Set the timeout in milliseconds, 0 to make getch non-blocking """
        # self.stdscr.timeout(milliseconds)
        self.window.timeout(milliseconds)

    def showtext(self, text, attr=curses.A_NORMAL):
        self.window.clear()
        self.addstr(1, 1, text, attr)
        self.refresh()

class MessageListWindow(Window):
    def __init__(self, height, width, start_y, start_x):
        super().__init__(height, width, start_y, start_x)
        self.messages = []
        self.enable_scrolling()

    def get_messages(self):
        return self.messages

    def add_message(self, message):
        self.messages.append(message)
        self.draw_messages()

    def clear_message(self):
        self.messages.clear()
        self.draw_messages()

    def draw_messages(self):
        self.window.clear()
        for i, message in enumerate(self.messages):
            self.addstr(i, 0, message)
        self.refresh()

class InputWindow(Window):
    def __init__(self, height, width, start_y, start_x):
        super().__init__(height, width, start_y, start_x)
        self.input_text = ""

    def get_input(self):
        return self.input_text

    def draw_input(self):
        self.window.clear()
        self.addstr(0, 0, "Enter a message:")
        self.addstr(1, 0, self.input_text)
        self.refresh()

    def handle_input(self, key):
        if key == 10:  # Enter key
            return True
        elif key == 127:  # Backspace key
            self.input_text = self.input_text[:-1]
        elif key >= 32 and key < 127:  # ASCII printable characters
            self.input_text += chr(key)
        self.draw_input()
        return False

class NonBlockingInputWindow(Window):
    def __init__(self, height, width, start_y, start_x):
        super().__init__(height, width, start_y, start_x)
        self.enable_non_blocking()
        self.keypad(True)  # Enable special keys like function keys and arrow keys
        self.echo(False)  # Disable automatic echoing of keys to the screen
        self.enter(True)  # React to keys instantly (no Enter key needed)


def main(stdscr):
    curses.curs_set(0)
    stdscr.clear()

    status_win = Window(3, curses.COLS, 0, 0)
    status2_win = Window(7, curses.COLS, 3, 0)
    messagelist_win = MessageListWindow(10, curses.COLS, 10, 0)
    input_win = NonBlockingInputWindow(1, curses.COLS, 20, 0)
    # input_win = InputWindow(3, curses.COLS, 10, 0)

    ctr = 0
    while True:

        key = input_win.getch()
        if key == ord('q'):
            exit(0)
        elif key == ord('a'):
            status_win.showtext(f"hello world: {ctr}")
            ctr+=1
        elif key == ord('s'):
            status2_win.showtext(f"hello world 2"*100)
            ctr+=1
        elif key is not None:
            messagelist_win.add_message(str(key))


if __name__ == "__main__":
    curses.wrapper(main)
