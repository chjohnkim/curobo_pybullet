from pynput import keyboard

class KeyboardHandler:
    def __init__(self):
        self.action = None
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        self.commands = {
            'c': 'capture',
            'q': 'quit',
            'f': 'freedrive',
            's': 'stop_freedrive',
        }

    def on_press(self, key):
        try:
            if key.char in self.commands:
                self.action = self.commands[key.char]
        except AttributeError:
            pass  # Handle special keys if needed
    
    def stop(self):
        self.listener.stop()

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
        if exc_type:
            print(f"An exception occurred: {exc_value}")