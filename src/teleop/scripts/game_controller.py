from __future__ import print_function, division
import evdev
import threading
import math
from time import sleep
import yaml

class GameController(object):
    def __init__(self, controller_name, code_map_path, **kw):

        self.config = dict(
            joystick_min = -1.0,
            joystick_max = 1.0,
            joystick_deadzone = 0.05,
            trigger_min = 0.0,
            trigger_max = 1.0,
            trigger_deadzone = 0.0
        )

        assert(set(kw).issubset(set(self.config)))
        self.config.update(kw)

        try:
            self.device = list(filter(
                lambda d: d.name == controller_name,
                [evdev.InputDevice(path) for path in evdev.list_devices()]
            ))[0]
        except IndexError as e:
            raise ValueError(
                "Unable to find '{}'. Make sure the controller is paired."
                .format(controller_name)
            )
        
        with open(code_map_path, 'r') as f:
            self.code_map = dict(yaml.safe_load(f))
        
        for item in self.code_map.values():
            self.__dict__[item['name']] = 0
        
        self._monitor_thread = threading.Thread(
            target=self._monitor_controller, args=()
        )
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def __getitem__(self, key):
        return self.__dict__[key]


    def read(self): # return the buttons/triggers that you care about in this methode
        x = self.joystick_x_left
        y = self.joystick_y_left
        a = self.a
        b = self.b
        rb = self['trigger_right']
        return [x, y, a, b, rb]


    def _monitor_controller(self):
        while True:
            for event in self.device.read_loop():
                self._process_event(event)
    

    def _process_event(self, event):
        evcode = str(event.code)
        if evcode not in self.code_map:
            return
        if event.type == 0: # sync event
            return
        
        code_item = self.code_map[evcode]
        evtype = code_item['type']
        evname = code_item['name']

        if evtype == 'ABS':
            evmin, evmax = code_item['min'], code_item['max']
            if evname == 'trigger_left' or evname == 'trigger_right':
                # trigger
                evvalue = self._check_deadzone(event, 'trigger')
                value = _linear_map(
                    evvalue,
                    evmin,
                    evmax,
                    self.config['trigger_min'],
                    self.config['trigger_max']
                )
            else:
                # joystick or equivalent
                evvalue = self._check_deadzone(event, 'joystick')
                value = _linear_map(
                    evvalue,
                    evmin,
                    evmax,
                    self.config['joystick_min'],
                    self.config['joystick_max']
                )
            self.__dict__[evname] = value
        else:
            self.__dict__[evname] = int(event.value)

    def _check_deadzone(self, event, abs_type):
        evvalue = event.value
        code = str(event.code)
        evmin, evmax = self.code_map[code]['min'], self.code_map[code]['max']
        span = evmax - evmin
        if abs_type == 'trigger':
            zero_point = evmin
            deadzone = self.config['trigger_deadzone']
        elif abs_type == 'joystick':
            zero_point = (evmax - evmin) / 2
            deadzone = self.config['joystick_deadzone']
        
        if abs(evvalue - zero_point) < abs(span * deadzone):
            return zero_point

        return event.value

def _linear_map(x, in_min, in_max, out_min, out_max):
    return ((x - in_min) / (in_max - in_min)) * (out_max - out_min) + out_min
