from __future__ import print_function, division
from evdev import InputDevice, list_devices, ecodes
import threading
import math
from time import sleep
import yaml

class GameController(object):
    def __init__(self, controller_name, code_map_path, **kw):
        """
        Initialize GameController object.

        Args:
            controller_name (str): Name of controller device to connect to.
            code_map_path (str): Path to button-code-mapping yaml file.
            **kw (optional):
                joystick_min (float): Minimum value of processed joystick.
                joystick_max (float): Maximum value of processed joystick.
                joystick_deadzone (float): Percentage of range of motion
                    that registers as zero starting from the middle.
                trigger_min (float): Minimum value of processed trigger.
                trigger_max (float): Maximum value of processed trigger.
                trigger_deadzone (float): Percentage of range of motion
                    that registers as zero starting from fully up.
                event_filter (List[str]): filtered_event_cb will only be called
                    on events in this list. If no filter and filtered_event_cb
                    is defined, it will be called for every processed event.
                filtered_event_cb (method(self, name)): Method that gets called
                    after event is processed. Only passes the name as an arg
                    because you can get the value by accessing self[name].
                raw_event_cb (method(event)): Method that gets called on every
                    event before processing.
        """

        defaults = dict(
            joystick_min = -1.0,
            joystick_max = 1.0,
            joystick_deadzone = 0.05,
            trigger_min = 0.0,
            trigger_max = 1.0,
            trigger_deadzone = 0.0,
            event_filter = None,
            filtered_event_cb = None,
            raw_event_cb = None,
        )

        assert(set(kw).issubset(set(defaults)))
        self.__dict__.update(defaults)
        self.__dict__.update(kw)

        try:
            self.device = list(filter(
                lambda d: d.name == controller_name,
                [InputDevice(path) for path in list_devices()]
            ))[0]
        except IndexError as e:
            raise ValueError(
                "Unable to find '{}'. Make sure the controller is paired."
                .format(controller_name)
            )
        
        with open(code_map_path, 'r') as f:
            tmp_map = dict(yaml.safe_load(f))
        self.code_map = dict(tmp_map)
        
        for k, v in tmp_map.items():
            if k.isdigit():
                self.code_map[int(k)] = v
            else:
                self.code_map[ecodes.ecodes[k]] = v
        
        for item in self.code_map.values():
            self.__dict__[item['name']] = 0
        
        self._monitor_thread = threading.Thread(
            target=self._monitor_controller, args=()
        )
        self._monitor_thread.daemon = True


    def begin(self):
        """
        Starts the controller monitoring thread.
        """
        self._monitor_thread.start()


    def __getitem__(self, key):
        """
        Lets the object work with string key accesses.
        """
        return self.__dict__[key]


    def _monitor_controller(self):
        while True:
            for event in self.device.read_loop():
                self._process_event(event)
    

    def _process_event(self, event):
        # invoke callback before filtering
        if self.raw_event_cb != None:
            self.raw_event_cb(event)

        if event.type == ecodes.SYN_REPORT or event.type == ecodes.EV_MSC:
            return

        evcode = event.code
        if evcode not in self.code_map:
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
                    self['trigger_min'],
                    self['trigger_max']
                )
            else:
                # joystick or equivalent
                evvalue = self._check_deadzone(event, 'joystick')
                value = _linear_map(
                    evvalue,
                    evmin,
                    evmax,
                    self['joystick_min'],
                    self['joystick_max']
                )
            self.__dict__[evname] = value
        else:
            self.__dict__[evname] = int(event.value)
        
        # call filtered_event_cb only after event is processed
        if self.filtered_event_cb != None:
            if self.event_filter == None or evname in self.event_filter:
                self.filtered_event_cb(self, evname)


    def _check_deadzone(self, event, abs_type):
        evvalue = event.value
        code = event.code
        evmin, evmax = self.code_map[code]['min'], self.code_map[code]['max']
        span = evmax - evmin
        if abs_type == 'trigger':
            zero_point = evmin
            deadzone = self['trigger_deadzone']
        elif abs_type == 'joystick':
            zero_point = (evmax - evmin) / 2
            deadzone = self['joystick_deadzone']
        
        if abs(evvalue - zero_point) < abs(span * deadzone):
            return zero_point

        return event.value

def _linear_map(x, in_min, in_max, out_min, out_max):
    return ((x - in_min) / (in_max - in_min)) * (out_max - out_min) + out_min
