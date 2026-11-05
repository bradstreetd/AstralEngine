import numpy as np
from frames import FrameTree

class Trigger:
    def __init__(self, trigger_type: str = None, terminal: bool = True, direction: int = 0):
        self.trigger_type = trigger_type
        self.terminal = terminal
        self.direction = direction

    def evaluate(self, time, state, mu):
        pass

    def __call__(self, time, state, mu):
        return self.evaluate(time, state, mu)

class TimeTrigger(Trigger):
    def __init__(self, target_time: float, terminal: bool = True, direction: int = 0):
        super().__init__(trigger_type="time", terminal=terminal, direction=direction)
        self.target_time = target_time

    def evaluate(self, time, state, mu):
        return time - self.target_time

class AngleTrigger(Trigger):
    def __init__(self, target_angle: float, angle_type: str = "true_anomaly", terminal: bool = True, direction: int = 0):
        super().__init__(trigger_type=angle_type, terminal=terminal, direction=direction)
        self.target_angle = target_angle
        self.angle_type = angle_type

    def evaluate(self, time, state, mu):
        pass


class Event:
    def __init__(self, name: str, trigger: Trigger):
        self.name = name
        self.trigger = trigger
        self.is_active = True

    def execute(self, time, state, **kwargs):
        pass


class Maneuver(Event):
    def __init__(self, name: str, trigger: Trigger, delta_v=None, frame: str = "rth"):
        super().__init__(name, trigger)
        self.delta_v = delta_v
        self.frame = frame

    # Needs to pass in frame tree and/or spacecraft itself?
    def execute(self, time, state, ft: FrameTree, sc=None, **kwargs):
        # Take the current velocity
        r_eci = state[:3]
        v_eci = state[3:6]

        if self.frame in ft.frames:
            actual_frame = self.frame
        else:
            if sc.local_frames[self.frame] in ft.frames:
                actual_frame = sc.local_frames[self.frame]
            else:
                raise ValueError(f"Can't find {self.frame} in frame tree")

        # Update the frame tree
        sc.update_FrameTree(ft, actual_frame, state, time)

        # Convert delta_v into inertial frame
        dv_eci = ft.transform(self.delta_v, actual_frame, 'eci')

        print(f'Applied {np.linalg.norm(dv_eci)} Delta V to {sc.name}')

        v_eci_new = v_eci + dv_eci

        return np.concatenate((r_eci, v_eci_new))

class EventScheduler:
    def __init__(self):
        self.events = []

    def add_event(self, event: Event):
        self.events.append(event)

    def get_triggers(self):
        return [event.trigger for event in self.events if event.is_active]

    def clear(self):
        self.events.clear()