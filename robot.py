import pygame
from pygame.math import Vector2 as Vec

MAX_VEL = 10
MAX_ACC = 5
class Robot:
    def __init__(self, p, env) -> None:
        self.pos = Vec(*p)
        self.vel = Vec()
        self.u = Vec()
        self.env = env
        self.sensor_density = env.sensor_density

    def step(self, dt) -> None:
        new_pos = self.pos + self.vel * dt
        rounded = (int(new_pos.x), int(new_pos.y))
        if self.env.map.get_at(rounded):
            pass
        else:
            self.pos = new_pos
        self.vel += self.u * dt
        self.vel = self.vel.clamp_magnitude(MAX_VEL)

    def set_control(self, sensor_readings, robot_readings):
        F_walls = Vec()
        for val in sensor_readings:
            rv = Vec(val[0], val[1])
            r = val[2]
            if r <= 0.00000001:
                continue
            F_walls += (1/r**3) * rv

        F_robs = Vec()
        for r in robot_readings:
            if r.magnitude() <= 0.00000001:
                continue
            F_robs += (1/r.magnitude()**3) * r
        
        ko = 20 * (32 / self.sensor_density)
        kr = 100
        v = 0.02
        m = 0.5

        F_total = (-ko * F_walls) + (-kr * F_robs)
        u = (F_total - v*self.vel)/m
        self.u = u.clamp_magnitude(MAX_ACC)