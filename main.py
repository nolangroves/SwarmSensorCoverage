import pygame
from pygame.locals import *
from environment import Environment
import matplotlib.pyplot as plt
import numpy as np


dt = 0.1
DRAW_WALL_FORCES = False
DRAW_ROB_FORCES = False
class Simulator:
    def __init__(self) -> None:
        self._running = True
        pygame.init()
        self.env = Environment("building.png", 
                               sensor_radius=100,
                               num_robots=20,
                               sensor_density=32)
        self.size = self.weight, self.height = self.env.size
        self._display_surf = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self._image_surf = self.env.base_image.convert()
        self._area_surf = pygame.Surface(self.size, pygame.SRCALPHA)
        self._area_mask = pygame.mask.from_surface(self._area_surf)
        self.area = 0
        self.t = 0
        self.i = 0
        self.t_data = []
        self.data = []
        self.surfs = []

    def handle_event(self, event: pygame.event.Event) -> None:
        if event.type == pygame.QUIT:
            self._running = False

    def loop(self) -> None:
        self.surfs = []
        self._area_mask.clear()
        for r in self.env.robots:
            vals, surf = self.env.get_sensor_readings_at(r.pos, draw=DRAW_WALL_FORCES)
            if DRAW_WALL_FORCES:
                self.surfs.append((surf, (r.pos.x - self.env.sensor_radius, r.pos.y - self.env.sensor_radius)))
            m = self.env.get_sensor_mask(r.pos)
            self._area_mask.draw(m, (0,0))
            robs, surf = self.env.get_nearby_robots(r.pos, m, draw=DRAW_ROB_FORCES)
            if DRAW_ROB_FORCES:
                self.surfs.append((surf, (r.pos.x - self.env.sensor_radius, r.pos.y - self.env.sensor_radius)))
            r.set_control(vals, robs)
        for r in self.env.robots:
            r.step(dt)

        self.area = self._area_mask.count() / 100


    def render(self) -> None:
        self._display_surf.blit(self._image_surf,(0,0))
        for s in self.surfs:
            self._display_surf.blit(s[0],s[1])

        for r in self.env.robots:
            pygame.draw.circle(self._display_surf, (0,0,255,255), r.pos, 3)
        
        self._area_mask.to_surface(self._area_surf, setcolor=(0,255,255,130), unsetcolor=(0,0,0,0))
        self._display_surf.blit(self._area_surf,(0,0))

        # a,_ = self.env.get_sensor_readings_at((210,200))
        # self._display_surf.blit(a,(210-self.env.sensor_radius,200-self.env.sensor_radius))
        pygame.display.flip()

    def plot(self):
        self.t_data.append(self.t)
        self.data.append(self.area)
        # pygame.image.save(self._display_surf, f'frames/{self.i:04}.png')
        if self.t == 0:
            pygame.image.save(self._display_surf, f'initial_{self.env.num_robots}.png')
        elif self.t >= 300:
            self._running = False
            


    def cleanup(self) -> None:
        pygame.image.save(self._display_surf, f'final_{self.env.num_robots}.png')

        plt.plot(self.t_data, self.data)
        plt.ylabel('Total coverage')
        plt.xlabel('Time')
        plt.savefig(f'area_plot.png')

    def main(self) -> None:
        
        while self._running:
            for event in pygame.event.get():
                self.handle_event(event)
            self.loop()
            self.render()
            self.plot()
            self.t += dt
            self.i += 1

        self.cleanup()
 
if __name__ == "__main__" :
    sim = Simulator()
    sim.main()