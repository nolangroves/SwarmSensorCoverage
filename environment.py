import pygame
import numpy as np
from pygame.math import Vector2 as Vec

from robot import Robot

class Environment:
    def __init__(self, image_filename:str, sensor_radius:int = 100, num_robots = 10, sensor_density=32) -> None:
        self.base_image = pygame.image.load(image_filename)
        gs_image = self.base_image.copy()
        gs_image.fill((255,255,255,255))
        self.map = gs_image.copy()
        pygame.transform.grayscale(self.base_image, gs_image)
        self.map: pygame.Mask = pygame.mask.from_threshold(gs_image, (0,0,0,255), (127,127,127,255))
        self.size = self.width, self.height = self.base_image.get_size()

        self.sensor_radius = sensor_radius
        self.sensor_density = sensor_density
        sensor_mask_surf = pygame.Surface((sensor_radius*2+1, sensor_radius*2+1))
        sensor_mask_surf.fill((0,0,0,255))
        pygame.draw.circle(sensor_mask_surf, (255,255,255,255), (sensor_radius+1, sensor_radius+1), sensor_radius)
        self.sensor_mask = pygame.mask.from_threshold(sensor_mask_surf, (0,0,0,255), (127,127,127,255))
        self.sensor_mask.invert()

        self.num_robots = num_robots
        self.robots = self.create_robots(num_robots, (740, 235), (880, 320))
        
        

    def create_robots(self, num_robots, top_corner, bottom_corner):
        robots = []
        initial_x = np.random.uniform(top_corner[0], bottom_corner[0], num_robots)
        initial_y = np.random.uniform(top_corner[1], bottom_corner[1], num_robots)

        for i in range(num_robots):
            robots.append(Robot((initial_x[i], initial_y[i]), self))

        return robots

    def get_sensor_readings_at(self, pos:tuple, draw=False):
        num_readings = self.sensor_density
        x, y = pos
        x -= self.sensor_radius
        y -= self.sensor_radius
        check = self.sensor_mask.overlap_mask(self.map, (-x,-y) )
        buf = np.array(check.to_surface().get_view()).reshape(check.get_size())
        mask = np.where(buf == 4278190080, 0,1)
        points = np.argwhere(mask) - self.sensor_radius
        t_vals = np.arctan2(points[:,1], points[:,0])
        r_vals = np.linalg.norm(points, axis=1)

        bins = np.linspace(-np.pi,np.pi, num_readings)
        digit = np.digitize(t_vals, bins)

        ret = []
        for i in range(num_readings):
            bmask = digit == i
            if not bmask.any():
                continue
            j = np.argmin(r_vals[bmask])
            val = np.zeros(3)
            val[0:2] =  points[bmask,:][j] 
            val[2] = r_vals[bmask][j]
            ret.append(val)

        surf = None
        if draw:
            surf = pygame.Surface(check.get_size(), pygame.SRCALPHA)
            surf.fill((0,0,0,0))
            for p in ret:
                p2 = (p[:2] + self.sensor_radius).astype(int)
                pygame.draw.line(surf, (255,0,0,255), (self.sensor_radius, self.sensor_radius), p2, 2)
                # surf.set_at( p2, (255,0,0,255))

        return ret, surf
    
    def get_nearby_robots(self, pos, mask, draw=False):
        ret = []
        surf = None
        # surf = pygame.Surface((self.sensor_radius*2 + 1, self.sensor_radius*2 + 1), pygame.SRCALPHA)
        # surf.fill((0,0,0,0))
        offset_pos = (pos.x -self.sensor_radius, pos.y - self.sensor_radius )
        for robot in self.robots:
            r = (robot.pos - pos)
            if r.magnitude() < self.sensor_radius and mask.get_at(pos):
                # surf.fill((0,0,0,0))
                # p = r
                # p2 = (p.elementwise() + self.sensor_radius)
                # pygame.draw.line(surf, (0,255,0,255), (self.sensor_radius, self.sensor_radius), (int(p2.x), int(p2.y)), 2)
                # test_mask = pygame.mask.from_surface(surf)
                # if test_mask.overlap(self.map, (-offset_pos[0], -offset_pos[1])) is None:
                ret.append(r)
        
        if draw:
            surf = pygame.Surface((self.sensor_radius*2 + 1, self.sensor_radius*2 + 1), pygame.SRCALPHA)
            surf.fill((0,0,0,0))
            for p in ret:
                p2 = (p.elementwise() + self.sensor_radius)
                pygame.draw.line(surf, (0,255,0,255), (self.sensor_radius, self.sensor_radius), (int(p2.x), int(p2.y)), 2)

        return ret, surf
    
    def get_sensor_mask(self, pos):
        even_surf = pygame.Surface(self.size, pygame.SRCALPHA)
        pygame.draw.circle(even_surf, (0,0,0,255), pos, 3)
        bins = np.linspace(0,360, self.sensor_density)
        odd_surf = even_surf.copy()
        surf = None

        for i in range(self.sensor_density):
            if i%2 == 0:
                surf = even_surf
            else:
                surf = odd_surf

            
            p1 = pos + Vec.from_polar((self.sensor_radius, bins[i]))
            p2 = pos + Vec.from_polar((self.sensor_radius, bins[(i+1) % len(bins)]))
            pygame.draw.polygon(surf, (0,0,0,255), [pos, p1, p2])

        odd_mask = pygame.mask.from_surface(odd_surf)
        odd_mask.erase(self.map, (0,0))
        odd_mask = odd_mask.connected_component(pos)

        even_mask = pygame.mask.from_surface(even_surf)
        even_mask.erase(self.map, (0,0))
        even_mask = even_mask.connected_component(pos)

        even_mask.draw(odd_mask, (0,0))

        return even_mask


