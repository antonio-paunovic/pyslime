import pygame, sys
from pygame.locals import *

import numpy as np

DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 600
WHITE=(255,255,255)
BLUE=(0,0,255)
YELLOW=(125,0,0)
BLACK=(0,0,0)

pygame.init()
DISPLAY= pygame.display.set_mode((DISPLAY_WIDTH, DISPLAY_HEIGHT))
pygame.display.set_caption('Hello World!')

class PhisicalEntity:

    dt = 0.005
    g = np.array([0, 9.81])

    def gravity(e):
        e.v += PhisicalEntity.g * PhisicalEntity.dt

    def friction(e):
        if np.isclose(e.v[0], 0): return
        frictionValue = 5. if e.v[0] < 0 else -5.
        friction = np.array([frictionValue, 0.])
        e.v += friction * PhisicalEntity.dt

    def __init__(self, x, y, vx, vy):
        self.s = np.array([x,y], dtype='float64')
        self.v = np.array([vx,vy], dtype='float64')
        self.forces = set()

    def __str__(self):
        return 's={}\nv={}'.format(self.s, self.v)

    def move(self, v):
        print("Moving:", self)
        self.v += v

    def update(self):
        for forceFn in self.forces:
            forceFn()
        self.s += self.v * PhisicalEntity.dt

class Slime(PhisicalEntity):
    def __init__(self, x, y, r, vx=0, vy=0):
        super().__init__(x,y,vx,vy)
        self.r = r
        self.forces.add(self.friction)

    def draw(self):
        s = pygame.Surface((2*self.r, self.r))
        c = pygame.draw.circle(s, YELLOW, (self.r,self.r), self.r)
        DISPLAY.blit(s, tuple(self.s))

    def move(self, v):
        if not (self.gravity in self.forces):
            super().move(v)

        if v[1] < 0: # up is negative
            self.forces.add(self.gravity)

    def update(self):
        # Slime out of the floor, get back on the floor.
        if self.s[1] > DISPLAY_HEIGHT - self.r:
            self.v[1] = 0.
            self.s[1] = DISPLAY_HEIGHT - self.r
            if self.gravity in self.forces:
                self.forces.remove(self.gravity)

        super().update()


class Ball(PhisicalEntity):
    def __init__(self, x, y, r, vx=0, vy=0):
        super().__init__(x,y,vx,vx)
        self.r = r
        self.forces.add(self.gravity)

    def draw(self):
        s = pygame.Surface((self.r*2, self.r*2))
        r = int(self.r)
        c = pygame.draw.circle(s, WHITE, (r,r), r)
        DISPLAY.blit(s, tuple(self.s))


slime1 = Slime(200., DISPLAY_HEIGHT-60, 60)
ball = Ball(200., 300., 10)
entities = [slime1, ball]

def rotate(vector, angle):
    """ Clock-wise rotation of vector around the angle."""
    return np.cos(angle)*vector[0] + np.sin(angle) * vector[1], \
        -np.sin(angle)* vector[0] + np.cos(angle) * vector[1]


while True: # main game loop

    DISPLAY.fill(BLACK)

    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                slime1.move([-15., 0])
            if event.key == pygame.K_RIGHT:
                slime1.move([15., 0])
            if event.key == pygame.K_UP:
                slime1.move([0., -50])

    # Collision detection: TODO abstract this away.
    if ball.s[1] >= DISPLAY_HEIGHT-2*ball.r:
        angle = np.pi
        x,y = rotate(ball.v, angle)
        print("floor {} {}, old {}".format(x,y, ball.v))
        (ball.v[0], ball.v[1]) = -x, y
        ball.update()

    elif ball.s[0] <= 0 :
        angle = -np.pi / 2
        x,y = rotate(ball.v, angle)
        print("wall {} {}, old {}".format(x,y, ball.v))
        ball.v[0], ball.v[1] = x, y
        ball.update()

    elif ball.s[0] >= DISPLAY_WIDTH:
        angle = np.pi / 2
        x,y = rotate(ball.v, angle)
        print ("wall {} {}, old {}".format(x,y, ball.v))
        ball.v[0], ball.v[1] = x, y
        ball.update()

    dist = np.linalg.norm((slime1.s+slime1.r) - (ball.s+ball.r), axis=0) # slime1 <-> ball euclidian distance
    if dist <= slime1.r + ball.r:
        d = slime1.s[1]-ball.s[1]-2*ball.r # distance from touching point to the floor
        r = slime1.r
        alpha = np.arcsin(d/r)
        beta = np.pi/2 - alpha
        x,y = rotate(ball.v, beta)
        ball.v[0], ball.v[1] = x, y
    # Collision detection end.

    for entity in entities:
        entity.update()
        entity.draw()

    pygame.display.update()
