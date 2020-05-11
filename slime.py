import pygame, sys
from pygame.locals import *

import numpy as np

DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 600
WHITE = (255,255,255)
BLUE = (0,0,255)
YELLOW = (125,0,0)
BLACK = (0,0,0)

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
    def __init__(self, x, y, r, vx=0, vy=0, color=YELLOW):
        super().__init__(x,y,vx,vy)
        self.r = r
        self.forces.add(self.friction)
        self.color = color

    def draw(self):
        s = pygame.Surface((2*self.r, self.r))
        c = pygame.draw.circle(s, self.color, (self.r,self.r), self.r)
        DISPLAY.blit(s, tuple(self.s))

    def move(self, v):
        # Slime is on the floor.
        if not (self.gravity in self.forces):
            super().move(v)

        # Slime is airborne.
        if v[1] < 0: # up is negative
            self.forces.add(self.gravity)

    def update(self):
        # Slime out of the floor, get back on the floor.
        if self.s[1] > DISPLAY_HEIGHT - self.r:
            self.v[1] = 0.
            self.s[1] = DISPLAY_HEIGHT - self.r
            if self.gravity in self.forces:
                self.forces.remove(self.gravity)

        if self.s[0] < self.r:
            self.s[0] = self.r
        if self.s[0] + self.r >= DISPLAY_WIDTH:
            self.s[0] = DISPLAY_WIDTH - 2*self.r

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

    def update(self):
        if self.s[1] < self.r:
            self.v[1] = 0.
            self.s[1] = self.r

        super().update()


slime1 = Slime(200., DISPLAY_HEIGHT-60, 60)
slime2 = Slime(400., DISPLAY_HEIGHT-60, 60, 0,0, BLUE)
slimes = [slime1]
ball = Ball(200., 300., 10, 30,  30)
entities = slimes + [ball]

def rotate(vector, angle):
    """ Clock-wise rotation of vector around the angle."""
    return np.cos(angle) * vector[0] + np.sin(angle) * vector[1], \
        -np.sin(angle) * vector[0] + np.cos(angle) * vector[1]


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

    for slime1 in slimes:

    # Collision detection: TODO abstract this away.
        # Bouncing of the floor.
        if ball.s[1] >= DISPLAY_HEIGHT-2*ball.r:
            nextPosition = ball.s + ball.v * PhisicalEntity.dt
            if nextPosition[1] > DISPLAY_HEIGHT-2*ball.r:
                ball.s[1] = DISPLAY_HEIGHT-2*ball.r

                impactAngle = np.arctan2(ball.v[1], ball.v[0])
                # print(f"Impact angle: {np.rad2deg(impactAngle)}, Out angle should be {-np.rad2deg(impactAngle)}")
                vx, vy = rotate(ball.v, 2 * impactAngle)
                # print("floor {} {}, old {}".format(vx,vy, ball.v))
                ball.v[0], ball.v[1] = vx, vy

                outAngle = np.arctan2(vy, vx)
                # print(f"Out angle is {np.rad2deg(outAngle)}")

        # Ball hit the left wall.
        elif ball.s[0] <= 0:
            nextPosition = ball.s + ball.v * PhisicalEntity.dt
            if nextPosition[0] <= 0:
                ball.s[0] = 0 #2 * ball.r

                impactAngle = np.arctan2(ball.v[1], ball.v[0])
                # print(f"Impact angle: {np.rad2deg(impactAngle)}, Out angle should be {np.rad2deg(np.pi-impactAngle)}")
                vx, vy = rotate(ball.v, np.pi + 2 * impactAngle)
                # print("floor {} {}, old {}".format(vx, vy, ball.v))
                ball.v[0], ball.v[1] = vx, vy

        # Ball hit the right wall.
        elif ball.s[0] >= DISPLAY_WIDTH - 2 * ball.r:
            nextPosition = ball.s + ball.v * PhisicalEntity.dt
            if nextPosition[0] >= DISPLAY_WIDTH - 2 * ball.r:
                ball.s[0] = DISPLAY_WIDTH - 2 * ball.r

                impactAngle = np.arctan2(ball.v[1], ball.v[0])
                # print(f"Impact angle: {np.rad2deg(impactAngle)}, Out angle should be {np.rad2deg(np.pi - impactAngle)}")
                vx, vy = rotate(ball.v, np.pi + 2 * impactAngle)
                # print("floor {} {}, old {}".format(vx, vy, ball.v))
                ball.v[0], ball.v[1] = vx, vy

        prev = ball.s - ball.v * PhisicalEntity.dt
        dist = np.linalg.norm((slime1.s+slime1.r) - (ball.s+ball.r), axis=0)  # slime1 <-> ball euclidian distance
        if dist <= slime1.r + ball.r:
            impactAngleBall = np.arctan2(ball.v[1], ball.v[0])
            impactAngleSlime = np.arctan2(slime1.v[1], slime1.v[0])

            centerImpactDistance = (slime1.s+slime1.r) - (ball.s+ball.r) #slime1.s - ball.s
            # print(str(np.linalg.norm((centerImpactDistance), axis=0)) + "   " + str(dist) + "  " + str(centerImpactDistance) + "  " + str(ball.s) +"  " + str(slime1.s))
            normalAngle = np.arctan2(centerImpactDistance[1], centerImpactDistance[0]) + np.pi

            tangentAngle = normalAngle + np.pi / 2

            # input("{} {}".format(np.rad2deg(normalAngle), np.rad2deg(tangentAngle)))

            slimeImpactVelocity = slime1.v

            x, y = rotate(ball.v, 2*tangentAngle + 2*impactAngleBall)

            ball.v[0], ball.v[1] = x + slimeImpactVelocity[0], y + slimeImpactVelocity[1]
            ball.v[0], ball.v[1] = 0.9 * ball.v[0], 0.9 * ball.v[1]

            ball.s = prev
            while np.linalg.norm((slime1.s+slime1.r) - (ball.s+ball.r), axis=0) <= slime1.r + ball.r:
                # ball.update()
                if ball.s[0] > slime1.s[0]:
                    ball.s[0] += 1 # (slime1.r + ball.r) - dist
                else:
                    ball.s[0] -=  1 # (slime1.r + ball.r) - dist
                ball.s[1] -= 1 #dist


        # Collision detection end.

    for entity in entities:
        entity.update()
        entity.draw()

    pygame.display.update()
