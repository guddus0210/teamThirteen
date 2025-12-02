#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
from collections import deque

ev3.speaker.beep()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
arm_motor = Motor(Port.B)

robot = DriveBase(left_motor, right_motor, 55.5 , 104)

left_cs = ColorSensor(Port.S3)
right_cs = ColorSensor(Port.S4)
object_detector = ColorSensor(Port.S2)

ult_cs = UltrasonicSensor(Port.S1)
N, E, S, W = 1, 2, 3, 4


def move_manhattan(start_xy, goal_xy, now_dir): #manhattan
    x, y=start_xy 
    gx, gy=goal_xy
    dx=gx-x
    dy=gy-y
    if dx!=0:
        target_dir=N if dy>0 else S
        now_dir=turn_min(now_dir, target_dir)
        steps=abs(dy)
        for _ in range(steps):
            n_move(1)
            y+=1 if target_dir==N else -1

        return (x, y), now_dir
    if dy!=0:
        target_dir =N if dy > 0 else S
        now_dir = turn_min(now_dir, target_dir)
        steps=abs(dy)
        for _ in range(steps):
            n_move(1)
            y+= 1 if target_dir == N else -1

def turn_min(now_dir, target_dir): #회전
    diff=(target_dir - now_dir)%4
    angle=[0, 90, 180, -90][diff]
    robot.turn(angle)
    return target_dir

def left_line_following(speed, kp):
    ult_distance = ult_cs.distance()
    if ult_distance <= 30:
        grab_object()
    threshold=50
    left_reflection =left_cs.reflection()
    error=left_reflection - threshold
    turn_rate=kp*error
    robot.drive(speed, turn_rate)

def right_line_following(speed, kp):
    ult_distance = ult_cs.distance()
    if ult_distance <= 30:
        grab_object()
    threshhold=50
    right_reflection=right_cs.reflection()
    error=right_reflection - threshold
    turn_rate=-kp*error
    robot.drive(speed, turn_rate)

def grab_object():
    arm_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=50)

def release_object():
    arm_motor.run_until_stalled(-200, then = Stop.COAST, duty_limit=50)

def n_move(n, direction="right"):
    for _ in range(n):
        if direction =="right":
            while right_cs.reflection() > 30:
                left_line_following(100, 1.2)
            while right_cs.reflection()<=30:
                right_line_following(100, 1.2)
        elif direction=="left":
            while left_cs.reflection()>30:
                right_line_following(100, 1.2)
            while left_cs.reflection()<=30:
                left_line_following(100, 1.2)

    robot.stop()


threshold = 20

start = (0, 0)
goal = (0, 2)
now_dir = N
object_color = None

release_object()

move_manhattan(start, goal, now_dir)
object_color=object_detector.color()
print(object_color)

if object_color == Color.RED:
    robot.turn(180)
    n_move(1, direction="left")
    robot.turn(-70)
    n_move(1, direction="right")
    robot.turn(90)
    n_move(1, direction="right")
    release_object()
if (object_color==Color.BLUE) or (object_color==Color.GREEN):
    robot.turn(180)
    n_move(1, direction="left")
    robot.turn(-70)
    wait(10)
    
    n_move(2, direction="right")
    robot.turn(90)
    robot.drive(200, 0)
    wait(100)
    release_object()

ev3.speaker.beep()


#bfs 관련
"""MAP =  [
    ".....",
    ".#...",
    ".#...",
]

G = [[1 if c== "#" else 0 for c in r] for r in MAP]

H, W  = len(G), len(G[0])

S, E = (0, 0), (4, 4)

def bfs(s, g):
    dist = [[None]*W for _ in range(H)]
    prev = [[None]*W for _ in range(H)]
    q = deque([s])
    dist[s[1]][s[0]] = 0
    while q:
        x, y q.popleft()
        if (x,y) == g:
            break
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x+dx, y+dy
            if 0 <= nx < W and 0<=ny<H and not G[ny][nx] and dist[ny][nx] is None:
                dist[ny][nx] = dist[y][x]+1
                prev[ny][nx] = (x, y)
                q.append((nx, ny))

    path = []
    if dist[g[1][g[0]] is not None]:
        p = g
        while p:
            path.append(q)
            p = prev[g[1]][g[0]]
        path.reverse()
    return path, dist"""