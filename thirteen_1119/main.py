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
ev3.speaker.beep()


left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

robot = DriveBase(left_motor, right_motor, 55.5 , 104)

# robot.straight(100), 10cm 전진

# robot.turn(360), 시계 방향 360도 회전

"""robot.drive(100, 0) 속도 100으로 전진
wait(1000)              1초 대기
robot.stop()            모터 스탑"""


#오른쪽으로 1바퀴 도는 코드
"""robot.drive(100, 360)
wait(1000)
robot.stop()"""


#ㄷ자 주행
"""robot.drive(30, 0)
wait(2000)
robot.stop()
robot.turn(90)
robot.drive(30, 0)
wait(2000)
robot.stop()
robot.turn(90)
robot.drive(30, 0)
wait(2000)
robot.stop()"""

#on-off line tracing 
"""cs = ColorSensor(Port.S4)

while True:
    if cs.color() == Color.BLACK:
        robot.drive(100, -60)
    else:
        robot.drive(100, 60)"""

"""BLACK=9
WHITE=85
threshold=(BLACK+WHITE)/2

DRIVE_SPEED=100
PROPORTIONAL_GAIN=1.2

while True:
    deviation = line_sensor.reflection() -threshold
    turn_rate = PROPORTIONAL_GAIN * deviation
    robot.drive(DRIVE_SPEED,turn_rate)"""

left_cs = ColorSensor(Port.S3)
right_cs = ColorSensor(Port.S4)

#센서 직접 입력(내가 만든거임)
"""WHITE = -1
BLACK = -1
while (WHITE == -1):
    if Button.LEFT in ev3.buttons.pressed():
        WHITE = right_cs.reflection()

while (BLACK == -1):
    if Button.RIGHT in ev3.buttons.pressed():
        BLACK = right_cs.reflection()

threshold = WHITE + BLACK / 2"""

#1칸 전진
"""threshold = 50
kp = 1.2

while True:
    left_reflection = left_cs.reflection()
    right_reflection = right_cs.reflection()
    if right_reflection < 30:
        robot.stop()
        break
    else:
        error = left_reflection - threshold
        turn_rate = kp * error
        robot.drive(100, turn_rate)
        wait(10)"""

#2칸 전진, for 문을 통해 n칸 전민 알고리즘 만들 수 있음
"""threshold = 50
kp = 1.2

while True:
    left_reflection = left_cs.reflection()
    right_reflection = right_cs.reflection()
    if right_reflection < 30:
        robot.stop()
        break
    else:
        error = left_reflection - threshold
        turn_rate = kp * error
        robot.drive(100, turn_rate)
        wait(10)

while True:
    left_reflection = left_cs.reflection()
    right_reflection = right_cs.reflection()
    if right_reflection > 30:
        robot.stop()
        break
    else:
        error = left_reflection - threshold
        turn_rate = kp * error
        robot.drive(100, turn_rate)
        wait(10)

while True:
    left_reflection = left_cs.reflection()
    right_reflection = right_cs.reflection()
    if right_reflection < 30:
        robot.stop()
        break
    else:
        error = left_reflection - threshold
        turn_rate = kp * error
        robot.drive(100, turn_rate)
        wait(10)"""

#N칸 전진 for문 ver
"""threshold = 50
kp = 1.2

for i in range (0, 2):
    while True:
        left_reflection = left_cs.reflection()
        right_reflection = right_cs.reflection()
        if right_reflection < 30:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)

    while True:
        left_reflection = left_cs.reflection()
        right_reflection = right_cs.reflection()
        if right_reflection > 30:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)"""

#절대 방향 ppt 46p
"""N = 0
E = 1
S = 2
W = 3

now_dir = 0
target_dir = 1

turn_table = [0, 90, 180, -90]

threshold = 50
kp = 1.2

for i in range (0, 4):
    while True:
        left_reflection = left_cs.reflection()
        right_reflection = right_cs.reflection()
        if right_reflection < 30:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)

    while True:
        left_reflection = left_cs.reflection()
        right_reflection = right_cs.reflection()
        if right_reflection > 30:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)

robot.turn(turn_table[target_dir])

for i in range (0, 2):
    while True:
        left_reflection = left_cs.reflection()
        right_reflection = right_cs.reflection()
        if right_reflection < 30:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)

    while True:
        left_reflection = left_cs.reflection()
        right_reflection = right_cs.reflection()
        if right_reflection > 30:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)"""


#1119 jae chool
N = 0
E = 1
S = 2
W = 3

now_dir = N
target_dir = W

turn_table = [0, 90, 180, -90] #turn 돌릴 때 target table 값 - now table 값 하면 됨

threshold = 50
kp = 1.2


#1칸전진
while True:
    left_reflection = left_cs.reflection()
    right_reflection = right_cs.reflection()
    if left_reflection < 30:
        robot.stop()
        break
    else:
        error = right_reflection - threshold
        turn_rate = kp * error
        robot.drive(100, -turn_rate)
        wait(10)

while True:
    left_reflection = left_cs.reflection()
    right_reflection = right_cs.reflection()
    if left_reflection > 30:
        robot.stop()
        break
    else:
        error = left_reflection - threshold
        turn_rate = kp * error
        robot.drive(100, -turn_rate)
        wait(10)

robot.turn(turn_table[3])

now_dir = W
target_dir = N

for i in range (0, 2):
    while True:
        left_reflection = left_cs.reflection()
        right_reflection = right_cs.reflection()
        if left_reflection < 30:
            robot.stop()
            break
        else:
            error = right_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, -turn_rate)
            wait(10)

    while True:
        left_reflection = left_cs.reflection()
        right_reflection = right_cs.reflection()
        if left_reflection > 30:
            robot.stop()
            break
        else:
            error = right_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, -turn_rate)
            wait(10)

robot.turn(turn_table[1])

now_dir = N
target_dir = E

for i in range (0, 2):
    while True:
        left_reflection = left_cs.reflection()
        right_reflection = right_cs.reflection()
        if right_reflection < 30:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)

    while True:
        left_reflection = left_cs.reflection()
        right_reflection = right_cs.reflection()
        if right_reflection > 30:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)

robot.turn(turn_table[1])

now_dir = E
target_dir = N

for i in range (0, 2):
    while True:
        left_reflection = left_cs.reflection()
        right_reflection = right_cs.reflection()
        if right_reflection < 30:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)

    while True:
        left_reflection = left_cs.reflection()
        right_reflection = right_cs.reflection()
        if right_reflection > 30:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)

robot.turn(turn_table[3])


now_dir = N

while True:
    left_reflection = left_cs.reflection()
    right_reflection = right_cs.reflection()
    if left_reflection < 30:
        robot.stop()
        break
    else:
        error = right_reflection - threshold
        turn_rate = kp * error
        robot.drive(100, -turn_rate)
        wait(10)

while True:
    left_reflection = left_cs.reflection()
    right_reflection = right_cs.reflection()
    if left_reflection > 30:
        robot.stop()
        break
    else:
        error = left_reflection - threshold
        turn_rate = kp * error
        robot.drive(100, -turn_rate)
        wait(10)

ev3.speaker.beep()