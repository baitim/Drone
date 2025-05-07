import pygame
import os
import sys

pygame.init()

W = 1000
H = 1000

THROTTLE_SECTION_WIDTH = 200

LINE_WIDTH = W / 30
LINE_GAP   = H / 20

LINE_COUNT = 10

IP = "192.168.50.147:80"

clock = pygame.time.Clock()
screen = pygame.display.set_mode((W + THROTTLE_SECTION_WIDTH, H))
font = pygame.font.Font(None, 36)

yaw   = 0
pitch = 0
roll  = 0
throttle = 0

yaw_max   = 30
pitch_max = 30
roll_max  = 30
throttle_max = 100    

throttle_set_verified = 0

def draw(yaw: float, pitch: float, roll: float, throttle: float) -> None:
    screen.fill((0,0,0))

    if pitch > 0:
        pygame.draw.rect(screen, (255, 255, 255), (W/2 - LINE_WIDTH/2 + 1, H/2 - LINE_GAP*LINE_COUNT*pitch/pitch_max, LINE_WIDTH + 1, LINE_COUNT*LINE_GAP*pitch/pitch_max))
    else: 
        pygame.draw.rect(screen, (255, 255, 255), (W/2 - LINE_WIDTH/2 + 1, H/2, LINE_WIDTH + 1, -LINE_GAP*LINE_COUNT*pitch/pitch_max))

    if roll > 0:
        pygame.draw.rect(screen, (255, 255, 255), (W/2, H/2 - LINE_WIDTH/2, LINE_GAP*LINE_COUNT*roll/roll_max, LINE_WIDTH))
    else: 
        pygame.draw.rect(screen, (255, 255, 255), (W/2 + LINE_GAP*LINE_COUNT*roll/roll_max, H/2 - LINE_WIDTH/2, -LINE_GAP*LINE_COUNT*roll/roll_max , LINE_WIDTH))

    if yaw > 0:
        pygame.draw.rect(screen, (255, 255, 255), (W/2, 0, LINE_GAP*LINE_COUNT*yaw/yaw_max, LINE_WIDTH))
    else: 
        pygame.draw.rect(screen, (255, 255, 255), (W/2 + LINE_GAP*LINE_COUNT*yaw/yaw_max, 0, -LINE_GAP*LINE_COUNT*yaw/yaw_max , LINE_WIDTH))

    pygame.draw.rect(screen, (255, 255, 255), (W, H*(1-throttle/throttle_max), THROTTLE_SECTION_WIDTH, H))

    print(f"[Y, P, R]: [{yaw}, {pitch}, {roll}]")
    print(f"THR: {throttle}")

    for level in range(1, LINE_COUNT+1):
        text_level = font.render(f"{level*10}", 1, (180, 0, 0))
        #Pitch
        screen.blit(text_level, (W/2 - LINE_WIDTH/2, H/2 - level*LINE_GAP))
        #Roll
        screen.blit(text_level, (W/2 + (level-1)*LINE_GAP, H/2 - LINE_WIDTH/2))
        #Yaw
        screen.blit(text_level,(W/2 + (level-1)*LINE_GAP, 0))

    for level in range(-1, -LINE_COUNT - 1, -1):
        text_level = font.render(f"{level*10}", 1, (180, 0, 0))
        #Pitch
        screen.blit(text_level, (W/2 - LINE_WIDTH/2, H/2 - (level+1)*LINE_GAP))
        #Roll
        screen.blit(text_level, (W/2 + level*LINE_GAP, H/2 - LINE_WIDTH/2))
        #Yaw
        screen.blit(text_level,(W/2 + level*LINE_GAP, 0))

    for level in range(-LINE_COUNT, LINE_COUNT + 1, 1):
        #Pitch_lines
        pygame.draw.aaline(screen, (255,0,0), (W/2 - LINE_WIDTH/2, H/2 + level*LINE_GAP), (W/2 + LINE_WIDTH/2, H/2 + level*LINE_GAP))

        #Roll_lines
        pygame.draw.aaline(screen, (255,0,0), (W/2 + level*LINE_GAP, H/2 - LINE_WIDTH/2), (W/2 + level*LINE_GAP, H/2 + LINE_WIDTH/2))
        
        #Yaw_lines
        pygame.draw.aaline(screen, (255,0,0), (W/2 + level*LINE_GAP, 0), (W/2 + level*LINE_GAP, LINE_WIDTH))
    

    for level in range(10+1):
        pygame.draw.aaline(screen, (255, 0, 0), (W, H*(1-level/10)), (W + THROTTLE_SECTION_WIDTH, H*(1-level/10)))

        text_level = font.render(f"{level*10}", 1, (180, 0, 0))
        screen.blit(text_level, (W + THROTTLE_SECTION_WIDTH/2, H*(1-level/10)))


STEP = 3
THROTTLE_STEP = 5

DISCRETIZATION_STEP = 0.5 #minimum Y/P/R/throttle difference

pressed = False
changed = False
yaw_old = yaw
pitch_old = pitch
roll_old = roll
throttle_old = throttle

while(1):

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
        elif event.type == pygame.KEYDOWN:
            match(event.key):
                case pygame.K_w:
                    pitch = min(pitch + STEP, pitch_max)
                case pygame.K_s:
                    pitch = max(pitch - STEP, -pitch_max)

                case pygame.K_a:
                    roll = max(roll - STEP, -roll_max)
                case pygame.K_d:
                    roll = min(roll + STEP, roll_max)
                
                case pygame.K_q:
                    yaw = max(yaw - STEP, -yaw_max)
                case pygame.K_e:
                    yaw = min(yaw + STEP, yaw_max)

                case pygame.K_LCTRL:
                    throttle = max(throttle - THROTTLE_STEP, 0)
                case pygame.K_LSHIFT:
                    throttle = min(throttle + THROTTLE_STEP, throttle_max)
                
                case pygame.K_r:
                    throttle = 0
                    throttle_old = -1

                case pygame.K_v:
                    throttle_set_verified = 1
                
                case pygame.K_k:
                    with open("PID_values.txt") as f:
                        f.readline()
                        K_prop = [float(i) for i in f.readline().split()[1:]]
                        K_intg = [float(i) for i in f.readline().split()[1:]]
                        K_diff = [float(i) for i in f.readline().split()[1:]]
                        
                        print(f"sended:\nP: {K_prop[0]} {K_prop[1]} {K_prop[2]}\nI: {K_intg[0]} {K_intg[1]} {K_intg[2]}\nD: {K_diff[0]} {K_diff[1]} {K_diff[2]}")
                        os.system(f"curl --data-binary '{K_prop[0]} {K_prop[1]} {K_prop[2]} {K_intg[0]} {K_intg[1]} {K_intg[2]} {K_diff[0]} {K_diff[1]} {K_diff[2]}' {IP}/PID")

        elif event.type == pygame.MOUSEBUTTONDOWN:
            pressed = True
            if event.pos[0] > W and 0 < event.pos[1] < H:
                throttle = (1 - event.pos[1]/H)*throttle_max
            else:
                if(0 < event.pos[0] < W):
                    roll = (event.pos[0]- W/2)/(LINE_COUNT*LINE_GAP)*roll_max
                if(0 < event.pos[1] < H):
                    pitch = (H/2 - event.pos[1])/(LINE_COUNT*LINE_GAP)*pitch_max

        elif event.type == pygame.MOUSEBUTTONUP:
            pressed = False
        elif (event.type == pygame.MOUSEMOTION) and pressed:
            if event.pos[0] > W and 0 < event.pos[1] < H:
                throttle = (1 - event.pos[1]/H)*throttle_max
            else:
                if(0 < event.pos[0] < W):
                    roll = (event.pos[0]- W/2)/(LINE_COUNT*LINE_GAP)*roll_max
                if(0 < event.pos[1] < H):
                    pitch = (H/2 - event.pos[1])/(LINE_COUNT*LINE_GAP)*pitch_max

    if(abs(throttle - throttle_old) > DISCRETIZATION_STEP or abs(yaw - yaw_old) > DISCRETIZATION_STEP or abs(roll - roll_old) > DISCRETIZATION_STEP or abs(pitch - pitch_old) > DISCRETIZATION_STEP):
        changed = True
        yaw_old = yaw
        pitch_old = pitch
        roll_old = roll
        throttle_old = throttle
    else:
        throttle = throttle_old
        yaw = yaw_old
        pitch = pitch_old
        roll = roll_old
        changed = False

    if(changed):
        os.system(f"curl --data-binary '{yaw} {pitch} {roll} {throttle} {throttle_set_verified}' {IP}/control")
        print("")

    draw(yaw, pitch, roll, throttle)
    pygame.display.update()
    clock.tick(20)
