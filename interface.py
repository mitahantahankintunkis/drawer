#!/usr/bin/env python3
from PIL import Image
from PIL import ImageDraw
import argparse
import math
import serial
import struct
import time


# Loads points exported by https://github.com/mitahantahankintunkis/linefier
# and changes their coordinate system to real world coordinates (millimeters)
def loadPoints(args):
    points = []
    with open(args.coordinates, 'r') as f:
        lines = f.read().split('\n')

        aspect_ratio = float(lines[0])

        # Calculating the paper's width and height
        height = args.height - args.padding * 2
        width  = int(height * aspect_ratio)
        if width > args.width - args.padding * 2:
            width  = args.width - args.padding * 2
            height = width / aspect_ratio

        # Offsets used to center the image
        offset_x = (args.width  - args.padding * 2 - width ) // 2
        offset_y = (args.height - args.padding * 2 - height) // 2

        for line in lines[1:-1]:
            x, y = line.split(' ')
            x = float(x) * width  + args.padding + args.offset_x + offset_x
            y = float(y) * height + args.padding + args.offset_y + offset_y

            # Linefier and the robot have different coordinate systems.
            # Linefier's y axis grows downwards while the robots grows
            # upwards. Fixing this by flipping the y axis
            y = args.height - y + args.offset_y * 2

            points.append((x, y))

    return points



# Simulates what the final image should look like
def simulateDrawing(points, args):
    scl = 5
    img = Image.new('RGB', (args.width * scl, args.height * scl), 'white')
    draw = ImageDraw.Draw(img)

    px = points[0][0] - args.offset_x
    py = points[0][1] - args.offset_y
    total_length = 0

    for x, y in points[1:]:
        tx = x - args.offset_x
        ty = y - args.offset_y

        draw.line((px * scl, py * scl, tx * scl, ty * scl), fill='#220044')
        total_length += ((tx - px)**2 + (ty - py)**2)**.5

        px = tx
        py = ty

    print(f'The device will draw a line of {round(total_length / 1000, 2)}m')

    img = img.transpose(Image.FLIP_TOP_BOTTOM)
    img.save('simulated.png')


# Calculates the arm angles for the robot. I'd like to do this on the arduino,
# but the arduino I'm using doesn't have doubles. When using floats the
# arduino starts to lose track of its pen's location after around 15 minutes
def coordToAngle(point, args):
    x, y = point
    dist = (x**2 + y**2)**.5

    # Angles
    arm_0_angle  =  math.acos((args.length0**2 + dist**2 - args.length1**2) / (2 * args.length0 * dist))
    arm_1_angle  = -math.acos((args.length1**2 + dist**2 - args.length0**2) / (2 * args.length1 * dist))
    angle_offset =  math.atan(y / x)
    arm_0_angle +=  angle_offset
    arm_1_angle +=  angle_offset
    arm_1_angle -=  arm_0_angle

    return arm_0_angle, arm_1_angle


# Sends points to the arduino over serial
def sendPoints(points, args):
    # Opening connection
    try:
        ser = serial.Serial(args.port, args.baud_rate)

    except serial.SerialException:
        print('Could not connect')
        return

    start_time = time.time()

    print('\n\nConnected')
    print('Moving the arm to the starting position...')

    for i in range(len(points)):
        # Waiting for the arduino to be ready
        if ser.read() == b'\x01':
            # Sending the next point
            try:
                a0, a1 = coordToAngle(points[i], args)
                ser.write(struct.pack('<f', a0) + struct.pack('<f', a1))
                ser.flush()

            except serial.SerialException:
                print('Communication error while sending angles')
                break

            # Printing information about the current progress
            percentage = round((i + 1) / len(points) * 100, 2)
            delta_t    = time.time() - start_time
            delta_m    = int(delta_t // 60)
            delta_s    = round(delta_t - delta_m * 60, 1)
            m_to_go    = '?'

            # Remaining time
            if percentage != 0:
                m_to_go = round((delta_t / (percentage / 100) - delta_t) // 60)

            print((f'i: {i}    '
                   f'X: {round(points[i][0]):>4}mm    '
                   f'Y: {round(points[i][1]):>4}mm      '
                   f'Progress: {percentage:>5}% done, '
                   f'{delta_m}m {delta_s}s gone,  '
                   f'{m_to_go}m left'))

            # Waiting for the user to make sure that the robot is ready to draw
            if i == 0 and input('\nContinue drawing? (y/n): ').lower() != 'y':
                break


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('coordinates', help='Path to file containing line coordinates')

    parser.add_argument('-x', '--width',   default=210,  type=int, help='Paper width in millimeters')
    parser.add_argument('-y', '--height',  default=297,  type=int, help='Paper height in millimeters')
    parser.add_argument('-p', '--padding', default=40,   type=int, help='Space between the drawing and the paper\'s edges in millimeters')
    parser.add_argument('--offset_x',      default=45,   type=int, help='Paper offset in millimeters along the x axis')
    parser.add_argument('--offset_y',      default=-148, type=int, help='Paper offset in millimeters along the y axis')
    parser.add_argument('--length0',       default=175,  type=int, help='Length of the first arm')
    parser.add_argument('--length1',       default=125,  type=int, help='Length of the second arm')

    parser.add_argument('--port',      default='/dev/ttyACM3', type=str, help='Serial port for the arduino')
    parser.add_argument('--baud_rate', default=115200 ,        type=int, help='Baud rate for the arduino')

    args = parser.parse_args()

    points = loadPoints(args)
    simulateDrawing(points, args)
    sendPoints(points, args)


if __name__ == '__main__': main()
