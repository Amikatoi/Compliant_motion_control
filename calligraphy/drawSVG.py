'''
    File name: readCharacter.py
    Author: Ya-Liang Chang b03901014@ntu.edu.tw
    Date created: 5/26/2017
    Date last modified: 5/26/2017
    Python Version: 3.6

    Path planning for 4 DOFs palletizing robot to write Chinese characters.

    Read characters from graphics.txt (See https://github.com/skishore/makemeahanzi).
    Each char is stored as a dictionary with three keys:
        character: The Unicode character for this glyph. Required.
        strokes: List of SVG path data for each stroke of this character, ordered by proper stroke order.
        medians: A list of stroke medians, in the same coordinate system as the SVG paths above.

        For example:
        {
        'character': '⺈',
        'strokes': ['M 441 666 Q 490 726 523 749 Q 525 750 526 751 Q 547 768 509 808 Q 486 830 469 833 Q 451 834 456 811 Q 461 792 441 757 Q 396 672 248 545 Q 232 535 232 528 Q 233 521 242 521 Q 288 521 423 651 L 441 666 Z', 'M 527 467 L 604 554 Q 653 615 705 653 Q 723 664 710 678 Q 696 692 655 714 Q 647 717 596 703 Q 454 668 441 666 L 423 651 Q 427 647 433 645 Q 457 637 541 651 Q 596 661 600 657 Q 604 653 598 639 Q 568 583 496 462 Q 521 466 527 467 Z'],
        'medians':[[[468, 819], [490, 772], [428, 689], [320, 583], [274, 547], [240, 529]], [[430, 652], [527, 665], [588, 681], [614, 681], [646, 664], [631, 632], [540, 504], [520, 478], [505, 469]]]
        }

    Then, write the character (use medians) in pygame, and transform to paths for palletizing robot.

'''
import json
import pygame
import os, sys
from xml.dom import minidom
from svg.path import parse_path#, Path, Line, Arc, CubicBezier, QuadraticBezier

"""
Parameters for SVG file.
"""
SVG_SCALE = 0.12
SVG_APPROX_STEP = 0.5 # For approximating curves to lines in sys.path


"""
Boundary of working area of the palletizing robot.
"""
X_MAX = 630
X_MIN = 330
Y_MAX = 350
Y_MIN = -350

"""
Show the path in working area on Pygame.
"""
WIDTH = X_MAX - X_MIN
HEIGHT = Y_MAX - Y_MIN
PYGAME_LINE_WIDTH = 4
PYGAME_THIN_LINE_WIDTH = 2
PYGAME_THICK_LINE_WIDTH = 7
NOT_ADJUST_THICKNESS = True
#NOT_ADJUST_THICKNESS = False

"""
Each stroke is laid out on a 1024x1024 size coordinate system where:
    The upper-left corner is at position (0, 900).
    The lower-right corner is at position (1024, -124).
However, the upper-left (+x, -y) corner of robot working area is about (630, -300)
         the upper-left (+x, +y) corner of pygame is about (0, 0), and there is no negative coordinate for pygame.
So the stroke need not be shifted and flip.

    Stroke point (x, y)
    Robot point (X, Y)
    X = -x + 1024(X_B)
    Y = -y + 900(Y_B)

And the characters need to be shrinked and shifted to the right position.
    Scaled robot point (X', Y')
    X' = X * scale_x + shift_x
    Y' = Y * scale_y + shift_y

Resulting
    pointList = [((-point[0] + X_B)*SCALE_X + SHIFT_X, (-point[1] + Y_B)*SCALE_Y + self.shift_y) for point in eachLine]

The following are related constant.
"""

SCALE_X = 0.12
SCALE_Y = 0.12
X_B = 1024
Y_B = 900
SHIFT_X = 420
SHIFT_Y = 50

Y_CHAR_DIS = 900 * SCALE_Y # For writing chars one by one without moving the paper
Y_INIT_POS = -3.5 * Y_CHAR_DIS # Initial char position
Z_ADJUST = 0 # For uneven working area
INK_POS = (INK_X, INK_Y) = (631, 0) # For automatic ink dipping
Z_THICK_ADJUST = 0.2 #Adjust z for thicker strokes
Z_THIN_ADJUST = 0.2


""" Constant for palletizing robot to touch the paper (down) and up. """
UP_Z = 150
DOWN_Z = 125

""" Constant for pygame color """
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

def help_message():
    print ("Usage: python3 drawSVG.py [-s | -d <image>]")
    print ("\t-s          write chinese sentence")
    print ("\t-d <image>  draw SVG file")

class Wrapper:
    def __init__(self, DBFilename):
        self.DBFilename = DBFilename
        self.sentence = ""
        self.filename = ""
        self.fileNum = 1
        self.screen = None

        """ Used to generate palletizing robot path code. """
        self.pathCodeList = []
        self.pointNo = 0
        self.create_robot_point(471, -6, 237)
        self.shift_y = Y_INIT_POS
        self.up_z = UP_Z
        self.down_z = DOWN_Z

    def goToNextFile(self):
        """ When too mush point in a script file, the palletizing robot will crush.
            Therefore a new file is needed. """
        self.print_all()

        self.fileNum += 1
        self.filename = self.filename.split('_')[0]
        self.filename += '_'+str(self.fileNum)
        self.pathCodeList = []
        self.pointNo = 0
        self.create_robot_point(471, -6, 237)

        self.pygame_wait()

    def initPygame(self):
        """ Initialize pygame window. """
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        self.screen.fill(WHITE)

    def pygame_wait(self):
        """ Wait until user press a key. """
        pygame.display.update()
        print ("Press button to continue ..")
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == pygame.KEYDOWN:
                    return event.key

    def robotCoorToPygameCoor(self, pointList):
        if (type(pointList) is tuple):
            (x, y) = pointList
            newPointList = (-x + X_MAX, y + Y_MAX)

        else:
            newPointList = [(-x + X_MAX, y + Y_MAX) for x, y in pointList]
        return newPointList

    def pygameCoorToRobotCoor(self, pointList):
        newPointList = [(-x + X_MAX, y - Y_MAX) for x, y in pointList]
        return newPointList

    def charMedianCorrToRobotCorr(self, pointList, shift_y=-1):
        shift_y = self.shift_y if shift_y == -1 else shift_y
        newPointList = [((-x + X_B)*SCALE_X + SHIFT_X, (-y + Y_B)*SCALE_Y + self.shift_y) for x, y in pointList]
        return newPointList

    def draw_image(self, image_file, svg_scale=SVG_SCALE):
        """ Draw the SVG file with some apporximation. """

        self.initPygame()
        self.down_z  += 0.2
        # Get path from SVG file
        self.filename = image_file.replace('/', '-').split('.')[0]
        doc = minidom.parse(image_file)
        path_strings = [path.getAttribute('d') for path
                            in doc.getElementsByTagName('path')]
        doc.unlink()
        count = 0
        for eachPath in path_strings:
            count += 1
            p = parse_path(eachPath)
            pointList = []
            for j in range(0, int(len(p)), 2): # Skip half object (lines, arcs...)
                eachObject = p[j]
                """Approximate the curve throught floating-point steps from 0 to 1"""
                for i in range(0, int(1.0/SVG_APPROX_STEP)):
                    i = round(i * SVG_APPROX_STEP, 3)
                    x = int(round(eachObject.point(i).real * svg_scale + SHIFT_X/2, 3))
                    y = int(round(eachObject.point(i).imag * svg_scale, 3))
                    print (i, x, y, flush=True)
                    pointList.append((x, y))
            pointList = [(point[0]+160, -point[1]+120) for point in pointList]
            self.generate_path(pointList)
            if count % 5 == 0:
                self.dipInk()

    def write_sentence(self, sentence):
        """ Find char data in the DB and write the sentence. """

        self.initPygame()
        self.sentence = sentence
        self.filename = sentence
        characters = list(sentence)
        print("Character list: ", characters)
        with open(self.DBFilename, "r") as f:
            all_char = f.readlines()
            charToWriteDataList = []
            charNotFoundList = []
            print("")
            count = 0
            for eachCharToWrite in sentence:
                print("Searching ", count, " / ", len(characters), " ...", end='\r')
                count += 1
                found = False
                for eachLine in all_char:
                    eachCharInDB = json.loads(eachLine)
                    if eachCharToWrite == eachCharInDB['character']:
                        charToWriteDataList.append(eachCharInDB)
                        found = True
                        """write the char in next position instead of moving paper"""
                        self.shift_y += Y_CHAR_DIS
                        if (self.shift_y > -110):
                            self.down_z = DOWN_Z + Z_ADJUST
                        if (self.shift_y > 40):
                            self.down_z = DOWN_Z
                        """ return to initial pos if out of working area """
                        if (self.shift_y > Y_MAX):
                            self.shift_y = Y_INIT_POS
                        self.write_character(eachCharInDB)
                        break
                if not found:
                    charNotFoundList.append(eachCharToWrite)
                if count != len(sentence):
                    self.dipInk()

            print("Searching ", count, " / ", len(characters), " ... Done.")
        self.create_robot_point(0, 0, 0, toNextFile=True) #End of work
        print("Found: ", [char['character'] for char in charToWriteDataList])
        print("Not found: ", [char for char in charNotFoundList])

    def dipInk(self):
        """ Dip the ink automatically in desired position. """
        inkPath = [(INK_X, INK_Y)]
        self.down_z += 3
        self.up_z -= 10
        self.generate_path(inkPath)
        self.create_robot_point(INK_X-36, INK_Y, self.up_z)
        self.down_z -= 3
        self.up_z += 10
        self.create_robot_point(INK_X-36, INK_Y, self.up_z)

    def write_character(self, char):
        """ Write the character on Pygame. """
        for eachLine in char['medians']:
            pointList = self.charMedianCorrToRobotCorr(eachLine)
            self.generate_path(pointList)

    def generate_path(self, pointList, up_Z=-1, down_Z=-1):
        """ Generate path for a line of the image. """
        count = 0
        toEnd = NOT_ADJUST_THICKNESS
        start_point = self.robotCoorToPygameCoor( pointList[0] )
        for (x, y) in pointList:
            upZ = self.up_z if up_Z == -1 else up_Z
            downZ = self.down_z if down_Z == -1 else down_Z
            if_draw = True

            end_point = self.robotCoorToPygameCoor( (x, y) )
            pygame.draw.line(self.screen, BLACK, start_point, end_point, PYGAME_LINE_WIDTH)
            pygame.display.update()
            if not toEnd:
                pressed_key = self.pygame_wait()
                if (pressed_key == pygame.K_a):
                    downZ -= Z_THICK_ADJUST
                    pygame.draw.line(self.screen, BLACK, start_point, end_point, PYGAME_THICK_LINE_WIDTH)
                    pygame.display.update()
                elif (pressed_key == pygame.K_d):
                    downZ += Z_THIN_ADJUST
                    pygame.draw.line(self.screen, WHITE, start_point, end_point, PYGAME_LINE_WIDTH)
                    pygame.draw.line(self.screen, BLACK, start_point, end_point, PYGAME_THIN_LINE_WIDTH)
                elif (pressed_key == pygame.K_e):
                    toEnd = True
                elif (pressed_key == pygame.K_p):
                    pygame.draw.line(self.screen, WHITE, start_point, end_point, PYGAME_LINE_WIDTH)
                    if_draw = False
            start_point = end_point

            if if_draw:
                """ Move to the stating point. """
                if count == 0:
                    self.create_robot_point(x, y, upZ)
                self.create_robot_point(x, y, downZ)
                """ Move to the last point. """
                if count == len(pointList) -1:
                    self.create_robot_point(x, y, upZ)
            count += 1

    def create_robot_point(self, x_coor, y_coor, z_coor, spin=0, toNextFile=False):
        """ Create point to code and check if going to next file. """
        self.add_robot_point_to_code(x_coor, y_coor, z_coor, spin)

        """ When too much points, got to the next file. """
        if (self.pointNo > 247):
            if (z_coor < UP_Z):
                self.add_robot_point_to_code(x_coor, y_coor, z_coor+5) #Up before next file
            self.add_robot_point_to_code(0, 0, 0)
            self.goToNextFile()
        if (toNextFile):
            self.goToNextFile()

    def add_robot_point_to_code(self, x_coor, y_coor, z_coor, spin=0):
        """ Create palletizing robot coordinate for the point. """

        self.pathCodeList.append(";************ Start Of Cartesian Coordinate Position/Pose Point #%d***************" % (self.pointNo) )
        #x
        self.pathCodeList.append("Q%d = %s" % (self.pointNo*4+1, x_coor))
        #y
        self.pathCodeList.append("Q%d = %s" % (self.pointNo*4+2, y_coor))
        #z
        self.pathCodeList.append("Q%d = %s" % (self.pointNo*4+3, z_coor))
        #spin
        self.pathCodeList.append("Q%d = %s" % (self.pointNo*4+4, spin))
        self.pathCodeList.append(";************ End Of Cartesian Coordinate Position/Pose Point   #%d***************" % (self.pointNo) )
        self.pathCodeList.append("")
        self.pointNo += 1


    def print_all(self):
        """ Print out all generated coordinates and write ouput file. """
        for eachLine in self.pathCodeList:
            print(eachLine)
        print ("End of file, write into %s.txt ... " % self.filename)

        if not os.path.isdir('out'):
            os.mkdir('out')
        with open('out/%s.txt' % self.filename, 'w') as write_file:
            for eachLine in self.pathCodeList:
                write_file.write(eachLine)
                write_file.write('\n')


def main():
    if len(sys.argv) < 2:
        help_message()
    else:
        DBFilename = 'graphics.txt'
        wrapper = Wrapper(DBFilename)
        if (sys.argv[1] == '-s'):
            sentence = input("Sentence to write: ")
            wrapper.write_sentence(sentence)
        elif (sys.argv[1] == '-d'):
            #image_file = input("image: ")
            image_file = sys.argv[2]
            if (os.path.isfile(image_file)):
                if (len(sys.argv) == 4):
                    wrapper.draw_image(image_file, float(sys.argv[3]))
                else:
                    wrapper.draw_image(image_file)


            else:
                print ("Error: Image not exists.")
                help_message()
                return
        else:
            help_message()
            return


        #wrapper.print_all()
        pygame.display.quit()
        pygame.quit()
        sys.exit()


if __name__ == '__main__':
    main()
