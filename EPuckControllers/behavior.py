from abc import ABC, abstractmethod


class Behavior(ABC):
    def __init__(self):
        self.resolX, self.resolY = 64, 64

    @abstractmethod
    def is_applicable(self, robot):
        pass

    @abstractmethod
    def execute(self, robot):
        pass

    def detect_box(self, image):
        """
        looks in current image for a black blob on a red background, from left to right
        :param
                resolX, resolY: int
                    image resolution in pixel
                image: PIL.Image
                    a rgb image with black blobs on red background
                xCenter: [int]
                    the center of the image: result of the function

        :return: true,  if black blob found
        """
        min_blob_width = 5
        x_start = -1
        x_center = [-1]
        for y in range(self.resolY):
            blob_width = 0
            for x in range(self.resolX):
                pixel = image.getpixel((x, y))
                if pixel == (0, 0, 0):  # black pixel: a box!
                    blob_width += 1
                    if blob_width == 1:
                        x_start = x
                else:
                    if blob_width >= min_blob_width:
                        x_center[0] = x_start + blob_width / 2
                        return True
                    elif blob_width > 0:
                        blob_width = 0
            if blob_width >= min_blob_width:
                x_center[0] = x_start + blob_width / 2
                return True

        return False


class FindPuckBehavior(Behavior):
    def is_applicable(self, robot):
        image = robot.getCameraImage()  # get camera image to find
        dist_vector = robot.getProximitySensorValues()  # get all sensor values
        x_pos = robot.getPose()[0]  # get current x position of robot

        if self.detect_box(image):
            robot.setMotorSpeeds(0, 0)
            return False
        elif dist_vector[0] < 0.04:  # enable find puck behavior if end of labyrinth is determined
            if x_pos < -0.3:
                return True
            else:
                return False
        elif 1.5 > x_pos > -0.3:  # never enable find puck behavior if still in labyrinth
            return False
        else:
            return True

    def execute(self, robot):
        robot.setMotorSpeeds(0.5, -0.5)


class HomingBehavior(Behavior):
    def is_applicable(self, robot):
        image = robot.getCameraImage()  # get camera image to find
        dist_vector = robot.getProximitySensorValues()  # get all sensor values
        x_pose = robot.getPose()[0]  # get current x position of robot

        if not self.detect_box(image):
            return False
        elif dist_vector[2] < 0.035 or dist_vector[3] < 0.035:
            return False
        elif x_pose > -0.3:  # no homing if end of labyrinth is not determined
            return False
        else:
            return True

    def execute(self, robot):
        robot.setMotorSpeeds(1, 1)


class WanderingBehavior(Behavior):
    def is_applicable(self, robot):
        return True

    def execute(self, robot):
        dist_vector = robot.getProximitySensorValues()  # get all sensor values

        if dist_vector[1] < 0.0480 and dist_vector[2] < 0.0480 or dist_vector[3] < 0.0480 or dist_vector[4] < 0.0480 and dist_vector[0] > 0.05:
            left_motor, right_motor = 1, -0.5
        elif dist_vector[1] < 0.0480:
            left_motor, right_motor = 1, 0.85
        elif dist_vector[2] < 0.05 and dist_vector[3] < 0.05:
            left_motor, right_motor = 1, 0.6
        elif dist_vector[1] > 0.05 or dist_vector[0] > 0.035:
            left_motor, right_motor = 0.45, 1
        else:
            left_motor, right_motor = 1, 1

        robot.setMotorSpeeds(left_motor, right_motor)
