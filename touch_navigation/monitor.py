import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Twist, Point, PointStamped
from cv_bridge import CvBridge, CvBridgeError


import cv2


class CameraMonitor(Node):

    def __init__(self):
        super().__init__('camera_monitor')

        self.pointCloud_subscriber = self.create_subscription(
            PointCloud2, '/intel_realsense_r200_depth/points', self.pointCloud_callback, 0)
        self.camera_subscriber = self.create_subscription(
            Image, '/intel_realsense_r200_depth/image_raw', self.camera_callback, 0)
        self.depth_subscriber = self.create_subscription(
            Image, '/intel_realsense_r200_depth/depth/image_raw', self.depth_callback, 0)
        self.point_publisher = self.create_publisher(
            PointStamped, '/monitor_point', 10)
        self.teleop_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # These variables assigned after
        self.rgb_camera_dimensions: list = [1920, 1080]
        self.depth_camera_dimensions: list = []
        self.teleop = Teleop(self)
        self.bridge_object = CvBridge()
        self.savedsceneObjects = TimedFunctionCache()
        self.pointCloud = PointCloud2()
        self.lastFoundPoint = Point()

    def camera_callback(self, msg: Image):
        cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
        # Configuring size of screen elements and scaling ratios
        self.rgb_camera_dimensions = [msg.width, msg.height]
        self.savedsceneObjects.width = self.rgb_camera_dimensions[0]
        self.savedsceneObjects.height = self.rgb_camera_dimensions[1]

        try:
            self.rgb_image = self.bridge_object.imgmsg_to_cv2(msg)
            # Couldn't scale image , it brokes image data array
            # self.rgb_image = cv2.resize(self.rgb_image, (self.rgb_camera_dimensions[0], self.rgb_camera_dimensions[1]))
        except CvBridgeError as e:
            rclpy.logging.LoggingSeverity(40)
            rclpy.logging.get_logger("OpenCV").warn(e)

        cycleScene(self)

        cv2.setMouseCallback("Image", mousePoints, self)

        # For perveting flickering
        self.savedsceneObjects.add('fake', cv2.rectangle, ((0, 0),
                                                           (0, 0), (0, 0, 0), 0))

        # Executing screen element functions
        for timedfunc in self.savedsceneObjects.arr:
            if timedfunc.finnish == -1:
                timedfunc.finnish = float('inf')

            # Remove dld functions
            if self.savedsceneObjects.i > timedfunc.finnish:
                if (timedfunc.finnish - timedfunc.start > 5):
                    # print(str(timedfunc.type) + " timed out on " + str(self.savedsceneObjects.i))

                    self.savedsceneObjects.arr.remove(timedfunc)

            if self.savedsceneObjects.i > timedfunc.start and self.savedsceneObjects.i <= timedfunc.finnish:
                # print()
                # print("Start: " + str(timedfunc.start ))
                # print("Now: " + str(self.savedsceneObjects.i))
                # print("Finnish: " + str(timedfunc.finnish))
                timedfunc.func(self.rgb_image, *timedfunc.args)

        cv2.imshow("Image", self.rgb_image)

        self.savedsceneObjects.i += 1

        key = cv2.waitKey(1)
        self.teleop.changeVelocity(key)

    def depth_callback(self, msg: Image):
        try:
            self.depth_image = self.bridge_object.imgmsg_to_cv2(msg)

            # Couldn't scale image, it brokes image data array
            # self.depth_image = cv2.resize(self.depth_image, (self.rgb_camera_dimensions[0], self.rgb_camera_dimensions[1]))

        except CvBridgeError as e:
            rclpy.logging.LoggingSeverity(40)
            rclpy.logging.get_logger("OpenCV").warn(e)

        # cv2.imshow("Depth_Image", self.depth_image)

    def pointCloud_callback(self, msg: PointCloud2):
        pointCloudData = pc2.read_points(
            msg, field_names=("x", "y", "z", "rgb"), reshape_organized_cloud=True)

        # Filter out invalid depths
        pointCloudData = filter(
            lambda point: point['z'] != 100 and point['z'] > 0, pointCloudData)

        self.pointCloud = pointCloudData


class TimedFunctionCache:
    '''Class for managing seen screen elements'''
    #!!!!! COLORS CONFIGURED AS BGR
    class timedFunction:
        def __init__(self, typ: str, func, args, start: int, finnish: int, order):
            self.type = typ
            self.func = func
            self.args = args
            self.start = start
            self.finnish = finnish
            self.order = order

        def run_(self):
            self.func(*self.args)

    def __init__(self, width=1920, height=1080):
        self.height = height
        self.width = width
        self.order = 0
        self.arr = list()
        self.i = 1

    def add(self, typ: str, function, arguments, start_bonus: int = 0, life: int = -1):
        self.order += 1
        start = self.i + start_bonus
        finnish = float('inf') if life == -1 else self.i + start_bonus + life

        timedFunc = self.timedFunction(
            typ, function, arguments, start=start, finnish=finnish, order=self.order)

        self.arr.append(timedFunc)

    def remove(self, wanted_type: str):
        for obj in self.arr:
            if obj.type == wanted_type:
                self.arr.remove(obj)
                return True
        return False

    def run(self, image):
        for timedfunc in self.arr:

            if timedfunc.finnish == -1:
                timedfunc.finnish = float('inf')

            # Remove Old function
            if self.i > timedfunc.finnish:
                # print(str(timedfunc.typ) + " timed out on " + str(self.i))
                self.arr.remove(timedfunc)

            if self.i > timedfunc.start and self.i <= timedfunc.finnish:

                '''
                print()  
                print("Start: " + str(timedfunc.start ))
                print("Now: " + str(self.i))
                print("Finnish: " + str(timedfunc.finnish))
                '''

                timedfunc.run_()

        cv2.imshow("Image", image)


def mousePoints(event, x, y, flags, params: CameraMonitor):

    if event == cv2.EVENT_LBUTTONDOWN:
        pass

    if event == cv2.EVENT_RBUTTONDOWN:
        publishRelativePos(params, x, y)


def publishRelativePos(obj: CameraMonitor, x, y):
    obj.lastFoundPoint = calculateRelativePositon(obj, x, y)
    if obj.lastFoundPoint != Point(x=0.0, y=0.0, z=0.0):
        obj.savedsceneObjects.remove('last_point')
        msg = PointStamped()
        header = Header()
        header.stamp = obj.get_clock().now().to_msg()
        header.frame_id = 'base_link'
        msg.header = header
        msg.point = obj.lastFoundPoint
        obj.point_publisher.publish(msg)


def cycleScene(object: CameraMonitor):
    '''
    Some coordinands are hard codded
    '''
    w = object.savedsceneObjects.width
    h = object.savedsceneObjects.height

    # Table Border
    object.savedsceneObjects.add('info_table_bg_border', cv2.rectangle, ((w - 16*25, h - 9*25),
                                                                         (w - 10, h - 10), (59, 34, 34), -1))
    # Table
    object.savedsceneObjects.add('info_table_bg', cv2.rectangle, ((w - 392, h - 221),
                                                                  (w - 10 - 8, h - 10 - 8), (105, 78, 74), -1))
    # CrossAir
    object.savedsceneObjects.add(
        "point", cv2.circle, ((w//2, h//2), 2, (255, 255, 255), -1), 0, 30)
    # Linear speed text
    object.savedsceneObjects.add('linear_speed', cv2.putText, (f'Linear Speed: {round(object.teleop.msg.linear.x,2)}m/s', (
        w - 16*24, h - 9*22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (228, 233, 242), 1), life=1)
    # Angular speed text
    object.savedsceneObjects.add('angular_speed', cv2.putText, (f'Angular Speed: {round(object.teleop.msg.angular.z,2)}m/s', (
        w - 16*24, h - 9*20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (228, 233, 242), 1), life=1)
    # Last found point
    coord_str = f'Last Found Point x: {round(object.lastFoundPoint.x,2)}m y: {round(object.lastFoundPoint.y,2)}m z: {round(object.lastFoundPoint.z,2)}m'
    object.savedsceneObjects.add('last_point', cv2.putText, (coord_str, (
        object.savedsceneObjects.width - 16*24, object.savedsceneObjects.height - 9*3), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (228, 233, 242), 1), life=1)


class Teleop():
    def __init__(self, obj: CameraMonitor):
        self.publisher = obj.teleop_publisher
        self.msg = Twist()
        self.linear_max = 0.26
        self.angular_max = 1.82
    pass

    def changeVelocity(self, key):

        if key == 115:  # s
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
        if self.msg.linear.x < self.linear_max:
            if key == 119:  # w
                self.msg.linear.x += 0.1
            if key == 120:  # x
                self.msg.linear.x -= 0.1

        if self.msg.angular.z < self.angular_max:
            if key == 97:  # a
                self.msg.angular.z += 0.1

            if key == 100:  # d
                self.msg.angular.z -= 0.1

        self.publisher.publish(self.msg)


def calculateRelativePositon(nodeObj: CameraMonitor, x_inPixel: int, y_inPixel: int) -> Point:
    foundArr = []
    depth = round(nodeObj.depth_image[y_inPixel][x_inPixel].item(), 4)

    # Changing origin
    w = nodeObj.savedsceneObjects.width
    h = nodeObj.savedsceneObjects.height
    x_inPixel_normalized = round(x_inPixel - w / 2)
    y_inPixel_normalized = round(-1*(y_inPixel - h / 2))

    # Search pointcloud for matching depths
    rclpy.logging.get_logger("Coordinates").info(
        f'Searching on x:{x_inPixel_normalized} y:{y_inPixel_normalized}')
    for point in nodeObj.pointCloud:
        point = Point(
            x=round(point['x'].item(), 4),
            y=-1*round(point['y'].item(), 4),
            z=round(point['z'].item(), 4)
        )

        if point.z == depth:
            foundArr.append(point)

    # Array has no depth matching points
    if len(foundArr) == 0:
        # Draw circle for point that didn't found
        nodeObj.savedsceneObjects.add(
            "invalid_point", cv2.circle, ((x_inPixel, y_inPixel), 5, (0, 0, 255), -1), 0, 30)
        rclpy.logging.get_logger("Coordinates").info(
            f'Couldn\'t found the coordinants on x:{x_inPixel_normalized} y:{y_inPixel_normalized}')
        return Point(x=0.0, y=0.0, z=0.0)

    for point in foundArr:
        # Ratios for comparison
        pxRatio = round(x_inPixel_normalized/y_inPixel_normalized, 1)
        mRatio = round(point.x/point.y, 1)

        if pxRatio == mRatio:
            # Draw circle for point
            nodeObj.savedsceneObjects.add(
                "point", cv2.circle, ((x_inPixel, y_inPixel), 5, (255, 255, 255), -1), 0, 30)

            # Display coordinates of found point
            coord_str = f'x: {point.x}m y: {point.y}m z: {point.z}m'
            nodeObj.savedsceneObjects.add(
                "point_coord", cv2.putText, (coord_str, (x_inPixel+10, y_inPixel), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 1), 0, 30)

            rclpy.logging.get_logger("Coordinates").info(
                'Found on ' + coord_str)
            return point

    # Draw circle for point that didn't found
    nodeObj.savedsceneObjects.add(
        "invalid_point", cv2.circle, ((x_inPixel, y_inPixel), 5, (0, 0, 255), -1), 0, 30)
    rclpy.logging.get_logger("Coordinates").info(
        f'Couldn\'t found the coordinants on x:{x_inPixel_normalized} y:{y_inPixel_normalized}')
    return Point(x=0.0, y=0.0, z=0.0)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = CameraMonitor()

    try:
        rclpy.spin(minimal_subscriber)
    except SystemExit:
        cv2.destroyAllWindows()
        rclpy.logging.get_logger("Quitting").info('Done')

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
