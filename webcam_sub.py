# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Colour segementation and connected components example added by Claude sSammut
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from visualization_msgs.msg import Marker 
from std_msgs.msg import String



class ImageSubscriber(Node):
  


  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    self.minSize = 200
    self.seenColors = []
    self.allowedDifference = 20
    super().__init__('image_subscriber')
    #self.publisher_ = self.create_publisher(Marker, '/visualization_marker', 10)
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
 #    'video_frames', 
      '/camera/image_raw/uncompressed', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    
    #super().__init__('blob_points') only call once. Unrelated to publishers and subscribers

    self.blob_publisher = self.create_publisher(String, 'blob_points', 10)
    self.blob_publisher
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
    print("INSIDE THE CALLBACK")
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)

  # Logic : -------------------------------------------------------------------
  # Check for any pink blob above threshold
  # If found, place marker for largest blob (only one marker placed per frame)
  # ----

  # Scan image for all pink blobs
  # If detect a pink blob, then save the largest blob's centroid
  # Otherwise return

  # If successful, check the other masks (green, blue, yellow)
  # Find blob that lines up with pink
  # Check if above or below pink centroid
  # Check if combo is already in list
  # If any fail, dont place marker

  # Add combo to list
  # Send marker via publisher
  # ---------------------------------------------------------------------------
 
  # +======+
  # | CODE |
  # +======+
  
  # Display the message on the console
  # self.get_logger().info('Receiving video frame')

  # Convert ROS Image message to OpenCV image
  # current_frame = self.br.imgmsg_to_cv2(data)
  
  # Convert BGR image to HSV
    hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
  
  
  # ---- Scan image for all pink blobs ---- #
  # Mask out everything except pixels in the range
    filter_lower = (130, 50, 50)
    filter_upper = (175, 255, 255)
    mask = cv2.inRange(hsv_frame, filter_lower, filter_upper)
    result = cv2.bitwise_and(current_frame, current_frame, mask=mask)
    #cv2.imshow("camera", result)
  # Run 4-way connected components, with statistics
    output = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)
    (numLabels, labels, stats, centroids) = output
    
    print(self.minSize)
    print(self.seenColors)
  # Get largest blob
    li = -1
    for i in range(1, numLabels):
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        area = stats[i, cv2.CC_STAT_AREA]
        if (area >= self.minSize):
          if li == -1 or area > stats[li, cv2.CC_STAT_AREA]:
            li = i
            print("New biggest is " + str(i) + " with area = " + str(area))
            print(x, y, w, h, area)

  # No largest blob found
    if li == -1:
      print("No blob found")
      return 0
  # Save pink stats
    pinkStats = stats[li]
    
    
    trackedColor = ""
  # Pink blob found, check the other masks (green, blue, yellow)
    colors = ("Blue", "Green", "Yellow")
    filters_lower = ((95, 80, 150), (60, 50, 50), (20, 150, 150))
    filters_upper = ((108, 255, 255), (90, 255, 255), (30, 255, 255))
  # Loop through colors
    for ci in range(0, 3):
      #print(colors[ci])
    # Mask out everything except pixels in the range
      mask = cv2.inRange(hsv_frame, filters_lower[ci], filters_upper[ci])
      result = cv2.bitwise_and(current_frame, current_frame, mask=mask)
    # Run 4-way connected components, with statistics
      output = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)
      (numLabels, labels, stats, centroids) = output
    
    # Try to match current color blob to pink blob
      for i in range(1, numLabels):
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
      # Check if area is large enough
        area = stats[i, cv2.CC_STAT_AREA]
        print(x, y, w, h, area)
        if (area >= self.minSize):
        # Check if similar enough position in frame (compare x)
          print(x, y, w, h, area)
          if abs(pinkStats[cv2.CC_STAT_LEFT] - x) < self.allowedDifference:
            print(colors[ci] + " matches pink's position")
            # Find what color is ontop
            
            if pinkStats[cv2.CC_STAT_TOP] < y:
              trackedColor = "Pink;" + colors[ci]
            else:
              trackedColor = colors[ci] + ";Pink"
            break
          

  # Check if a color match could be made OR if matched color is already in list
    print("The tracked color is " + trackedColor)
    if trackedColor == "" or trackedColor in self.seenColors:
      return 0
  # If new, add color to tracking list
    self.seenColors.append(trackedColor)
    
  # Send info to marker making node
  # INFO NEEDED
  #   - pinkStats <- used to get position of blob and get angle for placement
  #   - trackedColor <- used to determine what maker colors are used in placement code
	#angle
    hFOV = 62.2
    _, original_width, _ = current_frame.shape
    print(original_width)
    print(pinkStats)
    distanceFromCenter = (pinkStats[cv2.CC_STAT_LEFT] + pinkStats[cv2.CC_STAT_WIDTH]/2) - original_width/2
    print("Distance = " + str(distanceFromCenter))
    ratioFromCenter = distanceFromCenter/(original_width/2)
    resultAngle = ratioFromCenter * (hFOV/2)
    print(resultAngle)	

    blob_coordinates = str(str(trackedColor) + ";" + str(resultAngle))
    msg = String()
    msg.data = blob_coordinates

    print(f"Publishing blob coordinates: {blob_coordinates}")
    self.blob_publisher.publish(msg)
    
    #cv2.waitKey(0)
    return 1

def main(args=None):
  
  # Initialize the rclpy library
  print("In  main")
  
  rclpy.init(args=args)
  print("Innit done  main")
  
  # Create the node
  image_subscriber = ImageSubscriber()
  print("Image sub node made")
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
