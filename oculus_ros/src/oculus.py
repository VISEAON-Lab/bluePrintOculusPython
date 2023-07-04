#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

import sys
# sys.path.append("../..")
import bpHandler

import socket
import time

from sonarDisplay import warpSonar
from colormaps import ColorMap


# from marine_msgs import PingInfo
# from marine_acoustic_msgs import PingInfo

# from oculus_ros.msg import PingInfo, SonarImageData, ProjectedSonarImage

# t = PingInfo()

rospy.init_node('blueprint_oculus', anonymous=True)
polar_pub = rospy.Publisher("FLS/polar",Image, queue_size=10)
cartesian_pub = rospy.Publisher("FLS/cartesian",Image, queue_size=10) 

bridge = CvBridge()

# def callback(data):
#    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")  

#    # Here process your cv_image and then publish

#    processed_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
#    image_pub.publish(processed_image)

# image_sub = rospy.Subscriber("in_image_topic",Image,callback)




ws = warpSonar()
# warpIm = False


colormapper = ColorMap()

cv2.namedWindow('polar', 0)





try:    
      statusPort  = 52102
      tcpPort     = 52100

      udpStatusSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      udpStatusSock.bind(("", statusPort))

      # context = zmq.Context()
      M1200dTcpSock = None

      # get sonar status (and ip), as sonar init procedure...
      statusTic = time.time()
      statusCnt = 0.0

      r = rospy.Rate(1000)

      while True:
            time.sleep(0.001)
            status = bpHandler.getStatusMsg(udpStatusSock)
            if status is not None:
               statusCnt += 1
            if time.time() - statusTic >= 3:
               sps = statusCnt/(time.time()-statusTic)
               print("Status rate: %0.2fHz"%sps)
               statusTic = time.time()
               statusCnt = 0.0
            
            if (status is not None) and ('ipAddr' in status['status'].keys()):
               print("Initiate Tcp connection to: %s "%status['status']['ipAddr'])
               M1200dTcpSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
               M1200dTcpSock.connect( ("%s" %status['status']['ipAddr'], tcpPort) )
               break

            
            if rospy.is_shutdown():
                  print('exiting')
                  # exit()
                  break
            r.sleep()

      
      # Handle sonar data
      if M1200dTcpSock is not None:
            
            # init sonar values
            nBins           = 256
            pingRate        = 15    #[Hz] 
            gammaCorrection = 0xff  # 0xff -> 1 byte (0-255)
            rng             = 10    # [m] # in wide aperature up to 40[m], in narrow, up to 10[m]
            gainVal         = 60    # [%]
            sOs             = 0     # [m/s], speed of sound, 0->precalculated
            salinity        = 0     # ? (ppt}
            is16Bit         = False
            aperture        = 1     # 1-> wide, 2->low


            simpleFireMsg2 = bpHandler.createOculusFireMsg(status['hdr'], 
                                                   nBins, 
                                                   pingRate,
                                                   gammaCorrection, 
                                                   rng,
                                                   gainVal,
                                                   sOs,
                                                   salinity,
                                                   is16Bit,
                                                   aperture)

            pingTic = time.time()
            pingCnt = 0.0

            #userConfig = bpHandler.setUserConfigMsg(pingRate=0x01)
            #M1200dTcpSock.send(userConfig)

            doCahngeStatus = False
            nBeams = -1
            nRanges = -1

            dt = 0.5
            tic = time.time()-dt
            
            while True:
               time.sleep(0.001)
               if time.time() - tic >= dt:
                  M1200dTcpSock.sendall(simpleFireMsg2)
                  tic = time.time()
               sonData = bpHandler.handleOculusMsg(M1200dTcpSock)
               
               if sonData is not None and (sonData[0]['msgId']==0x23 or sonData[0]['msgId']==0x22):
                  pingCnt += 1
                  print(sonData)

                  # ping = PingInfo()
                  # ping.frequency = 0
                  # ping.sound_speed = 0
                  # ping.tx_beamwidths = 0
                  # ping.rx_beamwidths = 0
                  # print(ping)

                  # image_data = SonarImageData()
                  # print(image_data)

                  # projected_sonar_image= ProjectedSonarImage()
                  # projected_sonar_image.ping_info = ping
                  # projected_sonar_image.image = image_data
                  # print(projected_sonar_image)


                  nBeams = sonData[0]["nBeams"]
                  nRanges = sonData[0]["nRanges"]

                  cartesian = sonData[1]
                  # if warpIm:

                  cartesian = cv2.rotate(cartesian, cv2.ROTATE_180)
                  
                  polar = ws.warpSonarImage(sonData[0], sonData[1])

                  cartesian_color = colormapper.data['_inferno_data_float'][cartesian]
                  polar_color = colormapper.data['_inferno_data_float'][polar]

                  # processed_image = bridge.cv2_to_imgmsg(showIm, "bgr8")


                  cartesian_color_8bit = cv2.convertScaleAbs(cartesian_color, alpha=(255.0))
                  cartesian_msg = bridge.cv2_to_imgmsg(cartesian_color_8bit, "bgr8")
                  
                  polar_color_8bit = cv2.convertScaleAbs(polar_color, alpha=(255.0))
                  polar_msg = bridge.cv2_to_imgmsg(polar_color_8bit, "bgr8")



                  cartesian_pub.publish(cartesian_msg)
                  polar_pub.publish(polar_msg)

                  # rotate image 180 degrees

                  # save image
                  cv2.imwrite(r'/home/catkin_ws/src/bluePrintOculusPython/oculus_ros/src/image.jpg', polar_color)

                  cv2.imshow("polar", polar_color)
                  
                  #   cv2.imshow(winName, showIm)
                  key = cv2.waitKey(1)&0xff
                  #   if key==ord('h'):
                  #       print('set 512 bins')
                  #       nBins = 512
                  #       doCahngeStatus = True
                  #   elif key==ord('n'):
                  #       print('set 256 bins')
                  #       nBins = 256
                  #       doCahngeStatus = True
                  #   elif key == ord('r'):
                  #       rng += 0.5 #[m]
                  #       rng = min(40, rng)
                  #       print('set range to %f [m]'%rng)
                  #       doCahngeStatus = True
                  #   elif key == ord('f'):
                  #       rng -= 0.5 #[m]
                  #       rng = max(1, rng)
                  #       print('set range to %f [m]'%rng)
                  #       doCahngeStatus = True
                  #   elif key == ord('g'):
                  #       gainVal += 1 #[m]
                  #       gainVal = min(100, gainVal)
                  #       print('set gain to %f [%%]'%gainVal)
                  #       doCahngeStatus = True
                  #   elif key == ord('b'):
                  #       gainVal -= 1 
                  #       gainVal = max(1, gainVal)
                  #       print('set gain to %f [%%]'%gainVal)
                  #       doCahngeStatus = True
                  #   elif key==ord('z'):
                  #       is16Bit = not is16Bit
                  #       doCahngeStatus = True
                  #       print("toggle 16bit to:", is16Bit)
                  #   elif key==ord('a'):
                  #       if aperture==1:
                  #           aperture=2
                  #           print("toggle aperture to: narrow")
                  #       else:
                  #           aperture=1
                  #           print("toggle aperture to: wide")
                  #       doCahngeStatus = True
                  #   elif key==ord('q') or key == 27:
                  #       print('exit')
                  #       break 

                  
                  if doCahngeStatus:
                        doCahngeStatus = False 
                        simpleFireMsg2 = bpHandler.createOculusFireMsg(status['hdr'], 
                                                            nBins, 
                                                            pingRate,
                                                            gammaCorrection, 
                                                            rng,
                                                            gainVal,
                                                            sOs,
                                                            salinity,
                                                            is16Bit,
                                                            aperture)
                        M1200dTcpSock.sendall(simpleFireMsg2)

                  
               if time.time() - pingTic >= 3:
                  pps = pingCnt/(time.time()-pingTic)
                  print("ping rate: %0.2fHz, %d %d"%(pps, nBeams, nRanges) )
                  pingTic = time.time()
                  pingCnt = 0.0

               # ros spin once
               # rospy.sp
               if rospy.is_shutdown():
                  exit()
               r.sleep()
               

               
                  
except:           
   import traceback
   traceback.print_exc()
finally:
   if M1200dTcpSock is not None:
      print("terminate connection to sonar:tcp://%s:%s" %(status['status']['ipAddr'], tcpPort) )
      M1200dTcpSock.close() #"tcp://%s:%s" %(status['status']['ipAddr'], tcpPort) )
   
         
   
         
















# keep python from exiting until this node is stopped
rospy.spin()