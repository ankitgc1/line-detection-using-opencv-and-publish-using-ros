import cv2
import numpy as np
import rospy
from std_msgs.msg import Float32

cap = cv2.VideoCapture(0)

while(cap.isOpened()):
    _, img = cap.read()
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,50,150,apertureSize =3)
    lines = cv2.HoughLines(edges,1,np.pi/180,10)
    if lines is None:
        contonous
    else:
        for r,theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*r
            y0 = b*r
            x1 = int(x0+1000*(-b))
            y1 = int(y0+1000*(a))
            x2 = int(x0-1000*(-b))
            y2 = int(y0-1000*(a))
            cv2.line(img,(x1,y1),(x2,y2),(0,0,255),25)
            slope = theta
            cv2.imshow('image', img)
            pub = rospy.Publisher('chatter', Float32, queue_size=10)
            rospy.init_node('talker', anonymous=True)
            rate = rospy.Rate(10)
            rospy.loginfo(slope)
            pub.publish(slope)
            #rate.sleep()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        video()
    except rospy.ROSInterruptException:
        pass
