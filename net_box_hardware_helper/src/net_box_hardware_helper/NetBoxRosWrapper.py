import rospy
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger,TriggerResponse
from net_box_hardware_helper import NetBoxSocket

class NetBoxRosWrapper:
    def __init__(self):
        ip_adress=rospy.get_param("~ip_adress",'192.168.1.1')
        if ip_adress=='192.168.1.1':
            rospy.logwarn("Got default parameter for "+'ip_adress')

        port=rospy.get_param("~port",49152)
        if port==49152:
            rospy.logwarn("Got default parameter for "+'port')

        self.__frame_id=rospy.get_param("~frame_id",'ft_sensor_frame')
        if self.__frame_id=='ft_sensor_frame':
            rospy.logwarn("Got default parameter for "+'frame_id')

        self.__publisher=rospy.Publisher("Wrench",WrenchStamped,queue_size=10)
        self.__set_zero_srv=rospy.Service("~set_zero",Trigger,self.__setZeroCallback__)        
       

        self.__socket=NetBoxSocket(ip_adress,port)
        self.__socket.startStreaming()          
        sample=self.__socket.getSingleSample()       
        self.__time_offset=rospy.Time.now()-rospy.Time(sample.stamp)
        self.__socket.registerReceiveCb(self.__publish__)      
        
    def __setZeroCallback__(self,req):
        self.__socket.setSoftwareBias()
        res=TriggerResponse()
        res.success=True
        res.message="Set all measurements to zero values!"
        
    
    def __publish__(self,data):
        msg=WrenchStamped()
        msg.header.frame_id=self.__frame_id
        msg.header.stamp=self.__time_offset+rospy.Time(data.stamp)
        msg.wrench.force.x=data.Fx
        msg.wrench.force.y=data.Fy
        msg.wrench.force.z=data.Fz
        msg.wrench.torque.x=data.Mx
        msg.wrench.torque.y=data.My
        msg.wrench.torque.z=data.Mz
        self.__publisher.publish(msg)
