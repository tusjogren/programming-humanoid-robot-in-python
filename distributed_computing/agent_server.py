'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
import time
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))
from xmlrpc.server import SimpleXMLRPCServer
import threading

from inverse_kinematics import InverseKinematicsAgent


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        # Call the constructor of the parent class 'ServerAgent'
        super(ServerAgent, self).__init__()

        # Create a simple XML-RPC server bound to localhost on port 8000
        server = SimpleXMLRPCServer(("localhost", 8000), allow_none=True)

        # Register an instance of the current class (ServerAgent) to handle requests
        server.register_instance(self)

        # Register several functions to be accessible via XML-RPC
        # These functions allow remote control over certain functionalities
        server.register_function(self.set_angle, "set_angle")
        server.register_function(self.get_angle, "get_angle")
        server.register_function(self.get_posture, "get_posture")
        server.register_function(self.execute_keyframes, "execute_keyframes")
        server.register_function(self.get_transform, "get_transform")
        server.register_function(self.set_transform, "set_transform")

        # Create a new thread that runs the server's serve_forever method
        # This allows the server to handle requests in the background
        thread = threading.Thread(target=server.serve_forever)

        # Start the thread, initiating the server's request handling loop
        thread.start()

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        angle = self.perception.joint[joint_name]
        return angle
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        self.target_joints[joint_name] = angle        

    def get_posture(self):
        '''return current posture of robot'''
        posture = self.recognize_posture(self.perception)
        return posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        :param keyframes: A tuple containing the names of the motors, their corresponding times, and key values.
        '''
        # Unpack the keyframes tuple into names, times, and keys
        (names, times, keys) = keyframes

        # Calculate the duration of the longest keyframe sequence
        # It finds the latest time point at which any motor movement ends
        duration = max(map(self.last, times))

        # Store the keyframes as an attribute of the object
        self.keyframes = keyframes

        # Block the execution until the last movement is completed
        # This is done by pausing the thread for the duration of the longest keyframe sequence
        time.sleep(duration)

    def last(self, sequence):
        '''Returns the last element of a sequence
        Helper function
        '''
        return sequence[-1]

    def get_transform(self, name):
        '''get transform with given name
        '''
        transform = self.transforms[name]
        return transform

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        self.transforms[effector_name] = transform

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

