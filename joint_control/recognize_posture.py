'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
from keyframes import rightBellyToStand
from os import path
from os import listdir
import pickle



class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        # Get the directory path of the current script file.
        # Construct file paths relative to the script location.
        self.dir_path = path.dirname(path.realpath(__file__))

        # Load a pre-trained classifier from a pickle file.
        # open the file in binary read mode
        self.posture_classifier = pickle.load(open(self.dir_path+'\\robot_pose.pkl','rb'))
        # self.posture_classifier = None  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        # List all the directories within the 'robot_pose_data' folder. represent the classes.
        all_classes = listdir(self.dir_path+'\\robot_pose_data')

        # Collect the relevant joint and IMU data from the 'perception' object.
        # Contains angles and readings relevant to posture perception.
        posture_perception = [
            perception.joint['LHipYawPitch'], perception.joint['LHipRoll'],
            perception.joint['LHipPitch'], perception.joint['LKneePitch'],
            perception.joint['RHipYawPitch'], perception.joint['RHipRoll'],
            perception.joint['RHipPitch'], perception.joint['RKneePitch'],
            perception.imu[0], perception.imu[1]
        ]

        # Use our loaded posture classifier to predict the posture ID based on joint and IMU data.
        # The prediction method expects a list of lists, so we provide our features within a list.
        # We then select the first element [0] as predict returns a list of predictions.
        posture_id = self.posture_classifier.predict([posture_perception])[0]

        # Retrieve the posture name using the posture ID from the classes list.
        posture = all_classes[posture_id]

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = rightBellyToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
