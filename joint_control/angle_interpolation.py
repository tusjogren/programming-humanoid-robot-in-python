'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import wipe_forehead
from keyframes import leftBackToStand

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.startTime = -1

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        # target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        # If keyframes are empty, return empty target joints
        if self.keyframes == ([], [], []):
            return {}
        
        # Initialize the start time if not already set
        if self.startTime == -1:
            self.startTime = perception.time

        elapsed_time = perception.time - self.startTime
        names, times, keys = keyframes
        target_joints = {}
        joints_skipped = 0

        # Helper function to calculate angle. I am using cubic Bezier curve equation
        def calculate_bezier_angle(i, p0, p1, p2, p3):
            return ((1 - i) ** 3) * p0 + 3 * i * ((1 - i) ** 2) * p1 + 3 * (i ** 2) * (1 - i) * p2 + (i ** 3) * p3
        
        # Iterate through each joint and interpolate its angle
        for idx, joint_name in enumerate(names):
            joint_times = times[idx]
            
            if joint_times[-1] < elapsed_time: # Skip if the last keyframe time is before the elapsed time
                joints_skipped += 1
                if joints_skipped == len(names):
                    self.startTime = -1
                    self.keyframes = ([], [], [])
                continue

            # Find the correct keyframe for the elapsed time
            min_time, max_time = 0, 0
            kf_idx = 0
            for n, joint_time in enumerate(joint_times):
                max_time = joint_time
                if min_time <= elapsed_time <= max_time:
                    kf_idx = n
                    break
                min_time = max_time

            # Calculate interpolation factor
            i = (elapsed_time - min_time) / (max_time - min_time)
            
            # Determine Bezier control points
            if kf_idx == 0:
                p0, p1 = 0, 0
                p3 = keys[idx][kf_idx][0]
                p2 = p3 + keys[idx][kf_idx][1][2]
            else:
                p0 = keys[idx][kf_idx - 1][0]
                p1 = p0 + keys[idx][kf_idx - 1][2][2]
                p3 = keys[idx][kf_idx][0]
                p2 = p3 + keys[idx][kf_idx][1][2]

            # Compute interpolated angle
            angle = calculate_bezier_angle(i, p0, p1, p2, p3)
            
            target_joints[joint_name] = angle
            if joint_name == "LHipYawPitch":
                target_joints["RHipYawPitch"] = angle # LHipYawPitch and RHipYawPitch share the same angle

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    # agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    # agent.keyframes = wipe_forehead("")
    agent.keyframes = leftBackToStand()
    agent.run()
