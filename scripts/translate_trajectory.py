import os, sys

import numpy as np
import yaml


class PoseWithRotation:
    def __init__(self, time, translation_vec, rotation_mat):
        self.timestamp = time
        self.translation = translation_vec
        self.rotation = rotation_mat

    def print(self):
        print("PoseWithRotation")
        print(self.timestamp)
        print(self.translation)
        print(self.rotation)


class PoseWithQuaternion:
    def __init__(self, time, trans_vec, quat_vec):
        self.timestamp = time
        self.translation = trans_vec
        self.quat = quat_vec

    def print(self):
        print("PoseWithQuaternion")
        print(self.timestamp)
        print(self.translation)
        print(self.quat)

def rot2quat(rot):
    trace = np.trace(rot)

    # 3次元回転：p.30
    if trace < -1:
        if abs(trace+1) < 0.01:
            trace = -1
        else:
            print("translate_trajectory.py rot2quat() ERROR trace+1 = {}".format(trace+1) )

    qw = (np.sqrt(1+trace)) / 2
    if qw==0:
        qw = 1e-6
    qx = - (rot[1,2] - rot[2,1]) / (4*qw) 
    qy = - (rot[2,0] - rot[0,2]) / (4*qw)
    qz = - (rot[0,1] - rot[1,0]) / (4*qw)

    norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    qw /= norm
    qx /= norm
    qy /= norm
    qz /= norm

    # print("rot2quat")
    # print(trace, norm)
    # print(rot)
    # print(qx, qy, qz, qw)
    # print(np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw))
    return qx, qy, qz, qw

def quat2rot(qx, qy, qz, qw):
    r00 = qw*qw + qx*qx - qy*qy - qz*qz
    r01 = 2 * (qx*qy - qw*qz)
    r02 = 2 * (qx*qz + qw*qy)
    r10 = 2 * (qy*qx + qw*qz)
    r11 = qw*qw - qx*qx + qy*qy - qz*qz
    r12 = 2 * (qy*qz - qw*qx)
    r20 = 2 * (qx*qz - qw*qy)
    r21 = 2 * (qy*qz + qw*qx)
    r22 = qw*qw - qx*qx - qy*qy + qz*qz

    rot = np.array([
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22]
    ])

    return rot

def poseWithRotation2poseWithQuaternion(pose_with_rotation: PoseWithRotation):
    trans = pose_with_rotation.translation
    quat = rot2quat(pose_with_rotation.rotation)

    pose_with_quaternion = PoseWithQuaternion(pose_with_rotation.timestamp, trans, quat )

    return pose_with_quaternion

def poseWithQuaternion2poseWithRotation(pose_with_quaternion: PoseWithQuaternion):
    trans = pose_with_quaternion.translation

    quat = pose_with_quaternion.quat
    rot = quat2rot(quat[0], quat[1], quat[2], quat[3])

    pose_with_rotation = PoseWithRotation(pose_with_quaternion.timestamp, trans, rot)
    return pose_with_rotation


def translate_pose(pose_icc_quat: PoseWithQuaternion, transform_rc):
    transform_cr = np.linalg.inv(transform_rc)
    transform_cr_rot = transform_cr[:3,:3]
    transform_cr_trans = transform_cr[:3,3]

    transform_rc_rot = transform_rc[:3,:3]
    transform_rc_trans = transform_rc[:3,3]

    pose_icc = poseWithQuaternion2poseWithRotation(pose_icc_quat)
    # pose_icc_rot.print()

    # print("transform_rc_rot", transform_rc_rot)
    # print("transform_rc_trans", transform_rc_trans)
    # print("transform_cr_rot", transform_cr_rot)
    # print("transform_cr_trans", transform_cr_trans)

    rot_irc = transform_rc_rot @ pose_icc.rotation
    trans_irc = np.dot(transform_rc_rot , pose_icc.translation) + transform_rc_trans

    rot_irr = rot_irc @ transform_cr_rot
    trans_irr = np.dot(rot_irc , transform_cr_trans) + trans_irc

    pose_irr = PoseWithRotation(pose_icc_quat.timestamp, trans_irr, rot_irr)
    # pose_irc_rot.print()
    
    pose_irr_quat = poseWithRotation2poseWithQuaternion(pose_irr)
    # pose_irc_quat.print()

    return pose_irr_quat


def read_transform_matrix(path, mode):
    if not os.path.exists(path):
        print("[read_transform_matrix()] path does not exist. {}".format(path))

    if mode=="yaml":
        print("yaml mode")
        with open(path, 'r') as yml:
            config = yaml.load(yml, Loader=yaml.FullLoader)

        transform_rc = np.array(config["transform_rc"])

    elif mode=="txt":
        print("txt mode")
        with open(path, "r") as f:
            lines = [l.split("\n")[0] for l in f.readlines()]

        transform_rc = list()
        for i in range(4):
            row = [float(v) for v in lines[i].split(", ")]
            transform_rc.append(row)
        
        transform_rc = np.array(transform_rc)
    else:
        print("[read_transform_matrix()] invalid format: {}".format(path))
        return 

    print("transform_rc")
    print(transform_rc)

    return transform_rc

def main(tum_traj_path, transform_path, out_path):
    # 軌跡を読む
    with open(tum_traj_path, "r") as f:
        lines = [l.split("\n")[0] for l in f.readlines()]

    traj_list = []
    for l in lines:
        row = [float(v) for v in l.split(" ")]
        pose = PoseWithQuaternion(row[0], np.array([row[1], row[2], row[3]]), np.array([row[4], row[5], row[6], row[7]]))
        traj_list.append(pose)
    
    # 変換行列を読む
    ext = os.path.splitext(transform_path)[1][1:]
    transform_rc = read_transform_matrix(transform_path, ext)

    transformed_traj_list = []
    for pose_quat in traj_list:
        transformed_pose_quat = translate_pose(pose_quat, transform_rc)

        transformed_traj_list.append(transformed_pose_quat)

    # 変換後の軌跡をファイル出力
    with open(out_path, mode="w") as f:
        for pose_quat in transformed_traj_list:
            timestamp = pose_quat.timestamp
            t = pose_quat.translation
            q = pose_quat.quat
            
            line = "{:0.15g} {:0.9g} {:0.9g} {:0.9g} {:0.9g} {:0.9g} {:0.9g} {:0.9g}\n".format(timestamp, t[0],t[1],t[2],q[0],q[1],q[2], q[3])

            f.write(line)

def test_read_transform_matrix(yaml_path, ext_param_path):
    read_transform_matrix(yaml_path, "yaml")
    read_transform_matrix(ext_param_path, "txt")

if __name__=="__main__":
    if len(sys.argv) == 4:
        in_path = sys.argv[1]
        yaml_path = sys.argv[2]
        out_path = sys.argv[3]
    else:
        in_path = "/home/kitajima/catkin_ws/src/se2c_rgbdw_slam/log/frame_trajectory.txt"
        out_path = "/home/kitajima/catkin_ws/src/se2c_rgbdw_slam/log/frame_trajectory_transformed.txt"
        yaml_path = "/home/kitajima/catkin_ws/src/se2c_rgbdw_slam/data/NakBot/kinect2_new.yaml"
    
    main(in_path, yaml_path, out_path)