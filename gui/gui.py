import sys

sys.path.append("./")

from rixcore.node import Node
from rixcore.subscriber import Subscriber
from rixmsg.geometry.TF import TF
from rixmsg.sensor.JS import JS

import threading
import open3d as o3d
import numpy as np
import argparse
from typing import Dict, List

from jrdf_loader import JRDFLoader
from scene import Scene

from threading import Lock

# If using WSL, run `export XDG_SESSION_TYPE=x11` in terminal first.

vis = None
robot_model = None
robot_model_mutex = Lock()


def msg_to_matrix(transform):
    # input transform contains translation and rotation
    # translation contains x, y, and z
    # rotation is quaternion that contains w, x, y, z
    # Convert transform message into 4x4 homogenous transformation matrix
    t = np.eye(4)
    t[:3, 3] = np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z]
    )
    r = o3d.geometry.get_rotation_matrix_from_quaternion(
        np.array(
            [
                transform.rotation.w,
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
            ]
        )
    )
    t[:3, :3] = r

    return t


def fk_recursive(
    link: str,
    mstack: List[np.ndarray],
    tdict: Dict[str, np.ndarray],
    cdict: Dict[str, List[str]],
    global_tf: Dict[str, np.ndarray],
) -> None:
    children = cdict.get(link)
    if not children:
        return

    base_transform = mstack[-1]
    for child in children:
        child_tf = base_transform @ tdict[child]
        mstack.append(child_tf)
        global_tf[child] = child_tf
        fk_recursive(child, mstack, tdict, cdict, global_tf)
        mstack.pop()


def tf_callback(msg: TF) -> None:
    global robot_model, vis, robot_model_mutex
    if robot_model is None or vis is None:
        return

    tdict = {}
    cdict = {}
    append_cdict = cdict.setdefault  # local binding for speed
    for transform in msg.transforms:
        parent = transform.header.frame_id
        child = transform.child_frame_id
        tdict[child] = msg_to_matrix(transform.transform)
        append_cdict(parent, []).append(child)

    global_tf = {}
    mstack = [np.eye(4)]
    fk_recursive("world", mstack, tdict, cdict, global_tf)

    with robot_model_mutex:
        robot_model.update_pose(global_tf, vis)


def main(args):
    global robot_model, vis

    node = Node.create("gui", (args.ip, 48104))
    if not node.ok():
        print("Failed to initialize node")
        return

    sub = node.create_subscriber(TF, "/tf", tf_callback)
    if sub is None:
        print("Failed to subscribe")
        node.shutdown()
        return

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Import Scene Class
    scene = Scene(vis)

    loader = JRDFLoader()
    robot_model = loader.load(args.jrdf, vis)

    thr = threading.Thread(target=lambda: node.spin())
    thr.start()

    def animation_callback(vis):
        global robot_model, robot_model_mutex
        visual = robot_model.get_visuals()
        for _, meshes in visual.items():
            for mesh in meshes:
                vis.update_geometry(mesh)

    try:
        vis.register_animation_callback(animation_callback)
        vis.run()
    except KeyboardInterrupt as e:
        pass

    node.shutdown()
    vis.destroy_window()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Open3D Robot Model Viewer")
    parser.add_argument("jrdf", type=str, help="Path to the robot model JSON file")
    parser.add_argument(
        "-i", "--ip", type=str, default="127.0.0.1", help="IP Address of Rixhub process"
    )
    args = parser.parse_args()

    main(args)
