import numpy as np
import pytransform3d.visualizer as pv
import pytransform3d.trajectories as ptr
from movement_primitives.kinematics import Kinematics
import rosbag
from tf.transformations import quaternion_matrix
from movement_primitives.dmp import CartesianDMP
import pickle
import os
import time
from scipy.interpolate import interp1d
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf
import matplotlib.pyplot as plt


# pip install -U langchain langchain-community langchain-core langchain-text-splitters langchain-ollama chromadb beautifulsoup4 tiktoken pypdf lxml matplotlib
import os
import glob
from typing import List, Union, Optional

from langchain_community.document_loaders import WebBaseLoader, BSHTMLLoader, PyPDFLoader
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_ollama import OllamaEmbeddings 
from langchain_community.vectorstores import Chroma
from langchain_core.output_parsers import StrOutputParser
from langchain.prompts import ChatPromptTemplate
from langchain_ollama import ChatOllama
from langchain_core.runnables import RunnablePassthrough
from langchain_core.documents import Document
import sys # Add this import
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from sensor_msgs.msg import JointState


# ------------------------------------------- Functions --------------------------------------------
import re

current_script_dir = os.path.dirname(os.path.abspath(__file__))
my_scripts_dir = "/root/catkin_ws/src/my_scripts"
if my_scripts_dir not in sys.path:
    sys.path.insert(0, my_scripts_dir)
# --- End Path Setup ---

from assignment_2.dmp_controller import *
# Set User-Agent (recommended to avoid potential blocking and for identification)
os.environ["USER_AGENT"] = "MyRAGSystem/0.1 (e11806417@student.tuwien.ac.at or project-url)"

def load_documents(sources: List[str]) -> List[Document]:
    """
    Load documents from multiple sources, which can be either URLs or local file paths.
    
    Args:
        sources: List of URLs or local file paths
        
    Returns:
        List of loaded documents
    """
    all_documents = []
    
    for source in sources:
        try:
            if source.startswith(('http://', 'https://')):
                # Handle web URLs
                print(f"Loading web source: {source}")
                if source.endswith('.pdf'):
                    # Handle PDF URLs
                    loader = PyPDFLoader(source)
                else:
                    # Handle regular web pages
                    loader = WebBaseLoader(source)
                documents = loader.load()
                print(f"Loaded {len(documents)} document(s) from {source}")
            else:
                # Handle local files
                if os.path.isfile(source):
                    if source.endswith('.html'):
                        print(f"Loading local HTML file: {source}")
                        loader = BSHTMLLoader(source)
                        documents = loader.load()
                    elif source.endswith('.pdf'):
                        print(f"Loading local PDF file: {source}")
                        loader = PyPDFLoader(source)
                        documents = loader.load()
                    else:
                        print(f"Unsupported file type: {source}")
                        continue
                    print(f"Loaded {len(documents)} document(s) from {source}")
                elif os.path.isdir(source):
                    # Handle directory of HTML and PDF files
                    print(f"Loading HTML/PDF files from directory: {source}")
                    html_files = glob.glob(os.path.join(source, "*.html"))
                    pdf_files = glob.glob(os.path.join(source, "*.pdf"))
                    documents = []
                    
                    # Process HTML files
                    for html_file in html_files:
                        print(f"  Loading HTML: {html_file}")
                        loader = BSHTMLLoader(html_file)
                        documents.extend(loader.load())
                    
                    # Process PDF files
                    for pdf_file in pdf_files:
                        print(f"  Loading PDF: {pdf_file}")
                        loader = PyPDFLoader(pdf_file)
                        documents.extend(loader.load())
                        
                    print(f"Loaded {len(documents)} document(s) from directory {source}")
                else:
                    print(f"Unsupported file type or not found: {source}")
                    continue
            
            all_documents.extend(documents)
        except Exception as e:
            print(f"Error loading from {source}: {e}")
    
    return all_documents

def decode_move_tag(tag: str) -> dict:
    """
    Decodes a Tower of Hanoi move tag into a dictionary.

    Args:
        tag: The move tag string (e.g., "MD1AC").

    Returns:
        A dictionary with 'disk', 'start', and 'end' keys.
        Returns None if the tag format is invalid.
    """
    match = re.fullmatch(r"MD(\d+)([A-C])([A-C])", tag)
    if match:
        disk_number = int(match.group(1))
        source_peg = match.group(2)
        destination_peg = match.group(3)
        return {"cube": disk_number, "start": source_peg, "end": destination_peg}
    else:
        print(f"Warning: Invalid tag format encountered: {tag}")
        return None




# ------------------------------------------- DMP Motion Generator for Gazebo --------------------------------------------

class DMPMotionGenerator:
    def __init__(self, urdf_path, mesh_path=None, joint_names=None, base_link="world", end_effector_link="end_effector_link"):
        print("Initializing DMPMotionGenerator for Gazebo...")
        self.urdf_path = urdf_path
        self.mesh_path = mesh_path
        self.kin = self._load_kinematics(urdf_path, mesh_path)
        
        self.joint_names = joint_names or ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.gripper_joint_names = ["gripper", "gripper_sub"]
        self.base_link = base_link
        self.end_effector_link = end_effector_link
        self.chain = self.kin.create_chain(self.joint_names, base_link, end_effector_link)
        self.dmp = None
        self.IK_joint_trajectory = None
        self.gripper_trajectory = None
        
        if not rospy.core.is_initialized():
            rospy.init_node('dmp_motion_generator', anonymous=True)

    def _load_kinematics(self, urdf_path, mesh_path=None):
        with open(urdf_path, 'r') as f:
            return Kinematics(f.read(), mesh_path=mesh_path)

    def learn_from_rosbag(self, bag_path, joint_topic, dt=None, n_weights=10):
        transforms, joint_trajectory, gripper_trajectory, time_stamp = self._process_rosbag(bag_path, joint_topic)
        
        # Remove first 5 % and last 5% of trajectory
        print(f"Shapes of transforms, joint_trajectory, gripper_trajectory: {transforms.shape}, {joint_trajectory.shape}, {gripper_trajectory.shape}")
        start_idx = int(0.05 * len(gripper_trajectory))
        end_idx = int(0.95 * len(transforms))
        transforms = transforms[start_idx:end_idx, :, :]
        joint_trajectory = joint_trajectory[start_idx:end_idx, :]
        gripper_trajectory = gripper_trajectory[start_idx:end_idx]
        time_stamp = time_stamp[start_idx:end_idx] - time_stamp[start_idx]
        self.gripper_trajectory = gripper_trajectory
        
        print(f"Transforms shape: {transforms.shape}")
        Y = ptr.pqs_from_transforms(transforms[10:,:,:])
        if dt is None:
            dt = 1/self.frequency
        self.dmp = CartesianDMP(execution_time=max(time_stamp), dt=dt, n_weights_per_dim=n_weights)
        self.dmp.imitate(time_stamp[10:], Y)
        
        return Y, transforms, joint_trajectory, gripper_trajectory

    def _process_rosbag(self, bag_path, joint_topic):
        transforms = []
        joint_trajectory = []
        gripper_trajectory = []
        time_stamp = []
        
        print(f"Reading bag file: {bag_path}")
        bag = rosbag.Bag(bag_path)
        for topic, msg, t in bag.read_messages(topics=[joint_topic]):
            joint_pos = msg.position[:6]
            gripper_pos = msg.position[6] if len(msg.position) > 6 else 0.0
            joint_trajectory.append(joint_pos)
            gripper_trajectory.append(gripper_pos)

            transforms.append(self.chain.forward(joint_pos))
            time_stamp.append(msg.header.stamp.to_sec())    
        bag.close()
        
        transforms = np.array(transforms)
        joint_trajectory = np.array(joint_trajectory)
        gripper_trajectory = np.array(gripper_trajectory)
        time_stamp = np.array(time_stamp)
        
        dt = []
        for i in range(1, time_stamp.shape[0]):
            dt.append(time_stamp[i]- time_stamp[i-1])
        self.frequency = 1/ np.average(np.array(dt))
        
        positions = np.array([T[:3, 3] for T in transforms])
        mask, _ = self.remove_outliers_mad(positions, threshold=5.0)
        
        filtered_time = time_stamp[mask]
        normalized_time = filtered_time - filtered_time[0]
        
        return transforms[mask], joint_trajectory[mask], gripper_trajectory[mask], normalized_time

    def remove_outliers_mad(self, data, threshold=3.5):
        median = np.median(data, axis=0)
        diff = np.abs(data - median)
        mad = np.median(diff, axis=0)
        modified_z_score = 0.6745 * diff / (mad + 1e-6)
        mask = np.all(modified_z_score < threshold, axis=1)
        return mask, data[mask]

    def generate_trajectory(self, start_y=None, goal_y=None):
        print(f"Generating trajectory")
        if self.dmp is None:
            raise ValueError("No DMP model available. Learn or load a model first.")
            
        if start_y is not None:
            self.dmp.start_y = start_y
            print(f"Using custom start: {start_y}")
        else:
            print(f"Using default start: {self.dmp.start_y}")
            
        if goal_y is not None:
            self.dmp.goal_y = goal_y
            print(f"Using custom goal: {goal_y}")
        else:
            print(f"Using default goal: {self.dmp.goal_y}")
        
        T, Y = self.dmp.open_loop()
        trajectory = ptr.transforms_from_pqs(Y)
        return T, trajectory

    def save_dmp(self, filepath):
        if self.dmp is None:
            rospy.logerr("No DMP model available to save.")
            return
        if self.gripper_trajectory is None:
            rospy.logwarn("Gripper trajectory not available or not learned. Saving None for gripper_trajectory.")

        data_to_save = {
            'dmp': self.dmp,
            'gripper_trajectory': self.gripper_trajectory
        }
        try:
            with open(filepath, 'wb') as f:
                pickle.dump(data_to_save, f)
            rospy.loginfo(f"DMP and gripper trajectory saved to {filepath}")
        except Exception as e:
            rospy.logerr(f"Failed to save DMP data to {filepath}: {e}")

    def load_dmp(self, filepath):
        rospy.loginfo(f"Loading DMP data from {filepath}")
        try:
            with open(filepath, 'rb') as f:
                loaded_data = pickle.load(f)

            if isinstance(loaded_data, dict):
                if 'dmp' in loaded_data:
                    self.dmp = loaded_data['dmp']
                else:
                    rospy.logerr("Loaded dictionary is missing 'dmp' key.")
                    self.dmp = None

                if 'gripper_trajectory' in loaded_data:
                    self.gripper_trajectory = loaded_data['gripper_trajectory']
                    if self.gripper_trajectory is not None:
                         rospy.loginfo(f"Gripper trajectory loaded ({len(self.gripper_trajectory)} points).")
                    else:
                         rospy.loginfo("Loaded None for gripper trajectory.")
                else:
                    rospy.logwarn("Loaded dictionary is missing 'gripper_trajectory' key. Setting to None.")
                    self.gripper_trajectory = None
            else:
                rospy.logwarn("Loading old DMP format (only DMP object found). Gripper trajectory will be None.")
                self.dmp = loaded_data
                self.gripper_trajectory = None

            if self.dmp:
                rospy.loginfo("DMP object loaded successfully.")
            else:
                 rospy.logerr("Failed to load DMP object.")

        except FileNotFoundError:
            rospy.logerr(f"DMP file not found: {filepath}")
            self.dmp = None
            self.gripper_trajectory = None
        except Exception as e:
            rospy.logerr(f"Error loading DMP data from {filepath}: {e}")
            self.dmp = None
            self.gripper_trajectory = None
    
    def compute_IK_trajectory(self, trajectory, time_stamp, q0=None, subsample_factor=1):
        if q0 is None:
            q0 = np.array([-0.03834952, -0.84062147, 1.26093221, 0.00613592, 1.97576725, -0.00460194])
        
        if subsample_factor > 1:
            subsampled_trajectory = trajectory[::subsample_factor]
            subsampled_time_stamp = time_stamp[::subsample_factor]
            subsampled_gripper_trajectory = self.gripper_trajectory[::subsample_factor] if self.gripper_trajectory is not None else None
            print(f"Subsampled time from {len(time_stamp)} to {len(subsampled_time_stamp)} points")
            print(f"Subsampled trajectory from {len(trajectory)} to {len(subsampled_trajectory)} points")
        else:
            subsampled_trajectory = trajectory
            subsampled_time_stamp = time_stamp
            subsampled_gripper_trajectory = self.gripper_trajectory
        
        print(f"Solving inverse kinematics for {len(subsampled_trajectory)} points...")
        
        start_time = time.time()
        
        random_state = np.random.RandomState(0)
        joint_trajectory = self.chain.inverse_trajectory(
            subsampled_trajectory, random_state=random_state, orientation_weight=0.5)
            
        print(f"IK solved in {time.time() - start_time:.2f} seconds")
        
        return subsampled_trajectory, joint_trajectory, subsampled_gripper_trajectory, subsampled_time_stamp

    def visualize_trajectory(self, trajectory, joint_trajectory, q0=None):
        print(f"Plotting trajectory...")
        fig = pv.figure()
        fig.plot_transform(s=0.3)
        
        graph = fig.plot_graph(
            self.kin.tm, "world", show_visuals=False, show_collision_objects=True,
            show_frames=True, s=0.1, whitelist=[self.base_link, self.end_effector_link])

        fig.plot_transform(trajectory[0], s=0.15)
        fig.plot_transform(trajectory[-1], s=0.15)
        
        pv.Trajectory(trajectory, s=0.05).add_artist(fig)
        
        fig.view_init()
        fig.animate(
            animation_callback, len(trajectory), loop=True,
            fargs=(graph, self.chain, joint_trajectory))
        fig.show()


class GazeboTrajectoryPublisher:
    def __init__(self, joint_names=None, gripper_joint_names=None):
        if not rospy.core.is_initialized():
            rospy.init_node("gazebo_trajectory_publisher", anonymous=True)
        
        self.joint_names = joint_names or ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.gripper_joint_names = gripper_joint_names or ["gripper", "gripper_sub"]
        
        self.arm_pub = rospy.Publisher('/open_manipulator_6dof/arm_controller/command', 
                                     JointTrajectory, queue_size=10)
        self.gripper_pub = rospy.Publisher('/open_manipulator_6dof/gripper_controller/command', 
                                         JointTrajectory, queue_size=10)
        
        # Subscriber
        self.joint_state_topic = '/open_manipulator_6dof/joint_states'
        self._latest_joint_data = {
            "name": [],
            "position": [],
            "velocity": [],
            "effort": []
        }
        self._joint_state_lock = threading.Lock()
        try:
            self.joint_state_subscriber = rospy.Subscriber(
                self.joint_state_topic,
                JointState,
                self._joint_state_callback,
                queue_size=1 # Process most recent message
            )
            rospy.loginfo(f"[ROS Handler] Subscribed to joint states on '{self.joint_state_topic}'")
        except Exception as e:
            rospy.logerr(f"[ROS Handler] Failed to subscribe to {self.joint_state_topic}: {e}")
            self.joint_state_subscriber = None


        print(f"[Gazebo] Initialized publishers:")
        print(f"  - Arm: /open_manipulator_6dof/arm_controller/command")
        print(f"  - Gripper: /open_manipulator_6dof/gripper_controller/command")
        
        rospy.sleep(1.0)

    def publish_trajectory(self, joint_trajectory, gripper_trajectory, timestamps, execute_time_factor=1.0):
        if len(joint_trajectory) == 0:
            rospy.logwarn("[Gazebo] Empty trajectory provided")
            return
        
        print(f"[Gazebo] Publishing trajectory with {len(joint_trajectory)} points")
        
        arm_msg = JointTrajectory()
        arm_msg.header.stamp = rospy.Time.now()
        arm_msg.joint_names = self.joint_names
        
        gripper_msg = JointTrajectory()
        gripper_msg.header.stamp = rospy.Time.now()
        gripper_msg.joint_names = self.gripper_joint_names
        
        for i in range(len(joint_trajectory)):
            arm_point = JointTrajectoryPoint()
            arm_point.positions = joint_trajectory[i].tolist()
            arm_point.velocities = [0.0] * len(self.joint_names)
            arm_point.accelerations = [0.0] * len(self.joint_names)
            arm_point.time_from_start = rospy.Duration.from_sec(
                (timestamps[i] - timestamps[0]) * execute_time_factor
            )
            arm_msg.points.append(arm_point)
            
            if gripper_trajectory is not None and i < len(gripper_trajectory):
                gripper_point = JointTrajectoryPoint()
                gripper_value = gripper_trajectory[i]
                gripper_point.positions = [-2.0*gripper_value, -2.0*gripper_value]
                gripper_point.velocities = [0.0, 0.0]
                gripper_point.accelerations = [0.0, 0.0]
                gripper_point.time_from_start = rospy.Duration.from_sec(
                    (timestamps[i] - timestamps[0]) * execute_time_factor
                )
                gripper_msg.points.append(gripper_point)
        
        print(f"[Gazebo] Publishing arm trajectory with {len(arm_msg.points)} points")
        self.arm_pub.publish(arm_msg)
        
        if gripper_trajectory is not None and len(gripper_msg.points) > 0:
            print(f"[Gazebo] Publishing gripper trajectory with {len(gripper_msg.points)} points")
            self.gripper_pub.publish(gripper_msg)
        else:
            print(f"[Gazebo] No gripper trajectory to publish")
        
        print(f"[Gazebo] Trajectory published successfully")

    def publish_single_trajectory(self, full_trajectory, timestamps, execute_time_factor=1.0):
        if full_trajectory.shape[1] >= 6:
            arm_traj = full_trajectory[:, :6]
            gripper_traj = full_trajectory[:, 6] if full_trajectory.shape[1] > 6 else None
            
            self.publish_trajectory(arm_traj, gripper_traj, timestamps, execute_time_factor)
        else:
            rospy.logwarn(f"[Gazebo] Invalid trajectory shape: {full_trajectory.shape}")

    def publish_home_position(self, home_position=None, execution_time=5.0):
        if home_position is None:
            home_position = [-0.03834952, -0.84062147, 1.26093221, 0.00613592, 1.97576725, -0.00460194]
        
        print(f"[Gazebo] Publishing home position command...")
        print(f"[Gazebo] Home position: {home_position}")
        print(f"[Gazebo] Execution time: {execution_time} seconds")
        
        arm_msg = JointTrajectory()
        arm_msg.header.stamp = rospy.Time.now()
        arm_msg.joint_names = self.joint_names
        
        home_point = JointTrajectoryPoint()
        home_point.positions = home_position
        home_point.velocities = [0.0] * len(self.joint_names)
        home_point.accelerations = [0.0] * len(self.joint_names)
        home_point.time_from_start = rospy.Duration.from_sec(execution_time)
        
        arm_msg.points.append(home_point)
        
        self.arm_pub.publish(arm_msg)
        print(f"[Gazebo] Home position command published and latched")
    
    def _joint_state_callback(self, msg):
        """Internal callback to update the latest joint states."""
        with self._joint_state_lock:
            self._latest_joint_data["name"] = list(msg.name)
            self._latest_joint_data["position"] = list(msg.position)
            if len(msg.velocity) == len(msg.name):
                self._latest_joint_data["velocity"] = list(msg.velocity)
            else: # Fill with zeros if velocity is not available or mismatched
                self._latest_joint_data["velocity"] = [0.0] * len(msg.name)
            if len(msg.effort) == len(msg.name):
                self._latest_joint_data["effort"] = list(msg.effort)
            else: # Fill with zeros if effort is not available or mismatched
                self._latest_joint_data["effort"] = [0.0] * len(msg.name)


    def get_joint_states(self, desired_joint_order=None):
        """
        Retrieves the latest joint states for specified joints in a desired order.

        Args:
            desired_joint_order (list of str, optional): A list of joint names in the
                desired order for the output. If None, uses the
                `ordered_joint_names` the class was initialized with.

        Returns:
            dict: A dictionary with keys 'name', 'position', 'velocity', 'effort'.
                  Each key maps to a list of values corresponding to `desired_joint_order`.
                  Returns None if no data is available or if subscriber failed.
                  If a desired joint is not found in the latest message, its values will be None.
        """
            
        return {
            "name": self._latest_joint_data['name'], # Ensure it's a list copy
            "position": self._latest_joint_data['position'], # Ensure it's a list copy
            "velocity": self._latest_joint_data['velocity'], # Ensure it's a list copy
            "effort": self._latest_joint_data['effort'] # Ensure it's a list copy
        }


def animation_callback(step, graph, chain, joint_trajectory):
    chain.forward(joint_trajectory[step])
    graph.set_data()
    return graph

def save_trajectory_data(joint_trajectory, timestamps, filepath):
    data = {
        'trajectory': joint_trajectory,
        'timestamps': timestamps
    }
    with open(filepath, 'wb') as f:
        pickle.dump(data, f)
    print(f"[SAVE] Trajectory data saved to {filepath}")

def load_trajectory_data(filepath):
    with open(filepath, 'rb') as f:
        data = pickle.load(f)
    
    joint_trajectory = data['trajectory']
    timestamps = data['timestamps']
    print(f"[LOAD] Loaded trajectory from {filepath} (length={len(joint_trajectory)})")
    return joint_trajectory, timestamps

def interpolate_joint_trajectory(joint_traj, time_stamps, target_freq=20.0):
    num_joints = joint_traj.shape[1]
    duration = time_stamps[-1] - time_stamps[0]
    num_samples = int(duration * target_freq)
    new_timestamps = np.linspace(time_stamps[0], time_stamps[-1], num_samples)
    
    interp_traj = np.zeros((num_samples, num_joints))
    for i in range(num_joints):
        interpolator = interp1d(time_stamps, joint_traj[:, i], kind='linear', fill_value="extrapolate")
        interp_traj[:, i] = interpolator(new_timestamps)
    
    return interp_traj, new_timestamps

def get_cube_position(cube_name, timeout=1.0):
    """Get position of a cube by its TF frame name"""
    print(f"Getting {cube_name} position...")
    if not rospy.core.is_initialized():
        rospy.init_node('tf_xyz_fetcher', anonymous=True)
    
    listener = tf.TransformListener()
    try:
        print(f"Waiting for transform /world -> /{cube_name}...")
        listener.waitForTransform('/world', f'/{cube_name}', rospy.Time(0), rospy.Duration(timeout))
        
        trans, _ = listener.lookupTransform('/world', f'/{cube_name}', rospy.Time(0))
        print(f"{cube_name} position: {trans}")
        return trans
    except Exception as e:
        print(f"Error getting transform for {cube_name}: {e}")
        return None

def execute_motion(dmp_gen, dmp_save_path, cube_name, position_offset, publisher, 
                  motion_name="motion", execute_time_factor=5, visualize=False, target_pos=None, publish_enable=False):
    """Execute a motion (pick or place) with given parameters"""
    home_position =  [-0.09970875084400177, -0.1902136206626892, 1.4710875749588013, 0.04295146465301514, 1.702718734741211, -0.058291271328926086]
    print(f"\n=== Executing {motion_name} motion ===")
    
    # Learn from bag
    # print(f"Learning {motion_name} motion from bag: {bag_path}")
    # Y, transforms, joint_traj, gripper_traj = dmp_gen.learn_from_rosbag(
    #     bag_path, 
    #     '/gravity_compensation_controller/traj_joint_states'
    # )
    
    # # Save DMP
    # dmp_gen.save_dmp(dmp_save_path)
    dmp_gen.load_dmp(dmp_save_path)
    print(f"{motion_name.capitalize()} DMP loaded from: {dmp_save_path}")
    
    # Get target position
    if target_pos is not None:
        print(f"Using provided target position: {target_pos}")
        cube_position = target_pos
    else:
        cube_position = get_cube_position(cube_name)
    if cube_position is None:
        print(f"Failed to get {cube_name} position. Using default offset.")
        cube_position = [0.0, 0.0, 0.0]
    
    # Set start and goal
    new_start = dmp_gen.dmp.start_y.copy()
    new_goal = dmp_gen.dmp.goal_y.copy()


    # set default start position from current joint positions

    # new_start[:3]= dmp_gen.chain.forward()
    if publish_enable:
        # current_joint_states = publisher.get_joint_states()['position'][2:]
        # print(f"\n\nCurrent joint states: {current_joint_states}")
        new_start= _manual_pqs_from_transform(dmp_gen.chain.forward(home_position))
        print(f"New start position: {new_start[:3]}")
    print(f"Original goal: {new_goal[:3]}")
    new_goal[:3] = np.array(cube_position) + np.array(position_offset)
    print(f"New goal: {new_goal[:3]}")
    
    # Generate trajectory
    T, trajectory = dmp_gen.generate_trajectory(start_y=new_start, goal_y=new_goal)
    
    # Compute IK
    trajectory, IK_joint_trajectory, gripper_traj, T = dmp_gen.compute_IK_trajectory(
        trajectory, T, subsample_factor=3)
    
    # Apply smoothing
    window_size = 10
    if len(IK_joint_trajectory) > window_size:
        original_start = IK_joint_trajectory[0,:].copy()
        original_end = IK_joint_trajectory[-1,:].copy()

        smoothed_IK_joint_trajectory = np.zeros_like(IK_joint_trajectory)
        for i in range(IK_joint_trajectory.shape[1]):
            smoothed_IK_joint_trajectory[:, i] = np.convolve(IK_joint_trajectory[:, i], 
                                                           np.ones(window_size)/window_size, mode='same')

        smoothed_IK_joint_trajectory[0,:] = original_start
        smoothed_IK_joint_trajectory[-1,:] = original_end

        half_window = window_size // 2
        for i in range(IK_joint_trajectory.shape[1]):
            for j in range(half_window):
                alpha = j / float(half_window)
                smoothed_IK_joint_trajectory[j, i] = (1 - alpha) * original_start[i] + alpha * smoothed_IK_joint_trajectory[j, i]
            for j in range(half_window):
                alpha = j / float(half_window)
                idx_from_end = len(IK_joint_trajectory) - 1 - j
                smoothed_IK_joint_trajectory[idx_from_end, i] = (1 - alpha) * original_end[i] + alpha * smoothed_IK_joint_trajectory[idx_from_end, i]

        IK_joint_trajectory = smoothed_IK_joint_trajectory
        print(f"Applied moving average filter with window size {window_size} to IK trajectory.")
    else:
        print(f"Trajectory too short for smoothing (length {len(IK_joint_trajectory)})")

    # Visualize if requested
    
    if visualize:
        dmp_gen.visualize_trajectory(trajectory, IK_joint_trajectory)
    
    # Prepare full trajectory
    traj_length = min(IK_joint_trajectory.shape[0], len(gripper_traj) if gripper_traj is not None else IK_joint_trajectory.shape[0])
    IK_joint_trajectory = IK_joint_trajectory[:traj_length, :]
    
    if gripper_traj is not None:
        gripper_traj = gripper_traj[:traj_length]
        full_trajectory = np.hstack((IK_joint_trajectory, -gripper_traj.reshape(-1, 1)))
    else:
        gripper_traj = np.zeros(traj_length)
        full_trajectory = np.hstack((IK_joint_trajectory, -gripper_traj.reshape(-1, 1)))
    
    # Interpolate trajectory
    interpolated_traj, interpolated_time = interpolate_joint_trajectory(
        full_trajectory, T[:traj_length], target_freq=100.0)

    # Execute trajectory
    print(f"[{motion_name}] Starting trajectory execution...")
    
    # Clip trajectory to 95% of length to avoid oscillations in the learned motions
    clip_start= int(0.1 * len(interpolated_traj))
    clip_length = int(0.95 * len(interpolated_traj))
    arm_trajectory = interpolated_traj[clip_start:clip_length, :6]
    gripper_trajectory = interpolated_traj[clip_start:clip_length, 6]

    
    # print(f"\n\nGripper trajectory over 0.001 {np.where(gripper_trajectory>0.001)[0,0]}")
    if publish_enable:
        publisher.publish_trajectory(arm_trajectory, -gripper_trajectory, 
                                interpolated_time, execute_time_factor=execute_time_factor)
    
    # Wait for completion
    trajectory_execution_time = max(interpolated_time) * execute_time_factor
    
 
    
    
    if motion_name == "pick":
        delay = trajectory_execution_time + 2.0
        print(f"Waiting for trajectory execution to complete ({delay:.2f} seconds)...")
        rospy.sleep(trajectory_execution_time + 2.0)
        attach_cube_to_gripper(cube_name)
        print(f"[{motion_name}] Attached {cube_name} to gripper.")
    elif motion_name == "place":
        # Determine when gripper opens
        idx = np.where(gripper_trajectory > -0.001)[0][0]
        delay = interpolated_time[idx]*execute_time_factor
        print(f"Watingin for : {delay} till gripper opens at {idx}") 
        print(f"Waiting for trajectory execution to complete ({delay:.2f} seconds)...")
        rospy.sleep(delay)
        detach_cube_from_gripper(cube_name)
        print(f"[{motion_name}] Detached {cube_name} from gripper.")
        delay_2 = trajectory_execution_time - delay + 2.0
        print(f"Waiting for additional {delay_2:.2f} seconds for detach to complete...")
        rospy.sleep(delay_2)  # Wait for the detach to complete
    else: 
        rospy.sleep(trajectory_execution_time + 2.0)

    print(f"[{motion_name}] Motion completed successfully!")
    return True

def attach_cube_to_gripper(cube_name):
    # rospy.init_node('demo_detach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    rospy.loginfo("Attaching gripper and blue_cube")
    req = AttachRequest()
    req.model_name_1 = cube_name
    req.link_name_1 = f"{cube_name}::link"
    req.model_name_2 = "robot"
    req.link_name_2 = "robot::gripper_link_sub"

    attach_srv.call(req)

def detach_cube_from_gripper(cube_name):
    # rospy.init_node('demo_detach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    # Link them
    rospy.loginfo("Detaching gripper and blue_cube")
    req = AttachRequest()
    req.model_name_1 = cube_name
    req.link_name_1 = f"{cube_name}::link"
    req.model_name_2 = "robot"
    req.link_name_2 = "robot::gripper_link_sub"

    attach_srv.call(req)

def get_cubes_peg_position(peg_positions):
    """
    Via the position of the cubes, and the fixed peg positions, determine which cubes are on which pegs.
    Returns:
        A dictionary mapping peg names to lists of cube names on that peg.
    """
    # peg_positions = {
    #     "A": [0.18, 0.09, 0.05],
    #     "B": [0.18, 0.0, 0.05],
    #     "C": [0.18, -0.09, 0.05]
    # }
    
    cube_positions = {
        "cube_3": get_cube_position("green_cube", timeout=3.0),
        "cube_2": get_cube_position("red_cube", timeout=3.0),
        "cube_1": get_cube_position("blue_cube", timeout=3.0),
    }
    print(f"Cube positions: {cube_positions}")
    peg_cubes = {peg: [] for peg in peg_positions.keys()}
    
    for cube_name, position in cube_positions.items():
        if position is None:
            continue
        for peg_name, peg_pos in peg_positions.items():
            if np.linalg.norm(np.array(position[:2]) - np.array(peg_pos[:2])) < 0.03:  # Adjust threshold as needed
                peg_cubes[peg_name].append(cube_name)
                break        
    
    return peg_cubes


def initialise_rag_system():
    print("Initializing Retrieval Augmented Generation system...")

    # Define your sources directly in an array
    # Include a mix of local HTML files and web URLs as needed
    sources = [
        "/root/catkin_ws/src/my_scripts/assignment_3/Docker_volume/hanoi_tagged_solution_3_disks.html"
        # "https://www.geeksforgeeks.org/iterative-tower-of-hanoi/"  # Keeping the original source as well
    ]
    
    print("\nUsing the following document sources:")
    for idx, source in enumerate(sources, 1):
        print(f"{idx}. {source}")
    
    # 1. Load Data
    print("\nLoading documents from sources...")
    data = load_documents(sources)
    if not data:
        print("Error: No data loaded from any sources. Please check your URLs and file paths.")
        return
    print(f"Successfully loaded {len(data)} document(s) in total.")

    # 2. Split Data
    print("Splitting documents into manageable chunks...")
    text_splitter = RecursiveCharacterTextSplitter(chunk_size=500, chunk_overlap=50)
    all_splits = text_splitter.split_documents(data)
    print(f"Split documents into {len(all_splits)} chunks.")

    # 3. Initialize Embeddings
    OLLAMA_EMBEDDING_MODEL = "nomic-embed-text"
    print(f"Initializing Ollama embeddings ({OLLAMA_EMBEDDING_MODEL})...")
    try:
        # Ensure you have pulled the nomic-embed-text model in Ollama:
        # ollama pull nomic-embed-text
        embeddings = OllamaEmbeddings(model=OLLAMA_EMBEDDING_MODEL)
        # Test the embedding model
        print("Testing embedding model...")
        _ = embeddings.embed_query("Test query for embedding model.")
        print(f"Ollama embedding model '{OLLAMA_EMBEDDING_MODEL}' initialized successfully.")
    except Exception as e:
        print(f"Error initializing Ollama embeddings with model '{OLLAMA_EMBEDDING_MODEL}': {e}.")
        print(f"Ensure Ollama is running and you have pulled the '{OLLAMA_EMBEDDING_MODEL}' model (e.g., 'ollama pull {OLLAMA_EMBEDDING_MODEL}').")
        print("Also ensure you have installed 'pip install -U langchain-ollama'.")
        return

    # 4. Create Vector Store
    COLLECTION_NAME = "rag-chroma-nomic-embed"
    print(f"Creating ChromaDB vector store (in-memory, collection: {COLLECTION_NAME})...")
    try:
        vectorstore = Chroma.from_documents(
            documents=all_splits,
            collection_name=COLLECTION_NAME,
            embedding=embeddings,
        )
        retriever = vectorstore.as_retriever()
        print("Vector store created successfully.")
    except Exception as e:
        print(f"Error creating vector store: {e}")
        return

    # 5. Setup Ollama LLM for Chat
    OLLAMA_CHAT_MODEL = 'gemma3:4b' # Or your preferred Ollama chat model like 'llama3', 'mistral'
    print(f"Using Ollama chat model: {OLLAMA_CHAT_MODEL}")
    try:
        model_local = ChatOllama(model=OLLAMA_CHAT_MODEL)
        # Perform a quick test invocation to check connectivity
        print(f"Testing Ollama chat model '{OLLAMA_CHAT_MODEL}' connection...")
        _ = model_local.invoke("This is a test prompt.")
        print(f"Ollama chat model '{OLLAMA_CHAT_MODEL}' connection successful.")
    except Exception as e:
        print(f"Error connecting to Ollama chat model '{OLLAMA_CHAT_MODEL}': {e}")
        print(f"Please ensure Ollama is running and the specified model is available (e.g., 'ollama pull {OLLAMA_CHAT_MODEL}').")
        print("Also ensure you have installed 'pip install -U langchain-ollama'.")
        return

   
    template = """Answer the question based on the following context if relevant. If the context does not contain information related to the question, you may use your general knowledge to provide an answer.

    Context:
    {context}

    Question: {question}

    Answer:
    """
    prompt = ChatPromptTemplate.from_template(template)
    # 7. Define RAG Chain
    chain = (
        {"context": retriever, "question": RunnablePassthrough()}
        | prompt
        | model_local
        | StrOutputParser()
    )

    return chain

def get_peg_height(peg_name, current_state):
    """
    Get the height of a peg by its name.
    Args:
        peg_name: Name of the peg (e.g., "A", "B", "C").
    Returns:
        Height of the peg.
    """
    peg_heights = {
        "A": 0.05,
        "B": 0.05,
        "C": 0.05
    }
    for peg in current_state.keys():
        peg_heights[peg] += 0.02 * len(current_state[peg])
    return peg_heights

def get_peg_insert_position(peg_name, current_state, peg_positions):
    # peg_positions = {
    #     "A": [0.18, 0.09, 0.05],
    #     "B": [0.18, 0.0, 0.05],
    #     "C": [0.18, -0.09, 0.05]
    # }
    insert_position = peg_positions[peg_name].copy()
    print(insert_position)
    insert_position[2] = get_peg_height(peg_name, current_state)[peg_name]
    if peg_name == "B":
        insert_position[0] += 0.015 * len(current_state[peg_name])
    
    return insert_position


def extract_last_move_tag(answer_text: str) -> Optional[str]:
    """
    Extracts the last valid Tower of Hanoi move tag (e.g., MD1AC) from a string.

    Args:
        answer_text: The string potentially containing move tags.

    Returns:
        The last found move tag as a string, or None if no valid tag is found.
    """
    # Regex to find patterns like MD<digit><Peg><Peg> where Peg is A, B, or C
    # It looks for "MD" followed by one or more digits, then two characters from A, B, or C.
    move_tag_pattern = r"MD\d[A-C][A-C]"
    
    matches = re.findall(move_tag_pattern, answer_text)
    
    if matches:
        # Return the last match found
        return matches[-1]
    else:
        # No valid move tag found
        return None

def extract_all_move_tags(answer_text: str) -> List[str]:
    """
    Extracts all valid Tower of Hanoi move tags (e.g., MD1AC) from a string.

    Args:
        answer_text: The string potentially containing move tags.

    Returns:
        A list of all found move tags as strings, or an empty list if no valid tags are found.
    """
    # Regex to find patterns like MD<digit><Peg><Peg> where Peg is A, B, or C
    # It looks for "MD" followed by one or more digits, then two characters from A, B, or C.
    move_tag_pattern = r"MD\d[A-C][A-C]"
    
    matches = re.findall(move_tag_pattern, answer_text)
    
    return matches

def check_valid_move_tag(current_state, move) -> bool:
    """
    Check if the move tag is valid.
    
    Args:
        current_state: The current state of the pegs and cubes. dict mapping peg names to lists of cube names.
        move: dict containing the move details with keys 'cube', 'start', and 'end'.
    
    Returns:
        True if the tag is valid, False otherwise.
    """
    cube = move['cube']
    start_peg = move['start']
    end_peg = move['end']
    cube_names = ['blue_cube', 'red_cube', 'green_cube']

    cube_names_num = []
    
    for i in range(len(cube_names)):
        cube_names_num.append('cube_' + str(i+1))

    print(f"Cube name {cube}")
    # Check if the cube exists in the start peg
    print(f"Cube name: {cube_names[cube-1]}")
    print(f"Current state of start peg {start_peg}: {current_state[start_peg]}")
    if cube_names_num[cube-1] not in current_state[start_peg]:
        print(f"Cube {cube} is not on peg {start_peg}. Invalid move.")
        return False
    
    # Check if the end peg is valid
    if end_peg not in current_state:
        print(f"End peg {end_peg} does not exist. Invalid move.")
        return False
    
    # Check if the end peg is occupied by a larger cube
    print(f"Other cubes on end peg {end_peg}: {current_state[end_peg]}")
    for other_cube in current_state[end_peg]:

        if cube_names_num.index(other_cube) < (cube-1):
            print(f"Cannot place {cube} on {end_peg} because it is occupied by a smaller cube {other_cube}. Invalid move.")
            return False
    
    return True

# --- Helper function for manual PQS conversion ---
def _manual_pqs_from_transform(transform_matrix):
    """
    Manually converts a 4x4 transformation matrix to a PQS vector [x,y,z,qw,qx,qy,qz].
    Uses tf.transformations.
    """
    position = transform_matrix[0:3, 3]
    # quaternion_from_matrix returns (qx, qy, qz, qw)
    q_tf = quaternion_from_matrix(transform_matrix)
    # PQS format requires (qw, qx, qy, qz) for the quaternion part
    quaternion_pqs = np.array([q_tf[3], q_tf[0], q_tf[1], q_tf[2]])
    return np.concatenate((position, quaternion_pqs))

# ------------------------------------------- Main Execution --------------------------------------------

if __name__ == "__main__":
    
    cube_names = ['blue_cube', 'red_cube', 'green_cube']
    # Configuration
    urdf_path = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/urdf/open_manipulator_6dof.urdf'
    mesh_path = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/meshes'
    
    # Bag file paths
    pick_bag_path = '/root/catkin_ws/src/om_position_controller/recording/pick.bag'
    place_bag_path = '/root/catkin_ws/src/om_position_controller/recording/place.bag'  # Add your place bag path

    
    pick_bag_left_path = '/root/catkin_ws/src/recordings/pick_left11.bag'
    pick_bag_front_path = '/root/catkin_ws/src/recordings/pick_front10.bag'
    pick_bag_right_path = '/root/catkin_ws/src/recordings/pick_right11.bag'

    place_bag_left_path = '/root/catkin_ws/src/recordings/place_left10.bag'
    place_bag_front_path = '/root/catkin_ws/src/recordings/place_front10.bag'
    place_bag_right_path = '/root/catkin_ws/src/recordings/place_right10.bag'



    # DMP save paths
    pick_dmp_path = '/root/catkin_ws/src/om_position_controller/recording/pick_motion.pkl'
    place_dmp_path = '/root/catkin_ws/src/om_position_controller/recording/place_motion.pkl'

    pick_dmp_left_path = '/root/catkin_ws/src/recordings/pick_left_motion10.pkl'
    pick_dmp_front_path = '/root/catkin_ws/src/recordings/pick_front_motion10.pkl'
    pick_dmp_right_path = '/root/catkin_ws/src/recordings/pick_right_motion10.pkl'

    place_dmp_left_path = '/root/catkin_ws/src/recordings/place_left_motion10.pkl'
    place_dmp_front_path = '/root/catkin_ws/src/recordings/place_front_motion10.pkl'
    place_dmp_right_path = '/root/catkin_ws/src/recordings/place_right_motion10.pkl'

    # pick_dmp_path= '/root/catkin_ws/src/recordings/learned_pick_left.pkl'
    # place_dmp_path = '/root/catkin_ws/src/recordings/learned_place_right.pkl'
    
    # Home position
    home_position = [-0.09970875084400177, -0.1902136206626892, 1.4710875749588013, 0.04295146465301514, 1.702718734741211, -0.058291271328926086]

    # Peg positions 
    peg_positions = {
        "A": [0.16, 0.11, 0.05],
        "B": [0.18, 0.0, 0.05],
        "C": [0.16, -0.11, 0.05]
    }
    
    

    # ----------------------------------Initialize DMPMotionGenerator and GazeboTrajectoryPublisher ----------------------------------#
    dmp_gen = DMPMotionGenerator(
                    urdf_path, 
                    mesh_path,
                    base_link="world"
                )
    
    publisher = GazeboTrajectoryPublisher()
    rospy.sleep(2.0)  # Allow time for publishers to initialize

    # ----------------------------------Initialize the RAG System----------------------------------#
    chain = initialise_rag_system()
    goal_state = {'C': ['cube_3', 'cube_2', 'cube_1'], 'A': [], 'B': []}  # Goal state for Tower of Hanoi with 3 cubes

    i = 0

    current_state = get_cubes_peg_position(peg_positions)


    print(f"Current state of cubes on pegs: {current_state}")

    prompt = f"The current state of the pegs are arranged as follows: {current_state}. \
                Solve the tower of hanoi problem based on the current state." 
    answer = chain.invoke(prompt)
    print(f"RAG System Answer: {answer}")
    
    extracted_tags = extract_all_move_tags(answer)
    print(f"Extracted move tags from answer: {extracted_tags}")
    decoded_moves = [decode_move_tag(tag) for tag in extracted_tags]
    print(f"Decoded move tags: {decoded_moves}")

    for decoded_move in decoded_moves:
        pick_offset = [0.0, 0.0, 0.02]  # Slight offset above cube for picking
        place_offset = [0.0, 0.0, 0.01]  # Slight offset above peg for placing
        current_state = get_cubes_peg_position(peg_positions)
        if decoded_move['start'] == 'A':
            pick_bag_path = pick_bag_left_path
            pick_dmp_path = pick_dmp_left_path
        elif decoded_move['start'] == 'B':
            pick_bag_path = pick_bag_front_path
            pick_dmp_path = pick_dmp_front_path
        elif decoded_move['start'] == 'C':
            pick_bag_path = pick_bag_right_path
            pick_dmp_path = pick_dmp_right_path
        
        if decoded_move['end'] == 'A':
            place_bag_path = place_bag_left_path
            place_dmp_path = place_dmp_left_path
        elif decoded_move['end'] == 'B':
            place_bag_path = place_bag_front_path
            place_dmp_path = place_dmp_front_path
        elif decoded_move['end'] == 'C':
            place_bag_path = place_bag_right_path
            place_dmp_path = place_dmp_right_path
        
        # Check if the move is valid
        # Yet to implement this function
        if check_valid_move_tag(current_state, decoded_move):
            print(f"LLM move is valid: {decoded_move}")
            target_peg_pos = get_peg_insert_position(decoded_move['end'], current_state, peg_positions)
            # print(f"Insert peg position: {get_peg_insert_position(decoded_move['end'], current_state)}")
            pick_cube_name = cube_names[decoded_move['cube'] - 1]  # Assuming cube numbering starts from 1
            print(f"Cube to pick: {pick_cube_name}")
            success = execute_motion(
                dmp_gen=dmp_gen,
                dmp_save_path=pick_dmp_path,
                cube_name=pick_cube_name,
                position_offset=pick_offset,  # Slight offset above cube
                publisher=publisher,
                motion_name="pick",
                execute_time_factor=5,
                visualize=False,  # Set to False to skip visualization
                publish_enable=True,  # Set to True to publish trajectory
            )
            if not success:
                print("Pick motion failed!")
                exit(1)
            
            # 2. RETURN TO HOME
            print("\n=== Returning to Home Position ===")
            publisher.publish_home_position(
                home_position=home_position,
                execution_time=5.0
            )
            rospy.sleep(6.0)  # Wait for the robot to return to home position

            success = execute_motion(
                dmp_gen=dmp_gen,
                dmp_save_path=place_dmp_path,
                cube_name=pick_cube_name,
                position_offset=place_offset,  # Offset above green cube for placing
                publisher=publisher,
                motion_name="place",
                execute_time_factor=5,
                visualize=False, # Set to True if you want to visualize
                target_pos=target_peg_pos,  # Use the peg position for placing
                publish_enable=True,  # Set to True to publish trajectory
            )

            if not success:
                print("Place motion failed!")
                exit(1)
                
            # 4. FINAL RETURN TO HOME
            print("\n=== Final Return to Home Position ===")
            publisher.publish_home_position(
                home_position=home_position,
                execution_time=5.0
            )
            print("[Final] Returning to home position...")
            rospy.sleep(6.0)
            
            print("\n=== Pick and Place Operation Completed Successfully! ===")
        else:
            print(f"LLM move is invalid: {decoded_move}. Skipping this move.")
            pass