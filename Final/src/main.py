import time
import math
import re
import rospy

from pick_and_place import DMPMotionGenerator, GazeboTrajectoryPublisher, execute_motion, get_cube_position

def distance(pos1, pos2):
    return math.sqrt(
        (pos2[0] - pos1[0])**2 +
        (pos2[1] - pos1[1])**2 +
        (pos2[2] - pos1[2])**2
    )

class Cube():
    def __init__(self, name, value):
        self.name = name
        self.value = value
        self.position = [0,0,0]


class Rod():
    def __init__(self, name, value):
        self.name = name
        self.position = value
        self.cubes = []
    
    def sort_stack(self):
        self.cubes.sort(key=lambda cube: cube.position[2])

class Movement():
    def __init__(self, cube: Cube, start_rod: Rod,end_rod: Rod):
        self.cube      = cube
        self.start_rod = start_rod
        self.end_rod   = end_rod

class TowerOfHanoi():
    def __init__(self):
        self.rods = {}
        self.cubes = {}
        self.movements = []
    
    def addCube(self, cube: Cube):
        self.cubes[cube.value] = cube

    def addRod(self, rod: Rod):
        self.rods[rod.name] = rod

    def addMovement(self, Movement: Movement):
        self.movements.append(Movement)

    def getRodNames(self):
        return [rod for rod in self.rods]
    
    def printRodPositions(self):
        for rod in self.rods.values():
            print(f"Rod {rod.name}: {rod.position}")

    def printRodStates(self):
        for rod in self.rods.values():
            cube_list = [cube.value for cube in rod.cubes]
            print(f"Rod {rod.name}: {cube_list}")

    def init_cubes(self):
        for cube in self.cubes.values():
            cube.position = get_cube_position(cube.name, timeout=0.5)

    def assign_cubes_to_rods(self):
        # find closest rod
        for cube in self.cubes.values():
            cube.position = get_cube_position(cube.name, timeout=0.5)
            
            closest_rod = None
            min_dist = float('inf')
            for rod in self.rods.values():
                dist = distance(cube.position, rod.position)
                if dist < min_dist:
                    min_dist = dist
                    closest_rod = rod
            
            closest_rod.cubes.append(cube)

        for rod in self.rods.values():
            rod.sort_stack()

    def execute_movements(self):
        for m in self.movements:

            if len(m.end_rod.cubes) == 0: # no cube on rod
                debug_target = f"empty rod"
            else:
                goal_cube = m.end_rod.cubes[-1].name # 
                debug_target = f"on top of {goal_cube}"

            print(f"Moving {m.cube.name} to rod {m.end_rod.name} ({debug_target})")
            pick_place(m.cube, m.start_rod, m.end_rod)
        

def wait_for_user():
    input("Press Enter to continue...\n")

def parse_response(text, tower: TowerOfHanoi):
    matches = re.findall(r'MD(\d)([ABC])([ABC])', text)

    for disk_str, start, end in matches:
        disk = int(disk_str)

        cube = tower.cubes.get(disk)
        if not cube:
            raise ValueError(f"No cube defined for cube number {disk}!")

        for rod_name in (start, end):
            if rod_name not in tower.rods:
                raise ValueError(f"No Rod defined for Rod Name {rod_name}!")

        start_rod = tower.rods.get(start)
        end_rod   = tower.rods.get(end)

        movement = Movement(cube=cube, start_rod=start_rod, end_rod=end_rod)
        tower.addMovement(movement)

def pick_place( cube: Cube, start_rod: Rod, end_rod: Rod):
    print("=== Starting Pick and Place Operation ===")
    

    try:
        
        if cube is None:
            print("cube = None!")
            exit(1)

        if start_rod is None:
            print("cube = None!")
            exit(1)

        if end_rod is None:
            print("end_rod = None!")
            exit(1)

        # TODO change value based on block hight 
        offset_pick = (len(start_rod.cubes)-1)/10
        offset_place = (len(end_rod.cubes)-1)/10


        # Initialize components
        dmp_gen = DMPMotionGenerator(
            urdf_path, 
            mesh_path,
            base_link="world"
        )
        
        publisher = GazeboTrajectoryPublisher()
        rospy.sleep(2.0)
    

        

        # 1. PICK MOTION - Blue Cube
        success = execute_motion(
            dmp_gen=dmp_gen,
            bag_path=pick_bag_path,
            dmp_save_path=pick_dmp_path,
            cube_name=cube.name,
            position_offset=[0.0, 0.0, offset_pick],  # Slight offset above cube TODO offset
            publisher=publisher,
            motion_name="pick",
            execute_time_factor=5,
            visualize=False,  # Set to False to skip visualization
            cube_position=None
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
        print("[Home] Waiting for home position...")
        rospy.sleep(7.0)  # Wait for home position completion
        print("[Home] Home position reached!")
        
        # 3. PLACE MOTION - Green Cube
        success = execute_motion(
            dmp_gen=dmp_gen,
            bag_path=place_bag_path,
            dmp_save_path=place_dmp_path,
            cube_name=None,
            position_offset=[0.0, 0.0, offset_place],  # Offset above green cube for placing
            publisher=publisher,
            motion_name="place",
            execute_time_factor=5,
            visualize=False,  # Set to True if you want to visualize
            cube_position=end_rod.position
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
        rospy.sleep(7.0)
        
        print("\n=== Pick and Place Operation Completed Successfully! ===")
        
    except rospy.ROSInterruptException:
        print("[Main] ROS interrupted.")
    except Exception as e:
        print(f"[Main] Error during pick and place operation: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    # Configuration
    urdf_path = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/urdf/open_manipulator_6dof.urdf'
    mesh_path = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/meshes'
    
    # Bag file paths
    pick_bag_path = '/root/catkin_ws/src/om_position_controller/recording/pick.bag'
    place_bag_path = '/root/catkin_ws/src/om_position_controller/recording/place.bag'  # TODO change to our recorded bags

    # DMP save paths
    pick_dmp_path = '/root/catkin_ws/src/om_position_controller/recording/pick_motion.pkl'
    place_dmp_path = '/root/catkin_ws/src/om_position_controller/recording/place_motion.pkl'
    
    # Home position
    home_position = [-0.03834952, -0.84062147, 1.26093221, 0.00613592, 1.97576725, -0.00460194]
    # TODO I just copied this so we still need to adapt it


    # Set the rod positions corresponding to the cube positions
    toh = TowerOfHanoi()
    toh.addCube(Cube('blue_cube',1))
    toh.addCube(Cube('red_cube',2))
    toh.addCube(Cube('green_cube',3))

    print("\n\n=============== Mark Rod Positions ===============\n")
    print("Please mark the rod positions with the cubes \n")
    print("The smallest cube value correspods to the position of rod 'A', the second 'B' and so on... \n")
    wait_for_user()
    toh.init_cubes()

    toh.addRod(Rod('A', toh.cubes[1].position))
    toh.addRod(Rod('B', toh.cubes[2].position))
    toh.addRod(Rod('C', toh.cubes[3].position))

    print("\n\nSet rod positions to:\n")
    toh.printRodPositions()
    
    # Wait for User to Build Towers
    print("\n\n=============== Set Starting Position ===============\n")
    print("Please build the starting position and then enter a target rod for the final tower.\n")
    targetRod = None

    while True:
        target_rod = input(f"Enter target rod {toh.getRodNames()}:")
        if target_rod in toh.getRodNames():
            print(f"Selected Target Rod: {target_rod}")
            break
        else:
            print(f"Invalid Rod Name: {target_rod}. Try again!\n")
    
    toh.assign_cubes_to_rods()


    print("\n\n=============== Task Description ===============\n")
    print("Current State:")
    toh.printRodStates()
    print("\nTarget State:")
    for rod in toh.rods.values():
        if target_rod == rod.name:
            print(f"Rod {rod.name}: [3, 2, 1]") 
        else:
            print(f"Rod {rod.name}: []") 
    
    text = """
    1. MD2BC  
    2. MD1AB  
    """

    parse_response(text, toh)
    
    for m in toh.movements:
        print(f"Move cube from {m.cube.name} to {m.end_rod.name}")
        if len(m.end_rod.cubes) == 0:
            print(f"position {m.end_rod.position} ")
        else:
            print(f"Name {m.end_rod.cubes[-1].name} ")

    toh.execute_movements()

