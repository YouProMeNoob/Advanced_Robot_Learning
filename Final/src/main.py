import time
import math
import re
import rospy

from pick_and_place_V2 import DMPMotionGenerator, RealRobotTrajectoryPublisher, GazeboTrajectoryPublisher, execute_motion, get_cube_position

from chatgpt_call import ask_llm

home_position = [-0.03834952, -0.84062147, 1.26093221, 0.00613592, 1.97576725, -0.00460194]
home_position_close = [-0.03834952, -0.84062147, 1.26093221, 0.00613592, 1.97576725, -0.00460194, -0.01] 
home_position_open = [-0.03834952, -0.84062147, 1.26093221, 0.00613592, 1.97576725, -0.00460194, 0.01]

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
            cube.position = get_cube_position(cube.name, timeout=10)

    def assign_cubes_to_rods(self):
        
        for rod in self.rods.values():
            rod.cubes.clear()

        # find closest rod
        for cube in self.cubes.values():
            temp_pos = get_cube_position(cube.name, timeout=10)
            if temp_pos is not None:
                cube.position = temp_pos 
            
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
        for i, m in enumerate(self.movements[:]):

            if len(m.end_rod.cubes) == 0: # no cube on rod
                debug_target = f"empty rod"
            else:
                goal_cube = m.end_rod.cubes[-1].name # 
                debug_target = f"on top of {goal_cube}"

            print(f"Moving {m.cube.name} to rod {m.end_rod.name} ({debug_target})")
            pick_place(self, m.cube, m.start_rod, m.end_rod, return_home=(i==0))

            self.movements.remove(m)

    def print_movements(self):
        for m in self.movements:
            print(f"Move {m.cube.name} from {m.start_rod.name} to {m.end_rod.name}")
            if len(m.end_rod.cubes) == 0:
                print(f"position {m.end_rod.position} ")
            else:
                print(f"Name {m.end_rod.cubes[-1].name} ")

        

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

def pick_place(toh: TowerOfHanoi, cube: Cube, start_rod: Rod, end_rod: Rod, return_home:bool = True):
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


        # maybe adjust boundaries
        left_boundary = 0.04
        right_boundary = -0.04


        # Initialize components
        dmp_gen = DMPMotionGenerator(
            urdf_path, 
            mesh_path,
            base_link="world"
        )
        
        publisher = RealRobotTrajectoryPublisher() # change to GazeboTrajectoryPublisher() for sim
        rospy.sleep(2.0)

        pick_offset = [0.0, 0.0, 0.0];
        place_offset = [0.0, 0.0, 0.13]
        # Select the path (left, right middle)
        if right_boundary <= cube.position[1] <= left_boundary:
            pick_dmp_path =  pick_dmp_front_path
            pick_offset = [-0.023, 0.015, -0.022]

        elif right_boundary > cube.position[1]:
            pick_dmp_path =  pick_dmp_right_path
            pick_offset = [-0.025, 0.015, -0.025]

        elif left_boundary < cube.position[1]:
            pick_dmp_path =  pick_dmp_left_path
            pick_offset = [-0.03, 0.02, -0.015]
        
            

        if right_boundary <= end_rod.position[1] <= left_boundary:
            place_dmp_path =  place_dmp_front_path
            place_offset = [-0.025, 0.015, 0.01]

        elif right_boundary > end_rod.position[1]:
            place_dmp_path =  place_dmp_right_path
            # place_offset = [-0.035, 0.017, 0.03]
            place_offset = [-0.005, 0.017, 0.04]

        elif left_boundary < end_rod.position[1]:
            place_dmp_path =  place_dmp_left_path
            place_offset = [-0.035, 0.025, 0.025]
            # kleiner cube


        
        
        # RETURN TO HOME
        if return_home:
            print("\n=== Returning to Home Position ===")
            publisher.publish_home_position(
                current_position=home_position_open,
                home_position=home_position_open,
                execution_time=5.0
            )
            rospy.sleep(6.0)  # Wait for the robot to return to home position

        # 1. PICK MOTION - Blue Cube
        print(f"using path {pick_dmp_path}")
        joint_position = execute_motion(
                dmp_gen=dmp_gen,
                dmp_save_path=pick_dmp_path,
                cube_name=cube.name,
                position_offset=pick_offset,  # Slight offset above cube
                publisher=publisher,
                motion_name="pick",
                execute_time_factor=3,
                visualize=False,  # Set to False to skip visualization
                publish_enable=True,  # Set to True to publish trajectory
        )
        
        if joint_position is []:
            print("Pick motion failed!")
            exit(1)
        
        # 2. RETURN TO HOME
        print("\n=== Returning to Home Position ===")
        publisher.publish_home_position(
            current_position=joint_position,
            home_position=home_position_close,
            execution_time=5.0
        )
        print("[Home] Waiting for home position...")
        rospy.sleep(15.0)  # Wait for home position completion
        print("[Home] Home position reached!")
        
        pos_hight = 0.01

        if len(toh.rods[end_rod.name].cubes):
            pos_hight = len(toh.rods[end_rod.name].cubes) * 35 / 1000 + 0.04
            # end_cube_name = toh.rods[end_rod.name].cubes[-1].name
        
        end_rod.position[2] = pos_hight 

        joint_position = execute_motion(
            dmp_gen=dmp_gen,
            dmp_save_path=place_dmp_path,
            cube_name="",
            position_offset=place_offset,  # Offset above green cube for placing
            publisher=publisher,
            motion_name="place",
            execute_time_factor=3,
            visualize=False, # Set to True if you want to visualize
            target_pos=end_rod.position,  # Use the peg position for placing
            publish_enable=True,  # Set to True to publish trajectory
        )
        
        if joint_position is []:
            print("Place motion failed!")
            exit(1)
        
        # 4. FINAL RETURN TO HOME
        print("\n=== Final Return to Home Position ===")
        publisher.publish_home_position(
            current_position=joint_position,
            home_position=home_position_open,
            execution_time=5.0
        )
        print("[Final] Returning to home position...")
        rospy.sleep(6.0)
        
        print("\n=== Pick and Place Operation Completed Successfully! ===")
        
    except rospy.ROSInterruptException:
        print("[Main] ROS interrupted.")
    except Exception as e:
        print(f"[Main] Error during pick and place operation: {e}")
        import traceback
        traceback.print_exc()

def get_problem_description(toh: TowerOfHanoi, target_rod_name):
    num_disks = len(toh.cubes)

    result = "### Current Problem:\n"
    result += f"- Number of Disks: {num_disks}\n"
    result += "- Starting State (bottom disk first, top disk last):\n"

    for rod_name in sorted(toh.rods.keys()): 
        rod = toh.rods[rod_name]
        disk_values = [cube.value for cube in rod.cubes]
        result += f"    Rod {rod.name}: {disk_values}\n"

    
    result += "- Target State:\n"
    for rod_name in sorted(toh.rods.keys()):
        if rod_name == target_rod_name:
            # Target rod gets all disks, largest on bottom
            target_stack = list(range(num_disks, 0, -1))
            result += f"    Rod {rod_name}: {target_stack}\n"
        else:
            result += f"    Rod {rod_name}: []\n"

    return result


def get_yes_no(prompt, default="n"):
    """Prompt for yes/no input with optional default."""
    while True:
        choice = input(f"{prompt} ").strip().lower()
        if choice in {"y", "n", ""}:
            return choice if choice else default
        print("Invalid input. Please enter 'y' or 'n'.")

def get_valid_input(prompt, valid_values, to_upper=False):
    """Prompt until input is in valid_values."""
    while True:
        value = input(prompt).strip()
        if to_upper:
            value = value.upper()
        if value in valid_values:
            return value
        print(f"Invalid input. Please enter one of: {', '.join(valid_values)}.")


if __name__ == "__main__":


    # Configuration
    urdf_path = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/urdf/open_manipulator_6dof.urdf'
    mesh_path = '/root/catkin_ws/src/open_manipulator_friends/open_manipulator_6dof_description/meshes'
    
    # DMP paths
    pick_dmp_left_path = '/root/catkin_ws/src/recordings/pick_left_motion10.pkl'
    pick_dmp_front_path = '/root/catkin_ws/src/recordings/pick_front_motion10.pkl'
    pick_dmp_right_path = '/root/catkin_ws/src/recordings/pick_right_motion10.pkl'

    place_dmp_left_path = '/root/catkin_ws/src/recordings/place_left_motion10.pkl'
    place_dmp_front_path = '/root/catkin_ws/src/recordings/place_front_motion10.pkl'
    place_dmp_right_path = '/root/catkin_ws/src/recordings/place_right_motion10.pkl'
    

    # Set the rod positions corresponding to the cube positions

    toh = TowerOfHanoi()
    toh.addCube(Cube('cube_2',1))
    toh.addCube(Cube('cube_3',2))
    toh.addCube(Cube('cube_4',3))

    print("\n\n=============== Mark Rod Positions ===============\n")
    while True:
        choice = input("Do you want to choose Rod Positions (y/n): ").strip().lower()
        if choice == 'y':
            print("Please mark the rod positions with the cubes \n")
            print("The smallest cube value correspods to the position of rod 'A', the second 'B' and so on... \n")
            wait_for_user()
            toh.init_cubes()

            toh.addRod(Rod('A', toh.cubes[1].position))
            toh.addRod(Rod('B', toh.cubes[2].position))
            toh.addRod(Rod('C', toh.cubes[3].position))
            break

        elif choice == 'n':
            print("Using default rod positions. \n")
            toh.init_cubes()
            peg_positions = {
                "A": [0.16, 0.11, 0.05],
                "B": [0.18, 0.0, 0.05],
                "C": [0.16, -0.11, 0.05]
            }
            toh.addRod(Rod('A', peg_positions["A"]))
            toh.addRod(Rod('B', peg_positions["B"]))
            toh.addRod(Rod('C', peg_positions["C"]))
            break

        else:
            print("Invalid input. Try again.\n")

    print("\n\nSet rod positions to:\n")
    toh.printRodPositions()
    

    # Wait for User to Build Towers
    print("\n\n=============== Set Starting Position ===============\n")
    print("\nPlease build the starting position.")
    input("press Enter to continue")

    movements = ""
    while True:
        toh.assign_cubes_to_rods()
        use_llm = get_yes_no("Do you want to use the LLM (y/N):", default="n")

        if use_llm == 'y':
            print("\nPlease enter a target rod for the final tower.")
            
            target_rod = get_valid_input(
                f"Enter target rod {toh.getRodNames()}: ",
                toh.getRodNames(),to_upper=True
            )
            print(f"Selected Target Rod: {target_rod}")

            prompt = get_problem_description(toh, target_rod)
            movements = ask_llm(prompt)
            print(movements.output_text)
            movements = movements.output_text
            #TODO LLM
           
        else:
            use_default_manual = get_yes_no("Do you want to use the fixed default manual sequence? (y/N):", default="n")
            
            if use_default_manual == 'y':
                print("\nUsing default manual Tower of Hanoi sequence.")
                movements = (
                    "1. MD1AC\n"
                    "2. MD2AB\n"
                    "3. MD1CB\n"
                    "4. MD3AC\n"
                    "5. MD1BA\n"
                    "6. MD2BC\n"
                    "7. MD1AC"
                )
                print(f"\nDefault sequence:\n{movements}")

            else:
                movements = ""
                movements_count = 0
                print("\nPlease input the movements you want to execute:\n")

                while True:
                    
                    cube     = get_valid_input("Select cube (1, 2, 3): ", {"1", "2", "3"})
                    cube_rod = get_valid_input("Select cube rod (A, B, C): ", {"A", "B", "C"}, to_upper=True)
                    rod      = get_valid_input("Select rod (A, B, C): ", {"A", "B", "C"}, to_upper=True)

                    # todo chack wrong input

                    movements_count += 1

                    movements = movements + f"{movements_count}. MD{cube}{cube_rod}{rod}\n"
                    print(f"Movements so far:\n{movements}")

                    add_more = get_yes_no("Do you want to add one more movement? (y/N):", default="n")  

                    if add_more == 'n':
                        break


        print(f"\nFinal movements:\n{movements}")
        
        execute = get_yes_no("Do you want to execute these movements? (Y/n):", default="y")

        if execute == "y":
            print()
            parse_response(movements, toh)
            toh.execute_movements()
            movements = ""

        exit_program = get_yes_no("\nDo you want to exit? (Y/n):", default="y")
        if exit_program == "y":
            break
            

