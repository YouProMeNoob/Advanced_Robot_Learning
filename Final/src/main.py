import time
import math
import re
import rospy

from pick_and_place_V2 import DMPMotionGenerator, GazeboTrajectoryPublisher, execute_motion, get_cube_position

from chatgpt_call import ask_llm

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
        
        for rod in self.rods.values():
            rod.cubes.clear()

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
        for i, m in enumerate(self.movements[:]):

            if len(m.end_rod.cubes) == 0: # no cube on rod
                debug_target = f"empty rod"
            else:
                goal_cube = m.end_rod.cubes[-1].name # 
                debug_target = f"on top of {goal_cube}"

            print(f"Moving {m.cube.name} to rod {m.end_rod.name} ({debug_target})")
            pick_place(m.cube, m.start_rod, m.end_rod, return_home=(i==0))

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

def pick_place( cube: Cube, start_rod: Rod, end_rod: Rod, return_home:bool = False):
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

        offset_pick = 0.02 #(len(start_rod.cubes)-1)/10
        offset_place = 0.01 + (len(end_rod.cubes)) *0.04

        # TODO maybe adjust boundaries
        left_boundary = 0.04
        right_boundary = -0.04


        # Initialize components
        dmp_gen = DMPMotionGenerator(
            urdf_path, 
            mesh_path,
            base_link="world"
        )
        
        publisher = GazeboTrajectoryPublisher()
        rospy.sleep(2.0)
    
        # Select the path (left, right middle)
        if right_boundary <= cube.position[1] <= left_boundary:
            pick_dmp_path =  pick_dmp_front_path
        elif right_boundary > cube.position[1]:
            pick_dmp_path =  pick_dmp_right_path
        else:
            pick_dmp_path =  pick_dmp_left_path

        if right_boundary <= end_rod.position[1] <= left_boundary:
            place_dmp_path =  place_dmp_front_path
        elif right_boundary > cube.position[1]:
            place_dmp_path =  place_dmp_right_path
        else:
            place_dmp_path =  place_dmp_left_path
        
        
        # RETURN TO HOME
        if return_home:
            print("\n=== Returning to Home Position ===")
            publisher.publish_home_position(
                home_position=home_position,
                execution_time=5.0
            )
            rospy.sleep(6.0)  # Wait for the robot to return to home position

        # 1. PICK MOTION - Blue Cube
        print(f"using path {pick_dmp_path}")
        success = execute_motion(
                dmp_gen=dmp_gen,
                dmp_save_path=pick_dmp_path,
                cube_name=cube.name,
                position_offset=[0.0, 0.0, offset_pick],  # Slight offset above cube
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
        print("[Home] Waiting for home position...")
        rospy.sleep(6.0)  # Wait for home position completion
        print("[Home] Home position reached!")
        
        success = execute_motion(
            dmp_gen=dmp_gen,
            dmp_save_path=place_dmp_path,
            cube_name=cube.name,
            position_offset=[0.0, 0.0, offset_place],  # Offset above green cube for placing
            publisher=publisher,
            motion_name="place",
            execute_time_factor=5,
            visualize=False, # Set to True if you want to visualize
            target_pos=end_rod.position,  # Use the peg position for placing
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
    
    # Home position
    home_position = [-0.09970875084400177, -0.1902136206626892, 1.4710875749588013, 0.04295146465301514, 1.702718734741211, -0.058291271328926086]


    # Set the rod positions corresponding to the cube positions
    toh = TowerOfHanoi()
    toh.addCube(Cube('blue_cube',1))
    toh.addCube(Cube('red_cube',2))
    toh.addCube(Cube('green_cube',3))

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
            

