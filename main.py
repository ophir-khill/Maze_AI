from Robot import *
from MazeProblem import *
from Animation import Animation
from Heuristics import *
from Utilities import *
from Experiments import *


if __name__ == "__main__":
     # test_robot(BreadthFirstSearchRobot, [0])
    # test_robot(UniformCostSearchRobot, [99])
    test_robot(WAStartRobot, [4], heuristic=center_manhattan_heuristic)
    test_robot(WAStartRobot, [4], heuristic=ShorterRobotHeuristic, k=2)

# solve_and_display(BreadthFirstSearchRobot, 1)
#     solve_and_display(UniformCostSearchRobot, 6)
#     solve_and_display(WAStartRobot, 0, heuristic=center_manhattan_heuristic, w=0)
#     for i in [2, 3, 4, 5]:
#         shorter_robot_heuristic_experiment(i)