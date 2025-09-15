from hpp.corbaserver.manipulation.ur5 import Robot
from hpp.corbaserver.tools import Tools

from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import ProblemSolver, newProblem

loadServerPlugin("corbaserver", "manipulation-corba.so")

newProblem()
robot = Robot("robot", "ur5")
ps = ProblemSolver(robot)
distance = ps.hppcorba.problem.getDistance()

q0 = robot.getCurrentConfig()
qr = robot.shootRandomConfig()
print(distance.value(q0, qr))
weights = distance.getWeights()
weights[0] = 0
distance.setWeights(weights)
print(distance.value(q0, qr))

tools = Tools()
ior = ps.hppcorba.orb.object_to_string(distance)
tools.deleteServant(ior)

tools.shutdown()
