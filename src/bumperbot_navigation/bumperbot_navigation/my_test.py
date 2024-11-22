import math
import fields2cover as f2c

# Define coordinates (replace these with your actual coordinates)
import fields2cover as f2c

# Define coordinates
coordinates = [
    [0.0, 0.0],
    [0.0, 100.0],
    [100.0, 100.0],
    [100.0, 0.0],
    [0.0, 0.0]
]

# Create a vector of f2c::types::Point objects
points = f2c.VectorPoint()
for x, y in coordinates:
    point = f2c.Point(x, y)
    points.push_back(point)


# Create LinearRing from points
field_ring = f2c.LinearRing(points)


cell = f2c.Cell()
cell.addRing(field_ring)

cells = f2c.Cells()
cells.addGeometry(cell)

field = f2c.Field(cells)

orig_field = field.clone()

# Transform into UTM to work in meters
# f2c.Transform.transformToUTM(field)

robot = f2c.Robot(2.0, 6.0)
const_hl = f2c.HG_Const_gen()
no_hl = const_hl.generateHeadlands(field.field, 3.0 * robot.robot_width)
bf = f2c.SG_BruteForce()
swaths = bf.generateSwaths(math.pi, robot.op_width, no_hl.getGeometry(0))
snake_sorter = f2c.RP_Snake()
swaths = snake_sorter.genSortedSwaths(swaths)

robot.setMinRadius(2)
path_planner = f2c.PP_PathPlanning()
dubins = f2c.PP_DubinsCurves()
path = path_planner.searchBestPath(robot, swaths, dubins)

f2c.Visualizer.figure(71)
f2c.Visualizer.plot(field)
f2c.Visualizer.plot(no_hl)
f2c.Visualizer.plot(path)
f2c.Visualizer.save("Tutorial_7_1_UTM_MINE")

fig = f2c.Visualizer()
fig.figure()
fig.plot(field)
# fig.plot(swaths)
fig.plot(no_hl)
fig.plot(path)
fig.show()
print('end')

# Transform the generated path back to the previous CRS.
# path_gps = f2c.Transform.transformToPrevCRS(path, field)
# f2c.Transform.transformToPrevCRS(field)

# f2c.Visualizer.figure(72)
# f2c.Visualizer.plot(orig_field.getCellsAbsPosition())
# f2c.Visualizer.plot(path_gps)
# f2c.Visualizer.save("Tutorial_7_1_GPS")