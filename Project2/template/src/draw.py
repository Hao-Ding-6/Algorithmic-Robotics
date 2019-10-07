# Visualization tool
# Author: Haoran Liang

import matplotlib.pyplot as plt
import matplotlib.patches as patches

figure = plt.figure()
obstacles = figure.add_subplot(1,1,1)
# Drawing obstacles
# middle
obstacles.add_patch(
	patches.Rectangle((0.5, 0), 2.0, 1.0)
)

# upper middle
obstacles.add_patch(
	patches.Rectangle((-1.0, 2.0), 2.0, 1.0)
)

# lower middle
obstacles.add_patch(
	patches.Rectangle((-1.0, -2.0), 2.0, 1.0)
)

# left
obstacles.add_patch(
	patches.Rectangle((-3.0, -2.5), 1.0, 5.0)
)

# right
obstacles.add_patch(
	patches.Rectangle((3.35, -1.25), 0.5, 3.75)
)

##### Add robot here #####
# Draw a point
plt.plot(1.76196, 0.610881, 'bo')

figure.savefig('draw.png')