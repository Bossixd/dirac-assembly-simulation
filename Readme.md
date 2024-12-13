# Documentation for Dirac Inc,.
Contact [pcharoen3@gatech](pcharoen3@gatech) for clarifications

# Program Overview
The 3D modelling software used to create the animation and run the scripts is Blender. You can install it [here](https://docs.blender.org/manual/en/latest/getting_started/installing/index.html). To be able to see debug outputs in blener, run the Blender executable from the command line. The Blender executable should be located at `/Applications/Blender.app/Contents/MacOS` on Mac.  
There are a total of 4 scripts used to create the animation, ordered by the number at the beginning of the file name.  
This solution for the problem "Assembly Simulation for Components with Non-Rigid Bodies" utilizes the A* search algorithm with contraints included to limit the flexibility of the wire.

# 1. Normalize Spline
1-normalize-spline.py is a script used to normalize any spline into a set of points with a distance of 1 between them.

**`original_points` will have to be filled in!**

## Required Libraries
- numpy
- typing | `List`, `Tuple`

## Required Variables
| Variable Name | Description | Type |
| ------------- | ----------- | ---- |
| original_points | Points of the created Bezier curve. | `List[Tuple[float, float, float]]` |

## Functions
### resample_curve_points(points, target_distance)
Resample points on a curve to have a constant distance between points.

This is done by interpolating between the points of an already existing curve. First, the cumulative distances is calculated to know which segment each point lies on. Then the number of segments or new points is calculated. For each segment, the target_distance_along_curve is updated to check if the end of the wire has been reached. Then, the segment the point lies in is found and the distance between the points is calculated. Finally, the location of the point is determined by calculating the relative distance between the original segment's points.

**Inputs:**
| Variable Name | Description | Type | Default |
| ------------- | ----------- | ---- | ------- |
| points | Points on the curve. | `List[Tuple[float, float, float]]` | |
| target_distance | Distance between each point on the wire. | `float` | `1.0` |

**Output:**  
List of points on the curve | `List[Tuple[float, float, float]]`

---
### round_to_integers(points)

Rounds a tuple of floats representing a point in space to integer values.

**Inputs:**
| Variable Name | Description | Type | Default |
| ------------- | ----------- | ---- | ------- |
| points | Points to be rounded. | `List[Tuple[float, float, float]]` | |

**Output:**  
List of rounded points | `List[Tuple[int, int, int]]`

## Usage
Run `resample_curve_points()` and `round_to_integers()` on `original_points`. Print the `integer_points` to be used in the next script.

# 2. Create Blue Wire
2-create-blue-write.py is a script used to create the wire in the 3D modelling software.

## Required Libraries
- bpy
- bpy_types
- math
- typing | `List`, `Tuple`

## Required Variables
| Variable Name | Description | Type |
| ------------- | ----------- | ---- |
| scene | Blender scene. | `bpy.types.Scene` |
| control_point_coords | Coordinates of the wire to be modelled. | `List[Tuple[float, float, float]]`|


## Functions
### create_curve_object(control_point_coords, scene)
Creates a curve object in the specified scene in Blender.

A bezier curve will also be created under the curve object. The curve's control points will be set as the specified control points. To center the wire around one of the wire's ends and not the origin, each point's position is adjusted based on the wire's new center.

**`control_point_coords` will have to be filled in!**

**Inputs:**
| Variable Name | Description | Type | Default |
| ------------- | ----------- | ---- | ------- |
| control_point_coords | Points on the curve. | `List[Tuple[float, float, float]]` | |
| scene | Blender scene object. | `bpy.types.Scene` | |

**Output:**  
Curve Object | `bpy_types.Object`

---
### create_circle_object(points)

Creates a circle object with a specified radius and number of segments. This circle will be used as the wire's body and connected to the curve object created in the previous function.

**Inputs:**
| Variable Name | Description | Type | Default |
| ------------- | ----------- | ---- | ------- |
| radius | Radius of the circle. | `float` | |
| segments | Circle Segments. | `int` | |

**Output:**  
Circle Object | `bpy_types.Object`

## Usage
Run `create_curve_object()` and `create_circle_object()` to create the curve and cirble objects. Add `Screw` and `Curve` modifiers to the circle and hook the curve object as a parameter to the `Curve` modifier on the screw. Set the circle object's location to the curve object's location and add the circle object to the scene.

# 3. Modified A* Algorithm
2-modified-a-star-algorithm.py is a script used to search for a path for the wire to go through. 

**`control_point_coords` will have to be filled in!**

## Required Libraries
- bpy
- mathutils
- mathutils | `Vector`
- typing | `List`, `Tuple`
- heapq
- math
- 

## Required Variables
| Variable Name | Description | Type |
| ------------- | ----------- | ---- |
| start_point | Position of wire's head. | `Tuple[float, float, float]`|
| last_point | Position of wire's second point. | `Tuple[float, float, float]`|
| goal_point | Target position for A* Algorithm. | `Tuple[float, float, float]`|
| control_point_coords | Coordinates of the wire to be modelled. | `List[Tuple[float, float, float]]`|

## Classes
### Node
#### \_\_init\_\_(self, position, g_cost, h_cost)
Sets object variables. Position, g_cost, and h_cost are as specified. Parent is `None`.

#### \_\_lt\_\_(self, other)
Compares the current `Node` and another `Node`. Used in A* algorithm's heap.

#### \_\_ep\_\_(self, other)
Compares whether the current `Node` and another `Node` are equal.

## Functions
### create_sphere_at_point(name, x, y, z)
Creates a sphere object at the specified location in the specified scene in Blender.

A bezier curve will also be created under the curve object. The curve's control points will be set as the specified control points. To center the wire around one of the wire's ends and not the origin, each point's position is adjusted based on the wire's new center.


**Inputs:**
| Variable Name | Description | Type | Default |
| ------------- | ----------- | ---- | ------- |
| name | Name of the circle object. | `str` | |
| x | `x` coordinate of the sphere. | `int` | |
| y | `y` coordinate of the sphere. | `int` | |
| z | `z` coordinate of the sphere. | `int` | |

**Output:**  
Sphere Object | `bpy_types.Object`

---
### distance(v1, v2)

Calculates the euclidean distance between the vectors.

**Inputs:**
| Variable Name | Description | Type | Default |
| ------------- | ----------- | ---- | ------- |
| v1 | First vector. | `Vector` | |
| v2 | Second vector. | `Vector` | |

**Output:**  
Distance between vectors | `float`

---
### collision_detection_direction(origin, direction)

Performs a raycast from the origin towards the direction of the direction vector.

Performs a raycast from the origin torwards the direction with a distance of `10.0` until no more objects can be detected. If the collisions between the ray and an object are even, the origin is not inside the object. If the collisions are odd, the origin is inside the object.

**Inputs:**
| Variable Name | Description | Type | Default |
| ------------- | ----------- | ---- | ------- |
| origin | Origin of the raycast. | `Vector` | |
| direction | Direction of the raycast. | `Vector` | |

**Output:**  
Whether the origin is inside an object | `boolean`

---
### collision_detection(origin)

A wrapper function for the `collision_detection_direction` function.

It contains the direction vectors and caches the results to speed up path finding. **May be inefficient since coordinate points are floating point values**.

**Inputs:**
| Variable Name | Description | Type | Default |
| ------------- | ----------- | ---- | ------- |
| origin | Origin of the raycast. | `Vector` | |

**Output:**  
Whether the origin is inside an object | `boolean`

---
### create_orthogonal_basis(direction)

Calculates an orthonormal basis to the specified direction vector.

The generated basis will be used to generate possible paths in the A* algorithm. First, the direction is normalized, and Gram Schmidt is performed on vectors parallel to the coordinate axes.

**Inputs:**
| Variable Name | Description | Type | Default |
| ------------- | ----------- | ---- | ------- |
| origin | Origin of the raycast. | `Vector` | |

**Output:**  
Generated bases | `List[Vector]`

---
### generate_next_paths(start_point, last_point, max_turn_angle)

Generates next possible paths given 2 points of a wire and a max turn angle constraint.

The orthonormal bases, and sin and cos values are generated. Possible paths are straight and the points with maximum turn angle.

![image info](./assets/Path.png)

**Inputs:**
| Variable Name | Description | Type | Default |
| ------------- | ----------- | ---- | ------- |
| start_point | Current point in A* search. | `Vector` | |
| last_point | Previous point in A* search. | `Vector` | |
| max_turn_angle | Maximum turn angle of the wire | `int` | |

**Output:**  
Next possible paths and movement cost  | `List[Tuple[Tuple, int, int, int], int]`

---
### heuristic(a, b)

Calculates a heuristic for the A* Algorithm.

Calculates the 3D Euclidean distance between two points

**Inputs:**
| Variable Name | Description | Type | Default |
| ------------- | ----------- | ---- | ------- |
| a | First vector. | `Vector` | |
| b | Second vector. | `Vector` | |

**Output:**  
Heuristic value  | `float`

---
### get_neighbors(current)

Wrapper function for `generate_next_paths`. Passes the curret position of the `Node` and the parent position.

**Inputs:**
| Variable Name | Description | Type | Default |
| ------------- | ----------- | ---- | ------- |
| current | Current Node. | `Node` | |

**Output:**  
Neighbors  | `List[Node]`

---
### find_path_3d(start, last, goal, bounds, step_size, rolerance)

A* Algorithm to find the path.

**Inputs:**
| Variable Name | Description | Type | Default |
| ------------- | ----------- | ---- | ------- |
| start | Position of wire's head. | `Tuple[float, float, float]` | |
| last | Position of wire's second point. | `Tuple[float, float, float]` | |
| goal | Target position for A* Algorithm. | `Tuple[float, float, float]` | |
| tolerance | Distance from `goal` such that the search is complete. | `float` | `1.0` |

**Output:**  
Path from the start position to the end position  | `List[Tuple[float, float, float]]`

---
### visualize_path(start, last, goal, bounds, step_size, rolerance)

Visualizes path with spheres.

**Inputs:**
| Variable Name | Description | Type | Default |
| ------------- | ----------- | ---- | ------- |
| start | Position of wire's head. | `Tuple[float, float, float]` | |
| last | Position of wire's second point. | `Tuple[float, float, float]` | |
| goal | Target position for A* Algorithm. | `Tuple[float, float, float]` | |
| tolerance | Distance from `goal` such that the search is complete. | `float` | `1.0` |

**Output:**  
Path from the start position to the end position  | `List[Tuple[float, float, float]]`

## Usage
Run `find_path_3d` to generate path coordinates for the wire. Run `visualize_path` to visualize the path and print coordinates, which will be used by the next script to animate the wire. 

# 4. Animation
4-animation.py is a script used to animate the wire's trajectory.

## Required Libraries
- bpy
- mathutils

## Required Variables
| Variable Name | Description | Type |
| ------------- | ----------- | ---- |
| path | Path of the wire's trajectory. | `List[Tuple[float, float, float]]` |

## Usage
Get curve object and the curve's points. For each step, shift every point on the curve forward. The curve's head will be shifted to the next point in the path trajectory.
