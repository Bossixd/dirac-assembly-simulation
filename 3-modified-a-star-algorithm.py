import bpy
import mathutils
from typing import List, Tuple
import heapq
import math
from mathutils import Vector

start_point = (-23.40901756286621, 32.55461120605469, 0.03466916084289551)
last_point = (-24.40901756286621, 32.55461120605469, 0.03466916084289551)
goal_point = (0, 200, 2)
searched = {}

def create_sphere_at_point(name: str, x: float, y: float, z: float):
    """
    Create a sphere at the specified point with a unique name.
    
    Parameters:
    name (str): Base name for the sphere
    x, y, z (float): Coordinates for sphere center
    radius (float): Radius of the sphere
    
    Returns:
    bpy.types.Object: The created sphere object
    """
    # Create unique names for both the object and mesh
    sphere_name = f"Sphere_{name}"
    mesh_name = f"SphereMesh_{name}"
    
    # Check if an object with this name already exists
    if sphere_name in bpy.data.objects:
        # Either return the existing object or remove it
        existing_obj = bpy.data.objects[sphere_name]
        bpy.data.objects.remove(existing_obj, do_unlink=True)
    
    # Create new sphere
    bpy.ops.mesh.primitive_uv_sphere_add(
        radius=0.5,
        location=(x, y, z)
    )
    
    # Get the created sphere and set its name
    sphere = bpy.context.active_object
    sphere.name = sphere_name
    sphere.data.name = mesh_name
    
    return sphere

def distance(v1: Vector, v2: Vector) -> float:
    """
    Calculate Euclidean distance between two vectors.

    Parameters:
    v1, v2 (Vector): The vectors

    Returns:
    float: The Euclidean distance between the two vectors
    """
    s = 0
    for i in range(3):
        s += v1[i] - v2[i]
    if s < 0:
        s *= -1
        
    return math.sqrt(s)

def collision_detection_direction(origin: Vector, direction: Vector):
    """
    Detects collision in a given direction from an origin point in the Blender scene.

    This function performs a raycast from the specified origin point in the given 
    direction to detect collisions with objects in the Blender scene. It maintains 
    a count of how many times each object is intersected, storing the result in 
    a dictionary. The function returns True if the number of intersections for 
    any object is odd, indicating a collision.

    Parameters:
    origin (Vector): The starting point of the ray.
    direction (Vector): The direction in which the ray is cast.

    Returns:
    bool: True if a collision is detected, otherwise False.
    """
    scene = bpy.context.scene
    
    # Perform the raycast
    current_position = origin
    i = 0
    
    collision_dict = {}
    
    while (True):
        i += 1
        result, location, normal, index, obj, matrix = scene.ray_cast(
            bpy.context.view_layer.depsgraph,
            origin=current_position,           # Starting point of the ray
            direction=direction,               # Direction of the ray
            distance=10.0                      # Maximum distance to cast
        )
        
        if result:
            if obj.name == "wire_blue_circle_g":
                continue
            
            if obj.name == "wire_blue_circle_t":
                continue
            
            current_position = location - (Vector(current_position) - location) * 0.001 / distance(Vector(current_position), location)
            
            if obj.name not in collision_dict:
                collision_dict[obj.name] = 1
            else:
                collision_dict[obj.name] += 1
        else:
            break
    
    for key in collision_dict:
        if collision_dict[key] % 2 == 1:
            return True
    return False 

def collision_detection(origin: Vector):
    """
    Detects if there is a collision at the given origin point in any of the six orthogonal directions.
    
    Parameters:
    origin (Vector): A 3D point (Vector) representing the starting position to check for collisions.
    
    Returns:
    bool: True if a collision is detected in any direction, False otherwise.
    """
    directions = [
        Vector((1, 0, 0)),
        Vector((-1, 0, 0)),
        Vector((0, 1, 0)),
        Vector((0, -1, 0)),
        Vector((0, 0, 1)),
        Vector((0, 0, -1))
    ]
    
    if f"{origin[0]},{origin[1]},{origin[2]}" in searched:
        return searched[f"{origin[0]},{origin[1]},{origin[2]}"]
    
    for direction in directions:
        if (collision_detection_direction(origin, direction)):
            searched[f"{origin[0]},{origin[1]},{origin[2]}"] = True
            return True
    searched[f"{origin[0]},{origin[1]},{origin[2]}"] = False
    return False

# ------------------------------------------------------------------------

def create_orthogonal_basis(direction: Vector):
    """
    Create an orthogonal basis from a given direction vector
    
    Parameters:
    direction (Vector): Primary direction vector
    
    Returns:
    Tuple[Vector, Vector, Vector] Tuple of orthogonal vectors (direction, v1, v2)
    """
    
    # Normalize the direction vector
    primary_direction = direction.normalized()
    
    axes = [
        mathutils.Vector((0, 0, 1)),
        mathutils.Vector((1, 0, 0)),
        mathutils.Vector((0, 1, 0))
    ]
    
    # Gram Schmidt
    vectors = [
        primary_direction
    ]
    
    for axis in axes:
        if len(vectors) == 3:
            break

        new_direction = mathutils.Vector(axis)
        
        for vec in vectors:
            new_direction = new_direction - vec*(vec.dot(axis))/(vec.dot(vec))

        if new_direction == mathutils.Vector((0, 0, 0)):
            continue

        vectors.append(new_direction.normalized())
    
    return vectors

def generate_next_paths(start_point: Vector, last_point: Vector, max_turn_angle: int = 30):
    """
    Generate possible next path directions from start to last point
    
    Parameters:
        start_point (Vector): Starting 3D point
        last_point (Vector): Previous point in the path
        max_turn_angle (int): Maximum allowed turn angle in degrees
    
    Returns:
        List[Tuple[Vector, int]]: List of possible next path directions
    """
    # Calculate the current path direction
    current_direction = (start_point - last_point).normalized()
    
    # Generate Basis for end of travel cone
    bases = create_orthogonal_basis(current_direction)
    
    sin = abs(math.sin(max_turn_angle))
    cos = abs(math.cos(max_turn_angle))
    
    # Generate Possible Paths
    possible_paths = []
    for basis in bases:
        direction = mathutils.Vector(current_direction)
        direction = direction * sin
        
        aug_direction = direction + basis * cos
        res_path = start_point + aug_direction
        possible_paths.append((res_path.to_tuple(), 1))
        
        if (current_direction != basis):
            aug_direction = direction - basis * cos
            res_path = start_point + aug_direction
            possible_paths.append((res_path.to_tuple(), 1))
    
    return possible_paths

# ------------------------------------------------------------------------

class Node:
    def __init__(self, 
                    position: Tuple[float, float, float], 
                    g_cost: float = float('inf'), 
                    h_cost: float = float('inf')):
        self.position = position
        self.g_cost = g_cost  # Cost from start to current node
        self.h_cost = h_cost  # Estimated cost from current node to end
        self.f_cost = g_cost + h_cost  # Total estimated cost
        self.parent = None

    def __lt__(self, other):
        return self.f_cost < other.f_cost

    def __eq__(self, other):
        return self.position == other.position

def heuristic(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    """Calculate the 3D Euclidean distance between two points."""
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

def get_neighbors(current: Node) -> List[Node]:
    """
    Get all possible neighboring positions in 3D space.

    Parameters:
        current: Current node
        step_size: Distance between neighboring points
        bounds: Tuple of (min, max) for each dimension ((x_min, x_max), (y_min, y_max), (z_min, z_max))

    Returns:
        List of neighboring Node objects
    """
    neighbors = generate_next_paths(mathutils.Vector(current.position), mathutils.Vector(current.parent.position))

    return neighbors

def find_path_3d(start: Tuple[float, float, float], 
                    last: Tuple[float, float, float],
                    goal: Tuple[float, float, float],
                    tolerance: float = 1.0) -> List[Tuple[float, float, float]]:
    """
    Find a path from start to goal in 3D space using A* algorithm.
    
    Args:
        start: Starting point coordinates (x, y, z)
        goal: Goal point coordinates (x, y, z)
        occupied_points: Set of occupied point coordinates that cannot be passed through
        bounds: Tuple of (min, max) for each dimension ((x_min, x_max), (y_min, y_max), (z_min, z_max))
        step_size: Distance between neighboring points
        tolerance: Distance within which a point is considered to have reached the goal
    
    Returns:
        List of points representing the path from start to goal
    """
    # Initialize start node
    start_node = Node(start, 0.0, heuristic(start, goal))
    start_node.parent = Node(last, 0.0, heuristic(start, goal))
    
    # Initialize open and closed sets
    open_set = []
    heapq.heappush(open_set, start_node)
    closed_set = set()
    
    # Dictionary to keep track of nodes by position
    nodes = {start: start_node}
    
    while open_set:
        current = heapq.heappop(open_set)

        if (collision_detection(current.position)):
            continue
        
        # Check if we've reached the goal (within tolerance)
        if heuristic(current.position, goal) <= tolerance:
            # Reconstruct path
            path = []
            while current:
                path.append(current.position)
                current = current.parent
            return list(reversed(path))
        
        closed_set.add(current.position)
        
        # Get neighboring positions
        for neighbor_pos, movement_cost in get_neighbors(current):
            if neighbor_pos in closed_set:
                continue
                
            # Calculate new cost to neighbor
            new_g_cost = current.g_cost + movement_cost
            
            # Get or create neighbor node
            if neighbor_pos not in nodes:
                neighbor = Node(neighbor_pos)
                nodes[neighbor_pos] = neighbor
            else:
                neighbor = nodes[neighbor_pos]
            
            if new_g_cost >= neighbor.g_cost:
                continue
            
            # Update neighbor costs and parent
            neighbor.parent = current
            neighbor.g_cost = new_g_cost
            neighbor.h_cost = heuristic(neighbor_pos, goal)
            neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
            
            if neighbor.position not in [n.position for n in open_set]:
                heapq.heappush(open_set, neighbor)
    
    # No path found
    return []

def visualize_path(path):
    """
    Create spheres at each point along the path.
    
    Parameters:
    path: List of (x, y, z) coordinates
    scene: Blender scene
    sphere_radius: Radius for the sphere markers
    """
    if path:
        print("Creating spheres along path:")
        for index, point in enumerate(path):
            for ob in bpy.data.objects:
                if ob.name.startswith("Sphere"):
                    continue
            print(f"{point},")
            create_sphere_at_point(
                str(index), 
                point[0], 
                point[1], 
                point[2]
            )
            
        print(f"Created {len(path)} spheres")
    else:
        print("No path found!")

def main():
    # Find path
    path = find_path_3d(
        start_point,
        last_point,
        goal_point,
        tolerance=0.5
    )
    
    if path:
        visualize_path(path)
        pass
    else:
        print("No path found!")

# Example usage:
if __name__ == "__main__":
    main()
