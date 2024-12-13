import bpy
import mathutils
from typing import List, Set, Dict, Tuple
import heapq
import math
import bmesh
from mathutils.bvhtree import BVHTree
from mathutils import Vector

# Try looking into MDP
# Markov Decision Process

scene = bpy.context.scene 

def create_sphere_at_point(name, x, y, z):
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

searched = {}

def distance(v1, v2):
    s = 0
    for i in range(3):
        s += v1[i] - v2[i]
    if s < 0:
        s *= -1
    return math.sqrt(s)

def collision_detection_direction(origin, direction):
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
            direction=direction,     # Direction of the ray
            distance=100.0           # Maximum distance to cast
        )
        
        if result:
#            print(f"Ray hit object: {obj.name}")
#            print(f"Hit location: {location}")
#            print(f"Surface normal: {normal}")
            current_position = location - (Vector(current_position) - location) * 0.001 / distance(Vector(current_position), location)
            if obj.name != "wire_blue_circle_g":
                if obj.name not in collision_dict:
                    collision_dict[obj.name] = 1
                else:
                    collision_dict[obj.name] += 1
        else:
#            print("Ray did not hit anything")
#            print(collision_dict)
            break
    
    for key in collision_dict:
        if collision_dict[key] % 2 == 1:
            return True
    return False 
    
#    return result, obj, location

def collision_detection(origin):
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

class Node:
    def __init__(self, position: Tuple[float, float, float], g_cost: float = float('inf'), 
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

def get_neighbors(current: Node, step_size: float, bounds: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]) -> List[Node]:
    """
    Get valid neighboring positions in 3D space.
    
    Args:
        current: Current node
        step_size: Distance between neighboring points
        occupied_points: Set of occupied point coordinates
        bounds: Tuple of (min, max) for each dimension ((x_min, x_max), (y_min, y_max), (z_min, z_max))
    """
    neighbors = []
    # Generate neighbors in all 26 directions (3D grid)
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            for dz in [-1, 0, 1]:
                if dx == 0 and dy == 0 and dz == 0:
                    continue

                new_pos = (
                    current.position[0] + dx * step_size,
                    current.position[1] + dy * step_size,
                    current.position[2] + dz * step_size
                )

                # Check bounds
                if not (bounds[0][0] <= new_pos[0] <= bounds[0][1] and
                       bounds[1][0] <= new_pos[1] <= bounds[1][1] and
                       bounds[2][0] <= new_pos[2] <= bounds[2][1]):
                    continue
                
                collision = False
                for ob in bpy.data.objects:
                    try:
                        if collision_detection(new_pos):
                            collision = True
                            break
                    except:
                        pass
                
                if collision:
                    continue

                # Calculate movement cost (diagonal movements cost more)
                movement_cost = step_size * math.sqrt(dx*dx + dy*dy + dz*dz)
                
                neighbors.append((new_pos, movement_cost))

    return neighbors

def find_path_3d(start: Tuple[float, float, float],
                 last: Tuple[float, float, float], 
                 goal: Tuple[float, float, float],
                 bounds: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]],
                 step_size: float = 1.0,
                 tolerance: float = 0.1) -> List[Tuple[float, float, float]]:
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
    start_node.parent = last
    
    # Initialize open and closed sets
    open_set = []
    heapq.heappush(open_set, start_node)
    closed_set = set()
    
    # Dictionary to keep track of nodes by position
    nodes = {start: start_node}
    
    while open_set:
        current = heapq.heappop(open_set)
        
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
        for neighbor_pos, movement_cost in get_neighbors(current, step_size, bounds):
            print(neighbor_pos)
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
        
        break
    # No path found
    return []

def visualize_path(path, scene):
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
            inside = False
            for ob in bpy.data.objects:
                if ob.name.startswith("Sphere"):
                    continue
                try:
                    if collision_detection(point):
                        inside = True
                        print(ob.name)
                except:
                    pass
            print(f"{point},")
            sphere = create_sphere_at_point(
                str(index), 
                point[0], 
                point[1], 
                point[2]
            )
            
            # The sphere is automatically linked to the active collection
            # No need to explicitly link it
            
            # Optionally, you can add materials or modify the sphere here
            
        print(f"Created {len(path)} spheres")
    else:
        print("No path found!")

# Example usage:
if __name__ == "__main__":
    # Define example space
    start_point = (-25.0, 37, 2.0)
    last_point = (-26.0, 37, 2.0)
    goal_point = (-25.0, 90, 2.0)
    
    # Define bounds of the space
    bounds = ((-100.0, 100.0), (-100.0, 100.0), (-100.0, 100.0))
    
    # Find path
    path = find_path_3d(
        start_point,
        last_point,
        goal_point,
        bounds,
        step_size=1.0,
        tolerance=0.5
    )
    
    if path:
        visualize_path(path, bpy.context.scene)
        pass
    else:
        print("No path found!")
