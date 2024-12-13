import bpy
import mathutils
from mathutils import Vector
import math

def basic_raycast(origin, direction):
    """
    Perform a simple raycast from an origin in a specific direction
    """
    scene = bpy.context.scene
    
    # Perform the raycast
    result, location, normal, index, obj, matrix = scene.ray_cast(
        bpy.context.view_layer.depsgraph,
        origin=origin,           # Starting point of the ray
        direction=direction,     # Direction of the ray
        distance=100.0           # Maximum distance to cast
    )
    
    if result:
        print(f"Ray hit object: {obj.name}")
        print(f"Hit location: {location}")
        print(f"Surface normal: {normal}")
    else:
        print("Ray did not hit anything")
    
    return result, obj, location

def distance(v1, v2):
    s = 0
    for i in range(3):
        s += v1[i] - v2[i]
    if s < 0:
        s *= -1
    return math.sqrt(s)

def collision_detection(origin, direction):
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
            print(f"Ray hit object: {obj.name}")
            print(f"Hit location: {location}")
            print(f"Surface normal: {normal}")
            current_position = location - (Vector(current_position) - location) * 0.1/ distance(Vector(current_position), location)
            if obj.name not in collision_dict:
                collision_dict[obj.name] = 1
            else:
                collision_dict[obj.name] += 1
        else:
            print("Ray did not hit anything")
            print(collision_dict)
            break
    
    for key in collision_dict:
        if collision_dict[key] % 2 == 1:
            return True
    return False 
    
#    return result, obj, location

# Example usage
origin = (-25.0, 59.0, 50.0)
direction = mathutils.Vector((0, 0, -1))
print(collision_detection(origin, direction))
