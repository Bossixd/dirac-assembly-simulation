import bpy
import bpy_types
import math
from typing import List, Tuple

scene = bpy.context.scene

# Normalized Spline Coords
control_point_coords = [
    (-65,28,-119),
    (-64,29,-119),
    (-63,29,-119),
    (-62,29,-119),
    (-61,30,-119),
    (-60,30,-119),
    (-59,30,-119),
    (-58,31,-119),
    (-57,31,-119),
    (-56,31,-119),
    (-55,32,-119),
    (-54,32,-119),
    (-54,33,-119),
    (-53,33,-118),
    (-52,34,-118),
    (-51,34,-117),
    (-50,35,-117),
    (-50,35,-116),
    (-49,35,-115),
    (-49,35,-115),
    (-48,36,-114),
    (-47,36,-113),
    (-47,36,-113),
    (-46,37,-112),
    (-46,37,-111),
    (-45,37,-110),
    (-45,37,-109),
    (-45,37,-108),
    (-45,37,-107),
    (-44,37,-106),
    (-44,37,-105),
    (-44,37,-104),
    (-44,37,-103),
    (-43,37,-102),
    (-43,37,-101),
    (-43,37,-100),
    (-43,37,-99),
    (-43,37,-98),
    (-43,37,-97),
    (-43,37,-96),
    (-43,37,-95),
    (-43,37,-94),
    (-43,37,-93),
    (-43,37,-92),
    (-43,37,-91),
    (-43,37,-90),
    (-43,37,-89),
    (-43,37,-88),
    (-43,37,-87),
    (-43,37,-86),
    (-43,37,-85),
    (-43,37,-84),
    (-43,37,-83),
    (-43,37,-82),
    (-43,37,-81),
    (-43,37,-80),
    (-43,37,-79),
    (-43,37,-78),
    (-43,37,-77),
    (-43,37,-76),
    (-43,37,-75),
    (-43,37,-74),
    (-43,37,-73),
    (-43,37,-72),
    (-43,37,-71),
    (-43,37,-70),
    (-43,37,-69),
    (-43,37,-68),
    (-43,37,-67),
    (-43,37,-66),
    (-43,37,-65),
    (-43,37,-64),
    (-43,37,-63),
    (-43,37,-62),
    (-43,37,-61),
    (-43,37,-60),
    (-44,37,-59),
    (-44,37,-58),
    (-44,37,-57),
    (-44,37,-56),
    (-44,37,-55),
    (-44,37,-54),
    (-44,37,-53),
    (-44,37,-52),
    (-44,37,-51),
    (-44,37,-50),
    (-44,37,-49),
    (-44,37,-48),
    (-44,37,-47),
    (-44,37,-46),
    (-44,37,-45),
    (-44,37,-44),
    (-44,37,-43),
    (-44,37,-42),
    (-44,37,-41),
    (-44,37,-40),
    (-44,37,-39),
    (-44,37,-38),
    (-44,37,-37),
    (-44,37,-36),
    (-44,37,-35),
    (-44,37,-34),
    (-44,37,-33),
    (-44,37,-32),
    (-44,37,-31),
    (-44,37,-30),
    (-44,37,-29),
    (-44,37,-28),
    (-44,37,-27),
    (-44,37,-26),
    (-44,37,-25),
    (-44,37,-24),
    (-44,37,-23),
    (-44,37,-22),
    (-44,37,-21),
    (-44,37,-20),
    (-44,37,-19),
    (-44,37,-18),
    (-44,37,-17),
    (-43,37,-16),
    (-43,37,-15),
    (-43,37,-14),
    (-43,37,-13),
    (-43,37,-12),
    (-42,37,-11),
    (-42,37,-10),
    (-42,37,-9),
    (-42,37,-8),
    (-42,37,-8),
    (-41,37,-7),
    (-40,37,-6),
    (-39,37,-5),
    (-39,37,-5),
    (-38,37,-4),
    (-37,37,-3),
    (-37,37,-2),
    (-36,37,-2),
    (-35,37,-1),
    (-35,37,0),
    (-34,37,0),
    (-33,37,1),
    (-32,37,2),
    (-31,37,2),
    (-30,37,2),
    (-29,37,2),
    (-28,37,2),
    (-27,37,2),
    (-26,37,2),
    (-25,37,2),
    (-25,37,2),
]

def create_curve_object(control_point_coords: List[Tuple[float, float, float]], scene) -> bpy_types.Object:
    """
    Creates a curve object in the specified Blender scene.

    This function generates a new curve object with a Bezier spline and 
    adjusts its control points to center the curve around a specified point.
    It also links the curve object to the given scene for rendering.

    Parameters:
    control_point_coords (List[Tuple[float, float, float]]): Coordinates of control points for the Bezier curve.
    scene (bpy.types.Scene): The Blender scene to which the curve object will be linked.

    Returns:
    bpy_types.Object: The created curve object linked to the scene.
    """
    
    # Create a new curve object 
    curve_obj = bpy.data.objects.new('wire_blue_curve_g', bpy.data.curves.new('Curve', 'CURVE')) 

    # Add a new Bezier curve to the curve object 
    curve = curve_obj.data.splines.new('BEZIER') 

    # Set the resolution of the curve 
    curve.resolution_u = len(control_point_coords)

    curve.bezier_points.add(len(control_point_coords) - 1)

    curve_obj.location = control_point_coords[-1]

    x_adjustment = -curve_obj.location[0]
    y_adjustment = -curve_obj.location[1]
    z_adjustment = -curve_obj.location[2]

    curve_obj.location = (curve_obj.location[0], curve_obj.location[1], curve_obj.location[2])

    for i in range(len(control_point_coords)):

        curve.bezier_points[i].co = control_point_coords[i]
        curve.bezier_points[i].handle_left = control_point_coords[i]
        curve.bezier_points[i].handle_right = control_point_coords[i]
        
        # X Adjustment
        curve.bezier_points[i].co[0] += x_adjustment
        curve.bezier_points[i].handle_left[0] += x_adjustment
        curve.bezier_points[i].handle_right[0] += x_adjustment
        
        # Y Adjustment
        curve.bezier_points[i].co[1] += y_adjustment
        curve.bezier_points[i].handle_left[1] += y_adjustment
        curve.bezier_points[i].handle_right[1] += y_adjustment
        
        # Z Adjustment
        curve.bezier_points[i].co[2] += z_adjustment
        curve.bezier_points[i].handle_left[2] += z_adjustment
        curve.bezier_points[i].handle_right[2] += z_adjustment
        
    scene.collection.objects.link(curve_obj) 
    
    return curve_obj

# Circle -------------------------------------------------------------

def create_circle_object(radius: int, segments: int) -> bpy_types.Object:    
    """
    Create a circle object with the specified radius and number of segments.
    
    Parameters:
    radius (int): The radius of the circle
    segments (int): The number of segments to use for the circle
    
    Returns:
    bpy_types.Object: The created circle object
    """
    
    # Create a new mesh object
    mesh = bpy.data.meshes.new("CircleMesh")
    verts = []
    faces = []
    
    # Calculate vertex positions on a circle
    for i in range(segments + 1):
        angle = 2 * math.pi * i / segments
        x = math.cos(angle) * radius
        y = math.sin(angle) * radius
        verts.append((x, y, 0))
        
    # Create faces by connecting vertices in sequence
    for i in range(segments):
        faces.append((i, i + 1, segments + 1))
        
    # Add vertices and faces to mesh
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    
    # Create a new object from the mesh
    obj = bpy.data.objects.new("wire_blue_circle_g", mesh)
    return obj

def main():
    curve_obj = create_curve_object(control_point_coords, scene)
    circle_obj = create_circle_object(radius=1, segments=128)
    
    circle_obj.modifiers.new("Screw", "SCREW")
    circle_obj.modifiers[0].use_smooth_shade = True
    circle_obj.modifiers[0].screw_offset = 1
    circle_obj.modifiers[0].iterations = 155

    circle_obj.modifiers.new("Curve", "CURVE")
    circle_obj.modifiers[1].object = curve_obj
    circle_obj.modifiers[1].deform_axis = "POS_Z"

    circle_obj.location = curve_obj.location

    scene.collection.objects.link(circle_obj)
    
if __name__ == "__main__":
    main()
