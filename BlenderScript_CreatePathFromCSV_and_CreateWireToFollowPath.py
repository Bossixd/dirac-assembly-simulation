import bpy
import csv
from mathutils import Vector

#Creating Path in Blender:
#----------------------------------------------------------

def load_csv(filepath):
    """Load CSV and extract 3D coordinates as a list."""
    coords_list = []
    with open(filepath, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            # Assuming each row has exactly 3 values: x, y, z
            coords_list.append([float(row[0]), float(row[1]), float(row[2])])
    return coords_list

def create_nurbs_curve_from_coords(coords_list, curve_name='NURBS_Curve'):
    """Create a 3D NURBS curve from a list of coordinates."""
    # Make a new curve
    crv = bpy.data.curves.new(curve_name, 'CURVE')
    crv.dimensions = '3D'

    # Make a new spline in that curve
    spline = crv.splines.new(type='NURBS')

    # A spline point for each point
    spline.points.add(len(coords_list) - 1)  # One point already exists by default

    # Assign the point coordinates to the spline points
    for p, new_co in zip(spline.points, coords_list):
        p.co = (new_co + [1.0])  # Add NURBS weight

    # Make a new object with the curve
    obj = bpy.data.objects.new(curve_name, crv)
    bpy.context.scene.collection.objects.link(obj)

    return obj  # Return the curve object for later use

# Replace with the path to your CSV file
csv_file_path = "C:/Users/super/OneDrive/Documents/Dirac/data.csv"

# Run the script
coords = load_csv(csv_file_path)
nurbs_curve_obj = create_nurbs_curve_from_coords(coords)

# Create Wire (Cylinder)
#--------------------------------------------------

# Create a new mesh and object
mesh = bpy.data.meshes.new("CylinderMesh")
obj = bpy.data.objects.new("CylinderObject", mesh)

# Link the object to the current collection
bpy.context.collection.objects.link(obj)

# Define vertices (line segment points) for the cylinder path
vertices = [
    (0, 0, 0),  # Start point
    (0, 0, 7),  # End point
]

# Define edges to connect the vertices
edges = [(0, 1)]

# Create the mesh
mesh.from_pydata(vertices, edges, [])

# Add the Skin Modifier to the object
skin_modifier = obj.modifiers.new(name="Skin", type='SKIN')

# Ensure vertices are tagged for skinning
bpy.ops.object.select_all(action='DESELECT')
bpy.context.view_layer.objects.active = obj
obj.select_set(True)
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.object.mode_set(mode='OBJECT')

# Scale the cylinder by adjusting the Skin Modifier with a smaller radius
for vertex in obj.data.vertices:
    obj.data.skin_vertices[0].data[vertex.index].radius = (0.07, 0.07)  # Set radius (X, Y)

# Add a Subdivision Surface Modifier for smoothness
subdiv = obj.modifiers.new(name="Subdivision", type='SUBSURF')
subdiv.levels = 2  # Viewport levels
subdiv.render_levels = 2  # Render levels

# Enable smooth shading
bpy.context.view_layer.objects.active = obj
bpy.ops.object.shade_smooth()

# Now let's apply the Curve Modifier to the cylinder to follow the NURBS curve
# Add the Curve modifier to the CylinderObject and set the curve as NURBS_Curve
curve_modifier = obj.modifiers.new(name="Curve", type='CURVE')
curve_modifier.object = nurbs_curve_obj  # The created NURBS curve object
curve_modifier.deform_axis = 'NEG_Z'  # Deform along the Z-axis

# Align the object to the curve so the modifier can work properly
# We need to set the cylinder to follow the curve's axis (X, Y, or Z)
# Set the cylinder's axis to align with the curve
obj.rotation_euler = (0, 0, 0)  # Reset rotation if needed
obj.location = nurbs_curve_obj.location  # Place the wire at the curve's start

# Ensure the object is scaled and rotated properly to match the curve
# Apply all transformations so the modifier works as expected
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

# Center the view on the object
bpy.ops.object.select_all(action='DESELECT')
obj.select_set(True)
bpy.context.view_layer.objects.active = obj
bpy.ops.view3d.view_selected(use_all_regions=False)