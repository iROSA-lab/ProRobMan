import bpy

# define materials
def define_material(id, r, g, b):
    mat= bpy.data.materials.get(id)
    if mat is None:
        mat=bpy.data.materials.new(name=id)
        mat.use_nodes=True
        if mat.node_tree:
            mat.node_tree.links.clear()
            mat.node_tree.nodes.clear()
    nodes=mat.node_tree.nodes
    output = nodes.new(type='ShaderNodeOutputMaterial')
    shader = nodes.new(type='ShaderNodeBsdfGlossy')
    nodes["Glossy BSDF"].inputs[0].default_value = (r, g, b, 1)
    nodes["Glossy BSDF"].inputs[1].default_value = 0
    mat.node_tree.links.new(shader.outputs[0], output.inputs[0])
    return mat

def clear_the_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    
def project_to_cube(axis, target):
    bpy.ops.object.modifier_add(type='SHRINKWRAP')
    bpy.context.object.modifiers["Shrinkwrap"].wrap_method = 'PROJECT'
    if axis==2:
        bpy.context.object.modifiers["Shrinkwrap"].use_project_z = True
    if axis==1:
        bpy.context.object.modifiers["Shrinkwrap"].use_project_y = True
    if axis==0:
        bpy.context.object.modifiers["Shrinkwrap"].use_project_x = True
    bpy.context.object.modifiers["Shrinkwrap"].target = bpy.data.objects[target]
    bpy.context.object.modifiers["Shrinkwrap"].use_negative_direction = True
    bpy.context.object.modifiers["Shrinkwrap"].use_positive_direction = True
    bpy.context.object.modifiers["Shrinkwrap"].wrap_mode = 'OUTSIDE_SURFACE'

def update_location(location, projection_axis):
    location=list(location)
    if location[projection_axis]>0:
        location[projection_axis]+=0.0001
    else:
        location[projection_axis]-=0.0001
    location=tuple(location)
    return location

wood=define_material('Wood', 0.65, 0.45, 0.28)
def paint_surface(id, color, location, rotation, projection_axis):
    bpy.ops.mesh.primitive_plane_add(size=0.045, location=location, rotation=rotation)
    bpy.context.active_object.data.materials.append(color)
    bpy.context.active_object.name=f'plane_{id}'
    location=update_location(location, projection_axis)
#    project_to_cube(projection_axis, 'cube')
    bpy.ops.mesh.primitive_circle_add(radius=0.02, fill_type='TRIFAN', location=location, rotation=rotation)
    bpy.context.active_object.data.materials.append(wood)
    location=update_location(location, projection_axis)
#    project_to_cube(projection_axis, f'plane_{id}')
    bpy.ops.object.text_add(radius=0.038, location=location, rotation=rotation)
    bpy.context.active_object.data.materials.append(color)
    bpy.context.active_object.name=f'Text{id}'

def update_text(id, letter):
    bpy.ops.object.editmode_toggle()
    bpy.data.objects[f'Text{id}'].data.name=f'Text{id}'
    bpy.data.objects[f'Text{id}'].data.body = letter
    bpy.data.objects[f'Text{id}'].data.align_x = 'CENTER'
    bpy.data.objects[f'Text{id}'].data.align_y = 'CENTER'
    bpy.ops.object.editmode_toggle()

def generate_cube():
    bpy.ops.mesh.primitive_cube_add(size=0.045, location=(0, 0, 0), rotation=(0.0, 0, 0))
    bpy.context.active_object.name='cube'
    # paint the surfaces
    red=define_material('Red', 1, 0, 0)
    paint_surface(0, red, location=(0, 0, 0.0226), rotation=(0.0, 0, 0), projection_axis=2)
    paint_surface(1, red, location=(0, 0, -0.0226), rotation=(0.0, 0, 0), projection_axis=2)
    blue=define_material('Blue', 0, 1, 0)
    paint_surface(2, blue, location=(0, 0.0226, 0), rotation=(1.5708, 0, 0), projection_axis=1)
    paint_surface(3, blue, location=(0, -0.0226, 0), rotation=(1.5708, 0, 0), projection_axis=1)
    green=define_material('Green', 0, 0, 1)
    paint_surface(4, green, location=(0.0226, 0, 0), rotation=( 0, 1.5708, 0), projection_axis=0)
    paint_surface(5, green, location=(-0.0226, 0, 0), rotation=( 0, 1.5708, 0), projection_axis=0)

import numpy as np
upper_case=np.array(['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'])
for i in range(28):
    letters=np.random.choice(upper_case, 6)
    clear_the_scene()
    generate_cube()
    for i in range(6):
        update_text(i,letters[i])
    bpy.data.objects['Text5']