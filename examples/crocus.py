import subprocess
import os
import trimesh
def get_mesh(angle): #get the mesh of crocus with certain angle repective to the satellite
    with open("tmp.scad", "w") as f:
        f.write("""angle = {};
union() {{
    translate([-5,-5,0]){{
        cube([10,10,20]);
    }}
    for (i = [1:2]) {{
        rotate([0,0,180*i]) translate([-4.5,0,0]) rotate([0,angle,0]) translate([0,0,-10]){{
                cube([1,10,20], true);
        }}
    }}
}}""".format(angle))
    subprocess.call(['openscad', '-o', 'tmp.stl', 'tmp.scad'])
    mesh = trimesh.load_mesh("tmp.stl")
    os.remove('tmp.scad')
    os.remove('tmp.stl')
    return mesh
