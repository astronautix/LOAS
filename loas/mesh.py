import trimesh

class Meshou(trimesh.Trimesh):

    def yay(self):
        return self.area


a = trimesh.load_mesh("../examples/bunny.stl")
a.__class__ = Meshou
