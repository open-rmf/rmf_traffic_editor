from collections.abc import Iterable
import sys
import os

from .vertex import Vertex
from .edge import Edge
from .obstacle import Obstacle
from .object_manager import Manager


class FileWriter:
    def __init__(self, filename):
        self.filename = filename

    def open_file(self, mode="w+"):
        self.filehandle = open(self.filename, mode)

    def write_line(self, content):
        if(self.filehandle):
            if isinstance(content, Iterable):
                for item in content:
                    item_str = str(item) + " "
                    self.filehandle.write(item_str)
            else:
                self.filehandle.write(str(content))
            self.filehandle.write("\n")
            self.filehandle.flush()

    def close_file(self):
        self.filehandle.close()

    def set_data_manager(
            self,
            polygon_vertex_manager,
            edge_manager,
            obstacle_manager,
            polygon_manager):
        self.vertex_manager = polygon_vertex_manager
        self.edge_manager = edge_manager
        self.obstacle_manager = obstacle_manager
        self.polygon_manager = polygon_manager

    def generate_navmesh(self):
        # vertex part
        self.write_line(len(self.vertex_manager.data))
        for i in range(len(self.vertex_manager.data)):
            v = self.vertex_manager.data[i]
            self.write_line(v.get_coords())
        self.write_line(" ")

        # edge part
        self.write_line(len(self.edge_manager.data))
        for i in range(len(self.edge_manager.data)):
            e = self.edge_manager.data[i]
            self.write_line(e.get_variable())
        self.write_line(" ")

        # obstacle part
        self.write_line(len(self.obstacle_manager.data))
        for i in range(len(self.obstacle_manager.data)):
            o = self.obstacle_manager.data[i]
            self.write_line(o.get_variable())
        self.write_line(" ")

        # polygon part
        # nodes name
        self.filehandle.write("walkable")
        self.filehandle.write("\n")
        self.write_line(len(self.polygon_manager.data))
        self.write_line(" ")
        for i in range(len(self.polygon_manager.data)):
            polygon = self.polygon_manager.data[i]
            result = polygon.get_variable()
            # center
            self.write_line(result[0])
            # vertices
            tmp = list(result[1])
            tmp.insert(0, len(result[1]))
            self.write_line(tmp)
            # gradient
            self.write_line(result[2])
            # edges
            if(len(result[3]) > 0):
                tmp = list(result[3])
                tmp.insert(0, len(result[3]))
                self.write_line(tmp)
            else:
                self.write_line(0)
            # obstacle
            if(len(result[4]) > 0):
                tmp = list(result[4])
                tmp.insert(0, len(result[4]))
                self.write_line(tmp)
            else:
                self.write_line(0)
            # blank space
            self.write_line(" ")

        if self.filename[0] == '/':
            print("Generate: ", self.filename)
        else:
            print("Generate: ", os.getcwd() + '/' + self.filename)
