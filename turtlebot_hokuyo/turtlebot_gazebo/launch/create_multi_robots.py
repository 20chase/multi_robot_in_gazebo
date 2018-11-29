import argparse

import numpy as np

from lxml import etree
from lxml.etree import Element


class Creator(object):
    def __init__(self, file_name, robot_num, env_size=5.0):
        parser = etree.XMLParser(remove_blank_text=True)
        self.tree = etree.parse(file_name, parser)
        self.robot_num = robot_num
        self.env_size = env_size

    def run(self):
        root = self.tree.getroot()
        
        for i in range(self.robot_num):
            angle = i * 2 * np.pi / self.robot_num
            sx = self.env_size * np.cos(angle)
            sy = self.env_size * np.sin(angle)
            group = Element("group", ns="robot_"+str(i))
            param = Element("param", name="tf_prefix", value="robot_"+str(i))
            include = Element("include", file="$(find turtlebot_gazebo)/launch/includes/kobuki.launch.xml")
            robot_name = Element("arg", name="robot_name", value="robot_"+str(i))
            init_pose = Element("arg", name="init_pose", value="-x {} -y {} -z 0".format(sx, sy))
            include.append(robot_name)
            include.append(init_pose)
            group.append(param)
            group.append(include)
            root.append(group)

        self.tree.write("test.launch", pretty_print=True, xml_declaration=True, encoding="utf-8")
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Create multi-robot launch file for Gazebo")
    parser.add_argument(
        "--launch_file", default="empty.launch", type=str
    )
    parser.add_argument(
        "--robot_num", default=2, type=int
    )
    parser.add_argument(
        "--env_size", default=5.0, type=float
    )

    args = parser.parse_args()

    creator = Creator(args.launch_file, args.robot_num, args.env_size)
    creator.run()
    
