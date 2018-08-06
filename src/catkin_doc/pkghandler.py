
import os
import magic

class PkgHandler:
    def __init__(self):
        pass

    def search_for_python(self, pkg_name):
        for filename in os.listdir(pkg_name):
            if os.path.isdir(pkg_name + "/" + filename):
                self.search_for_python(pkg_name + "/" + filename)
            elif os.path.isfile(pkg_name + "/" + filename):
              filetype = magic.from_file(pkg_name + "/" + filename)
              if ("python" in filetype) | ("Python" in filetype):
                  self.check_if_ros_node(pkg_name + "/" + filename)

    def check_if_ros_node(self, filename):
        file = open(filename, "r")
        if "rospy.init_node" in file.read():
            print(filename)




