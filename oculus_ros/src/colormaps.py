import numpy as np
import os
import re



class ColorMap:
    def __init__(self, filename=r'/home/catkin_ws/src/bluePrintOculusPython/oculus_ros/src/ColorMaps.cpp'):
        self.data = self.parse_cpp_file(filename)

    # # Simple class to mimic the color map functionality in the reference.
    # _inferno_data_uint8 = np.array([
    #     [0, 0, 3],       
    #     [0, 0, 4],     
    #     [0, 0, 6],     
    #     [1, 0, 7],
    #     [1, 1, 9], 
    #     # Add more colors as required...
    # ])

    def lookup(self, intensity):
        # Lookup the color based on the intensity.
        return self.data['_inferno_data_uint8'][intensity]
        # return self.data['_inferno_data_float'][intensity]
    

    def parse_cpp_file(self, filename):
        # Regular expression to pick numbers within braces
        regex_pattern = r'{([^}]*)}'

        with open(filename) as f:
            lines = f.read().splitlines()

        arrays = {}
        current_array = ""
        for line in lines:
            if "_data_" in line:
                current_array = line.split("::")[1].split("[")[0]
                arrays[current_array] = []
            elif "{" in line and "}" in line:
                matches = re.findall(regex_pattern, line)
                for match in matches:
                    arrays[current_array].append([float(d) for d in match.split(",")])

        # convert to numpy arrays
        for k, v in arrays.items():
            arrays[k] = np.array(v)

        return arrays

def main():
    colormapper = ColorMap(filename='ColorMaps.cpp')
    # data = colormapper.parse_cpp_file(filename)
    _inferno_data_uint8 = colormapper.data["_inferno_data_uint8"]
    print(_inferno_data_uint8)

if __name__ == "__main__":
    main()
