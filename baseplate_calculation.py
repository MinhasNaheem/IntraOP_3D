import numpy as np
import configparser
target=[-174.687,56.155,-14.338]
baseplate_fiducials=np.array(([0.000,0.000,0.000],[27.300,99.011,0.059],[-65.739,114.124,0.097],[-65.851,-5.855,0.331]))
fiducial_coordinates=[]
for fid in range(len(baseplate_fiducials)):
    fid_coord=target-baseplate_fiducials[fid]
    fiducial_coordinates.append(fid_coord)
fiducial_coordinates=np.array(fiducial_coordinates)
# columns_to_negate = [0, 1]
fiducial_coordinates = (np.vstack((-fiducial_coordinates[:, 0], -fiducial_coordinates[:, 1], fiducial_coordinates[:, 2]))).T
print(fiducial_coordinates)
config=configparser.ConfigParser()
# Add fiducial sections
for i, coord in enumerate(fiducial_coordinates):
    config.add_section(f"fiducial{i}")
    config.set(f"fiducial{i}", "x", str(round(coord[0],3)))
    config.set(f"fiducial{i}", "y", str(round(coord[1],3)))
    config.set(f"fiducial{i}", "z", str(round(coord[2],3)))

# Add [geometry] section
config.add_section("geometry")
config.set("geometry", "count", str(len(fiducial_coordinates)))
config.set("geometry", "id", "51001")

# Save to INI file
with open("D:\IntraOP_3D\GeometryFiles\geometry91001.ini", "w") as configfile:
    config.write(configfile)
