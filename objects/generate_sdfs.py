import csv
import os

# SDF file template
sdf_template = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{model_name}">
    <link name="{model_name}_link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx>
          <iyy>{iyy}</iyy>
          <izz>{izz}</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>
      <visual name="{model_name}_visual">
        <geometry>
          <mesh>
            <uri>{obj_filename}</uri>
            <scale>{scale} {scale} {scale}</scale>
          </mesh>
        </geometry>
        <material>
          <diffuse>0.8 0.2 0.2 1</diffuse>
          <ambient>0.3 0.1 0.1 1</ambient>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

def generate_sdfs():
    csv_file = 'objects.csv'
    
    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} not found. Please create the CSV file first.")
        return

    print("Generating 48 SDF files...")
    
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:

            # parse inertia string "ixx iyy izz"
            inertias = row['inertia'].split()
            ixx = inertias[0]
            iyy = inertias[1]
            izz = inertias[2]
            
            # prepare data
            data = {
                'model_name': row['shape'] + "_variant", # basic naming
                'mass': row['mass'],
                'ixx': ixx,
                'iyy': iyy,
                'izz': izz,
                'obj_filename': row['obj_filename'],
                'scale': str(float(row['size'])/10)
            }
            
            filename = row['sdf_file']
            content = sdf_template.format(**data)
            
            with open(filename, 'w') as out:
                out.write(content)
                
    print("Done!")

if __name__ == "__main__":
    generate_sdfs()
