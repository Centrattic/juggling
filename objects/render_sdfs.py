#!/usr/bin/env python3
"""
Script to render SDF files and save images of the 3D objects.
"""

import os
import xml.etree.ElementTree as ET
import trimesh
import numpy as np
from pathlib import Path

def parse_sdf(sdf_path):
    """Parse SDF file to extract OBJ filename, scale, and material properties."""
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    
    mesh_elem = root.find('.//mesh')
    if mesh_elem is None:
        return None
    
    obj_filename = mesh_elem.find('uri').text
    scale_elem = mesh_elem.find('scale')
    if scale_elem is not None:
        scale_str = scale_elem.text.split()
        scale = float(scale_str[0])/10.0
    else:
        scale = 1.0
    
    # Find material properties
    material_elem = root.find('.//material')
    diffuse = [0.8, 0.2, 0.2, 1.0]  # Default
    if material_elem is not None:
        diffuse_elem = material_elem.find('diffuse')
        if diffuse_elem is not None:
            diffuse = [float(x) for x in diffuse_elem.text.split()]
    
    return {
        'obj_filename': obj_filename,
        'scale': scale,
        'color': diffuse[:3]  # RGB only
    }

def render_object(obj_path, scale, color, output_path, resolution=(800, 600)):
    """Render an OBJ file and save as image."""

    mesh = trimesh.load(obj_path)
    
    if scale != 1.0:
        mesh.apply_scale(scale)
    
    # Set color - need to create arrays with proper shape
    color_rgba = np.array([int(c * 255) for c in color] + [255], dtype=np.uint8)
    
    # Try to set face colors (most common)
    if hasattr(mesh.visual, 'face_colors'):
        num_faces = len(mesh.faces)
        mesh.visual.face_colors = np.tile(color_rgba, (num_faces, 1))
    elif hasattr(mesh.visual, 'vertex_colors'):
        num_vertices = len(mesh.vertices)
        mesh.visual.vertex_colors = np.tile(color_rgba, (num_vertices, 1))
    else:
        # Create a simple material color
        mesh.visual.material = trimesh.visual.material.SimpleMaterial(
            diffuse=color + [1.0]
        )
    
    scene = trimesh.Scene([mesh])
    
    bounds = scene.bounds
    center = bounds.mean(axis=0)
    scene.camera_transform = scene.camera.look_at(
        points=[center],
        rotation=np.eye(3)
    )
    
    size = bounds.ptp().max()
    distance = size * 2.5
    scene.camera_transform[:3, 3] = center + np.array([distance, distance * 0.5, distance * 0.8])
    
    png = scene.save_image(resolution=resolution)
    
    with open(output_path, 'wb') as f:
        f.write(png)

def main():
    script_dir = Path(__file__).parent
    objects_dir = script_dir
    
    images_dir = objects_dir / 'images'
    images_dir.mkdir(exist_ok=True)
    
    sdf_files = sorted(objects_dir.glob('*.sdf'))
    
    if not sdf_files:
        print("No SDF files found in the objects directory.")
        return
    
    print(f"Found {len(sdf_files)} SDF files. Rendering...")
    
    for sdf_path in sdf_files:
        print(f"Processing {sdf_path.name}...")
        
        sdf_data = parse_sdf(sdf_path)
        if sdf_data is None:
            print(f"  Warning: Could not parse {sdf_path.name}, skipping.")
            continue
        
        obj_path = objects_dir / sdf_data['obj_filename']
        if not obj_path.exists():
            print(f"  Warning: OBJ file {obj_path} not found, skipping.")
            continue
        
        image_name = sdf_path.stem + '.png'
        output_path = images_dir / image_name
        
        try:
            render_object(
                str(obj_path),
                sdf_data['scale'],
                sdf_data['color'],
                str(output_path)
            )
            print(f"  Saved: {output_path}")
        except Exception as e:
            print(f"  Error rendering {sdf_path.name}: {e}")
    
    print(f"\nDone! Images saved to {images_dir}")

if __name__ == "__main__":
    main()

