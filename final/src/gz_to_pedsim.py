#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import math
import sys

def get_pose(node):
    """ Extracts x, y, yaw from a pose string 'x y z r p y' """
    if node is None or node.find('pose') is None:
        return 0.0, 0.0, 0.0
    
    data = node.find('pose').text.split()
    x, y = float(data[0]), float(data[1])
    # Yaw is usually the 6th element (index 5), but sometimes orientation is quaternion.
    # This parser assumes Euler 'x y z r p y' which is standard for Gazebo <pose> tags.
    yaw = float(data[5]) if len(data) > 5 else 0.0
    return x, y, yaw

def rotate_point(x, y, theta):
    """ Rotates a point (x,y) by theta radians """
    x_new = x * math.cos(theta) - y * math.sin(theta)
    y_new = x * math.sin(theta) + y * math.cos(theta)
    return x_new, y_new

def convert_world_to_xml(world_file, output_file):
    tree = ET.parse(world_file)
    root = tree.getroot()
    
    obstacles = []
    
    print(f"Reading {world_file}...")

    # Find the world element
    world = root.find('world')
    if world is None:
        print("Error: Could not find <world> tag.")
        return

    # Iterate through all models in the world
    for model in world.findall('model'):
        model_name = model.get('name')
        mx, my, myaw = get_pose(model)
        
        # Heuristic: We only want to convert structural elements like "Wall" or "office_env"
        # We skip small items like 'book', 'cup', 'pen' to keep the map clean.
        # You can remove this check to convert EVERYTHING.
        if "Wall" not in model_name and "office" not in model_name and "obstacle" not in model_name:
            continue

        print(f"Processing Model: {model_name}")

        for link in model.findall('link'):
            lx, ly, lyaw = get_pose(link)
            
            # Combine model pose and link pose (Simple 2D approximation)
            # Global Yaw = Model Yaw + Link Yaw
            final_yaw = myaw + lyaw
            
            # Global Position (Rotate link pos by model yaw, then add model pos)
            rx, ry = rotate_point(lx, ly, myaw)
            final_x = mx + rx
            final_y = my + ry

            # Find collision geometry box size
            collision = link.find('collision')
            if collision:
                box = collision.find('.//box/size')
                if box is not None:
                    sx, sy, sz = map(float, box.text.split())
                    
                    # Determine which axis is the "length" of the wall
                    # Walls are usually long and thin.
                    if sx > sy:
                        length = sx
                        # Wall runs along X axis
                        angle_offset = 0
                    else:
                        length = sy
                        # Wall runs along Y axis
                        angle_offset = 1.5708 # 90 degrees

                    total_angle = final_yaw + angle_offset
                    
                    # Calculate Start (x1, y1) and End (x2, y2) of the line
                    dx = (length / 2.0) * math.cos(total_angle)
                    dy = (length / 2.0) * math.sin(total_angle)

                    x1 = final_x - dx
                    y1 = final_y - dy
                    x2 = final_x + dx
                    y2 = final_y + dy

                    obstacles.append((x1, y1, x2, y2))

    # Generate Output XML
    print(f"Found {len(obstacles)} wall segments.")
    with open(output_file, 'w') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<scenario>\n')
        f.write('    \n')
        
        for obs in obstacles:
            f.write(f'    <obstacle x1="{obs[0]:.2f}" y1="{obs[1]:.2f}" x2="{obs[2]:.2f}" y2="{obs[3]:.2f}" type="line"/>\n')

        # Add dummy agent and waypoints so the file is valid and loadable immediately
        f.write('\n    \n')
        f.write('    <waypoint id="start" x="0" y="0" r="1"/>\n')
        f.write('    <waypoint id="end" x="5" y="5" r="1"/>\n')
        f.write('    <agent x="0" y="0" n="1" dx="0" dy="0" type="1">\n')
        f.write('        <addwaypoint id="start"/>\n')
        f.write('        <addwaypoint id="end"/>\n')
        f.write('    </agent>\n')
        
        f.write('</scenario>\n')
    
    print(f"Successfully created {output_file}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 gz_to_pedsim.py input.world [output.xml]")
    else:
        input_world = sys.argv[1]
        output_xml = sys.argv[2] if len(sys.argv) > 2 else "converted_scene.xml"
        convert_world_to_xml(input_world, output_xml)