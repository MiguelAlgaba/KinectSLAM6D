#!/usr/bin/python

import argparse
import sys
import os
from associate import *
from evaluate import *
from generate_pointcloud import *
from PIL import Image, ImageDraw

focalLength = 525.0
centerX = 319.5
centerY = 239.5

def point(pose,px,py,pz):
    p = pose.dot(numpy.matrix([[px],[py],[pz],[1]]))
    X = p[0,0]
    Y = p[1,0]
    Z = p[2,0]
    u = X/Z * focalLength + centerX
    v = Y/Z * focalLength + centerY
    return [u,v]
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    This script plots a trajectory into an image sequence. 
    ''')
    parser.add_argument('image_list', help='input image list (format: timestamp filename)')
    parser.add_argument('trajectory_file', help='input trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('out_image', help='file name of the result (format: png)')
    args = parser.parse_args()
    
    image_list = read_file_list(args.image_list)
    pose_list = read_file_list(args.trajectory_file)
    traj = read_trajectory(args.trajectory_file)

    matches = associate(image_list, pose_list,0,0.02)

    stamps = image_list.keys()
    stamps.sort()
    
    matches_dict = dict(matches)
    for stamp in stamps:
        image_file = image_list[stamp][0]
        image = Image.open(image_file)
        print "image stamp: %f"%stamp
        
        if stamp in matches_dict: 
            print "pose stamp: %f"%matches_dict[stamp]
            pose = traj[matches_dict[stamp]]
            
            stamps = traj.keys()
            stamps.sort()
        
            xy = []    
            draw = ImageDraw.Draw(image)
            size = 0.01
            
            for s in stamps:
                p = traj[s]
                rel_pose = numpy.dot(numpy.linalg.inv(pose),p)
                if rel_pose[2,3]<0.01: continue
                u,v = point(rel_pose,0,0,0)
                if u<0 or v<0 or u>640 or v>480: continue
                draw.line(point(rel_pose,0,0,0) + point(rel_pose,size,0,0), fill="#ff0000")
                draw.line(point(rel_pose,0,0,0) + point(rel_pose,0,size,0), fill="#00ff00")
                draw.line(point(rel_pose,0,0,0) + point(rel_pose,0,0,size), fill="#0000ff")
            del draw
            
        image.save(os.path.splitext(args.out_image)[0]+"-%f.png"%stamp)
    
    