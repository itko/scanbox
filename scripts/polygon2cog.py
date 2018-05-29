# -*- coding: utf-8 -*-


""" 
@author: Felice Serena

This script reads images annotated with polygones,
It reads camera calibration files,
It computes the 3d cog for each label per frame window
"""

import sys, os, csv, cv2, re
import numpy as np

# get help with reading and transformations
import validation as v

# from: https://github.com/Fluci/truthChecker/blob/master/src/truthChecker.py
import truthChecker as tc

def findFrameIndices(rex, grp, searchDir):
    """
    Takes a compiled regular expression and a search directory.
    Checks each entry of the directory, if a match is found, 
    the frame number is extracted according to the groupt name `grp`
    """
    files = os.listdir(searchDir)
    frame_indices = []
    for f in files:
        m = rex.match(f)
        if m is None:
            continue
        frame_indices.append(m.group(grp))
    return frame_indices

def computeNearestPoints(ray1, ray2):
    """
    Takes two rays g and h (origin, direction),
    and computes for each ray the closest point to the other ray
    returns [Fg, Fh], Fg: nearest point to h on g, Fh: nearest point to g on h
    """
    # https://de.wikipedia.org/wiki/Windschiefe#Bestimmung_der_Lotfu%C3%9Fpunkte
    a = ray1[0]
    b = ray2[0]
    v = ray1[1]
    w = ray2[1]
    n = np.cross(v, w)
    n1 = np.cross(v, n)
    n2 = np.cross(w, n)
    # nearest point to g on h
    Fh = (np.dot(a, n1) - np.dot(b, n1)) / np.dot(w, n1) * w + b

    # nearest point to h on g
    Fg = (np.dot(b, n2) - np.dot(a, n2)) / np.dot(v, n2) * v + a

    return [Fg, Fh]


def triangulate(framePts, frameC2W, params):
    """
    Takes a list of points `framePts` (one point per camera),
    a list of camera to world transforms,
    and a list of camera parameters (focallengths baselines ccx ccy)
    It then returns one point in 3d space such that the camera rays
    through framePts pass as close as possible
    """
    rays = []
    # build the rays through the camera center and the framePt
    for f in range(len(framePts)):
        focallength = float(params[f][0])
        ccx = float(params[f][2])
        ccy = float(params[f][3])

        p2 = framePts[f]
        p2h = focallength * np.array([p2[0] + v.X_SHIFT - ccx, p2[1] + v.Y_SHIFT - ccy, 1.0])
        p3 = np.array([p2[0], p2[1], 1.0, 1])
        o = np.array([0,0,0,1])
        c2w = frameC2W[f]
        p3 = np.matmul(c2w, p3)
        o = np.matmul(c2w, o)
        direction = p3 - o
        rays.append((o[0:3], direction[0:3]))
    rn = len(rays)
    # find pairwise nearest points
    points = []
    for i in range(rn):
        for j in range(i+1,rn):
            ps = computeNearestPoints(rays[i], rays[j])
            points.extend(ps)
    matrix = np.array(points)
    center = np.sum(matrix, axis=0)
    return center

if __name__ == "__main__":

    N_STREAMS = 4
    #Parse Arguments
    
    if len(sys.argv) >= 4:
        annotation_dir = os.path.abspath(sys.argv[1])
        params_dir = os.path.abspath(sys.argv[2])
        out_dir = os.path.abspath(sys.argv[3])
    
    if len(sys.argv) < 4 or not os.path.isdir(annotation_dir) or not os.path.isdir(params_dir):

        print('USAGE: \n\
                Argument 1: directory of annotated images, should hold a subfolder "annotations"\n\
                Argument 2: directory of params*.csv files\n\
                Argument 3: directory for output')
        sys.exit(1)
   
    annotationsFolder = os.path.join(annotation_dir, 'annotations')
    if not os.path.isdir(annotationsFolder):
        print('Can\'t find "annotations" subfolder in ' + str(annotation_dir))
        sys.exit(1)
    
    camchain_path = os.path.join(params_dir,'params_camchain.csv')
    if os.path.isfile(camchain_path):
        camchain = v.read_matrix_chain(camchain_path)
    else:
        print('ERROR: params_camchain.csv not found. Terminating...')
        sys.exit(1)
    
    R_path = os.path.join(params_dir,'params_R.csv')
    if os.path.isfile(R_path):
        R = v.read_matrix_chain(R_path)
    else:
        print('ERROR: params_R.csv not found. Terminating...')
        sys.exit(1)
     
    if not os.path.isdir(out_dir):
        os.mkdir(out_dir)


    frame_indices = findFrameIndices(re.compile("pic_s_(?P<streamIndex>\d+)_f_(?P<frameIndex>\d+)\.png"), 'frameIndex', annotation_dir)
    for frame in frame_indices:
        param_file = os.path.join(params_dir,"params_f_" + str(frame) + ".csv")
        if not os.path.isfile(param_file):
            print("Parameter .csv not found - skipping frame " + str(frame))
            continue
        params = v.read_params(param_file)
        # create world to camera 4x4 transform
        w2c = [None] * N_STREAMS
        c2w = [None] * N_STREAMS
        for stream in range(N_STREAMS):
            w2c[stream] = projection_matrix = v.make_projection_matrix(camchain,R,stream);
            c2w[stream] = np.linalg.inv(w2c[stream])
        pics = [None] * N_STREAMS
        annotationsPath = [None] * N_STREAMS
        annotations = [None] * N_STREAMS
        label2Annotation = [None] * N_STREAMS
        labels = set()
        controlPoints = []
        for stream in range(N_STREAMS):
            png_file = "pic_s_" + str(stream+1) + "_f_" + str(frame) + ".png";
            png_path = os.path.join(annotation_dir, png_file)
            if not os.path.isfile(png_path):
                print("PNG for frame " + str(frame) + " stream " + str(stream) + " not found. Skipping.")
                continue
            pics[stream] = png_path
            annP = tc.findMatchForImage(annotationsFolder, png_file)
            annotationsPath[stream] = annP
            if annP is None:
                #print("No annotation found for frame " + str(frame) + ", stream " + str(stream) + " in " + str(annotationsFolder) + " for file " + png_file)
                pass
            ann = tc.readAnnotations(annP)
            annotations[stream] = ann;
            if ann == None:
                ann = []
                
            l2a = {}
            for a in ann:
                l2a[a.name] = a
                labels.add(a.name)
            label2Annotation[stream] = l2a
        
        for l in labels:
            appearsIn = []
            for s in range(N_STREAMS):
                if l in label2Annotation[s]:
                    appearsIn.append(s)
            if len(appearsIn) < 2:
                print("Label " + str(l) + " does not appear in enough frames for triangulation, skipping frame " + str(frame) + " ...")
                continue
            cogs = []
            ts = []
            lParams = []
            for s in appearsIn:
                cogs.append(label2Annotation[s][l].shape.centroid())
                ts.append(c2w[s])
                lParams.append(params[stream])
            centerPoint = triangulate(cogs, ts, lParams)
            controlPoints.append(centerPoint)
    # write found 3d points to csv
    csv_path = os.path.join(out_dir, 'controlPoints_' + str(frame) + '.csv')
    with open(csv_path, 'rb') as csv_file:
        csv_writer = csv.writer(csv_file)
        for c in controlPoints:
            csv_writer.writerow(c)

