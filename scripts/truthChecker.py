import os
import numpy as np

def findMatchForImage(annotationsFolder, imgName):
    """
    Tries to find the matching annotation file.
    Returns either the path to the annotation file or None
    """
    parts = imgName.rsplit('.', 1)
    base = parts[0]
    ext = parts[1]
    if len(base) is 0:
        # skip, hidden file
        return None
    xml = os.path.join(annotationsFolder, base + ".xml")
    if not os.path.isfile(xml):
       return None 
    return xml

def findDataPairs(folder, annotationFolder):
    """
    Searches an image directory "folder" and an annotation directory "annotationFolder".
    It then looks for each image, if there is a matching annotation (according to name).
    If there is, it creates a dictionary `{'img', 'xml'}` with the values set to the paths of the files
    """
    dataPairs = []
    
    if not os.path.isdir(annotationFolder):
        print("No annotations folder found at " + annotationFolder)
        return dataPairs
    imgs = os.listdir(folder)
    for img in imgs:
        imgPath = os.path.join(folder, img)
        if not os.path.isfile(imgPath):
            continue
        xml = findMatchForImage(annotationFolder, img)
        if xml == None:
            print("No annotation xml found for file " + imgPath)
            continue

        dataPairs.append({'img': imgPath, 'xml': xml})
    
    return dataPairs

def searchFolders(folders, annotationSubFolder):
    """
    Like `findDataPairs`, it searches folders and their subfolders for matches 
    between images and annotations. The difference is, it takes a list of absolute paths (`folders`),
    and a relative directory path `annotationSubFolder` such that it looks for the annotations relative to the
    main folders. 
    It then returns a merged list of all found pairs.
    """
    truthDirs = []
    
    for f in folders:
        f = os.path.abspath(f)
        if os.path.isdir(f):
            truthDirs.append(f)
        else:
            print("Directory " + str(f) + " not found, skipping...")
    
    print("found directories:")
    for t in truthDirs:
        print(t)
    
    dataPairs = []
    
    for t in truthDirs:
        annotationFolder = os.path.join(t, annotationSubFolder)
        newPairs = findDataPairs(t, annotationFolder)
        dataPairs = dataPairs + newPairs
    
    return dataPairs

class BoundingBox:
    def __init__(self, xmin, ymin, xmax, ymax):
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
    def drawTo(self, pic, lineColor, lineWidth, centerRadius=2):
        minS = (int(self.xmin), int(self.ymin))
        maxS = (int(self.xmax), int(self.ymax))
        cv2.rectangle(pic,maxS,minS, lineColor, lineWidth)
        C = self.centroid()
        C = (int(C[0]), int(C[1]))
        cv2.circle(pic, C, centerRadius, lineColor, lineWidth)
    def centroid(self):
        """ The arithmetic mean position of all points in the shape """
        return ((self.xmax + self.xmin)/2.0, (self.ymax + self.ymin)/2.0)

class Polygon:
    def __init__(self, xlist, ylist):
        n = len(xlist)
        # just to make math simple down the road, add the first point as last
        xlist.append(xlist[0])
        ylist.append(ylist[0])
        self.xs = np.array(xlist)
        self.ys = np.array(ylist)
        assert(len(xlist) == len(ylist))
    def drawTo(self, pic, lineColor, lineWidth, centerRadius=2):
        n = len(self.xs)-1
        for i in range(n):
            xstart = int(self.xs[i])
            ystart = int(self.ys[i])
            xend = int(self.xs[i+1])
            yend = int(self.ys[i+1])
            cv2.line(pic, (xstart, ystart), (xend, yend), lineColor, lineWidth)
        C = self.centroid()
        C = (int(C[0]), int(C[1]))
        print(C)
        cv2.circle(pic, C, centerRadius, lineColor, lineWidth)
    
    def centroid(self):
        """ The arithmetic mean position of all points in the shape """
        # https://en.wikipedia.org/wiki/Centroid#Centroid_of_a_polygon 
        n = len(self.xs) - 1
        xs = self.xs
        ys = self.ys
        # A = 1/2 sum_{i = 0...n-1} ( x_i y_{i+1} - x_{i+1} y_i)
        # COMMON_i = (x_i * y_{i+1} - x_{i+1} y_i)
        L = np.multiply(xs[:-1],ys[1:])
        R = np.multiply(xs[1:], ys[:-1])
        COMMON = L - R 
        # A = 1/2 sum_{i = 0...n-1] COMMON_i
        A = 1.0/2.0 * np.sum(COMMON)
        # Cx = 1 / (6A) * sum_{i = 0...n-1} (x_i + x_{i+1})(x_i * y_{i+1} - x_{i+1} y_i)
        # Cy = 1 / (6A) * sum_{i = 0...n-1} (y_i + y_{i+1})(x_i * y_{i+1} - x_{i+1} y_i)
        # Cx = 1 / (6A) * sum_{i = 0...n-1} (x_i + x_{i+1}) * COMMON_i
        Cx = 1.0 / (6.0*A) * np.dot(xs[:-1] + xs[1:], COMMON)
        # Cy = 1 / (6A) * sum_{i = 0...n-1} (y_i + y_{i+1}) * COMMON_i
        Cy = 1.0 / (6.0*A) * np.dot(ys[:-1] + ys[1:], COMMON)
        # naive approach:
        # CCx = np.sum(xs[:-1])/n
        # CCy = np.sum(ys[:-1])/n
        return (Cx, Cy)

class Annotation:
    def __init__(self, name, boundingShape):
        self.name = name
        self.shape = boundingShape

import xml.etree.ElementTree as ET
def readAnnotations(xml):
    if xml is None:
        return None
    xml = os.path.abspath(xml)
    if not os.path.isfile(xml):
        print("given annotation path is not a file: " + xml)
        return []
    tree = ET.parse(xml)
    root = tree.getroot()
    annotations = []
    for obj in root.iter('object'):
        bb = obj.find('bndbox')
        polygon = obj.find('polygon')
        if bb is not None:
            xmin = bb.find('xmin').text
            ymin = bb.find('ymin').text
            xmax = bb.find('xmax').text
            ymax = bb.find('ymax').text
            boundingShape = BoundingBox(float(xmin), float(ymin), float(xmax), float(ymax))
        elif polygon is not None:
            xlist = []
            ylist = []
            for coord in polygon:
                if coord.tag[0] is "x":
                    xlist.append(float(coord.text))
                elif coord.tag[0] is "y":
                    ylist.append(float(coord.text))
            boundingShape = Polygon(xlist, ylist) 
        ann = Annotation(obj.find('name').text, boundingShape)
        annotations.append(ann)
    return annotations
    def __str__(self):
        return "xmin: " + str(self.xmin) + ", xmax: " + str(self.xmax) + ", ymin: " + str(self.ymin) + ", ymax: " + str(self.ymax)

def labelsToData(dataPairs):
    annotations = []
    labels = set()
    for p in dataPairs:
        anns = readAnnotations(p['xml'])
        annotations.append(anns);
        for a in anns:
            labels.add(a.name)
    
    labelToData = {}
    for l in labels:
        labelToData[l] = []
    for i in range(0, len(dataPairs)):
        for a in annotations[i]:
            data = {}
            data['i'] = i
            data['img'] = dataPairs[i]['img']
            data['xml'] = dataPairs[i]['xml']
            data['annotation'] = a
            labelToData[a.name].append(data)
    return labelToData


if __name__ == "__main__":
    import numpy as np
    import cv2, sys
    
    print("The First argument should be the path to your ground truth folder.")
    print("<-/->: Use forward/backward arrows to navigate.")
    print("Escape: Hit escape to skip walking through a label.")
    print("Q: quit application immediately")
    
    # where-ever your ground truth folder is
    folders = sys.argv[1:]
    
    annotationSubFolder = "annotations"
    
    dataPairs = searchFolders(folders, annotationSubFolder) 
    
    print("Found " + str(len(dataPairs)) + " data pairs")
    bbColor = (255,0,0)
    bbWidth = 2
    textColor = (200, 200, 200)
    textSize = 0.8
    labelToData = labelsToData(dataPairs)
    for label in labelToData.keys():
        i = 0
        while i < len(labelToData[label]):
            e = labelToData[label][i]
            imgPath = e['img']
            print(imgPath)
            a = e['annotation']
            pic = cv2.imread(imgPath,cv2.IMREAD_COLOR)
            a.shape.drawTo(pic, bbColor, bbWidth);
            ident = label + ": ..." + str(imgPath[-50:])
            height, width, channels = pic.shape
            cv2.putText(pic, ident, (0,20), cv2.FONT_HERSHEY_COMPLEX_SMALL, textSize, textColor)
            cv2.imshow("img", pic)
            key = cv2.waitKey(0)
            if key == 27:
                break
            elif key == 113:
                exit()
            elif key == 2 or key == 65361: # back arrow
                i = max(i - 1, 0)
            elif key == 3 or key == 65363: # forward arrow
                i = i + 1
            else:
                i = i + 1
  
    

