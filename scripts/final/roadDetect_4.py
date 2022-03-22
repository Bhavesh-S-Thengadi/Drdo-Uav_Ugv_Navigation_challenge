# IITG InterIIT 2022
"""The script requires the following packages:
  - numpy
  - opencv-python
  - scikit-image
"""

import os
from typing import Tuple
import numpy as np
import cv2 as cv
from skimage.filters import threshold_multiotsu
import math


class RoadImageProcessor:
    """RoadImageProcessor class. 
    @method loadImages:     Load the given numpy matrices as depth and rgb images. Removes old loaded images if any.
    @method readImages:     Same as loadImages but fetches images from files.
    @method getImages:      Return rgb, greyscale and depth images currently stored in the instance.
    @method depthGradient:  Find the absolute gradient image of the currently loaded depth image. 
    Useful for detecting flat areas like roads.
    @method greyLaplacian   Find the laplacian of the currently loaded greyscale image. Useful for edge detection.
    @method depthMultiOtsu: Run the Multi-Otsu algorithm on the currently loaded depth image.
    @method saveImgs:       Save an array of images in a given directory.
    """
    def __init__(self, pt1, pt2, box) -> None:
        self.rgb   = None
        self.depth = None
        self.grey  = None

        self.depthGrad =  None
        self.depthOtsu =  None
        self.greylap   =  None

        # self.pt1 = np.array((320,200), dtype=np.int32)
        # self.pt2 = np.array((320,280), dtype=np.int32)
        # self.box = np.ones((4,2), dtype=np.int32)
        # self.box *= 320
        self.pt1 = pt1
        self.pt2 = pt2
        self.box = box
        pass

    def loadImages(self, imgRGB: np.ndarray, imgDepth: np.ndarray) -> None:
        """Load RGB and Depth images given ndarrays"""
        self.rgb = imgRGB.copy()
        self.depth = imgDepth.copy()
        self.grey = cv.cvtColor(imgRGB, cv.COLOR_BGR2GRAY)
        self.loadedNew = True
        self.depthGrad =  None
        self.depthOtsu =  None
        self.greylap   =  None

    
    def readImages(self, rgbPath: str, depthPath: str, debug=False) -> None:
        """Load images given filepaths"""
        if debug:
            print("Fetching:\n{}\n{}".format(rgbPath, depthPath))
        rgb = np.array(cv.imread(rgbPath, 1), dtype=np.float64)/255
        grey = np.array(cv.imread(rgbPath, 0), dtype=np.float64)/255
        depth = np.array(cv.imread(depthPath, 0), dtype=np.float64)/255
        self.rgb = rgb
        self.depth = depth
        self.grey = grey
        self.depthGrad =  None
        self.depthOtsu =  None
        self.greylap   =  None


    def getImages(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self.rgb, self.grey, self.depth

    def normalise(self, img: np.ndarray) -> np.ndarray:
        """Normalise pixel float values to between 0 and 1"""
        return cv.normalize(img, None, 1, 0, cv.NORM_MINMAX, dtype=cv.CV_64F)


    def getAbsGrad(self, origImg: np.ndarray, scale = True) -> np.ndarray:
        """Get values of absolute image gradient = sqrt(fxfx + fyfy)"""
        img = origImg
        if scale:
            img = cv.GaussianBlur(origImg, ksize=(31,31), sigmaX=15, sigmaY=15, borderType=cv.BORDER_DEFAULT)
        imgx = np.abs(cv.Sobel(img, cv.CV_64F, dx=1, dy=0, ksize=5))
        imgy = np.abs(cv.Sobel(img, cv.CV_64F, dx=0, dy=1, ksize=5))
        imgx[imgx<0.05] = 0                                     # Removes discretisation error
        imgy[imgy<0.05] = 0                                     # Removes discretisation error
        imgd = self.normalise(np.sqrt(imgx**2 + imgy**2))       # Could replace this by (np.abs(imgx) + np.abs(imgy))/2
        return imgd

    def depthGradient(self, blackout_flag, dead_end_flag, dead_end_flag_2) -> np.ndarray:
        """Apply getAbsGrad on depth data & return uint8 values between 0 and 255. Useful for detecting the 'road region'. Uses Depth data."""
        
        if self.depthGrad is not None: # If already computed
            return self.depthGrad

        imggrad = self.getAbsGrad(self.depth)
        if dead_end_flag == 1:
            imggrad[imggrad > 0.15] = -1
            imggrad[imggrad >= 0] = 1
            imggrad[imggrad < 0] = 0
        elif dead_end_flag == 2:
            imggrad[imggrad > 0.08] = -1
            imggrad[imggrad >= 0] = 1
            imggrad[imggrad < 0] = 0
        elif dead_end_flag == 3:
            imggrad[imggrad > 0.02] = -1
            imggrad[imggrad >= 0] = 1
            imggrad[imggrad < 0] = 0
        elif dead_end_flag == 4:
            imggrad[imggrad > 0.02] = -1
            imggrad[imggrad >= 0] = 1
            imggrad[imggrad < 0] = 0
        elif dead_end_flag == 5:
            imggrad[imggrad > 0.05] = -1
            imggrad[imggrad >= 0] = 1
            imggrad[imggrad < 0] = 0
        else:
            imggrad[imggrad > 0.06] = -1
            imggrad[imggrad >= 0] = 1
            imggrad[imggrad < 0] = 0

        imggrad = cv.morphologyEx(imggrad, cv.MORPH_OPEN, cv.getStructuringElement(cv.MORPH_RECT, (9, 9)))
        imggrad255 = cv.normalize(np.sqrt(imggrad), None, 255, 0, cv.NORM_MINMAX, cv.CV_8U)
        self.depthGrad= imggrad255
        return imggrad255    
    
    def greyLaplacian(self, threshold=0.005) -> np.ndarray:
        """Detects lines in the image.
        @param threshold: suppresses pixel values (0-1) below threshold to remove discretisation noise.
        """

        if self.greylap is not None: # If already computed
            return self.greylap

        imgGrey = self.grey 
        imgBlur = imgGrey
        imgBlur = cv.GaussianBlur(imgGrey, ksize=(5,5), sigmaX=5, sigmaY=5, borderType=cv.BORDER_DEFAULT)
        imglap = cv.Laplacian(imgBlur, cv.CV_64F, None, borderType=cv.BORDER_DEFAULT)
        imglap[imglap < threshold] = 0                          # Removes discretisation error
        imglap255 = cv.normalize(np.abs(imglap), None, 255, 0, cv.NORM_MINMAX, dtype=cv.CV_8U)
        self.greylap = imglap255
        return imglap255


    def depthMultiOtsu(self, classes=3) -> np.ndarray:
        """Performs Multi-Otsu on depth data. """

        if self.depthOtsu is not None: # If already computed
            return self.depthOtsu

        img = self.depth
        thresholds = threshold_multiotsu(img, classes=classes)
        regions = np.digitize(img, bins=thresholds)
        regions = np.array(regions * (240//classes), dtype=np.uint8)
        self.depthOtsu = regions
        return regions


    def saveImgs(self, imgs, path, title, idxStart=0) -> None:
        """Save the list of images in the directory specified by path with the list of names specified by title
        Images are always stored in contiguous sequences starting at idxStart.

        @param List[np.ndarray] imgs: list of images to be saved
        @param String           path: path to directory where images are to be saved
        @param String           title: String containing a %d placeholder for the sequence number of each image.
        @param Int              idxStart: Integer to start sequence numbering in filenames
        """
        n = len(imgs)
        for img, i in zip(imgs, range(idxStart, idxStart+n)):
            cv.imwrite(os.path.join(path, title % i), img)


    def getLargestComponent(self, img):
        nComp, output, stats, centroids = cv.connectedComponentsWithStats(img, connectivity=8)
        sizes = stats[:, -1]
        maxLab = 1
        maxSz = sizes[1]
        for i in range(1, nComp):
            if sizes[i] > maxSz:
                maxLab = i
                maxSz = sizes[i]
        img2 = np.zeros_like(output, dtype=np.uint8)
        img2[output == maxLab] = 255
        return img2, nComp

    def getRoadMask(self, blackout_flag, dead_end_flag, dead_end_flag_2):
        depthGrad = self.depthGradient(blackout_flag, dead_end_flag, dead_end_flag_2)
        avg_depth_grad = depthGrad.mean()
        if avg_depth_grad < 80.0:
            blackout_flag = 1
            print("Image Blackout!! {}".format(avg_depth_grad))
        else:
            blackout_flag = 0
            print("Image Clear!! {}".format(avg_depth_grad))
        roadMask, nComp = self.getLargestComponent(depthGrad)
        return roadMask, nComp, blackout_flag

    def eulerDist(self, pt1, pt2):
        ans = math.sqrt((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2) 
        return ans

    def carDetect(self, roadMask, nComp):
        contours,hierarchy = cv.findContours(roadMask, 1, 2)
        if len(contours) >= 2:
            cnt = contours[0]
            rect = cv.minAreaRect(cnt)
            box = cv.boxPoints(rect)

            ys = box[:, 1].copy()
            points = []
            for _ in range(4):
                min_idx = np.argmin(ys)
                point = box[min_idx]
                points.append(point)
                ys[min_idx] = 1000
            pt1_y = np.array((points[0]+points[1])/2, dtype=np.int32)
            pt2_y = np.array((points[2]+points[3])/2, dtype=np.int32)
            v1 = (pt1_y - pt2_y)

            xs = box[:, 0].copy()
            points = []
            for _ in range(4):
                min_idx = np.argmin(xs)
                point = box[min_idx]
                points.append(point)
                xs[min_idx] = 1000
            pt1_x = np.array((points[0]+points[1])/2, dtype=np.int32)
            pt2_x = np.array((points[2]+points[3])/2, dtype=np.int32)
            v2 = (pt1_x - pt2_x)

            if np.linalg.norm(v1) > np.linalg.norm(v2):
                pt1 = pt1_y
                pt2 = pt2_y
            else:
                pt1 = pt1_x
                pt2 = pt2_x

            self.pt1 = pt1
            self.pt2 = pt2
            self.box = box

            w = np.linalg.norm(box[1] - box[0])
            h = np.linalg.norm(box[2] - box[1])
            area = w*h
            flag = 1
            if area > 16000:
                print("Car Detected!!   Area: {}".format(w*h))
                return pt1, pt2, box, flag
            else:
                print("Car Lost!!")
                return self.pt1, self.pt2, self.box, flag
        else:
            flag = 0
            print("Car Lost!!")
            return self.pt1, self.pt2, self.box, flag
            # use prev
    
    def targetVector(self, blackout_flag, dead_end_flag, dead_end_flag_2):
        roadMask, nComp, blackout_flag = self.getRoadMask(blackout_flag, dead_end_flag, dead_end_flag_2)

        pt1, pt2, box, flag = self.carDetect(roadMask, nComp)
        carForward = pt1 - pt2

        # print(pt1, pt2, pt2-pt1)

        croadMask = cv.cvtColor(roadMask, cv.COLOR_GRAY2BGR)
        box = np.int0(box)
        # print(box)
        mid = (box[0] + box[2]) // 2
        len = max(self.eulerDist(box[0], box[1]), self.eulerDist(box[1], box[2]))
        len = int(len)

        cv.fillConvexPoly(roadMask, box, 255)
        # cv.imshow('img2', roadMask)
        # cv.waitKey(10000)

        tlen = len
        xmid = mid[0]
        
        target_y = max(mid[1] - tlen, 0)
        while(roadMask[target_y, xmid] == 0 and (tlen-len//2) > 2):
            # print('Target Length:', tlen)
            tlen = len//2 + (tlen-len//2)//2
            target_y = max(mid[1] - tlen, 0)
            
        # print(target_y, xmid)
        left, right = 0, self.depth.shape[1]
        ct = 0
        # print(np.amax([target_y,:]))
        for i in range(self.depth.shape[1]):
            if xmid - i >= 0 and roadMask[target_y, xmid-i] == 0 and ct%2 == 0:
                left = xmid-i
                ct += 1
            if xmid + i < self.depth.shape[1] and roadMask[target_y, xmid+i] == 0 and ct < 2:
                right = xmid+i
                ct += 2

            if ct == 3:
                break
        
        croadMask[target_y, :, :] = 0.5
        target = ((left+right)//2, target_y)
        center = ((box[0] + box[2])/2).astype(int)
        
        cv.arrowedLine(croadMask, (mid[0], mid[1]), target, (0,0,255), 5)
        cv.arrowedLine(croadMask, pt2, pt1, (255,0,255), 5)
        cv.drawContours(croadMask,[box],0,(255.0,0),4)
        cv.circle(croadMask, center, 10, (255, 255, 0), -1)
        targetVector = (target[0] - mid[0], target[1] - mid[1])
        targetAngle = math.atan2(targetVector[1], targetVector[0]) - math.atan2(carForward[1], carForward[0])
        targetAngle *= 180/math.pi
        # print(targetAngle)
        return croadMask, targetAngle, pt1, pt2, box, carForward, center, flag, blackout_flag, dead_end_flag, dead_end_flag_2