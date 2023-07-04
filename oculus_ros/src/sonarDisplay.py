import cv2
import numpy as np


class warpSonar:
    def __init__(self):
        self.srcX = None
        self.srcY = None

        self.mapx = None
        self.mapy = None


    def jls_extract_def(self):
        
        return 

    def createMaps(self, srcX, srcY, degVec):

        self.srcY = srcY
        self.srcX = srcX

        radVec = np.deg2rad(degVec)

        #radVec2 = np.linspace(radVec[0], radVec[-1], len(radVec)*2)
        #import ipdb; ipdb.set_trace()

        extW = srcX/2+srcY*np.sin(radVec[0])
        
        if extW < 0:
            self.mapx = np.zeros( (srcY, int(srcX+2*np.abs(extW)) ), dtype='float32')
            self.mapy = self.mapx.copy()
            shiftX = np.abs(extW)
        else:
            self.mapx = np.zeros( (srcY, int(srcX) ), dtype='float32')
            self.mapy = self.mapx.copy()
            shiftX = 0

        ang     = radVec[-1] 
        denFac  = 1

        for i,teta in enumerate(np.array(np.linspace(-ang,ang,srcX*denFac))):
        #for i,teta in enumerate(radVec):
            #for j in range(srcY):
            for j in np.linspace(0, srcY, srcY*denFac):
                b=j
                py = b*np.cos(teta)
                px = srcX/2+b*np.sin(teta)
                try:
                    self.mapx[int(py), int(px+shiftX)]=i/denFac
                    self.mapy[int(py), int(px+shiftX)]=j
                except:
                    pass


    def warpSonarImage(self, metadata, img):
        # sx,sy=img.shape[1],img.shape[0]
        # if (sx != self.srcX) or (sy != self.srcY):
        #     degVec = metadata["beamsDeg"]
        #     self.createMaps(sx,sy,degVec)
        #     print('init')

        # # warped = cv2.remap(img, self.mapx, self.mapy, cv2.INTER_LINEAR)
        # warped = cv2.remap(img, self.mapx, self.mapy, cv2.INTER_CUBIC)
        
        # return warped
        return self.cartesian_to_polar(metadata, img)
    
    
    def cartesian_to_polar(self, metadata, sonar_data):
        n_ranges = metadata['nRanges']
        azimuth_bounds = np.deg2rad(metadata['beamsDeg'])
        minus_width = int(np.floor(n_ranges * np.sin(azimuth_bounds[0])))
        plus_width = int(np.ceil(n_ranges * np.sin(azimuth_bounds[-1])))
        width = plus_width - minus_width
        origin_x = abs(minus_width)

        db = (azimuth_bounds[-1] - azimuth_bounds[0]) / metadata['nBeams'] if metadata['nBeams'] > 0 else 1
        dy, dx = np.indices((n_ranges, width))
        dx -= origin_x
        dy = n_ranges - dy

        range_val = np.sqrt(dx * dx + dy * dy).astype('int')
        azimuth = np.arctan2(dx, dy)
        yp = ((azimuth - azimuth_bounds[0]) / db).astype('int')

        mask = (range_val >= 0) & (range_val < sonar_data.shape[0]) & (yp >= 0) & (yp < sonar_data.shape[1])
        new_image = np.zeros(shape=(n_ranges, width), dtype='uint8')
        new_image[dy[mask], dx[mask] + origin_x] = sonar_data[range_val[mask], yp[mask]]

        return new_image



if __name__=='__main__':
    import pickle
    
    cv2.namedWindow('Original Image', 0)
    cv2.namedWindow('Warped Image', 0)
    

    with open('inData.pkl', 'rb') as fid:
        metadata, sonImg = pickle.load(fid)

    cv2.imshow('Original Image', sonImg)
    ws = warpSonar()

    scales = [1.0, 1.2, 1.5, 2.0, 3.0]
    
    for scale in scales:

        inImg = cv2.resize(sonImg, None, fx=1, fy=scale)

        warped = ws.warpSonarImage(metadata, inImg)
        warped = cv2.flip(warped, 0)
        print('--->', warped.shape, inImg.shape)

        cv2.imshow('Warped Image', warped)
        cv2.waitKey(0)

    cv2.destroyAllWindows()
