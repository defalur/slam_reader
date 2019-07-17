import cv2

class GenericReader:
    def read(self):
        raise NotImplementedError("Not implemented")

class ImageReader(GenericReader):
    def __init__(self, src, prefix, start, ext):
        self.prefix = src + '/' + prefix
        self.frame_id = start
        self.ext = ext

    def read(self):
        name = prefix_full + str(frame_id) + '.' + ext
        self.frame_id += 1
        return cv2.imread(name, cv2.IMREAD_GRAYSCALE)

class VideoReader(GenericReader):
    def __init__(self, path):
        self.path = path
        self.capture = cv2.VideoCapture(path)

    def read(self):
        ret, frame = cap.read()
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)