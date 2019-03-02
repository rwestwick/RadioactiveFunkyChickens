import cv
 
from PyQt4 import QtCore
 
class CameraDevice(QtCore.QObject):
 
    _DEFAULT_FPS = 30
 
    newFrame = QtCore.pyqtSignal(cv.iplimage)
 
    def __init__(self, cameraId=0, mirrored=False, parent=None):
        super(CameraDevice, self).__init__(parent)
 
        self.mirrored = mirrored
 
        self._cameraDevice = cv.CaptureFromCAM(cameraId)
 
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._queryFrame)
        self._timer.setInterval(1000/self.fps)
 
        self.paused = False
 
    @QtCore.pyqtSlot()
    def _queryFrame(self):
        frame = cv.QueryFrame(self._cameraDevice)
        if self.mirrored:
            mirroredFrame = cv.CreateImage(cv.GetSize(frame), frame.depth, \
                frame.nChannels)
            cv.Flip(frame, mirroredFrame, 1)
            frame = mirroredFrame
        self.newFrame.emit(frame)
 
    @property
    def paused(self):
        return not self._timer.isActive()
 
    @paused.setter
    def paused(self, p):
        if p:
            self._timer.stop()
        else:
            self._timer.start()
 
    @property
    def frameSize(self):
        w = cv.GetCaptureProperty(self._cameraDevice, \
            cv.CV_CAP_PROP_FRAME_WIDTH)
        h = cv.GetCaptureProperty(self._cameraDevice, \
            cv.CV_CAP_PROP_FRAME_HEIGHT)
        return int(w), int(h)
 
    @property
    def fps(self):
        fps = int(cv.GetCaptureProperty(self._cameraDevice, cv.CV_CAP_PROP_FPS))
        if not fps > 0:
            fps = self._DEFAULT_FPS
        return fps
