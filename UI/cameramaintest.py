def _main():
 
    @QtCore.pyqtSlot(cv.iplimage)
    def onNewFrame(frame):
        cv.CvtColor(frame, frame, cv.CV_RGB2BGR)
        msg = "processed frame"
        font = cv.InitFont(cv.CV_FONT_HERSHEY_DUPLEX, 1.0, 1.0)
        tsize, baseline = cv.GetTextSize(msg, font)
        w, h = cv.GetSize(frame)
        tpt = (w - tsize[0]) / 2, (h - tsize[1]) / 2
        cv.PutText(frame, msg, tpt, font, cv.RGB(255, 0, 0))
 
    import sys
 
    app = QtGui.QApplication(sys.argv)
 
    cameraDevice = CameraDevice(mirrored=True)
 
    cameraWidget1 = CameraWidget(cameraDevice)
    cameraWidget1.newFrame.connect(onNewFrame)
    cameraWidget1.show()
 
    cameraWidget2 = CameraWidget(cameraDevice)
    cameraWidget2.show()
 
    sys.exit(app.exec_())
