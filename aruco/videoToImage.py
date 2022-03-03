import cv2
import os

class VideoToImage():
    def __init__(self, video_path, n_images):
        self.video_path = video_path
        #self.image_path = image_path
        self.n_images = n_images+1 # number of images to capture
        self.vidcap = cv2.VideoCapture(self.video_path)
        self.sec = 0
        self.count = 0
        fps = self.vidcap.get(cv2.CAP_PROP_FPS)      # OpenCV2 version 2 used "CV_CAP_PROP_FPS"
        frame_count = int(self.vidcap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.duration = frame_count/fps
        #number of image is calculated as follows:
        #number of image = (video duration in seconds) * (frame rate)
        self.frame_rate = self.n_images/self.duration
        print(self.duration)
    def main(self):
        for self.count in range(self.n_images):
            self.sec += 1/self.frame_rate
            self.vidcap.set(cv2.CAP_PROP_POS_MSEC,self.sec*1000)
            hasFrames,image = self.vidcap.read()
            self.count += 1
            file_exists = os.path.exists("image"+str(self.count)+".jpg")
            if hasFrames:
                if not file_exists:
                    cv2.imwrite("image"+str(self.count)+".jpg", image)     # save frame as JPG file
                else:
                    print("file exists")
                    break

if __name__ == '__main__':
    trial = VideoToImage('video4.mp4', 120)
    trial.main()
