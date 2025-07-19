import cv2
import os

def extractFrames(videoPath, outputDir):
    cap = cv2.VideoCapture(videoPath)
    os.makedirs(outputDir, exist_ok=True)
    count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        filename = os.path.join(outputDir, f"frame{count:06d}.png")
        cv2.imwrite(filename, frame)
        count += 1
    cap.release()

if __name__ == "__main__":
    videoPath = "1_international/tests/output_vfr_1.mp4"
    outputDir = "1_international/tests/victims/frames"
    extractFrames(videoPath, outputDir)