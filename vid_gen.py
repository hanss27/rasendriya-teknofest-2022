import cv2
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-p', "--port", type=int, default=0, help='Video source port')
parser.add_argument('-r', "--resize", type=int, default=200, help='Video resize')
parser.add_argument('-f', "--fps", type=int, default=15, help='Video fps')
args = parser.parse_args()

cap = cv2.VideoCapture(args.port)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

writer = cv2.VideoWriter('calibcam.mp4', cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (args.resize, args.resize))

while True:
  ret,frame= cap.read()

  if not ret:
    break

  frame = frame[0:0+height, height-int(width/2):int(width/2)+height]
  frame = cv2.resize(frame, (args.resize, args.resize))
  print(frame.shape[1], frame.shape[0])
  writer.write(frame)

  cv2.imshow('frame', frame)

  if cv2.waitKey(args.fps) & 0xFF == 27:
    break

cap.release()
writer.release()
cv2.destroyAllWindows()
