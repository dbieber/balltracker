from optparse import OptionParser

import os
import sys

def create_video():
    FPS = 30
    endTime = self.video['duration']
    frametimes = list(np.arange(0, endTime, 1000.0 / FPS)) + [endTime]

    # Stream to ffmpeg: http://stackoverflow.com/questions/13294919/can-you-stream-images-to-ffmpeg-to-construct-a-video-instead-of-saving-them-t
    now = int(time.time())
    video_out = "vid%d.mp4" % now
    p = subprocess.Popen(['ffmpeg', '-y', '-f', 'image2pipe', '-vcodec', 'png', '-r', str(FPS), '-i', '-', '-vcodec', 'mpeg4', '-qscale', '5', '-r', str(FPS), video_out], stdin=subprocess.PIPE)
    for t in frametimes:
        self.draw_to_time(t)
        self.image.save(p.stdin, 'PNG')

    p.communicate()

def parseline(line):
    line = line.strip()
    if line[0] == "%":
        return

    tokens = line.split(" ")
    scale = 1
    return [float(tokens[0]),
            int(scale*float(tokens[1])),
            int(scale*float(tokens[2])),
            int(scale*float(tokens[3]))]

def visualize(filename):
    f = open(filename)
    for line in f:
        line = parseline(line)
        if line is None:
            continue
        print time

def main():
    parser = OptionParser()
    parser.add_option("-f", "--file", dest="filename")
    # parser.add_option("-c", "--camera", dest="camera", type=int)

    (options, args) = parser.parse_args()
    print options.filename
    visualize(options.filename)

if __name__ == '__main__':
    main()
