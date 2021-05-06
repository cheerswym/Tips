import os
import sys

pngs = os.listdir(sys.argv[1])
output_format = sys.argv[2] if len(sys.argv) > 2 else "gif"

for seq in pngs:
    cmd = "ffmpeg -framerate 10 -pattern_type glob -i '{0}/{1}/*.png' -r 15 '{0}/{1}.{2}'".format(
        sys.argv[1], seq, output_format)
    os.system(cmd)
