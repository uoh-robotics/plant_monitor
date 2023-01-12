import argparse
import base64

import requests
import numpy as np
import cv2 as cv
from common_pyutil.monitor import Timer


timer = Timer()


def show_live(host, port, flip=0, convert=None):
    server = f"http://{host}:{port}"
    i = 0
    while True:
        key = cv.waitKey(1)
        with timer:
            resp = requests.get(f"{server}/get_frame")
        img = cv.imdecode(np.frombuffer(base64.b64decode(resp.content), dtype=np.uint8),
                          flags=cv.IMREAD_COLOR)
        img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)
        img = cv.resize(img, (img.shape[0], img.shape[1]))
        # if convert:
        #     img = img[:, :, ::-1]
        if key == ord("a"):
            resp = requests.get(f"{server}/horizontal?delta=1")
            print("Left")
        elif key == ord("d"):
            resp = requests.get(f"{server}/horizontal?delta=-1")
            print("Right")
        elif key == ord("x"):
            resp = requests.get(f"{server}/horizontal?delta=0")
            print("Stopping")
        elif key == ord("s"):
            resp = requests.get(f"{server}/swing_a?delta=100")
            print("Top Up")
        elif key == ord("w"):
            resp = requests.get(f"{server}/swing_a?delta=-100")
            print("Top Up")
        elif key == 81:
            resp = requests.get(f"{server}/rotate?delta=100")
            print("Rotating left")
        elif key == 83:
            resp = requests.get(f"{server}/rotate?delta=-100")
            print("Rotating right")
        elif key == 82:
            resp = requests.get(f"{server}/swing_b?delta=100")
            print("Bottom Up")
        elif key == 84:
            resp = requests.get(f"{server}/swing_b?delta=-100")
            print("Bottom down")
        elif key == ord("q") or key == 27:
            print("Aborted Rotation")
            break
        try:
            cv.imshow("img", img)
            # print(i)
            i += 1
        except KeyboardInterrupt:
            cv.destroyAllWindows()
    cv.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("host")
    parser.add_argument("-p", "--port", type=int, default=8080)
    parser.add_argument("--no-bgr2rgb", dest="bgr2rgb", action="store_false")
    args = parser.parse_args()
    show_live(args.host, args.port, convert=args.bgr2rgb)
