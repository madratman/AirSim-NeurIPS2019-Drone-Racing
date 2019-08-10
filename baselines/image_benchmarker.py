import airsimneurips as airsim
import time
import cv2
import numpy as np

def main():
    num_requests = 1000
    client = airsim.MultirotorClient()
    client.confirmConnection()
    level_names = ["Soccer_Field_Easy", "Soccer_Field_Medium", "ZhangJiaJie_Medium", "Building99_Hard"]

    for (level_idx, level_name) in enumerate(level_names):
        client.simLoadLevel(level_name)
        time.sleep(3)
        request = [airsim.ImageRequest("fpv_cam", airsim.ImageType.Scene, False, False)]

        start_time_all = time.time()

        for _ in range(num_requests):
            # iter_time = start_time
            response = client.simGetImages(request)
            # iter_time = 1000 * (time.time() - start_time)
            # print "iter_time  = {} milliseconds".format(iter_time)
            # img_rgb_1d = np.fromstring(response[0].image_data_uint8, dtype=np.uint8) 
            # img_rgb = img_rgb_1d.reshape(response[0].height, response[0].width, 3)
            # cv2.imshow("img_rgb", img_rgb)
            # cv2.waitKey(1)

        avg_fps = 1.0 / ((time.time() - start_time_all) / float(num_requests))
        avg_time = 1000 * ((time.time() - start_time_all) / float(num_requests))
        # print(level_name + ": {} milliseconds for {} num_requests".format(avg_time, num_requests))
        print(level_name + ": {} avg_fps for {} num_requests".format(avg_fps, num_requests))

if __name__ == "__main__":
    main()