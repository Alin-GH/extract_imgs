# coding:utf-8
import rospy, rosbag
import os
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import pcl, logging
import argparse
import glob

def Args_B2P():
    parse = argparse.ArgumentParser(
        usage="python miivi.py  ---  -cbs ./bag_path -sp  "
    )

    parse.add_argument("-cbs", "--camera_bag_path", type=str, required=True)

    parse.add_argument("-sp", "--save_path", type=str, required=True)
    args = parse.parse_args()
    return args


def saveImage(topic, msg, time, save_path):
    path = save_path

    if not os.path.exists(path):
        os.makedirs(path)

    path = save_path + "/" + "time_" + str(time) + ".jpg"
    img_data = np.fromstring(msg.data, np.uint8)
    cv_image = cv2.imdecode(img_data, cv2.IMREAD_COLOR)

    cv2.imwrite(path, cv_image)


def getImage(topic, msg, image_arr, times_arr, target_topic):
    if topic == str(target_topic):
        msg_time = msg.header.stamp.to_nsec()

        times_arr.append(msg_time)
        image_arr.append(msg)




topics = [

    "/pandar_pointcloud",
    "/dev/video4/compressed",  #camera f30
    "/dev/video2/compressed",  #camera l60
    "/dev/video0/compressed",  #camera l120
    "/dev/video6/compressed",  #camera f120
    "/dev/video5/compressed",  #camera f60
    "/dev/video3/compressed", #camera r60
    "/dev/video1/compressed", #camera_r120
    "/dev/video7/compressed"  #camera_fd120

]


if __name__ == "__main__":
    rospy.init_node("listener", anonymous=True)    
    logging.basicConfig(level=logging.DEBUG)
    logging.info("trans bag")

    args = Args_B2P()

    camera_bag_path = args.camera_bag_path

    save_path = args.save_path
    print(len(topics))
    print(topics[8])

    '''sort files'''
    ##sort 
    bag_name_cameras = [x for x in sorted(glob.glob(camera_bag_path+'/miivi*.bag'))]

    print("bag_name_camera", bag_name_cameras)

    for bagfile_cs in bag_name_cameras:

        image_f30_arr =[]
        time_f30_arr = []

        image_f60_arr =[]
        time_f60_arr = []

        image_f120_arr = []
        time_f120_arr = []

        image_fd120_arr =[]
        time_fd120_arr = []

        bag_name_camera = bagfile_cs 

        print("bag_name_camera", bagfile_cs)


        step = 5

        ###loading camera
        if (os.path.splitext(bag_name_camera)[1] == ".bag"):
            bagimg = rosbag.Bag(bag_name_camera)
            
            ind = 0
            for topic, msg, t in bagimg.read_messages(topics=topics):

                # ind += 1
                # if ind % step !=0:
                #     continue

                getImage(topic, msg, image_f30_arr, time_f30_arr, topics[1])  #getImage camear f30
                getImage(topic, msg, image_f60_arr, time_f60_arr, topics[5])  #getImage camear f60
                getImage(topic, msg, image_f120_arr, time_f120_arr, topics[4])  #getImage camear f120
                getImage(topic, msg, image_fd120_arr, time_fd120_arr, topics[8])  #getImage camear r120

            bagimg.close()

    
        print("len of image f30 f60 f120 fd120  is ", \
            len(image_f30_arr), \
                len(image_f60_arr), \
                    len(image_f120_arr), \
                        len(image_fd120_arr)
            )
        ##相机录制的长度不一致处理
        '''相机长度不一致'''
        min_len_camera = min(len(image_f30_arr), len(image_f60_arr), len(image_f120_arr),len(image_fd120_arr))
        
        print('camera min length ', min_len_camera)


        image_f120_arr = image_f120_arr[:min_len_camera]
        image_f60_arr = image_f60_arr[:min_len_camera]
        image_f30_arr = image_f30_arr[:min_len_camera]
        image_fd120_arr = image_fd120_arr[:min_len_camera]



        time_f120_arr = time_f120_arr[:min_len_camera]
        time_f60_arr = time_f60_arr[:min_len_camera]
        time_f30_arr = time_f30_arr[:min_len_camera]
        time_fd120_arr = time_fd120_arr[:min_len_camera]


        '''path to save '''

        save_camear_f30 = save_path + "/" + "camera_f30" + "/" 
        save_camear_f120 = save_path + "/" + "camera_f120" + "/" 
        save_camear_f60 = save_path + "/" + "camera_f60" + "/" 
        save_camear_fd120 = save_path + "/" + "camera_fd120" + "/" 


        ##compare time-stamp
        for idx, time in enumerate(time_f60_arr):

            img_msg = image_f60_arr[idx]
            image_time = time
            print(image_time)

            saveImage(topics[2], img_msg, image_time, save_camear_f60)
        print(" f30 save image ok!")

        for idx, time in enumerate(time_f30_arr):

            img_msg = image_f30_arr[idx]
            image_time = time
            print(image_time)

            saveImage(topics[1], img_msg, image_time, save_camear_f30)
        print(" f60 save image ok!")

     
        for idx, time in enumerate(time_f120_arr):

            img_msg = image_f120_arr[idx]
            image_time = time
            print(image_time)

            saveImage(topics[1], img_msg, image_time, save_camear_f120)
        print(" f120 save image ok!")

        for idx, time in enumerate(time_fd120_arr):

            img_msg = image_fd120_arr[idx]
            image_time = time
            print(image_time)

            saveImage(topics[1], img_msg, image_time, save_camear_fd120)
        print(" fd120 save image ok!")            
