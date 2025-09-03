import argparse
import time
import os
from sys import platform
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image

from models import *
from utils.datasets import *
from utils.utils import *

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

from geometry_msgs.msg import PoseWithCovarianceStamped
import traceback
    
class CategoryCounter:
    def __init__(self):
        self.counter = {}
    
    def add_category(self, category):
        self.counter[category] = self.counter.get(category, 0) + 1
    
    def get_sorted_categories(self):
        sorted_categories = sorted(self.counter.items(), key=lambda x: x[1], reverse=True)
        filtered_categories = [(category, count) for category, count in sorted_categories if count >= 5]
        return filtered_categories


counter = CategoryCounter()
# 识别E区作物，当 x>2.226 时，开始识别，当 y<-2.376 时停止识别
# 识别D区作物，当 y<-3.418 时，开始识别，当 y>-3.418 时停止识别
# 识别C区作物，当 y>-2.498 时，开始识别，当 y<-2.498 时停止识别
# 识别B区作物，当 y<-3.514 时，开始识别，当 y>-3.514 时停止识别
# 识别F区果实，当 y<-1.198 and x<2.254 时，开始识别，当 y>-1.198 时停止识别
'''
2.254, -0.140
2.702, -2.516
2.673, -3.391
4.984, -2.509
4.706, -3.592
0.878, -1.198
'''
scan_place=[["position.x>2.254","position.y<-2.516"],["position.y<-2.516","position.x>3.800"],["position.y>-2.498","position.y<-2.498"],["position.y<-3.514","position.y>-3.514"],[["position.y<-1.198 ","position.x<2.254"],"position.y>-1.198"]]

place_flag=0
place_recode=0
def amcl_pose_callback(msg):
    global place_flag
    global place_recode
    # 处理接收到的位置信息
    # 在这个例子中，我们只打印位置坐标
    position = msg.pose.pose.position
    #print("Position: x={}, y={}, z={}".format(position.x, position.y, position.z))
    if place_recode<5:
        #前四个点状态不一样，place_flag为1是进去的状态，2是出来的状态，0是空闲状态
        if(place_recode<4 and eval(scan_place[place_recode][0]) and place_flag==0):
            place_flag=1
        elif (place_recode<4 and eval(scan_place[place_recode][1]) and place_flag==1):
            place_flag=2
            place_recode+=1
        elif (place_recode==4 and place_flag==0 and eval(scan_place[place_recode][0][0]) and eval(scan_place[place_recode][0][1]) ):
            place_flag=1
        elif (place_recode==4 and place_flag==1 and eval(scan_place[place_recode][1]) ):
            place_flag=2
            place_recode+=1

goal_index=1

is_achieve=0
final_nzw=set()
#西瓜，黄瓜，玉米
gs_num={"西瓜":[1,1,1,1],"黄瓜":[1,3,2,2],"玉米":[1,2,2,1]}
start_time=0
def callback(data):
    global goal_index

    global is_achieve
    if data.status.status == 3:
        
        if goal_index < 9:
            # 存储数据,拿多个数据
            
            if(goal_index==5 or goal_index==8):
                goal_index+=1
                return
            is_achieve=1
            goal_index+=1
        else:
            # 播报
            with open('/home/ucar/Desktop/ucar/src/counter.txt', 'a+') as k:
                k.write(f"=================================={place_recode}\n")
                sorted_categories = counter.get_sorted_categories()
                k.write(str(sorted_categories))
                k.write("\n\n")
            with open('/home/ucar/Desktop/ucar/src/fame_data.txt', 'a+') as f:
                counts = {"玉米": 0, "黄瓜": 0, "西瓜": 0, "小麦": 0, "水稻": 0}
                if place_recode == 5:
                    sorted_categories = counter.get_sorted_categories()
                    for item in sorted_categories:
                        category = item[0][:2]  # 提取前两个字作为分类
                        if item[0][2:4] == "果实":
                            counts[category] += gs_num[category][int(item[0][-1])-1]
                    max_category = max(counts, key=counts.get)  # 获取数量最高的分类
                    max_count = counts[max_category]  # 获取数量最高的数量
                    f.write(str(max_category)+"\n"+str(max_count)+"\n")

            os.system("sh /home/ucar/Desktop/ucar/src/AIsound/bin/result.sh")

def detect(
        cfg,
        data_cfg,
        weights,
        images,
        output='output',  # 输出文件夹
        img_size=416,
        conf_thres=0.5,
        nms_thres=0.5,
        save_txt=False,
        save_images=True,
        webcam=False
):
    global is_achieve
    global final_nzw
    global start_time
    global place_flag
    global place_recode
    global counter
    
    device = torch_utils.select_device()
    if os.path.exists(output):
        shutil.rmtree(output)  # 删除输出文件夹
    os.makedirs(output)  # 创建新的输出文件夹

    # 初始化模型
    if ONNX_EXPORT:
        s = (416, 416)  # onnx模型图像大小
        model = Darknet(cfg, s)
    else:
        model = Darknet(cfg, img_size)

    # 加载权重
    if weights.endswith('.pt'):  # pytorch格式
        model.load_state_dict(torch.load(weights, map_location=device)['model'])
        print("--------------------模型权重加载完成！！！----------------------------")
        os.system("play xiaoxin.MP3")
    else:  # darknet格式
        _ = load_darknet_weights(model, weights)

    # 融合Conv2d + BatchNorm2d层
    model.fuse()

    # 评估模式
    model.to(device).eval()

    if ONNX_EXPORT:
        img = torch.zeros((1, 3, s[0], s[1]))
        torch.onnx.export(model, img, 'weights/export.onnx', verbose=True)
        return

    # 设置数据加载器
    vid_path, vid_writer = None, None
    if webcam:
        save_images = False
        dataloader = LoadWebcam(img_size=img_size)
    else:
        dataloader = LoadImages(images, img_size=img_size)

    # 获取类别和颜色
    classes = load_classes(parse_data_cfg(data_cfg)['names'])
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(classes))]

    for i, (path, img, im0, vid_cap) in enumerate(dataloader):
        t = time.time()
        save_path = str(Path(output) / Path(path).name)

        # 获取检测结果
        img = torch.from_numpy(img).unsqueeze(0).to(device)
        pred, _ = model(img)
        det = non_max_suppression(pred, conf_thres, nms_thres)[0]

        if det is not None and len(det) > 0:
            # 将416的框缩放到真实图像大小
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
            #final_result=[]
            # 将结果打印到屏幕上
            for c in det[:, -1].unique():
                final_nzw.add(classes[int(c)])
                counter.add_category(str(classes[int(c)]))
                
            if(place_flag==1):
                if(start_time==0):
                    start_time=time.time()
                    final_nzw=set()
                    counter=CategoryCounter()

            if(place_flag==2):
                if place_recode<5:
                    with open('/home/ucar/Desktop/ucar/src/fame_data.txt','a+') as f:
                        counts = {"玉米":0,"黄瓜":0,"西瓜":0,"小麦":0,"水稻":0}
                        if  place_recode<5:
                            sorted_categories = counter.get_sorted_categories()
                            for item in sorted_categories:
                                category = item[0][:2]  # 提取前两个字作为分类
                                if item[0][2:4]=="作物":
                                    counts[category] += 1
                            max_category = max(counts, key=counts.get)  # 获取数量最高的分类
                            max_count = counts[max_category]  # 获取数量最高的数量
                            f.write(str(max_category)+"\n")
                    with open('/home/ucar/Desktop/ucar/src/counter.txt','a+') as k:
                        k.write(f"=================================={place_recode}\n")
                        sorted_categories = counter.get_sorted_categories()
                        k.write(str(sorted_categories))
                        k.write("\n\n")

                place_flag=0
                final_nzw=set()
                
                is_achieve=0
                start_time=0


            #print(final_result,"\n")
            # 绘制检测结果的边界框和标签
            for *xyxy, conf, cls_conf, cls in det:
                if save_txt:  # 写入文件
                    with open(save_path + '.txt', 'a') as file:
                        file.write(('%g ' * 6 + '\n') % (*xyxy, cls, conf))

                # 将边界框添加到图像中
                label = '%s %.2f' % (classes[int(cls)], conf)
                im0 = plot_one_box(xyxy, im0, label=label, color=colors[int(cls)])
        #print('完成。 (%.3fs)' % (time.time() - t))

        if webcam:  # 显示实时网络摄像头
            # cv2.imshow(weights, im0)

            im0 = cv2.resize(im0, (640, 480)) # 宽 高
            image_temp.header = Header(stamp=rospy.Time.now())   #定义图片header
            image_temp.data=np.array(im0).tostring()   #图片内容，这里要转换成字符串

            cam_pub.publish(image_temp)

        if save_images:  # 保存带有检测结果的生成图像
            if dataloader.mode == 'video':
                if vid_path != save_path:  # 新视频
                    vid_path = save_path
                    if isinstance(vid_writer, cv2.VideoWriter):
                        vid_writer.release()  # 释放之前的视频编写器
                    width = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    height = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    fps = vid_cap.get(cv2.CAP_PROP_FPS)
                    vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'avc1'), fps, (width, height))
                vid_writer.write(im0)
            else:
                cv2.imwrite(save_path, im0)

    if save_images and platform == 'darwin':  # macos
        os.system('open ' + output + ' ' + save_path)
        


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--cfg', type=str, default='cfg/yolov3-tiny.cfg', help='cfg文件路径')
    parser.add_argument('--data-cfg', type=str, default='data/ucar.data', help='ucar.data文件路径')
    parser.add_argument('--weights', type=str, default='weights/best.pt', help='权重文件路径')
    parser.add_argument('--images', type=str, default='JPEGImages', help='图片路径')
    parser.add_argument('--img-size', type=int, default=416, help='每个图像维度的大小')
    parser.add_argument('--conf-thres', type=float, default=0.6, help='目标置信度阈值')
    parser.add_argument('--nms-thres', type=float, default=0.65, help='非极大值抑制的iou阈值')
    parser.add_argument('--webcam', type=bool, default=True, help='是否开摄像头进行预测')
    opt = parser.parse_args()
    print(opt)

    rospy.init_node("ucar_camera", anonymous=True)
    camera_topic_name=rospy.get_param('~cam_topic_name',default="/ucar_camera/image_raw")
    cam_pub=rospy.Publisher(camera_topic_name,Image,queue_size=1)
    os.system("rosparam set /rosout/logger_level WARN")

    
    image_temp=Image()                    #创建一个ROS的用于发布图片的Image()消息
    image_temp.header.frame_id = 'opencv' #定义图片header里的id号
    image_temp.height=480        #定义图片高度
    image_temp.width=640          #定义图片宽度
    image_temp.encoding='bgr8'            #图片格式    
    image_temp.is_bigendian=True
    image_temp.step=640*3         #告诉ROS图片每行的大小 28是宽度3是3byte像素（rgb）
   
    with open('/home/ucar/Desktop/ucar/src/fame_data.txt','w') as f:
        pass
    with open('/home/ucar/Desktop/ucar/src/except.txt','w') as f:
        pass
    with open('/home/ucar/Desktop/ucar/src/hhh.txt','w') as f:
        pass
    with open('/home/ucar/Desktop/ucar/src/counter.txt','w') as k:
        k.write("接下来运行的区域是:    E D C B F   \n\n")
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback)
    rospy.wait_for_service('/move_base/make_plan')
    
    # 创建订阅者，订阅/amcl_pose话题
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    # 每隔300毫秒获取一次位置信息
    rospy.Rate(1 / 0.1).sleep()
    
    try:
        with torch.no_grad():
            detect(
                opt.cfg,
                opt.data_cfg,
                opt.weights,
                opt.images,
                img_size=opt.img_size,
                conf_thres=opt.conf_thres,
                nms_thres=opt.nms_thres,
                webcam=opt.webcam
            )
    except Exception as e:
        with open('/home/ucar/Desktop/ucar/src/except.txt','a+') as f:
            f.write(str(e.args)+"\n")
            f.write(traceback.format_exc())