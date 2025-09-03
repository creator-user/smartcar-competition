import argparse
import time
from datetime import datetime
import os
from sys import platform
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import PIL
from models import *
from utils.datasets import *
from utils.utils import *

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import traceback
queue = set()

class CategoryCounter:
    def __init__(self):
        self.counter = {}
    
    def add_category(self, category):
        self.counter[category] = self.counter.get(category, 0) + 1
    
    def get_sorted_categories(self):
        sorted_categories = sorted(self.counter.items(), key=lambda x: x[1], reverse=True)
        return sorted_categories

start_time=0
is_achieve=0

def rotate_callback(msg):
    global is_achieve
    if msg.data=="A":
        is_achieve=1

        

counter = CategoryCounter()
# 识别E区作物，当 x>2.226 时，开始识别，当 y<-2.376 时停止识别
# 识别D区作物，当 y<-3.418 时，开始识别，当 y>-3.418 时停止识别
# 识别C区作物，当 y>-2.498 时，开始识别，当 y<-2.498 时停止识别
# 识别B区作物，当 y<-3.514 时，开始识别，当 y>-3.514 时停止识别
# 识别F区果实，当 y<-0.557 and x<1.626 时，开始识别，当 y>-0.557 时停止识别

scan_place=[["position.x>2.226","position.y<-2.376"],["position.y<-3.418","position.y>-3.418"],["position.y>-2.498","position.y<-2.498"],["position.y<-3.514","position.y>-3.514"],[["position.y<-0.557 ","position.x<1.626"],"position.y>-0.557"]]

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
            with open('/home/ucar/Desktop/ucar/src/1.txt','a+') as f:
                f.write("1\n")
            place_flag=1
        elif (place_recode<4 and eval(scan_place[place_recode][1]) and place_flag==1):
            with open('/home/ucar/Desktop/ucar/src/1.txt','a+') as f:
                f.write("2\n")
            place_flag=2
            place_recode+=1
        elif (place_recode==4 and place_flag==0 and eval(scan_place[place_recode][0][0]) and eval(scan_place[place_recode][0][1]) ):
            with open('/home/ucar/Desktop/ucar/src/1.txt','a+') as f:
                f.write("3\n")
            place_flag=1
        elif (place_recode==4 and place_flag==1 and eval(scan_place[place_recode][1]) ):
            with open('/home/ucar/Desktop/ucar/src/1.txt','a+') as f:
                f.write("4\n")
            place_flag=2
            place_recode+=1

goal_index=1


final_nzw=set()
#西瓜，黄瓜，玉米
gs_num={"西瓜":[1,1,1,1],"黄瓜":[1,3,2,2],"玉米":[1,2,2,1]}
start_time=0
def callback(data):
    global goal_index
    global is_achieve
    if data.status.status == 3:
        
        if goal_index < 11:
            # 存储数据,拿多个数据
            if(goal_index==6 or goal_index==10):
                goal_index+=1
                return
            is_achieve=1
            goal_index+=1
        else:
            # 播报
            with open('/home/ucar/Desktop/ucar/src/test_data.txt','a+') as k:
                k.write(str(queue)+"\n")
            with open('/home/ucar/Desktop/ucar/src/fame_data.txt','a+') as f:
                counts = {"玉米":0,"黄瓜":0,"西瓜":0,"小麦":0,"水稻":0}
                if place_recode==5:
                    for item in queue:
                        category = item[:2]  # 提取前两个字作为分类
                        if category=="其他":
                            continue
                        if item[2:4]=="果实":
                            counts[category] += gs_num[category][int(item[-1])-1]
                    max_category = max(counts, key=counts.get)  # 获取数量最高的分类
                    max_count = counts[max_category]  # 获取数量最高的数量
                    f.write(str(max_category)+"\n"+str(max_count)+"\n")
            os.system("sh /home/ucar/Desktop/ucar/src/AIsound/bin/result.sh")


def save_image_to_folder(image_data, folder_path, file_name):
    # 指定保存路径和文件名
    save_path = folder_path+"/"+file_name+".png"

    # 将im0保存为图像文件
    im = PIL.Image.fromarray(cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB))
    im.save(save_path)


image_folder=["E","D","C","B","F","F"]
cnt=0

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
    global queue
    global cnt
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
    photos_flag=0

    for i, (path, img, im0, vid_cap) in enumerate(dataloader):
        t = time.time()
        save_path = str(Path(output) / Path(path).name)

        # 获取检测结果
        img = torch.from_numpy(img).unsqueeze(0).to(device)
        pred, _ = model(img)
        det = non_max_suppression(pred, conf_thres, nms_thres)[0]
        
        if is_achieve==1:
            is_achieve=0
            cnt=3

            with open('/home/ucar/Desktop/ucar/src/1.txt','a+') as f:
                f.write("test\n")

        if cnt>0:
            photos_flag=1
            cnt-=1
        if det is not None and len(det) > 0:
            # 将416的框缩放到真实图像大小
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

            # 绘制检测结果的边界框和标签
            for *xyxy, conf, cls_conf, cls in det:
                if save_txt:  # 写入文件
                    with open(save_path + '.txt', 'a') as file:
                        file.write(('%g ' * 6 + '\n') % (*xyxy, cls, conf))
                        
                # 将边界框添加到图像中
                label = '%s %.2f' % (classes[int(cls)], conf)
                im0 = plot_one_box(xyxy, im0, label=label, color=colors[int(cls)])
                
            # 记录每张图片的数据
            final_nzw=set()
            
            for c in det[:, -1].unique():
                final_nzw.add(classes[int(c)])
                counter.add_category(str(classes[int(c)]))
            if photos_flag==1:
                queue=queue.union(final_nzw)
                save_image_to_folder(im0,"/home/ucar/Desktop/ucar/src/ucar_camera/src/image_result/"+str(image_folder[place_recode]),str(int(time.time()*1000000)%1000000))

            if(place_flag==2):
                with open('/home/ucar/Desktop/ucar/src/test_data.txt','a+') as k:
                    k.write(str(queue)+"\n")
                with open('/home/ucar/Desktop/ucar/src/fame_data.txt','a+') as f:
                    counts = {"玉米":0,"黄瓜":0,"西瓜":0,"小麦":0,"水稻":0}
                    if  place_recode<5:
                        for item in queue:
                            category = item[:2]  # 提取前两个字作为分类
                            if item[2:4]=="作物":
                                counts[category] += 1
                        max_category = max(counts, key=counts.get)  # 获取数量最高的分类
                        max_count = counts[max_category]  # 获取数量最高的数量
                        f.write(str(max_category)+"\n")
                        
                with open('/home/ucar/Desktop/ucar/src/counter.txt','a+') as k:
                    k.write(f"=================================={place_recode}\n")
                    #sorted_categories = counter.get_sorted_categories()
                    #k.write(str(sorted_categories))
                    k.write(str(queue))
                    k.write("\n\n")
                place_flag=0
                counter=CategoryCounter()
                queue=set()
        photos_flag=0
        if webcam:  # 显示实时网络摄像头
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
        
def delete_files(folder_path):
    for root, dirs, files in os.walk(folder_path):
        for file in files:
            file_path = os.path.join(root, file)
            os.remove(file_path)
            print(f"Deleted file: {file_path}")



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
    # 指定要删除文件的文件夹路径
    folder_path = './image_result'

    # 调用函数删除文件
    delete_files(folder_path)
    rospy.init_node("ucar_camera", anonymous=True)
    camera_topic_name=rospy.get_param('~cam_topic_name',default="/ucar_camera/image_raw")
    cam_pub=rospy.Publisher(camera_topic_name,Image,queue_size=1)

    image_temp=Image()                    #创建一个ROS的用于发布图片的Image()消息
    image_temp.header.frame_id = 'opencv' #定义图片header里的id号
    image_temp.height=480        #定义图片高度
    image_temp.width=640          #定义图片宽度
    image_temp.encoding='bgr8'            #图片格式    
    image_temp.is_bigendian=True
    image_temp.step=640*3         #告诉ROS图片每行的大小 28是宽度3是3byte像素（rgb）
   
    with open('/home/ucar/Desktop/ucar/src/fame_data.txt','w') as f:
        pass
    with open('/home/ucar/Desktop/ucar/src/test_data.txt','w') as f:
        pass
    with open('/home/ucar/Desktop/ucar/src/1.txt','w') as f:
        pass
    with open('/home/ucar/Desktop/ucar/src/except.txt','w') as f:
        pass
    with open('/home/ucar/Desktop/ucar/src/counter.txt','w') as k:
        k.write("接下来运行的区域是:    E D C B F   \n\n")
        
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback)
    rospy.wait_for_service('/move_base/make_plan')
    
    # 创建订阅者，订阅/amcl_pose话题
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    # 每隔300毫秒获取一次位置信息
    rospy.Rate(1 / 0.1).sleep()

    # 创建订阅者，订阅/rotate话题
    rospy.Subscriber('/rotate', String, rotate_callback)
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
