# YOLOv5 🚀 by Ultralytics, AGPL-3.0 license
"""
Run YOLOv5 detection inference on images, videos, directories, globs, YouTube, webcam, streams, etc.

Usage - sources:
    $ python detect.py --weights yolov5s.pt --source 0                               # webcam
                                                     img.jpg                         # image
                                                     vid.mp4                         # video
                                                     screen                          # screenshot
                                                     path/                           # directory
                                                     list.txt                        # list of images
                                                     list.streams                    # list of streams
                                                     'path/*.jpg'                    # glob
                                                     'https://youtu.be/Zgi9g1ksQHc'  # YouTube
                                                     'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python detect.py --weights yolov5s.pt                 # PyTorch
                                 yolov5s.torchscript        # TorchScript
                                 yolov5s.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                 yolov5s_openvino_model     # OpenVINO
                                 yolov5s.engine             # TensorRT
                                 yolov5s.mlmodel            # CoreML (macOS-only)
                                 yolov5s_saved_model        # TensorFlow SavedModel
                                 yolov5s.pb                 # TensorFlow GraphDef
                                 yolov5s.tflite             # TensorFlow Lite
                                 yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
                                 yolov5s_paddle_model       # PaddlePaddle
"""

import argparse
import os
import platform
import sys
from pathlib import Path
import rospy
import torch
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# import sys
# sys.path.append('/home/jaewoong/catkin_ws/devel/lib/python3/dist-packages')
from get_axis.msg import Yolodetect


FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, smart_inference_mode


rospy.init_node('center_point_publisher', anonymous=True)
# center = rospy.Publisher('center_point', Point, queue_size=10)
# length_pub = rospy.Publisher('bounding_box_length', Point, queue_size=10)
object_detect_pub = rospy.Publisher('object_detect', Yolodetect, queue_size = 10)

@smart_inference_mode()
def run(
        weights=ROOT / 'yolov5l.pt',  # model path or triton URL
        source=ROOT / 'data/images',  # file/dir/URL/glob/screen/0(webcam)
        data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        # conf_thres=0.25,  # confidence threshold
        conf_thres=0.9,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        save_txt=False,  # save results to *.txt
        save_conf=False,  # save confidences in --save-txt labels
        save_crop=False,  # save cropped prediction boxes
        nosave=False,  # do not save images/videos
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        update=False,  # update all models
        project=ROOT / 'runs/detect',  # save results to project/name
        name='exp',  # save results to project/name
        exist_ok=False,  # existing project/name ok, do not increment
        line_thickness=3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
        vid_stride=1,  # video frame-rate stride
):
    try:
        while not rospy.is_shutdown():
            source = str(source)
            save_img = not nosave and not source.endswith('.txt')  # save inference images
            is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS) ##source가 이미지인지 비디오 파일인지 확인한다.
            is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://')) ## source가 url인지 확인한다.
            webcam = source.isnumeric() or source.endswith('.streams') or (is_url and not is_file) ## source가 웹캠 또느 비디오 스트림인지 확인한다.
            screenshot = source.lower().startswith('screen') ## source가 스크린샷인지 확인.
            if is_url and is_file:
                source = check_file(source)  # download

            # Directories
            save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
            (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

            # Load model
            device = select_device(device)
            model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
            stride, names, pt = model.stride, model.names, model.pt
            imgsz = check_img_size(imgsz, s=stride)  # check image size 이미지 사이즈 체크 - 640,640
            # print("imgsz",imgsz)
            # Dataloader
            bs = 1  # batch_size
            if webcam: ## 웹캠을 사용하면
                view_img = check_imshow(warn=True) #check_imshow에서 이미지를 디스플레이 할 수 있는지에 대한 검사를 하는 코드이다. 디스플레이 할 수 있으면 view_imgs 값은 True 값이 된다.
                dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
                # print("dataset", dataset)
                bs = len(dataset)
            elif screenshot:
                dataset = LoadScreenshots(source, img_size=imgsz, stride=stride, auto=pt)
            else:
                dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
            vid_path, vid_writer = [None] * bs, [None] * bs
            # print('webcam', webcam)
            # Run inference
            model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup
            seen, windows, dt = 0, [], (Profile(), Profile(), Profile())
            for path, im, im0s, vid_cap, s in dataset:                       ## im = yolo 모델에 맞게 해상도가 바뀐 이미지 - 640x640, im0 = realsense 카메라 해상도의 이미지 - 640x480
                # print(f"Image shape: {im.shape}")
                with dt[0]:
                    im = torch.from_numpy(im).to(model.device) # 이미지를 PyTorch 텐서로 변환하고, 디바이스에 올림
                    im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
                    im /= 255  # 0 - 255 범위를 0.0 - 1.0 범위로 변환
                    if len(im.shape) == 3:
                        im = im[None]  # expand for batch dim

                # Inference
                with dt[1]:
                    visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
                    pred = model(im, augment=augment, visualize=visualize)

                # NMS
                with dt[2]:
                    pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

                # Second-stage classifier (optional)
                # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

                # Process predictions
                for i, det in enumerate(pred):  # per image
                    seen += 1
                    if webcam:  # batch_size >= 1

                        p, im0, frame = path[i], im0s[i].copy(), dataset.count
                        s += f'{i}: '
                    else:
                        p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)

                    p = Path(p)  # to Path
                    save_path = str(save_dir / p.name)  # im.jpg
                    txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # im.txt
                    s += '%gx%g ' % im.shape[2:]  # print string 480x640 출력
                    # print("im.shape[2:]", im.shape[2:])
                    # print("SSSs", s)
                    gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
                    imc = im0.copy() if save_crop else im0  # for save_crop
                    annotator = Annotator(im0, line_width=line_thickness, example=str(names))
                    if len(det):
                        # Rescale boxes from img_size to im0 size
                        det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                        # Print results
                        for c in det[:, 5].unique():
                            n = (det[:, 5] == c).sum()  # detections per class
                            s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string
                        # detected_objects = []
                        # # Write results
                        # for *xyxy, conf, cls in reversed(det):
                        #     x1, y1, x2, y2 = xyxy ##
                        #     center_x = (x1+x2)/2 ##
                        #     center_y = (y1+y2)/2 ##
                        #     length_x = x2 - x1
                        #     length_y = y2 - y1
                        #     object_detect = Yolodetect()
                        #     object_detect.center.x = float(center_x)
                        #     object_detect.center.y = float(center_y)
                        #     object_detect.length.x = float(length_x)
                        #     object_detect.length.y = float(length_y)
                        #     object_detect.name = str(names[int(c)])
                        #     object_detect.num = int(len(det))
                        #     # print(len(det))

                        #     # point = Point()
                        #     # point.x = center_x
                        #     # point.y = center_y
                        #     # length = Point()
                        #     # length.x = length_x
                        #     # length.y = length_y

                        #     # Publish the Point message
                        #     # center.publish(point)
                        #     # length_pub.publish(length)
                        #     object_detect_pub.publish(object_detect)
                        #     # depth = depth_frame[int(center_y.cpu().numpy()), int(center_x.cpu().numpy())]

                        #     # print(f"Depth at center: {depth}")
                        #     print(f"Center coordinates: ({center_x}, {center_y})")
                        #     # print(dataset.depth_info.get_distance(center_x, center_y))
                        detected_objects = []

                        # 객체 감지 정보 처리
                        for *xyxy, conf, cls in reversed(det):
                            x1, y1, x2, y2 = xyxy
                            center_x = (x1 + x2) / 2
                            center_y = (y1 + y2) / 2
                            length_x_1 = x1
                            length_y_1 = y1
                            length_x_2 = x2
                            length_y_2 = y2

                            detected_objects.append({
                                'center_x': float(center_x),
                                'center_y': float(center_y),
                                'length_x_1': float(length_x_1),
                                'length_y_1': float(length_y_1),
                                'length_x_2': float(length_x_2),
                                'length_y_2': float(length_y_2),
                                'name': str(names[int(cls)]),
                                'num': int(len(det)),
                                'xyxy': xyxy, # 추가: 후속 처리에 사용될 xyxy 정보
                                'cls': cls, # 추가: 후속 처리에 사용될 cls 정보
                                'conf': conf # 추가: 후속 처리에 사용될 conf 정보
                            })

                        for obj in detected_objects:
                            object_detect = Yolodetect()
                            object_detect.center.x = obj['center_x']
                            object_detect.center.y = obj['center_y']
                            object_detect.length_1.x = obj['length_x_1']
                            object_detect.length_1.y = obj['length_y_1']
                            object_detect.length_2.x = obj['length_x_2']
                            object_detect.length_2.y = obj['length_y_2']
                            object_detect.name = obj['name']
                            object_detect.num = obj['num']
                            object_detect_pub.publish(object_detect)

                            xyxy, cls, conf = obj['xyxy'], obj['cls'], obj['conf'] # 추가
                            if save_txt:  # Write to file
                                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                                line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                                with open(f'{txt_path}.txt', 'a') as f:
                                    f.write(('%g ' * len(line)).rstrip() % line + '\n')

                            if save_img or save_crop or view_img:  # Add bbox to image
                                c = int(cls)  # integer class
                                label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                                annotator.box_label(xyxy, label, color=colors(c, True))
                            if save_crop:
                                save_one_box(xyxy, imc, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)

                    # Stream results
                    im0 = annotator.result()
                    if view_img: ##위에서 디스플레이 검사를 한다면 이 코드가 실행됨. view_img = True
                        if platform.system() == 'Linux' and p not in windows:
                            windows.append(p)
                            cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                            cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
                        cv2.imshow(str(p), im0)
                        cv2.waitKey(1)  # 1 millisecond

                        # cv2.namedWindow('Depth', cv2.WINDOW_NORMAL)
                        # cv2.imshow('Depth', depth_frame)
                        # cv2.waitKey(1)


                    # Save results (image with detections)
                    if save_img:
                        if dataset.mode == 'image':
                            cv2.imwrite(save_path, im0)
                        else:  # 'video' or 'stream'
                            if vid_path[i] != save_path:  # new video
                                vid_path[i] = save_path
                                if isinstance(vid_writer[i], cv2.VideoWriter):
                                    vid_writer[i].release()  # release previous video writer
                                if vid_cap:  # video
                                    fps = vid_cap.get(cv2.CAP_PROP_FPS)
                                    w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                                    h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                                else:  # stream
                                    fps, w, h = 30, im0.shape[1], im0.shape[0]
                                save_path = str(Path(save_path).with_suffix('.mp4'))  # force *.mp4 suffix on results videos
                                vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                            vid_writer[i].write(im0)
                # Print time (inference-only)
                LOGGER.info(f"{s}{'' if len(det) else '(no detections), '}{dt[1].dt * 1E3:.1f}ms")

            # Print results
            print("seen", seen)
            t = tuple(x.t / seen * 1E3 for x in dt)  # speeds per image
            LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
            if save_txt or save_img:
                s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
                LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
            if update:
                strip_optimizer(weights[0])  # update model (to fix SourceChangeWarning)
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Shutting down")
def parse_opt():
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'yolov5s.pt', help='model path or triton URL')
        parser.add_argument('--source', type=str, default=ROOT / 'data/images', help='file/dir/URL/glob/screen/0(webcam)')
        parser.add_argument('--data', type=str, default=ROOT / 'data/coco128.yaml', help='(optional) dataset.yaml path')
        parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
        parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
        parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
        parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
        parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
        parser.add_argument('--view-img', action='store_true', help='show results')
        parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
        parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
        parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
        parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
        parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
        parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
        parser.add_argument('--augment', action='store_true', help='augmented inference')
        parser.add_argument('--visualize', action='store_true', help='visualize features')
        parser.add_argument('--update', action='store_true', help='update all models')
        parser.add_argument('--project', default=ROOT / 'runs/detect', help='save results to project/name')
        parser.add_argument('--name', default='exp', help='save results to project/name')
        parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
        parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
        parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
        parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
        parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
        parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
        parser.add_argument('--vid-stride', type=int, default=1, help='video frame-rate stride')
        opt = parser.parse_args()
        opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
        print_args(vars(opt))
        return opt
    except KeyboardInterrupt:
            return

def main(opt):
    try:
        check_requirements(ROOT / 'requirements.txt', exclude=('tensorboard', 'thop'))
        run(**vars(opt))
    except KeyboardInterrupt:
            return
if __name__ == '__main__':
    # realsense_camera()
    opt = parse_opt()
    main(opt)




# def image_callback(msg):
#     bridge = CvBridge()
#     current_image = bridge.imgmsg_to_cv2(msg, "bgr8")
#     cv2.imshow('Image Stream', current_image)
#     cv2.waitKey(1)


# def realsense_camera():
#     rospy.init_node('matrix_calculation_server')
#     rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
#     rospy.spin()