import os

import torch.nn.functional as F

from utils.parse_config import *
from utils.utils import *

ONNX_EXPORT = False

def create_modules(module_defs):
    """
    从module_defs中的模块配置构造层块的模块列表
    """
    hyperparams = module_defs.pop(0)
    output_filters = [int(hyperparams['channels'])]
    module_list = nn.ModuleList()
    yolo_layer_count = 0
    for i, module_def in enumerate(module_defs):
        modules = nn.Sequential()

        if module_def['type'] == 'convolutional':
            bn = int(module_def['batch_normalize'])
            filters = int(module_def['filters'])
            kernel_size = int(module_def['size'])
            pad = (kernel_size - 1) // 2 if int(module_def['pad']) else 0
            modules.add_module('conv_%d' % i, nn.Conv2d(in_channels=output_filters[-1],
                                                        out_channels=filters,
                                                        kernel_size=kernel_size,
                                                        stride=int(module_def['stride']),
                                                        padding=pad,
                                                        bias=not bn))
            if bn:
                modules.add_module('batch_norm_%d' % i, nn.BatchNorm2d(filters))
            if module_def['activation'] == 'leaky':
                modules.add_module('leaky_%d' % i, nn.LeakyReLU(0.1, inplace=True))

        elif module_def['type'] == 'maxpool':
            kernel_size = int(module_def['size'])
            stride = int(module_def['stride'])
            if kernel_size == 2 and stride == 1:
                modules.add_module('_debug_padding_%d' % i, nn.ZeroPad2d((0, 1, 0, 1)))
            maxpool = nn.MaxPool2d(kernel_size=kernel_size, stride=stride, padding=int((kernel_size - 1) // 2))
            modules.add_module('maxpool_%d' % i, maxpool)

        elif module_def['type'] == 'upsample':
            # upsample = nn.Upsample(scale_factor=int(module_def['stride']), mode='nearest')  # 警告：已弃用
            upsample = Upsample(scale_factor=int(module_def['stride']))
            modules.add_module('upsample_%d' % i, upsample)

        elif module_def['type'] == 'route':
            layers = [int(x) for x in module_def['layers'].split(',')]
            filters = sum([output_filters[i + 1 if i > 0 else i] for i in layers])
            modules.add_module('route_%d' % i, EmptyLayer())

        elif module_def['type'] == 'shortcut':
            filters = output_filters[int(module_def['from'])]
            modules.add_module('shortcut_%d' % i, EmptyLayer())

        elif module_def['type'] == 'yolo':
            anchor_idxs = [int(x) for x in module_def['mask'].split(',')]
            # 提取锚点
            anchors = [float(x) for x in module_def['anchors'].split(',')]
            anchors = [(anchors[i], anchors[i + 1]) for i in range(0, len(anchors), 2)]
            anchors = [anchors[i] for i in anchor_idxs]
            nc = int(module_def['classes'])  # 类别数
            img_size = hyperparams['height']
            # 定义检测层
            yolo_layer = YOLOLayer(anchors, nc, img_size, yolo_layer_count, cfg=hyperparams['cfg'])
            modules.add_module('yolo_%d' % i, yolo_layer)
            yolo_layer_count += 1

        # 注册模块列表和输出滤波器数
        module_list.append(modules)
        output_filters.append(filters)

    return hyperparams, module_list


class EmptyLayer(nn.Module):
    """'route'和'shortcut'层的占位符"""

    def __init__(self):
        super(EmptyLayer, self).__init__()

    def forward(self, x):
        return x


class Upsample(nn.Module):
    # 自定义上采样层（nn.Upsample会给出弃用警告消息）

    def __init__(self, scale_factor=1, mode='nearest'):
        super(Upsample, self).__init__()
        self.scale_factor = scale_factor
        self.mode = mode

    def forward(self, x):
        return F.interpolate(x, scale_factor=self.scale_factor, mode=self.mode)


class YOLOLayer(nn.Module):
    def __init__(self, anchors, nc, img_size, yolo_layer, cfg):
        super(YOLOLayer, self).__init__()

        self.anchors = torch.Tensor(anchors)
        self.na = len(anchors)  # 锚点数（3）
        self.nc = nc  # 类别数（80）
        self.nx = 0  # 初始化x网格点数
        self.ny = 0  # 初始化y网格点数

        if ONNX_EXPORT:  # 网格必须在__init__中计算
            stride = [32, 16, 8][yolo_layer]  # 此层的步幅
            if cfg.endswith('yolov3-tiny.cfg'):
                stride *= 2

            ng = (int(img_size[0] / stride), int(img_size[1] / stride))  # 网格点数
            create_grids(self, max(img_size), ng)

    def forward(self, p, img_size, var=None):
        if ONNX_EXPORT:
            bs = 1  # 批量大小
        else:
            bs, ny, nx = p.shape[0], p.shape[-2], p.shape[-1]
            if (self.ny, self.nx) != (ny, nx):
                create_grids(self, img_size, (ny, nx), p.device)

        # p.view(bs, 255, 13, 13) -- > (bs, 3, 13, 13, 85)  # (bs, anchors, grid, grid, classes + xywh)
        p = p.view(bs, self.na, self.nc + 5, self.ny, self.nx).permute(0, 1, 3, 4, 2).contiguous()  # 预测

        if self.training:
            return p

        elif ONNX_EXPORT:
            # 常量不能广播，确保正确的形状！
            ngu = self.ng.repeat((1, self.na * self.nx * self.ny, 1))
            grid_xy = self.grid_xy.repeat((1, self.na, 1, 1, 1)).view((1, -1, 2))
            anchor_wh = self.anchor_wh.repeat((1, 1, self.nx, self.ny, 1)).view((1, -1, 2)) / ngu

            # p = p.view(-1, 5 + self.nc)
            # xy = torch.sigmoid(p[..., 0:2]) + grid_xy[0]  # x, y
            # wh = torch.exp(p[..., 2:4]) * anchor_wh[0]  # width, height
            # p_conf = torch.sigmoid(p[:, 4:5])  # Conf
            # p_cls = F.softmax(p[:, 5:85], 1) * p_conf  # SSD-like conf
            # return torch.cat((xy / ngu[0], wh, p_conf, p_cls), 1).t()

            p = p.view(1, -1, 5 + self.nc)
            xy = torch.sigmoid(p[..., 0:2]) + grid_xy  # x, y
            wh = torch.exp(p[..., 2:4]) * anchor_wh  # width, height
            p_conf = torch.sigmoid(p[..., 4:5])  # Conf
            p_cls = p[..., 5:85]
            # 广播仅在CoreML的第一维上受支持。请参见onnx-coreml/_operators.py
            # p_cls = F.softmax(p_cls, 2) * p_conf  # SSD-like conf
            p_cls = torch.exp(p_cls).permute((2, 1, 0))
            p_cls = p_cls / p_cls.sum(0).unsqueeze(0) * p_conf.permute((2, 1, 0))  # F.softmax() equivalent
            p_cls = p_cls.permute(2, 1, 0)
            return torch.cat((xy / ngu, wh, p_conf, p_cls), 2).squeeze().t()

        else:  # 推理
            io = p.clone()  # 推理输出
            io[..., 0:2] = torch.sigmoid(io[..., 0:2]) + self.grid_xy  # xy
            io[..., 2:4] = torch.exp(io[..., 2:4]) * self.anchor_wh  # wh yolo方法
            # io[..., 2:4] = ((torch.sigmoid(io[..., 2:4]) * 2) ** 3) * self.anchor_wh  # wh power方法
            io[..., 4:] = torch.sigmoid(io[..., 4:])  # p_conf, p_cls
            # io[..., 5:] = F.softmax(io[..., 5:], dim=4)  # p_cls
            io[..., :4] *= self.stride

            # 从[1, 3, 13, 13, 85]重塑为[1, 507, 85]
            return io.view(bs, -1, 5 + self.nc), p

class Darknet(nn.Module):
    """YOLOv3目标检测模型"""

    def __init__(self, cfg, img_size=(416, 416)):
        super(Darknet, self).__init__()

        self.module_defs = parse_model_cfg(cfg)
        self.module_defs[0]['cfg'] = cfg
        self.module_defs[0]['height'] = img_size
        self.hyperparams, self.module_list = create_modules(self.module_defs)
        self.yolo_layers = get_yolo_layers(self)

        # 保存权重时需要写入头信息
        self.header_info = np.zeros(5, dtype=np.int32)  # 前五个是头信息
        self.seen = self.header_info[3]  # 训练期间看到的图像数量

    def forward(self, x, var=None):
        img_size = max(x.shape[-2:])
        layer_outputs = []
        output = []

        for i, (module_def, module) in enumerate(zip(self.module_defs, self.module_list)):
            mtype = module_def['type']
            if mtype in ['convolutional', 'upsample', 'maxpool']:
                x = module(x)
            elif mtype == 'route':
                layer_i = [int(x) for x in module_def['layers'].split(',')]
                if len(layer_i) == 1:
                    x = layer_outputs[layer_i[0]]
                else:
                    x = torch.cat([layer_outputs[i] for i in layer_i], 1)
            elif mtype == 'shortcut':
                layer_i = int(module_def['from'])
                x = layer_outputs[-1] + layer_outputs[layer_i]
            elif mtype == 'yolo':
                x = module[0](x, img_size)
                output.append(x)
            layer_outputs.append(x)

        if self.training:
            return output
        elif ONNX_EXPORT:
            output = torch.cat(output, 1)  # 将3层85 x (507, 2028, 8112)连接成85 x 10647
            return output[5:85].t(), output[:4].t()  # ONNX分数，框
        else:
            io, p = list(zip(*output))  # 推理输出，训练输出
            return torch.cat(io, 1), p

    def fuse(self):
        # 在整个模型中融合Conv2d + BatchNorm2d层
        fused_list = nn.ModuleList()
        for a in list(self.children())[0]:
            for i, b in enumerate(a):
                if isinstance(b, nn.modules.batchnorm.BatchNorm2d):
                    # 将此bn层与前一个conv2d层融合
                    conv = a[i - 1]
                    fused = torch_utils.fuse_conv_and_bn(conv, b)
                    a = nn.Sequential(fused, *list(a.children())[i + 1:])
                    break
            fused_list.append(a)
        self.module_list = fused_list
        # model_info(self)  # yolov3-spp从225层减少到152层


def get_yolo_layers(model):
    a = [module_def['type'] == 'yolo' for module_def in model.module_defs]
    return [i for i, x in enumerate(a) if x]  # [82, 94, 106] for yolov3


def create_grids(self, img_size, ng, device='cuda'):
    ny, nx = ng  # x and y grid size
    self.img_size = img_size
    self.stride = img_size / max(ng)

    # 构建xy偏移量
    yv, xv = torch.meshgrid([torch.arange(ny), torch.arange(nx)])
    self.grid_xy = torch.stack((xv, yv), 2).to(device).float().view((1, 1, ny, nx, 2))

    # 构建wh增益
    self.anchor_vec = self.anchors.to(device) / self.stride
    self.anchor_wh = self.anchor_vec.view(1, self.na, 1, 1, 2).to(device)
    self.ng = torch.Tensor(ng).to(device)
    self.nx = nx
    self.ny = ny


def load_darknet_weights(self, weights, cutoff=-1):
    # 解析并加载存储在'weights'中的权重
    # cutoff: 保存0到cutoff之间的层（如果cutoff = -1，则保存所有层）
    weights_file = weights.split(os.sep)[-1]

    # 如果本地没有可用的权重，则尝试下载权重
    if not os.path.isfile(weights):
        try:
            os.system('wget https://pjreddie.com/media/files/' + weights_file + ' -O ' + weights)
        except IOError:
            print(weights + '未找到。\n请尝试https://drive.google.com/drive/folders/1uxgUBemJVw9wZsdpboYbzUN4bcRhsuAI')

    # 建立截止点
    if weights_file == 'darknet53.conv.74':
        cutoff = 75
    elif weights_file == 'yolov3-tiny.conv.15':
        cutoff = 15

    # 打开权重文件
    with open(weights, 'rb') as f:
        header = np.fromfile(f, dtype=np.int32, count=5)  # 前五个是头信息

        # 保存权重时需要写头信息
        self.header_info = header

        self.seen = header[3]  # 训练期间看到的图像数量
        weights = np.fromfile(f, dtype=np.float32)  # 其余是权重

    ptr = 0
    for i, (module_def, module) in enumerate(zip(self.module_defs[:cutoff], self.module_list[:cutoff])):
        if module_def['type'] == 'convolutional':
            conv_layer = module[0]
            if module_def['batch_normalize']:
                # 加载BN偏差、权重、运行均值和运行方差
                bn_layer = module[1]
                num_b = bn_layer.bias.numel()  # 偏差数量
                # 偏差
                bn_b = torch.from_numpy(weights[ptr:ptr + num_b]).view_as(bn_layer.bias)
                bn_layer.bias.data.copy_(bn_b)
                ptr += num_b
                # 权重
                bn_w = torch.from_numpy(weights[ptr:ptr + num_b]).view_as(bn_layer.weight)
                bn_layer.weight.data.copy_(bn_w)
                ptr += num_b
                # 运行均值
                bn_rm = torch.from_numpy(weights[ptr:ptr + num_b]).view_as(bn_layer.running_mean)
                bn_layer.running_mean.data.copy_(bn_rm)
                ptr += num_b
                # 运行方差
                bn_rv = torch.from_numpy(weights[ptr:ptr + num_b]).view_as(bn_layer.running_var)
                bn_layer.running_var.data.copy_(bn_rv)
                ptr += num_b
            else:
                # 加载卷积偏差
                num_b = conv_layer.bias.numel()
                conv_b = torch.from_numpy(weights[ptr:ptr + num_b]).view_as(conv_layer.bias)
                conv_layer.bias.data.copy_(conv_b)
                ptr += num_b
            # 加载卷积权重
            num_w = conv_layer.weight.numel()
            conv_w = torch.from_numpy(weights[ptr:ptr + num_w]).view_as(conv_layer.weight)
            conv_layer.weight.data.copy_(conv_w)
            ptr += num_w

    return cutoff



def save_weights(self, path='model.weights', cutoff=-1):
    # 将PyTorch模型转换为Darket格式（*.pt转换为*.weights）
    # 注意：如果应用了model.fuse()，则无法使用
    with open(path, 'wb') as f:
        self.header_info[3] = self.seen  # 训练期间看到的图像数量
        self.header_info.tofile(f)

        # 遍历层
        for i, (module_def, module) in enumerate(zip(self.module_defs[:cutoff], self.module_list[:cutoff])):
            if module_def['type'] == 'convolutional':
                conv_layer = module[0]
                # 如果有批量归一化，则先加载bn
                if module_def['batch_normalize']:
                    bn_layer = module[1]
                    bn_layer.bias.data.cpu().numpy().tofile(f)
                    bn_layer.weight.data.cpu().numpy().tofile(f)
                    bn_layer.running_mean.data.cpu().numpy().tofile(f)
                    bn_layer.running_var.data.cpu().numpy().tofile(f)
                # 加载卷积偏差
                else:
                    conv_layer.bias.data.cpu().numpy().tofile(f)
                # 加载卷积权重
                conv_layer.weight.data.cpu().numpy().tofile(f)


def convert(cfg='cfg/yolov3-spp.cfg', weights='weights/yolov3-spp.weights'):
    # 根据扩展名（即*.weights转换为*.pt，反之亦然）在PyTorch和Darknet格式之间进行转换
    # from models import *; convert('cfg/yolov3-spp.cfg', 'weights/yolov3-spp.weights')

    # 初始化模型
    model = Darknet(cfg)

    # 加载权重并保存
    if weights.endswith('.pt'):  # 如果是PyTorch格式
        model.load_state_dict(torch.load(weights, map_location='cuda')['model'])
        save_weights(model, path='converted.weights', cutoff=-1)
        print("成功：将'%s'转换为'converted.weights'" % weights)

    elif weights.endswith('.weights'):  # 如果是Darknet格式
        _ = load_darknet_weights(model, weights)
        chkpt = {'epoch': -1, 'best_loss': None, 'model': model.state_dict(), 'optimizer': None}
        torch.save(chkpt, 'converted.pt')
        print("成功：将'%s'转换为'converted.pt'" % weights)

    else:
        print('错误：不支持的扩展名。')

