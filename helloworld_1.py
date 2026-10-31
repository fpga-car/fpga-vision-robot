import sensor, image, lcd, time
from maix import KPU
from machine import UART
from fpioa_manager import fm
import gc, sys

labels = ['红色水瓶']
# MaixHub yolo2-tiny 模型: 输入224x224, 输出region 7x7
anchor = (1.56, 2.47, 2.47, 3.91, 4.25, 5.91, 4.91, 3.78, 3.06, 4.0)

# 串口引脚: P7=IO7(TX)发FPGA, P6=IO6(RX)
fm.register(7, fm.fpioa.UART1_TX, force=True)
fm.register(6, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 115200, 8, 0, 0, timeout=1000, read_buf_len=256)

lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((224, 224))   # 裁剪到模型输入尺寸
sensor.skip_frames(time=1000)

net_w, net_h = 224, 224
od_img = image.Image(size=(net_w, net_h))   # 模型运算用的独立图像

kpu = KPU()
print('load model...')
kpu.load_kmodel(0x300000, 1903016)   # 关键1: 从Flash读取, 需要"地址 + 模型大小"
kpu.init_yolo2(anchor, anchor_num=5,
               img_w=224, img_h=224,
               net_w=224, net_h=224,
               layer_w=7, layer_h=7,       # 关键2: region层 224/32=7
               threshold=0.5, nms_value=0.3, classes=1)
print('ok')

while True:
    img = sensor.snapshot()                 # 224x224 原始图(给LCD显示)
    od_img.draw_image(img, 0, 0)            # 拷入运算图
    od_img.pix_to_ai()                      # 转成KPU需要的r8g8b8
    kpu.run_with_output(od_img)             # 关键3: 推理
    dect = kpu.regionlayer_yolo2()          # 关键3: yolo2后处理

    msg = ''
    if len(dect) > 0:
        for d in dect:
            x, y, w, h = d[0], d[1], d[2], d[3]
            c = d[4]
            conf = d[5] if len(d) > 5 else 0
            img.draw_rectangle(x, y, w, h, color=(255, 0, 0))
            img.draw_string(x, y, '%s %.2f' % (labels[c], conf), color=(255, 0, 0), scale=1.5)
            msg += '%d:%d:%d:%d:%d:%.2f:%s\n' % (x, y, w, h, c, conf, labels[c])
        uart.write(msg.encode())            # 从P7引脚发给FPGA
        print(msg, end='')

    lcd.display(img)
    gc.collect()
