#encoding: UTF-8
#!/usr/bin/env python2

class GrabParams(object):
#基本参数
	ratio = 0.214
	#                   [x   y    z   俯仰  横滚  航向 ]	
	coords_high_right = [94, -63, 310, -90, 50, -95]
	coords_high_left  = [94,  63, 310, -90, 50, -93]

	coords_low_right  = [176, -61, 225, -75, 45, -83]
	coords_low_left   = [176,  61, 225, -78, 57, -85]
	grab_direct = "right"
	if grab_direct == "right":
		y_bias = 5
		x_bias = 40
	debug = True #True         
	ONNX_MODEL = '/home/robuster/beetle_ai/scripts/beetle_obj.onnx'
	IMG_SIZE = 640
	done = False
	usb_dev = "/dev/arm" 
	baudrate = 115200

#需要调试的参数
	cap_num = 2   #摄像头编号

	set_down_direction = "right"#放置方向，位于车左还是右
	grab_low_right    = 300     #低左   的机械臂高度      加+高   减-低  以5为单位调节   
	grab_low_left     = 295     #低右   的机械臂高度      同上
	grab_high_right   = 350     #高右   的机械臂高度  
	grab_high_left    = 350		#高左   的机械臂高度  
#夹取机械臂俯仰角调节
	pitch_low_right    = 17     #低左   的机械臂高度      加+向上   减-向下  以2左右为单位调节   
	pitch_low_left     = 5      #低右   的机械臂高度      同上
	pitch_high_right   = 5      #高右   的机械臂高度  
	pitch_high_left    = 7		#高左   的机械臂高度  
#夹取机械臂横滚角调节
	roll_low_right    = -5      #低左   的机械臂高度      加+向右   减-向左  以2左右为单位调节   
	roll_low_left     = 0       #低右   的机械臂高度      同上
	roll_high_right   = -5      #高右   的机械臂高度  
	roll_high_left    = 5		#高左   的机械臂高度  


#测距误差抵消     +x 多前进x cm ， -x 少前进x cm 
	move_power_high_right  = 2       #高右
	move_power_high_left   = 3.5     #高左
	move_power_low_right   = -1.5    #低右
	move_power_low_left    = -0.5    #低左

#物块放置测距误差，基本不用动
	go_diff = 0
#判断识别物是否是目标，  对应数字，改detect_target
	classes = ("bird", "clock", "cat","banana ","apple ")
	#             0       1       2      3        4
	detect_target = 1

grabParams = GrabParams()
