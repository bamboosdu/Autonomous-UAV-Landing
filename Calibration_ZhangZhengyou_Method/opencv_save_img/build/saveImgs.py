import cv2

cap = cv2.VideoCapture(0) #计算机自带的摄像头为0，外部设备为1
width=800
height=600
cap.set(cv2.CAP_PROP_FRAME_WIDTH,width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
i=0
while(1):
    ret,frame = cap.read()  #ret:True/False,代表有没有读到图片  frame:当前截取一帧的图片
    #cv2.imshow("capture",frame) 
    print("got the picture:",ret)   
    if (cv2.waitKey(0) & 0xFF) == ord('s'): #不断刷新图像，这里是1ms 返回值为当前键盘按键值
        #gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) #RGB图像转为单通道的灰度图像
        #gray = cv2.resize(gray,(320,240)) #图像大小为320*240
        cv2.imwrite('./imgs/%04d.jpg'%i,frame)
        i += 1
        print("Yes, I have been saved")
    if (cv2.waitKey(0) & 0xFF) == ord('q'):
        break  
    
cap.release()
cv2.destroyAllWindows()

