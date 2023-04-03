
import cv2
import numpy as np

cap = cv2.VideoCapture("drive.mp4")
#cap = cv2.VideoCapture(0)


def changePerspective(frame):

    width, height = frame.shape[1], frame.shape[0]
    
    # change perspective


    # perspective points
    pts1 = np.float32([[560, 440],[710, 440],[200, 640],[1000, 640]])
    pts2 = np.float32([[0,0],[width,0],[0,height],[width,height]])
    # end perspective points
    
    # perspective points draw circle
    pts1_ = np.array([[560, 440],[710, 440],[200, 640],[1000, 640]])
    pts2_ = np.array([[0,0],[width,0],[0,height],[width,height]])

    matrix = cv2.getPerspectiveTransform(pts1,pts2)
    birds_eye = cv2.warpPerspective(frame, matrix, (width, height))
    return birds_eye



def selectRegion(cap):
    ret,frame = cap.read()
    frame = cv2.resize(frame,(640,480))
    roi = cv2.selectROI(frame)
    return roi



def region(frame):
    frame = cv2.resize(frame,(640,480))
    roi = frame[317:399,239:389]
    roi = frame = cv2.resize(roi,(640,480))
    return roi

def region_of_intersest(frame):
    frame = cv2.resize(frame,(640,480))
    height = frame.shape[0]
    polygons = np.array([
        [(150,height),(500,height),(292,231)]
    ])

    mask = np.zeros_like(frame)
    cv2.fillPoly(frame,polygons,255)

    masked_frame = cv2.bitwise_and(frame,frame,mask=mask)
    pts = cv2.findNonZero(mask)

    x, y, w, h = cv2.boundingRect(pts)
    crop = masked_frame[y:y+h, x:x+w]


    crop = cv2.resize(crop,(640,480))
    return crop



def cannyFilter(frame):
    grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # smooth image
    kernel_size = 5
    blur = cv2.GaussianBlur(grayscale, (kernel_size, kernel_size), 0)
    # canny edge dedection
    low_t = 50
    high_t = 95
    edges = cv2.Canny(blur, low_t, high_t)
    return edges



def createLine(edges,frame):
    # detect line with houg line transform
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, maxLineGap=50)        
    # for i in lines:
    #     x1,x2,y1,y2=i[0]
    #     cv2.line(frame,(x1,x2),(y1,y2),(0,255,0),3)  
    return lines

def steering(lines,frame):
    for line in lines:
        x1,y1,x2,y2 = line[0]
    
        # classification right and left line
        if x1 < 640 or x2 < 640:
            x1_left = x1
            x2_left = x2
            y1_left = y1
            y2_left = y2
        
        elif x1 > 640 or x2 > 640:
            x1_right = x1
            x2_right = x2
            y1_right = y1
            y2_right = y2

            try:
                # calculate middle points
                x1_mid = int((x1_right + x1_left)/2)
                x2_mid = int((x2_right + x2_left)/2)
                cv2.line(frame, (640, 300), (x2_mid, 420), (0, 255, 0), 2)
                
                
                # create straight pipe line in middle of the frame
                x_1, x_2 = 640, 640
                y_1, y_2 = 300, 420
                cv2.line(frame, (x_1,y_1), (x_2, y_2), (0, 0, 255), 2)
            
                
                # calculate 3 point beetween angle
                point_1 = [x_1, y_1]
                point_2 = [x_2, y_2]
                point_3 = [x2_mid, 420]
                
                radian = np.arctan2(point_2[1] - point_1[1], point_2[0] - point_1[0]) - np.arctan2(point_3[1] - point_1[1], point_3[0] - point_1[0])
                angle = (radian *180 / np.pi)

                if angle < -30:
                    print('left')
                elif angle > 25:
                    print('right')
                elif angle > -25 and angle < 25:
                    print('straight')
                    continue
            
            except NameError:
                continue
    else:
        pass
        
    frame = cv2.resize(frame,(640,480))
    return frame




def main():

    while 1:

        ret,frame = cap.read()

        if ret:

            birds_eye = changePerspective(frame) 

            edges = cannyFilter(birds_eye)
            # lines = createLine(edges,birds_eye)   

            # birds_eye = steering(lines,birds_eye)
            original_perspective = cv2.resize(frame, (640,480))
            
            cv2.imshow("original_eye", original_perspective)
            cv2.imshow("birds_eye", birds_eye)

        
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
        
        elif ret == False:
            break
    cap.release()
    cv2.destroyAllWindows()  



main()
