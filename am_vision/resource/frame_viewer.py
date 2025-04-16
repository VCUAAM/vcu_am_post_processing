from dcam560.Vzense_api_560 import *

def main():

    #Defining arrays for user display of controls
    datamodes = ["Depth and RGB","IR and RGB","Depth, IR, and RGB","WDR"]
    ranges = ["Near","Mid","Far","XFar"]
    resos = ["640x480","1600x1200","800x600"]

    #Defining camera object
    camera = VzenseTofCam()

    #Initializing camera and configuring image settings
    device_info = camera.init()
    camera.set_depth_range()
    #camera.set_mapper(Sensor.RGB,True)
    #camera.set_RGB_distortion_correction(True)
    camera.set_depth_distortion_correction(False)
    camera.set_compute_depth_correction(True)
    depth_max, value_min, value_max = camera.get_measuring_range()

    #Defining logic variables for control of image and inputs
    ds,ms,rs = False,False,False
    dw,rw,iw = False,False,False

    #Configuring user display of controls
    print("/**********************************************************************/")
    print("M/m: Change data mode")
    print("D/d: Change depth range")
    print("R/r: Change RGB resolution")
    print("Esc: Exit program ")
    print("/**********************************************************************/")

    while True:
        try:
            #Read frame 
            frameready = camera.read_frame()

            #Generating depth image if the depth datamode is selected
            if frameready and frameready.depth:
                dw = True      
                depthframe = camera.get_frame(Frame.Depth)
                depth = camera.gen_image(depthframe, Frame.Depth)

                #Window configuration for images
                cv2.namedWindow('Depth Image', cv2.WINDOW_KEEPRATIO)
                cv2.imshow("Depth Image", depth)

                depe = 0

            #If there is no depth frame 10 times, destroy the window
            elif dw == True:
                depe += 1
                if depe > 10:
                    dw = False
                    cv2.destroyWindow("Depth Image")

            #Generating IR image if the IR datamode is selected
            if frameready and frameready.ir:
                iw = True
                irframe = camera.get_frame(Frame.IR)
                ir = camera.gen_image(irframe, Frame.IR)
                #irC = cv2.applyColorMap(ir,cv2.COLORMAP_RAINBOW)
                #Window configuration for images
                cv2.namedWindow('IR Image', cv2.WINDOW_KEEPRATIO)
                cv2.imshow("IR Image", ir)

                ire = 0
        
            #If there is no IR frame 10 times, destroy the window
            elif iw == True:
                ire += 1
                if ire > 10:
                    iw = False
                    cv2.destroyWindow("IR Image")

            #Generating RGB and polygon image if the RGB datamode is selected 
            if frameready and frameready.rgb:
                rw = True      
                rgbframe = camera.get_frame(Frame.RGB)
                rgb = camera.gen_image(rgbframe,Frame.RGB)

                #Image processing to get a simple polygon of largest contour in view
                '''
                img_gray = cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY)
                ret,thresh = cv2.threshold(img_gray,160,255,cv2.THRESH_BINARY)
                polcont = np.ones(thresh.shape[:2], dtype="uint8") * 255
                contours,hierarchy = cv2.findContours(thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
                max_cont = max(contours, key=cv2.contourArea)
                eps = .01*cv2.arcLength(max_cont,True)
                approx = cv2.approxPolyDP(max_cont,eps,True)
                cv2.drawContours(polcont,[approx],-1,(0,0,0),2)
                '''
                #Window configuration for images
                #cv2.namedWindow('Polycon View', cv2.WINDOW_KEEPRATIO)
                cv2.namedWindow('RGB Image', cv2.WINDOW_NORMAL)
                #cv2.imshow("Polycon View", polcont)
                cv2.imshow("RGB Image", rgb)

                rgbe = 0

            #If there is no IR frame 10 times, destroy the window
            elif rw == True:
                #rgbe += 1
                if rgbe > 10:
                    rw = False
                    #cv2.destroyWindow("Polycon View")
                    cv2.destroyWindow("RGB Image")

            #Initializing wait key to record user input when opencv window is in frame
            key = cv2.waitKey(10)

            #Escape key to end program
            if key == 27:
                cv2.destroyAllWindows()
                print("---end---")
                exit()

            #Checking to see if user wants to change data mode and displaying options if keypress is m or M
            elif (key == ord('m') or key == ord('M')) and not(rs == True or ds == True):
                print("Available Data Modes:")
                for index, element in enumerate(datamodes):
                    print(index + 1, element)
                print("Type number of desired datamode:")

                ms = True
            
            #Checking which option user selected for data mode change
            elif (key == 49 or key == 50 or key == 51 or key == 52 or key == 53) and ms == True:

                #WDR Mode needs to have object initialized to denote selected depth ranges, and style and output mode need to be set
                if key == 52:
                    WDRMode = PsWDROutputMode()
                    WDRMode.totalRange = 3
                    WDRMode.range1 = 0
                    WDRMode.range1Count = 1
                    WDRMode.range2 = 2
                    WDRMode.range2Count = 1
                    WDRMode.range3 = 5
                    WDRMode.range3Count = 1
                    camera.set_WDR_style(WDR_Style.Fusion)
                    camera.set_WDR_output_mode(WDRMode)
                    camera.set_data_mode(DataMode.WDR_Depth)

                #Setting the data mode otherwise
                else:
                    camera.set_data_mode(DataMode(key - 49))
                
                ms = False

            #TODO Fix RGB resolution change, does not work
            #Checking to see if user wants to change resolution and displaying options if keypress is r or R
            elif (key == ord('r') or key == ord('R')) and not(ms == True or ds == True):
                print("Available Resolutions:")
                for index, element in enumerate(resos):
                    print(index + 1, element)
                print("Type number of desired resolution:")

                rs = True
        
            #Checking which option user selected for resolution change
            elif (key == 49 or key == 50 or key == 51) and rs == True:
                match key:
                    case 49:
                        camera.set_RGB_resolution(Reso._640x480)
                    case 50:
                        camera.set_RGB_resolution(Reso._1600x1200)
                    case 51:
                        camera.set_RGB_resolution(Reso._800x600)

                rs = False

            #Checking to see if user wants to change data mode and displaying options if keypress is d or D
            elif (key == ord('d') or key == ord('D')) and not(ms == True or rs == True):
                print("Available Depth Ranges:")
                for index, element in enumerate(ranges):
                    print(index + 1, element)
                print("Type number of desired range:")

                ds = True
            
            #Checking which option user selected for depth ranges
            elif (key == 49 or key == 50 or key == 51 or key == 52) and ds == True:

                #XFar is displayed seperately as it's the only one besides Near, Mid, and Far that work, and the index != the appropriate key for XFar (see Vsense_enums_560)
                if key == 52:
                    key = 54
                key -= 49
                camera.set_depth_range(key)
                
                depth_max, value_min, value_max = camera.get_measuring_range()

                ds = False
                        
        except Exception as e:
            print(e)
        finally:
            time.sleep(.001)


if __name__ == '__main__':
    main()
