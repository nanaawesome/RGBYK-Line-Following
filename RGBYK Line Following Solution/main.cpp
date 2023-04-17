// Include files for required libraries
#include <stdio.h>
#include "main.hpp"
#include "opencv_aee.hpp"

#include "pi2c.h"
Pi2c car(0x04);

using namespace std;
using namespace cv;

void setup(void) {
    setupCamera(320, 240);  // Enable the camera for OpenCV
}

void follow_line(Mat line,int16_t *cumulative_error,int16_t *prev_error);

int main(int argc, char **argv) {
    setup();  // Call a setup function to prepare IO and devices
    namedWindow("Photo");  // Create a GUI window called photo
    //new code:
    int16_t cumulative_error=0,prev_error;
    bool isColoured=false, turnOnif=false;
    int minNum,maxNum;
    while (1)  // Main loop to perform image processing
    {
        Mat frame;

        while (frame.empty())
            frame = captureFrame();  // Capture a frame from the camera and store in a new matrix variable

        flip(frame,frame,0);
        flip(frame,frame,1);

        //new code
        Mat line = frame.clone();
        line = line(Range(140,240),Range(20,300));
        Mat lineHSV,linebinary;
        cvtColor(line,lineHSV,COLOR_BGR2HSV);
        GaussianBlur(lineHSV,lineHSV,Size(11,11),0,0);
        //end of new code

        // Convert the image to hsv
        Mat HSVImage;
        cvtColor(frame, HSVImage, COLOR_BGR2HSV);
        //Apply blur
        GaussianBlur(HSVImage,HSVImage,Size(11,11),0,0);
        Mat symbol_frame;
        // convert the image to binary image using the inRange function for pink pixels
        inRange(HSVImage, Scalar(110, 35, 35), Scalar(180, 255, 255), symbol_frame);
        Mat kernel = getStructuringElement(MORPH_ELLIPSE,Size(7,4));
        morphologyEx(symbol_frame,symbol_frame,MORPH_OPEN,kernel);

        // find the contours of the image saving them as img_symbol_contours
        vector<vector<Point>> img_symbol_contours;
        findContours(symbol_frame, img_symbol_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        drawContours(frame, img_symbol_contours, -1, Scalar(0, 0, 255), 2);

        // check if corners of contour is greater than 4
        vector<Point2f> corners;
        for (int i = 0; i < img_symbol_contours.size(); i++) {
            if (img_symbol_contours[i].size() >= 4) {
                approxPolyDP(img_symbol_contours[i], corners, arcLength(img_symbol_contours[i], true) * 0.02, true);
                if (corners.size() == 4) {
                    // draw the contours
                    drawContours(frame, img_symbol_contours, i, Scalar(0, 255, 0), 2);
                    // draw the corners
                    for (int j = 0; j < corners.size(); j++) {
                        circle(frame, corners[j], 5, Scalar(0, 0, 255), 2);
                    }
                }
            }
        }

        // check if there are 4 corners
        if (corners.size() == 4) {
        // change the perspective of symbol_check so that the 4 corners found are the new corners
            Mat perspective_transform = getPerspectiveTransform(corners, vector<Point2f>{Point2f(0, 0), Point2f(0, 350), Point2f(350, 350), Point2f(350, 0)});
            warpPerspective(symbol_frame, symbol_frame, perspective_transform, Size(350, 350));
            // make an array of Mat objects to store the symbols and their rotated forms
            //array locations: 0 is for circle, 1-4 is for Star and its rotated forms, 5-8 is for Triangle and its rotated forms, 9-12 is for Umbrella and its rotated forms
            Mat symbols[13];
            symbols[0] = imread("Circle (Red Line).png");
            String filenames[4]={"Star (Green Line).png","Triangle (Blue Line).png","Umbrella (Yellow Line).png"};
            int rotations[4]={0,ROTATE_90_CLOCKWISE,ROTATE_180,ROTATE_90_COUNTERCLOCKWISE};
            for(int i=1;i<13;i+=4){
                symbols[i]=imread(filenames[((i+3)/4)-1]);
                for(int j=0;j<4;j++){
                    rotate(symbols[i],symbols[i+j],rotations[j]);
                }
            }
            int max_pixels=-999, max_index;
            for (int i = 0; i < 13; i++) {
            // convert the symbols to hsv and binary
                cvtColor(symbols[i], symbols[i], COLOR_BGR2HSV);
                inRange(symbols[i], Scalar(110, 50, 50), Scalar(180, 255, 255), symbols[i]);
                Mat result;
                // Compare frame with original symbols
                compare(symbol_frame, symbols[i], result, CMP_EQ);
                // convert the pixel count to a percentage (Total pixels: 350*350)
                int pixel_count = (countNonZero(result)/122500.00)*100;
                // find which symbol comparison has the most similar pixels
                if(pixel_count>max_pixels){
                    max_pixels=pixel_count; //75%
                    max_index=i;
                }
            }
            String symbol_name;
            if(max_index>=1 && max_index<=4 && max_pixels>75){
                    symbol_name="Star"; //Green
                    printf("Following Green Line\n");
                    inRange(lineHSV, Scalar(35, 50, 40), Scalar(77, 255, 255), linebinary);
                    isColoured=true;
                    turnOnif=true;
                    minNum=35,maxNum=77;
                }
                else if(max_index>=5 && max_index<=8 && max_pixels>75){
                    symbol_name="Triangle"; //Blue
                    printf("Following Blue Line\n");
                    inRange(lineHSV, Scalar(78, 50, 40), Scalar(120, 255, 255), linebinary);
                    isColoured=true;
                    turnOnif=true;
                    minNum=78,maxNum=120;
                }
                else if(max_index>=8 && max_index<=12 && max_pixels>75){
                    symbol_name="Umbrella"; //Yellow
                    printf("Following Yellow Line\n");
                    inRange(lineHSV, Scalar(20, 50, 40), Scalar(34, 255, 255), linebinary);
                    isColoured=true;
                    turnOnif=true;
                    minNum=20,maxNum=34;
                }
                else if(max_index==0 && max_pixels>75){
                    symbol_name="Circle"; //Red
                    printf("Following Red Line\n");
                    inRange(lineHSV, Scalar(0, 50, 40), Scalar(19, 255, 255), linebinary);
                    isColoured=true;
                    turnOnif=true;
                    minNum=0,maxNum=19;
                }
                //stop motors
        }else if(isColoured && countNonZero(linebinary)>0){
            inRange(lineHSV, Scalar(0, 50, 40), Scalar(19, 255, 255), linebinary);
        }else {
            printf("Following Black Line\n");
            inRange(lineHSV, Scalar(0, 0, 0), Scalar(178, 255, 60), linebinary);
            follow_line(linebinary,&cumulative_error,&prev_error);
            isColoured=false;
        }

        if(turnOnif){
            inRange(lineHSV, Scalar(minNum, 50, 40), Scalar(maxNum, 255, 255), linebinary);
            turnOnif=false;
        }

        /*imshow("Photo", line);                         // Display the image in the window
        //imshow("Black and White", linebinary);
        int key = waitKey(1);                           // Wait 1ms for a keypress (required to update windows)

        key = (key == 255) ? -1 : key;                      // Check if the ESC key has been pressed
        if (key == 27)
            break;*/
    }
    closeCV();  // Disable the camera and close any windows
    return 0;
}

/*
imshow("HSV", HSVImage);*/
void follow_line(Mat line,int16_t *cumulative_error,int16_t *prev_error){
    int16_t PID,error;
    double Kp=0.4,Ki=0,Kd=0.3;
    //Morphology operation
    Mat maskMorph = getStructuringElement(MORPH_ELLIPSE, Size(10, 10));
    dilate(line,line,maskMorph);
    int weighted_pixels[line.cols];
    int unweightedpixels[line.cols];
    //Initialise all array locations to 0.
    for(int i=0;i<line.cols;i++){
        weighted_pixels[i]=0;
        unweightedpixels[i]=0;
    }
    int weight=-140;
    for(int i=0;i<line.cols;i++){
        for(int j=0;j<line.rows;j++){
            int pixels=static_cast<int>(line.at<unsigned char>(j,i));
            weighted_pixels[i]+=(pixels/255);
            unweightedpixels[i]+=(pixels/255);
        }
        weighted_pixels[i]=weighted_pixels[i]*weight;
        weight++;
    }
    int sum_weight=0;
    int sum=0;
    int weighted_average;
    for(int i=0;i<line.cols;i++){
        sum_weight+=weighted_pixels[i];
        sum+=unweightedpixels[i];
    }
    weighted_average=sum_weight/sum;
    error=weighted_average;

    //PID Code for Arduino
    //when going left, error positive. when going right, negative
    PID = (Kp*error)+(Ki*(*cumulative_error))+(Kd*(error-(*prev_error)));
    cout << "PID:  " << PID<< endl;
    car.i2cWriteArduinoInt(PID);
    *prev_error = error;
    *cumulative_error+=error;
}
