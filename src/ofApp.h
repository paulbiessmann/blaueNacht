#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"
#include "ofxGui.h"
#include <stdlib.h>

#define HOST "localhost"


class dancers{
    
//    ofVec2f p;
//    ofVec2f pOld;
//    ofVec2f vel;
//    ofVec2f velAbs;
//    ofVec2f blobSizes;
//    ofVec2f velNorm;
//   // float velAbs;
//    bool triggerN;
    
};

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    void setupGui();
    void drawGui();
    
    float vecAbs(ofVec2f);
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    void exit();
    
    
    
    ofTrueTypeFont font;
    ofxOscSender sender;
    ofBuffer imgAsBuffer;
    ofImage img;
    
    
    // -- OSC --
    int port = 8003;
    
    // --- Kinect stuff ----
    ofxKinect kinect;
    ofxCvColorImage colorImg;
    
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    ofxCvContourFinder contourFinder;
    
    int nearThreshold;
    int farThreshold;
    int angle;
    
    
    bool drawContour = true;
    //----------
    
    // --- swarm numbers
    vector <ofVec2f> points;
    vector <ofVec2f> pointsOld;
    vector <ofVec2f> vel;
    vector <float> velAbs;
    vector <bool> triggerN;
    ofVec2f velAvg;
    float   velAbsAvg;
    ofVec2f velAvgOld;
    bool    massTrigger = false;
    float   time0 = 0;
    
    
    
    ofVec2f massCenter;
    ofVec2f center;
    
    float   chaos;
    float   chaosOld;
    float   velAlignment;
    float   velDiff;
    float   distCenterSum;
    float   blobSizeDiffSum;
    
    
    float dist, distNorm;
    float distAvg;
    vector <float> distN;
    vector <float> blobSizes;
    vector <float> blobLen;
    
    
    
    // tweakable:
    
    
    // GUI
    bool bHideGui;
    ofxPanel gui;
    ofxFloatSlider nearKinectThresh;
    ofxFloatSlider farKinectThresh;
    ofxFloatSlider triggerThresh;
    ofxFloatSlider posSmooth; // 0..1;
    ofxIntSlider minArea;
    ofxToggle blur;
    ofxToggle mirror;
    
    
};
