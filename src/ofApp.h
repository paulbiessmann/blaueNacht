#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"

#define HOST "localhost"
#define PORT 6003

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
    ofVec2f p;
    ofVec2f p2;
    
    
    int minArea = 400;
    
    bool drawContour = true;
    //----------
    
    // --- swarm numbers
    vector <ofVec2f> points;
    vector <ofVec2f> pointsOld;
    vector <ofVec2f> vel;
    vector <ofVec2f> velNorm;
    vector <float> velAbs;
    ofVec2f velAvg;
    ofVec2f velAvgNorm;
    ofVec2f velAvgOld;
    bool    velTrigger = false;
    float time0 = 0;
    
    ofVec2f massCenter;
    
    
    
    
    float dist, distNorm;
    float distAvg;
    vector <float> distN;
    vector <float> blobSizes;
    vector <float> blobLen;
};
