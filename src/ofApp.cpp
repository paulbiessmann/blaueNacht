#include "ofApp.h"
int maxBlobs = 4;
//--------------------------------------------------------------
void ofApp::setup(){
    
    ofSetBackgroundColor(0);
    ofSetFrameRate(30);
    
    // open an outgoing connection to HOST:PORT
    sender.setup(HOST, PORT);
    
    
    
    //--- kinect Stuff -----
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    // enable depth->video image calibration
    kinect.setRegistration(true);
    kinect.init();
    kinect.open();        // opens first available kinect
    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    nearKinectThresh = nearThreshold = 255;
    farKinectThresh = farThreshold = 212;
    
//    angle = -20;
//    kinect.setCameraTiltAngle(angle);
    
//    int     minArea = 400;
//    float   posSmooth = 0.5; // 0..1
//    float   triggerThresh = 20;
    
    setupGui();
    
    
    dancers d[maxBlobs];
    
    points.resize(maxBlobs);
    pointsOld.resize(maxBlobs);
    vel.resize(maxBlobs);
    velNorm.resize(maxBlobs);
    velAbs.resize(maxBlobs);
    distN.resize(maxBlobs * maxBlobs);
    blobSizes.resize(maxBlobs);
    blobLen.resize(maxBlobs);
    triggerN.resize(maxBlobs);
    for(int i=0; i<maxBlobs; i++){
        massCenter.set(0,0);
        points[i].set(0,0);
        triggerN[i] = false;
    }

    massCenter.set(0,0);
}

//-------------------------------------------------------------
void ofApp::setupGui(){
    
    string guiPath = "blaueNacht.xml";
    gui.setup("blaueNacht", guiPath, 20,20);
    gui.add(nearKinectThresh.setup("Kinect Near Thr", 255, 0.0, 255.0));
    gui.add(farKinectThresh.setup("Kinect Far Thr", 212, 0.0, 255.0));
    gui.add(triggerThresh.setup("Trigger Threshold", 10, 0.0, 20.0));
    gui.add(posSmooth.setup("Position Smoothening", 0.5, 0.01, 0.99));
    gui.add(minArea.setup("Minimal Blob Area", 500, 100, 4000));
    gui.add(blur.setup("Blur Image", false));
    gui.add(mirror.setup("Mirror Image", true));

    //    gui.add(chaos.setup("Swarm", 0.0, 0.0, 2.0));

    //    gui.loadFromFile(guiPath);
}


//--------------------------------------------------------------
void ofApp::update(){
    kinect.update();
    if(kinect.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels());
        
        // Mirror and Blur the Image:
        if(mirror){
            grayImage.mirror(false, true);
        }
        if(blur){
            grayImage.blurHeavily();
        }
        
        // we do threshold
        grayThreshNear = grayImage;
        grayThreshFar = grayImage;
        grayThreshNear.threshold(nearKinectThresh, true);
        grayThreshFar.threshold(farKinectThresh);
        cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        
        
        // update the cv images
        grayImage.flagImageChanged();
        
        
        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        contourFinder.findContours(grayImage, minArea, (kinect.width*kinect.height)/3, 20, false);
        
        int numBlobs = contourFinder.blobs.size();
        
        float time  = ofGetElapsedTimef();
        float dt    = time - time0;
        time0   = time;
        velAvgOld = velAvg;
        velAvg.set(0,0);
        velAvgNorm.set(0,0);
        massCenter.set(0,0);
        
        for (int i = 0;  i < numBlobs; i++){
            ofxCvBlob & blob = contourFinder.blobs[i];
            float velAbsOld = 0;
            ofVec2f c( blob.centroid.x, blob.centroid.y );
            
            pointsOld[i] = points[i];
            points[i]   = pointsOld[i] * posSmooth + c * (1-posSmooth); // LP Filtering
        
            // don't calc speed if the values jump
            if(pointsOld[i].x > 1 && pointsOld[i].y > 1 && pointsOld[i].x - points[i].x < 100){
                vel[i] = (pointsOld[i] - points[i]);// / dt;
                
            }
            velAbsOld = velAbs[i];
            velAbs[i] = vecAbs(vel[i]) + velAbsOld * 0.9;
            velNorm[i] = vel[i];
            velNorm[i].normalize();
            velAvg += vel[i];
            velAvgNorm += velNorm[i];
            
            if(velAbs[i] - velAbsOld > triggerThresh){
                triggerN[i] = true;
            }else{
                triggerN[i] = false;
            }
                
            
            blobSizes[i] = blob.area;
            
            massCenter += points[i];
            
        }
        if(numBlobs>1){
            velAvg /= numBlobs;
            velAvgNorm /= numBlobs;
            massCenter /= numBlobs;
        }
//        for (int i=numBlobs; i<maxBlobs; i++){
//            points[i].set(0,0);
//        }
        
        
        // Trigger Signal aus Average Velocity (Betrag):
        float velAvgAbs = vecAbs(velAvg);
        float velAvgAbsOld = vecAbs(velAvgOld);
        if (velAvgAbs - velAvgAbsOld > triggerThresh){
            massTrigger = true;
        }else{
            massTrigger = false;
        }
            
        
        // --- Distances --- :
        int count = 0;
        int countBlobs = 0;
        distAvg = 0;
        for(int i=0; i<maxBlobs; ++i){
            for(int j=i+1; j<maxBlobs; ++j){
                if(j<numBlobs && (points[i].x > 0 && points[i].y > 0 &&  points[j].x > 0 && points[j].y > 0)){
                    distN[count] = ofDist(points[i].x, points[i].y, points[j].x, points[j].y);
                }
                else{
                    distN[count] = 0;
                }
                if(distN[count] > 10){
                    distAvg += distN[count];
                    countBlobs++;
                }
                count++;
            }
        }
        if (numBlobs > 1){
                dist = ofDist(points[0].x, points[0].y, points[1].x, points[1].y);
                distNorm = ofMap( dist, 0, kinect.width, 0, 1 );
        }
        else{
            distNorm = 0;
        }
        
        if(countBlobs > 1){
            distAvg /= countBlobs;
        }
        
        
        float bX = ofMap( points[0].x, 0, kinect.width, 0, 127 );
        float bY = ofMap( points[0].y, 0, kinect.height, 0, 127 );
        float b2X = ofMap( points[1].x, 0, kinect.width, 0, 127 );
        float b2Y = ofMap( points[1].y, 0, kinect.height, 0, 127 );
        
        
        // *** Send OSC Data ***

        /*    data:
                points[i]
                massXY
                speed
                speedAvg
                distN[i]
                distAvg
                sizes[i]

                 Bsp: for loop{
                    /blobs [i]  .xy  .vel  .size
                 }
                 /mass  .distAvg .velAvg
         */
        
        
        if(ofGetFrameNum() % 2 == 0){
            
            
            
            ofxOscMessage m;
            m.setAddress("/mass");
            m.addFloatArg(velAvgAbs);
            m.addFloatArg(distAvg);
            if(massTrigger){
                m.addTriggerArg();
            }
            
            sender.sendMessage(m, false);
            
            m.setAddress("/blob");
            for (int i = 0; i < maxBlobs; i++){
                
                m.addFloatArg(ofMap( points[i].x, 0, kinect.width, 0.0, 1.0 ));
                m.addFloatArg(ofMap( points[i].y, 0, kinect.height, 0.0, 1.0 ));
                m.addFloatArg(velAbs[i]);
                m.addFloatArg(blobSizes[i]);
                if(triggerN[i]){
                    m.addTriggerArg();
                }
                else{
                    m.addIntArg(0);
                }
                
            }
            sender.sendMessage(m, false);
            m.clear();
            

        }

        
        if(0){ //log
            //ofLogNotice() << "numBlobs: " << numBlobs;
            ofLogNotice() << "bx: " << bX << "speed: " << velAbs[0];
            ofLogNotice() << "distN: " << distNorm;
            ofLogNotice() << "blobSize 0: " << blobSizes[0];
            //            ofLogNotice() << "blobLen 0: " << blobLen[0];
        }
        
        
        
    }
    //--- end kinect ---
    ofSetWindowTitle(ofToString(ofGetFrameRate(), 2));
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    if(drawContour){
        contourFinder.draw(0, 0, ofGetWidth(), ofGetHeight());
    }
    else{
        grayImage.draw(0,0, ofGetWidth(), ofGetHeight());
    }
    
    ofFill();
    ofSetColor(255,255,255);
    
    string swarmMsg = "speedAbs ";
    ofDrawBitmapString(swarmMsg, 300, 20);
    for(int i = 0; i<4; i++){
        ofDrawRectangle(300, 10+30*i, velAbs[i] * 0.2, 10);
    }
    swarmMsg = "dist ";
    ofDrawBitmapString(swarmMsg, 300, 200);
    for(int i = 0; i<8; i++){
        ofDrawBitmapString(ofToString(distN[i]), 300, 210+30*i);
        ofDrawRectangle(300, 220+30*i, distN[i] * 0.1 , 10);
    }
    //ofDrawCircle(500, 300, distAvg);
    ofDrawBitmapString("distAvg ", 500, 500);
    ofDrawBitmapString(ofToString(distAvg), 520, 520);
    
    
    swarmMsg = "blobSizes ";
    ofDrawBitmapString(swarmMsg, 300, 800);
    ofDrawRectangle(300, 820, blobSizes[0]*0.1, 20);
    
    
    // draw center points
    ofSetColor(0,0,255);
    int numBlobsCV = contourFinder.blobs.size();
    int numBlobs = 0;  // don't use all blobs
    for (int i=0; i<numBlobsCV; i++){
        int pTx = ofMap(points[i].x, 0, kinect.width, 0, ofGetWidth() );
        int pTy = ofMap(points[i].y, 0, kinect.height, 0, ofGetHeight() );
        float vAbs_tmp = ofClamp(velAbs[i] , 1, 60);
        
        ofSetColor(255,255,0);
        if(triggerN[i]){
            ofSetColor(255,0,0);
        }
        ofDrawCircle(pTx, pTy, vAbs_tmp +5);
    }
    
    // Draw Massen Center
    ofSetColor(0, 100, 100);
    int mCx = ofMap(massCenter.x, 0, kinect.width, 0, ofGetWidth() );
    int mCy = ofMap(massCenter.y, 0, kinect.height, 0, ofGetHeight() );

    ofDrawCircle(mCx, mCy, vecAbs(velAvg) + 5);


    ofSetColor(0,0,255);
    for(int i=0; i<maxBlobs; ++i){
        for(int j=i+1; j<maxBlobs; ++j){
            int pIx = ofMap(points[i].x, 0, kinect.width, 0, ofGetWidth() );
            int pIy = ofMap(points[i].y, 0, kinect.height, 0, ofGetHeight() );
            int pJx = ofMap(points[j].x, 0, kinect.width, 0, ofGetWidth() );
            int pJy = ofMap(points[j].y, 0, kinect.height, 0, ofGetHeight() );
            ofDrawLine(pIx, pIy, pJx, pJy);
        }
    }
    
    ofSetColor(255);
    
    
    // Daten:
    ofPushMatrix();
    ofTranslate(500, 300);
    ofSetColor(250);
    
    //    swarmMsg = "Average Speed X " + ofToString(swarmSpeed.x);
    //    ofDrawBitmapString(swarmMsg, -100, -100);
    //    swarmMsg = "Average Speed Y " + ofToString(swarmSpeed.y);
    //    ofDrawBitmapString(swarmMsg, -100, -80);
    //
    //    float swarmV = abs(swarmSpeedNorm.x) + abs(swarmSpeedNorm.y);
    //
    //    float swarmValue = ofMap(swarmV, 0, 1.4, 0, 1.0);
    //    swarmMsg = "Swarm Value " + ofToString(swarmValue, 4);
    //    ofDrawBitmapString(swarmMsg, -100, 100);
    //    ofDrawRectangle(-55, 110, swarmValue*100, 10);
    //    ofSetColor(255,0,0);
    //    ofDrawRectangle(swarmValue*100-55, 109, 3, 12);
    //
    //    ofDrawBitmapString("Chaos", -100, 120);
    //    ofDrawBitmapString("Order", 50, 120);
    //
    //    ofSetColor(0,0,0,50);
    //    ofDrawRectangle( - 75, 0 , 150, 1);
    //    ofDrawRectangle( 0, -75, 1, 150);
    //
    //    ofSetColor(255,0,0);
    //    ofDrawRectangle(swarmSpeed.x*10, swarmSpeed.y*10, 5, 5);
    ofPopMatrix();
    
    
//    ofDrawBitmapString("far " + ofToString(farKinectThresh), 20, 400);
//    ofDrawBitmapString("near " + ofToString(nearKinectThresh), 20, 430);
    
    if(!bHideGui){
        gui.draw();
    }
    
    // Draw Kinect Thresholds:
    ofNoFill();
    ofScale(0.5,0.5);
    ofTranslate(300, 255);
    ofRotateZDeg(180);
    ofSetColor(150);
    ofDrawRectangle(0, 0, 300, 255);
    ofSetColor(255,0,0);
    ofDrawLine(0, farKinectThresh, 300, farKinectThresh);
    ofSetColor(0,255,0);
    ofDrawLine(0, nearKinectThresh, 300, nearKinectThresh);
    
    

    
}

//--------------------------------------------------------------
float ofApp::vecAbs(ofVec2f vec){
    float vAbs = sqrt((vec.x * vec.x) + (vec.y * vec.y));
    
    return vAbs;
}


//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == 'a' || key == 'A'){
        ofxOscMessage m;
        m.setAddress("/test");
        m.addIntArg(50);
        //m.addFloatArg(3.5f);
        //m.addStringArg("hello");
        //m.addFloatArg(ofGetElapsedTimef());
        sender.sendMessage(m, false);
    }
    
    switch(key){
            
        case 'c':
            drawContour = !drawContour;
            break;
        case '>':
        case '.':
            farThreshold++;
            if (farThreshold > 255) farThreshold = 255;
            break;
            
        case '<':
        case ',':
            farThreshold--;
            if (farThreshold < 0) farThreshold = 0;
            break;
            
        case '+':
        case '=':
            nearThreshold++;
            if (nearKinectThresh > 255) nearKinectThresh = 255;
            break;
            
        case '-':
            nearThreshold--;
            if (nearKinectThresh < 0) nearKinectThresh = 0;
            break;
            
        case OF_KEY_UP:
            angle++;
            if(angle>30) angle=30;
            kinect.setCameraTiltAngle(angle);
            break;
            
        case OF_KEY_DOWN:
            angle--;
            if(angle<-30) angle=-30;
            kinect.setCameraTiltAngle(angle);
            break;
            
        case 'g':
            bHideGui = !bHideGui;
            break;
    }
    
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    //    ofxOscMessage m;
    //    m.setAddress("/mouse/position");
    //    m.addIntArg(x);
    //    m.addIntArg(y);
    //    sender.sendMessage(m, false);
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}

//--------------------------------------------------------------
void ofApp::exit() {
    // kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
    
#ifdef USE_TWO_KINECTS
    kinect2.close();
#endif
}
