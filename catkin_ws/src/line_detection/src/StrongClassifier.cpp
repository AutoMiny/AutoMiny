//
//  StrongClassifier.cpp
//  TestExtendedDetector
//
/**
Copyright (c)
Audi Autonomous Driving Cup. Team MomenTUM . All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 **/

/***************************************************************************
 * $Author:: Paul Bergmann $  $Date:: 2015-01-15 13:29:48#$ $Rev:: 26104  $*
 ***************************************************************************/

#include "StrongClassifier.h"

//parse a strong classifier from a file
/*
 expected file format:
 <type>2<x>2<y>3<w>2<h>3<thresh>2<pol>g<alpha>0.2<end>
 */
StrongClassifier::StrongClassifier(string filename,double strongThreshold)
{    
    //open file
    ifstream inputstream;
    inputstream.open(filename.c_str());
    
    if(!inputstream.is_open())
    {
        cout << "ERROR: CANT OPEN FEATURE FILE: " << filename << endl;
        return;
    }
    
    string newline;
    while (getline(inputstream, newline)){
        
        //find attribute positions in string
        unsigned long typePos     = newline.find("<type>");
        unsigned long xPos        = newline.find("<x>");
        unsigned long yPos        = newline.find("<y>");
        unsigned long wPos        = newline.find("<w>");
        unsigned long hPos        = newline.find("<h>");
        unsigned long threshPos   = newline.find("<thresh>");
        unsigned long polPos      = newline.find("<pol>");
        unsigned long alphaPos    = newline.find("<alpha>");
        unsigned long endPos      = newline.find("<end>");;

        //parse attributes from string
        string typeString   = newline.substr(typePos,xPos).substr(6);
        string xString      = newline.substr(xPos,yPos-xPos).substr(3);
        string yString      = newline.substr(yPos,wPos-yPos).substr(3);
        string wString      = newline.substr(wPos,hPos-wPos).substr(3);
        string hString      = newline.substr(hPos,threshPos-hPos).substr(3);
        string threshString = newline.substr(threshPos,polPos-threshPos).substr(8);
        string polString    = newline.substr(polPos,alphaPos-polPos).substr(5);
        string alphaString  = newline.substr(alphaPos,endPos-alphaPos).substr(7);
     
        //convert string attributes to corresponding datatypes
        int type = atoi(typeString.c_str());
        int x = atoi(xString.c_str());
        int y = atoi(yString.c_str());
        int w = atoi(wString.c_str());
        int h = atoi(hString.c_str());
        int thresh = atoi(threshString.c_str());
        char polarity = (polString.at(0) == 'g') ? '>' : '<';
        double alpha = atof(alphaString.c_str());
        
        
        //create the weak classifier
        Feature f(type, x, y, w, h);
        WeakClassifier weakClassifier(f, thresh, polarity, alpha);

        weakClassifiers.push_back(weakClassifier);
    }
    threshold = strongThreshold;
}

StrongClassifier::StrongClassifier(vector<WeakClassifier> initClassifiers,double threshold = 0.5)
{
    weakClassifiers = initClassifiers;
}

int StrongClassifier::classifyImage(Mat integralImage,Mat rotatedIntegralImage)
{
    double alphaSum = 0.0;
    double classSum = 0.0;
    
    for(int i = 0;i < (int)weakClassifiers.size();i++)
    {
        int classResult = weakClassifiers.at(i).classifyImage(integralImage, rotatedIntegralImage);
        double alpha = weakClassifiers.at(i).getAlpha();
        
        classSum += classResult * alpha;
        alphaSum += alpha;
    }
    
    if(classSum / alphaSum >= threshold)    return 1;
    else                                    return 0;
}

void StrongClassifier::print()
{
    cout << "--StrongClassifer";
    cout << "(Strong Threshold: " << threshold << ")--" <<  endl;
    
    for(int i = 0;i < (int)weakClassifiers.size();i++)
    {
        weakClassifiers.at(i).print();
    }
    cout << "--/StrongClassifier--" << endl;
}
