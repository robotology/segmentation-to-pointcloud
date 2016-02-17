/* 
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini, Tanis Mar
 * email:  ugo.pattacini@iit.it, tanis.mar@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <vector>
#include <algorithm>
#include <string>
#include <fstream>
#include <iomanip>

#include <opencv2/opencv.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


/*******************************************************************************/
class Obj3DrecModule : public RFModule, public PortReader
{
protected:

    vector<cv::Point> contour;    
    vector<cv::Point> floodPoints;
    cv::Point seed;
    cv::Rect rect;

    string homeContextPath;
    string savename;
    string fileFormat;
    int fileCount;

    int downsampling;
    double spatial_distance;
    int color_distance;
    Mutex mutex;    
    bool go,flood3d,flood,seg;

    BufferedPort<ImageOf<PixelMono> > portDispIn;
    BufferedPort<ImageOf<PixelRgb> > portDispOut;
    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<Bottle> portPointsOut;
    BufferedPort<Bottle> portSeedIn;
    Port portContour;
    RpcClient portSFM;
    RpcClient portSeg;
    RpcServer portRpc;

    std::string                         name;               //name of the module

    /*******************************************************************************/
    bool read(ConnectionReader &connection)
    {
        Bottle data; data.read(connection);
        if (data.size()>=2)
        {
            LockGuard lg(mutex);
            cv::Point point(data.get(0).asInt(),data.get(1).asInt());
            contour.push_back(point);
        }

        return true;
    }

public:
    /*******************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        name = rf.check("name", Value("obj3Drec"), "Getting module name").asString();

        homeContextPath=rf.getHomeContextPath().c_str();
        savename = rf.check("savename", Value("cloud3D"), "Default file savename").asString();
        fileFormat = rf.check("format", Value("off"), "Default file format").asString();

        cout << "Files will be saved in "<< homeContextPath << " folder, as " << savename <<"N." << fileFormat <<", with increasing numeration N"  << endl;
        fileCount = 0;

        downsampling=std::max(1,rf.check("downsampling",Value(1)).asInt());
        spatial_distance=rf.check("spatial_distance",Value(0.004)).asDouble();
        color_distance=rf.check("color_distance",Value(6)).asInt();


        printf("Opening ports\n" );
        bool ret= true;
        ret = ret && portDispIn.open("/"+name+"/disp:i");
        ret = ret && portImgIn.open("/"+name+"/img:i");
        ret = ret && portSeedIn.open("/"+name+"/seed:i");
        ret = ret && portContour.open("/"+name+"/contour:i");

        ret = ret && portPointsOut.open("/"+name+"/pnt:o");
        ret = ret && portDispOut.open("/"+name+"/disp:o");

        ret = ret && portSFM.open("/"+name+"/SFM:rpc");
        ret = ret && portSeg.open("/"+name+"/seg:rpc");
        ret = ret && portRpc.open("/"+name+"/rpc:i");

        if (!ret){
            printf("Problems opening ports\n");
            return false;
        }
        printf("Ports opened\n");


        portContour.setReader(*this);
        //portSeedIn.setReader(*this);
        attach(portRpc);

        go=flood3d=flood=seg=false;

        rect =cv::Rect(1, 1, 0,0);

        return true;
    }

    /*******************************************************************************/
    bool interruptModule()
    {
        portDispIn.interrupt();
        portDispOut.interrupt();
        portImgIn.interrupt();
        portContour.interrupt();
        portPointsOut.interrupt();
        portSFM.interrupt();
        portSeg.interrupt();
        portRpc.interrupt();
        return true;
    }

    /*******************************************************************************/
    bool close()
    {
        portDispIn.close();
        portDispOut.close();
        portImgIn.close();
        portContour.close();
        portPointsOut.close();
        portSFM.close();
        portSeg.close();
        portRpc.close();
        return true;
    }

    /*******************************************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /*******************************************************************************/
    bool updateModule()
    {
        //================ Read ports for images and data ====================

        // read images
        ImageOf<PixelMono> *imgDispIn=portDispIn.read();
        if (imgDispIn==NULL)
            return false;

        ImageOf<PixelRgb> *imgIn=portImgIn.read();
        if (imgIn==NULL)
            return false;

        LockGuard lg(mutex);
        
        ImageOf<PixelRgb> &imgDispOut=portDispOut.prepare();
        imgDispOut.resize(imgDispIn->width(),imgDispIn->height());

        // yarp iamges to openCV
        cv::Mat imgInMat=cv::cvarrToMat((IplImage*)imgIn->getIplImage());
        cv::Mat imgDispInMat=cv::cvarrToMat((IplImage*)imgDispIn->getIplImage());
        cv::Mat imgDispOutMat=cv::cvarrToMat((IplImage*)imgDispOut.getIplImage());
        cv::cvtColor(imgDispInMat,imgDispOutMat,CV_GRAY2RGB);

        // Display on the disparity image the selected points
        PixelRgb color(255,255,0);
        for (size_t i=0; i<floodPoints.size(); i++)
            imgDispOut.pixel(floodPoints[i].x,floodPoints[i].y)=color;

        // Display on the disparity image the bounding rectangle
        cv::rectangle(imgDispOutMat,rect,cv::Scalar(255,50,0));


        // Read seed point from port
        Bottle *seedBot = portSeedIn.read(false);
        if (seedBot!=NULL) {
            seed.x = seedBot->get(0).asInt();
            seed.y = seedBot->get(1).asInt();
        }
        // Display the seed point on the disparity image too.
        cv::circle(imgDispOutMat,seed, 3,cv::Scalar(0,0,255), 2);


        //================ Get 3D points from diverse segmentation methods ====================

        // Methods with select a regon from a seed point
        if (flood3d||flood||seg)
        {
            Bottle &bpoints=portPointsOut.prepare();
            vector<Vector> points;

            if (flood3d)
            {
                cout << "3D points flood3D "<<endl;
                Bottle cmd,reply;
                cmd.addString("Flood3D");
                cmd.addInt(seed.x);
                cmd.addInt(seed.y);
                cmd.addDouble(spatial_distance);
                if (portSFM.write(cmd,reply))
                {
                    for (int i=0; i<reply.size(); i+=5)
                    {
                        int x=reply.get(i+0).asInt();
                        int y=reply.get(i+1).asInt();
                        PixelRgb px=imgIn->pixel(x,y);

                        Vector point(6,0.0);
                        point[0]=reply.get(i+2).asDouble();
                        point[1]=reply.get(i+3).asDouble();
                        point[2]=reply.get(i+4).asDouble();
                        point[3]=px.r;
                        point[4]=px.g;
                        point[5]=px.b;

                        points.push_back(point);
                        floodPoints.push_back(cv::Point(x,y));

                        Bottle &bpoint = bpoints.addList();
                        bpoint.addDouble(point[0]);
                        bpoint.addDouble(point[1]);
                        bpoint.addDouble(point[2]);
                    }
                }

                cout << "Retrieved " << points.size() << " 3D points"  <<endl;
            }
            else if (flood)
            {
                cout << "3D points from 2D color flood "<<endl;

                // Set flooding parameters
                //PixelMono c = imgDispIn->pixel(seed.x,seed.y);
                cv::Scalar delta(color_distance);

                // flood region and copy flood onto mask
                cv::Mat mask = cv::Mat::zeros(imgInMat.rows + 2, imgInMat.cols + 2, CV_8U);
                cv::floodFill(imgDispInMat, mask, seed, cv::Scalar(255), NULL, delta, delta,  4 + (255 << 8) | cv::FLOODFILL_FIXED_RANGE| cv::FLOODFILL_MASK_ONLY);

                // Get the contours of the color flooded region from mask.
                vector<vector<cv::Point> > contoursFlood;
                vector<cv::Vec4i> hierarchy;
                cv::findContours(mask, contoursFlood, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                vector<cv::Point> contourFlood = contoursFlood[0];

                // Get the 3D points from the 2D points within the contour.
                pointsFromContour(imgIn, contourFlood, rect, points, bpoints);

                cout << "Retrieved " << points.size() << " 3D points"  <<endl;

            }else if (seg)
            {
                cout << "Extracting 3D points from segmented blob "<<endl;

                // Get segmented region from external segmentation module
                Bottle cmdSeg, replySeg;
                cmdSeg.addString("get_component_around");
                cmdSeg.addInt(seed.x);
                cmdSeg.addInt(seed.y);
                if (portSeg.write(cmdSeg,replySeg))
                {
                    Bottle* pixelList=replySeg.get(0).asList();

                    if (pixelList->size()==0)
                    {
                        cout << "Empty bottle received" <<endl;
                        seg=false;
                        points.clear();
                        bpoints.clear();
                        portPointsOut.write(); // Return empty bottle if no data was received.
                        return true;
                    }
                    cout << "Read " << pixelList->size() << " points from segmentation algorithm" <<endl;
                    cv::Mat binImg = cv::Mat(imgDispInMat.rows, imgDispInMat.cols, CV_8U, 0.0);
                    for (int i=0; i<pixelList->size(); i++)
                    {
                        Bottle* point=pixelList->get(i).asList();
                        int x = point->get(0).asDouble();
                        int y = point->get(1).asDouble();
                        binImg.at<uchar>(y,x) = 255;
                    }

                    // Get the contours of the segmented region.
                    vector<vector<cv::Point> > contoursSeg;
                    vector<cv::Vec4i> hierarchy;
                    cv::findContours(binImg, contoursSeg, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                    vector<cv::Point> contourSeg = contoursSeg[0];
                    //cout << "Contours extracted."  <<endl;

                    // Get the 3D points from the 2D points within the contour.
                    pointsFromContour(imgIn, contourSeg, rect, points, bpoints);

                    cout << "Retrieved " << points.size() << " 3D points"  <<endl;
                }
            }

            if (points.size()>0) {
                portPointsOut.write();
            }else                {
                portPointsOut.unprepare();
            }

            points.clear();
            bpoints.clear();
            flood=flood3d=seg=false;
        }


        // Region is given by contour selected on disp image
        if (contour.size()>0)
        {
            Bottle &bpoints=portPointsOut.prepare();
            vector<Vector> points;

            vector<vector<cv::Point> > contours;
            contours.push_back(contour);            
            cv::drawContours(imgDispOutMat,contours,0,cv::Scalar(255,255,0));

            if (go)
            {
                cout << "3D points from selected contour "<<endl;

                pointsFromContour(imgIn, contour, rect, points, bpoints);

                cout << "Retrieved " << points.size() << " 3D points"  <<endl;

                if (points.size()>0) {
                    portPointsOut.write();
                }else                {
                    portPointsOut.unprepare();
                }

                go =false;
                points.clear();
                bpoints.clear();
            }
        }

        portDispOut.write();
        return true;
    }


    /*******************************************************************************/
    bool savePoints (vector<Vector> points)
    {        // Save points in the desired format
        if (points.size()>0)
        {
            ofstream fout;
            stringstream fileName;
            string fileNameFormat;
            fileName.str("");
            fileName << homeContextPath + "/" + savename.c_str() << fileCount;

            if (fileFormat == "ply")
            {
                fileNameFormat = fileName.str()+".ply";
                cout << "Saving as " << fileNameFormat << endl;
                fout.open(fileNameFormat.c_str());
                if (fout.is_open())
                {
                    fout << "ply\n";
                    fout << "format ascii 1.0\n";
                    fout << "element vertex " << points.size() <<"\n";
                    fout << "property float x\n";
                    fout << "property float y\n";
                    fout << "property float z\n";
                    fout << "property uchar diffuse_red\n";
                    fout << "property uchar diffuse_green\n";
                    fout << "property uchar diffuse_blue\n";
                    fout << "end_header\n";

                    for (unsigned int i=0; i<points.size(); i++){
                        fout << points[i][0] << " " <<      points[i][1] << " " <<      points[i][2] << " " << (int)points[i][3] << " " << (int)points[i][4] << " " << (int)points[i][5] << "\n";
                        //plyfile << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " " << (int)cloud->at(i).r << " " << (int)cloud->at(i).g << " " << (int)cloud->at(i).b << "\n";
                    }

                    fout.close();
                    cout << "Points saved as " << fileNameFormat << endl;
                    fileCount++;
                }
            }else if (fileFormat == "off"){
                fileNameFormat = fileName.str()+".off";
                cout << "Saving as " << fileNameFormat << endl;
                fout.open(fileNameFormat.c_str());
                if (fout.is_open())
                {

                    fout<<"COFF"<<endl;
                    fout<<points.size()<<" 0 0"<<endl;
                    fout<<endl;
                    for (size_t i=0; i<points.size(); i++)
                    {
                        fout<<points[i].subVector(0,2).toString(3,4).c_str()<<" "<<
                              points[i].subVector(3,5).toString(0,3).c_str()<<endl;
                    }
                    fout<<endl;
                }

                fout.close();
                cout << "Points saved as " << fileNameFormat << endl;
                fileCount++;

            }else if (fileFormat == "none"){
                cout << "Points not saved" << endl;
            }

            return true;
        }
        cout << "Point cloud is empty, could not save" << endl;
        return false;
    }

    /*******************************************************************************/
    bool pointsFromContour(const ImageOf<PixelRgb> *imgIn, const vector<cv::Point> contourIn, cv::Rect &boundBox, vector<Vector> &pointsInContour, Bottle &bpoints)
    {
        boundBox = cv::boundingRect(contourIn);

        Bottle cmd,reply;
        cmd.addString("Rect");
        cmd.addInt(boundBox.x);     cmd.addInt(boundBox.y);
        cmd.addInt(boundBox.width); cmd.addInt(boundBox.height);
        cmd.addInt(downsampling);
        if (portSFM.write(cmd,reply))
        {
            int idx=0;
            for (int x=boundBox.x; x<boundBox.x+boundBox.width; x+=downsampling)
            {
                for (int y=boundBox.y; y<boundBox.y+boundBox.height; y+=downsampling)
                {
                    if (cv::pointPolygonTest(contourIn,cv::Point2f((float)x,(float)y),false)>0.0)
                    {                        
                        floodPoints.push_back(cv::Point(x,y));

                        Vector point(6,0.0);
                        point[0]=reply.get(idx+0).asDouble();
                        point[1]=reply.get(idx+1).asDouble();
                        point[2]=reply.get(idx+2).asDouble();

                        Bottle &bpoint = bpoints.addList();
                        bpoint.addDouble(point[0]);
                        bpoint.addDouble(point[1]);
                        bpoint.addDouble(point[2]);

                        if (norm(point)>0.0)
                        {
                            PixelRgb px=imgIn->pixel(x,y);
                            point[3]=px.r;
                            point[4]=px.g;
                            point[5]=px.b;

                            bpoint.addDouble(point[3]);
                            bpoint.addDouble(point[4]);
                            bpoint.addDouble(point[5]);

                            pointsInContour.push_back(point);
                        }
                    }

                    idx+=3;
                }
            }
        }
    }
    /*******************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString().c_str();
        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (cmd=="clear")
        {   //XXX make clear in a function so that it can be called from the other routines, and clean the yarpviews when any order is called.
            LockGuard lg(mutex);
            contour.clear();
            floodPoints.clear();
            rect = cv::Rect(1,1,0,0);            
            go=flood3d=flood=seg=false;
            reply.addVocab(ack);
        }
        else if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands are:");
            reply.addString("help - produces this help");
            reply.addString("go - gets pointcloud from the selected polygon on the disp image");
            reply.addString("flood int(color_distance) int int (coords(opt))- gets pointcloud from 2D color flood. User has to select the seed pixel from the disp image");
            reply.addString("flood3d double(spatial_distance) int int (coords(opt))- gets pointcloud from 3D color flood (based on depth). User has to select the seed pixel from the disp image");
            reply.addString("seg int int (coords(opt))- gets pointcloud from an externally segmented blob. User has to select the seed pixel from the disp image");
            reply.addString("setFormat string(fileformat)- sets the format in which the points will be saved. 'fileformat' can be  'ply', 'off' or 'none'.");
            reply.addString("setFileName string(filename)- sets the base name given to the files where the 3D points will be saved. ");
            return true;
        }
        else if ((cmd=="go") || (cmd=="flood3d")|| (cmd=="flood")|| (cmd=="seg"))
        {
            if (portSFM.getOutputCount()==0)
                reply.addVocab(nack);
            else
            {
                LockGuard lg(mutex);
                if (cmd=="go")
                {
                    if (contour.size()>2)
                    {
                        go=true;
                        reply.addVocab(ack);
                    }
                    else
                        reply.addVocab(nack);
                }
                else if (cmd=="flood3d")
                {
                    reply.addVocab(ack);
                    contour.clear();
                    floodPoints.clear();
                    if (command.size()>=2){
                        spatial_distance=command.get(1).asDouble();
                        reply.addInt(spatial_distance);
                    }

                    if (command.size()>=4){
                        cv::Point coords2D;
                        coords2D.x=command.get(2).asInt();
                        coords2D.y=command.get(3).asInt();
                        contour.push_back(coords2D);
                        reply.addInt(coords2D.x);
                        reply.addInt(coords2D.y);
                    }
                    flood3d = true;

                }
                else if (cmd=="flood")
                {
                    reply.addVocab(ack);
                    contour.clear();
                    floodPoints.clear();
                    if (command.size()>=2){
                        color_distance=command.get(1).asInt();
                        reply.addInt(color_distance);
                    }

                    if (command.size()>=4){
                        cv::Point coords2D;
                        coords2D.x=command.get(2).asInt();
                        coords2D.y=command.get(3).asInt();
                        contour.push_back(coords2D);
                        reply.addInt(coords2D.x);
                        reply.addInt(coords2D.y);
                    }


                    flood=true;

                }
                else if (cmd=="seg")
                {
                    contour.clear();
                    reply.addVocab(ack);
                    if (command.size()>=3){
                        cv::Point coords2D;
                        coords2D.x=command.get(1).asInt();
                        coords2D.y=command.get(2).asInt();
                        contour.push_back(coords2D);    
                        reply.addInt(coords2D.x);
                        reply.addInt(coords2D.y);
                    }                   
                    seg=true;
                    
                }
            }
        }
        else if (cmd=="setFormat")
        {
            if (command.size()>=2){
                string format = command.get(1).asString();
                if ((format=="ply")||(format=="off")||(format=="none")){
                    fileFormat = format;
                    reply.addVocab(ack);
                    reply.addString("Format set correctly");
                } else {
                    reply.addVocab(nack);
                    reply.addString("No valid format chosen. Choose ply/off/none");
                }
            }
        }
        else if (cmd=="setFileName")
        {
            if (command.size()>=2){
                savename = command.get(1).asString();
                reply.addVocab(ack);
                reply.addString("File Name set correctly");
            }
        }
        else 
            RFModule::respond(command,reply);
        
        return true;
    }
};


/*******************************************************************************/
int main(int argc,char *argv[])
{   
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("unable to find YARP server!");
        return 1;
    }

    Obj3DrecModule mod;
    ResourceFinder rf;
    rf.setDefaultContext("obj3Drec");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}

