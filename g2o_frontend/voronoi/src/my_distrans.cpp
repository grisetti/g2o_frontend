#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Core>


#include <iostream>
#include <stdio.h>
#include <deque>

using namespace cv;
using namespace Eigen;
using namespace std;

typedef deque<Vector2i> myDeque;



static const Scalar colors[] =
{
    Scalar(0,0,0),
    Scalar(255,0,0),
    Scalar(255,128,0),
    Scalar(255,255,0),
    Scalar(0,255,0),
    Scalar(0,128,255),
    Scalar(0,255,255),
    Scalar(0,0,255),
    Scalar(255,0,255)
};


const char* keys =
{
    "{1| |stuff.jpg|input image file}"
};


void voroExtractor(Mat& input, Mat& boolean, Mat& output)
{
    myDeque que;

    int rows = input.rows;
    int cols = input.cols;

    for(int x = 0; x < rows; ++x)
    {
        for(int y = 0; y < cols; ++y)
        {
            if((input.at<float>(x, y) != 0) && (boolean.at<uchar>(x, y) == 0))
            {
                que.push_back(Vector2i(x, y));
            }

            while(!que.empty())
            {
                Vector2i parent = que.front();
                que.pop_front();

                int x = parent.x();
                int y = parent.y();
                boolean.at<uchar>(x, y) = 1;

                Vector2i bestChild;
                float bestDistance = input.at<float>(x, y);

                cout << "VISITING: " << x << ", " << y << endl;
                cout << "CURR DIST: " << bestDistance << endl;


                for(short int r = -1; r < 2; ++r)
                {
                    for(short int c = -1; c < 2; ++c)
                    {
                        int currentX = x+r;
                        int currentY = y+c;

                        cout << "currX: " << currentX << ", currY: " << currentY << endl;

                        //Out of boundaries
                        if((currentX < 0) || (currentY < 0) || (currentX >= rows) || (currentY >= cols))
                        {
                            continue;
                        }

                        //Checking the parent node
                        if((currentX == x) && (currentY == y))
                        {
                            continue;
                        }

                        if(((input.at<float>(currentX, currentY)) == 0) || ((boolean.at<uchar>(currentX, currentY)) != 0))
                        {
                            continue;
                        }

                        boolean.at<uchar>(currentX, currentY) = 1;

                        if(input.at<float>(currentX, currentY) >= bestDistance)
                        {
                            bestDistance = input.at<float>(currentX, currentY);
                            bestChild = Vector2i(currentX, currentY);
                        }
                    }
                }
                cout << "BEST CHILD: " << bestChild.x() << ", " << bestChild.y() << endl;
                cout << "BEST DISTANCE: " << bestDistance << endl;
                if((bestChild.x() != parent.x()) && (bestChild.y() != parent.y()))
                {
                    que.push_back(bestChild);
                    output.at<uchar>(bestChild.x(), bestChild.y()) = 255;
                }
            }
        }
    }
}



int main( int argc, const char** argv )
{
    CommandLineParser parser(argc, argv, keys);
    string filename = parser.get<string>("1");
    Mat gray = imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    if(gray.empty())
    {
        printf("Cannot read image file: %s\n", filename.c_str());
        return -1;
    }

    namedWindow("Voronoi", 1);

    Mat dist = Mat::zeros(gray.rows, gray.cols, CV_32FC1);
    distanceTransform(gray, dist, CV_DIST_L2, CV_DIST_MASK_PRECISE);

    Mat boolean = Mat::zeros(gray.rows, gray.cols, CV_8UC1);
    Mat voro = Mat::zeros(gray.rows, gray.cols, CV_8UC1);

    voroExtractor(dist, boolean, voro);

    imshow("Voronoi", voro);

    for(;;)
    {
        int c = cvWaitKey(0) & 255;

        if(c == 27)
            break;
    }
    return 0;
}
