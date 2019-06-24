#include <ros/ros.h>

#include "opencv2/opencv_modules.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video/tracking.hpp"

#include <math.h>
#include <algorithm>
#include <random>
#include <chrono>   
#include <cstdlib>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>



using namespace ros;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;


class Birdeye{

  private:
  ros::NodeHandle nh;
  image_transport::Publisher pub;
  ros::Subscriber sub;
  sensor_msgs::ImagePtr msg_;
  image_transport::ImageTransport it;
  cv::Mat pre_img;
  cv::Mat src_copy;
  cv::Mat dst_img;
  cv::Mat topview_matrix;
  cv::Mat feature_transform_matrix;
  cv::Mat robust_mat;
  vector<vector<float>> keypoint_2d;
  Point2f pre_key[4];
  Point2f src_key[4];
  bool activation;

  public:

  Birdeye() : it(nh)
  {
    
    topview_matrix = (cv::Mat_<float>(3,3) << 

    -8.74991570e-02, -1.54560676e+00, 1.10646406e+03, 
    9.49495267e-02, -2.11025424e+00, 1.39216464e+03, 
    8.51838817e-05, -1.63333434e-03, 1.00000000e+00

    // -4.94678689e-02, -1.41673089e+00,  6.71760437e+02,
    // -3.65263375e-14, -1.64154193e+00,  7.63394602e+02,
    // -6.66513286e-17, -2.20918221e-03,  1.00000000e+00
       
   // straight with curve bag file information of perspective matrix
   // -2.69066028e-01, -1.50593829e+00,  1.16355172e+03,
   // -7.42116798e-02, -2.09412460e+00,  1.37387381e+03,
   // -6.74651529e-05, -1.55858409e-03,  1.00000000e+00

    );

    pub = it.advertise("/output/birdeye",1);
    sub = nh.subscribe("/camera/image_color/compressed", 1, &Birdeye::compressedback, this);
  }

  cv::Mat Robust_TransformMatrix(vector<vector<float>> keypoint_2d_, int maximum_comb_number)
  {
    // obtain a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine e(seed);

    if(keypoint_2d_.size()==4) maximum_comb_number = 1;
    Eigen::MatrixXf measure_A = Eigen::MatrixXf::Zero(8,8);
    Eigen::VectorXf measure_B = Eigen::VectorXf::Zero(8);

    int numbering = 0;
    while( numbering < maximum_comb_number )
    {
      Eigen::MatrixXf measure_A_part(8,8);
      Eigen::VectorXf measure_B_part(8);

      shuffle(keypoint_2d_.begin(), keypoint_2d_.end(), e);
      for(vector<vector<float>>::iterator it=keypoint_2d_.begin(); it!=keypoint_2d_.begin() + 4; it++)
      {

        float tx = (*it)[0];
        float ty = (*it)[1];
        float x = (*it)[2];
        float y = (*it)[3];


        Eigen::VectorXf part_row(8);
        part_row << x, y, 1, 0, 0, 0, -tx*x, -tx*y;
        Eigen::VectorXf part_row2(8);
        part_row2 << 0, 0, 0, x, y, 1, -ty*x, -ty*y; 

        measure_A_part.row(2* (it-keypoint_2d_.begin())) = part_row;
        measure_A_part.row(2* (it-keypoint_2d_.begin())+1) = part_row2;
        measure_B_part(2* (it-keypoint_2d_.begin())) = tx;
        measure_B_part(2* (it-keypoint_2d_.begin())+1) = ty;
      }

      measure_A += measure_A_part;
      measure_B += measure_B_part;
      numbering++;
    }

    // Eigen::MatrixXf A = measure_A.transpose()*measure_A;
    // Eigen::VectorXf min_eigvec = A.inverse() * measure_A.transpose() *measure_B;

    Eigen::ColPivHouseholderQR<Eigen::MatrixXf> dec(measure_A/maximum_comb_number);
    Eigen::VectorXf min_eigvec = dec.solve(measure_B/maximum_comb_number);

    cout << "Error is " <<endl; 
    cout << measure_A * min_eigvec /maximum_comb_number - measure_B/maximum_comb_number <<endl; 
    cout << "-------------------------------------" <<endl;
    // Eigen::VectorXf min_eigvec = Eigen::VectorXf::Zero(8);
    // Eigen::EigenSolver<Eigen::MatrixXf> es(A);
    // vector<float> eig_mag;

    // for(register int ind=0; ind<9; ind++)
    // {
    //   eig_mag.push_back(es.eigenvalues().real()(ind));
    // }

    
    // std::vector<float>::iterator min_pointer = std::min_element(eig_mag.begin(), eig_mag.end());
    // cout << es.eigenvalues().real().transpose() <<endl;
    //     cout << eig_mag[min_pointer-eig_mag.begin()]<< endl;
    // Eigen::VectorXf min_eigvec = es.eigenvectors().col(8).real();
    // Eigen::VectorXf min_eigvec = V;
    // min_eigvec = min_eigvec / min_eigvec(8);
    // cout << measure_A * min_eigvec <<endl;



    cv::Mat robust_mat = (cv::Mat_<float>(3,3) << min_eigvec(0), min_eigvec(1), min_eigvec(2), 
                                                  min_eigvec(3), min_eigvec(4), min_eigvec(5), 
                                                  min_eigvec(6), min_eigvec(7), 1);

    // cout << "robust_mat" << endl;
    // cout << robust_mat << endl;
    // cout << "_----------------------------"<<endl;
    return robust_mat;
  }

  void compressedback(const sensor_msgs::CompressedImage::ConstPtr& msg)
  {

    // parsing current image
    cv::Mat src_img = cv::imdecode(cv::Mat(msg->data), 1);
      

    if (src_img.empty()) {
      ROS_ERROR("error: can not load image");
    }

    cv::warpPerspective(src_img, dst_img, topview_matrix, src_img.size(), cv::INTER_LINEAR);

    // Feature Extraction (ORB) and mathcing (BF)
    if(!pre_img.empty())
    {
        Ptr<ORB> detector = ORB::create(200);

        std::vector<KeyPoint> keypoints_pre, keypoints_src;
        Mat descriptors_pre, descriptors_src;        
        
        detector->detectAndCompute(pre_img, noArray(), keypoints_pre, descriptors_pre);
        detector->detectAndCompute(dst_img, noArray(), keypoints_src, descriptors_src);

        // feature matching of Brute Force;
        BFMatcher matcher;
        std::vector< DMatch > matches;
        vector< DMatch > good_matches;
        matcher.match( descriptors_pre, descriptors_src, matches);

        double max_dist = 0; double min_dist = 100;

        for( int i = 0; i < descriptors_pre.rows; i++ )
        { double dist = matches[i].distance;
          if( dist < min_dist ) min_dist = dist;
          if( dist > max_dist ) max_dist = dist;
        }
        
        for( int i = 0; i < descriptors_pre.rows; i++ )
        { if( matches[i].distance <= max(1.5*min_dist, 0.02) )
          { good_matches.push_back( matches[i]); }
        }

        std::sort(good_matches.begin(), good_matches.end(), [](const DMatch &a, const DMatch &b){return a.distance < b.distance;});

        int ind = 0;
        keypoint_2d.clear();

        for(std::vector<DMatch>::iterator it = good_matches.begin(); it!=good_matches.end(); it++)
          {
 
            // if(  keypoints_pre[(*it).queryIdx].pt.y < 420
            //   || keypoints_pre[(*it).queryIdx].pt.y > 750 ){continue;}

            if(abs(keypoints_pre[(*it).queryIdx].pt.x-keypoints_src[(*it).trainIdx].pt.x) > 2
              || abs(keypoints_pre[(*it).queryIdx].pt.y-keypoints_src[(*it).trainIdx].pt.y) > 2){continue;}

            vector<float> key_vec{keypoints_pre[(*it).queryIdx].pt.x, keypoints_pre[(*it).queryIdx].pt.y,
            keypoints_src[(*it).trainIdx].pt.x, keypoints_src[(*it).trainIdx].pt.y};
            keypoint_2d.push_back(key_vec);


            if(ind<=3)
            {
              pre_key[ind].x = keypoints_pre[(*it).queryIdx].pt.x;
              pre_key[ind].y = keypoints_pre[(*it).queryIdx].pt.y;
              src_key[ind].x = keypoints_src[(*it).trainIdx].pt.x;
              src_key[ind].y = keypoints_src[(*it).trainIdx].pt.y;
              ind++;
            }
            
          } // for end

        if(keypoint_2d.size() < 4 || ind <= 3)
        { 
          activation = false;
          cout << "no function "<<endl;
        }
        else
        {
          activation = true;

          cout <<"Size is : " <<good_matches.size()<< ", "<<keypoint_2d.size() <<endl;
          robust_mat = Birdeye::Robust_TransformMatrix(keypoint_2d, 50);

        } //else end

              namedWindow("before", WINDOW_NORMAL);
              imshow("before", dst_img);

              Mat img_matches;
              drawMatches( pre_img, keypoints_pre, dst_img, keypoints_src, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                     vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
              namedWindow("Matches", WINDOW_NORMAL);
              imshow("Matches", img_matches);

              
              if(activation)
              {
                
                cv::Mat lam = getPerspectiveTransform(src_key, pre_key);
                // cout << "lam"<<endl;
                // cout << lam<<endl;
                // cout << "-----------------------------------------"<<endl;

                warpPerspective(dst_img, src_copy, robust_mat, dst_img.size(), INTER_NEAREST);

              }
              else{
                src_copy = dst_img;
              }
          
          namedWindow("after", WINDOW_NORMAL);
          imshow("after", src_copy);        

          if( waitKey(30) == 27 ){
            destroyAllWindows();
            exit(0);
          }
  
    } // if end

    msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src_copy).toImageMsg();
    pub.publish(msg_);

    // image copy(src -> pre image)
    pre_img = dst_img;

  } //void compressedback end
}; //class end

int main(int argc, char **argv)
{ 
  
  ros::init(argc, argv, "image_publisher_subscriber");
  Birdeye bird;
  ros::spin();
  
}
