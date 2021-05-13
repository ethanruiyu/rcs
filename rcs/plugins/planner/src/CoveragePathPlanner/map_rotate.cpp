//
// Created by tuqirui on 21-1-12.
//

#include "CoveragePathPlanner/map_rotate.h"

float cal_rat_angle(cv::Mat map, std::string map_path)
{
    // Parameter configuration

    //find contours
    std::vector<std::vector<cv::Point>> obstacle_contours;
    std::vector<std::vector<cv::Point>> wall_contours;
    //cv::imshow("aa",src);
    //cv::waitKey(0);

    cv::Mat edges, dst;
//    cv::cvtColor(src,dst,CV_BGR2GRAY);
    cv::Canny(map, edges, 50, 150, 3);
    std::vector<Vec4f> lines;
    //std::vector<std::vector<float>> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI/180, 50, 20, 20);

    cv::Mat maprgb;
    cvtColor(map, maprgb, cv::COLOR_GRAY2BGR);
    for(auto line : lines)
    {
        float x0, y0, x1, y1;
        x0 = line[0];
        y0 = line[1];
        x1 = line[2];
        y1 = line[3];
        cv::Point p1 = {int(x0),int(y0)};
        cv::Point p2 = {int(x1),int(y1)};
        cv::line(maprgb, p1, p2, cv::Scalar(127, 0, 255), 5);
    }


    float mink;
//    std::vector<float> linek;
//    for(auto line : lines)
//    {
//        float x0, y0, x1, y1;
//        x0 = line[0];
//        y0 = line[1];
//        x1 = line[2];
//        y1 = line[3];
//        //x0, y0, x1, y1 = line;
//        linek.push_back((y1 - y0)/(x1 - x0));
//    }
//    std::vector<float> verticalk;
//    for(float i : linek)
//    {
//        //std::vector<float> temp = i * linek;
//        std::vector<float> temp;
//        for(float n : linek)
//        {
//            temp.push_back(i * n);
//        }
//        for(float j : temp)
//        {
//            if (0.9 < abs(j) && abs(j) < 1.1)
//                verticalk.push_back(i);
//        }
//    }
//    if(verticalk.size() != 0)
//    {
//        auto iter = std::min_element(std::begin(verticalk),std::end(verticalk));
//        mink = *iter;
//    }
    if(lines.empty())
    {
        return 0;
    }
    else
    {
        std::vector<float> line_length;
        for(auto line : lines)
        {
            float x2, y2, x3, y3;
            x2 = line[0];
            y2 = line[1];
            x3 = line[2];
            y3 = line[3];
            //x2, y2, x3, y3 = line;
            float length = (y3 - y2)*(y3 - y2) + (x3 - x2)*(x3 - x2);
            line_length.push_back(length);
        }
        auto iter1 = std::max_element(line_length.begin(),line_length.end());
        //float longest = *iter1;
        int index1 = std::distance(std::begin(line_length), iter1);
        float x4, y4, x5, y5;
        x4 = lines[index1][0];
        y4 = lines[index1][1];
        x5 = lines[index1][2];
        y5 = lines[index1][3];
        //x4, y4, x5, y5 = temp1;
        mink = (y5 - y4)/(x5 - x4);

        cv::Point p3 = {int(x4),int(y4)};
        cv::Point p4 = {int(x5),int(y5)};
        cv::line(maprgb, p3, p4, cv::Scalar(255, 0, 0), 5);
    }
    float a = std::atan(mink);
    float angle = (a/CV_PI) * 180;

    cv::imwrite(map_path + "/edge.png", maprgb);
    return angle;
}

void rotate_map(cv::Mat coverage_origin, cv::Mat& coverage_rat, float angle)
{
    cv::Mat img_origin = coverage_origin.clone();
    threshold(img_origin,img_origin,0,255,cv::THRESH_BINARY); //(0, 1)二值图 转成 （0, 255）二值图
    cv::bitwise_not(img_origin, img_origin);

    cv::Mat img_origin_rgb;
    cv::cvtColor(img_origin, img_origin_rgb, CV_GRAY2BGR);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img_origin, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    for(int i=0; i<contours.size(); i++)
    {
        cv::drawContours(img_origin_rgb,contours, i, cv::Scalar(255, 0, 0), 3, 8, hierarchy);
    }
    cv::drawContours(img_origin_rgb,contours, 0, cv::Scalar(255, 0, 0), 3, 8, hierarchy);
//    cv::imshow("img_origin", img_origin_rgb);
//    cv::waitKey(0);

    std::vector<std::vector<cv::Point>> contours_poly_list;
    for(auto contour : contours)
    {
        std::vector<cv::Point> contours_poly;
        cv::approxPolyDP(contour, contours_poly, 5, true);
        contours_poly_list.push_back(contours_poly);
    }
    for(auto contour_polygon : contours_poly_list)
    {
        for(auto vertex : contour_polygon)
        {
            cv::circle(img_origin_rgb, vertex, 10, cv::Scalar(0, 0, 255));
        }
    }
//    cv::imshow("img_origin2", img_origin_rgb);
//    cv::waitKey(0);


    cv::Point point_rat;

    std::vector<std::vector<cv::Point>> contours_poly_list_rat;
    cv::Point2f center_rat( (float)(img_origin.cols/2) , (float) (img_origin.rows/2));
//    float a = (angle/180)*CV_PI;
    cv::Mat rat_matrix = getRotationMatrix2D( center_rat, -angle, 1.0 );
    for(auto contour_polygon : contours_poly_list)
    {
        std::vector<cv::Point> contour_polygon_rat;
        for(auto vertex : contour_polygon)
        {
            point_rat.x = rat_matrix.at<double>(0,0) * vertex.x + rat_matrix.at<double>(0,1) * vertex.y + rat_matrix.at<double>(0,2) - (img_origin.cols/2 - img_origin.cols/2);
            point_rat.y = rat_matrix.at<double>(1,0) * vertex.x + rat_matrix.at<double>(1,1) * vertex.y + rat_matrix.at<double>(1,2) - (img_origin.rows/2 - img_origin.rows/2);
            contour_polygon_rat.push_back(point_rat);
        }
        contours_poly_list_rat.push_back(contour_polygon_rat);
    }

    for(auto contour_polygon : contours_poly_list_rat)
    {
        for(auto vertex : contour_polygon)
        {
            cv::circle(img_origin_rgb, vertex, 10, cv::Scalar(0, 255, 0));
        }
    }
//    cv::imshow("img_origin3", img_origin_rgb);
//    cv::waitKey(0);

    cv::Mat img_rat = cv::Mat(img_origin.rows, img_origin.cols, CV_8UC1, cv::Scalar(255));
    for(int i=0; i<contours_poly_list_rat.size(); i++)
    {
        if(hierarchy[i][3] != -1) //如果存在父轮廓 则多边形是虚拟墙
        {
            int npt1;
            npt1 = contours_poly_list_rat[i].size();
            cv::Point coverage_area[1][npt1];
            for(int j=0; j<contours_poly_list_rat[i].size(); j++)
            {
                coverage_area[0][j] = contours_poly_list_rat[i][j];
            }
            const cv::Point* ppt[1]={coverage_area[0]};
            int npt[]={npt1};
            cv::fillPoly(img_rat, ppt, npt, 1, cv::Scalar(0), 0);
        }
    }
    coverage_rat = cv::Mat::zeros(img_origin.size(), CV_8UC1);
    for(int i=0; i<contours_poly_list_rat.size(); i++)
    {
        if(hierarchy[i][3] == -1) //如果不存在父轮廓 则多边形是清扫区域
        {
            cv::Mat mask = cv::Mat(img_origin.rows, img_origin.cols, CV_8UC1, cv::Scalar(0));
            int npt1;
            npt1 = contours_poly_list_rat[i].size();
            cv::Point coverage_area[1][npt1];
            for(int j=0; j<contours_poly_list_rat[i].size(); j++)
            {
                coverage_area[0][j] = contours_poly_list_rat[i][j];
            }

            const cv::Point* ppt[1]={coverage_area[0]};
            int npt[]={npt1};
            cv::fillPoly(mask, ppt, npt, 1, cv::Scalar(255), 0);
            img_rat.copyTo(coverage_rat, mask);
        }
    }
    cv::bitwise_not(coverage_rat, coverage_rat);
    threshold(coverage_rat,coverage_rat,0,1,cv::THRESH_BINARY); //(0, 255)二值图 转成 （0, 1）二值图
//    cv::imshow("coverage_rat", coverage_rat);
//    cv::waitKey(0);

}

void rotate_arbitrarily_angle(cv::Mat &src,cv::Mat &dst,float angle)
{


    int maxBorder =(int) (cv::max(src.cols, src.rows)* 1.414 ); //即为sqrt(2)*max
    int dx = (maxBorder - src.cols)/2;
    int dy = (maxBorder - src.rows)/2;
    cv::Mat dst_1, dst_1_visual;
    copyMakeBorder(src, dst_1, dy, dy, dx, dx, cv::BORDER_CONSTANT, cv::Scalar(1));

    cv::Mat img_rotate;
    rotate_map(dst_1, img_rotate, angle);
    dst = img_rotate;
}

std::vector<cv::Point> path_transform(std::vector<cv::Point> path, cv::Mat1b map_transform, cv::Mat1b map_original, double angle)
{
    cv::Point point_rat;
    std::vector<cv::Point> path_rat;
    cv::Point2f center_rat( (float)(map_transform.cols/2) , (float) (map_transform.rows/2));
//    float a = (angle/180)*CV_PI;
    cv::Mat rat_matrix = getRotationMatrix2D( center_rat, -angle, 1.0 );
    for(cv::Point point : path)
    {
        point_rat.x = rat_matrix.at<double>(0,0) * point.x + rat_matrix.at<double>(0,1) * point.y + rat_matrix.at<double>(0,2) - (map_transform.cols/2 - map_original.cols/2);
        point_rat.y = rat_matrix.at<double>(1,0) * point.x + rat_matrix.at<double>(1,1) * point.y + rat_matrix.at<double>(1,2) - (map_transform.rows/2 - map_original.rows/2);
        path_rat.push_back(point_rat);
    }
    return path_rat;
}
