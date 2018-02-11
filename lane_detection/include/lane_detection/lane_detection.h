#ifndef LANE_DETECTION
#define LANE_DETECTION

#define all(v) (v).begin(),(v).end()

using namespace cv;
using namespace std;

struct Line{
    float ml, cl, mr, cr;
    bool flag;
};

Scalar YELLOW_LOW(10, 0, 100);
Scalar YELLOW_HIGH(40, 255, 255);
Scalar WHITE_LOW(0, 200, 0);
Scalar WHITE_HIGH(255, 255, 255);

Mat empty_image(Mat &frame){
    return Mat::zeros(frame.rows, frame.cols, CV_8UC3);
}

Mat filter_yw(Mat &frame){
    Mat mask_white, mask_yellow;
    cvtColor(frame, mask_white, COLOR_BGR2HLS);
    cvtColor(frame, mask_yellow, COLOR_BGR2HLS);

    inRange(mask_yellow, YELLOW_LOW, YELLOW_HIGH, mask_yellow);
    inRange(mask_white, WHITE_LOW, WHITE_HIGH, mask_white);

    Mat frame_mask,frame_masked;
    bitwise_or(mask_yellow, mask_white, frame_mask);
    return frame_mask;
}

void blur_canny(Mat &frame){
    GaussianBlur(frame, frame, Size(5,5), 0, 0);
    Canny(frame, frame, 100, 150, 3);
}

float find_slope(Vec4i coords){
    return (float)(coords[3] - coords[1])/(float)(coords[2] - coords[0]);
}

float find_intercept(Vec4i coords, float slope){
    return coords[1] - slope*coords[0];
}

pair<float, float> line_equation(const pair<int,int> &x, const pair<int,int> &y){
    pair<float, float> param;
    param.first = (float)(y.second - y.first)/(x.second - x.first);
    param.second = param.first*x.first;
    return param;
}

void hough_lines(Mat &edges, Mat &lines_image, Line &lines){
    vector<Vec4i> hough_lines;
    HoughLinesP(edges, hough_lines, 1, CV_PI/180, 30, 100, 180);
    Point p1,p2;

    vector<pair<float,float> > left_lines;
    vector<pair<float,float> > right_lines;
    vector<float> left_length;
    vector<float> right_length;

    int size = hough_lines.size();

    for(size_t i = 0; i < hough_lines.size(); i++){
        Vec4i l = hough_lines[i];
        p1 = Point(l[0], l[1]);
        p2 = Point(l[2], l[3]);
        float slope = find_slope(l);
        float intercept = find_intercept(l, slope);
        float length = cv::norm(p2 - p1);
        if(slope<0){
            left_lines.push_back(make_pair(slope, intercept));
            left_length.push_back(length);
        }else if(slope>0){
            right_lines.push_back(make_pair(slope, intercept));
            right_length.push_back(length);
        }
    }

    if(left_lines.size()==0 || right_lines.size()==0){
        lines.flag = false;
        return;
    }else{
        float l_length = std::accumulate(all(left_length), 0.0);
        float r_length = std::accumulate(all(right_length), 0.0);

        pair<float, float> left_lane;
        pair<float, float> right_lane;
        for(int i=0;i<left_lines.size();i++){
            left_lane.first += left_length[i]*left_lines[i].first;
            left_lane.second += left_length[i]*left_lines[i].second;
        }
        for(int i=0;i<right_lines.size();i++){
            right_lane.first += right_length[i]*right_lines[i].first;
            right_lane.second += right_length[i]*right_lines[i].second;
        }
        left_lane.first /= l_length;
        left_lane.second /= l_length;
        right_lane.first /= r_length;
        right_lane.second /= r_length;

        int y1 = edges.rows;
        int y2 = 0.60*edges.rows;

        pair<int,int> l_x,r_x;
        l_x.first = (y1 - left_lane.second)/left_lane.first;
        r_x.first = (y1 - right_lane.second)/right_lane.first;
        l_x.second = (y2 - left_lane.second)/left_lane.first;
        r_x.second = (y2 - right_lane.second)/right_lane.first;

        line(lines_image, Point(l_x.first, y1), 
            Point(l_x.second, y2), Scalar(0,255,0), 4, LINE_AA);
        line(lines_image, Point(r_x.first, y1), 
            Point(r_x.second, y2), Scalar(0,255,0), 4, LINE_AA);

        pair<float,float> left_params = line_equation(l_x, make_pair(y1, y2));
        pair<float,float> right_params = line_equation(r_x, make_pair(y1, y2));

        lines.flag = true;

        lines.ml = left_params.first;
        lines.cl = left_params.second;

        lines.mr = right_params.first;
        lines.cr = right_params.second;
    }
}

void select_roi(Mat &frame){
    int rows = frame.rows;
    int cols = frame.cols;

    Point points[1][4];
    points[0][0] = Point(cols*0.15, rows*0.95);
    points[0][1] = Point(cols*0.4, rows*0.6);
    points[0][2] =  Point(cols*0.55, rows*0.6);
    points[0][3] = Point(cols*0.90, rows*0.95);

    Mat img = empty_image(frame);

    const Point* ppt[1] = {points[0]};
    int npt[] = {4};
    fillPoly(img, ppt, npt, 1, Scalar(255,255,255), 8);
    cvtColor(img, img, COLOR_BGR2GRAY);
    bitwise_and(frame, img, frame);
}

void lane_detect(Mat &frame, Line &l){
    Mat frame_copy = frame;
    frame_copy = filter_yw(frame_copy);

    blur_canny(frame_copy);
    select_roi(frame_copy);

    Mat lines_image = empty_image(frame);
    hough_lines(frame_copy, lines_image, l);
    addWeighted(frame, 1.0, lines_image, 1.0, 0.0,frame);
}

#endif