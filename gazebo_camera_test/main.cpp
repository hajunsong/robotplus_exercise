#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <mutex>
#include <thread>


void read_camera_parameters(double cmtx[9], double dist[5]){
    std::ifstream cam_param_file("../camera_parameters/intrinsic.dat");
    std::string line, temp_string;
    int indx = 0;
    
    if(cam_param_file.is_open()){
        std::cout << "file opened successfully" << std::endl;
        while(std::getline(cam_param_file, line)){
            // std::cout << line << std::endl;
            std::istringstream iss(line);
            if(indx >= 1 && indx <= 3){
                iss >> cmtx[(indx-1)*3 + 0] >> cmtx[(indx-1)*3 + 1] >> cmtx[(indx-1)*3 + 2] >> temp_string;
            }
            else if(indx == 5){
                iss >> dist[0] >> dist[1] >> dist[2] >> dist[3] >> dist[4] >> temp_string;
            }
            indx++;
        }
    }
    else{
        std::cout << "Unable to open file" << std::endl;
    }
}

void get_qr_coords(cv::Mat cmtx, cv::Mat dist, cv::Mat points, std::vector<cv::Point2d> &axis_points, cv::Mat &rvec, cv::Mat &tvec){
    // Selected coordinate points for each corner of QR code.
    std::vector<cv::Point3d> qr_edges{
        cv::Point3d(0, 0, 0), 
        cv::Point3d(0, 1, 0), 
        cv::Point3d(1, 1, 0), 
        cv::Point3d(1, 0, 0)
    };

    // Determine the orientation of QR code coordinate system with respect to camera coordinates system.
    bool ret = cv::solvePnP(qr_edges, points, cmtx, dist, rvec, tvec);

    std::vector<cv::Point3d> unitv_points{
        cv::Point3d(0, 0, 0),
        cv::Point3d(1, 0, 0),
        cv::Point3d(0, 1, 0),
        cv::Point3d(0, 0, 1)
    };

    if(ret){
        cv::projectPoints(unitv_points, rvec, tvec, cmtx, dist, axis_points);
    }
}

void show_axes(cv::Mat frame, cv::Mat cmtx, cv::Mat dist){
    cv::Mat points;

    cv::QRCodeDetector qr;

    // cv::Mat axis_points;
    std::vector<cv::Point2d> axis_points;
    cv::Mat rvec(3, 1, cv::DataType<double>::type);
    cv::Mat tvec(3, 1, cv::DataType<double>::type);

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Mat img_threshold, img_bw;
    cv::threshold(gray, img_threshold, 5, 255, cv::THRESH_BINARY);

    bool ret_qr = qr.detect(img_threshold, points);
    // cv::imshow("bynary", img_threshold);
    // std::cout << "ret_qr : " << ret_qr << std::endl;

    if(ret_qr){
        get_qr_coords(cmtx, dist, points, axis_points, rvec, tvec);
        std::cout << "axis points : " << axis_points << std::endl;
        // std::cout << "rvec : " << rvec.t() << std::endl;
        // std::cout << "tvec : " << tvec.t() << std::endl;

        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);

        std::cout <<  "rmat : " << std::endl;
        std::cout << rmat << std::endl;

        std::cout << "rvec : " << std::endl;
        std::cout << rvec << std::endl;

        std::vector<cv::Scalar> colors{
            // BGR color format
            cv::Scalar(0, 0, 255), // Red
            cv::Scalar(0, 255, 0), // Green
            cv::Scalar(255, 0, 0)  // Blue
        };

        // check axes points are projected to camera view.
        if(axis_points.size() > 0){
            cv::Point2i origin(axis_points[0]);

            for(int i = 1; i < axis_points.size(); i++){
                cv::Point2i p(axis_points[i]);

                cv::line(frame, origin, p, colors[i - 1], 5);
            }
        }
    }
}


// 전역 변수로 최근 수신한 이미지를 저장
cv::Mat g_image;
std::mutex g_image_mutex;

// 이미지 콜백 함수 (Gazebo 메세지가 수신될 때 호출됨)
void imageCallback(ConstImageStampedPtr &msg)
{
    // Gazebo 이미지 메시지로부터 width, height 정보 획득
    int width = msg->image().width();
    int height = msg->image().height();

    // std::cout << "width : " << width << std::endl;
    // std::cout << "height : " << height << std::endl;

    // 이미지 데이터가 RGB8 형식(3채널)라고 가정
    // 데이터를 안전하게 복사하기 위해 임시 cv::Mat 생성
    cv::Mat rgbImage(height, width, CV_8UC3);
    // 문자열 형태로 저장된 이미지 데이터를 메모리로 복사
    size_t expectedSize = width * height * 3;
    if (msg->image().data().size() < expectedSize)
    {
        std::cerr << "이미지 데이터 크기가 예상보다 작습니다." << std::endl;
        return;
    }
    memcpy(rgbImage.data, msg->image().data().c_str(), expectedSize);

    // OpenCV는 기본적으로 BGR 순서이므로, RGB에서 BGR로 변환
    cv::Mat bgrImage;
    cv::cvtColor(rgbImage, bgrImage, cv::COLOR_RGB2BGR);

    // 전역 변수에 업데이트 (뮤텍스를 사용하여 스레드 안전하게)
    {
        std::lock_guard<std::mutex> lock(g_image_mutex);
        g_image = bgrImage.clone();
    }
}

int main(int argc, char **argv)
{
    // Gazebo 클라이언트로서 초기화 (서버에 접속)
    gazebo::client::setup(argc, argv);

    // Transport 노드를 생성 및 초기화
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // 구독할 토픽 설정 (여기서는 예시로 "~/camera/image"로 지정)
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/robot/link6/Realsense/image", imageCallback);

    // cv::Mat mat = cv::imread("1.jpg");
    // cv::imshow("Gazebo Camera", mat);
    // cv::waitKey(0);

    std::cout << "이미지 데이터를 구독 중입니다. 화면을 보려면 창을 활성화 시키세요." << std::endl;
    std::cout << "종료하려면 ESC 키를 누르세요." << std::endl;

    // OpenCV 창 생성
    // cv::namedWindow("Gazebo Camera", cv::WINDOW_AUTOSIZE);

    double param_cmtx[9], param_dist[5];
    
    read_camera_parameters(param_cmtx, param_dist);

    cv::Mat cmtx(3, 3, cv::DataType<double>::type, param_cmtx);
    cv::Mat dist(5, 1, cv::DataType<double>::type, param_dist);

    // 메인 루프 : 전역 변수에 업데이트된 이미지가 있으면 화면 출력
    while (true)
    {
        cv::Mat frame, result;
        {
            std::lock_guard<std::mutex> lock(g_image_mutex);
            if (!g_image.empty()){
                frame = g_image.clone();
            
                show_axes(frame, cmtx, dist);
                // cv::Mat points;

                // cv::QRCodeDetector qr;

                // // cv::Mat axis_points;
                // std::vector<cv::Point2d> axis_points;
                // cv::Mat rvec(3, 1, cv::DataType<double>::type);
                // cv::Mat tvec(3, 1, cv::DataType<double>::type);

                // cv::Mat gray;
                // cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                // cv::Mat img_threshold, img_bw;
                // cv::threshold(gray, img_threshold, 5, 255, cv::THRESH_BINARY);

                // bool ret_qr = qr.detect(img_threshold, points);
                // // cv::imshow("bynary", img_threshold);
                // // std::cout << "ret_qr : " << ret_qr << std::endl;

                // if(ret_qr){
                //     get_qr_coords(cmtx, dist, points, axis_points, rvec, tvec);
                //     // std::cout << "axis points : " << axis_points << std::endl;
                //     // std::cout << "rvec : " << rvec.t() << std::endl;
                //     // std::cout << "tvec : " << tvec.t() << std::endl;

                //     cv::Mat rmat;
                //     cv::Rodrigues(rvec, rmat);
                //     std::cout <<  "rmat : " << std::endl;
                //     std::cout << rmat << std::endl;

                //     std::vector<cv::Scalar> colors{
                //         // BGR color format
                //         cv::Scalar(255, 0, 0),
                //         cv::Scalar(0, 255, 0),
                //         cv::Scalar(0, 0, 255),
                //         cv::Scalar(0, 0, 0)
                //     };

                //     // check axes points are projected to camera view.
                //     if(axis_points.size() > 0){
                //         cv::Point2i origin(axis_points[0]);

                //         for(int i = 1; i < axis_points.size(); i++){
                //             cv::Point2i p(axis_points[i]);

                //             cv::line(frame, origin, p, colors[i], 5);
                //         }
                //     }
                // }
            }
        }

        if (!frame.empty())
        {
            cv::imshow("Gazebo Camera", frame);
            // 1ms 동안 대기하며, ESC 키(27)가 입력되면 종료
            if (cv::waitKey(1) == 27)
                break;
        }
        // Gazebo 메시지 처리를 위한 잠시 대기
        gazebo::common::Time::MSleep(10);
    }

    // 자원 정리
    gazebo::client::shutdown();
    cv::destroyAllWindows();
    return 0;
}
