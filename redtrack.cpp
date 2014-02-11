#include <signal.h>
#include <cstdint>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mavconn.h>
#include <interface/shared_mem/SHMImageClient.h>


int sysid;
int compid;
bool verbose;

bool quit = false;

lcm_t* lcm;

struct timeval tv;

void
signalHandler(int signal)
{
	if (signal == SIGINT)
	{
		fprintf(stderr, "# INFO: Quitting...\n");
		quit = true;
		exit(EXIT_SUCCESS);
	}
}

double*
trackObject(cv::Mat imgThresh)
{
	cv::Moments moments;
	moments = cv::moments(imgThresh, 1); // Array of moments 
	double moment10 = moments.m10; // X value
	double moment01 = moments.m01; // Y value
	double area = moments.m00; // 0 value
	static double errorPos[2];
	static double errorX;
	static double errorY;

	double width = imgThresh.size().width;
	double height = imgThresh.size().height;

	if(area > 10) {
		static int posX = 0;
		static int posY = 0;
		posX = moment10/area; // Position of X value
		posY = moment01/area; // Position of Y value
		// Now we calculate distance from the centre of the image

		errorX = posX / width;
		errorY = posY / height;
	} else {
	//printf("\nObject not locked");
		errorX = 0.0;
		errorY = 0.0;
	}

	errorPos[0] = errorX;
	errorPos[1] = errorY;
	return errorPos;
}

void
sendError(double* error)
{
	mavlink_message_t msg;
  
	mavlink_debug_vect_t mavout;
	strcpy(mavout.name, "REDTRACKR");

	gettimeofday(&tv, NULL);

	mavout.x = (float) error[0];
	mavout.y = (float) error[1];
	mavout.time_usec = tv.tv_usec;
	
  	mavlink_msg_debug_vect_encode(getSystemID(), 88, &msg, &mavout);
	sendMAVLinkMessage(lcm, &msg);
}

/* Image processing */
void 
process(cv::Mat img) 
{
	cv::GaussianBlur(img, img, cv::Size(3, 3), 0, 0);  

	cv::Mat imgHSV, imgThresh;
      	cvtColor(img, imgHSV, CV_BGR2HSV);
	
	cv::inRange(imgHSV, cv::Scalar(170,160,60,0), cv::Scalar(180,256,256,0), imgThresh);
      	cv::GaussianBlur(imgThresh, imgThresh, cv::Size(3, 3), 0, 0);
    
	double* errorMAV = trackObject(imgThresh); // Calls F-4
	sendError(errorMAV);

	cv::namedWindow("Thresh Image");
	cv::imshow("Thresh Image", imgThresh);
}


/**
 * @brief Handle incoming MAVLink packets containing images
 *
 */
void
imageHandler(const lcm_recv_buf_t* rbuf, const char* channel,
			 const mavconn_mavlink_msg_container_t* container, void* user)
{
	const mavlink_message_t* msg = getMAVLinkMsgPtr(container);

	// Pointer to shared memory data
	std::vector<px::SHMImageClient>* clientVec =
			reinterpret_cast< std::vector<px::SHMImageClient>* >(user);

	cv::Mat imgToSave;

	for (size_t i = 0; i < clientVec->size(); ++i)
	{
		px::SHMImageClient& client = clientVec->at(i);

		if ((client.getCameraConfig() & px::SHMImageClient::getCameraNo(msg)) != px::SHMImageClient::getCameraNo(msg))
		{
			continue;
		}

		cv::Mat img;
		if (client.readMonoImage(msg, img))
		{
			cv::namedWindow("Received Image");
			cv::imshow("Received Image", img);
			process(img);
		}
	}
	
	// This is critical
	cv::waitKey(3);
}

int main(int argc, char* argv[])
{
	lcm = lcm_create("udpm://");
	if (!lcm)
	{
		fprintf(stderr, "# ERROR: Cannot initialize LCM.\n");
		exit(EXIT_FAILURE);
	}

	std::vector<px::SHMImageClient> clientVec;
	clientVec.resize(4);

	clientVec.at(0).init(true, px::SHM::CAMERA_FORWARD_LEFT);
	clientVec.at(1).init(true, px::SHM::CAMERA_FORWARD_LEFT, px::SHM::CAMERA_FORWARD_RIGHT);
	clientVec.at(2).init(true, px::SHM::CAMERA_DOWNWARD_LEFT);
	clientVec.at(3).init(true, px::SHM::CAMERA_DOWNWARD_LEFT, px::SHM::CAMERA_DOWNWARD_RIGHT);

	// Ready to roll
	fprintf(stderr, "# INFO: Image handler loaded\n");

	// Subscribe to MAVLink messages on the image channel
	mavconn_mavlink_msg_container_t_subscription_t* imgSub = mavconn_mavlink_msg_container_t_subscribe(lcm, MAVLINK_IMAGES, &imageHandler, &clientVec);

	signal(SIGINT, signalHandler);

	while (!quit)
	{
		// Block waiting for new image messages,
		// once an image is received it will be
		// displayed
		lcm_handle(lcm);
	}

	mavconn_mavlink_msg_container_t_unsubscribe(lcm, imgSub);
	lcm_destroy(lcm);

	return 0;
}
