#include <ros/ros.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/GPSRAW.h>
#include "/usr/include/yaml-cpp/yaml.h"
#include <iostream>
#include <vector>
#include <sensor_msgs/Imu.h>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <math.h>
#include <tf/tf.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <thread>
#include <mutex>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>

#define PI 3.14159265358979323846

double* wgs84ToEcef(double lat, double lon, double h) 
{
		double a = 6378137;
		double b = 6356752.3142;
		double f = (a - b) / a;
		double e_sq = f * (2 - f);
		double lamb = (PI/180)*lat;
		double phi = (PI/180)*lon;
		double s = sin(lamb);
		double N = a / sqrt(1 - e_sq * s * s);
 
		double sin_lambda = sin(lamb);
		double cos_lambda = cos(lamb);
		double sin_phi = sin(phi);
		double cos_phi = cos(phi);
 
		double x = (h + N) * cos_lambda * cos_phi;
		double y = (h + N) * cos_lambda * sin_phi;
		double z = (h + (1 - e_sq) * N) * sin_lambda;
        // *result = x;
        // *(result + 1) = y;
        // *(result + 2) = z;
		return new double[3]{x,y,z};
}

double* ecefToEnu(double x, double y, double z, double lat, double lng, double height) 
{
		double a = 6378137;
		double b = 6356752.3142;
		double f = (a - b) / a;
		double e_sq = f * (2 - f);
		double lamb = (PI/180)*lat;
		double phi = (PI/180)*lng;
		double s = sin(lamb);
		double N = a / sqrt(1 - e_sq * s * s);
		double sin_lambda = sin(lamb);
		double cos_lambda = cos(lamb);
		double sin_phi = sin(phi);
		double cos_phi = cos(phi);
 
		double x0 = (height + N) * cos_lambda * cos_phi;
		double y0 = (height + N) * cos_lambda * sin_phi;
		double z0 = (height + (1 - e_sq) * N) * sin_lambda;
 
		double xd = x - x0;
		double yd = y - y0;
		double zd = z - z0;
 
		double t = -cos_phi * xd - sin_phi * yd;
 
		double xEast = -sin_phi * xd + cos_phi * yd;
		double yNorth = t * sin_lambda + cos_lambda * zd;
		double zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;
		return new double[3] { xEast, yNorth, zUp };
	}

void wgs84_to_enu(std::vector<std::vector<double>> point_wgs84, std::vector<std::vector<double>> &point){

        std::cout << "USE WGS84 POINTS" << std::endl;
        int num_point = point_wgs84.size();
        point.clear();

        for(int i=0;i<num_point;i++){
            double *arr_ecef;
            double *arr_enu;
            arr_ecef = wgs84ToEcef(point_wgs84[i][0], point_wgs84[i][1], point_wgs84[i][2]);
            arr_enu = ecefToEnu(arr_ecef[0], arr_ecef[1], arr_ecef[2], point_wgs84[0][0], point_wgs84[0][1], point_wgs84[0][2]);
            std::vector<double> vector_enu(arr_enu, arr_enu+3);
            vector_enu[2] = point_wgs84[i][3];
            point.push_back(vector_enu);
        }
        for(int i=0;i<num_point;i++){
            std::cout << "point_enu " + std::to_string(i) + ": [ ";
            for( int i2 = 0; i2 < 3; i2++){
                std::cout << point[i][i2] << " ";
            }
            std::cout << "]" << std::endl;
        }
    
}

double* rotate_points_from_mag(double x, double y, double z, double yaw_mag_NED){

    //rotate local position
    Eigen::Matrix3f R;
    Eigen::Vector3f point_fixed;
    R <<  cos(-yaw_mag_NED), sin(-yaw_mag_NED),   0, 
         -sin(-yaw_mag_NED), cos(-yaw_mag_NED),   0, 
          0,                 0,                   1;

    point_fixed.segment(0,1) << x;
    point_fixed.segment(1,1) << y;
    point_fixed.segment(2,1) << 1;

    point_fixed = R * point_fixed;

    return new double[3]{point_fixed[0],point_fixed[1], z};
}

double* enu_to_wgs84(double x_east, double y_north, double z_up, double lati, double longti,double height, double yaw_mag_NED)
{
    double *arr_enu;
    arr_enu = rotate_points_from_mag(x_east, y_north, z_up, -yaw_mag_NED);
    x_east = arr_enu[0];
    y_north = arr_enu[1];
    delete arr_enu;
	// printf("enu_location: %f, %f \n", x_east, y_north);

	// double pi = std::acos(-1.0);
	double a = 6378137.0;    //earth radius in meters
	double b = 6356752.3142; //earth semiminor in meters 
	double f = (a - b) / a;
	double e_sq = f * (2-f);
 
	double lamb = lati * M_PI / 180.0;
	double phi = longti * M_PI / 180.0;
	double sl = std::sin(lamb);
	double N = a / std::sqrt(1 - e_sq * sl * sl);
	double sin_lambda = std::sin(lamb);
	double cos_lambda = std::cos(lamb);
	double sin_phi = std::sin(phi);
	double cos_phi = std::cos(phi);
	double x0 = (height + N) * cos_lambda * cos_phi;
	double y0 = (height + N) * cos_lambda * sin_phi;
	double z0 = (height + (1 - e_sq) * N) * sin_lambda;
	double t = cos_lambda * z_up - sin_lambda * y_north;
	double zd = sin_lambda * z_up + cos_lambda * y_north;
	double xd = cos_phi * t - sin_phi * x_east;
	double yd = sin_phi * t + cos_phi * x_east;
	
	//Convert from ECEF cartesian coordinates to 
	//latitude, longitude and height.  WGS-8
	double x = xd + x0;
	double y = yd + y0;
	double z = zd + z0;
	double x2 = std::pow(x, 2);
	double y2 = std::pow(y, 2);
	double z2 = std::pow(z, 2);
	double e = std::sqrt (1-std::pow((b/a) , 2));
	double b2 = b*b;
	double e2 = e*e;
	double ep = e*(a/b); 
	double r = std::sqrt(x2+y2); 
	double r2 = r*r;
	double E2 = a*a - b*b;
	double F = 54*b2*z2;
	double G = r2 + (1-e2)*z2 - e2*E2;
	double c = (e2*e2*F*r2)/(G*G*G); 
	double s = std::pow(( 1 + c + std::sqrt(c*c + 2*c) ) ,(1/3)); 
	double P = F / (3 * std::pow((s+1/s+1), 2) * G*G); 
	double Q = std::sqrt(1+2*e2*e2*P);
	double ro = -(P*e2*r)/(1+Q) + std::sqrt((a*a/2)*(1+1/Q) - (P*(1-e2)*z2)/(Q*(1+Q)) - P*r2/2);
	double tmp = std::pow((r - e2*ro), 2); 
	double U = std::sqrt( tmp + z2 ); 
	double V = std::sqrt( tmp + (1-e2)*z2 ); 
	double zo = (b2*z)/(a*V); 
	double high = U*( 1 - b2/(a*V) );
	double lat = std::atan( (z + ep*ep*zo)/r );
	double temp = std::atan(y/x);
	double lon = temp - M_PI;
	if (x >= 0) {
		lon = temp;
	}
	else if (x < 0 && y >= 0) {
		lon = M_PI + temp;
	}

	// *result = lat/(pi/180);
	// *(result + 1) = lon/(pi/180);
	// *(result + 2) = high;
	// printf("%f %f/n", lat/(M_PI/180), lon/(M_PI/180));
    return new double[3] { lat/(M_PI/180), lon/(M_PI/180), high };
}
