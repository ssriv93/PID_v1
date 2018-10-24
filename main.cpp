#include <iostream>
#include "PID.h"
#include "PID.cpp"
#include <math.h>
#include <boost/foreach.hpp>
#include <stdio.h>
#include <iomanip>

using namespace std;

//GPS Location from the global planner - file reading is giving error in ROS hence use as inline.
double gps_map[][2] = {
{1,0
},{1,1
},{1,2
},{1,3
},{1,4
},{1,5
},{2,5
},{2,4
},{2,3
},{2,2
},{2,1
},{2,0
},{3,0
},{3,1
},{3,2
},{3,3
},{3,4
},{3,5
}};

//Fake curr Data - This array will be removed.
double curr_poss_fake_array[][2] = {
{1.1,0
},{1,1
},{1,2
},{1,3
},{1,4
},{1,5
},{2,5
},{2,4
},{2,3
},{2,2
},{2,1
},{2,0
},{3,0
},{3,1
},{3,2
},{3,3
},{3,4
},{3,5
}};

//Number of points in Global Planner.
const int len_of_points = sizeof(gps_map)/sizeof(gps_map[0]);

//New array with same Lat and Long from Global Planner and additional theta value - theta will be the angle of line segment from current point to next point in global map.
double map_1[len_of_points][3];

//3D Array for line end points. x1,y1 and x2,y2 pair for each line
double set_of_lines[len_of_points][2][2]; // the max number of lines can not be more than len_of_points

//this will keep the count of number of lines in global planner
int line_counter = 0;

double global_vr=5; //to-do Tuning and putting correct value here
double global_vl=5; //to-do Tuning and putting correct value here

//this function will fill "map_1" array with latitude longitude and theta
void make_follow_path(){

    double first_x = gps_map[0][0] ;
    double first_y = gps_map[0][1] ;

	//array for all thetas
	double thetas[len_of_points];

	//below loop will calculate theta for each point
    for(int i=0;i<len_of_points-1;i++){
        double y2minusy1 = gps_map[i+1][1]-gps_map[i][1];
        double x2minusx1 = gps_map[i+1][0]-gps_map[i][0];
        double slope = y2minusy1/x2minusx1;
        double theta = atan2(y2minusy1,x2minusx1);
        thetas[i] = theta;
    }
    thetas[len_of_points-1]=0;

//copy all to map_1 redundant loop but it will run only once so okay
    for(int i=0;i<len_of_points;i++){
        map_1[i][0] = gps_map[i][0];
        map_1[i][1] = gps_map[i][1];
        map_1[i][2] = thetas[i];
    }
}

//this function will fill the 3D array set_of_lines with pair of {{x1,y1},{x2,y2}}
void make_lines(){

    double start_point[2];
    double end_point[2];
    double line_points[2][2];
    for(int i=0;i<len_of_points;i++){

        double current_x = map_1[i][0];
        double current_y = map_1[i][1];
        double current_theta = map_1[i][2];

        for(int j=i;j<len_of_points;j++){
            //ignore till theta does not change (points on 1 line will have same theta value)
            if(current_theta!=map_1[j][2]){
                start_point[0] = current_x;
                start_point[1] = current_y;
                end_point[0] = map_1[j][0];
                end_point[1] = map_1[j][1];
                line_points[0][0] = start_point[0];
                line_points[0][1] = start_point[1];
                line_points[1][0] = end_point[0];
                line_points[1][1] = end_point[1];
                i=j;
                break;
            }
        }

    set_of_lines[line_counter][0][0] = line_points[0][0];
    set_of_lines[line_counter][0][1] = line_points[0][1];
    set_of_lines[line_counter][1][0] = line_points[1][0];
    set_of_lines[line_counter][1][1] = line_points[1][1];
    line_counter++;
    }

}

double slope_of_line(double x1,double y1,double x2,double y2){

    double slope;
    if(x1!=x2){
        slope = (y2-y1)/(x2-x1);
    }
	//if x1 == x2 than slope can be either 90 or -90
    else if(y1>y2){
        slope = -1.57; //--90
    }
    else{
        slope = 1.57; //90
    }
    return slope;
}

//ROS TO-DO This data should come from RTK GPS in real time
//TO-DO Replace this function as mentioned ^
//0 for x and 1 for y
double getPosition(int c,int ind){
    double ret_pos = 0.0;
    if(c==0){
        ret_pos = curr_poss_fake_array[ind][0];
    }
    else if(c==1){
        ret_pos = curr_poss_fake_array[ind][1];
    }
    return ret_pos;
}

//Calculate distance of a point from a line (perpendicular)
double distance_to_Line(double x, double y, double x1,double y1,double x2,double y2){
    double normalLength = hypot(x2 - x1, y2 - y1);
    double distance = (double)((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)) / normalLength;
    return distance;
}

//calculate CTE error - basically callig above function - redundant
double calculate_cte(double curr_x,double curr_y,double x1,double y1,double x2,double y2){
    double cte = distance_to_Line(curr_x,curr_y,x1,y1,x2,y2);
    return cte;
}

//calculate diff in angle of line the robot is following and yaw of vehicle - only subtraction
double calculate_angle_diff(double veh_angle,double curr_line_slope){
    return veh_angle - curr_line_slope;
}

//ROS TO-DO This data should come from WHEEL ENCODER in real time  OR need to carry the desired velocity although not sure if that is okay
//TO-DO Replace this function as mentioned ^
double get_speed(int wheel){
    if(wheel == 0){
        return global_vr;
    }
    return global_vl;
}

//ROS TO-DO This data should come from COMPASS in real time
//TO-DO Replace this function as mentioned ^
double get_angle(){
    return 0.0;
}

int do_path_set(){
    make_follow_path(); // this will add theta values to the set of lat and lon received from global planner
    make_lines(); // this will fill the 3d array set_of_lines with x,y coordinate pairs of the line SEGMENT.
    //cout<<"NUM OF POINTS "<<len_of_points<<'\n';
    //cout<<"LINE NUMER COUTNER "<<line_counter<<'\n';
}
//PID for postion
PID pid_p;
//PID for angle
PID pid_a;
void init_pid(){

	//to-do tuning
	double Kp = -0.2;
	double Ki = -0.004;
	double Kd = -3.0;
	pid_p.Init(Kp,Ki,Kd);
	pid_a.Init(Kp,Ki,Kd);
}

double* do_pid(double position_x,double position_y)
{
	//increasing output precision. No effect on PID
    //cout<<"Doing PID 1"<<"\n";
    std::cout << std::fixed;
    std::cout << std::setprecision(5);

	double max_wheel_vel = 15;
	double min_wheel_vel = 0.5;
	double wp = 1.0;
	double wv = 1.0;
    //cout<<"D1"<<"\n";


    static int i=0; //line number of current line - it will be retained and hence static

	double vehicle_pos[2];
    vehicle_pos[0] = position_x;
    vehicle_pos[1] = position_y;

    //Calculate slope of the current line - Line will change only one Point reaches the end point for previous line with some tolerance.
    double curr_line_slope = slope_of_line(set_of_lines[i][0][0],set_of_lines[i][0][1],set_of_lines[i][1][0],set_of_lines[i][1][1]);
    //cout<<"D2"<<"\n";
    //TO-DO Below See
    //REMOVE THIS COMMENT WHEN ANGLE FUNC IS GOOD double veh_angle = get_angle(); //to-do should be in radian // to-do currently this function is not giving correct data- must fetch from compass.
    //MAKE BELOW COMMENT WHEN ANGLE FUNC IS GOOD
    double veh_angle = curr_line_slope;
    //double veh_angle = get_angle();

    //linear velocity
    double vel_lin[2];
    vel_lin[0] = get_speed(0); // 0 is R // data should either some from wheel encoder or previous desired should be used. (not sure if that is okay)
    vel_lin[1] = get_speed(1); // 1 is L //data should either some from wheel encoder or previous desired should be used. (not sure if that is okay)
    //cout<<"D3"<<"\n";
    //dseired velocity as calculated ny PID - Distance
    double desired_vel_distpid[2];

    //dseired velocity as calculated ny PID - Distance
    double desired_vel_angpid[2];

    //Weitghted desired velocity
    double desired_vel_w[2];

    double cte = calculate_cte(vehicle_pos[0],vehicle_pos[1],set_of_lines[i][0][0],set_of_lines[i][0][1],set_of_lines[i][1][0],set_of_lines[i][1][1]);
    //difference in vehicle oreintaton and line slope
    double angle_diff = calculate_angle_diff(veh_angle,curr_line_slope);

    //PID results are stored here-
    double total_error_in_position;
    double total_error_in_angle;

    //calucate pid position
    pid_p.UpdateError(cte);
    total_error_in_position = pid_p.TotalError();

    //calculate pid angle
    pid_a.UpdateError(angle_diff);
    total_error_in_angle = pid_a.TotalError();
    //cout<<"D4"<<"\n";
    //change line when distance from end of line is small
    double dist_from_end_of_line = fabs(vehicle_pos[0]-set_of_lines[i][1][0])+fabs(vehicle_pos[1]-set_of_lines[i][1][1]);
    double min_dis_ep = 0.00001; // to-do tune the value
    if(dist_from_end_of_line<min_dis_ep){
        //cout<<"Line change"<<"\n";
        //init_pid();
        i++;
    }
    double total_error_pid = total_error_in_position + total_error_in_angle;
    //PID Implementation - Depending on the value of different PIDs - Change VR and VL

     if(total_error_pid>=0){
            desired_vel_w[0] = vel_lin[0]+wp*total_error_pid; //to-do Tuning
            desired_vel_w[1] = vel_lin[1]-wp*total_error_pid; //to-do Tuning
        }
    else if(total_error_pid<0){
            desired_vel_w[0] = vel_lin[0]+wp*total_error_pid; //to-do Tuning
            desired_vel_w[1] = vel_lin[1]-wp*total_error_pid; //to-do Tuning
        }

    //even if velocity of 1 wheel is exceeding, increment and decrement must be proportional.
    while(((fabs(desired_vel_w[0])<min_wheel_vel) || (fabs(desired_vel_w[1])<min_wheel_vel)) || ((fabs(desired_vel_w[0])>max_wheel_vel) || (fabs(desired_vel_w[1])>max_wheel_vel))){
        //cout<<"desired_vel_w[0] : "<<desired_vel_w[0]<<" and desired_vel_w[1] : "<<desired_vel_w[1]<<"\n";
        if((fabs(desired_vel_w[0])<min_wheel_vel) || (fabs(desired_vel_w[1])<min_wheel_vel)){
            //cout<<"Increasing"<<"\n";
            desired_vel_w[0]*=1.10;
            desired_vel_w[1]*=1.10;
        }
        else if((fabs(desired_vel_w[0])>max_wheel_vel) || (fabs(desired_vel_w[1])>max_wheel_vel)){
            //cout<<"Decreasing"<<"\n";
            desired_vel_w[0]*=0.90;
            desired_vel_w[1]*=0.90;
        }
        //sleep(1);
    }

    global_vr = desired_vel_w[0];
    global_vl = desired_vel_w[1];


    //TO-DO ROS PUBLISH DESIRED VELOCITY
    //cout<<"vel_lin "<<vel_lin[0] <<", "<<vel_lin[1]<<"\n";
    //cout<<"CTE "<<cte<<'\n';
    //cout<<"Current Point "<<vehicle_pos[0]<<", "<<vehicle_pos[1]<<'\n';
    //cout<<"Current Line slope "<<curr_line_slope<<'\n';
    //cout<<"Difference in Angle "<<angle_diff<<'\n';
    //cout<<"Curr Line Number "<<i<<" End pt "<<set_of_lines[i][0][0]<<", "<<set_of_lines[i][0][1]<<'\n';
    //cout<<"Curr Dist "<<" from end of line no. "<<i<<" = "<<dist_from_end_of_line<<'\n';
    //cout<<"Position PID  "<<total_error_in_position<<'\n';
    //cout<<"Angle PID  "<<total_error_in_angle<<'\n';
    //cout<<"Total PID  "<<total_error_pid<<'\n';
    //cout<<"Desired velocity  "<<desired_vel_w[0] <<", "<<desired_vel_w[1]<<"\n";

    static double ret_val_vel[2]; // array [ right vel , left vel ]
    ret_val_vel[0]=desired_vel_w[0];
    ret_val_vel[1]=desired_vel_w[1];
    return ret_val_vel;
}

int main(){

    double *des_vel;
    do_path_set();
    init_pid();
    for(int i=0;i<len_of_points;i++){
        //cout<<"Begin"<<"\n";
        double pos_x = getPosition(0,i); //Currently this position is not giving correct RTK GPS Point ROS -TO-DO
        double pos_y = getPosition(1,i); //Currently this position is not giving correct RTK GPS Point ROS -TO-DO
        //cout<<"Do PID"<<"\n";
        des_vel = do_pid(pos_x,pos_y);
        //TO-DO ROS PUBLISH DESIRED VELOCITY
        cout<<"VR "<<*(des_vel+0)<<", VL "<<*(des_vel+1)<<"\n";
        cout<<"\n\n";
    }
   return 0;
}
