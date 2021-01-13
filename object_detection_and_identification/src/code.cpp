#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
using namespace std;

//global variable setup
float matrix[6][6] = {0.0};
float original_x;
float original_y;
float original_phi;
int ideal_case[] = {0,0,0,0,0,0,0};
float min_distance = 1000.0;
int counter = 1;
float boxlocation_x;
float boxlocation_y;
float boxlocation_phi;
float target_x;
float target_y;
float target_phi;
bool done = false;
float pi = 3.141592;
bool start = true;
int template_id;


//function calculating the distance between two points
float distance_calc(float x1, float x2, float y1, float y2){
    return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
 }

//finding the ideal sequence using brute-force algorithm
void idealcase_finder(float matrix[6][6]){
    int cases[] = {1, 2, 3, 4, 5};
    sort(cases, cases+5);
    do{
        float total_distance = 0;
        int temp_case[] = {0, cases[0], cases[1], cases[2], cases[3], cases[4], 0};
        cout << "Calculating Distance for:  " << temp_case[0] << temp_case[1] << temp_case[2] << temp_case[3] << temp_case[4] << temp_case[5] << temp_case[6] << endl;
        for (int i=0; i<6; i++){
            int first = temp_case[i];
            int second = temp_case[i+1];
            total_distance = total_distance + matrix[first][second];
        }
        cout << "Distance for this path is:  " << total_distance << endl;
        if (total_distance < min_distance){
            min_distance = total_distance;
            for (int i=0; i<7; i++){
                ideal_case[i] = temp_case[i];
            }
        }
    } while(next_permutation(cases,cases+5));
}

//function finding target location and target angle
void target_finder (float x_location, float y_location, float angle){
    if (angle >= 0){
        target_x = x_location + 0.75*cos(angle);
        target_y = y_location + 0.75*sin(angle);
        target_phi = angle - pi;
    }
    else{
        target_x = x_location + 0.75*cos(angle);
        target_y = y_location + 0.75*sin(angle);
        target_phi = angle + pi;
    }
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);

    //Giving the robot some delay for establishing subscriber
    ros::Duration(4.00).sleep();

    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    
    //Create a text file with box coordinate and template ID
    ofstream finaloutput;
    finaloutput.open("finaloutput.txt");

    // Execute strategy.
    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        //to be executed at the beginning
        if (start){
            //storing initial robot position
            original_x = robotPose.x;
            original_y = robotPose.y;
            original_phi = robotPose.phi;

            cout << "Original X:  " << original_x << endl;
            cout << "Original Y:  " << original_y << endl;
            cout << "Original Phi:  " << original_phi << endl << endl;

            //creating a Hamilton Circuit
            for (int i=1; i<6; i++){
                float box_x = boxes.coords[i-1][0];
                float box_y = boxes.coords[i-1][1];
                matrix[0][i] = distance_calc(original_x, box_x, original_y, box_y);
                matrix[i][0] = matrix[0][i];
            }
            for (int i=1; i<6; i++){
                float box_x1 = boxes.coords[i-1][0];
                float box_y1 = boxes.coords[i-1][1];
                for (int j=i+1; j<6; j++){
                    float box_x2 = boxes.coords[j-1][0];
                    float box_y2 = boxes.coords[j-1][1];
                    if (i != j){
                        matrix[i][j] = distance_calc(box_x1, box_x2, box_y1, box_y2);
                        matrix[j][i] = matrix[i][j];
                    }
                }
            }

            //printing out the matrix for testing purpose
            cout << "<Hamilton Circuit>" << endl;
            for (int x=0; x<6; x++){
                for (int y=0; y<6; y++){
                    cout << matrix[x][y] << "    ";
                }
                cout << endl;
            }
            
            //run idealcase_finder to find the optimal path
            idealcase_finder(matrix);
            
            //printing minimum distance and the ideal case for testing purpose
            cout << endl << "Minimal distance for travel is:  " << min_distance <<endl;
            cout << "Ideal Path:  ";
            for (int i=0; i<7; i++){
                cout << ideal_case[i];
            }
            cout << endl << endl;
            start = false;
        }


        if (counter == 6){
            target_x = original_x;
            target_y = original_y;
            target_phi = original_phi;
            done = true;
            cout << "Return to the Original Position!" << endl;
        }
        else{
            int boxnumber = ideal_case[counter];
            cout << "move to box number:  " << boxnumber << endl;
            boxlocation_x = boxes.coords[boxnumber-1][0];
            boxlocation_y = boxes.coords[boxnumber-1][1];
            boxlocation_phi = boxes.coords[boxnumber-1][2];
            //finding target coordinate and target angle using target_finder
            target_finder(boxlocation_x, boxlocation_y, boxlocation_phi);
        }

        //printing target location and angle for testing purpose
        cout << "target x:  " << target_x << endl;
        cout << "target y:  " << target_y << endl;
        cout << "target phi:  " << target_phi << endl;

        //move to target
        Navigation::moveToGoal(target_x, target_y, target_phi);

        ros::spinOnce();

        //print current location after navigation
        cout << "we are at x:  " << robotPose.x << "  y:  " << robotPose.y << "  phi:  " << robotPose.phi << endl << endl;

        if (done){
            cout << "Arrived at the Original Position!" << endl;
            break;
        }

        ros::spinOnce();
        
        //Here is where the image processing should be
        //Template 1: Raisin Bran, Template 2: Cinnamon Toast Crunch, Template 3: Rice Kispies
        template_id = imagePipeline.getTemplateID(boxes);
        cout << "template id for box " << ideal_case[counter] << " is: " <<  template_id << endl << endl;
        
        //writing box coordinate and tag information into text file
        if (finaloutput.is_open()){
            finaloutput << "For box number: ";
            finaloutput << ideal_case[counter];
            finaloutput << "\n";
            finaloutput << "<Location>\n";
            finaloutput << "X: ";
            finaloutput << boxlocation_x;
            finaloutput << "\n";
            finaloutput << "Y: ";
            finaloutput << boxlocation_y;
            finaloutput << "\n";
            finaloutput << "Phi: ";
            finaloutput << boxlocation_phi;
            finaloutput << "\n";
            finaloutput << "<Tag ID>\n";
            if (template_id == 1){
                finaloutput << "Raisin Bran!!\n\n";
            }
            else if (template_id == 2){
                finaloutput << "Cinnamon Toast Crunch!!\n\n";
            }
            else if (template_id == 3){
                finaloutput << "Rice Krispies!!\n\n";
            }
            else if (template_id == 0){
                finaloutput << "Blank!!\n\n";
            }
            else if (template_id == -1){
                finaloutput << "Something is Wrong!!\n\n";
            }
        }
        else cout << "Unable to Open File!";

        counter = counter + 1;
        ros::Duration(0.01).sleep();
    }
    //closing the file
    finaloutput.close();
    return 0;
}