#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include <typeinfo>
#include <vector>

//#define PRINT_POINT_VECTORS
//#define PRINT_AXIS_VECTORS
//#define PRINT_PROJECTION_POINTS
//#define PRINT_BOUNDARIES

using namespace std;

const char SAPARATOR[2] = " ";
const float MINIMUM_DISTANCE_THRESHOLD = 0.001f;

static Eigen::Vector2f GetProjectionPointOnAxis(
    Eigen::Vector2f in_point,
    Eigen::Vector2f axis_vector)
{
    float numerator = in_point.dot(axis_vector);
    float denominator = axis_vector.dot(axis_vector);
    return (numerator/denominator) * axis_vector;
}

static float GetDistance(
    Eigen::Vector2f axis_point,
    Eigen::Vector2f boundary_point)
{
    Eigen::Vector2f distance_vector =
        axis_point - boundary_point;
    return sqrtf(pow(distance_vector(0), 2) + pow(distance_vector(1), 2));
}
int main(){

    // read raw data
    fstream file;
    char buffer[20];
    vector<Eigen::Vector2f> point_vectors;

    file.open("../data/data_1.txt", ios::in);
    if(!file)
        cout << "fail to open the file" << endl;
    else
    {
        while (!file.eof())
        {
            Eigen::Vector2f point_vector;
            file.getline(buffer,sizeof(buffer));
            if(buffer != '\0')
            {
                char *token;
                token = strtok(buffer, SAPARATOR);
                if(token != NULL)
                {
                    point_vector(0) = atof(token);
                    token = strtok(NULL, SAPARATOR);
                    point_vector(1) = atof(token);
                    point_vectors.push_back(point_vector);
                }
            }
        }
        file.close();
    }

    #ifdef PRINT_POINT_VECTORS
    for(unsigned int i = 0; i < point_vectors.size(); i++)
        cout << point_vectors[i](0) << ' ' << point_vectors[i](1) << endl;
    #endif

    // pose estimation function
    cout << "pose estimation starts" << endl;

    vector<float> total_loss;
    for (unsigned int theda = 0; theda < 90; theda ++)
    {
        Eigen::Rotation2Df rotation(theda * 3.14159 / 180);
        Eigen::Vector2f x_axis_vector;
        Eigen::Vector2f y_axis_vector;
        x_axis_vector << 1.0f, 0.0f;
        y_axis_vector << 0.0f, 1.0f;

        Eigen::Vector2f axis_1_vector =
            rotation.toRotationMatrix() * x_axis_vector;
        Eigen::Vector2f axis_2_vector =
            rotation.toRotationMatrix() * y_axis_vector;

        #ifdef PRINT_AXIS_VECTORS
            cout << axis_1_vector << endl;
            cout << axis_2_vector << endl;
        #endif

        vector<Eigen::Vector2f> point_vectors_on_axis_1;
        vector<Eigen::Vector2f> point_vectors_on_axis_2;

        for (unsigned int i = 0; i < point_vectors.size(); i++)
        {
            point_vectors_on_axis_1.push_back(
                GetProjectionPointOnAxis(point_vectors[i], axis_1_vector));
            point_vectors_on_axis_2.push_back(
                GetProjectionPointOnAxis(point_vectors[i], axis_2_vector));
        }

        #ifdef PRINT_PROJECTION_POINTS
        if(theda == 45)
        {
            for(unsigned int i = 0; i < point_vectors_on_axis_1.size(); i++)
            {
                //cout << point_vectors_on_axis_1[i] << endl;
                cout << point_vectors_on_axis_2[i] << endl;
            }
        }
        #endif

        Eigen::Vector2f max_point_axis_1;
        Eigen::Vector2f min_point_axis_1;
        Eigen::Vector2f max_point_axis_2;
        Eigen::Vector2f min_point_axis_2;
        float current_horizontal_min = 500;
        float current_horizontal_max = -500;
        float current_vertical_min = 500;
        float current_vertical_max = -500;

        for(unsigned int i = 0; i < point_vectors_on_axis_1.size(); i++)
        {
            if(point_vectors_on_axis_1[i](0) > current_horizontal_max)
            {
                current_horizontal_max = point_vectors_on_axis_1[i](0);
                max_point_axis_1 = point_vectors_on_axis_1[i];
            }
            if(point_vectors_on_axis_1[i](0) < current_horizontal_min)
            {
                current_horizontal_min = point_vectors_on_axis_1[i](0);
                min_point_axis_1 = point_vectors_on_axis_1[i];
            }
        }

        for(unsigned int i = 0; i < point_vectors_on_axis_2.size(); i++)
        {
            if(point_vectors_on_axis_2[i](1) > current_vertical_max)
            {
                current_vertical_max = point_vectors_on_axis_2[i](1);
                max_point_axis_2 = point_vectors_on_axis_2[i];
            }
            if(point_vectors_on_axis_2[i](1) < current_vertical_min)
            {
                current_vertical_min = point_vectors_on_axis_2[i](1);
                min_point_axis_2 = point_vectors_on_axis_2[i];
            }
        }

        #ifdef PRINT_BOUNDARIES
        if(theda == 45)
        {
            cout << max_point_axis_1 << endl;
            cout << min_point_axis_1 << endl;
            cout << max_point_axis_2 << endl;
            cout << min_point_axis_2 << endl;
        }
        #endif

        float current_loss = 0;
        for(unsigned int i = 0; i < point_vectors.size(); i++)
        {
            float minimum_distance = 500;
            if(GetDistance(point_vectors_on_axis_1[i], max_point_axis_1) < minimum_distance)
                minimum_distance = GetDistance(point_vectors_on_axis_1[i], max_point_axis_1);
            if(GetDistance(point_vectors_on_axis_1[i], min_point_axis_1) < minimum_distance)
                minimum_distance = GetDistance(point_vectors_on_axis_1[i], min_point_axis_1);
            if(GetDistance(point_vectors_on_axis_2[i], max_point_axis_2) < minimum_distance)
                minimum_distance = GetDistance(point_vectors_on_axis_2[i], max_point_axis_2);
            if(GetDistance(point_vectors_on_axis_2[i], min_point_axis_2) < minimum_distance)
                minimum_distance = GetDistance(point_vectors_on_axis_2[i], min_point_axis_2);
                current_loss += minimum_distance;
        }
        total_loss.push_back(current_loss);
    }

    float minimum_loss = total_loss[0];
    unsigned int output_angle = 0;
    for(unsigned int theda = 1; theda < 90; theda++)
    {
        if(total_loss[theda] < minimum_loss)
        {
            minimum_loss = total_loss[theda];
            output_angle = theda;
        }
    }
    cout << output_angle << endl;
    return 0;
}
