#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <numeric>
#include "Eigen/Dense"
#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"
#include "parameter_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_arguments(int argc, char* argv[]) 
{
    string usage_instructions = "Usage instructions: ";
    usage_instructions += argv[0];
    usage_instructions += " <path_to_input_measurement_file> <output_file_name> <mode(either 0 or 1)>\n";
    usage_instructions += "Example: ./UnscentedKF input.txt output.txt 0";

    bool has_valid_args = false;

    // make sure the user has provided input and output files
    if (argc == 1)
        cerr << usage_instructions << endl;
    else if (argc == 2)
        cerr << "Please include an output file.\n" << usage_instructions << endl;
    else if (argc == 3)
        cerr << "Please include a mode (0:normal or 1:explore).\n" << usage_instructions << endl;
    else if (argc == 4)
        has_valid_args = true;
    else if (argc > 4)
        cerr << "Too many arguments.\n" << usage_instructions << endl;

    if (!has_valid_args)
        exit(EXIT_FAILURE);
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) 
{
    if (!in_file.is_open()) 
    {
        cerr << "Cannot open input file: " << in_name << endl;
        exit(EXIT_FAILURE);
    }

    if (!out_file.is_open()) 
    {
        cerr << "Cannot open output file: " << out_name << endl;
        exit(EXIT_FAILURE);
    }
}

void get_NIS(UKF ukf, vector<MeasurementPackage> measurement_pack_list, vector<double>& NIS_R, vector<double>& NIS_L)
{
    size_t number_of_measurements = measurement_pack_list.size();

    for (size_t k = 0; k < number_of_measurements; ++k) 
    {
        ukf.ProcessMeasurement(measurement_pack_list[k]);
        if (k > 2) NIS_R.push_back(ukf.NIS_radar_);
        NIS_L.push_back(ukf.NIS_laser_);
    }

}

void run_process(UKF ukf, vector<MeasurementPackage> measurement_pack_list, 
        vector<GroundTruthPackage> gt_pack_list, ostream& out_file, 
        vector<VectorXd>& estimations, vector<VectorXd>& ground_truth)
{
    size_t number_of_measurements = measurement_pack_list.size();

    for (size_t k = 0; k < number_of_measurements; ++k) 
    {
        // Call the UKF-based fusion
        ukf.ProcessMeasurement(measurement_pack_list[k]);

        // timestamp
        out_file << measurement_pack_list[k].timestamp_ << "\t"; // pos1 - est

        // output the state vector
        out_file << ukf.x_(0) << "\t"; // pos1 - est
        out_file << ukf.x_(1) << "\t"; // pos2 - est
        out_file << ukf.x_(2) << "\t"; // vel_abs -est
        out_file << ukf.x_(3) << "\t"; // yaw_angle -est
        out_file << ukf.x_(4) << "\t"; // yaw_rate -est

        // output lidar and radar specific data
        if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) 
        {
            // sensor type
            out_file << "lidar" << "\t";

            // NIS value
            out_file << ukf.NIS_laser_ << "\t";

            // output the lidar sensor measurement px and py
            out_file << measurement_pack_list[k].raw_measurements_(0) << "\t";
            out_file << measurement_pack_list[k].raw_measurements_(1) << "\t";

        } 
        else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) 
        {
            // sensor type
            out_file << "radar" << "\t";

            // NIS value
            out_file << ukf.NIS_radar_ << "\t";

            // output radar measurement in cartesian coordinates
            float ro = measurement_pack_list[k].raw_measurements_(0);
            float phi = measurement_pack_list[k].raw_measurements_(1);
            out_file << ro * cos(phi) << "\t"; // px measurement
            out_file << ro * sin(phi) << "\t"; // py measurement
        }

        // output the ground truth
        out_file << gt_pack_list[k].gt_values_(0) << "\t";
        out_file << gt_pack_list[k].gt_values_(1) << "\t";
        out_file << gt_pack_list[k].gt_values_(2) << "\t";
        out_file << gt_pack_list[k].gt_values_(3) << "\n";

        // convert ukf x vector to cartesian to compare to ground truth
        VectorXd ukf_x_cartesian_ = VectorXd(4);

        float x_estimate_ = ukf.x_(0);
        float y_estimate_ = ukf.x_(1);
        float vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
        float vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));
        
        ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;
        
        estimations.push_back(ukf_x_cartesian_);
        ground_truth.push_back(gt_pack_list[k].gt_values_);

    }
}

void print_column_names(ofstream& file_name)
{
    // column names for output file
    file_name << "time_stamp" << "\t";  
    file_name << "px_state" << "\t";
    file_name << "py_state" << "\t";
    file_name << "v_state" << "\t";
    file_name << "yaw_angle_state" << "\t";
    file_name << "yaw_rate_state" << "\t";
    file_name << "sensor_type" << "\t";
    file_name << "NIS" << "\t";  
    file_name << "px_measured" << "\t";
    file_name << "py_measured" << "\t";
    file_name << "px_ground_truth" << "\t";
    file_name << "py_ground_truth" << "\t";
    file_name << "vx_ground_truth" << "\t";
    file_name << "vy_ground_truth" << "\n";
}

int main(int argc, char* argv[]) 
{

    check_arguments(argc, argv);

    string in_file_name_ = argv[1];
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);

    string out_file_name_ = argv[2];
    ofstream out_file_(out_file_name_.c_str(), ofstream::out);

    int mode = atoi(argv[3]);
    enum RunMode
    {
        NORMAL,
        EXPLORE
    } run_mode;

    check_files(in_file_, in_file_name_, out_file_, out_file_name_);

    vector<MeasurementPackage> measurement_pack_list;
    vector<GroundTruthPackage> gt_pack_list;

    string line;

    while (getline(in_file_, line)) 
    {
        string sensor_type;
        MeasurementPackage meas_package;
        GroundTruthPackage gt_package;
        istringstream iss(line);
        long long timestamp;

        // reads first element from the current line
        iss >> sensor_type;

        if (sensor_type.compare("L") == 0) 
        {
            // laser measurement

            // read measurements at this timestamp
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        } 
        else if (sensor_type.compare("R") == 0) 
        {
            // radar measurement

            // read measurements at this timestamp
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float phi;
            float ro_dot;
            iss >> ro;
            iss >> phi;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro, phi, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }

        // read ground truth data to compare later
        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
        gt_package.gt_values_ = VectorXd(4);
        gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
        gt_pack_list.push_back(gt_package);
    }

    // used to compute the RMSE later
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    size_t number_of_measurements = measurement_pack_list.size();

    // print the header in the output file
    print_column_names(out_file_);

    vector<double> NIS_R;
    vector<double> NIS_L;

    // take action based on user input
    switch (mode)
    {
        case RunMode::NORMAL:       // run UKF based on fixed parameters
            {
                ParameterPackage best_param;
                best_param.STD_A = 0.8;
                best_param.STD_YAWDD = 0.4;
                best_param.MULT_P1 = 0.2;
                best_param.MULT_P2 = 0.4;

                UKF ukf(best_param);
                run_process(ukf, measurement_pack_list, gt_pack_list, out_file_, estimations, ground_truth);
            }
            break;
        case RunMode::EXPLORE:      // run UKF based on parameter combinations
            {
                vector<ParameterPackage> param_list;
                string p_file_name_ = "param.in";
                ifstream p_file_(p_file_name_.c_str(), ifstream::in);

                if (!p_file_.is_open()) 
                {
                    cerr << "Cannot open input file: " << p_file_name_ << endl;
                    cerr << "Create a file named param.in" << endl;
                    cerr << "Each line should have a combination of the following parameter values:" << endl;
                    cerr << "STD_A STD_YAWDD MULT_P1 MULT_P2" << endl;
                    cerr << "See ParameterPackage for definitions" <<endl;
                    exit(EXIT_FAILURE);
                }

                int n_param = 0;
                while (getline(p_file_, line)) 
                {
                    ParameterPackage par_package;
                    istringstream iss1(line);
                    double std_a;
                    double std_yawdd;
                    double mult_p1;
                    double mult_p2;

                    iss1 >> std_a;
                    iss1 >> std_yawdd;
                    iss1 >> mult_p1;
                    iss1 >> mult_p2;

                    par_package.STD_A = std_a;
                    par_package.STD_YAWDD = std_yawdd;
                    par_package.MULT_P1 = mult_p1;
                    par_package.MULT_P2 = mult_p2;
                    param_list.push_back(par_package);
                    n_param++;
                }
                std::cout << "number of parameter combinations = " << n_param << std::endl;

                int ncross_r, ncross_l;
                int ncross_sum = 10000;
                int chosen_index = 0;

                for (int i=0; i<n_param; i++)
                {
                    // Create a UKF instance
                    UKF ukf(param_list[i]);

                    NIS_R = {};
                    NIS_L = {};
                    estimations = {};
                    ground_truth = {};
                    ncross_r = 0;
                    ncross_l = 0;

                    get_NIS(ukf, measurement_pack_list, NIS_R, NIS_L);

                    double sum_r = std::accumulate(std::begin(NIS_R), std::end(NIS_R), 0.0);
                    double m_r =  sum_r / NIS_R.size();

                    double sum_l = std::accumulate(std::begin(NIS_L), std::end(NIS_L), 0.0);
                    double m_l =  sum_l / NIS_L.size();

                    double accum_r = 0.0;
                    double accum_l = 0.0;
                    std::for_each (std::begin(NIS_R), std::end(NIS_R), [&](const double d) 
                    {
                        accum_r += (d - m_r) * (d - m_r);
                        if (d > 7.815) ncross_r++;
                    });
                    std::for_each (std::begin(NIS_L), std::end(NIS_L), [&](const double d) 
                    {
                        accum_l += (d - m_l) * (d - m_l);
                        if (d > 5.991) ncross_l++;
                    });

                    double stdev_r = sqrt(accum_r / (NIS_R.size()-1));
                    double stdev_l = sqrt(accum_l / (NIS_L.size()-1));

                    std::cout << i << "\t" << ukf.std_a_ << "\t" << ukf.std_yawdd_ << "\t" << param_list[i].MULT_P1 << "\t" << param_list[i].MULT_P2 << "\t";
                    std::cout << m_r << "\t" << m_l << "\t";
                    std::cout << stdev_r << "\t" << stdev_l << "\t";
                    std::cout << *std::max_element(NIS_R.begin(), NIS_R.end()) << "\t";
                    std::cout << *std::max_element(NIS_L.begin(), NIS_L.end()) << "\t";
                    std::cout << ncross_r << "\t" << ncross_l << std::endl;
                    if (ncross_sum > ncross_r + ncross_l)
                    {
                        ncross_sum = ncross_r + ncross_l;
                        chosen_index = i;
                    }
                            
                }
                // run the filter with the best parameter set
                std::cout << "best values are \n";
                std::cout << "std_a = " << "\t" << param_list[chosen_index].STD_A << std::endl;
                std::cout << "std_yawdd = " << "\t" << param_list[chosen_index].STD_YAWDD << std::endl;
                std::cout << "mult_p1 = " << "\t" << param_list[chosen_index].MULT_P1 << std::endl;
                std::cout << "mult_p2 = " << "\t" << param_list[chosen_index].MULT_P2 << std::endl;
                UKF ukf(param_list[chosen_index]);
                run_process(ukf, measurement_pack_list, gt_pack_list, out_file_, estimations, ground_truth);
            }
            break;
        default:
            std::cout << "Wrong mode, select either 0 or 1!!!" << std::endl;
            return 0;
    }

    // compute the accuracy (RMSE)
    Tools tools;
    cout << "RMSE" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;

    // close files
    if (out_file_.is_open()) 
        out_file_.close();

    if (in_file_.is_open())
        in_file_.close();

    cout << "Done!" << endl;
    return 0;
}
