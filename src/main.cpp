#include <iostream>
#include <stdlib.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/common/common_headers.h>
#include <pcl/common/angles.h> // deg2rad, rad2deg, normAngle (normalize angle to (-pi, pi))

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <boost/filesystem.hpp> // get file extension from input arg etc.
#include <boost/program_options.hpp>
#include <boost/any.hpp>


typedef pcl::PointXYZRGB PointT;
namespace po = boost::program_options;


// struct for axis inputs, throw error if input not allowed
struct  po_sac_axis {
    po_sac_axis(std::string const& val):
        value(val)
    { }
    std::string value;
};

void validate(boost::any& vec, std::vector<std::string> const& values, po_sac_axis* /* target_type */, int) {

    // make sure no previous assignment to 'vec' was made
    po::validators::check_first_occurrence(vec);

    // extract the first string from 'values'. If there is more than one string, it's an error, and an exception will be thrown
    std::string const& str = po::validators::get_single_string(values);

    if (       str == "x" || str == "y" || str == "z"
               || str == "xy" || str == "yx"
               || str == "xz" || str == "zx"
               || str == "yz" || str == "zy"
               || str == "xyz" || str == "xzy" || str == "yxz" || str == "yzx" || str == "zxy" || str == "zyx"){
        vec = boost::any(po_sac_axis(str));
    }
    else {
        std::cerr << "\nError: Unexpected axis setting for SAC perpendicular plane model. Allowed inputs are \"z\", \"y\" \"x\", combinations like \"yx\" or all \"zyx\".\n \n";
        throw po::validation_error(po::validation_error::invalid_option_value);
    }
}


int main (int argc, char** argv) {

    //----- boost program options: parse command line
    //----- boost filesystem: get file path, file name, file extension

    // variables that will store parsed command line values
    std::string input_file;
    std::string pp_axis; // axis/axes to which perpendicular plane model/s is/are estimated
    std::string output_format;
    std::string one_all;
    int max_iterations;

    float eps_angle_xy; // max. angle for sac plane models
    float eps_angle_xz;
    float eps_angle_yz;

    float d_thresh_xy; // distance threshold for inliers in sac plane model
    float d_thresh_xz;
    float d_thresh_yz;

    float multiplier_xy; // if (current inliers < previous inliers*multiplier) -> SAC stops sampling
    float multiplier_xz;
    float multiplier_yz;

    // setup options
    po::options_description desc("Inputs and options");
    desc.add_options()
            ("help", "List of input arguments. Default values will be used if there is no input for an argument.")
            ("input", po::value<std::string>(&input_file), "Set the path to the input file (.ply or .pcd). Use quotation marks (--input \"/path to the file\") if there are spaces in the file path.")
            ("axis", po::value<po_sac_axis>(), "Set axis for SAC perpendicular plane model: xy-plane (--axis z), xz-plane (--axis y), yz-plane (--axis x), a combination (--axis yx) or all (--axis zyx) [default: zyx].")
            //("axis", po::value<std::string>(&pp_axis)->default_value("zyx"), "Set axis for SAC perpendicular plane model: xy-plane (--axis z), xz-plane (--axis y), yz-plane (--axis x), a combination (--axis yx) or all (--axis zyx) [default: zyx].")
            ("output", po::value<std::string>(&output_format)->default_value("ply"), "Set the output file format (--output ply or --output pcd) for the filtered cloud [default: ply].")
            ("nplanes", po::value<std::string>(&one_all)->default_value("all"), "Try to find all planes (all) or only one plane (one) [default: all].")
            ("max-i", po::value<int>(&max_iterations), "Set the max. iterations per axis as integer [default: not set].")
            ("angle-xy", po::value<float>(&eps_angle_xy)->default_value(2.0), "Set the EpsAngle for xy-plane [default: 2.0°].")
            ("angle-xz", po::value<float>(&eps_angle_xz)->default_value(10.0), "Set the EpsAngle for xz-plane [default: 10.0°].")
            ("angle-yz", po::value<float>(&eps_angle_yz)->default_value(10.0), "Set the EpsAngle for yz-plane [default: 10.0°].")
            ("distance-xy", po::value<float>(&d_thresh_xy)->default_value(0.03), "Set the distance threshold value for xy-plane [default: 0.03].")
            ("distance-xz", po::value<float>(&d_thresh_xz)->default_value(0.04), "Set the distance threshold value for xz-plane [default: 0.04].")
            ("distance-yz", po::value<float>(&d_thresh_yz)->default_value(0.04), "Set the distance threshold value for yz-plane [default: 0.04].")
            ("multi-xy", po::value<float>(&multiplier_xy)->default_value(0.25), "Set multiplier value for SAC loop sample termination condition. New plane model must contain at least (multiplier)*(points of previous plane model) [default: 0.25].")
            ("multi-xz", po::value<float>(&multiplier_xz)->default_value(0.25), "Set multiplier value for SAC loop sample termination condition. New plane model must contain at least (multiplier)*(points of previous plane model) [default: 0.25].")
            ("multi-yz", po::value<float>(&multiplier_yz)->default_value(0.25), "Set multiplier value for SAC loop sample termination condition. New plane model must contain at least (multiplier)*(points of previous plane model) [default: 0.25].")
            ;

    // parse input
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    else if (vm.count("input")==0) {
        std::cout << "\nError: No input file was set. Set an input file.\n";
        return 1;
    }

    else if (output_format != "ply" && output_format != "pcd"){
        std::cout << "\nError: Unexpected output file format. Allowed inputs are \"ply\" and \"pcd\".\n";
        return 1;
    }

    else if (vm.count("axis")== 0) {
        pp_axis="zyx"; // [default: zyx]
    }

    else if (vm.count("axis")!= 0) {
        pp_axis=vm["axis"].as<po_sac_axis>().value;
    }

    boost::filesystem::path file_path (input_file);
    std::string file_name_no_ext = file_path.stem().string();
    std::string file_extension = file_path.extension().string();
    std::string file_name_with_ext = file_path.filename().string();

    //-----

    // cloud objects for input/output clouds
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());

    pcl::PointCloud<PointT>::Ptr cloud_inliers_sac_xy (new pcl::PointCloud<PointT> ());
    pcl::PointCloud<PointT>::Ptr cloud_outliers_sac_xy (new pcl::PointCloud<PointT> ());
    pcl::PointCloud<PointT>::Ptr cloud_inliers_sac_xy_out (new pcl::PointCloud<PointT> ());

    pcl::PointCloud<PointT>::Ptr cloud_inliers_sac_xz (new pcl::PointCloud<PointT> ());
    pcl::PointCloud<PointT>::Ptr cloud_outliers_sac_xz (new pcl::PointCloud<PointT> ());
    pcl::PointCloud<PointT>::Ptr cloud_inliers_sac_xz_out (new pcl::PointCloud<PointT> ());

    pcl::PointCloud<PointT>::Ptr cloud_inliers_sac_yz (new pcl::PointCloud<PointT> ());
    pcl::PointCloud<PointT>::Ptr cloud_outliers_sac_yz (new pcl::PointCloud<PointT> ());
    pcl::PointCloud<PointT>::Ptr cloud_inliers_sac_yz_out (new pcl::PointCloud<PointT> ());

    std::cerr << "Axis setting for SAC plane models: " << pp_axis << "\n";

    // load input cloud
    if (file_extension == ".ply") {
        std::cerr << "\nLoading point cloud...\n";
        pcl::io::loadPLYFile(input_file, *cloud);
    }

    else if (file_extension == ".pcd") {
        std::cerr << "\nLoading point cloud...\n";
        pcl::io::loadPCDFile (input_file, *cloud);
    }

    else if (file_extension != ".ply" && file_extension != ".pcd") {
        std::cout << "\nError: Input is no point cloud or has unexpected file format. Allowed input formats are \"ply\" and \"pcd\".\n";
        return 1;
    }

    std::cerr << "\nPoint cloud before filtering: " << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList (*cloud) << ").\n";

    int cloud_unfiltered_size = cloud->width * cloud->height;
    std::string cloud_unfiltered_data_points = pcl::getFieldsList (*cloud);


    // SACMODEL_PERPENDICULAR_PLANE Z
    //-------------------------------

    int counter_while_sac_xy = 0;
    if (pp_axis.find("z") != std::string::npos) {
        std::cerr <<"\nFiltering planes perpendicular to z-axis...\n";

        // create the filtering object + segmentation parameters
        pcl::ModelCoefficients::Ptr coefficients_sac_xy (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_sac_xy (new pcl::PointIndices);

        pcl::SACSegmentation<PointT> sac_xy;
        sac_xy.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
        sac_xy.setMethodType (pcl::SAC_RANSAC);
        sac_xy.setOptimizeCoefficients (true);
        sac_xy.setDistanceThreshold (d_thresh_xy); // float distance_threshold for plane model estimation, walls (xz, yz planes) need higher threshold (typically)
        sac_xy.setAxis(Eigen::Vector3f(0,0,1)); // planes perpendicular to axis: (0,0,1) -> xy, (1,0,0) -> yz, (0,1,0) -> xz
        sac_xy.setEpsAngle(pcl::deg2rad(eps_angle_xy)); // rad*180/pi = deg
        sac_xy.setInputCloud (cloud);

        //int counter_while_sac_xy = 0;
        int max_iterations_sac_xy = 1;
        int pre_inliers_sac_xy = 0; //previous model inliers, loop iteration i-1
        int iter_start_xy =100;
        bool true_sac_xy = true;

        while (true_sac_xy) {

            sac_xy.setMaxIterations(iter_start_xy*max_iterations_sac_xy);
            sac_xy.segment (*inliers_sac_xy, *coefficients_sac_xy); //(*PointIndices, *ModelCoefficients)

            if (inliers_sac_xy->indices.size () == 0) {
                PCL_ERROR ("Could not estimate a planar model on xy-plane for the input cloud.\n");
                break;
                //return (-1);
            }

            if ( counter_while_sac_xy > 0 && inliers_sac_xy->indices.size () < pre_inliers_sac_xy*multiplier_xy) { // stops if "gradient" too small
                std::cout <<"\nCould not estimate a planar model on xy-plane for remaining points in cloud.\n";
                true_sac_xy=false;
                break;
            }

            // extract inliers
            pcl::ExtractIndices<PointT> extract_sac_xy;
            extract_sac_xy.setInputCloud (cloud);
            extract_sac_xy.setIndices (inliers_sac_xy);
            extract_sac_xy.setNegative (false); // extract the inliers in consensus model (the part to be removed from point cloud)
            extract_sac_xy.filter (*cloud_inliers_sac_xy); // cloud_inliers contains the found plane

            // extract outliers
            extract_sac_xy.setNegative (true);	// extract the outliers (interesting part of point cloud)
            extract_sac_xy.filter (*cloud_outliers_sac_xy); // cloud_outliers contains everything but the plane(s)

            *cloud=*cloud_outliers_sac_xy;
            *cloud_inliers_sac_xy_out+=*cloud_inliers_sac_xy;

            std::cout <<"\nCounter iteration xy-plane: " << counter_while_sac_xy << "\n";
            std::cout <<"SAC max. iterations xy-plane: " << iter_start_xy*(max_iterations_sac_xy) << "\n";
            std::cout <<"SAC inliers in iteration xy-plane: " << inliers_sac_xy->indices.size () << "\n";
            std::cout <<"SAC inliers in previous iteration: " << pre_inliers_sac_xy << "\n";

            counter_while_sac_xy++;
            max_iterations_sac_xy=counter_while_sac_xy*100;
            pre_inliers_sac_xy = inliers_sac_xy->indices.size ();

            if (one_all =="one"){
                true_sac_xy=false;
            }

            if (vm.count("max-i")!= 0 && counter_while_sac_xy > (max_iterations-1)) {
                true_sac_xy=false;
            }
        }

        std::cerr << "PointCloud after filtering (inlieres perpendicular to z): " << cloud_inliers_sac_xy_out->width * cloud_inliers_sac_xy_out->height
                  << " data points (" << pcl::getFieldsList (*cloud_inliers_sac_xy_out) << ").\n";

        std::cerr << "PointCloud after filtering (outliers perpendicular to z): " << cloud->width * cloud->height
                  << " data points (" << pcl::getFieldsList (*cloud) << ").\n";
    }
    //-------------------------


    // SACMODEL_PERPENDICULAR_PLANE Y
    //-------------------------------

    int counter_while_sac_xz = 0;
    if (pp_axis.find("y") != std::string::npos) {
        std::cerr <<"\nFiltering planes perpendicular to y-axis...\n";

        // create the filtering object + segmentation parameters
        pcl::ModelCoefficients::Ptr coefficients_sac_xz (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_sac_xz (new pcl::PointIndices);

        pcl::SACSegmentation<PointT> sac_xz;
        sac_xz.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
        sac_xz.setMethodType (pcl::SAC_RANSAC);
        sac_xz.setOptimizeCoefficients (true);
        sac_xz.setDistanceThreshold (d_thresh_xz); // float distance_threshold for plane model estimation, walls (xz, yz planes) need higher threshold (typically)
        sac_xz.setAxis(Eigen::Vector3f(0,1,0)); // planes perpendicular to axis: (0,0,1) -> xy, (1,0,0) -> yz, (0,1,0) -> xz
        sac_xz.setEpsAngle(pcl::deg2rad(eps_angle_xz)); // rad*180/pi = deg
        sac_xz.setInputCloud (cloud);

        //int counter_while_sac_xz = 0;
        int max_iterations_sac_xz = 1;
        int pre_inliers_sac_xz = 0; // previous model inliers, loop iteration i-1
        int iter_start_xz =500;
        bool true_sac_xz = true;

        while (true_sac_xz) {

            sac_xz.setMaxIterations(iter_start_xz*max_iterations_sac_xz);
            sac_xz.segment (*inliers_sac_xz, *coefficients_sac_xz); //(*PointIndices, *ModelCoefficients)

            if (inliers_sac_xz->indices.size () == 0) {
                PCL_ERROR ("Could not estimate a planar model on xz-plane for the input cloud.\n");
                break;
                //return (-1);
            }

            if ( counter_while_sac_xz > 0 && inliers_sac_xz->indices.size() < pre_inliers_sac_xz*multiplier_xz) { // stops if "gradient" too small
                std::cout <<"\nCould not estimate a planar model on xz-plane for remaining points in cloud.\n";
                true_sac_xz=false;
                break;
            }

            // extract inliers
            pcl::ExtractIndices<PointT> extract_sac_xz;
            extract_sac_xz.setInputCloud (cloud);
            extract_sac_xz.setIndices (inliers_sac_xz);
            extract_sac_xz.setNegative (false); // extract the inliers in consensus model (the part to be removed from point cloud)
            extract_sac_xz.filter (*cloud_inliers_sac_xz); // cloud_inliers contains the found plane

            // extract outliers
            extract_sac_xz.setNegative (true);	// extract the outliers (interesting part of point cloud)
            extract_sac_xz.filter (*cloud_outliers_sac_xz); // cloud_outliers contains everything but the plane(s)

            *cloud=*cloud_outliers_sac_xz;
            *cloud_inliers_sac_xz_out+=*cloud_inliers_sac_xz;

            std::cout <<"\nCounter iteration xz-plane: " << counter_while_sac_xz << "\n";
            std::cout <<"SAC max. iterations xz-plane: " << iter_start_xz*(max_iterations_sac_xz) << "\n";
            std::cout <<"SAC inliers in iteration xz-plane: " << inliers_sac_xz->indices.size () << "\n";
            std::cout <<"SAC inliers in previous iteration: " << pre_inliers_sac_xz << "\n";

            counter_while_sac_xz++;
            max_iterations_sac_xz=counter_while_sac_xz*20;
            pre_inliers_sac_xz = inliers_sac_xz->indices.size ();

            if (one_all =="one"){
                true_sac_xz=false;
            }

            if (vm.count("max-i")!= 0 && counter_while_sac_xz > (max_iterations-1)) {
                true_sac_xz=false;
            }
        }

        std::cerr << "PointCloud after filtering (inlieres perpendicular to y): " << cloud_inliers_sac_xz_out->width * cloud_inliers_sac_xz_out->height
                  << " data points (" << pcl::getFieldsList (*cloud_inliers_sac_xz_out) << ").\n";

        std::cerr << "PointCloud after filtering (outliers perpendicular to y): " << cloud->width * cloud->height
                  << " data points (" << pcl::getFieldsList (*cloud) << ").\n";
    }
    //-------------------------


    // SACMODEL_PERPENDICULAR_PLANE X
    //-------------------------------

    int counter_while_sac_yz = 0;
    if (pp_axis.find("x") != std::string::npos) {
        std::cerr <<"\nFiltering planes perpendicular to x-axis...\n";

        // create the filtering object + segmentation parameters
        pcl::ModelCoefficients::Ptr coefficients_sac_yz (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_sac_yz (new pcl::PointIndices);

        pcl::SACSegmentation<PointT> sac_yz;
        sac_yz.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
        sac_yz.setMethodType (pcl::SAC_RANSAC);
        sac_yz.setOptimizeCoefficients (true);
        sac_yz.setDistanceThreshold (d_thresh_yz); // float distance_threshold for plane model estimation, walls (xz, yz planes) need higher threshold (typically)
        sac_yz.setAxis(Eigen::Vector3f(1,0,0)); // planes perpendicular to axis: (0,0,1) -> xy, (1,0,0) -> yz, (0,1,0) -> xz
        sac_yz.setEpsAngle(pcl::deg2rad(eps_angle_yz)); // rad*180/pi = deg
        sac_yz.setInputCloud (cloud);

        //int counter_while_sac_yz = 0;
        int max_iterations_sac_yz = 1;
        int pre_inliers_sac_yz = 0; // previous model inliers, loop iteration i-1
        int iter_start_yz =500;
        bool true_sac_yz = true;

        while (true_sac_yz) {

            sac_yz.setMaxIterations(iter_start_yz*max_iterations_sac_yz);
            sac_yz.segment (*inliers_sac_yz, *coefficients_sac_yz); //(*PointIndices, *ModelCoefficients)

            if (inliers_sac_yz->indices.size () == 0) {
                PCL_ERROR ("Could not estimate a planar model on yz-plane for the input cloud.\n");
                break;
                //return (-1);
            }

            if ( counter_while_sac_yz > 0 && inliers_sac_yz->indices.size() < pre_inliers_sac_yz*multiplier_yz) { // stops if "gradient" too small
                std::cout <<"\nCould not estimate a planar model on yz-plane for remaining points in cloud.\n";
                true_sac_yz=false;
                break;
            }

            // extract inliers
            pcl::ExtractIndices<PointT> extract_sac_yz;
            extract_sac_yz.setInputCloud (cloud);
            extract_sac_yz.setIndices (inliers_sac_yz);
            extract_sac_yz.setNegative (false); // extract the inliers in consensus model (the part to be removed from point cloud)
            extract_sac_yz.filter (*cloud_inliers_sac_yz); // cloud_inliers contains the found plane

            // extract outliers
            extract_sac_yz.setNegative (true);	// extract the outliers (interesting part of point cloud)
            extract_sac_yz.filter (*cloud_outliers_sac_yz); // cloud_outliers contains everything but the plane(s)

            *cloud=*cloud_outliers_sac_yz;
            *cloud_inliers_sac_yz_out+=*cloud_inliers_sac_yz;

            std::cout <<"\nCounter iteration yz-plane: " << counter_while_sac_yz << "\n";
            std::cout <<"SAC max. iterations yz-plane: " << iter_start_yz*(max_iterations_sac_yz) << "\n";
            std::cout <<"SAC inliers in iteration yz-plane: " << inliers_sac_yz->indices.size () << "\n";
            std::cout <<"SAC inliers in previous iteration: " << pre_inliers_sac_yz << "\n";

            counter_while_sac_yz++;
            max_iterations_sac_yz=counter_while_sac_yz*20;
            pre_inliers_sac_yz = inliers_sac_yz->indices.size ();

            if (one_all =="one"){
                true_sac_yz=false;
            }

            if (vm.count("max-i")!= 0 && counter_while_sac_yz > (max_iterations-1)) {
                true_sac_yz=false;
            }
        }

        std::cerr << "PointCloud after filtering (inlieres perpendicular to x): " << cloud_inliers_sac_yz_out->width * cloud_inliers_sac_yz_out->height
                  << " data points (" << pcl::getFieldsList (*cloud_inliers_sac_yz_out) << ").\n";

        std::cerr << "PointCloud after filtering (outliers perpendicular to x): " << cloud->width * cloud->height
                  << " data points (" << pcl::getFieldsList (*cloud) << ").\n";
    }
    //-------------------------

    int cloud_filtered_size_final = cloud->width * cloud->height;

    // create timestamp for directories where point clouds will be saved
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [64];
    time (&rawtime);
    timeinfo = localtime (&rawtime);
    strftime (buffer,64,"%Y-%m-%d-%H-%M",timeinfo); //Year, Month, Day, Hour, Minute

    // create output directory
    std::string ss_output_files;
    ss_output_files = "../output_files";
    boost::filesystem::path output_files_path (ss_output_files);


    if (boost::filesystem::is_directory (output_files_path)) {
        std::cerr <<"\nOutput directory already exists. Output will be saved in \"/output_files\"\n";
    }

    else {
        boost::filesystem::create_directory(ss_output_files); // create folder in root
        std::cerr <<"\nOutput directory was created. Output will be saved in \"/output_files\"\n";
    }


    // create save directory where edited point clouds will be saved
    std::stringstream ss_folder_path_pc_sac;
    ss_folder_path_pc_sac << ss_output_files << "/" << buffer << "_cloud_plane_seg";
    boost::filesystem::create_directory(ss_folder_path_pc_sac.str()); // create folder in output dir


    // create save path for outliers cloud
    std::stringstream ss_outliers_save;
    ss_outliers_save << "./" << ss_folder_path_pc_sac.str() << "/" << file_name_no_ext << "_" << pp_axis <<"_planes_removed." << output_format; // save filtered point cloud (outliers) in folder in current path

    // save filtered clouds as .ply
    if (output_format == "ply") {

        // sac model for all planes in selection -> xy, xz, yz plane(s) removed
        pcl::io::savePLYFileASCII(ss_outliers_save.str(), *cloud);

        if (pp_axis.find("z") != std::string::npos) {

            // sac_xy .ply -> inliers sac model xy-plane(s) (the part which is removed)
            std::stringstream ss_inliers_xy_save;
            ss_inliers_xy_save << "./" << ss_folder_path_pc_sac.str() << "/" << file_name_no_ext << "_sac_xy_inliers." << output_format; // save filtered point cloud (inliers) in folder in current path
            pcl::io::savePLYFileASCII(ss_inliers_xy_save.str(), *cloud_inliers_sac_xy_out);
        }

        if (pp_axis.find("y") != std::string::npos) {

            // sac_xz .ply -> inliers sac model xz-plane(s) (the part which is removed)
            std::stringstream ss_inliers_xz_save;
            ss_inliers_xz_save << "./" << ss_folder_path_pc_sac.str() << "/" << file_name_no_ext << "_sac_xz_inliers." << output_format; // save filtered point cloud (inliers) in folder in current path
            pcl::io::savePLYFileASCII(ss_inliers_xz_save.str(), *cloud_inliers_sac_xz_out);
        }

        if (pp_axis.find("x") != std::string::npos) {

            // sac_yz .ply -> inliers sac model yz-plane(s) (the part which is removed)
            std::stringstream ss_inliers_yz_save;
            ss_inliers_yz_save << "./" << ss_folder_path_pc_sac.str() << "/" << file_name_no_ext << "_sac_yz_inliers." << output_format; // save filtered point cloud (inliers) in folder in current path
            pcl::io::savePLYFileASCII(ss_inliers_yz_save.str(), *cloud_inliers_sac_yz_out);
        }
    }

    // save filtered clouds as .pcd
    else if (output_format == "pcd") {

        // sac model for all planes in selection -> xy, xz, yz plane(s) removed
        pcl::io::savePCDFileASCII(ss_outliers_save.str(), *cloud);

        if (pp_axis.find("z") != std::string::npos) {

            // sac_xy .ply -> inliers sac model xy-plane(s) (the part which is removed)
            std::stringstream ss_inliers_xy_save;
            ss_inliers_xy_save << "./" << ss_folder_path_pc_sac.str() << "/" << file_name_no_ext << "_sac_xy_inliers." << output_format; // save filtered point cloud (inliers) in folder in current path
            pcl::io::savePCDFileASCII(ss_inliers_xy_save.str(), *cloud_inliers_sac_xy_out);
        }

        if (pp_axis.find("y") != std::string::npos) {

            // sac_xz .ply -> inliers sac model xz-plane(s) (the part which is removed)
            std::stringstream ss_inliers_xz_save;
            ss_inliers_xz_save << "./" << ss_folder_path_pc_sac.str() << "/" << file_name_no_ext << "_sac_xz_inliers." << output_format; // save filtered point cloud (inliers) in folder in current path
            pcl::io::savePCDFileASCII(ss_inliers_xz_save.str(), *cloud_inliers_sac_xz_out);
        }

        if (pp_axis.find("x") != std::string::npos) {

            // sac_yz .ply -> inliers sac model yz-plane(s) (the part which is removed)
            std::stringstream ss_inliers_yz_save;
            ss_inliers_yz_save << "./" << ss_folder_path_pc_sac.str() << "/" << file_name_no_ext << "_sac_yz_inliers." << output_format; // save filtered point cloud (inliers) in folder in current path
            pcl::io::savePCDFileASCII(ss_inliers_yz_save.str(), *cloud_inliers_sac_yz_out);
        }
    }


    // create and write some information to log file
    std::ofstream logfile ("./" + ss_folder_path_pc_sac.str() + "/" + buffer + "_" + file_name_no_ext +"_sac_log.txt");

    logfile << "Point cloud file name: " << file_name_with_ext << std::endl;
    logfile << "SAC perpendicular plane model axis/axes selected: " << pp_axis << std::endl;
    logfile << "Setting all planes/one plane: " << one_all << std::endl;
    logfile << "Setting max. iterations per axis: " << max_iterations << std::endl;

    logfile << "\nPoint cloud before filtering: " << cloud_unfiltered_size
            << " data points (" << cloud_unfiltered_data_points << ")." << std::endl;

    logfile << "Point cloud after filtering (outliers): " << cloud_filtered_size_final
            << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

    logfile << (static_cast<double>(cloud_filtered_size_final)/static_cast<double>(cloud_unfiltered_size))*100 << "% of all points are outliers / not within segmented plane(s)." << std::endl;

    if (pp_axis.find("z") != std::string::npos) {

        logfile << "\nSAC perpendicular plane z-axis eps angle (xy-plane): " << eps_angle_xy << "°" << std::endl;
        logfile << "SAC perpendicular plane distance threshold (xy-plane): " << d_thresh_xy << std::endl;
        logfile << "SAC search/sample termination condition threshold multiplier (xy-plane): " << multiplier_xy << std::endl;
        logfile << "SAC plane models found (xy-plane): " << counter_while_sac_xy << std::endl;
        logfile << "Point cloud after filtering (inliers xy-plane): " << cloud_inliers_sac_xy_out->width * cloud_inliers_sac_xy_out->height
                << " data points (" << pcl::getFieldsList (*cloud_inliers_sac_xy_out) << ")." << std::endl;
    }

    if (pp_axis.find("y") != std::string::npos) {
        logfile << "\nSAC perpendicular plane y-axis eps angle (xz-plane): " << eps_angle_xz << "°" << std::endl;
        logfile << "SAC perpendicular plane distance threshold (xz-plane): " << d_thresh_xz << std::endl;
        logfile << "SAC search/sample termination condition threshold multiplier (xz-plane): " << multiplier_xz << std::endl;
        logfile << "SAC plane models found (xz-plane): " << counter_while_sac_xz << std::endl;
        logfile << "Point cloud after filtering (inliers xz-plane): " << cloud_inliers_sac_xz_out->width * cloud_inliers_sac_xz_out->height
                << " data points (" << pcl::getFieldsList (*cloud_inliers_sac_xz_out) << ")." << std::endl;
    }

    if (pp_axis.find("x") != std::string::npos) {
        logfile << "\nSAC perpendicular plane x-axis eps angle (yz-plane): " << eps_angle_yz << "°" << std::endl;
        logfile << "SAC perpendicular plane distance threshold (yz-plane): " << d_thresh_yz << std::endl;
        logfile << "SAC search/sample termination condition threshold multiplier (yz-plane): " << multiplier_yz << std::endl;
        logfile << "SAC plane models found (yz-plane): " << counter_while_sac_yz << std::endl;
        logfile << "Point cloud after filtering (inliers yz-plane): " << cloud_inliers_sac_yz_out->width * cloud_inliers_sac_yz_out->height
                << " data points (" << pcl::getFieldsList (*cloud_inliers_sac_yz_out) << ")." << std::endl;
    }

    logfile.close();


    // Visualization after filtering process has finished
    // planes perpendicular to axis: (0,0,1) -> xy, (1,0,0) -> yz, (0,1,0) -> xz
    // inliers colored standard colors for axes: x=RED, y=GREEN, z=BLUE

    // coordinate system for viewer output
    PointT min_p1, max_p1, label_p1; // scaling + coordinate system axis label
    pcl::getMinMax3D (*cloud, min_p1, max_p1);
    Eigen::Vector3f pc_scale;
    pc_scale = max_p1.getVector3fMap() - min_p1.getVector3fMap();
    float scaling_factor = (pc_scale(0) + pc_scale(1) + pc_scale(2)) / 3; // average scale of the point cloud -> scale coordinate system cross in viewer

    pcl::visualization::PCLVisualizer viewer ("PCL visualizer");
    viewer.addCoordinateSystem (scaling_factor*0.5);

    label_p1.getArray3fMap() << scaling_factor*0.5, 0, 0;
    viewer.addText3D ("x", label_p1, scaling_factor*0.05, 1, 0, 0);

    label_p1.getArray3fMap() << 0, scaling_factor*0.5, 0;
    viewer.addText3D ("y", label_p1, scaling_factor*0.05, 0, 1, 0);

    label_p1.getArray3fMap() << 0, 0, scaling_factor*0.5;
    viewer.addText3D ("z", label_p1, scaling_factor*0.05, 0, 0, 1);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_inliers_handler_xy (cloud_inliers_sac_xy_out, 20, 20, 255); // show inliers for z (defined plane in vector) in BLUE
    viewer.addPointCloud (cloud_inliers_sac_xy_out, cloud_inliers_handler_xy, "cloud inliers z");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_inliers_handler_xz (cloud_inliers_sac_xz_out, 20, 255, 20); // show inliers (defined plane in vector) in GREEN
    viewer.addPointCloud (cloud_inliers_sac_xz_out, cloud_inliers_handler_xz, "cloud inliers y");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_inliers_handler_yz (cloud_inliers_sac_yz_out, 255, 20, 20); // show inliers (defined plane in vector) in RED
    viewer.addPointCloud (cloud_inliers_sac_yz_out, cloud_inliers_handler_yz, "cloud inliers x");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_outliers_handler (cloud, 200, 200, 200); // show everything else (outliers) in GRAY
    viewer.addPointCloud (cloud, cloud_outliers_handler, "cloud outliers");

    viewer.resetCamera(); // reset camera parameters and render -> top view whole cloud

    while (!viewer.wasStopped ()) {
        viewer.spinOnce (100);
    }

    return (0);
}
