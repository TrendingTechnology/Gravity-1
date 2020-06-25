//
//  LiDAR_main.cpp
//  Gravity
//
//  Created by Hassan Hijazi on 3 April 2020.
//
//
#include <stdio.h>
#include <gravity/solver.h>
#ifdef USE_OPT_PARSER
#include <optionParser.hpp>
#endif
#include <gravity/rapidcsv.h>
#include <gravity/matplotlibcpp.h>
#include <DataSet.h>
#include "lasreader.hpp"
#include "laswriter.hpp"
#include <time.h>
using namespace std;

int main (int argc, char * argv[])
{
    int output = 0;
    double tol = 1e-6;
    double solver_time_end, total_time_end, solve_time, total_time;
    double grid_step = 1;
    map<pair<int,int>,tuple<int,double,double,double,UAVPoint*>> gridz1, gridz2, gridz, gridy1, gridy2, gridy, gridx1, gridx2, gridx;
    /* Grid with all cells where <x,y> is the key and <nb_measurements, min_z, max_z, av_z> is the cell data */
    int ground_z = 0;
    int ground_x = 0;
    int ground_y = 0;
    string log_level="0";
    string LiDAR_file1 = string(prj_dir)+"/data_sets/LiDAR/test1.las";
    string LiDAR_file2 = string(prj_dir)+"/data_sets/LiDAR/test2.las";
    string GPS_file = string(prj_dir)+"/data_sets/LiDAR/RawGNSS.csv";
    
#ifdef USE_OPT_PARSER
    /** create a OptionParser with options */
    op::OptionParser opt;
    opt.add_option("h", "help", "shows option help"); // no default value means boolean options, which default value is false
    opt.add_option("f", "file", "Input file name (def. ../data_sets/LiDAR/*)", fname_phi );
    opt.add_option("l", "log", "Log level (def. 0)", log_level );
    
    /** parse the options and verify that all went well. If not, errors and help will be shown */
    bool correct_parsing = opt.parse_options(argc, argv);
    
    if(!correct_parsing){
        return EXIT_FAILURE;
    }
    
    LiDAR_file = opt["f"];
    output = op::str2int(opt["l"]);
    output = 5;
    bool has_help = op::str2bool(opt["h"]);
    /** show help */
    if(has_help) {
        opt.show_help();
        exit(0);
    }
#else
    if(argc>1){
        LiDAR_file1 = argv[1];
    }
    if(argc>2){
        LiDAR_file2 = argv[2];
    }
    if(argc>3){
        GPS_file = argv[3];
    }
    if(argc>4){
        char *p;
        grid_step =  strtol(argv[4], &p, 10);
    }
#endif

    rapidcsv::Document  GPS_data(GPS_file);
    int n = GPS_data.GetRowCount();
    vector<UAVPoint> UAVPoints(n);
    vector<LidarPoint> LidarPointsz;
    vector<LidarPoint> LidarPointsx;
    vector<LidarPoint> LidarPointsy;
    map<double,Frame> framesz, frames1z, framesx, frames1x, framesy, frames1y;
    vector<double> uav_x, uav_y, uav_z;
    vector<double> uav_x1, uav_y1, uav_z1;
    for (int i = 0; i< n-1; i++) { // Input iterator
        UAVPoints[i]._x = GPS_data.GetCell<double>("UtmPos_X", i);
        UAVPoints[i]._y = GPS_data.GetCell<double>("UtmPos_Y", i);
        UAVPoints[i]._latitude = GPS_data.GetCell<double>("AbsPos_Y", i);
        UAVPoints[i]._longitude = GPS_data.GetCell<double>("AbsPos_X", i);
        UAVPoints[i]._height = GPS_data.GetCell<double>("AbsPos_Z", i);
        UAVPoints[i]._roll = GPS_data.GetCell<double>("Roll", i);
        UAVPoints[i]._pitch = GPS_data.GetCell<double>("Pitch", i);
        UAVPoints[i]._yaw = GPS_data.GetCell<double>("Heading", i);
        double unix_time = round(GPS_data.GetCell<double>("Time", i)*10);
        UAVPoints[i].set_unix_time(unix_time);
//        uav_x.push_back(UAVPoints[i]._longitude+582690.8242);
//        uav_y.push_back(UAVPoints[i]._latitude+4107963.58);
//        uav_z.push_back(UAVPoints[i]._height);
//        DebugOff("Unix time = " << GPS_data.GetCell<double>("Unix Time", i) << endl);
//        DebugOff("Latitude = " << GPS_data.GetCell<double>("Latitude (degrees)", i) << endl);
//        DebugOff("Longitude = " << GPS_data.GetCell<double>("Longitude (degrees)", i) << endl);
//        DebugOff("Height = " << GPS_data.GetCell<double>("Height", i) << endl);
//        auto frame_ptr = frames.insert(make_pair(UAVPoints[i]._unix_time, Frame(UAVPoints[i]._unix_time)));
//        frame_ptr.first->second.add_UAV_point(UAVPoints[i]);
//        UAVPoints[i]._latitude = GPS_data.GetCell<double>("Latitude (degrees)", i);
//        UAVPoints[i]._longitude = GPS_data.GetCell<double>("Longitude (degrees)", i);
//        UAVPoints[i]._height = GPS_data.GetCell<double>("Height", i);
//        UAVPoints[i].set_unix_time(GPS_data.GetCell<double>("Unix Time", i));
        /* If longitude = -116.06925521318, x = 582690.824176164 */
//        uav_x.push_back((UAVPoints[i]._longitude*582690.824176164/(-116.06925521318))*1e-5);
        uav_x.push_back(UAVPoints[i]._x);
        uav_y.push_back(UAVPoints[i]._y);
        uav_z.push_back(UAVPoints[i]._height);
        DebugOff("Unix time = " << GPS_data.GetCell<double>("Unix Time", i) << endl);
        auto frame_ptrz = framesz.insert(make_pair(UAVPoints[i]._unix_time, Frame(UAVPoints[i]._unix_time)));
        auto frame_ptrx = framesx.insert(make_pair(UAVPoints[i]._unix_time, Frame(UAVPoints[i]._unix_time)));
        auto frame_ptry = framesy.insert(make_pair(UAVPoints[i]._unix_time, Frame(UAVPoints[i]._unix_time)));
        frame_ptrz.first->second.add_UAV_point(UAVPoints[i]);
        frame_ptrx.first->second.add_UAV_point(UAVPoints[i]);
        frame_ptry.first->second.add_UAV_point(UAVPoints[i]);
    }
    DebugOn("Read " << n << "UAV points" << endl);
//    LiDAR_file = "/Users/l297598/Downloads/PostPost_DAG4_lidar_LDRD_Reserve_20200326_LAS_and_ANPP_files/Roll_Pitch_Yaw_automation_2020_03_26/3.ins_postpost_SGZ_DAG4_L_2_2019_07_31_18_combined_LDRD_Reserve_20200326_Frames_2130_2222_RPY_1.375_0.9_-0.55_NoKin_adc.las";
//   LiDAR_file = "/Users/l297598/Downloads/PostPost_DAG4_lidar_LDRD_Reserve_20200326_LAS_and_ANPP_files/Roll_Pitch_Yaw_automation_2020_03_26/1.raw_postpost_SGZ_DAG4_L_2_2019_07_31_18_combined_LDRD_Reserve_20200326_Frames_2130_2222_RPY_0_0_0_NoKin_adc.las";
//    LiDAR_file = "/Users/l297598/Downloads/PostPost_DAG4_lidar_LDRD_Reserve_20200326_LAS_and_ANPP_files/Roll_Pitch_Yaw_automation_2020_03_26/6.ins_postpost_SGZ_DAG4_L_2_2019_07_31_18_combined_LDRD_Reserve_20200326_Frames_2130_2222_and_2511_2587_RPY_1.375_0.9_-0.55_NoKin_adc.las";
//   LiDAR_file = "/Users/l297598/Downloads/PostPost_DAG4_lidar_LDRD_Reserve_20200326_LAS_and_ANPP_files/Roll_Pitch_Yaw_automation_2020_03_26/5.raw_postpost_SGZ_DAG4_L_2_2019_07_31_18_combined_LDRD_Reserve_20200326_Frames_2130_2222_and_2511_2587_RPY_0_0_0_NoKin_adc.las";
//    LiDAR_file = "/Users/l297598/Downloads/PostPost_DAG4_lidar_LDRD_Reserve_20200326_LAS_and_ANPP_files/Roll_Pitch_Yaw_automation_2020_03_26/7.raw_postpost_SGZ_DAG4_L_2_2019_07_31_18_combined_LDRD_Reserve_20200326_Frames_2130_2587_RPY_0_0_0_NoKin_adc.las";
//    LiDAR_file = "/Users/l297598/Downloads/PostPost_DAG4_lidar_LDRD_Reserve_20200326_LAS_and_ANPP_files/Roll_Pitch_Yaw_automation_2020_03_26/8.ins_postpost_SGZ_DAG4_L_2_2019_07_31_18_combined_LDRD_Reserve_20200326_Frames_2130_2587_RPY_1.375_0.9_-0.55_NoKin_adc.las";
//    LiDAR_file = "/Users/l297598/Downloads/PostPost_DAG4_lidar_LDRD_Reserve_20200326_LAS_and_ANPP_files/PostPost_DAG4_lidar_LDRD_Reserve_20200326_LAS_files/raw_postpost_1g11d_DAG4_L_2_2019_07_31_18_combined_LDRD_Reserve_20200326_Frames_2892_3073_RPY_0_0_0_NoKin_adc.las";
//    LiDAR_file = "/Users/l297598/Downloads/PostPost_DAG4_lidar_LDRD_Reserve_20200326_LAS_and_ANPP_files/PostPost_DAG4_lidar_LDRD_Reserve_20200326_LAS_files/ins_postpost_1g11d_DAG4_L_2_2019_07_31_18_combined_LDRD_Reserve_20200326_Frames_2892_3073_RPY_1.375_0.9_-0.55_NoKin_adc.las";
    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(LiDAR_file1.c_str());
    lasreadopener.set_populate_header(TRUE);
    param<> x1("x1"), x2("x2"), y1("y1"), y2("y2"), z1("z1"), z2("z2");
    param<> x_uav1("x_uav1"), y_uav1("y_uav1"), z_uav1("z_uav1");
    param<> x_uav2("x_uav2"), y_uav2("y_uav2"), z_uav2("z_uav2");
    param<> x_uav("x_uav"), y_uav("y_uav"), z_uav("z_uav");
    param<> pre_x("pre_x"), pre_y("pre_y"), pre_z("pre_z");
    int xdim1=0, ydim1=0, zdim1=0;
    if (!lasreadopener.active())
    {
        throw invalid_argument("ERROR: no input specified\n");
    }
    vector<double> x_vec1,y_vec1,z_vec1,zmin_vec1,zmax_vec1, xmin_vec1, xmax_vec1, ymin_vec1, ymax_vec1;
    vector<double> x_vec2,y_vec2,z_vec2,zmin_vec2,zmax_vec2, xmin_vec2, xmax_vec2, ymin_vec2, ymax_vec2;
    vector<double> x_shift1,y_shift1,z_shift1;
    vector<double> x_shift2,y_shift2,z_shift2;
    vector<double> x_shift,y_shift,z_shift;
    vector<double> uav_roll1,uav_pitch1,uav_yaw1;
    vector<double> uav_roll2,uav_pitch2,uav_yaw2;
    vector<double> x_combined,y_combined,z_combined,zmin_combined,zmax_combined,xmin_combined,xmax_combined,ymin_combined, ymax_combined;
    set<double> timestamps;
    while (lasreadopener.active())
    {
        LASreader* lasreader = lasreadopener.open();
        if (lasreader == 0)
        {
            throw invalid_argument("ERROR: could not open lasreader\n");
        }

        xdim1 = ceil(lasreader->header.max_x*grid_step - lasreader->header.min_x*grid_step);
        ydim1 = ceil(lasreader->header.max_y*grid_step - lasreader->header.min_y*grid_step);
//      zdim1 = ceil(lasreader->header.max_z*100*grid_step - lasreader->header.min_z*100*grid_step);
        ground_z = floor(lasreader->header.min_z);
//        ground_x = floor(lasreader->header.min_x);
//        ground_y = floor(lasreader->header.min_y);
        size_t nb_pts_per_framez = lasreader->npoints/framesz.size()+1;
//        size_t nb_pts_per_framex = lasreader->npoints/framesx.size()+1;
//        DebugOn("nb_pts_per_framex" << nb_pts_per_framex << endl);
//        size_t nb_pts_per_framey = lasreader->npoints/framesy.size()+1;
        DebugOn("Number of points = " << lasreader->npoints << endl);
//      DebugOn("Number of points per frame = " <<  nb_pts_per_frame << endl);
        DebugOn("min x axis = " << lasreader->header.min_x << endl);
        DebugOn("max x axis = " << lasreader->header.max_x << endl);
        DebugOn("min y axis = " << lasreader->header.min_y << endl);
        DebugOn("max y axis = " << lasreader->header.max_y << endl);
        DebugOn("min z axis = " << lasreader->header.min_z << endl);
        DebugOn("max z axis = " << lasreader->header.max_z << endl);
//        uav_x1.push_back(lasreader->header.min_x);
//        uav_y1.push_back(lasreader->header.min_y);
//        uav_z1.push_back(ground_z);
        DebugOn("dimension in x axis = " << xdim1 << endl);
        DebugOn("dimension in y axis = " << ydim1 << endl);
//        DebugOn("dimension in z axis = " << zdim1 << endl);
//
//
        int nb_dots; /* Number of measurements inside cell */
        int xpos, ypos;
        double z, min_z, max_z, av_z;
        pair<int,int> posz;
        size_t nb_ptsz = 0;
        tuple<int,double,double,double,UAVPoint*> cellz; /* <nb_dots,min_z,max_z,av_z> */
        /* Now get rid of the first points
        lasreader->read_point();
        auto gps_time = lasreader->point.get_gps_time();
        while (lasreader->point.get_gps_time()==gps_time)
        {
            lasreader->read_point();
        }
         */
        auto framesz_it = framesz.begin();
        while (lasreader->read_point())
        {
            if(nb_ptsz==0){
                DebugOn(to_string_with_precision(10.*(lasreader->point.get_gps_time()+315964800. - 18.),24) << ": (" << to_string_with_precision(lasreader->point.get_x(),10) <<"," << to_string_with_precision(lasreader->point.get_y(),10) << ","<< to_string_with_precision(lasreader->point.get_z(),10) <<")"<<endl);
//                return 0;
            }
            xpos = (round((lasreader->point.get_x()*grid_step)));
            ypos = (round((lasreader->point.get_y()*grid_step)));
            if (ypos < 0)
            {
                fprintf(stderr, "ERROR: ypos = %d\n", ypos);
            }


            //            z = floor((lasreader->point.get_z()*100*grid_step - lasreader->header.min_z*100*grid_step));
//            x = lasreader->point.get_x();
//            y = lasreader->point.get_y();
            z = lasreader->point.get_z();
            timestamps.insert(lasreader->point.get_gps_time()+1./(nb_ptsz%10));
            DebugOff("GPS time = " << to_string_with_precision(lasreader->point.get_gps_time(),24) << endl);
            LidarPointsz.push_back(LidarPoint(lasreader->point.get_gps_time(),xpos,ypos,z));

            if(nb_ptsz>=nb_pts_per_framez && nb_ptsz%nb_pts_per_framez==0 && next(framesz_it)!=framesz.end()){
                framesz_it++;
            }
            auto framez_ptr = framesz.insert(make_pair(framesz_it->first, Frame(LidarPointsz.back()._unix_time*10+(nb_ptsz%10))));
            framez_ptr.first->second.add_lidar_point(LidarPointsz.back());
            if(framez_ptr.second){
                throw invalid_argument("Frame with missing UAV point at " + to_string(LidarPointsz.back()._unix_time*10+(nb_ptsz%10)));
            }
            else{
                LidarPointsz.back()._uav_pt = framez_ptr.first->second._uav_point;
                frames1z.insert(make_pair(LidarPointsz.back()._unix_time*10+(nb_ptsz%10), Frame(framez_ptr.first->second)));
//                uav_x1.push_back((frame_ptr.first->second._uav_points.front()->_longitude+582690.8242)*1e-5);
//                uav_y1.push_back((frame_ptr.first->second._uav_points.front()->_latitude+4107963.58)*1e-5);
//                uav_z1.push_back(frame_ptr.first->second._uav_points.front()->_height*100);
            }
            DebugOff("Added lidar point to frame at " << LidarPointsz.back()._unix_time << " : " << LidarPointsz.back()._hour << ":" << LidarPointsz.back()._minutes << ":" << LidarPointsz.back()._seconds << endl);
            //            xpos = floor((lasreader->point.get_x()*grid_step - lasreader->header.min_x*grid_step));
            //            ypos = floor((lasreader->point.get_y()*grid_step - lasreader->header.min_y*grid_step));

            //            z = round((lasreader->point.get_z()*100*grid_step));
//            xpos -= frame_ptr.first->second._uav_point->_x;
//            ypos -= frame_ptr.first->second._uav_point->_y;
//            z -= frame_ptr.first->second._uav_point->_height;
            posz = make_pair(xpos,ypos);
            cellz = make_tuple(1,z,z,z,framez_ptr.first->second._uav_point);
            DebugOff("new z value = " << z << endl);
            DebugOff("fractional z value = " << lasreader->point.get_z() << endl);
            DebugOff("z range = " << grid_maxz[pos] - grid_minz[pos] << endl << endl);
            auto map_pair = gridz1.insert(make_pair(posz,cellz));
            auto map_pair_all = gridz.insert(make_pair(posz,cellz));
            if (!map_pair.second) {/* Cell already exists */
//                DebugOn("Found overlapping cell after " << nb_pts << endl);
//                DebugOn("(" << to_string_with_precision(lasreader->point.get_x(),10) <<"," << to_string_with_precision(lasreader->point.get_y(),10) << ","<< to_string_with_precision(lasreader->point.get_z(),10) <<")"<<endl);
//                                return 0;
                nb_dots = get<0>(map_pair.first->second);
                min_z = get<1>(map_pair.first->second);
                max_z = get<2>(map_pair.first->second);
                av_z = get<3>(map_pair.first->second);
                /* Update cell data */
                get<0>(map_pair.first->second)++;
                if(min_z>z){
                    DebugOff("updating  min z at cell [" << xpos << "," << ypos << "]" << endl);
                    DebugOff("previous min z = " << min_z << endl);
                    DebugOff("new min z = " << z << endl);
                    get<1>(map_pair.first->second) = z;
                }
                if(max_z<z){
                    DebugOff("updating  max z at cell [" << xpos << "," << ypos << "]" << endl);
                    DebugOff("previous max z = " << max_z << endl);
                    DebugOff("new max z = " << z << endl);
                    get<2>(map_pair.first->second) = z;
                }
                av_z *= nb_dots;/* remove previous denominator */
                get<3>(map_pair.first->second) = (av_z + z)/(nb_dots+1); /* Update denominator */
            }
            if (!map_pair_all.second) {/* Cell already exists */
                nb_dots = get<0>(map_pair_all.first->second);
                min_z = get<1>(map_pair_all.first->second);
                max_z = get<2>(map_pair_all.first->second);
                av_z = get<3>(map_pair_all.first->second);
                /* Update cell data */
                get<0>(map_pair_all.first->second)++;
                if(min_z>z){
                    get<1>(map_pair_all.first->second) = z;
                }
                if(max_z<z){
                    get<2>(map_pair_all.first->second) = z;
                }
                av_z *= nb_dots;/* remove previous denominator */
                get<3>(map_pair_all.first->second) = (av_z + z)/(nb_dots+1); /* Update denominator */
            }
            nb_ptsz++;
        }
        DebugOn("Read " << nb_ptsz << " points" << endl);
        DebugOn("2D grid has " << gridz1.size() << " cells " << endl);
        DebugOn("xdim = " << xdim1 << endl);
        DebugOn("ydim = " << ydim1 << endl);

        int cell_counter = 0, max_nb_dots = 0;
        double av_nb_dots = 0, max_z_range = 0, z_range = 0,  av_z_range = 0;
        for (auto &cellz : gridz1) {
            DebugOff("Cell num " << cell_counter++ << " at [" << cell.first.first << "," << cell.first.second << "]" << endl);
            nb_dots = get<0>(cellz.second);
            min_z = get<1>(cellz.second);
            max_z = get<2>(cellz.second);
            av_z = get<3>(cellz.second);
            x_vec1.push_back(cellz.first.first);
            x_shift1.push_back(get<4>(cellz.second)->_x);
            x_shift.push_back(get<4>(cellz.second)->_x);
            x_uav1.add_val(to_string(cellz.first.first)+","+to_string(cellz.first.second),get<4>(cellz.second)->_x);
            x1.add_val(to_string(cellz.first.first)+","+to_string(cellz.first.second), cellz.first.first);
            y_vec1.push_back(cellz.first.second);
            y_shift1.push_back(get<4>(cellz.second)->_y);
            y_shift.push_back(get<4>(cellz.second)->_y);
            y_uav1.add_val(to_string(cellz.first.first)+","+to_string(cellz.first.second),get<4>(cellz.second)->_y);
            y1.add_val(to_string(cellz.first.first)+","+to_string(cellz.first.second), cellz.first.second);
            z_vec1.push_back(av_z);
            zmin_vec1.push_back(min_z);
            zmax_vec1.push_back(max_z);
            z_shift1.push_back(get<4>(cellz.second)->_height);
            z_shift.push_back(get<4>(cellz.second)->_height);
            z_uav1.add_val(to_string(cellz.first.first)+","+to_string(cellz.first.second),get<4>(cellz.second)->_height);
            z1.add_val(to_string(cellz.first.first)+","+to_string(cellz.first.second), max_z);
            uav_roll1.push_back(get<4>(cellz.second)->_roll);
            uav_pitch1.push_back(get<4>(cellz.second)->_pitch);
            uav_yaw1.push_back(get<4>(cellz.second)->_yaw);
            if(max_nb_dots<nb_dots)
                max_nb_dots = nb_dots;
            av_nb_dots += nb_dots;
            z_range = max_z - min_z;
            DebugOff("z range = " << z_range << endl);
            av_z_range += z_range;
            if(z_range > max_z_range)
                max_z_range = z_range;
            DebugOff("z min = " << min_z << endl);
            DebugOff("z max = " << max_z << endl);
            DebugOff("z av = " << av_z << endl);
            DebugOff("nb points = " << nb_dots << endl << endl);
        }
        av_nb_dots /= gridz1.size();
        av_z_range /= gridz1.size();
        DebugOn("Max z range for a cell = " << max_z_range << endl);
        DebugOn("Average z range for a cell = " << av_z_range << endl);
        DebugOn("Average points per cell = " << av_nb_dots << endl);
        DebugOn("Max points per cell = " << max_nb_dots << endl << endl);
//        LASwriteOpener laswriteopener;
//        laswriteopener.set_file_name("compressed.laz");
//        LASwriter* laswriter = laswriteopener.open(&lasreader->header);
//
//        while (lasreader->read_point()) laswriter->write_point(&lasreader->point);
//
//        laswriter->close();
//        delete laswriter;
       lasreader->close();
        delete lasreader;
    }
    lasreadopener.set_file_name(LiDAR_file1.c_str());
    lasreadopener.set_populate_header(TRUE);
    if (!lasreadopener.active())
    {
        throw invalid_argument("ERROR: no input specified\n");
    }
    while (lasreadopener.active())
    {
        LASreader* lasreader = lasreadopener.open();
        if (lasreader == 0)
        {
            throw invalid_argument("ERROR: could not open lasreader for file\n");
        }
        ydim1 = ceil(lasreader->header.max_y*grid_step - lasreader->header.min_y*grid_step);
        zdim1 = ceil(lasreader->header.max_z*100*grid_step - lasreader->header.min_z*100*grid_step);
        ground_x = floor(lasreader->header.min_x);
       size_t nb_pts_per_framex = lasreader->npoints/framesx.size()+1;
       DebugOn("nb_pts_per_framex" << nb_pts_per_framex << endl);
       DebugOn("Number of points = " << lasreader->npoints << endl);

       DebugOn("dimension in z axis = " << zdim1 << endl);
       DebugOn("dimension in y axis = " << ydim1 << endl);
        //        DebugOn("dimension in z axis = " << zdim1 << endl);
                
                
       int nb_dots; /* Number of measurements inside cell */
       int ypos, zpos;
       double x, min_x, max_x, av_x;
       pair<int,int> posx;
       size_t nb_ptsx = 0;
       tuple<int,double,double,double,UAVPoint*> cellx; /* <nb_dots,min_z,max_z,av_z> */
        
       auto frames_it = framesx.begin();
             while (lasreader->read_point())
                {
                    
                    zpos = (round((lasreader->point.get_z()*grid_step)));
                    ypos = (round((lasreader->point.get_y()*grid_step)));
                    if (ypos < 0)
                    {
                        fprintf(stderr, "ERROR: ypos = %d\n", ypos);
                    }
                    

                    //            z = floor((lasreader->point.get_z()*100*grid_step - lasreader->header.min_z*100*grid_step));
                    x = lasreader->point.get_x();

                    timestamps.insert(lasreader->point.get_gps_time()+1./(nb_ptsx%10));
                    DebugOff("GPS time = " << to_string_with_precision(lasreader->point.get_gps_time(),24) << endl);
                    LidarPointsx.push_back(LidarPoint(lasreader->point.get_gps_time(),ypos, zpos,x));

                    if(nb_ptsx>=nb_pts_per_framex && nb_ptsx%nb_pts_per_framex==0 && next(frames_it)!=framesx.end()){
                        frames_it++;
                
                    }
                    auto framex_ptr = framesx.insert(make_pair(frames_it->first, Frame(LidarPointsx.back()._unix_time*10+(nb_ptsx%10))));
                    framex_ptr.first->second.add_lidar_point(LidarPointsx.back());
                    if(framex_ptr.second){
                        throw invalid_argument("Frame with missing UAV point at " + to_string(LidarPointsx.back()._unix_time*10+(nb_ptsx%10)));
                    }
                    else{
                        LidarPointsx.back()._uav_pt = framex_ptr.first->second._uav_point;
                        frames1x.insert(make_pair(LidarPointsx.back()._unix_time*10+(nb_ptsx%10), Frame(framex_ptr.first->second)));
        //                uav_x1.push_back((frame_ptr.first->second._uav_points.front()->_longitude+582690.8242)*1e-5);
        //                uav_y1.push_back((frame_ptr.first->second._uav_points.front()->_latitude+4107963.58)*1e-5);
        //                uav_z1.push_back(frame_ptr.first->second._uav_points.front()->_height*100);
                    }
                    DebugOff("Added lidar point to frame at " << LidarPointsx.back()._unix_time << " : " << LidarPointsx.back()._hour << ":" << LidarPointsx.back()._minutes << ":" << LidarPointsx.back()._seconds << endl);
                    //            xpos = floor((lasreader->point.get_x()*grid_step - lasreader->header.min_x*grid_step));
                    //            ypos = floor((lasreader->point.get_y()*grid_step - lasreader->header.min_y*grid_step));
                   
                    //            z = round((lasreader->point.get_z()*100*grid_step));
        //            xpos -= frame_ptr.first->second._uav_point->_x;
        //            ypos -= frame_ptr.first->second._uav_point->_y;
        //            z -= frame_ptr.first->second._uav_point->_height;
                    posx = make_pair(ypos,zpos);
                    cellx = make_tuple(1,x,x,x,framex_ptr.first->second._uav_point);
                
                    auto map_pair = gridx1.insert(make_pair(posx,cellx));
                    auto map_pair_all = gridx.insert(make_pair(posx,cellx));
                    if (!map_pair.second) {/* Cell already exists */
        //                DebugOn("Found overlapping cell after " << nb_pts << endl);
        //                DebugOn("(" << to_string_with_precision(lasreader->point.get_x(),10) <<"," << to_string_with_precision(lasreader->point.get_y(),10) << ","<< to_string_with_precision(lasreader->point.get_z(),10) <<")"<<endl);
        //                                return 0;
                        nb_dots = get<0>(map_pair.first->second);
                        min_x = get<1>(map_pair.first->second);
                        max_x = get<2>(map_pair.first->second);
                        av_x = get<3>(map_pair.first->second);
                        /* Update cell data */
                        get<0>(map_pair.first->second)++;
                        if(min_x>x){
                            DebugOff("updating  min x at cell [" << ypos << "," << zpos << "]" << endl);
                            DebugOff("previous min x = " << min_x << endl);
                            DebugOff("new min x = " << z << endl);
                            get<1>(map_pair.first->second) = x;
                        }
                        if(max_x<x){
                            DebugOff("updating  max x at cell [" << ypos << "," << zpos << "]" << endl);
                            DebugOff("previous max x = " << max_x << endl);
                            DebugOff("new max x = " << x << endl);
                            get<2>(map_pair.first->second) = x;
                        }
                        av_x *= nb_dots;/* remove previous denominator */
                        get<3>(map_pair.first->second) = (av_x + x)/(nb_dots+1); /* Update denominator */
                    }
                    if (!map_pair_all.second) {/* Cell already exists */
                        nb_dots = get<0>(map_pair_all.first->second);
                        min_x = get<1>(map_pair_all.first->second);
                        max_x = get<2>(map_pair_all.first->second);
                        av_x = get<3>(map_pair_all.first->second);
                        /* Update cell data */
                        get<0>(map_pair_all.first->second)++;
                        if(min_x>x){
                            get<1>(map_pair_all.first->second) = x;
                        }
                        if(max_x<x){
                            get<2>(map_pair_all.first->second) = x;
                        }
                        av_x *= nb_dots;/* remove previous denominator */
                        get<3>(map_pair_all.first->second) = (av_x + x)/(nb_dots+1); /* Update denominator */
                    }
                    nb_ptsx++;
                }
                DebugOn("Read " << nb_ptsx << " points yz plane file 1" << endl);
                DebugOn("2D grid has " << gridx1.size() << " cells " << endl);
                DebugOn("zdim = " << zdim1 << endl);
                DebugOn("ydim = " << ydim1 << endl);
                
                int cell_counterx = 0, max_nb_dotsx = 0;
                double av_nb_dotsx = 0, max_x_range = 0, x_range = 0,  av_x_range = 0;
                for (auto &cellx : gridx1) {
                    DebugOff("Cell num " << cell_counterx++ << " at [" << cellx.first.first << "," << cellx.first.second << "]" << endl);
                    nb_dots = get<0>(cellx.second);
                    min_x = get<1>(cellx.second);
                    max_x = get<2>(cellx.second);
                    av_x = get<3>(cellx.second);
                    y_vec1.push_back(cellx.first.first);
                    y_shift1.push_back(get<4>(cellx.second)->_y);
                    y_shift.push_back(get<4>(cellx.second)->_y);
                    y_uav1.add_val(to_string(cellx.first.first)+","+to_string(cellx.first.second),get<4>(cellx.second)->_y);
                    y1.add_val(to_string(cellx.first.first)+","+to_string(cellx.first.second), cellx.first.first);
                    z_vec1.push_back(cellx.first.second);
                    z_shift1.push_back(get<4>(cellx.second)->_height);
                    z_shift.push_back(get<4>(cellx.second)->_height);
                    z_uav1.add_val(to_string(cellx.first.first)+","+to_string(cellx.first.second),get<4>(cellx.second)->_height);
                    z1.add_val(to_string(cellx.first.first)+","+to_string(cellx.first.second), cellx.first.second);
                    x_vec1.push_back(av_x);
                    xmin_vec1.push_back(min_x);
                    xmax_vec1.push_back(max_x);
                    x_shift1.push_back(get<4>(cellx.second)->_x);
                    x_shift.push_back(get<4>(cellx.second)->_x);
                    x_uav1.add_val(to_string(cellx.first.first)+","+to_string(cellx.first.second),get<4>(cellx.second)->_x);
                    x1.add_val(to_string(cellx.first.first)+","+to_string(cellx.first.second), max_x);
                    uav_roll1.push_back(get<4>(cellx.second)->_roll);
                    uav_pitch1.push_back(get<4>(cellx.second)->_pitch);
                    uav_yaw1.push_back(get<4>(cellx.second)->_yaw);
                    if(max_nb_dotsx<nb_dots)
                        max_nb_dotsx = nb_dots;
                    av_nb_dotsx += nb_dots;
                    x_range = max_x - min_x;
                    DebugOff("x range = " << x_range << endl);
                    av_x_range += x_range;
                    if(x_range > max_x_range)
                        max_x_range = x_range;
                    DebugOff("x min = " << min_x << endl);
                    DebugOff("x max = " << max_x << endl);
                    DebugOff("x av = " << av_x << endl);
                    DebugOff("nb points = " << nb_dots << endl << endl);
                }
                av_nb_dotsx /= gridx1.size();
                av_x_range /= gridx1.size();
                DebugOn("Max x range for a cell = " << max_x_range << endl);
                DebugOn("Average x range for a cell = " << av_x_range << endl);
                DebugOn("Average points per cell = " << av_nb_dotsx << endl);
                DebugOn("Max points per cell = " << max_nb_dotsx << endl << endl);
        lasreader->close();
        delete lasreader;
    }
    
    int xdim2=0, ydim2=0, zdim2=0;
    
    lasreadopener.set_file_name(LiDAR_file2.c_str());
    lasreadopener.set_populate_header(TRUE);
    if (!lasreadopener.active())
    {
        throw invalid_argument("ERROR: no input specified\n");
    }
    while (lasreadopener.active())
    {
        LASreader* lasreader = lasreadopener.open();
        if (lasreader == 0)
        {
            throw invalid_argument("ERROR: could not open lasreader for second file\n");
        }

        xdim1 = ceil(lasreader->header.max_x*grid_step - lasreader->header.min_x*grid_step);
        ydim1 = ceil(lasreader->header.max_y*grid_step - lasreader->header.min_y*grid_step);
//        zdim1 = ceil(lasreader->header.max_z*grid_step - lasreader->header.min_z*grid_step);
        ground_z = floor(lasreader->header.min_z);
//        ground_x = floor(lasreader->header.min_x);
//        ground_y = floor(lasreader->header.min_y);
        auto nb_pts_per_frame = lasreader->npoints/framesz.size();
        DebugOn("Number of points = " << lasreader->npoints << endl);
        DebugOn("Number of points per frame = " <<  nb_pts_per_frame << endl);
        DebugOn("min x axis = " << lasreader->header.min_x << endl);
        DebugOn("max x axis = " << lasreader->header.max_x << endl);
        DebugOn("min y axis = " << lasreader->header.min_y << endl);
        DebugOn("max y axis = " << lasreader->header.max_y << endl);
        DebugOn("dimension in x axis = " << xdim1 << endl);
        DebugOn("dimension in y axis = " << ydim1 << endl);


        int nb_dots; /* Number of measurements inside cell */
        int xpos, ypos;
        double z, min_z, max_z, av_z;
        pair<int,int> posz;
        size_t nb_ptsz = 0;
        tuple<int,double,double,double,UAVPoint*> cellz; /* <nb_dots,min_z,max_z,av_z> */
        /* Now get rid of the first points
         lasreader->read_point();
         auto gps_time = lasreader->point.get_gps_time();
         while (lasreader->point.get_gps_time()==gps_time)
         {
         lasreader->read_point();
         }
         */
        auto framesz_it = framesz.begin();
        while (lasreader->read_point())
        {
            if(nb_ptsz==0){
                DebugOn(to_string_with_precision(10.*(lasreader->point.get_gps_time()+315964800. - 18.),24) << ": (" << to_string_with_precision(lasreader->point.get_x(),10) <<"," << to_string_with_precision(lasreader->point.get_y(),10) << ","<< to_string_with_precision(lasreader->point.get_z(),10) <<")"<<endl);
                //                return 0;
            }
            xpos = (round((lasreader->point.get_x()*grid_step)));
            ypos = (round((lasreader->point.get_y()*grid_step)));
            if (ypos < 0)
            {
                fprintf(stderr, "ERROR: ypos = %d\n", ypos);
            }


            //            z = lasreader->point.get_Z();
            //            z = floor((lasreader->point.get_z()*100*grid_step - lasreader->header.min_z*100*grid_step));
            z = lasreader->point.get_z();
            timestamps.insert(lasreader->point.get_gps_time()+1./(nb_ptsz%10));
            DebugOff("GPS time = " << to_string_with_precision(lasreader->point.get_gps_time(),24) << endl);
            LidarPointsz.push_back(LidarPoint(lasreader->point.get_gps_time(),xpos,ypos,z));
            if(nb_ptsz>=nb_pts_per_frame && nb_ptsz%nb_pts_per_frame==0 && next(framesz_it)!=framesz.end()){
                framesz_it++;
            }
            auto framez_ptr = framesz.insert(make_pair(framesz_it->first, Frame(LidarPointsz.back()._unix_time*10+(nb_ptsz%10))));
//            auto frame_ptr = frames.insert(make_pair(LidarPoints.back()._unix_time*10+(nb_pts%10), Frame(LidarPoints.back()._unix_time*10+(nb_pts%10))));
            framez_ptr.first->second.add_lidar_point(LidarPointsz.back());
            if(framez_ptr.second){
                throw invalid_argument("Frame with missing UAV point at " + to_string(LidarPointsz.back()._unix_time*10+(nb_ptsz%10)));
            }
            else{
                LidarPointsz.back()._uav_pt = framez_ptr.first->second._uav_point;
                frames1z.insert(make_pair(LidarPointsz.back()._unix_time*10+(nb_ptsz%10), Frame(framez_ptr.first->second)));
                //                uav_x1.push_back((frame_ptr.first->second._uav_points.front()->_longitude+582690.8242)*1e-5);
                //                uav_y1.push_back((frame_ptr.first->second._uav_points.front()->_latitude+4107963.58)*1e-5);
                //                uav_z1.push_back(frame_ptr.first->second._uav_points.front()->_height*100);
            }
            DebugOff("Added lidar point to frame at " << LidarPointsz.back()._unix_time << " : " << LidarPointsz.back()._hour << ":" << LidarPointsz.back()._minutes << ":" << LidarPointsz.back()._seconds << endl);
            //            xpos = floor((lasreader->point.get_x()*grid_step - lasreader->header.min_x*grid_step));
            //            ypos = floor((lasreader->point.get_y()*grid_step - lasreader->header.min_y*grid_step));

            //            z = round((lasreader->point.get_z()*100*grid_step));
            //            xpos -= frame_ptr.first->second._uav_point->_x;
            //            ypos -= frame_ptr.first->second._uav_point->_y;
            //            z -= frame_ptr.first->second._uav_point->_height;
            posz = make_pair(xpos,ypos);
            cellz = make_tuple(1,z,z,z,framez_ptr.first->second._uav_point);
            DebugOff("new z value = " << z << endl);
            DebugOff("fractional z value = " << lasreader->point.get_z() << endl);
            DebugOff("z range = " << grid_maxz[pos] - grid_minz[pos] << endl << endl);
            auto map_pair = gridz2.insert(make_pair(posz,cellz));
            auto map_pair_all = gridz.insert(make_pair(posz,cellz));
            if (!map_pair.second) {/* Cell already exists */
                //                DebugOn("Found overlapping cell after " << nb_pts << endl);
                //                DebugOn("(" << to_string_with_precision(lasreader->point.get_x(),10) <<"," << to_string_with_precision(lasreader->point.get_y(),10) << ","<< to_string_with_precision(lasreader->point.get_z(),10) <<")"<<endl);
                //                                return 0;
                nb_dots = get<0>(map_pair.first->second);
                min_z = get<1>(map_pair.first->second);
                max_z = get<2>(map_pair.first->second);
                av_z = get<3>(map_pair.first->second);
                /* Update cell data */
                get<0>(map_pair.first->second)++;
                if(min_z>z){
                    DebugOff("updating  min z at cell [" << xpos << "," << ypos << "]" << endl);
                    DebugOff("previous min z = " << min_z << endl);
                    DebugOff("new min z = " << z << endl);
                    get<1>(map_pair.first->second) = z;
                }
                if(max_z<z){
                    DebugOff("updating  max z at cell [" << xpos << "," << ypos << "]" << endl);
                    DebugOff("previous max z = " << max_z << endl);
                    DebugOff("new max z = " << z << endl);
                    get<2>(map_pair.first->second) = z;
                }
                av_z *= nb_dots;/* remove previous denominator */
                get<3>(map_pair.first->second) = (av_z + z)/(nb_dots+1); /* Update denominator */
            }
            if (!map_pair_all.second) {/* Cell already exists */
                nb_dots = get<0>(map_pair_all.first->second);
                min_z = get<1>(map_pair_all.first->second);
                max_z = get<2>(map_pair_all.first->second);
                av_z = get<3>(map_pair_all.first->second);
                /* Update cell data */
                get<0>(map_pair_all.first->second)++;
                if(min_z>z){
                    get<1>(map_pair_all.first->second) = z;
                }
                if(max_z<z){
                    get<2>(map_pair_all.first->second) = z;
                }
                av_z *= nb_dots;/* remove previous denominator */
                get<3>(map_pair_all.first->second) = (av_z + z)/(nb_dots+1); /* Update denominator */
            }
            nb_ptsz++;
        }
        DebugOn("Read " << nb_ptsz << " points in file 2 xy plane" << endl);
        DebugOn("2D grid2 has " << gridz2.size() << " cells " << endl);
        DebugOn("xdim2 = " << xdim2 << endl);
        DebugOn("ydim2 = " << ydim2 << endl);

        int cell_counter = 0, max_nb_dots = 0;
        double av_nb_dots = 0, z_range = 0,  max_z_range = 0, av_z_range = 0;
        for (auto &cellz : gridz2) {
            DebugOff("Cell num " << cell_counter++ << " at [" << cell.first.first << "," << cell.first.second << "]" << endl);
            nb_dots = get<0>(cellz.second);
            min_z = get<1>(cellz.second);
            max_z = get<2>(cellz.second);
            av_z = get<3>(cellz.second);
            x_vec2.push_back(cellz.first.first);
            x_shift2.push_back(get<4>(cellz.second)->_x);
            x_shift.push_back(get<4>(cellz.second)->_x);
            x_uav2.add_val(to_string(cellz.first.first)+","+to_string(cellz.first.second),get<4>(cellz.second)->_x);
            x2.add_val(to_string(cellz.first.first)+","+to_string(cellz.first.second), cellz.first.first);
            y_vec2.push_back(cellz.first.second);
            y_shift2.push_back(get<4>(cellz.second)->_y);
            y_shift.push_back(get<4>(cellz.second)->_y);
            y_uav2.add_val(to_string(cellz.first.first)+","+to_string(cellz.first.second),get<4>(cellz.second)->_y);
            y2.add_val(to_string(cellz.first.first)+","+to_string(cellz.first.second), cellz.first.second);
            z_vec2.push_back(av_z);
            zmin_vec2.push_back(min_z);
            zmax_vec2.push_back(max_z);
            z_shift2.push_back(get<4>(cellz.second)->_height);
            z_shift.push_back(get<4>(cellz.second)->_height);
            z_uav2.add_val(to_string(cellz.first.first)+","+to_string(cellz.first.second),get<4>(cellz.second)->_height);
            z2.add_val(to_string(cellz.first.first)+","+to_string(cellz.first.second), max_z);
            uav_roll2.push_back(get<4>(cellz.second)->_roll);
            uav_pitch2.push_back(get<4>(cellz.second)->_pitch);
            uav_yaw2.push_back(get<4>(cellz.second)->_yaw);
            if(max_nb_dots<nb_dots)
                max_nb_dots = nb_dots;
            av_nb_dots += nb_dots;
            z_range = max_z - min_z;
            DebugOff("z range = " << z_range << endl);
            av_z_range += z_range;
            if(z_range > max_z_range)
                max_z_range = z_range;
            DebugOff("z min = " << min_z << endl);
            DebugOff("z max = " << max_z << endl);
            DebugOff("z av = " << av_z << endl);
            DebugOff("nb points = " << nb_dots << endl << endl);
        }
        av_nb_dots /= gridz2.size();
        av_z_range /= gridz2.size();
        DebugOn("2D grid has " << gridz2.size() << " cells " << endl);
        DebugOn("Max z range for a cell in grid2 = " << max_z_range << endl);
        DebugOn("Average z range for a cell in grid2 = " << av_z_range << endl);
        DebugOn("Average points per cell in grid2 = " << av_nb_dots << endl);
        DebugOn("Max points per cell in grid2 = " << max_nb_dots << endl << endl);
        //        LASwriteOpener laswriteopener;
        //        laswriteopener.set_file_name("compressed.laz");
        //        LASwriter* laswriter = laswriteopener.open(&lasreader->header);
        //
        //        while (lasreader->read_point()) laswriter->write_point(&lasreader->point);
        //
        //        laswriter->close();
        //        delete laswriter;
              lasreader->close();
                delete lasreader;
            }

            lasreadopener.set_file_name(LiDAR_file2.c_str());
            lasreadopener.set_populate_header(TRUE);
            if (!lasreadopener.active())
            {
                throw invalid_argument("ERROR: no input specified\n");
            }
            while (lasreadopener.active())
            {
                LASreader* lasreader = lasreadopener.open();
                if (lasreader == 0)
                {
                    throw invalid_argument("ERROR: could not open lasreader for second file\n");
                }
     
                ydim1 = ceil(lasreader->header.max_y*grid_step - lasreader->header.min_y*grid_step);
                zdim1 = ceil(lasreader->header.max_z*grid_step - lasreader->header.min_z*grid_step);
//               ground_x = floor(lasreader->header.min_x);
                auto nb_pts_per_frame = lasreader->npoints/framesx.size();
//                DebugOn("Number of points = " << lasreader->npoints << endl);
//                DebugOn("Number of points per frame = " <<  nb_pts_per_frame << endl);
//                DebugOn("min x axis = " << lasreader->header.min_x << endl);
//                DebugOn("max x axis = " << lasreader->header.max_x << endl);
//                DebugOn("min y axis = " << lasreader->header.min_y << endl);
//                DebugOn("max y axis = " << lasreader->header.max_y << endl);
//                DebugOn("dimension in y axis = " << ydim1 << endl);
//                DebugOn("dimension in z axis = " << zdim1 << endl);
                
                
                int nb_dots; /* Number of measurements inside cell */
                int ypos, zpos;
                double x, min_x, max_x, av_x;
                pair<int,int> posx;
                size_t nb_ptsx = 0;
                tuple<int,double,double,double,UAVPoint*> cellx; /* <nb_dots,min_z,max_z,av_z> */
               auto framesx_it = framesx.begin();
                        while (lasreader->read_point())
                        {
                            if(nb_ptsx==0){
                                DebugOn(to_string_with_precision(10.*(lasreader->point.get_gps_time()+315964800. - 18.),24) << ": (" << to_string_with_precision(lasreader->point.get_x(),10) <<"," << to_string_with_precision(lasreader->point.get_y(),10) << ","<< to_string_with_precision(lasreader->point.get_z(),10) <<")"<<endl);
                //                return 0;
                            }
                            zpos = (round((lasreader->point.get_z()*grid_step)));
                            ypos = (round((lasreader->point.get_y()*grid_step)));
                            if (ypos < 0)
                            {
                                fprintf(stderr, "ERROR: ypos = %d\n", ypos);
                            }
                            

                            //            z = floor((lasreader->point.get_z()*100*grid_step - lasreader->header.min_z*100*grid_step));
                            x = lasreader->point.get_x();
                //            y = lasreader->point.get_y();
        //                    z = lasreader->point.get_z();
                            timestamps.insert(lasreader->point.get_gps_time()+1./(nb_ptsx%10));
                            DebugOff("GPS time = " << to_string_with_precision(lasreader->point.get_gps_time(),24) << endl);
                            LidarPointsx.push_back(LidarPoint(lasreader->point.get_gps_time(),zpos,ypos,x));
                //            LidarPointsx.push_back(LidarPoint(lasreader->point.get_gps_time(),ypos,zpos,x));
                //            LidarPointsy.push_back(LidarPoint(lasreader->point.get_gps_time(),xpos,zpos,y));
                            if(nb_ptsx>=nb_pts_per_frame && nb_ptsx%nb_pts_per_frame==0 && next(framesx_it)!=framesx.end()){
                                framesx_it++;
                            }
                            auto framex_ptr = framesx.insert(make_pair(framesx_it->first, Frame(LidarPointsx.back()._unix_time*10+(nb_ptsx%10))));
                            framex_ptr.first->second.add_lidar_point(LidarPointsx.back());
                            if(framex_ptr.second){
                                throw invalid_argument("Frame with missing UAV point at " + to_string(LidarPointsx.back()._unix_time*10+(nb_ptsx%10)));
                            }
                            else{
                                LidarPointsx.back()._uav_pt = framex_ptr.first->second._uav_point;
                                frames1x.insert(make_pair(LidarPointsx.back()._unix_time*10+(nb_ptsx%10), Frame(framex_ptr.first->second)));
                //                uav_x1.push_back((frame_ptr.first->second._uav_points.front()->_longitude+582690.8242)*1e-5);
                //                uav_y1.push_back((frame_ptr.first->second._uav_points.front()->_latitude+4107963.58)*1e-5);
                //                uav_z1.push_back(frame_ptr.first->second._uav_points.front()->_height*100);
                            }
                            DebugOff("Added lidar point to frame at " << LidarPointsx.back()._unix_time << " : " << LidarPointsx.back()._hour << ":" << LidarPointsx.back()._minutes << ":" << LidarPointsx.back()._seconds << endl);
                            //            xpos = floor((lasreader->point.get_x()*grid_step - lasreader->header.min_x*grid_step));
                            //            ypos = floor((lasreader->point.get_y()*grid_step - lasreader->header.min_y*grid_step));
                           
                            //            z = round((lasreader->point.get_z()*100*grid_step));
                //            xpos -= frame_ptr.first->second._uav_point->_x;
                //            ypos -= frame_ptr.first->second._uav_point->_y;
                //            z -= frame_ptr.first->second._uav_point->_height;
                            posx = make_pair(ypos,zpos);
                            cellx = make_tuple(1,x,x,x,framex_ptr.first->second._uav_point);
                        
                            auto map_pair = gridx2.insert(make_pair(posx,cellx));
                            auto map_pair_all = gridx.insert(make_pair(posx,cellx));
                            if (!map_pair.second) {/* Cell already exists */
                //                DebugOn("Found overlapping cell after " << nb_pts << endl);
                //                DebugOn("(" << to_string_with_precision(lasreader->point.get_x(),10) <<"," << to_string_with_precision(lasreader->point.get_y(),10) << ","<< to_string_with_precision(lasreader->point.get_z(),10) <<")"<<endl);
                //                                return 0;
                                nb_dots = get<0>(map_pair.first->second);
                                min_x = get<1>(map_pair.first->second);
                                max_x = get<2>(map_pair.first->second);
                                av_x = get<3>(map_pair.first->second);
                                /* Update cell data */
                                get<0>(map_pair.first->second)++;
                                if(min_x>x){
                                    get<1>(map_pair.first->second) = x;
                                }
                                if(max_x<x){
                                    get<2>(map_pair.first->second) = x;
                                }
                                av_x *= nb_dots;/* remove previous denominator */
                                get<3>(map_pair.first->second) = (av_x + x)/(nb_dots+1); /* Update denominator */
                            }
                            if (!map_pair_all.second) {/* Cell already exists */
                                nb_dots = get<0>(map_pair_all.first->second);
                                min_x = get<1>(map_pair_all.first->second);
                                max_x = get<2>(map_pair_all.first->second);
                                av_x = get<3>(map_pair_all.first->second);
                                /* Update cell data */
                                get<0>(map_pair_all.first->second)++;
                                if(min_x>x){
                                    get<1>(map_pair_all.first->second) = x;
                                }
                                if(max_x<x){
                                    get<2>(map_pair_all.first->second) = x;
                                }
                                av_x *= nb_dots;/* remove previous denominator */
                                get<3>(map_pair_all.first->second) = (av_x + x)/(nb_dots+1); /* Update denominator */
                            }
                            nb_ptsx++;
                        }
                        DebugOn("Read fixed yz" << nb_ptsx << " points" << endl);
                        DebugOn("2D grid has " << gridx2.size() << " cells " << endl);
                        DebugOn("zdim = " << zdim1 << endl);
                        DebugOn("ydim = " << ydim1 << endl);
                        
                        int cell_counterx = 0, max_nb_dotsx = 0;
                        double av_nb_dotsx = 0, max_x_range = 0, x_range = 0,  av_x_range = 0;
                        for (auto &cellx : gridx1) {
                          
                            nb_dots = get<0>(cellx.second);
                            min_x = get<1>(cellx.second);
                            max_x = get<2>(cellx.second);
                            av_x = get<3>(cellx.second);
                            y_vec2.push_back(cellx.first.first);
                            y_shift2.push_back(get<4>(cellx.second)->_y);
                            y_shift.push_back(get<4>(cellx.second)->_y);
                            y_uav2.add_val(to_string(cellx.first.first)+","+to_string(cellx.first.second),get<4>(cellx.second)->_y);
                            y2.add_val(to_string(cellx.first.first)+","+to_string(cellx.first.second), cellx.first.first);
                            z_vec2.push_back(cellx.first.second);
                            z_shift2.push_back(get<4>(cellx.second)->_height);
                            z_shift.push_back(get<4>(cellx.second)->_height);
                            z_uav2.add_val(to_string(cellx.first.first)+","+to_string(cellx.first.second),get<4>(cellx.second)->_height);
                            z2.add_val(to_string(cellx.first.first)+","+to_string(cellx.first.second), cellx.first.second);
                            x_vec2.push_back(av_x);
                            xmin_vec2.push_back(min_x);
                            xmax_vec2.push_back(max_x);
                            x_shift2.push_back(get<4>(cellx.second)->_x);
                            x_shift.push_back(get<4>(cellx.second)->_x);
                            x_uav2.add_val(to_string(cellx.first.first)+","+to_string(cellx.first.second),get<4>(cellx.second)->_x);
                            x2.add_val(to_string(cellx.first.first)+","+to_string(cellx.first.second), max_x);
                            uav_roll2.push_back(get<4>(cellx.second)->_roll);
                            uav_pitch2.push_back(get<4>(cellx.second)->_pitch);
                            uav_yaw2.push_back(get<4>(cellx.second)->_yaw);
                            if(max_nb_dotsx<nb_dots)
                                max_nb_dotsx = nb_dots;
                            av_nb_dotsx += nb_dots;
                            x_range = max_x - min_x;
                            DebugOff("x range = " << x_range << endl);
                            av_x_range += x_range;
                            if(x_range > max_x_range)
                                max_x_range = x_range;
                            DebugOff("x min = " << min_x << endl);
                            DebugOff("x max = " << max_x << endl);
                            DebugOff("x av = " << av_x << endl);
                            DebugOff("nb points = " << nb_dots << endl << endl);
                        }
                        av_nb_dotsx /= gridx1.size();
                        av_x_range /= gridx1.size();
                        DebugOn("Max x range for a cell = " << max_x_range << endl);
                        DebugOn("Average x range for a cell = " << av_x_range << endl);
                        DebugOn("Average points per cell = " << av_nb_dotsx << endl);
                        DebugOn("Max points per cell = " << max_nb_dotsx << endl << endl);
        lasreader->close();
        delete lasreader;
    }
    int cell_counterz = 0, max_nb_dotsz = 0, nb_dotsz = 0;
    double av_nb_dotsz = 0, av_z_range = 0, max_z_range = 0, z_range = 0, min_z = 0, max_z = 0, av_z = 0;
    indices cellsz("cellsz");
    int nb_overlapz = 0;
    bool insert = true;
    for (auto &cellz : gridz) {
        DebugOff("Cell num " << cell_counterz++ << " at [" << cellz.first.first << "," << cellz.first.second << "]" << endl);
        if(gridz1.count(cellz.first)>0 && gridz2.count(cellz.first)){
            nb_overlapz++;
            if(nb_overlapz%10==1){
            cellsz.insert(to_string(cellz.first.first) + "," + to_string(cellz.first.second));
            }
            insert = !insert;
            nb_dotsz = get<0>(cellz.second);
            min_z = get<1>(cellz.second);
            max_z = get<2>(cellz.second);
            av_z = get<3>(cellz.second);
            if(max_nb_dotsz<nb_dotsz)
                max_nb_dotsz = nb_dotsz;
            av_nb_dotsz += nb_dotsz;
            z_range = max_z - min_z;
            DebugOff("z range = " << z_range << endl);
            av_z_range += z_range;
            if(z_range > max_z_range)
                max_z_range = z_range;
            DebugOff("z min = " << min_z << endl);
            DebugOff("z max = " << max_z << endl);
            DebugOff("z av = " << av_z << endl);
            DebugOff("nb points = " << nb_dotsz << endl << endl);
        }
    }
    av_nb_dotsz /= nb_overlapz;
    av_z_range /= nb_overlapz;
    DebugOn("Number of overlapping points in plane xy = " << nb_overlapz << endl);
    DebugOn("Max z range for a cell in combined grid = " << max_z_range << endl);
    DebugOn("Average z range for a cell in combined grid = " << av_z_range << endl);
    DebugOn("Average points per cell in combined grid = " << av_nb_dotsz << endl);
    DebugOn("Max points per cell in combined grid = " << max_nb_dotsz << endl << endl);
        int max_nb_pointsz = 0, av_nb_pointsz = 0, nb_empty_framesz = 0;
        for (const auto &frame: framesz) {
            int nb_ptsz = frame.second._lidar_points->size();
            if(nb_ptsz==0)
                nb_empty_framesz++;
            else{
                max_nb_pointsz = std::max(max_nb_pointsz, nb_ptsz);
                av_nb_pointsz += nb_ptsz;
            }
        }
        av_nb_pointsz/=(framesz.size()-nb_empty_framesz);
        DebugOn("Number of empty frames in xy plane = " << nb_empty_framesz << endl);
        DebugOn("Average number of points per frame = " << av_nb_pointsz << endl);
        DebugOn("Max number of points per frame = " << max_nb_pointsz << endl);
        DebugOn("Number of timestamps = " << timestamps.size() << endl);

        //        LASwriteOpener laswriteopener;
        //        laswriteopener.set_file_name("compressed.laz");
        //        LASwriter* laswriter = laswriteopener.open(&lasreader->header);
        //
        //        while (lasreader->read_point()) laswriter->write_point(&lasreader->point);
        //
        //        laswriter->close();
        //        delete laswriter;

//        lasreader->close();
//        delete lasreader;
//    }
    for (const auto &frame: frames1z) {
        uav_x1.push_back((frame.second._uav_point->_x));
        uav_y1.push_back((frame.second._uav_point->_y));
        uav_z1.push_back(frame.second._uav_point->_height);
    }

        int cell_counterx = 0, max_nb_dotsx = 0, nb_dotsx = 0;
        double av_nb_dotsx = 0, av_x_range = 0, max_x_range = 0, x_range = 0, min_x = 0, max_x = 0, av_x = 0;
        indices cellsx("cellsx");
        int nb_overlapx = 0;
        bool insertx = true;
        for (auto &cellx : gridx) {
            DebugOff("Cell num " << cell_counterx++ << " at [" << cellx.first.first << "," << cellx.first.second << "]" << endl);
            if(gridx1.count(cellx.first)>0 && gridx2.count(cellx.first)){
               nb_overlapx++;
                if(nb_overlapx%10==1){
                cellsx.insert(to_string(cellx.first.first) + "," + to_string(cellx.first.second));
                }
                insertx = !insertx;
                nb_dotsx = get<0>(cellx.second);
                min_x = get<1>(cellx.second);
                max_x = get<2>(cellx.second);
                av_x = get<3>(cellx.second);
                if(max_nb_dotsx<nb_dotsx)
                    max_nb_dotsx = nb_dotsx;
                av_nb_dotsx += nb_dotsx;
                x_range = max_x - min_x;
                DebugOff("x range = " << x_range << endl);
                av_x_range += x_range;
                if(x_range > max_x_range)
                    max_x_range = x_range;
                DebugOff("x min = " << min_x << endl);
                DebugOff("x max = " << max_x << endl);
                DebugOff("x av = " << av_x << endl);
                DebugOff("nb points = " << nb_dotsx << endl << endl);
            }
        }
          av_nb_dotsx /= nb_overlapx;
          av_x_range /= nb_overlapx;
        DebugOn("Number of overlapping points in plane yz = " << nb_overlapx << endl);
        DebugOn("Max x range for a cell in combined grid = " << max_x_range << endl);
        DebugOn("Average x range for a cell in combined grid = " << av_x_range << endl);
        DebugOn("Average points per cell in combined grid = " << av_nb_dotsx << endl);
        DebugOn("Max points per cell in combined grid = " << max_nb_dotsx << endl << endl);
            int max_nb_pointsx = 0, av_nb_pointsx = 0, nb_empty_framesx = 0;
            for (const auto &frame: framesx) {
                int nb_ptsx = frame.second._lidar_points->size();
                if(nb_ptsx==0)
                    nb_empty_framesx++;
                else{
                    max_nb_pointsx = std::max(max_nb_pointsx, nb_ptsx);
                    av_nb_pointsx += nb_ptsx;
                }
            }
            av_nb_pointsx/=(framesx.size()-nb_empty_framesx);
            DebugOn("Number of empty frames in xy plane = " << nb_empty_framesx << endl);
            DebugOn("Average number of points per frame = " << av_nb_pointsx << endl);
            DebugOn("Max number of points per frame = " << max_nb_pointsx << endl);
            DebugOn("Number of timestamps = " << timestamps.size() << endl);
    //
    //        //        LASwriteOpener laswriteopener;
    //        //        laswriteopener.set_file_name("compressed.laz");
    //        //        LASwriter* laswriter = laswriteopener.open(&lasreader->header);
    //        //
    //        //        while (lasreader->read_point()) laswriter->write_point(&lasreader->point);
    //        //
    //        //        laswriter->close();
    //        //        delete laswriter;
    //
    //        lasreader->close();
    //        delete lasreader;
    //    }
        for (const auto &frame: frames1x) {
            uav_x1.push_back((frame.second._uav_point->_x));
            uav_y1.push_back((frame.second._uav_point->_y));
            uav_z1.push_back(frame.second._uav_point->_height);
        }



    using namespace gravity;
    double roll_1 = 0, pitch_1 = 0, yaw_1 = 0;
    double roll_2 = 0, pitch_2 = 0, yaw_2 = 0;

    bool solve_lidar = true;
    if (solve_lidar) {
        Model<> M("Lidar");
        var<> new_x1("new_x1"), new_y1("new_y1"), new_z1("new_z1");
        var<> new_x2("new_x2"), new_y2("new_y2"), new_z2("new_z2");
        var<> x_diff("x_diff", pos_), y_diff("y_diff", pos_), z_diff("z_diff", pos_);
        var<> yaw1("yaw1", -0.1, 0.1), pitch1("pitch1", -0.1, 0.1), roll1("roll1", -0.1, 0.1);
        var<> yaw2("yaw2", -0.1, 0.1), pitch2("pitch2", -0.1, 0.1), roll2("roll2", -0.1, 0.1);

    //    var<> sin_yaw1("yaw", -1, 1), sin_pitch1("pitch", -1, 1), sin_roll1("roll", -1, 1);
    //    var<> sin_yaw2("yaw", -1, 1), sin_pitch2("pitch", -0.1, 0.1), roll2("roll", -0.1, 0.1);
        
    //
    //    var<> yaw1("yaw1", 0, 0), pitch1("pitch1", 0, 0), roll1("roll1", 0, 0);
    //    var<> yaw2("yaw2", 0, 0), pitch2("pitch2", 0, 0), roll2("roll2", 0, 0);

    //    var<> abs_yaw1("abs_yaw1", 0, 0.1), abs_pitch1("abs_pitch1", 0, 0.1), abs_roll1("abs_roll1", 0, 0.1);
    //    var<> abs_yaw2("abs_yaw2", 0, 0.2), abs_pitch2("abs_pitch2", 0, 0.2), abs_roll2("abs_roll2", 0, 0.2);
    //    var<> yaw1("yaw1"), pitch1("pitch1"), roll1("roll1");
    //    var<> yaw2("yaw2"), pitch2("pitch2"), roll2("roll2");
        M.add(yaw1.in(R(1)),pitch1.in(R(1)),roll1.in(R(1)));
        M.add(yaw2.in(R(1)),pitch2.in(R(1)),roll2.in(R(1)));
    //    M.add(abs_yaw1.in(R(1)),abs_pitch1.in(R(1)),abs_roll1.in(R(1)));
    //    M.add(abs_yaw2.in(R(1)),abs_pitch2.in(R(1)),abs_roll2.in(R(1)));
    //    yaw1 = 0.1; yaw2 = -0.1; pitch1 = 0.1; pitch2 = 0.1; roll1 = 0.1; roll2 = -0.1;
    //    yaw1 = -0.01; roll1 = 0.001; pitch1 = 0.001;
    //    pitch1 = 0.017;
//        M.add(new_x1.in(cellsz), new_y1.in(cellsz), new_z1.in(cellsz));
//        M.add(new_x2.in(cellsz), new_y2.in(cellsz), new_z2.in(cellsz));
        M.add(new_x1.in(cellsx), new_y1.in(cellsx), new_z1.in(cellsx));
        M.add(new_x2.in(cellsx), new_y2.in(cellsx), new_z2.in(cellsx));
    //    M.add(x_diff.in(cells), y_diff.in(cells), z_diff.in(cells));
//        M.add(x_diff.in(cellsz), y_diff.in(cellsz));
//        M.add(z_diff.in(cellsz));
        M.add(x_diff.in(cellsx), y_diff.in(cellsx));
        M.add(z_diff.in(cellsx));

        Constraint<> Equal_pitch("Equal_pitch");
        Equal_pitch += pitch1 + pitch2;
        M.add(Equal_pitch==0);
        
        Constraint<> Equal_roll("Equal_roll");
        Equal_roll += roll1 + roll2;
        M.add(Equal_roll==0);
        
    //    Constraint<> x_norm("x_norm");
    //    x_norm += x_diff - pow((new_x1 - new_x2),2);
    //    M.add(x_norm.in(cells)>=0);
    //
    //    Constraint<> y_norm("y_norm");
    //    y_norm += y_diff - pow((new_y1 - new_y2),2);
    //    M.add(y_norm.in(cells)>=0);
    //
    //    Constraint<> z_norm("z_norm");
    //    z_norm += z_diff - pow((new_z1 - new_z2),2);
    //    M.add(z_norm.in(cells)>=0);
        
    //    Constraint<> yaw_abs1("yaw_abs1");
    //    yaw_abs1 += abs_yaw1 - yaw2;
    //    M.add(yaw_abs1 >= 0);
    //
    //    Constraint<> yaw_abs2("yaw_abs2");
    //    yaw_abs2 += abs_yaw1 + yaw2;
    //    M.add(yaw_abs2 >= 0);
    //
    //
    //    Constraint<> roll_abs1("roll_abs1");
    //    roll_abs1 += abs_roll1 - roll2;
    //    M.add(roll_abs1 >= 0);
    //
    //    Constraint<> roll_abs2("roll_abs2");
    //    roll_abs2 += abs_roll1 + roll2;
    //    M.add(roll_abs2 >= 0);
    //
    //    Constraint<> pitch_abs1("pitch_abs1");
    //    pitch_abs1 += abs_pitch1 - pitch2;
    //    M.add(pitch_abs1 >= 0);
    //
    //    Constraint<> pitch_abs2("pitch_abs2");
    //    pitch_abs2 += abs_pitch1 + pitch2;
    //    M.add(pitch_abs2 >= 0);
        
//
        Constraint<> x_abs1("x_abs1");
        x_abs1 += x_diff - (new_x1 - new_x2);
        M.add(x_abs1.in(cellsx)>=0);

        Constraint<> x_abs2("x_abs2");
        x_abs2 += x_diff - (new_x2 - new_x1);
        M.add(x_abs2.in(cellsx)>=0);


//        Constraint<> y_abs1("y_abs1");
//        y_abs1 += y_diff - (new_y1 - new_y2);
//        M.add(y_abs1.in(cellsx)>=0);
//
//       Constraint<> y_abs2("y_abs2");
//        y_abs2 += y_diff - (new_y2 - new_y1);
//        M.add(y_abs2.in(cellsx)>=0);
//
//        Constraint<> z_abs1("z_abs1");
//        z_abs1 += z_diff - (new_z1 - new_z2);
//        M.add(z_abs1.in(cellsz)>=0);
//
//        Constraint<> z_abs2("z_abs2");
//        z_abs2 += z_diff - (new_z2 - new_z1);
//        M.add(z_abs2.in(cellsz)>=0);

        auto ids = yaw1.repeat_id(cellsx.size());
        
        /* alpha = yaw_, beta = pitch_ and gamma = roll_ */
        Constraint<> x_rot1("x_rot1");
        x_rot1 += new_x1 - x_uav1.in(cellsx);
        x_rot1 -= (x1.in(cellsx)-x_uav1.in(cellsx))*cos(yaw1.in(ids))*cos(pitch1.in(ids)) + (y1.in(cellsx)-y_uav1.in(cellsx))*(cos(yaw1.in(ids))*sin(pitch1.in(ids))*sin(roll1.in(ids)) - sin(yaw1.in(ids))*cos(roll1.in(ids))) + (z1.in(cellsx)-z_uav1.in(cellsx))*(cos(yaw1.in(ids))*sin(pitch1.in(ids))*cos(roll1.in(ids)) + sin(yaw1.in(ids))*sin(roll1.in(ids)));
        M.add(x_rot1.in(cellsx)==0);

    //    Constraint<> x_rot2("x_rot2");
    //    x_rot2 += new_x2 + x_uav2.in(cells);
    //    x_rot2 -= (x2.in(cells)-x_uav2.in(cells))*cos(yaw1.in(ids))*cos(pitch1.in(ids)) + (y2.in(cells)-y_uav2.in(cells))*(cos(yaw1.in(ids))*sin(pitch1.in(ids))*sin(roll1.in(ids)) - sin(yaw1.in(ids))*cos(roll1.in(ids))) + (z2.in(cells)-z_uav2.in(cells))*(cos(yaw1.in(ids))*sin(pitch1.in(ids))*cos(roll1.in(ids)) + sin(yaw1.in(ids))*sin(roll1.in(ids)));
    //    M.add(x_rot2.in(cells)==0);
        
//        Constraint<> x_rot2("x_rot2");
//        x_rot2 += new_x2 - x_uav2.in(cellsz);
//        x_rot2 -= (x2.in(cellsz)-x_uav2.in(cellsz))*cos(yaw2.in(ids))*cos(pitch2.in(ids)) + (y2.in(cellsz)-y_uav2.in(cellsz))*(cos(yaw2.in(ids))*sin(pitch2.in(ids))*sin(roll2.in(ids)) - sin(yaw2.in(ids))*cos(roll2.in(ids))) + (z2.in(cellsz)-z_uav2.in(cellsz))*(cos(yaw2.in(ids))*sin(pitch2.in(ids))*cos(roll2.in(ids)) + sin(yaw2.in(ids))*sin(roll2.in(ids)));
//        M.add(x_rot2.in(cellsz)==0);
        Constraint<> x_rot2("x_rot2");
               x_rot2 += new_x2 - x_uav2.in(cellsx);
               x_rot2 -= (x2.in(cellsx)-x_uav2.in(cellsx))*cos(yaw2.in(ids))*cos(pitch2.in(ids)) + (y2.in(cellsx)-y_uav2.in(cellsx))*(cos(yaw2.in(ids))*sin(pitch2.in(ids))*sin(roll2.in(ids)) - sin(yaw2.in(ids))*cos(roll2.in(ids))) + (z2.in(cellsx)-z_uav2.in(cellsx))*(cos(yaw2.in(ids))*sin(pitch2.in(ids))*cos(roll2.in(ids)) + sin(yaw2.in(ids))*sin(roll2.in(ids)));
               M.add(x_rot2.in(cellsx)==0);
        
        Constraint<> y_rot1("y_rot1");
        y_rot1 += new_y1 - y_uav1.in(cellsx);
        y_rot1 -= (x1.in(cellsx)-x_uav1.in(cellsx))*sin(yaw1.in(ids))*cos(pitch1.in(ids)) + (y1.in(cellsx)-y_uav1.in(cellsx))*(sin(yaw1.in(ids))*sin(pitch1.in(ids))*sin(roll1.in(ids)) + cos(yaw1.in(ids))*cos(roll1.in(ids))) + (z1.in(cellsx)-z_uav1.in(cellsx))*(sin(yaw1.in(ids))*sin(pitch1.in(ids))*cos(roll1.in(ids)) - cos(yaw1.in(ids))*sin(roll1.in(ids)));
        M.add(y_rot1.in(cellsx)==0);
        
//        Constraint<> y_rot1("y_rot1");
//           y_rot1 += new_y1 - y_uav1.in(cellsz);
//           y_rot1 -= (x1.in(cellsz)-x_uav1.in(cellsz))*sin(yaw1.in(ids))*cos(pitch1.in(ids)) + (y1.in(cellsz)-y_uav1.in(cellsz))*(sin(yaw1.in(ids))*sin(pitch1.in(ids))*sin(roll1.in(ids)) + cos(yaw1.in(ids))*cos(roll1.in(ids))) + (z1.in(cellsz)-z_uav1.in(cellsz))*(sin(yaw1.in(ids))*sin(pitch1.in(ids))*cos(roll1.in(ids)) - cos(yaw1.in(ids))*sin(roll1.in(ids)));
//           M.add(y_rot1.in(cellsz)==0);
        
//        Constraint<> y_rot2("y_rot2");
//        y_rot2 += new_y2 - y_uav2.in(cellsz);
//        y_rot2 -= (x2.in(cellsz)-x_uav2.in(cellsz))*sin(yaw2.in(ids))*cos(pitch2.in(ids)) + (y2.in(cellsz)-y_uav2.in(cellsz))*(sin(yaw2.in(ids))*sin(pitch2.in(ids))*sin(roll2.in(ids)) + cos(yaw2.in(ids))*cos(roll2.in(ids))) + (z2.in(cellsz)-z_uav2.in(cellsz))*(sin(yaw2.in(ids))*sin(pitch2.in(ids))*cos(roll2.in(ids)) - cos(yaw2.in(ids))*sin(roll2.in(ids)));
//        M.add(y_rot2.in(cellsz)==0);
        
        Constraint<> y_rot2("y_rot2");
               y_rot2 += new_y2 - y_uav2.in(cellsx);
               y_rot2 -= (x2.in(cellsx)-x_uav2.in(cellsx))*sin(yaw2.in(ids))*cos(pitch2.in(ids)) + (y2.in(cellsx)-y_uav2.in(cellsx))*(sin(yaw2.in(ids))*sin(pitch2.in(ids))*sin(roll2.in(ids)) + cos(yaw2.in(ids))*cos(roll2.in(ids))) + (z2.in(cellsx)-z_uav2.in(cellsx))*(sin(yaw2.in(ids))*sin(pitch2.in(ids))*cos(roll2.in(ids)) - cos(yaw2.in(ids))*sin(roll2.in(ids)));
               M.add(y_rot2.in(cellsx)==0);
    //    Constraint<> y_rot2("y_rot2");
    //    y_rot2 += new_y2 + y_uav2.in(cells);
    //    y_rot2 -= (x2.in(cells)-x_uav2.in(cells))*sin(yaw1.in(ids))*cos(pitch1.in(ids)) + (y2.in(cells)-y_uav2.in(cells))*(sin(yaw1.in(ids))*sin(pitch1.in(ids))*sin(roll1.in(ids)) + cos(yaw1.in(ids))*cos(roll1.in(ids))) + (z2.in(cells)-z_uav2.in(cells))*(sin(yaw1.in(ids))*sin(pitch1.in(ids))*cos(roll1.in(ids)) - cos(yaw1.in(ids))*sin(roll1.in(ids)));
    //    M.add(y_rot2.in(cells)==0);
        
        
//        Constraint<> z_rot1("z_rot1");
//        z_rot1 += new_z1 - z_uav1.in(cellsz);
//        z_rot1 -= (x1.in(cellsz)-x_uav1.in(cellsz))*sin(-1*pitch1.in(ids)) + (y1.in(cellsz)-y_uav1.in(cellsz))*(cos(pitch1.in(ids))*sin(roll1.in(ids))) + (z1.in(cellsz)-z_uav1.in(cellsz))*(cos(pitch1.in(ids))*cos(roll1.in(ids)));
//        M.add(z_rot1.in(cellsz)==0);
        Constraint<> z_rot1("z_rot1");
            z_rot1 += new_z1 - z_uav1.in(cellsx);
            z_rot1 -= (x1.in(cellsx)-x_uav1.in(cellsx))*sin(-1*pitch1.in(ids)) + (y1.in(cellsx)-y_uav1.in(cellsx))*(cos(pitch1.in(ids))*sin(roll1.in(ids))) + (z1.in(cellsx)-z_uav1.in(cellsx))*(cos(pitch1.in(ids))*cos(roll1.in(ids)));
            M.add(z_rot1.in(cellsx)==0);
        
        Constraint<> z_rot2("z_rot2");
        z_rot2 += new_z2 - z_uav2.in(cellsx);
        z_rot2 -= (x2.in(cellsx)-x_uav2.in(cellsx))*sin(-1*pitch2.in(ids)) + (y2.in(cellsx)-y_uav2.in(cellsx))*(cos(pitch2.in(ids))*sin(roll2.in(ids))) + (z2.in(cellsx)-z_uav2.in(cellsx))*(cos(pitch2.in(ids))*cos(roll2.in(ids)));
        M.add(z_rot2.in(cellsx)==0);
//        Constraint<> z_rot2("z_rot2");
//         z_rot2 += new_z2 - z_uav2.in(cellsz);
//         z_rot2 -= (x2.in(cellsz)-x_uav2.in(cellsz))*sin(-1*pitch2.in(ids)) + (y2.in(cellsz)-y_uav2.in(cellsz))*(cos(pitch2.in(ids))*sin(roll2.in(ids))) + (z2.in(cellsz)-z_uav2.in(cellsz))*(cos(pitch2.in(ids))*cos(roll2.in(ids)));
//         M.add(z_rot2.in(cellsz)==0);
    //    Constraint<> z_rot2("z_rot2");
    //    z_rot2 += new_z2 + z_uav2.in(cells);
    //    z_rot2 -= (x2.in(cells)-x_uav2.in(cells))*sin(-1*pitch1.in(ids)) + (y2.in(cells)-y_uav2.in(cells))*(cos(pitch1.in(ids))*sin(roll1.in(ids))) + (z2.in(cells)-z_uav2.in(cells))*(cos(pitch1.in(ids))*cos(roll1.in(ids)));
    //    M.add(z_rot2.in(cells)==0);
        
//        M.min(sum(z_diff)/nb_overlapz);
        
        M.min(sum(x_diff));
//        M.min(sum(x_diff +y_diff + z_diff));
        
    //    M.print();
        
        solver<> S(M,ipopt);
        S.run();
        
        
        for (int i = 0; i<x_rot1.get_nb_instances(); i++) {
            pre_x.add_val(x_rot1.eval(i));
            pre_y.add_val(y_rot1.eval(i));
            pre_z.add_val(z_rot1.eval(i));
            x_uav.add_val(x_uav1.eval(i));
            y_uav.add_val(y_uav1.eval(i));
            z_uav.add_val(z_uav1.eval(i));
        }
        for (int i = 0; i<x_rot2.get_nb_instances(); i++) {
            pre_x.add_val(x_rot2.eval(i));
            pre_y.add_val(y_rot2.eval(i));
            pre_z.add_val(z_rot2.eval(i));
            x_uav.add_val(x_uav2.eval(i));
            y_uav.add_val(y_uav2.eval(i));
            z_uav.add_val(z_uav2.eval(i));
        }
//    M.print_solution();
    
        pitch_1 = pitch1.eval();
        roll_1 = roll1.eval();
        yaw_1 = yaw1.eval();
    //    pitch_2 = pitch1.eval();
    //    roll_2 = roll1.eval();
    //    yaw_2 = yaw1.eval();
        pitch_2 = pitch2.eval();
        roll_2 = roll2.eval();
        yaw_2 = yaw2.eval();
    }
    DebugOn("Pitch1 = " << pitch_1*180/pi << endl);
    DebugOn("Roll1 = " << roll_1*180/pi << endl);
    DebugOn("Yaw1 = " << yaw_1*180/pi << endl);
    DebugOn("Pitch2 = " << pitch_2*180/pi << endl);
    DebugOn("Roll2 = " << roll_2*180/pi << endl);
    DebugOn("Yaw2 = " << yaw_2*180/pi << endl);
    //    roll_ = -0.55;
    //    pitch_ = 0.01375;
    //    pitch_ = 1.375*pi/180.;
    //    roll_ = -0.55*pi/180.;
    //    roll_ = -0.9*pi/180.;
    //    roll_ = 1.375*pi/180.;
    //    pitch_ = 0.9*pi/180.;
    //    yaw_ = -0.55*pi/180.;
    //    yaw_ = 0.9*pi/180.;
    //    yaw_ = -0.55;
    double alpha = 0, beta = 0, gamma = 0; /* alpha = yaw_, beta = pitch_ and gamma = roll_ */
    //    alpha = -0.55;
    //    beta = 0.9;
    //    gamma = 1.375;
    //    beta = 0.02;
    //    gamma = 0.01375;
    //    beta = 0.01;
    //    gamma = 1.375;
    //    alpha = -30.*pi/180.;
    //    beta = 30.*pi/180.;
    //    gamma = 30.*pi/180.;
    auto tot_pts = x_vec1.size()+x_vec2.size();
    x_combined.resize(tot_pts);
//    x_combined.resize(x_vec1.size());
    y_combined.resize(tot_pts);
    z_combined.resize(tot_pts);
    zmin_combined.resize(tot_pts);
    zmax_combined.resize(tot_pts);
    double shifted_x, shifted_y, shifted_z;
    gamma = roll_1;
    beta = pitch_1;
    alpha = yaw_1;
    for (auto i = 0; i< x_vec1.size(); i++) {
        //        gamma = roll_ + uav_roll1[i];
        //        beta = pitch_ + uav_pitch1[i];
        //        alpha = yaw_ + uav_yaw_1[i];
        shifted_x = x_vec1[i] - x_shift1[i];
        shifted_y = y_vec1[i] - y_shift1[i];
        shifted_z = zmax_vec1[i] - z_shift1[i];
        x_combined[i] = shifted_x*cos(alpha)*cos(beta) + shifted_y*(cos(alpha)*sin(beta)*sin(gamma) - sin(alpha)*cos(gamma)) + shifted_z*(cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma));
        y_combined[i] = shifted_x*sin(alpha)*cos(beta) + shifted_y*(sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) + shifted_z*(sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma));
        zmax_combined[i] = shifted_x*(-sin(beta)) + shifted_y*(cos(beta)*sin(gamma)) + shifted_z*(cos(beta)*cos(gamma));
        x_combined[i] += x_shift1[i];
        y_combined[i] += y_shift1[i];
        zmax_combined[i] += z_shift1[i];
        
        //        x_combined[i] = x_vec1[i]*cos(alpha)*cos(beta) + y_vec1[i]*(cos(alpha)*sin(beta)*sin(gamma) - sin(alpha)*cos(gamma)) + zmax_vec1[i]*(cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma));
        //        y_combined[i] = x_vec1[i]*sin(alpha)*cos(beta) + y_vec1[i]*(sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) + zmax_vec1[i]*(sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma));
        //        zmax_combined[i] = x_vec1[i]*(-sin(beta)) + y_vec1[i]*(cos(beta)*sin(gamma)) + zmax_vec1[i]*(cos(beta)*cos(gamma));
        //        x_combined[i] = x_vec1[i]*cos(alpha)*cos(beta) + y_vec1[i]*sin(alpha)*cos(beta) + zmax_vec1[i]*(-sin(beta));
        //        y_combined[i] = x_vec1[i]*(cos(alpha)*sin(beta)*sin(gamma) - sin(alpha)*cos(gamma)) + y_vec1[i]*(sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) + zmax_vec1[i]*(cos(beta)*sin(gamma));
        //        zmax_combined[i] = x_vec1[i]*(cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma)) + y_vec1[i]*(sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma)) + zmax_vec1[i]*(cos(beta)*cos(gamma));
        
        //        x_combined[i] = x_vec1[i];
        //        y_combined[i] = y_vec1[i];
        //        z_combined[i] = x_vec1[i]*(-sin(beta)) + y_vec1[i]*(cos(beta)*sin(gamma)) + z_vec1[i]*(cos(beta)*cos(gamma));
        //        zmin_combined[i] = x_vec1[i]*(-sin(beta)) + y_vec1[i]*(cos(beta)*sin(gamma)) + zmin_vec1[i]*(cos(beta)*cos(gamma));
        
    }
    /* Moving to second flight line assumed to be in opposite direction */
//    roll_2 *= -1;
////    pitch_ *= -1;
//    yaw_2 *= -1;
    gamma = roll_2;
    beta = pitch_2;
    alpha = yaw_2;
        for (auto i = 0; i< x_vec2.size(); i++) {
    //        gamma = roll_ + uav_roll2[i];
    //        beta = pitch_ + uav_pitch2[i];
    //        alpha = yaw_ + uav_yaw2[i];
            shifted_x = x_vec2[i] - x_shift2[i];
            shifted_y = y_vec2[i] - y_shift2[i];
            shifted_z = zmax_vec2[i] - z_shift2[i];
            x_combined[x_vec1.size()+i] = shifted_x*cos(alpha)*cos(beta) + shifted_y*(cos(alpha)*sin(beta)*sin(gamma) - sin(alpha)*cos(gamma)) + shifted_z*(cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma));
            y_combined[x_vec1.size()+i] = shifted_x*sin(alpha)*cos(beta) + shifted_y*(sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) + shifted_z*(sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma));
            zmax_combined[x_vec1.size()+i] = shifted_x*(-sin(beta)) + shifted_y*(cos(beta)*sin(gamma)) + shifted_z*(cos(beta)*cos(gamma));
            x_combined[x_vec1.size()+i] += x_shift2[i];
            y_combined[x_vec1.size()+i] += y_shift2[i];
            zmax_combined[x_vec1.size()+i] += z_shift2[i];
        }
    
    bool solve_post = false;
    double post_roll = 0, post_pitch = 0, post_yaw = 0;
    if(solve_post) {
        tot_pts = pre_x.get_dim();
        indices all_pts("all_pts");
        all_pts = range(1, tot_pts);
        DebugOn("all_pts.size() = " << all_pts.size()<< endl);
        DebugOn("tot_pts = " << tot_pts << endl);
        Model<> MPost("MPost");
        var<> new_x("new_x"), new_y("new_y"), new_z("new_z");
        var<> zmax("zmax"), zmin("zmin");
        
        var<> yaw("yaw", -0.1, 0.1), pitch("pitch", -0.1, 0.1), roll("roll", -0.1, 0.1);
        MPost.add(yaw.in(R(1)),pitch.in(R(1)),roll.in(R(1)));
        MPost.add(new_x.in(all_pts), new_y.in(all_pts), new_z.in(all_pts));
        MPost.add(zmax.in(R(1)), zmin.in(R(1)));
        
        auto z_ids = zmax.repeat_id(tot_pts);
        
//        Constraint<> z_max("z_max");
//        z_max += zmax.in(z_ids) - new_z;
//        MPost.add(z_max.in(all_pts)>=0);
//
//        Constraint<> z_min("z_min");
//        z_min += zmin.in(z_ids) - new_z;
//        MPost.add(z_min.in(all_pts)<=0);
        
        auto ids = yaw.repeat_id(tot_pts);
        
        /* alpha = yaw_, beta = pitch_ and gamma = roll_ */
        Constraint<> x_rot("x_rot");
        x_rot += new_x - x_uav;
        x_rot -= (pre_x - x_uav)*cos(yaw.in(ids))*cos(pitch.in(ids)) + (pre_y)*(cos(yaw.in(ids))*sin(pitch.in(ids))*sin(roll.in(ids)) - sin(yaw.in(ids))*cos(roll.in(ids))) + (pre_z - z_uav)*(cos(yaw.in(ids))*sin(pitch.in(ids))*cos(roll.in(ids)) + sin(yaw.in(ids))*sin(roll.in(ids)));
        MPost.add(x_rot.in(all_pts)==0);
        
        Constraint<> y_rot("y_rot");
        y_rot += new_y - y_uav;
        y_rot -= (pre_x - x_uav)*sin(yaw.in(ids))*cos(pitch.in(ids)) + (pre_y - y_uav)*(sin(yaw.in(ids))*sin(pitch.in(ids))*sin(roll.in(ids)) + cos(yaw.in(ids))*cos(roll.in(ids))) + (pre_z - z_uav)*(sin(yaw.in(ids))*sin(pitch.in(ids))*cos(roll.in(ids)) - cos(yaw.in(ids))*sin(roll.in(ids)));
        MPost.add(y_rot.in(all_pts)==0);
        
        Constraint<> z_rot("z_rot");
        z_rot += new_z - z_uav;
        z_rot -= (pre_x - x_uav)*sin(-1*pitch.in(ids)) + (pre_y - y_uav)*(cos(pitch.in(ids))*sin(roll.in(ids))) + (pre_z - z_uav)*(cos(pitch.in(ids))*cos(roll.in(ids)));
        MPost.add(z_rot.in(all_pts)==0);
        
//        MPost.min(zmax - zmin);
        MPost.min(norm2(new_z - 1255));
//        MPost.print();
        
        solver<> S(MPost,ipopt);
        S.run();
        post_roll = roll.eval();
        post_pitch = pitch.eval();
        post_yaw = yaw.eval();
        
    }
    if(post_roll!=0 || post_pitch!=0 || post_yaw!=0){
        gamma = post_roll;
        beta = post_pitch;
        alpha = post_yaw;
        for (auto i = 0; i< x_combined.size(); i++) {
            shifted_x = x_combined[i] - x_shift[i];
            shifted_y = y_combined[i] - y_shift[i];
            shifted_z = zmax_combined[i] - z_shift[i];
            x_combined[i] = shifted_x*cos(alpha)*cos(beta) + shifted_y*(cos(alpha)*sin(beta)*sin(gamma) - sin(alpha)*cos(gamma)) + shifted_z*(cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma));
            y_combined[i] = shifted_x*sin(alpha)*cos(beta) + shifted_y*(sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma)) + shifted_z*(sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma));
            zmax_combined[i] = shifted_x*(-sin(beta)) + shifted_y*(cos(beta)*sin(gamma)) + shifted_z*(cos(beta)*cos(gamma));
            x_combined[i] += x_shift[i];
            y_combined[i] += y_shift[i];
            zmax_combined[i] += z_shift[i];
        }
    }
    DebugOn("Post-Pitch = " << post_pitch*180/pi << endl);
    DebugOn("Post-Roll = " << post_roll*180/pi << endl);
    DebugOn("Post-Yaw = " << post_yaw*180/pi << endl);

    
    
    
    namespace plt = matplotlibcpp;
   
    std::map<std::string, std::string> keywords, keywords2;
    keywords["marker"] = "s";
    keywords["linestyle"] = "None";
    keywords["ms"] = "0.05";
//    plt::plot3(x_combined, y_combined, zmax_combined, keywords);
    plt::plot3(uav_x1, uav_y1, uav_z1, x_combined, y_combined, zmax_combined, keywords);
//        plt::plot3(uav_x, uav_y, uav_z, x_combined, y_combined, zmax_combined, keywords);
    keywords2["marker"] = "s";
    keywords2["ms"] = "0.1";
    //    plt::plot3(uav_x1, uav_y1, uav_z1, uav_x, uav_y, uav_z, keywords2);
    //    plt::plot3(uav_x1, uav_y1, uav_z1, keywords);
    //    plt::plot3(x_vec2, y_vec2, zmax_vec2, keywords);
    //    plt::colorbar();
    // Enable legend.
    //    plt::legend();
    plt::show();
    
    DebugOn("Saving new las file");
//    LASreadOpener lasreadopener_final;
//    lasreadopener_final.set_file_name(LiDAR_file1.c_str());
//    lasreadopener_final.set_populate_header(TRUE);
//    LASreader* lasreader = lasreadopener_final.open();
    LASheader lasheader;
    lasheader.global_encoding = 1;
    lasheader.x_scale_factor = 0.01;
    lasheader.y_scale_factor = 0.01;
    lasheader.z_scale_factor = 0.01;
    lasheader.x_offset =  500000.0;
    lasheader.y_offset = 4100000.0;
    lasheader.z_offset = 0.0;
    lasheader.point_data_format = 1;
    lasheader.point_data_record_length = 28;
    
    LASwriteOpener laswriteopener;
    laswriteopener.set_file_name("corrected.las");
    LASwriter* laswriter = laswriteopener.open(&lasheader);
    LASpoint laspoint;
    laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);
    for (auto i = 0; i< x_combined.size(); i++) {
        laspoint.set_x(x_combined[i]);
        laspoint.set_y(y_combined[i]);
        laspoint.set_z(zmax_combined[i]);
        laswriter->write_point(&laspoint);
        laswriter->update_inventory(&laspoint);
    }
    laswriter->update_header(&lasheader, TRUE);
    laswriter->close();
    delete laswriter;
    return 0;
}
