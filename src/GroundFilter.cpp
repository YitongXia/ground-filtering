/*
  GEO1015.2021
  hw03 
  --
  Fengyan Zhang
  5462150
  Yitong Xia
  5445825
*/

#include "GroundFilter.h"
#include <queue>
// -- LAS reading and writing
#include <lasreader.hpp>

#include <laswriter.hpp>

// -- CGAL delaunay triangulation
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>

// -- CGAL kd-tree
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>

void groundfilter_tin(const std::vector<Point> &pointcloud, const json &jparams) {
    /*
      !!! TO BE COMPLETED !!!

      Function that performs ground filtering using TIN refinement and writes the result to a new LAS file.

      !!! You are free to subdivide the functionality of this function into several functions !!!

      Inputs:
        pointcloud: input point cloud (an Nx3 numpy array),
        jparams: a dictionary jparams with all the parameters that are to be used in this function:
          - resolution:    resolution (cellsize) for the initial grid that is computed as part of the ground filtering algorithm,
          - distance:      distance threshold used in the ground filtering algorithm,
          - angle:         angle threshold used in the ground filtering algorithm in degrees,
          - output_las:    path to output .las file that contains your ground classification,
    */
    typedef CGAL::Projection_traits_xy_3<Kernel> Gt;
    typedef CGAL::Delaunay_triangulation_2<Gt> DT;
    typedef CGAL::Simple_cartesian<Point> K;
    typedef K::Point_3 Point_d;

    double resolution = jparams["resolution"];
    double distance = jparams["distance"];
    double angle = jparams["angle"];
    std::string output_las = jparams["output_las"];

    double x_min = pointcloud[0][0];
    double y_min = pointcloud[0][1];
    double x_max = pointcloud[0][0];
    double y_max = pointcloud[0][1];
    for (int i = 1; i < pointcloud.size(); i++) {
        if (x_min > pointcloud[i][0]) x_min = pointcloud[i][0];
        if (y_min > pointcloud[i][1]) y_min = pointcloud[i][1];
        if (x_max < pointcloud[i][0]) x_max = pointcloud[i][0];
        if (y_max < pointcloud[i][1]) y_max = pointcloud[i][1];
    }

    //double resolution = jparams["resolution"];
    int ncols = ceil((x_max - x_min) / resolution);
    int nrows = ceil((y_max - y_min) / resolution);

    //build initial grid
    std::vector<const Point *> grid;
    std::vector<Point> init_tin;
    std::vector<int> class_labels;
    std::vector<Point> tin;

    for (int i = 0; i < nrows * ncols; ++i) {
        grid.push_back(0);
    }

    for (int i = 0; i < pointcloud.size(); ++i) {
        int row = floor((pointcloud[i].x() - x_min) / resolution);
        int col = floor((pointcloud[i].y() - y_min) / resolution);
        int index = col + row * nrows;
        if (grid[index] == 0)
            grid[index] = &pointcloud[i];
        else if (grid[index] != 0) {
            if (pointcloud[i].z() < grid[index]->z())
                grid[index] = &pointcloud[i];
        }
    }

    for (int i = 0; i < grid.size(); ++i) {
        init_tin.emplace_back(*grid[i]);
    }

    //construct DT
    DT dt;
    for (int i = 0; i < init_tin.size(); ++i) {
        dt.insert(init_tin[i]);
    }

    //iterate every point in the point cloud data
    for (int i = 0; i < pointcloud.size(); ++i) {

        // if convex hull needed
        dt.insert(pointcloud[i]);
        DT::Face_handle triangle = dt.locate(pointcloud[i]);
        DT::Vertex_handle v0 = triangle->vertex(0);
        DT::Vertex_handle v1 = triangle->vertex(1);
        DT::Vertex_handle v2 = triangle->vertex(2);
        double d0 = CGAL::squared_distance(v0->point(), pointcloud[i]);
        double d1 = CGAL::squared_distance(v1->point(), pointcloud[i]);
        double d2 = CGAL::squared_distance(v2->point(), pointcloud[i]);

        /*
        std::cout<<"Pointcloud_i is " << pointcloud[i]<<std::endl;
        std::cout<<"v0 is "<<v0->point() <<std::endl;
        std::cout<<"v1 is "<<v1->point()  <<std::endl;
        std::cout<<"v2 is "<<v2->point()  <<std::endl;
         */


        Kernel::Plane_3 plane = Kernel::Plane_3(v0->point(), v1->point(), v2->point());
        double h = CGAL::squared_distance(pointcloud[i], plane);
        if (h == 0) {
            tin.emplace_back(pointcloud[i]);
            class_labels.emplace_back(1);
        } else if (h != 0) {
            double temp_angle[3];
            temp_angle[0] = asin(h / d0);
            temp_angle[1] = asin(h / d1);
            temp_angle[2] = asin(h / d2);
            double max_angle = temp_angle[0];
            for (int j = 0; j < 3; j++) {
                if (temp_angle[j] > max_angle) { max_angle = temp_angle[j]; }
            }
            if (h <= distance && max_angle <= angle) {
                tin.emplace_back(pointcloud[i]);
                class_labels.emplace_back(1);
            } else {
                class_labels.emplace_back(2);
                tin.emplace_back(pointcloud[i]);
            }
        }
    }
    std::cout << "the number of point " << tin.size() << std::endl;
    std::cout << "the number of class labels " << class_labels.size() << std::endl;
    write_lasfile(jparams["output_las"], tin, class_labels);
}

void groundfilter_csf(const std::vector<Point>& pointcloud, const json& jparams) {
  /*
  !!! TO BE COMPLETED !!!
    
  Function that performs ground filtering using CSF and writes the result to a new LAS file.

  !!! You are free to subdivide the functionality of this function into several functions !!!
    
  Inputs:
    pointcloud: input point cloud (an Nx3 numpy array),
    jparams: a dictionary with all the parameters that are to be used in this function:
      - resolution:     resolution of the cloth grid,
      - epsilon_zmax:   tolerance to stop the iterations,
      - epsilon_ground: threshold used to classify ground points,
      - output_las:     path to output .las file that contains your ground classification
  */
  typedef CGAL::Search_traits_3<Kernel> TreeTraits;
  typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
  typedef Neighbor_search::Tree Tree;

  // double resolution = j["resolution"];
  // double epsilon_zmax = j["epsilon_zmax"];
  // double epsilon_ground = j["epsilon_ground"];
  // std::string output_las = jparams["output_las"];

  // //-- print the first 5 points in the pointcloud, which are CGAL Point_3
  // //-- https://doc.cgal.org/latest/Kernel_23/classCGAL_1_1Point__3.html
  // int i = 0;
  // for (auto p : pointcloud) {
  //   std::cout << "(" << p.x() << ", " << p.y() << ", " << p.z()  << ")" << std::endl;
  //   i++;
  //   if (i == 5) 
  //     break;
  // }

  //-- TIP
  //-- construct and query kd-tree:
  // https://doc.cgal.org/latest/Spatial_searching/index.html#title5
  //
  // Tree tree(pointcloud.begin(), pointcloud.end());  
  // const unsigned int N = 1;
  // Point query_point = Point(0,0,0);
  // Neighbor_search search_result(tree, query_point, N);
  //
  // for(auto res : search_result) {
  //   Point neighbour_point = res.first;
  //   double distance = res.second;
  // }

  //-- TIP
  //-- write the results to a new LAS file
  // std::vector<int> class_labels;
  // write_lasfile(jparams["output_las"], pointcloud, class_labels);
}



std::vector<Point> read_lasfile(const json& jparams) {
  /*
  Function to read points from a LAS file

  Inputs:
    jparams["filename"]:   the filename to read the LAS file to

  Returns:
    a std::vector<Point> with the points from the LAS file
  */
  std::string filename = jparams["filename"];
	LASreadOpener lasreadopener;
	lasreadopener.set_file_name(filename.c_str());
	LASreader* lasreader = lasreadopener.open();
	
	if (!lasreader){
		std::cerr << "cannot read las file: " << filename << "\n";
		exit(1);
	}

  //-- store each point in a CGAL Point_3 object
  //-- https://doc.cgal.org/latest/Kernel_23/classCGAL_1_1Point__3.html
	std::vector<Point> points;
	while (lasreader->read_point()) {
		points.push_back( 
			Point(
				lasreader->point.get_x(),
				lasreader->point.get_y(),
				lasreader->point.get_z()
			)
		);
	}
	lasreader->close();
	delete lasreader;

	return points;
}



void write_lasfile(const std::string filename, const std::vector<Point>& pointcloud, const std::vector<int>& class_labels) {
  /*
  Function to write a new LAS file with point labels (for the LAS classification field)

  Inputs:
    filename:   the filename to write the LAS file to
    pointcloud: input point cloud (a vector of Points),
    Labels:     Contains point labels. Should be a vector of ints of the same size as pointcloud (ie. one label for each point in the same order as pointcloud). Uses LAS classification codes, ie 2 = ground. 1 = unclassified.
  */
  LASwriteOpener laswriteopener;
  laswriteopener.set_file_name(filename.c_str());

  LASheader lasheader;
  lasheader.x_scale_factor = 0.01;
  lasheader.y_scale_factor = 0.01;
  lasheader.z_scale_factor = 0.01;
  lasheader.x_offset = 0.0;
  lasheader.y_offset = 0.0;
  lasheader.z_offset = 0.0;
  lasheader.point_data_format = 0;
  lasheader.point_data_record_length = 20;

  LASpoint laspoint;
  laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);

  LASwriter* laswriter = laswriteopener.open(&lasheader);
  if (laswriter == 0)
  {
    std::cerr << "ERROR: could not open laswriter\n";
    exit(1);
  }

	if (pointcloud.size()!=class_labels.size()) {
		std::cerr << "ERROR: points has a different size than class_labels\n";
		exit(1);
	}

  for (size_t i=0; i<pointcloud.size(); ++i) {
		const Point& p = pointcloud[i];
		const int& label = class_labels[i];

    laspoint.set_x(p[0]);
    laspoint.set_y(p[1]);
    laspoint.set_z(p[2]);
		laspoint.set_classification(label);

    laswriter->write_point(&laspoint);
    laswriter->update_inventory(&laspoint);    
  } 

  laswriter->update_header(&lasheader, TRUE);
  laswriter->close();
  delete laswriter;
}
