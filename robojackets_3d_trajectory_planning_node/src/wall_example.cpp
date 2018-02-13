#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace std;
using namespace octomap;


void print_query_info(point3d query, OcTreeNode* node)
{
  if (node != NULL)
  {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else 
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}

int main(int argc, char** argv) {

  const double kresolution_octomap = 1.0f;
  const double kwall_minimum_x = -10.0f;
  const double kwall_maximum_x = +10.0f;
  const double kwall_minimum_z = -2.0f;
  const double kwall_maximum_z = +2.0f;

  cout << endl;
  cout << "generating example map with a wall" << endl;

  OcTree tree (kresolution_octomap);  // create empty tree with resolution kresolution_octomap

  // insert occupied measurements for a wall
  double x_wall, y_wall, z_wall;
  y_wall = 40;
  for(int redundant_runs = 0; redundant_runs < 5; redundant_runs++)
  {
    cout << "Running wall generation" << endl;
    for(x_wall = kwall_minimum_x; x_wall <= kwall_maximum_x; x_wall += kresolution_octomap)
    {
      for(z_wall = kwall_minimum_z; z_wall <= kwall_maximum_z; z_wall += kresolution_octomap)
      {
        octomap::point3d start(0, 0, 0);
        octomap::point3d end(x_wall, y_wall, z_wall);

        tree.insertRay(start, end);
      }
    }
  }

/*


  // insert some measurements of occupied cells

  for (int x=-20; x<20; x++) {
    for (int y=-20; y<20; y++) {
      for (int z=-20; z<20; z++) {
        point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // insert some measurements of free cells

  for (int x=-30; x<30; x++) {
    for (int y=-30; y<30; y++) {
      for (int z=-30; z<30; z++) {
        point3d endpoint ((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
        tree.updateNode(endpoint, false);  // integrate 'free' measurement
      }
    }
  }
*/

  cout << endl;
  cout << "performing some queries:" << endl;

  point3d query (0., 0., 0.);
  OcTreeNode* result;

  double x_query, y_query, z_query, y_delta_query;
  y_query = 40;
  for(x_query = kwall_minimum_x; x_query <= kwall_maximum_x; x_query += kresolution_octomap)
  {
    for(z_query = kwall_minimum_z; z_query <= kwall_maximum_z; z_query += kresolution_octomap)
    {
      // 2m in front of wall
      y_delta_query = -2.0f;
      query = point3d(x_query, y_query+y_delta_query, z_query);
      result = tree.search (query);
      print_query_info(query, result);

      // Inside wall
      y_delta_query = 0.0f;
      query = point3d(x_query, y_query+y_delta_query, z_query);
      result = tree.search (query);
      print_query_info(query, result);

      // 2m behind of wall
      y_delta_query = +2.0f;
      query = point3d(x_query, y_query+y_delta_query, z_query);
      result = tree.search (query);
      print_query_info(query, result);

      cout << "-----------------------------------------" << endl;
    }
  }
  cout << endl;
  cout << "performing some queries:" << endl;
  
  query = point3d(0., 0., 0.);
  result = tree.search (query);
  print_query_info(query, result);

  query = point3d(-1.,-1.,-1.);
  result = tree.search (query);
  print_query_info(query, result);

  query = point3d(1.,1.,1.);
  result = tree.search (query);
  print_query_info(query, result);


  cout << endl;
  tree.writeBinary("wall_tree.bt");
  cout << "wrote example file wall_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis wall_tree.bt"  << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;  

}
