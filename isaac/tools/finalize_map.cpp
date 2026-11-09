// Offline consolidation of the real WM+LTM graph with RTAB-Map's own optimizer.
// No source/world/GT poses are read by this program. Run on an own DB COPY with
// the simulator and online SLAM stopped, then use the unchanged DB exporter.
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Parameters.h>
#include <iostream>
#include <memory>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  rtabmap::ParametersMap params;
  params[rtabmap::Parameters::kMemIncrementalMemory()] = "false";
  params[rtabmap::Parameters::kMemInitWMWithAllNodes()] = "true";
  params[rtabmap::Parameters::kRtabmapMemoryThr()] = "0";
  rtabmap::Rtabmap slam;
  slam.init(params, argv[1], true);  // Preserve the recorded source optimizer parameters.
  std::map<int, rtabmap::Transform> poses;
  std::multimap<int, rtabmap::Link> links;
  slam.getGraph(poses, links, true, true); // optimized=true, global=true includes LTM.
  auto lastPose = slam.getLastLocalizationPose();
  slam.close();
  int landmarks = 0, nodes = 0;
  for (const auto &entry : poses) {
    if (entry.first < 0) ++landmarks;
    if (entry.first > 0) ++nodes;
  }
  if (nodes < 50 || landmarks < 2) {
    std::cerr << "Incomplete global graph: " << nodes << " nodes, " << landmarks << " landmarks\n";
    return 3;
  }
  std::unique_ptr<rtabmap::DBDriver> database(rtabmap::DBDriver::create());
  if (!database->openConnection(argv[1])) return 4;
  database->saveOptimizedPoses(poses, lastPose);
  database->closeConnection();
  std::cout << "GLOBAL_GRAPH_FINALIZED nodes=" << nodes << " landmarks=" << landmarks
            << " links=" << links.size() << std::endl;
  return 0;
}
