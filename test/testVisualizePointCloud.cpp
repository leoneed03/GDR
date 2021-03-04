//
// Copyright (c) Leonid Seniukov. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//


#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <fstream>

#include <pcl/visualization/cloud_viewer.h>
#include "poseEstimation.h"
#include "CorrespondenceGraph.h"
#include "groundTruthTransformer.h"
#include "SmoothPointCloud.h"


#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/range/irange.hpp>


void visualizeSimple() {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<std::pair<std::vector<double>, std::vector<int>>> XYZ_RGBs;

    for (int i = 0; i < 200; ++i) {
        int rgb = (i * (256 / 20)) % 256;
        XYZ_RGBs.push_back({{1.0 * i, 1.0 * i, 1.0 * i},
                            {rgb,     rgb,     rgb}});
    }

    for (int i = 0; i < XYZ_RGBs.size(); ++i) {

        auto &pointXYZ = XYZ_RGBs[i].first;
        auto &pointRGB = XYZ_RGBs[i].second;
        pcl::PointXYZRGB pointToBeAdded;
        pointToBeAdded.x = pointXYZ[0];
        pointToBeAdded.y = pointXYZ[1];
        pointToBeAdded.z = pointXYZ[2];

        pointToBeAdded.r = pointRGB[0];
        pointToBeAdded.g = pointRGB[1];
        pointToBeAdded.b = pointRGB[2];

        input_cloud->push_back(pointToBeAdded);


//        if (i % 3 == 1 || i == XYZ_RGBs.size()) {
//            // Filtering input scan to roughly 10% of original size to increase speed of registration.
//            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//            pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> approximate_voxel_filter;
//            approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);
//            approximate_voxel_filter.setInputCloud(input_cloud);
//            approximate_voxel_filter.filter(*filtered_cloud);
//            std::swap(filtered_cloud, input_cloud);
//        }
    }
//
//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//    viewer->setBackgroundColor(0, 0, 0);
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(input_cloud);
//    viewer->addPointCloud<pcl::PointXYZRGB>(input_cloud, rgb, "sample cloud");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//    viewer->addCoordinateSystem(1.0);
//    viewer->initCameraParameters();
//    while (!viewer->wasStopped()) {
//    }
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(input_cloud);

    while (!viewer.wasStopped()) {
    }

}


TEST(testVisualizationGT, SmoothedPointCloudGroundTruthPosesNumber2) {

    std::string pathToImageDirectoryRGB = "../../data/360_dataset_sampled/each5/rgb";
    std::string pathToImageDirectoryD = "../../data/360_dataset_sampled/each5/depth";

    auto imagesRgb = gdr::readRgbData(pathToImageDirectoryRGB);
    auto imagesD = gdr::readRgbData(pathToImageDirectoryD);
//    gdr::CorrespondenceGraph correspondenceGraph("../../data/360_dataset_sampled/each5/rgb",
//                                                 "../../data/360_dataset_sampled/each5/depth",
//                                                 517.3,318.6,
//                                                 516.5, 255.3);
    gdr::CameraRGBD camera(517.3, 318.6,
                           516.5, 255.3);
//    correspondenceGraph.computeRelativePoses();
//    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
//    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
//    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();

//    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();


    std::string absolutePoses = "../../data/360_dataset_sampled/each5/groundtruth_new.txt";
//    std::string absolutePoses = "../../data/360_dataset_sampled/each5/BA_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);
    std::vector<gdr::VertexCG> vertices;
    std::vector<gdr::VertexCG *> verticesPointers;

    Sophus::SE3d poseZero = posesInfo[0].getSophusPose();

    ASSERT_EQ(posesInfo.size(), imagesRgb.size());
    ASSERT_EQ(posesInfo.size(), imagesD.size());
    for (int i = 0; i < posesInfo.size(); ++i) {
        vertices.push_back(
                gdr::VertexCG(i, camera, imagesRgb[i], imagesD[i], poseZero.inverse() * posesInfo[i].getSophusPose()));
    }

    for (int i = 0; i < posesInfo.size(); ++i) {
        verticesPointers.push_back(&vertices[i]);
    }
    gdr::SmoothPointCloud smoothCloud;
    smoothCloud.registerPointCloudFromImage(verticesPointers);

}


TEST(testVisualizationGT, SmoothedPointCloudGroundTruth) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb",
                                                 "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
//    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
//    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
//    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();

//    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();


    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> poses = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);
    std::vector<gdr::VertexCG *> vertices;

    assert(poses.size() == correspondenceGraph.verticesOfCorrespondence.size());
    assert(poses.size() == 19);

    Sophus::SE3d pose0;
    pose0.setQuaternion(poses[0].getOrientationQuat());
    pose0.translation() = poses[0].getTranslation();
    for (int i = 0; i < poses.size(); ++i) {
        auto *vertex = &correspondenceGraph.verticesOfCorrespondence[i];
        Sophus::SE3d poseI;
        poseI.setQuaternion(poses[i].getOrientationQuat());
        poseI.translation() = poses[i].getTranslation();

        vertex->setRotationTranslation(pose0.inverse() * poseI);

        std::cout << "pose " << i << " qx qy qz qw: " << vertex->getRotationQuat().coeffs().transpose() << " tx ty tz: "
                  << vertex->getEigenMatrixAbsolutePose4d().topRightCorner<3, 1>().transpose() << std::endl;

        auto se3 = vertex->getEigenMatrixAbsolutePose4d();
        vertices.push_back(vertex);
    }
    gdr::SmoothPointCloud smoothCloud;
    smoothCloud.registerPointCloudFromImage(vertices);

}


TEST(testVisualization, SmoothedPointCloud) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb",
                                                 "../../data/plantDataset_19_3/depth",
                                                 517.3, 318.6,
                                                 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();
    std::vector<gdr::VertexCG *> vertices;
    for (auto &vertex: correspondenceGraph.verticesOfCorrespondence) {
        vertices.push_back(&vertex);
    }


    std::cout << "total Umeyama poses " << correspondenceGraph.totalMeausedRelativePoses << std::endl;
    std::cout << " ICP refined poses " << correspondenceGraph.refinedPoses << " percentage:  "
              << 1.0 * correspondenceGraph.refinedPoses / correspondenceGraph.totalMeausedRelativePoses << std::endl;

    gdr::SmoothPointCloud smoothCloud;
    smoothCloud.registerPointCloudFromImage(vertices);

}

TEST(testVisualization, ShonanConverges) {

    int iterations = 10;
    for (int i = 0; i < iterations; ++i) {
        gdr::CorrespondenceGraph correspondenceGraph("../../data/360_dataset_sampled/each5/rgb",
                                                     "../../data/360_dataset_sampled/each5/depth",
                                                     517.3, 318.6,
                                                     516.5, 255.3);
        correspondenceGraph.computeRelativePoses();
        std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
        std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
        std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
        std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();
        std::vector<gdr::VertexCG *> vertices;
        for (auto &vertex: correspondenceGraph.verticesOfCorrespondence) {
            vertices.push_back(&vertex);
        }
    }
    std::cout << "shonan converged " << iterations << std::endl;
    ASSERT_TRUE(true);

}

TEST(testVisualization, cabinet360turn_each10) {

//    std::set<int> sampledIndices;
//    for (int i = 0; i < 993; i += 10) {
//        sampledIndices.insert(i);
//    }
//    gdr::GTT::prepareDataset("/home/leo/Desktop/datasets/rgbd_dataset_freiburg3_large_cabinet",
//                             "../../data/" + datasetName,
//                             sampledIndices,
//                             "");

    std::string datasetName = "cabinetDataset_100_10";
    gdr::CorrespondenceGraph correspondenceGraph("../../data/" + datasetName + "/rgb",
                                                 "../../data/" + datasetName + "/depth",
                                                 535.4, 320.1,
                                                 539.2, 247.6);
    correspondenceGraph.computeRelativePoses();
    bool isConnected = true;

    std::vector<std::vector<int>> components;
    correspondenceGraph.bfs(0, isConnected, components);
    ASSERT_TRUE(isConnected);
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();
    std::vector<gdr::VertexCG *> vertices;
    for (auto &vertex: correspondenceGraph.verticesOfCorrespondence) {
        vertices.push_back(&vertex);
    }

    std::string absolutePosesGT = "../../data/360_2/groundtruth_new.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePosesGT);
    std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/360_sampled/BA_378.txt";
    std::ofstream computedPoses(outputName);

    assert(posesInfo.size() == correspondenceGraph.verticesOfCorrespondence.size());
    assert(!posesInfo.empty());
    for (int i = 0; i < correspondenceGraph.verticesOfCorrespondence.size(); ++i) {
        const auto &poseBA = bundleAdjustedPoses[i];
        Sophus::SE3d movedPose = posesInfo[0].getSophusPose() * poseBA;
        const auto to = movedPose.translation();
        computedPoses.precision(std::numeric_limits<double>::max_digits10);
        computedPoses << posesInfo[i].getTimestamp() << ' ';
        for (int j = 0; j < 3; ++j) {
            computedPoses << to[j] << ' ';
        }
        auto quatComputed = movedPose.unit_quaternion();

        computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                      << quatComputed.w() << std::endl;
    }

    std::cout << "total Umeyama poses " << correspondenceGraph.totalMeausedRelativePoses << std::endl;
    std::cout << " ICP refined poses " << correspondenceGraph.refinedPoses << " percentage:  "
              << 1.0 * correspondenceGraph.refinedPoses / correspondenceGraph.totalMeausedRelativePoses << std::endl;

    double voxelSize = 0.001;
    gdr::SmoothPointCloud smoothCloud;
    smoothCloud.registerPointCloudFromImage(vertices, voxelSize, voxelSize, voxelSize);

}

TEST(testVisualization, SmoothedPointCloud360OfficeEach2) {

    std::set<int> sampledIndices;
    for (int i = 0; i < 755; i += 2) {
        sampledIndices.insert(i);
    }
    gdr::GTT::prepareDataset("/home/leoneed/Desktop/360dataset", "/home/leoneed/testGDR1/GDR/data/360_2",
                             sampledIndices, "");
    gdr::CorrespondenceGraph correspondenceGraph("../../data/360_2/rgb",
                                                 "../../data/360_2/depth",
                                                 517.3, 318.6,
                                                 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    bool isConnected = true;

    std::vector<std::vector<int>> components;
    correspondenceGraph.bfs(0, isConnected, components);
    ASSERT_TRUE(isConnected);
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();
    std::vector<gdr::VertexCG *> vertices;
    for (auto &vertex: correspondenceGraph.verticesOfCorrespondence) {
        vertices.push_back(&vertex);
    }

    std::string absolutePosesGT = "../../data/360_2/groundtruth_new.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePosesGT);
    std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/360_sampled/BA_378.txt";
    std::ofstream computedPoses(outputName);

    assert(posesInfo.size() == correspondenceGraph.verticesOfCorrespondence.size());
    assert(!posesInfo.empty());
    for (int i = 0; i < correspondenceGraph.verticesOfCorrespondence.size(); ++i) {
        const auto &poseBA = bundleAdjustedPoses[i];
        Sophus::SE3d movedPose = posesInfo[0].getSophusPose() * poseBA;
        const auto to = movedPose.translation();
        computedPoses.precision(std::numeric_limits<double>::max_digits10);
        computedPoses << posesInfo[i].getTimestamp() << ' ';
        for (int j = 0; j < 3; ++j) {
            computedPoses << to[j] << ' ';
        }
        auto quatComputed = movedPose.unit_quaternion();

        computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                      << quatComputed.w() << std::endl;
    }

    std::cout << "total Umeyama poses " << correspondenceGraph.totalMeausedRelativePoses << std::endl;
    std::cout << " ICP refined poses " << correspondenceGraph.refinedPoses << " percentage:  "
              << 1.0 * correspondenceGraph.refinedPoses / correspondenceGraph.totalMeausedRelativePoses << std::endl;

    double voxelSize = 0.001;
    gdr::SmoothPointCloud smoothCloud;
    smoothCloud.registerPointCloudFromImage(vertices, voxelSize, voxelSize, voxelSize);

}


using namespace boost;

template<typename TimeMap>
class bfs_time_visitor : public default_bfs_visitor {
    typedef typename property_traits<TimeMap>::value_type T;
public:
    bfs_time_visitor(TimeMap tmap, T &t) : m_timemap(tmap), m_time(t) {}

    template<typename Vertex, typename Graph>
    void discover_vertex(Vertex u, const Graph &g) const {
        put(m_timemap, u, m_time++);
    }

    TimeMap m_timemap;
    T &m_time;
};


struct VertexProps {
    boost::default_color_type color;
    std::size_t discover_time;
};

int bfsBoost() {
    using namespace boost;
    // Select the graph type we wish to use
    typedef adjacency_list<listS, listS, undirectedS,
            VertexProps> graph_t;
    // Set up the vertex IDs and names
    enum {
        r, s, t, u, v, w, x, y, N
    };
    const char *name = "rstuvwxy";
    // Specify the edges in the graph
    typedef std::pair<int, int> E;
    E edge_array[] = {E(r, s), E(r, v), E(s, w), E(w, r), E(w, t),
                      E(w, x), E(x, t), E(t, u), E(x, y), E(u, y)
    };
    // Create the graph object
    const int n_edges = sizeof(edge_array) / sizeof(E);
    typedef graph_traits<graph_t>::vertices_size_type v_size_t;
    graph_t g(edge_array, edge_array + n_edges, v_size_t(N));

    // Typedefs
    typedef graph_traits<graph_t>::vertex_descriptor Vertex;
    typedef graph_traits<graph_t>::vertices_size_type Size;
    typedef Size *Iiter;

    Size time = 0;
    typedef property_map<graph_t, std::size_t VertexProps::*>::type dtime_map_t;
    dtime_map_t dtime_map = get(&VertexProps::discover_time, g);
    bfs_time_visitor<dtime_map_t> vis(dtime_map, time);
    breadth_first_search(g, vertex(s, g), color_map(get(&VertexProps::color, g)).
            visitor(vis));

    // a vector to hold the discover time property for each vertex
    std::vector<Size> dtime(num_vertices(g));
    graph_traits<graph_t>::vertex_iterator vi, vi_end;
    std::size_t c = 0;
    for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi, ++c)
        dtime[c] = dtime_map[*vi];

    // Use std::sort to order the vertices by their discover time
    std::vector<graph_traits<graph_t>::vertices_size_type> discover_order(N);
    integer_range<int> range(0, N);
    std::copy(range.begin(), range.end(), discover_order.begin());
    std::sort(discover_order.begin(), discover_order.end(),
              indirect_cmp<Iiter, std::less<Size> >(&dtime[0]));

    std::cout << "order of discovery: ";
    for (int i = 0; i < N; ++i)
        std::cout << name[discover_order[i]] << " ";
    std::cout << std::endl;

    return EXIT_SUCCESS;
}


class V {
};

class C {
};


#include <boost/graph/connected_components.hpp>


using namespace boost;

void draw_test() {
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, V, C> MyGraph;
    typedef boost::graph_traits<MyGraph>::vertex_descriptor vertex_descriptor;
    MyGraph g;
    vertex_descriptor a = add_vertex(V(), g);
    vertex_descriptor b = add_vertex(V(), g);
    vertex_descriptor c = add_vertex(V(), g);
    vertex_descriptor d = add_vertex(V(), g);

    vertex_descriptor e = add_vertex(V(), g);

    vertex_descriptor f1 = add_vertex(V(), g);
    vertex_descriptor f2 = add_vertex(V(), g);

    add_edge(a, b, g);
    add_edge(b, c, g);
    add_edge(c, d, g);
    add_edge(a, d, g);
    add_edge(f1, f2, g);

    std::ofstream outf("../../tools/data/temp/min.dot");
    boost::write_graphviz(outf, g);

    std::vector<int> component(boost::num_vertices(g));
    size_t num_components = boost::connected_components(g, component.data());

    for (int i = 0; i < component.size(); ++i) {
        std::cout << " vertex " << i << ": component " << component[i] << std::endl;
    }

}

TEST(testGraphVisualization, gv) {
    draw_test();
}

TEST(testVisualizationA, __SmoothedPointCloud360OfficeSmallEach5_boost_bfs) {

    std::string datasetFolder = "360_3_poses";
//    gdr::GTT::prepareDataset("/home/leoneed/Desktop/360dataset", "/home/leoneed/testGDR1/GDR/data/" + datasetFolder,
//                             sampledIndices, "");
    gdr::CorrespondenceGraph correspondenceGraph("../../data/" + datasetFolder + "/rgb",
                                                 "../../data/" + datasetFolder + "/depth",
                                                 517.3, 318.6,
                                                 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    correspondenceGraph.bfsDrawToFile("../../tools/data/temp/360degreePoses15.gv");

}


TEST(testVisualization, SmoothedPointCloud360OfficeSmallEach5_poses_15) {

    std::set<int> sampledIndices;

    for (int i = 345; i < 420; i += 5) {
        sampledIndices.insert(i);
    }
    std::cout << " size of sample: " << sampledIndices.size() << std::endl;

    std::string datasetFolder = "360_3_poses";
//    gdr::GTT::prepareDataset("/home/leoneed/Desktop/360dataset", "/home/leoneed/testGDR1/GDR/data/" + datasetFolder,
//                             sampledIndices, "");
    gdr::CorrespondenceGraph correspondenceGraph("../../data/" + datasetFolder + "/rgb",
                                                 "../../data/" + datasetFolder + "/depth",
                                                 517.3, 318.6,
                                                 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();



    std::vector<std::vector<int>> components;
    bool isConnected = true;
    correspondenceGraph.bfs(0, isConnected, components);
    ASSERT_TRUE(isConnected);
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();
    std::vector<gdr::VertexCG *> vertices;
    for (auto &vertex: correspondenceGraph.verticesOfCorrespondence) {
        vertices.push_back(&vertex);
    }

    std::string absolutePosesGT = "../../data/" + datasetFolder + "/groundtruth_new.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePosesGT);
    std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/360_sampled/BA_3.txt";
    std::ofstream computedPoses(outputName);

    assert(posesInfo.size() == correspondenceGraph.verticesOfCorrespondence.size());
    assert(!posesInfo.empty());
    for (int i = 0; i < correspondenceGraph.verticesOfCorrespondence.size(); ++i) {
        const auto &poseBA = bundleAdjustedPoses[i];
        Sophus::SE3d movedPose = posesInfo[0].getSophusPose() * poseBA;
        const auto to = movedPose.translation();
        computedPoses.precision(std::numeric_limits<double>::max_digits10);
        computedPoses << posesInfo[i].getTimestamp() << ' ';
        for (int j = 0; j < 3; ++j) {
            computedPoses << to[j] << ' ';
        }
        auto quatComputed = movedPose.unit_quaternion();

        computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                      << quatComputed.w() << std::endl;
    }

    std::cout << "total Umeyama poses " << correspondenceGraph.totalMeausedRelativePoses << std::endl;
    std::cout << " ICP refined poses " << correspondenceGraph.refinedPoses << " percentage:  "
              << 1.0 * correspondenceGraph.refinedPoses / correspondenceGraph.totalMeausedRelativePoses << std::endl;

    // compare umeyama poses vs ICP refined umeyama
    ASSERT_EQ(correspondenceGraph.transformationMatricesLoRansac.size(),
              correspondenceGraph.transformationMatricesICP.size());
    int umeyamaMatrixIsBetter = 0;
    int icpMatrixIsBetter = 0;
    int totalMatrices = 0;

    std::vector<std::vector<int>> pairsICPbetterGT(correspondenceGraph.verticesOfCorrespondence.size());

    for (int i = 0; i < correspondenceGraph.transformationMatricesLoRansac.size(); ++i) {
        ASSERT_EQ(correspondenceGraph.transformationMatricesLoRansac[i].size(),
                  correspondenceGraph.transformationMatricesICP[i].size());

        for (auto &indexToAndMatrix: correspondenceGraph.transformationMatricesLoRansac[i]) {

            int indexTo = indexToAndMatrix.first;
            auto &poseUmeyama = correspondenceGraph.transformationMatricesLoRansac[i].find(indexTo)->second;
            auto &poseICP = correspondenceGraph.transformationMatricesICP[i].find(indexTo)->second;
            Sophus::SE3d relativePoseGT = posesInfo[poseUmeyama.getIndexFrom()].getSophusPose().inverse() *
                                          posesInfo[poseUmeyama.getIndexTo()].getSophusPose();

            ASSERT_EQ(poseUmeyama.getIndexFrom(), poseICP.getIndexFrom());
            ASSERT_EQ(poseUmeyama.getIndexTo(), poseICP.getIndexTo());
            ASSERT_EQ(poseUmeyama.getIndexFrom(), i);

            double errorRotUmeyama = poseUmeyama.getRelativeRotation().angularDistance(
                    relativePoseGT.unit_quaternion());
            double errorRotICP = poseICP.getRelativeRotation().angularDistance(relativePoseGT.unit_quaternion());

            double errorTUmeyama = (poseUmeyama.getRelativeTranslation() - relativePoseGT.translation()).norm();
            double errorTICP = (poseICP.getRelativeTranslation() - relativePoseGT.translation()).norm();

            std::cout
                    << "===========================================NEXT RELATIVE POSE=========================================="
                    << std::endl;
            std::cout << "From: " << poseUmeyama.getIndexFrom() << " To: " << poseUmeyama.getIndexTo() << std::endl;
            std::cout << "    Umeyama: " << "\tR: " << errorRotUmeyama << " \tt: " << errorTUmeyama << std::endl;
            std::cout << "    ICP    : " << "\tR: " << errorRotICP << " \tt: " << errorTICP << std::endl;
            if (errorRotUmeyama + errorTUmeyama < errorRotICP + errorTICP) {
                ++umeyamaMatrixIsBetter;
            } else {
                ++icpMatrixIsBetter;
                pairsICPbetterGT[poseUmeyama.getIndexFrom()].push_back(poseUmeyama.getIndexTo());
                std::cout << "                                                       _________REFINED__________"
                          << std::endl;
            }
            ++totalMatrices;
        }
    }
    ASSERT_EQ(umeyamaMatrixIsBetter + icpMatrixIsBetter, totalMatrices);

    std::cout << " ICP according to CG (keypoint best number of matches) refined poses "
              << correspondenceGraph.refinedPoses << " / " << correspondenceGraph.totalMeausedRelativePoses
              << " percentage:  "
              << 1.0 * correspondenceGraph.refinedPoses / correspondenceGraph.totalMeausedRelativePoses
              << std::endl;
    std::cout << "ICP refined: " << icpMatrixIsBetter << " / " << totalMatrices << " = "
              << 1.0 * icpMatrixIsBetter / totalMatrices << std::endl;

    std::cout << "FROM GT ICP was better on pairs:" << std::endl;
    int space = 3;
    std::set<std::pair<int, int>> ICPrefinedGTwereBetter;
    for (int i = 0; i < pairsICPbetterGT.size(); ++i) {
        std::cout << "    from pose = " << std::setw(space) << i << ": ";
        std::sort(pairsICPbetterGT[i].begin(), pairsICPbetterGT[i].end());

        for (const auto &index: pairsICPbetterGT[i]) {
            if (index > i) {
                std::cout << std::setw(space) << index;
                ICPrefinedGTwereBetter.insert(std::make_pair(i, index));
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl << "FROM CG ICP was better on pairs:" << std::endl;

    int refinedWithError = 0;
    int refinedTruly = 0;
    int refinedTotallyCG = 0;

    for (int i = 0; i < correspondenceGraph.pairsWhereGotBetterResults.size(); ++i) {

        std::cout << "    from pose = " << std::setw(space) << i << ": ";
        std::sort(correspondenceGraph.pairsWhereGotBetterResults[i].begin(),
                  correspondenceGraph.pairsWhereGotBetterResults[i].end());

        for (const auto &index: correspondenceGraph.pairsWhereGotBetterResults[i]) {
            if (index > i) {
                if (ICPrefinedGTwereBetter.find(std::make_pair(i, index)) == ICPrefinedGTwereBetter.end()) {
                    ++refinedWithError;
                } else {
                    ++refinedTruly;
                }
                ++refinedTotallyCG;
                std::cout << std::setw(space) << index;
            }
        }
        std::cout << std::endl;
    }

    std::cout << "refined truly " << refinedTruly << " of " << refinedTotallyCG << " = "
              << 1.0 * refinedTruly / refinedTotallyCG << std::endl;
    ASSERT_EQ(refinedTruly + refinedWithError, refinedTotallyCG);

//    gdr::SmoothPointCloud smoothCloud;
//    smoothCloud.registerPointCloudFromImage(vertices);

}


TEST(testVisualization, SmoothedPointCloud360OfficeSmallEach5_60_poses) {

    std::set<int> sampledIndices;
    for (int i = 300; i < 600; i += 5) {
        sampledIndices.insert(i);
    }
    gdr::GTT::prepareDataset("/home/leoneed/Desktop/360dataset", "/home/leoneed/testGDR1/GDR/data/360_small_5",
                             sampledIndices, "");
    gdr::CorrespondenceGraph correspondenceGraph("../../data/360_small_5/rgb",
                                                 "../../data/360_small_5/depth",
                                                 517.3, 318.6,
                                                 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();

    std::vector<std::vector<int>> components;
    bool isConnected = true;
    correspondenceGraph.bfs(0, isConnected, components);
    ASSERT_TRUE(isConnected);
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();
    std::vector<gdr::VertexCG *> vertices;
    for (auto &vertex: correspondenceGraph.verticesOfCorrespondence) {
        vertices.push_back(&vertex);
    }

    std::string absolutePosesGT = "../../data/360_small_5/groundtruth_new.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePosesGT);
    std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/360_sampled/BA_60.txt";
    std::ofstream computedPoses(outputName);

    assert(posesInfo.size() == correspondenceGraph.verticesOfCorrespondence.size());
    assert(!posesInfo.empty());
    for (int i = 0; i < correspondenceGraph.verticesOfCorrespondence.size(); ++i) {
        const auto &poseBA = bundleAdjustedPoses[i];
        Sophus::SE3d movedPose = posesInfo[0].getSophusPose() * poseBA;
        const auto to = movedPose.translation();
        computedPoses.precision(std::numeric_limits<double>::max_digits10);
        computedPoses << posesInfo[i].getTimestamp() << ' ';
        for (int j = 0; j < 3; ++j) {
            computedPoses << to[j] << ' ';
        }
        auto quatComputed = movedPose.unit_quaternion();

        computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                      << quatComputed.w() << std::endl;
    }

    std::cout << "total Umeyama poses " << correspondenceGraph.totalMeausedRelativePoses << std::endl;
    std::cout << " ICP refined poses " << correspondenceGraph.refinedPoses << " percentage:  "
              << 1.0 * correspondenceGraph.refinedPoses / correspondenceGraph.totalMeausedRelativePoses << std::endl;

    gdr::SmoothPointCloud smoothCloud;
    smoothCloud.registerPointCloudFromImage(vertices);

}


TEST(testVisualization, SmoothedPointCloud360Office) {

    std::set<int> sampledIndices;
    for (int i = 0; i < 755; i += 5) {
        sampledIndices.insert(i);
    }
//    gdr::GTT::prepareDataset("/home/leoneed/Desktop/360dataset", "/home/leoneed/testGDR1/GDR/data/360_dataset_sampled", sampledIndices, "each5");
    gdr::CorrespondenceGraph correspondenceGraph("../../data/360_dataset_sampled/each5/rgb",
                                                 "../../data/360_dataset_sampled/each5/depth",
                                                 517.3, 318.6,
                                                 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    bool isConnected = true;
    std::vector<std::vector<int>> components;
    correspondenceGraph.bfs(0, isConnected, components);
    ASSERT_TRUE(isConnected);
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();
    std::vector<gdr::VertexCG *> vertices;
    for (auto &vertex: correspondenceGraph.verticesOfCorrespondence) {
        vertices.push_back(&vertex);
    }

    std::string absolutePosesGT = "../../data/360_dataset_sampled/each5/groundtruth_new.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePosesGT);
    std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/360_sampled/BA_150.txt";
    std::ofstream computedPoses(outputName);

    assert(posesInfo.size() == correspondenceGraph.verticesOfCorrespondence.size());
    assert(!posesInfo.empty());
    for (int i = 0; i < correspondenceGraph.verticesOfCorrespondence.size(); ++i) {
        const auto &poseBA = bundleAdjustedPoses[i];
        Sophus::SE3d movedPose = posesInfo[0].getSophusPose() * poseBA;
        const auto to = movedPose.translation();
        computedPoses.precision(std::numeric_limits<double>::max_digits10);
        computedPoses << posesInfo[i].getTimestamp() << ' ';
        for (int j = 0; j < 3; ++j) {
            computedPoses << to[j] << ' ';
        }
        auto quatComputed = movedPose.unit_quaternion();

        computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                      << quatComputed.w() << std::endl;
    }

    std::cout << "total Umeyama poses " << correspondenceGraph.totalMeausedRelativePoses << std::endl;
    std::cout << " ICP refined poses " << correspondenceGraph.refinedPoses << " percentage:  "
              << 1.0 * correspondenceGraph.refinedPoses / correspondenceGraph.totalMeausedRelativePoses << std::endl;

    gdr::SmoothPointCloud smoothCloud;
    smoothCloud.registerPointCloudFromImage(vertices);

}


TEST(testBundleAdjustment, BundleAdjustedUsingDepthPosesAreBetterThanAveraged) {

    gdr::CorrespondenceGraph correspondenceGraph("../../data/plantDataset_19_3/rgb",
                                                 "../../data/plantDataset_19_3/depth",
                                                 517.3,
                                                 318.6, 516.5, 255.3);
    correspondenceGraph.computeRelativePoses();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsNoRobust = correspondenceGraph.performRotationAveraging();
    std::vector<Eigen::Quaterniond> computedAbsoluteOrientationsRobust = correspondenceGraph.optimizeRotationsRobust();
    std::vector<Eigen::Vector3d> computedAbsoluteTranslationsIRLS = correspondenceGraph.optimizeAbsoluteTranslations();
    std::vector<Sophus::SE3d> bundleAdjustedPoses = correspondenceGraph.performBundleAdjustmentUsingDepth();


    for (int i = 0; i < bundleAdjustedPoses.size(); ++i) {
        ASSERT_LE(bundleAdjustedPoses[i].unit_quaternion().angularDistance(
                correspondenceGraph.verticesOfCorrespondence[i].getRotationQuat().normalized()), 1e-10);
    }
    std::string absolutePoses = "../../data/files/absolutePoses_19.txt";
    std::vector<gdr::poseInfo> posesInfo = gdr::GTT::getPoseInfoTimeTranslationOrientation(absolutePoses);

    std::vector<Eigen::Vector3d> absoluteTranslationsFromGroundTruth;

    for (int i = 0; i < posesInfo.size(); ++i) {
        absoluteTranslationsFromGroundTruth.push_back(posesInfo[i].getTranslation());
    }
    assert(posesInfo.size() == correspondenceGraph.verticesOfCorrespondence.size());
    assert(posesInfo.size() == correspondenceGraph.verticesOfCorrespondence.size());


    double errorRotRobust = 0;
    for (int i = 0; i < computedAbsoluteOrientationsRobust.size(); ++i) {
        const auto &quat = computedAbsoluteOrientationsRobust[i];
        double dErrorRobust = quat.angularDistance(
                posesInfo[0].getOrientationQuat().inverse().normalized() * posesInfo[i].getOrientationQuat());
        std::cout << i << " -- pose error robust is: " << dErrorRobust << std::endl;
        errorRotRobust += dErrorRobust;
    }


    double errorRotBA = 0;
    for (int i = 0; i < correspondenceGraph.verticesOfCorrespondence.size(); ++i) {
        auto quatBA = correspondenceGraph.verticesOfCorrespondence[i].getRotationQuat();
        double dError = quatBA.angularDistance(
                posesInfo[0].getOrientationQuat().inverse().normalized() * posesInfo[i].getOrientationQuat());
        std::cout << i << " -- pose error BA is: " << dError << std::endl;
        errorRotBA += dError;
    }


    auto zeroT = absoluteTranslationsFromGroundTruth[0];

    for (auto &translations: absoluteTranslationsFromGroundTruth) {
        translations -= zeroT;
    }

    std::cout << "_______________________VS_______________________________________" << std::endl;
    for (int i = 0; i < absoluteTranslationsFromGroundTruth.size(); ++i) {
        const auto &t = absoluteTranslationsFromGroundTruth[i];
        const auto &to = computedAbsoluteTranslationsIRLS[i];
        std::cout << i << ": \t" << t[0] << " \t" << t[1] << " \t" << t[2] << std::endl;
        std::cout << " : \t" << to[0] << " \t" << to[1] << " \t" << to[2] << std::endl;
    }

    std::cout << "______________________________________________________________" << std::endl;


    double sumErrors = 0;
    double sumErrorsSquared = 0;
    double dev = 0;

//    std::string outputName = "absolutePoses_19_BA.txt";
    std::string outputName = "/home/leoneed/Desktop/evaluate_ate_scale/b/absolutePoses_19_BA_usingDepth.txt";
    std::ofstream computedPoses(outputName);

    assert(computedAbsoluteTranslationsIRLS.size() == absoluteTranslationsFromGroundTruth.size());
    for (int i = 0; i < posesInfo.size(); ++i) {
        double currentL2Error = (absoluteTranslationsFromGroundTruth[i] - computedAbsoluteTranslationsIRLS[i]).norm();
        computedPoses.precision(std::numeric_limits<double>::max_digits10);
        std::cout << i << ":\t" << currentL2Error << std::endl;
        computedPoses << posesInfo[i].getTimestamp() << ' ';
        const auto to = bundleAdjustedPoses[i].translation();
        for (int j = 0; j < 3; ++j) {
            computedPoses << to[j] << ' ';
        }
        auto quatComputed = bundleAdjustedPoses[i].unit_quaternion();

        computedPoses << quatComputed.x() << ' ' << quatComputed.y() << ' ' << quatComputed.z() << ' '
                      << quatComputed.w() << std::endl;
        sumErrors += currentL2Error;
        sumErrorsSquared += pow(currentL2Error, 2);

    }
    double meanError = sumErrors / posesInfo.size();
    double meanSquaredError = sumErrorsSquared / posesInfo.size();


    correspondenceGraph.printConnectionsRelative(std::cout);
    std::cout << "IRLS for translations result" << std::endl;
    std::cout << "E(error) = " << meanError << std::endl;
    std::cout << "standard deviation(error) = " << meanSquaredError - pow(meanError, 2) << std::endl;
    std::cout << "______________________ROTATION REPORT______________________" << std::endl;
    std::cout << "Mean Rot angle error BA " << errorRotBA / correspondenceGraph.verticesOfCorrespondence.size()
              << std::endl;
    std::cout << "Mean Rot angle error robust " << errorRotRobust / correspondenceGraph.verticesOfCorrespondence.size()
              << std::endl;

    ASSERT_LE(errorRotBA, errorRotRobust);
    ASSERT_LE(meanError, 0.15);
}


int main(int argc, char *argv[]) {

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

