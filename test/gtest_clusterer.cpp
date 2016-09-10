// Bring in my package's API, which is what I'm testing
#include <multilaser_surveillance/2dclusterer.h>
// Bring in gtest
#include <gtest/gtest.h>

class Pt2 {
public:
  Pt2() : x(0), y(0) {}
  Pt2(double x_, double y_) : x(x_), y(y_) {}
  double x, y;
};

void EXPECT_PTS_EQ(const Pt2 & A, const Pt2 & B) {
  EXPECT_NEAR(A.x, B.x, 1E-2);
  EXPECT_NEAR(A.y, B.y, 1E-2);
}


TEST(TestSuite, empty) {
  std::vector<Pt2> data, cluster_centers;
  std::vector<unsigned int> cluster_indices;
  cluster_indices.push_back(1);
  unsigned int nclusters = 2;
  cluster(data, cluster_indices, nclusters);
  EXPECT_EQ(nclusters, 0);
  EXPECT_EQ(cluster_indices.size(), data.size());
  barycenters(data, cluster_indices, nclusters, cluster_centers);
  EXPECT_EQ(cluster_centers.size(), nclusters);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, singleton) {
  for (unsigned int npts = 1; npts < 10; ++npts) {
    std::vector<Pt2> data, cluster_centers;
    for (unsigned int i = 0; i < npts; ++i)
      data.push_back(Pt2(2, 3));
    std::vector<unsigned int> cluster_indices;
    unsigned int nclusters = 2;
    cluster(data, cluster_indices, nclusters);
    EXPECT_EQ(nclusters, 1);
    EXPECT_EQ(cluster_indices.size(), data.size());
    EXPECT_EQ(cluster_indices.front(), 0);
    barycenters(data, cluster_indices, nclusters, cluster_centers);
    EXPECT_EQ(cluster_centers.size(), nclusters);
    EXPECT_PTS_EQ(cluster_centers.front(), data.front());
  } // end for npts
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, average) {
  std::vector<Pt2> data, cluster_centers;
  data.push_back(Pt2(2, 3));
  data.push_back(Pt2(2.1, 3));
  std::vector<unsigned int> cluster_indices;
  unsigned int nclusters = 1;
  cluster(data, cluster_indices, nclusters);
  EXPECT_EQ(nclusters, 1);
  EXPECT_EQ(cluster_indices.size(), data.size());
  EXPECT_EQ(cluster_indices.front(), 0);
  EXPECT_EQ(cluster_indices.back(), 0);
  barycenters(data, cluster_indices, nclusters, cluster_centers);
  EXPECT_EQ(cluster_centers.size(), nclusters);
  EXPECT_PTS_EQ(cluster_centers.front(), Pt2(2.05, 3));
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, remote) {
  std::vector<Pt2> data, cluster_centers;
  data.push_back(Pt2(2, 3));
  data.push_back(Pt2(4, 3));
  std::vector<unsigned int> cluster_indices;
  unsigned int nclusters = 1;
  cluster(data, cluster_indices, nclusters);
  EXPECT_EQ(nclusters, 2);
  EXPECT_EQ(cluster_indices.size(), data.size());
  EXPECT_EQ(cluster_indices.front(), 0);
  EXPECT_EQ(cluster_indices.back(), 1);
  barycenters(data, cluster_indices, nclusters, cluster_centers);
  EXPECT_EQ(cluster_centers.size(), nclusters);
  EXPECT_PTS_EQ(cluster_centers.front(), Pt2(2, 3));
  EXPECT_PTS_EQ(cluster_centers.back(), Pt2(4, 3));
}

////////////////////////////////////////////////////////////////////////////////

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
