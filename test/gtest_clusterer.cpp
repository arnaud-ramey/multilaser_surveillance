// Bring in my package's API, which is what I'm testing
#include <multilaser_surveillance/2dclusterer.h>
#include <math.h>
// Bring in gtest
#include <gtest/gtest.h>

class Pt2 {
public:
  Pt2() : x(0), y(0) {}
  Pt2(double x_, double y_) : x(x_), y(y_) {}
  double x, y;
};

void ASSERT_PTS_EQ(const Pt2 & A, const Pt2 & B, double max_dist = 1E-2) {
  ASSERT_NEAR(A.x, B.x, max_dist) << "(" << A.x << "," << A.y
                                  << ") differ from (" << B.x << "," << B.y << ")";
  ASSERT_NEAR(A.y, B.y, max_dist) << "(" << A.x << "," << A.y
                                  << ") differ from (" << B.x << "," << B.y << ")";
}


TEST(TestSuite, empty) {
  std::vector<Pt2> pts, cluster_centers;
  std::vector<unsigned int> cluster_indices;
  cluster_indices.push_back(1);
  unsigned int nclusters = 2;
  cluster(pts, cluster_indices, nclusters);
  ASSERT_EQ(nclusters, 0);
  ASSERT_EQ(cluster_indices.size(), pts.size());
  barycenters(pts, cluster_indices, nclusters, cluster_centers);
  ASSERT_EQ(cluster_centers.size(), nclusters);
}

////////////////////////////////////////////////////////////////////////////////

void test_singleton(unsigned int npts) {
  std::vector<Pt2> pts, cluster_centers;
  for (unsigned int i = 0; i < npts; ++i)
    pts.push_back(Pt2(2, 3));
  std::vector<unsigned int> cluster_indices;
  unsigned int nclusters = 2;
  cluster(pts, cluster_indices, nclusters);
  ASSERT_EQ(nclusters, 1);
  ASSERT_EQ(cluster_indices.size(), pts.size());
  ASSERT_EQ(cluster_indices.front(), 0);
  barycenters(pts, cluster_indices, nclusters, cluster_centers);
  ASSERT_EQ(cluster_centers.size(), nclusters);
  ASSERT_PTS_EQ(cluster_centers.front(), pts.front());
}

TEST(TestSuite, singleton1)  { test_singleton(1); }
TEST(TestSuite, singleton2)  { test_singleton(2); }
TEST(TestSuite, singleton3)  { test_singleton(3); }
TEST(TestSuite, singleton10) { test_singleton(10); }

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, average) {
  std::vector<Pt2> pts, cluster_centers;
  pts.push_back(Pt2(2, 3));
  pts.push_back(Pt2(2.1, 3));
  std::vector<unsigned int> cluster_indices;
  unsigned int nclusters = 1;
  cluster(pts, cluster_indices, nclusters);
  ASSERT_EQ(nclusters, 1);
  ASSERT_EQ(cluster_indices.size(), pts.size());
  ASSERT_EQ(cluster_indices.front(), 0);
  ASSERT_EQ(cluster_indices.back(), 0);
  barycenters(pts, cluster_indices, nclusters, cluster_centers);
  ASSERT_EQ(cluster_centers.size(), nclusters);
  ASSERT_PTS_EQ(cluster_centers.front(), Pt2(2.05, 3));
}

////////////////////////////////////////////////////////////////////////////////

void test_remote(unsigned int npts) {
  std::vector<Pt2> pts, cluster_centers;
  for (unsigned int i = 0; i < npts; ++i)
    pts.push_back(Pt2(i, i));
  std::vector<unsigned int> cluster_indices;
  unsigned int nclusters = 1;
  cluster(pts, cluster_indices, nclusters);
  ASSERT_EQ(nclusters, npts);
  ASSERT_EQ(cluster_indices.size(), pts.size());
  for (unsigned int i = 0; i < npts; ++i)
    ASSERT_EQ(cluster_indices[i], i);
  barycenters(pts, cluster_indices, nclusters, cluster_centers);
  ASSERT_EQ(cluster_centers.size(), nclusters);
  for (unsigned int i = 0; i < npts; ++i)
    ASSERT_PTS_EQ(cluster_centers[i], pts[i]);
}

TEST(TestSuite, remote1) { test_remote(1);}
TEST(TestSuite, remote2) { test_remote(2);}
TEST(TestSuite, remote3) { test_remote(3);}
TEST(TestSuite, remote10) { test_remote(10);}

////////////////////////////////////////////////////////////////////////////////

void test_circles(unsigned int ncircles, unsigned int nptspercircle = 10) {
  std::vector<Pt2> pts, cluster_centers;
  double radius=.1;
  for (unsigned int pti = 0; pti < nptspercircle; ++pti) {
    double theta = 2. * M_PI * pti / nptspercircle;
    for (unsigned int ci = 0; ci < ncircles; ++ci) {
      pts.push_back(Pt2(ci+radius*cos(theta), ci+radius*sin(theta)));
    } // end for ci
  } // end for pti
  std::vector<unsigned int> cluster_indices;
  unsigned int nclusters = 1;//, npts = pts.size();
  cluster(pts, cluster_indices, nclusters);
  ASSERT_EQ(nclusters, ncircles);
  ASSERT_EQ(cluster_indices.size(), pts.size());
  //for (unsigned int i = 0; i < npts; ++i)
    //ASSERT_EQ(cluster_indices[i], i % nptspercircle);
  barycenters(pts, cluster_indices, nclusters, cluster_centers);
  ASSERT_EQ(cluster_centers.size(), nclusters);
  for (unsigned int i = 0; i < nclusters; ++i)
    ASSERT_PTS_EQ(cluster_centers[i], Pt2(i, i));
}

TEST(TestSuite, circles1) { test_circles(1); }
TEST(TestSuite, circles2) { test_circles(2); }
TEST(TestSuite, circles3) { test_circles(3); }
TEST(TestSuite, circles10) { test_circles(10); }

////////////////////////////////////////////////////////////////////////////////

void test_circle_center(double xc, double yc, double radius,
                        double nptspercircle, double npts/*, double noise = 0*/) {
  std::vector<Pt2> pts;
  for (unsigned int pti = 0; pti < npts; ++pti) {
    double theta = 2. * M_PI * pti / nptspercircle;
    pts.push_back(Pt2(xc+radius*cos(theta),
                      yc+radius*sin(theta)));
  } // end for pti
  Pt2 ans;
  bool ok = best_fit_circle(pts, radius, ans);
  ASSERT_TRUE(ok);
  ASSERT_PTS_EQ(ans, Pt2(xc, yc), 1E-1);
}

TEST(TestSuite, fullcircle1) { test_circle_center(0,  0, .1,  10, 10); }
TEST(TestSuite, fullcircle2) { test_circle_center(1,  2, .1,  10, 10); }
TEST(TestSuite, fullcircle3) { test_circle_center(-3, 4, 10,  100, 100); }
TEST(TestSuite, fullcircle4) { test_circle_center(5, -6, 10,  100, 100); }
TEST(TestSuite, halfcircle1) { test_circle_center(0,  0, .1,  10, 5); }
TEST(TestSuite, halfcircle2) { test_circle_center(1,  2, .1,  10, 5); }
TEST(TestSuite, halfcircle3) { test_circle_center(-3, 4, 10,  100, 50); }
TEST(TestSuite, halfcircle4) { test_circle_center(5, -6, 10,  100, 25); }

////////////////////////////////////////////////////////////////////////////////

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
