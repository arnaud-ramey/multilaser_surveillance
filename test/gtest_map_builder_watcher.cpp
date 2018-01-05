// Bring in my package's API, which is what I'm testing
#include <multilaser_surveillance/map_builder_watcher.h>
// Bring in gtest
#include <gtest/gtest.h>

static const double RAD2DEG = 57.2957795130823208768;
static const double DEG2RAD = 0.01745329251994329577;
class Pt2 {
public:
  Pt2() : x(0), y(0) {}
  Pt2(double x_, double y_) : x(x_), y(y_) {}
  double x, y;
};
typedef MapBuilderWatcher<Pt2> MBW;

void ASSERT_PTS_NEAR(const Pt2 & A, const Pt2 & B, double max_dist = 1E-2) {
  ASSERT_NEAR(A.x, B.x, max_dist) << "(" << A.x << "," << A.y
                                  << ") differ from (" << B.x << "," << B.y << ")";
  ASSERT_NEAR(A.y, B.y, max_dist) << "(" << A.x << "," << A.y
                                  << ") differ from (" << B.x << "," << B.y << ")";
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, LiteObstacleMapTest) {
  LiteObstacleMap obs;
  // wrong signs
  ASSERT_FALSE(obs.create(-10, 10, -10, 10, .05, .10));
  // incorrect pix2m
  ASSERT_FALSE(obs.create(-10, -10, 10, 10, 25, .10));
  // correct values
  ASSERT_TRUE(obs.create(-10, -10, 10, 10, .5, .10));
}



////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, SurveillanceDevice) {
  for (int x = -5; x < 5; ++x) {
    for (int y = -5; y < 5; ++y) {
      Pt2 t(x, y), out;
      MBW::SurveillanceDevice device("d0", t, DEG2RAD * 45);
      device.device2world(Pt2(0, 0), out);
      ASSERT_PTS_NEAR(out, t);
      for (int rad = 0; rad < 10; ++rad) {
        device.device2world(Pt2(rad, 0), out);
        ASSERT_PTS_NEAR(out, Pt2(t.x + .5 * rad * sqrt(2), t.y + .5 * rad * sqrt(2)));
        device.device2world(Pt2(0, rad), out);
        ASSERT_PTS_NEAR(out, Pt2(t.x - .5 * rad * sqrt(2), t.y + .5 * rad * sqrt(2)));
      } // end for rad
    } // end for y
  } // end for x
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty) {
  ASSERT_NO_FATAL_FAILURE( MBW mbw; );
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, update_no_device) {
  MBW mbw;
  ASSERT_EQ(MBW::MODE_SURVEILLANCE, mbw.get_mode());
  ASSERT_EQ(0, mbw.get_outliers().size());
  ASSERT_EQ(0, mbw.get_obstacle_map().get_occupied_cells());
  MBW::Scan scan;
  for (unsigned int i = 0; i < 10; ++i) {
    ASSERT_NO_FATAL_FAILURE();
    ASSERT_FALSE(mbw.update_scan(i, scan));
  }
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, mode_change) {
  MBW mbw;
  MBW::SurveillanceDevice d0("d0", Pt2(0,0), 0);
  mbw.add_device(d0);
  mbw.set_mode(MBW::MODE_AUTO_BUILD_MAP);
  ASSERT_EQ(MBW::MODE_AUTO_BUILD_MAP, mbw.get_mode());
  double t0 = 3;
  mbw.set_auto_mode_timeout(t0);
  double t = mbw.get_auto_mode_timeout();
  ASSERT_EQ(t0, t);
  for (int i = 0; i < t+1; ++i) {
    sleep(1);
    MBW::Scan scan;
    ASSERT_TRUE(mbw.update_scan(0, scan));
  }
  ASSERT_EQ(MBW::MODE_SURVEILLANCE, mbw.get_mode());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, save_load) {
  MBW mbw;
  mbw.add_device( MBW::SurveillanceDevice ("d", Pt2(0,0), 0));
  mbw.set_mode(MBW::MODE_BUILD_MAP);
  ASSERT_TRUE(mbw.create_map(-20, -20, 20, 20));
  MBW::Scan scan;
  scan.push_back(Pt2(1, 0));
  ASSERT_TRUE(mbw.update_scan(0, scan));
  ASSERT_EQ(1, mbw.get_obstacle_map().get_occupied_cells());
  ASSERT_TRUE(mbw.get_obstacle_map().save("/tmp/gtest_mbw"));
  LiteObstacleMap map;
  ASSERT_TRUE(map.load("/tmp/gtest_mbw"));
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, simple_point) {
  for (int ntries = 0; ntries < 10; ++ntries) {
    Pt2 t(-5  + rand()%10, -5  + rand()%10);
    double infr = 1E-2 * (10 + rand()%100), pix2m = .01;
    MBW::SurveillanceDevice d0("d0", t, 0);
    MBW mbw;
    mbw.add_device(d0);
    mbw.set_mode(MBW::MODE_AUTO_BUILD_MAP);
    mbw.set_auto_mode_timeout(.1); // 100 ms
    // create map around a single point
    ASSERT_TRUE(mbw.create_map(-20, -20, 20, 20, pix2m, infr));
    MBW::Scan scan, outliers;
    scan.push_back(Pt2(1, 1));
    ASSERT_TRUE(mbw.update_scan(0, scan));
    ASSERT_EQ(1, mbw.get_obstacle_map().get_occupied_cells());
    // square around (1, 1)
    int exp_cells = 1E4 * pow(2 * infr, 2) + 400 * infr + 1;
    ASSERT_NEAR(exp_cells, mbw.get_obstacle_map().get_occupied_inflated_cells(), 1);
    // now switch mode
    usleep(200E3); // 110 ms
    mbw.check_auto_mode();
    ASSERT_EQ(MBW::MODE_SURVEILLANCE, mbw.get_mode());
    // create a scan with an inlier and an outlier
    Pt2 inlier1(1, 1);
    Pt2 inlier2(1+infr/2, 1+infr/2);
    Pt2 outlier1(1 + 2* pix2m + 2* infr, 1),
        outlier1w (t.x + outlier1.x, t.y + outlier1.y);
    ASSERT_TRUE( mbw.get_obstacle_map().is_occupied(t.x+inlier1.x,
                                                    t.y+inlier1.y));
    ASSERT_TRUE( mbw.get_obstacle_map().is_occupied(t.x+inlier2.x,
                                                    t.y+inlier2.y));
    ASSERT_FALSE(mbw.get_obstacle_map().is_occupied(t.x+outlier1.x,
                                                    t.y+outlier1.y));
    scan.push_back( inlier1);
    scan.push_back( inlier2);
    scan.push_back(outlier1);
    ASSERT_TRUE(mbw.update_scan(0, scan));
    // now check results
    ASSERT_EQ(1, mbw.noutliers());
    outliers = mbw.get_outliers();
    ASSERT_EQ(1, outliers.size());
    ASSERT_PTS_NEAR(outlier1w, outliers.front());
  } // end for ntries
} // end TEST

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, simple_circle) {
  unsigned int ndevices = 3;
  for (int ntries = 0; ntries < 10; ++ntries) {
    Pt2 t(-5  + rand()%10, -5  + rand()%10);
    double infr = .1, pix2m = .01;
    MBW mbw;
    ASSERT_TRUE(mbw.create_map(-15, -15, 15, 15, pix2m, infr));// init map
    for (unsigned int i = 0; i < ndevices; ++i) // create devices
      mbw.add_device( MBW::SurveillanceDevice ("d", t, 360. * i * DEG2RAD / ndevices) );
    mbw.set_mode(MBW::MODE_AUTO_BUILD_MAP);
    mbw.set_auto_mode_timeout(1); // 1000 ms
    // half circle scan
    MBW::Scan scan;
    double radius = 1 + drand48(), fov = 360. / ndevices;
    for (int deg = -fov/2; deg <= fov/2; ++deg) {
      scan.push_back(Pt2(radius * cos(deg * DEG2RAD), radius * sin(deg * DEG2RAD)));
      //printf("Adding (%g, %g)\n", scan.back().x, scan.back().y);
    } // end for deg
    for (unsigned int i = 0; i < ndevices; ++i)
      ASSERT_TRUE(mbw.update_scan(i, scan));
    ASSERT_TRUE(mbw.get_obstacle_map().save("/tmp/gtest_mbw"));
    // square around (1, 1)
    int exp_cells = 2. * M_PI * radius / pix2m;
    ASSERT_NEAR(exp_cells, mbw.get_obstacle_map().get_occupied_cells(), 1);
    double surface = M_PI * (pow(radius + infr, 2) - pow(radius - infr, 2));
    int exp_inflated_cells = surface / pix2m;
    ASSERT_NEAR(exp_inflated_cells, mbw.get_obstacle_map().get_occupied_inflated_cells(), 1);
    // now switch mode
    usleep(1100E3); // 110 ms
    mbw.check_auto_mode();
    ASSERT_EQ(MBW::MODE_SURVEILLANCE, mbw.get_mode());
  } // end for ntries
} // end TEST

////////////////////////////////////////////////////////////////////////////////

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
