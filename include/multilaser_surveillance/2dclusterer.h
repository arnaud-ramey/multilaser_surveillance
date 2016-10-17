#ifndef CLUSTERER_H
#define CLUSTERER_H

#include <vector>
#include <map>
#include <sstream>
#include <stdio.h>
#include <math.h>
#include "lmmin.h"
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>


//////////////////////////////////////////////////////////////////////////////

template<class Pt2>
static inline bool are_neighbors(const Pt2 & A, const Pt2 & B,
                                 const double & cluster_tolerance_sq) {
  double dx = (A.x - B.x), dx2 = dx * dx;
  if (dx2 > cluster_tolerance_sq)
    return false;
  double dy = (A.y - B.y);
  return (dx2 + dy * dy) <= cluster_tolerance_sq;
}

//////////////////////////////////////////////////////////////////////////////

/*!
   * Find the main cluster in a set of points.
   * \param pts
   *    the input cloud
   * \param cluster_tolerance
   *    the spatial cluster tolerance as a measure in the L2 Euclidean space, in meters.
   *    A reasonable value would be 0.1f (10 cm).
   * \return
   *   true if success
   */
template<class Pt2>
static bool cluster(const std::vector<Pt2> & pts,
                    std::vector<unsigned int> & cluster_indices,
                    unsigned int & nclusters,
                    const double & cluster_tolerance = 0.1f) {
  // http://www.boost.org/doc/libs/1_61_0/libs/graph/example/connected_components.cpp
  boost::adjacency_list <boost::vecS, boost::vecS, boost::undirectedS> G;
  unsigned int npts = pts.size();
  double cluster_tolerance_sq = cluster_tolerance * cluster_tolerance;
  for (unsigned int i = 0; i < npts; ++i) {
    for (unsigned int j = i; j < npts; ++j) {
      if (are_neighbors(pts[i], pts[j], cluster_tolerance_sq))
        boost::add_edge(i, j, G);
    } // end for j
  } // end for i
  cluster_indices.resize(npts);
  nclusters = boost::connected_components(G, &cluster_indices[0]);
  return true; // success
} // end cluster()

//////////////////////////////////////////////////////////////////////////////

template<class Pt2>
inline void add_to(Pt2 & A, const Pt2 & B) {
  A.x += B.x;
  A.y += B.y;
}

template<class Pt2>
inline void mult_by(Pt2 & A, const double & alpha) {
  A.x *= alpha;
  A.y *= alpha;
}

template<class Pt2>
Pt2 Pt2ctor(const double & x, const double & y) {
  Pt2 ans;
  ans.x = x;
  ans.y = y;
  return ans;
}

////////////////////////////////////////////////////////////////////////////////

template<class Pt2>
static bool barycenters(const std::vector<Pt2> & pts,
                        const std::vector<unsigned int> & cluster_indices,
                        const unsigned int & nclusters,
                        std::vector<Pt2> & cluster_centers) {
  cluster_centers.clear();
  if (nclusters == 0) {
    //printf("barycenters(): empty cloud.\n");
    return true;
  }
  cluster_centers.resize(nclusters, Pt2ctor<Pt2>(0., 0.));
  std::vector<unsigned int> cluster_sizes(nclusters, 0);
  unsigned int npts = pts.size();
  for (unsigned int pi = 0; pi < npts; ++pi) {
    unsigned int cluster_idx = cluster_indices[pi];
    if (cluster_idx >= nclusters) {
      printf("Incorrect cluster index %i, shoudl be in [O, %i]\n",
             cluster_idx, nclusters-1);
      continue;
    }
    add_to(cluster_centers[ cluster_idx ], pts[pi]);
    ++cluster_sizes[ cluster_idx ];
  } // end for pi
  // normalize
  for (unsigned int ci = 0; ci < nclusters; ++ci)
    mult_by(cluster_centers[ci], 1. / cluster_sizes[ci]);
  return true;
} // end barycenters()

////////////////////////////////////////////////////////////////////////////////

template<class Pt2>
double dist_sq(const double* A, const Pt2& B) {
  double dx = A[0] - B.x, dy = A[1] - B.y;
  return dx * dx + dy * dy;
}

template<class Pt2>
struct LMFitData {
  const std::vector<Pt2>* pts;
  double radiussq;
};

// http://apps.jcns.fz-juelich.de/doku/sc/lmfit:nonlinear-equations-example
template<class Pt2>
void best_fit_fn(const double *center, const int resultsize,
                 const void *data, double *res, int */*info*/ ) {
  LMFitData<Pt2>* lmdata = (LMFitData<Pt2>*) data;
  for (int i = 0; i < resultsize; ++i)
    res[i] = fabs(lmdata->radiussq - dist_sq<Pt2>(center, lmdata->pts->at(i) ));
}

template<class Pt2>
bool best_fit_circle(const std::vector<Pt2> & pts,
                     const double & radius,
                     Pt2 & ans) {
  if (pts.size() < 2) {
    printf( "best_fit_circle(): need at least 2 points, got %i\n", (int) pts.size());
    return false;
  }
  const int centersize = 2, npts = pts.size(); /* dimension of the problem */
  double center[2]; /* parameter vector center=(x,y) */
  // take barycenter as first guess
  center[0] = pts.front().x;
  center[1] = pts.front().y;
  for (int i = 1; i < npts; ++i) {
    center[0] += pts[i].x;
    center[1] += pts[i].y;
  } // end for var
  center[0] *= 1. / npts;
  center[1] *= 1. / npts;

  // call lmfit
  LMFitData<Pt2> lmdata;
  lmdata.pts = &pts;
  lmdata.radiussq = radius * radius;
  lm_control_struct control = lm_control_double;
  lm_status_struct  status;
  //control.verbosity  = 31;
  control.verbosity  = 0;
  lmmin( centersize, center, npts, &lmdata, best_fit_fn<Pt2>, &control, &status );

  /* print results */
  //  printf( "Results: status after %d function evaluations: Error:%g, '%s'\n",
  //          status.nfev, status.fnorm, lm_infmsg[status.outcome]);
  if (status.outcome > 3) {
    printf( "best_fit_circle(): lmmin() returned an error after %d function evaluations: '%s'\n",
            status.nfev, lm_infmsg[status.outcome]);
    return false;
  }
  ans.x = center[0];
  ans.y = center[1];
  return true;
} // en best_fit_circle()

////////////////////////////////////////////////////////////////////////////////

template<class Pt2>
static bool best_fit_circles(const std::vector<Pt2> & pts,
                             const std::vector<unsigned int> & cluster_indices,
                             const unsigned int & nclusters,
                             const double & radius,
                             std::vector<Pt2> & cluster_centers) {
  cluster_centers.clear();
  if (nclusters == 0) {
    //printf("barycenters(): empty cloud.\n");
    return true;
  }
  std::vector<Pt2> cluster;
  cluster_centers.reserve(nclusters);
  unsigned int npts = pts.size();
  for (unsigned int i = 0; i < nclusters; ++i) {
    cluster.clear();
    for (unsigned int j = 0; j < npts; ++j) {
      if (cluster_indices[j] == i)
        cluster.push_back(pts[j]);
    } // end for j
    Pt2 center;
    if (!best_fit_circle(cluster, radius, center)) {
      printf( "best_fit_circle() returned an error.\n");
      continue;
    }
    cluster_centers.push_back(center);
  } // end for i
  return true;
} // end barycenters()


#endif // CLUSTERER_H
