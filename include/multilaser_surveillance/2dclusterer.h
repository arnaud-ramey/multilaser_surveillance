#ifndef CLUSTERER_H
#define CLUSTERER_H

#include <vector>
#include <stdio.h>

template<class Pt2>
static inline double distsq(const Pt2 & A, const Pt2 & B) {
  double dx = (A.x - B.x), dy = (A.y - B.y);
  return dx * dx + dy * dy;
}

//////////////////////////////////////////////////////////////////////////////

/*!
   * Find the main cluster in a set of points.
   * \param data
   *    the input cloud
   * \param cluster_tolerance
   *    the spatial cluster tolerance as a measure in the L2 Euclidean space, in meters.
   *    A reasonable value would be 0.1f (10 cm).
   * \return
   *   true if success
   */
template<class Pt2>
static bool cluster(const std::vector<Pt2> & data,
                    std::vector<unsigned int> & cluster_indices,
                    unsigned int & nclusters,
                    const double & cluster_tolerance = 0.1f) {
  nclusters = 0;
  if (data.size() == 0) {
    printf("cluster(): clustering an empty cloud.\n");
    cluster_indices.clear();
    return true; // no error needed
  }
  unsigned int npts = data.size();
  cluster_indices.resize(npts, 0);
  double cluster_tolerance_sq = cluster_tolerance * cluster_tolerance;
  for (unsigned int i = 0; i < npts; ++i) {
    bool was_added = false;
    for (unsigned int j = 0; j < i; ++j) {
      if (distsq(data[i], data[j]) > cluster_tolerance_sq)
        continue;
      cluster_indices[i] = cluster_indices[j];
      was_added = true;
      break;
    } // end for ci
    // if was not added, create new cluster with data[pi]
    if (was_added)
      continue;
    cluster_indices[i] = nclusters;
    ++nclusters;
  } // end for pi

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
static bool barycenters(const std::vector<Pt2> & data,
                        const std::vector<unsigned int> & cluster_indices,
                        const unsigned int & nclusters,
                        std::vector<Pt2> & cluster_centers) {
  if (nclusters == 0) {
    printf("barycenters(): empty cloud.\n");
    cluster_centers.clear();
    return true;
  }
  cluster_centers.resize(nclusters, Pt2());
  std::vector<unsigned int> cluster_sizes(nclusters, 0);
  unsigned int npts = data.size();
  for (unsigned int pi = 0; pi < npts; ++pi) {
    unsigned int cluster_idx = cluster_indices[pi];
    if (cluster_idx >= nclusters) {
      printf("Incorrect cluster index %i, shoudl be in [O, %i]\n",
             cluster_idx, nclusters-1);
      continue;
    }
    add_to(cluster_centers[ cluster_idx ], data[pi]);
    ++cluster_sizes[ cluster_idx ];
  } // end for pi
  // normalize
  for (unsigned int ci = 0; ci < nclusters; ++ci)
    mult_by(cluster_centers[ci], 1. / cluster_sizes[ci]);
  return true;
}

#endif // CLUSTERER_H
