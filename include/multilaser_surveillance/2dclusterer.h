#ifndef CLUSTERER_H
#define CLUSTERER_H

#include <vector>
#include <map>
#include <sstream>
#include <stdio.h>

//! from https://avdongre.wordpress.com/2011/12/06/disjoint-set-pts-structure-c/
class DisjointSets {
public:
  struct DisjointSet {
    size_t parent;
    unsigned rank;
    DisjointSet(size_t i) : parent(i), rank(0) { }
  };

  DisjointSets(size_t n){
    forest.reserve(n);
    for (size_t i=0; i<n; i++)
      forest.push_back(DisjointSet(i));
    was_changed = true;
  }

  size_t find(size_t i){
    if (forest[i].parent == i)
      return i;
    size_t old_parent = forest[i].parent;
    forest[i].parent = find(forest[i].parent);
    was_changed = (old_parent != forest[i].parent);
    return forest[i].parent;
  }

  void merge(size_t i, size_t j) {
    size_t root_i = find(i);
    size_t root_j = find(j);
    if (root_i == root_j)
      return;
    was_changed = true;
    if (forest[root_i].rank < forest[root_j].rank)
      forest[root_i].parent = root_j;
    else if (forest[root_i].rank > forest[root_j].rank)
      forest[root_j].parent = root_i;
    else {
      forest[root_i].parent = root_j;
      forest[root_j].rank += 1;
    }
  }
  void reset_was_changed() { was_changed = false; }

  std::vector<DisjointSet> forest;
  bool was_changed;
}; // end class DisjointSets

template<class Vec>
std::string vec2str(const Vec & v) {
  std::ostringstream out;
  for (unsigned int i = 0; i < v.size(); ++i)
    out << v[i] << ", ";
  return out.str();
}

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
  nclusters = 0;
  if (pts.size() == 0) {
    // printf("cluster(): clustering an empty cloud.\n");
    cluster_indices.clear();
    return true; // no error needed
  }
  unsigned int npts = pts.size();
  double cluster_tolerance_sq = cluster_tolerance * cluster_tolerance;
  // compute neighbors matrix - careful, defined if j > i
  std::vector< std::vector<bool> > neighbors ( npts, std::vector<bool> ( npts, false ) );
  for (unsigned int i = 0; i < npts; ++i) {
    for (unsigned int j = i+1; j < npts; ++j) {
      neighbors[i][j] = are_neighbors(pts[i], pts[j], cluster_tolerance_sq);
    } // end for j
  } // end for i

  // print matrix
  //for (unsigned int i = 0; i < npts; ++i) {
  //  for (unsigned int j = 0; j < npts; ++j) printf("%c", neighbors[i][j] ? 'X' : '.');
  //  printf("\n");
  //} // end for i

  DisjointSets set(npts);
  //unsigned int niters = 0;
  while (set.was_changed) {
    //printf("set iteration:%i\n", niters++);
    set.reset_was_changed();
    for (unsigned int i = 0; i < npts; ++i) {
      for (unsigned int j = i+1; j < npts; ++j) {
        if (neighbors[i][j]) {
          //printf("Merging %i and %i\n", j, i);
          set.merge(j, i);
        }
      } // end for j
    } // end for i
  } // end while (set.was_changed)

  // convert all the different values of parents into a [0,1,..,nclusters] list
  std::map<size_t, unsigned int> parent2cluster;
  cluster_indices.resize(npts);
  for (unsigned int i = 0; i < npts; ++i) {
    size_t parent = set.forest[i].parent;
    if (parent2cluster.find(parent) == parent2cluster.end())
      parent2cluster.insert(std::make_pair(parent, parent2cluster.size()));
    cluster_indices[i] = parent2cluster[parent];
  }

  nclusters = parent2cluster.size(); // parent is equal to (ncluster-1)
  // print cluster_indices
  //printf("cluster_indices:%s\n", vec2str(cluster_indices).c_str());
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
}

#endif // CLUSTERER_H
