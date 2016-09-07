#ifndef _WANDERER_H_
#define _WANDERER_H_

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "timer.h"

//#define DEBUG_PRINT(...)   {}
#define DEBUG_PRINT(...)   printf(__VA_ARGS__)

/*! \brief   split a string
 * \param   str the long string
 * \param   delim the string which is the separator between words, for instance '/'
 * \param   results a pointer towards a vector of string, will contain the results
 */
inline void StringSplit(const std::string & str, const std::string & delim,
                        std::vector<std::string>* results) {
  // maggieDebug3("StringSplit(str:'%s', delim:'%s')", str.c_str(), delim.c_str());
  results->clear();
  if (str == "")
    return;
  size_t delim_pos, search_pos = 0;
  while (search_pos <= str.size() - 1) {
    delim_pos = str.find(delim, search_pos);
    //maggieDebug1("delim_pos:%i, search_pos:%i", delim_pos, search_pos);
    if (delim_pos == std::string::npos) { // no more delim
      results->push_back(str.substr(search_pos));
      return;
    }
    if (delim_pos > 0) // == 0 only happens if str starts with delim
      results->push_back(str.substr(search_pos, delim_pos - search_pos));
    search_pos = delim_pos + delim.size();
    // quit if we reached the end of the std::string or std::string empty
  }
} // end StringSplit();

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class LiteObstacleMap {
public:
  LiteObstacleMap() {}

  //////////////////////////////////////////////////////////////////////////////

  template<class Pt2>
  bool create(const std::vector<Pt2> & obstacles,
              double pix2m = 0.05) { // 5 cm per pixel
    Timer timer;
    unsigned int npts = obstacles.size();
    if (obstacles.empty()) {
      printf("LiteObstacleMap::create(): empty scan!\n");
      return false;
    }
    // find xmin, ymin, xmax, ymax
    _xmax = _xmin = obstacles.front().x;
    _ymax = _ymin = obstacles.front().y;
    for (unsigned int i = 0; i < npts; ++i) {
      if (_xmin > obstacles[i].x)
        _xmin = obstacles[i].x;
      else if (_xmax < obstacles[i].x)
        _xmax = obstacles[i].x;
      if (_ymin > obstacles[i].y)
        _ymin = obstacles[i].y;
      else if (_ymax < obstacles[i].y)
        _ymax = obstacles[i].y;
    } // end for i
    // reset map
    _pix2m = pix2m;
    _m2pix = 1./pix2m;
    int w = 1 + (_xmax - _xmin) * _m2pix, h = 1 + (_ymax - _ymin) * _m2pix;
    DEBUG_PRINT("LiteObstacleMap::create(): bounds (%g,%g)->(%g,%g), dims(%i,%i)\n",
                _xmin, _ymin, _xmax, _ymax, w, h);
    _map.create(h, w);
    _map.setTo(0);
    // fill map
    for (unsigned int i = 0; i < npts; ++i) {
      _map( m2pix( obstacles[i] ) ) = 255;
    } // end for i
    //    cv::imshow("LiteObstacleMap", _map);
    //    cv::waitKey(500);
    timer.printTime("creating LiteObstacleMap");
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! inflation_radius in meters
  bool inflate(const double & inflation_radius) {
    if (_map.empty()) {
      printf("LiteObstacleMap::inflate(): empty map!\n");
      return false;
    }
    double radpix = inflation_radius * _m2pix;
    if (radpix == 0)
      return true;
    // http://docs.opencv.org/2.4/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html
    int dilation_type = cv::MORPH_RECT;
    cv::Mat element = cv::getStructuringElement( dilation_type,
                                                 cv::Size( 2*radpix + 1, 2*radpix+1 ),
                                                 cv::Point( radpix, radpix ) );
    cv::dilate(_map, _map, element);
    //    cv::imshow("LiteObstacleMap-inflated", _map);
    //    cv::waitKey(500);
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline bool is_free(const double & x, const double & y) const {
    if (x <= _xmin || x >= _xmax || y <= _ymin || y >= _ymax)
      return false; // out of bounds
    return (_map.at<uchar>(m2pix(x, y)) == 0);
  }
  template<class Pt2f>
  inline bool is_free(const Pt2f & pt) {
    return is_free(pt.x, pt.y);
  }

  //////////////////////////////////////////////////////////////////////////////

  //protected:
  inline cv::Point m2pix(const double & x, const double & y) const {
    return cv::Point( (x - _xmin) * _m2pix,
                      (y - _ymin) * _m2pix);
  }
  template<class Pt2f>
  inline cv::Point m2pix(const Pt2f & p) const {
    return m2pix(p.x, p.y);
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  double _xmin, _ymin, _xmax, _ymax, _pix2m, _m2pix;
  cv::Mat1b _map;
}; // end class LiteObstacleMap

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

template<class Pt2>
class MultiLaserSurveillance {
public:
  typedef std::vector<Pt2> Scan;
  typedef std::vector<Pt2> Map;
  typedef std::vector<Pt2> OutlierPtList;
  enum Status {
    STATUS_MAP_NOT_DONE = 0,
    STATUS_MAP_DONE = 1
  };

  class Device {
  public:
    Device(const std::string & name,
           const Pt2 & pos,
           const double & orien) :
      _name(name), _pos(pos), _orien(orien), _map_nscans(0) {
      _cosorien = cos(orien);
      _sinorien = sin(orien);
    }

    inline void device2world(const Pt2 & in,
                             Pt2 & out) const {
      out.x = _cosorien * in.x - _sinorien * in.y + _pos.x;
      out.y = _sinorien * in.x + _cosorien * in.y + _pos.y;
    }

    inline void set_last_scan(const Scan & scan) {
      unsigned int npts = scan.size();
      _last_scan.resize(npts);
      _last_scan_timer.reset();
      // convert to map frame
      for (unsigned int i = 0; i < npts; ++i)
        device2world(scan[i], _last_scan[i]);
    }

    std::string _name; //!< device name
    Pt2 _pos; //!< meters, in world coordinates
    double _orien, _cosorien, _sinorien; //!< radians, and cached values of cos and sin
    OutlierPtList _outliers; //!< outliers detected by this device
    Timer _last_scan_timer;
    Scan _last_scan; //!< cached last scan, in map coordinates
    Scan _map_scans; //!< aggregated map scans to build the global map
    unsigned int _map_nscans; //!< number of aggregated map scans in _map_scans
  }; // end class Device

  //////////////////////////////////////////////////////////////////////////////

  // ctor
  MultiLaserSurveillance() {
    _need_recompute_outliers = true;
    _need_recompute_scan = true;
    _status = STATUS_MAP_NOT_DONE;
    _map_total_nscans = 0;
  }

  //////////////////////////////////////////////////////////////////////////////

  void add_device(const Device & d) {
    DEBUG_PRINT("add_device( (%g, %g), %g)\n", d._pos.x, d._pos.y, d._orien);
    _devices.push_back(d);
  }

  //////////////////////////////////////////////////////////////////////////////

  bool fine_tune_devices(const Map & map) {
    for (unsigned int i = 0; i < ndevices(); ++i) {
      // TODO
    } // end for i
    return false; // failure
  } // end fine_tune_devices()

  //////////////////////////////////////////////////////////////////////////////

  static const unsigned int MAP_NSCANS_PER_DEVICE = 10;

  bool update_scan(unsigned int & device_idx,
                   const Scan & scan) {
    if (device_idx >= ndevices()) {
      printf("Error: device_idx %i > ndevices %i\n", device_idx, ndevices());
      return false;
    }
    // DEBUG_PRINT("update_scan(%i)\n", device_idx);

    _need_recompute_outliers = true;
    _need_recompute_scan = true;
    Device* d = &(_devices[device_idx]);
    d->set_last_scan(scan);

    // aggregate scan if map not done
    if (_status == STATUS_MAP_NOT_DONE) {
      bool need_add = (d->_map_nscans < MAP_NSCANS_PER_DEVICE);
      if (!need_add) // nothing to do
        return true;
      std::copy(d->_last_scan.begin(), d->_last_scan.end(), std::back_inserter(d->_map_scans));
      ++ d->_map_nscans;
      ++_map_total_nscans;
      if (d->_map_nscans < MAP_NSCANS_PER_DEVICE)
        return true;
      DEBUG_PRINT("Device '%s': got the required %i scans.\n",
                  d->_name.c_str(), MAP_NSCANS_PER_DEVICE);
      // compute obstacle map if not done yet and possible
      if (_map_total_nscans == MAP_NSCANS_PER_DEVICE * ndevices())
        return recompute_map();
      return true;
    } // end if (_status == STATUS_MAP_NOT_DONE)

    // otherwise find outliers
    // status == STATUS_MAP_DONE
    // check if in map
    // DEBUG_PRINT("Device '%s': checking for outliers\n", d->_name.c_str());
    d->_outliers.clear();
    unsigned int npts = scan.size();
    for (unsigned int i = 0; i < npts; ++i) {
      Pt2 & pt = d->_last_scan[i];
      if (_obstacle_map.is_free(pt)) {
        // DEBUG_PRINT("Found outlier (%g, %g)!\n", pt.x, pt.y);
        d->_outliers.push_back(pt);
      }
    } // end for i
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

protected:
  bool recompute_outliers_if_needed() {
    if (!_need_recompute_outliers)
      return false;
    _need_recompute_outliers = false;
    // compute size
    unsigned int size = 0;
    for (unsigned int i = 0; i < ndevices(); ++i)
      size += _devices[i]._outliers.size();
    _outliers.clear();
    _outliers.reserve(size);
    for (unsigned int i = 0; i < ndevices(); ++i) {
      Device* d = &(_devices[i]);
      std::copy(d->_outliers.begin(), d->_outliers.end(), std::back_inserter(_outliers));
    } // end for i
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool recompute_scan_if_needed() {
    if (!_need_recompute_scan)
      return false;
    // DEBUG_PRINT("recompute_scan_if_needed()\n");
    _need_recompute_scan = false;
    // compute size
    unsigned int size = 0;
    for (unsigned int i = 0; i < ndevices(); ++i)
      size += _devices[i]._last_scan.size();
    _scan.clear();
    _scan.reserve(size);
    for (unsigned int i = 0; i < ndevices(); ++i) {
      Device* d = &(_devices[i]);
      std::copy(d->_last_scan.begin(), d->_last_scan.end(), std::back_inserter(_scan));
    } // end for i
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool recompute_map() {
    DEBUG_PRINT("Recomputing map...\n");
    // compute size
    unsigned int size = 0;
    for (unsigned int i = 0; i < ndevices(); ++i)
      size += _devices[i]._map_scans.size();
    std::vector<Pt2> map_scans;
    map_scans.reserve(size);
    for (unsigned int i = 0; i < ndevices(); ++i) {
      Device* d = &(_devices[i]);
      std::copy(d->_map_scans.begin(), d->_map_scans.end(), std::back_inserter(map_scans));
    } // end for i
    // compute real map
    if (!_obstacle_map.create(map_scans)
        || !_obstacle_map.inflate(0.10)) {
      printf("Map creating failed!\n");
      return false;
    }
    _status = STATUS_MAP_DONE;
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline unsigned int ndevices() const { return _devices.size(); }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  LiteObstacleMap _obstacle_map;
  Scan _outliers, _scan;
  bool _need_recompute_outliers, _need_recompute_scan;
  unsigned int _map_total_nscans;
  std::vector<Device> _devices;
  Status _status;
}; // end class MultiLaserSurveillance

#endif // _WANDERER_H_
