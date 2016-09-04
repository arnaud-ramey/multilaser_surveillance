#ifndef _WANDERER_H_
#define _WANDERER_H_

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
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

class LiteObstacleMap {
public:
  LiteObstacleMap() {}

  template<class Pt2>
  bool create(const std::vector<Pt2> & obstacles,
              double pix2m = .05) {
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
    _map.create(w, h);
    _map.setTo(0);
    // fill map
    for (unsigned int i = 0; i < npts; ++i) {
      _map( m2pix( obstacles[i] ) ) = 255;
    } // end for i
    return true;
  }

  bool inflate(const double inflation_radius) {
    if (_map.empty()) {
      printf("LiteObstacleMap::inflate(): empty map!\n");
      return false;
    }
    return true;
  }

  inline bool is_free(const double & x, const double & y) {
    if (x <= _xmin || x >= _xmax || y <= _ymin || y >= _ymax)
      return false; // out of bounds
    return (_map.at<uchar>(m2pix(x, y)) == 0);
  }

  template<class Pt2f>
  inline bool is_free(const Pt2f & pt) {
    return is_free(pt.x, pt.y);
  }

protected:
  template<class Pt2f>
  inline cv::Point m2pix(const Pt2f p) {
    return cv::Point(p.x, p.y);
  }
  inline cv::Point m2pix(const double & x, const double & y) {
    return cv::Point( (x - _xmin) * _m2pix,
                      (y - _ymin) * _pix2m);
  }

  double _xmin, _ymin, _xmax, _ymax, _pix2m, _m2pix;
  cv::Mat1b _map;
}; // end class LiteObstacleMap

////////////////////////////////////////////////////////////////////////////////

template<class Pt2>
class MultiLaserSurveillance {
protected:
  typedef std::vector<Pt2> Scan;
  typedef std::vector<Pt2> Map;
  typedef std::vector<Pt2> OutlierPtList;

  class Device {
  public:
    Pt2 pos; // meters
    double orien, cosorien, sinorien; // radians
    Scan _last_scan; // in map coordinates
    OutlierPtList _outlier_list;
  };

  void add_device(const Pt2& pos, const double & orien) {
    DEBUG_PRINT("add_device( (%g, %g), %g)\n", pos.x, pos.y, orien);
    Device d;
    d.pos = pos;
    d.orien = orien;
    d.cosorien = cos(orien);
    d.sinorien = sin(orien);
    _devices.push_back(d);
  }

  bool fine_tune_devices(const Map & map) {
    for (unsigned int i = 0; i < ndevices(); ++i) {
      // TODO
    } // end for i
    return false; // failure
  } // end fine_tune_devices()

  bool update_scan(unsigned int & device_idx,
                   const Scan & scan) {
    if (device_idx >= ndevices()) {
      printf("Error: device_idx %i > ndevices %i\n", device_idx, ndevices());
      return false;
    }
    Device* d = &(_devices[device_idx]);
    // find outliers
    d->_outlier_list.clear();
    unsigned int npts = scan.size();
    for (unsigned int i = 0; i < npts; ++i) {
    // convert to map frame
      double x = scan[i].x, y = scan[i].y;
      double xw = d->pos.x + (d->cosorien * x + d->sinorien * y);
      double yw = d->pos.y + (d->sinorien * x - d->cosorien * y);
      // check if in map
      if (_obstacle_map.is_free(xw, yw)) {
        Pt2 outlier;
        outlier.x = xw;
        outlier.y = yw;
        d->_outlier_list.push_back(outlier);
      }
    } // end for i
    return true;
  }

  OutlierPtList retrieve_outliers() {

  }

  Scan retrieve_global_scan() {

  }


  inline unsigned int ndevices() const {
    return _devices.size();
  }

  LiteObstacleMap _obstacle_map;
  Scan _buffer;
  std::vector<Device> _devices;
}; // end class MultiLaserSurveillance

#endif // _WANDERER_H_
