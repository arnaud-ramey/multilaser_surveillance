#ifndef _WANDERER_H_
#define _WANDERER_H_

#include <vector>
#include <string>
#include <math.h>
#include <multilaser_surveillance/timer.h>
//#include "lite_obstacle_map.h"

//#define DEBUG_PRINT(...)   {}
#define DEBUG_PRINT(...)   printf(__VA_ARGS__)
#define DEFAULT_CELL2M            .05 // 1 cell = 5 cm
#define DEFAULT_INFLATION_RADIUS  .10 // 10 cm

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

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

template<class Pt2, class ObstacleMap>
class MapBuilderWatcher {
public:
  typedef std::vector<Pt2> Scan;
  typedef std::vector<Pt2> OutlierPtList;
  typedef std::vector<Pt2> Map;
  enum Mode {
    MODE_BUILD_MAP = 0,
    MODE_SURVEILLANCE = 1
  };
  static const unsigned int MAP_NSCANS_PER_DEVICE = 1000;

  //////////////////////////////////////////////////////////////////////////////

  class SurveillanceDevice {
  public:
    SurveillanceDevice(const std::string & name,
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
    unsigned int _map_nscans; //!< number of scans aggregated in global map
  }; // end class SurveillanceDevice

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  MapBuilderWatcher() {
    _mode = MODE_BUILD_MAP;
    _map_total_nscans = 0;
    _need_recompute_scan = true;
    _need_recompute_outliers = true;
  }

  //////////////////////////////////////////////////////////////////////////////

  void add_device(const SurveillanceDevice & d) {
    DEBUG_PRINT("add_device( (%g, %g), %g)\n", d._pos.x, d._pos.y, d._orien);
    _devices.push_back(d);
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
      SurveillanceDevice* d = &(_devices[i]);
      std::copy(d->_last_scan.begin(), d->_last_scan.end(), std::back_inserter(_scan));
    } // end for i
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline unsigned int ndevices() const { return _devices.size(); }

  //////////////////////////////////////////////////////////////////////////////

  bool fine_tune_devices(const Map & map) {
    for (unsigned int i = 0; i < ndevices(); ++i) {
      // TODO
    } // end for i
    return false; // failure
  } // end fine_tune_devices()

  //////////////////////////////////////////////////////////////////////////////

  bool update_scan(unsigned int & device_idx,
                   const Scan & scan) {
    if (device_idx >= ndevices()) {
      printf("Error: device_idx %i > ndevices %i\n", device_idx, ndevices());
      return false;
    }
    // DEBUG_PRINT("update_scan(%i)\n", device_idx);
    _need_recompute_scan = true;
    SurveillanceDevice* d = &(_devices[device_idx]);
    d->set_last_scan(scan);

    // aggregate scan if map not done
    if (_mode == MODE_BUILD_MAP) {
      // compute real map
      unsigned int npts = d->_last_scan.size();
      for (unsigned int i = 0; i < npts; ++i) {
        if (!add_obstacle(_obstacle_map, d->_last_scan[i])) {
          printf("Map creating failed!\n");
          return false;
        }
      }
      ++ d->_map_nscans;
      ++_map_total_nscans;
      if (d->_map_nscans % 25 == 1)
        DEBUG_PRINT("Device '%s': got %i scans.\n", d->_name.c_str(), d->_map_nscans);
      return true;
    } // end if (_mode == MODE_BUILD_MAP)

    // otherwise find outliers
    // status == MODE_SURVEILLANCE
    // check if in map
    // DEBUG_PRINT("Device '%s': checking for outliers\n", d->_name.c_str());
    _need_recompute_outliers = true;
    d->_outliers.clear();
    unsigned int npts = scan.size();
    for (unsigned int i = 0; i < npts; ++i) {
      Pt2 & pt = d->_last_scan[i];
      if (!is_occupied(_obstacle_map, pt)) {
        // DEBUG_PRINT("Found outlier (%g, %g)!\n", pt.x, pt.y);
        d->_outliers.push_back(pt);
      }
    } // end for i
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

protected:
  virtual bool is_occupied(const ObstacleMap & map, const Pt2 & pt) = 0;
  virtual bool add_obstacle(ObstacleMap & map, const Pt2 & pt) = 0;

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
    _outlier_laser_ids.clear();
    _outlier_laser_ids.reserve(size);
    for (unsigned int i = 0; i < ndevices(); ++i) {
      SurveillanceDevice* d = &(_devices[i]);
      std::copy(d->_outliers.begin(), d->_outliers.end(), std::back_inserter(_outliers));
      _outlier_laser_ids.insert(_outlier_laser_ids.end(), d->_outliers.size(), i);
    } // end for i
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  ObstacleMap _obstacle_map;
  Mode _mode;
  std::vector<SurveillanceDevice> _devices;
  unsigned int _map_total_nscans;
  bool _need_recompute_scan;
  Scan _outliers, _scan;
  //! For each outlier, the ID of the laser it comes from.
  //! Should be unsigned short really,
  //! but must be float for compatibility with sensor_msgs/ChannelFloat32
  std::vector<float> _outlier_laser_ids;
  bool _need_recompute_outliers;
}; // end class MapBuilderWatcher

#endif // _WANDERER_H_
