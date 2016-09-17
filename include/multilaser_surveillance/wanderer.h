#ifndef _WANDERER_H_
#define _WANDERER_H_

#include <vector>
#include <fstream>      // std::ifstream, std::ofstream
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <multilaser_surveillance/timer.h>

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

#define DEFAULT_PIX2M             .05 // 1 pixel = 5 cm
#define DEFAULT_INFLATION_RADIUS  .10 // 10 cm

class LiteObstacleMap {
public:
  LiteObstacleMap() {
    create(-10, 10, -10, 10);
  }

  //////////////////////////////////////////////////////////////////////////////

  bool save(const std::string & prefix) const {
    printf("LiteObstacleMap::save('%s')\n", prefix.c_str());
    std::string img_filename = prefix + ".png";
    if (!cv::imwrite(img_filename, _map_as_img)) {
      printf("Could not write to file '%s'\n", img_filename.c_str());
      return false;
    }
    std::string inflated_img_filename = prefix + "_inflated.png";
    if (!cv::imwrite(inflated_img_filename, _inflated_map_as_img)) {
      printf("Could not write to file '%s'\n", inflated_img_filename.c_str());
      return false;
    }
    std::string csv_filename = prefix + ".csv";
    std::ofstream csv_content(csv_filename.c_str());
    if (!csv_content.is_open()) {
      printf("Could not write to file '%s'\n", csv_filename.c_str());
      return false;
    }
    csv_content << "xmin," << _xmin << ",ymin," << _ymin;
    csv_content << ",xmax," << _xmax << ",ymax," << _ymax;
    csv_content << ",pix2m," << _pix2m << ",inflation_radius," << _inflation_radius;
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool load(const std::string & prefix) {
    printf("LiteObstacleMap::load('%s')\n", prefix.c_str());
    std::string csv_filename = prefix + ".csv", line;
    std::ifstream csv_content(csv_filename.c_str());
    if (!csv_content.is_open() || !std::getline (csv_content,line)) {
      printf("Could not read file '%s'\n", csv_filename.c_str());
      return false;
    }
    std::vector<std::string> words;
    StringSplit(line, ",", &words);
    if (words.size() < 12) {
      printf("File '%s' is corrupted, shoud contain 12 values!\n", csv_filename.c_str());
      return false;
    }
    _xmin = atof(words[1].c_str());
    _ymin = atof(words[3].c_str());
    _xmax = atof(words[5].c_str());
    _ymax = atof(words[7].c_str());
    _pix2m = atof(words[9].c_str());
    _inflation_radius = atof(words[11].c_str());
    create(_xmin, _ymin, _xmax, _ymax, _pix2m, _inflation_radius);
    // images
    std::string img_filename = prefix + ".png";
    _map_as_img = cv::imread(img_filename, cv::IMREAD_GRAYSCALE);
    if (_map_as_img.empty()) {
      printf("Could not read file '%s'\n", img_filename.c_str());
      return false;
    }
    inflate();
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  void create(double xmin, double ymin, double xmax, double ymax,
              double pix2m = DEFAULT_PIX2M, double inflation_radius = DEFAULT_INFLATION_RADIUS) {
    _xmin = xmin;
    _xmax = xmax;
    _ymin = ymin;
    _ymax = ymax;
    _pix2m = pix2m;
    _m2pix = 1./pix2m;
    _inflation_radius = inflation_radius;
    // reset map
    _w = 1 + (_xmax - _xmin) * _m2pix;
    _h = 1 + (_ymax - _ymin) * _m2pix;
    DEBUG_PRINT("LiteObstacleMap::create(): bounds (%g,%g)->(%g,%g), dims(%i,%i)\n",
                _xmin, _ymin, _xmax, _ymax, _w, _h);
    _map_as_img.create(_h, _w);
    _map_as_img.setTo(0);
    _map_as_img.copyTo(_inflated_map_as_img);
  }

  //////////////////////////////////////////////////////////////////////////////

  template<class Pt2>
  bool add_obstacles(const std::vector<Pt2> & obstacles) {
    // fill map
    unsigned int npts = obstacles.size();
    for (unsigned int i = 0; i < npts; ++i) {
      const Pt2* curr = &(obstacles[i]);
      if (curr->x > _xmin && curr->x < _xmax && curr->y > _ymin && curr->y < _ymax)
        _map_as_img( m2pix(*curr) ) = 255;
    } // end for i
    //cv::imshow("LiteObstacleMap", _map_as_img); cv::waitKey(500);
    inflate();
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline bool is_occupied(const double & x, const double & y) const {
    if (x <= _xmin || x >= _xmax || y <= _ymin || y >= _ymax)
      return false; // out of bounds
    return _inflated_map_as_img.at<uchar>(m2pix(x, y));
  }
  template<class Pt2f>
  inline bool is_occupied(const Pt2f & pt) {
    return is_occupied(pt.x, pt.y);
  }

  //////////////////////////////////////////////////////////////////////////////

  template<class Int>
  inline void export2vector(std::vector<Int> & out) const {
    out.resize(_w * _h, 0); // free
    for (unsigned int row = 0; row < _h; ++row) {
      const uchar* data = _inflated_map_as_img.ptr<uchar>(row);
      for (unsigned int col = 0; col < _w; ++col) {
        if (data[col])
          out[col + row * _w] = 100; // occupied
      } // end loop col
    } // end loop row
  } // end export2vector

  //////////////////////////////////////////////////////////////////////////////

  inline unsigned int get_width() const  { return _w; }
  inline unsigned int get_height() const { return _h; }
  inline double get_xmin() const         { return _xmin; }
  inline double get_ymin() const         { return _ymin; }
  inline double get_xmax() const         { return _xmax; }
  inline double get_ymax() const         { return _ymax; }
  inline double get_pix2m() const        { return _pix2m; }

  //////////////////////////////////////////////////////////////////////////////
protected:
  //////////////////////////////////////////////////////////////////////////////

  //! _inflation_radius in meters
  bool inflate() {
    if (_map_as_img.empty()) {
      printf("LiteObstacleMap::inflate(): empty map!\n");
      return false;
    }
    int radpix = _inflation_radius * _m2pix;
    if (radpix == 0)
      return true;
    // http://docs.opencv.org/2.4/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html
    int dilation_type = cv::MORPH_RECT;
    cv::Mat element = cv::getStructuringElement( dilation_type,
                                                 cv::Size( 2*radpix + 1, 2*radpix+1 ),
                                                 cv::Point( radpix, radpix ) );
    cv::dilate(_map_as_img, _inflated_map_as_img, element);
    //cv::imshow("LiteObstacleMap-inflated", _map_as_img); cv::waitKey(500);
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

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

  double _xmin, _ymin, _xmax, _ymax, _pix2m, _m2pix, _inflation_radius;
  unsigned int _w, _h;
  cv::Mat1b _map_as_img, _inflated_map_as_img;
}; // end class LiteObstacleMap

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

template<class Pt2>
class MultiLaserSurveillance {
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

  MultiLaserSurveillance() {
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
      if (!_obstacle_map.add_obstacles(d->_last_scan)) {
        printf("Map creating failed!\n");
        return false;
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
      if (!_obstacle_map.is_occupied(pt)) {
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
      SurveillanceDevice* d = &(_devices[i]);
      std::copy(d->_outliers.begin(), d->_outliers.end(), std::back_inserter(_outliers));
    } // end for i
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  LiteObstacleMap _obstacle_map;
  Mode _mode;
  std::vector<SurveillanceDevice> _devices;
  unsigned int _map_total_nscans;
  bool _need_recompute_scan;
  Scan _outliers, _scan;
  bool _need_recompute_outliers;
}; // end class MultiLaserSurveillance

#endif // _WANDERER_H_
