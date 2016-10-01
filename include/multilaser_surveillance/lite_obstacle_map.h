#ifndef LITE_OBSTACLE_MAP_H
#define LITE_OBSTACLE_MAP_H

#include <fstream>      // std::ifstream, std::ofstream
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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
    csv_content << ",cell2m," << _cell2m << ",inflation_radius," << _inflation_radius;
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
    _cell2m = atof(words[9].c_str());
    _inflation_radius = atof(words[11].c_str());
    create(_xmin, _ymin, _xmax, _ymax, _cell2m, _inflation_radius);
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
              double cell2m = DEFAULT_CELL2M, double inflation_radius = DEFAULT_INFLATION_RADIUS) {
    _xmin = xmin;
    _xmax = xmax;
    _ymin = ymin;
    _ymax = ymax;
    _cell2m = cell2m;
    _m2pix = 1./cell2m;
    _inflation_radius = inflation_radius;
    // reset map
    _w = 1 + (_xmax - _xmin) * _m2pix;
    _h = 1 + (_ymax - _ymin) * _m2pix;
    printf("LiteObstacleMap::create(): bounds (%g,%g)->(%g,%g), dims(%i,%i)\n",
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
      return true; // out of bounds
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
  inline double get_cell2m() const        { return _cell2m; }

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

  double _xmin, _ymin, _xmax, _ymax, _cell2m, _m2pix, _inflation_radius;
  unsigned int _w, _h;
  cv::Mat1b _map_as_img, _inflated_map_as_img;
}; // end class LiteObstacleMap

#endif // LITE_OBSTACLE_MAP_H

