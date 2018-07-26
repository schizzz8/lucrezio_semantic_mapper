#include "detection.h"

Detection::Detection(){
  _size=0;
  _top_left = Eigen::Vector2i(10000,10000);
  _bottom_right = Eigen::Vector2i(-10000,-10000);
}

Detection::Detection(const std::string &type_,
                     const Eigen::Vector2i &top_left_,
                     const Eigen::Vector2i &bottom_right_,
                     const std::vector<Eigen::Vector2i> &pixels_,
                     const Eigen::Vector3i &color_):
  _type(type_),
  _top_left(top_left_),
  _bottom_right(bottom_right_),
  _pixels(pixels_),
  _color(color_),
  _size(0){}

void Detection::setType(const std::string &type){

  _type = type;

  int c;

  if(type == "sink")
    c = 1;
  if(type == "burner_stove")
    c = 2;
  if(type == "table_ikea_bjursta")
    c = 3;
  if(type == "chair_ikea_borje")
    c = 4;
  if(type == "couch")
    c = 5;
  if(type == "table_tv")
    c = 6;
  if(type == "tv_samsung")
    c = 7;
  if(type == "cabinet_ikea_malm_big")
    c = 8;
  if(type == "milk")
    c = 9;
  if(type == "salt")
    c = 10;
  if(type == "tomato_sauce")
    c = 11;
  if(type == "zwieback")
    c = 12;

  std::stringstream stream;
  stream << std::setw(6) << std::setfill('0') << std::hex << ((float)c/12.0f)*16777215;
  std::string result(stream.str());

  unsigned long r_value = std::strtoul(result.substr(0,2).c_str(), 0, 16);
  unsigned long g_value = std::strtoul(result.substr(2,2).c_str(), 0, 16);
  unsigned long b_value = std::strtoul(result.substr(4,2).c_str(), 0, 16);

  _color = Eigen::Vector3i(r_value,g_value,b_value);

  _size=0;
  _top_left = Eigen::Vector2i(10000,10000);
  _bottom_right = Eigen::Vector2i(-10000,-10000);

}
