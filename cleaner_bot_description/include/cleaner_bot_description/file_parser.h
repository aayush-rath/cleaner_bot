#ifndef FILE_PARSER_H
#define FILE_PARSER_H

#include <tinyxml2.h>
#include <string>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <sstream>

std::string get_mesh_path(const std::string& model_sdf_path, const std::string& model_root, const std::string& model_name);
void read_world_file(const std::string& world_file_path, std::unordered_map<std::string, std::vector<double>>& models);

#endif